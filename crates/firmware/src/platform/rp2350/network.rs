//! RP2350 WiFi Network Configuration
//!
//! Provides WiFi initialization and network stack management for Pico 2 W using
//! the CYW43439 WiFi chip and Embassy network stack.
//!
//! # WiFi Connection Flow
//!
//! ```text
//! 1. Load WiFi parameters from Flash (ParameterStore)
//! 2. Initialize CYW43439 driver (PIO, DMA, firmware)
//! 3. Spawn network task on Embassy executor
//! 4. Join WPA2 network with SSID/password
//! 5. Configure DHCP or static IP
//! 6. Return network stack and control handles
//! ```
//!
//! # Fallback Strategy
//!
//! - Empty SSID: Skip WiFi, UART-only mode
//! - Connection timeout (30s): Retry with exponential backoff (1s, 2s, 4s, 8s, 16s)
//! - After 5 failures: Disable WiFi, continue UART-only
//!
//! # Memory Usage
//!
//! - CYW43439 driver: ~20 KB RAM
//! - embassy-net stack: ~15 KB RAM
//! - Total: ~35 KB RAM
//!
//! # Example
//!
//! ```no_run
//! use pico_trail::platform::rp2350::network::{WifiConfig, initialize_wifi};
//! use pico_trail::parameters::{ParameterStore, wifi::WifiParams};
//!
//! #[embassy_executor::main]
//! async fn main(spawner: embassy_executor::Spawner) {
//!     let mut flash = Rp2350Flash::new();
//!     let mut param_store = ParameterStore::load_from_flash(&mut flash).unwrap();
//!     WifiParams::register_defaults(&mut param_store);
//!
//!     let wifi_params = WifiParams::from_store(&param_store);
//!     if wifi_params.is_configured() {
//!         let config = WifiConfig::from_params(&wifi_params);
//!         let (stack, control) = initialize_wifi(spawner, config, p).await.unwrap();
//!     }
//! }
//! ```

use crate::parameters::wifi::WifiParams;
use embassy_executor::Spawner;
use embassy_net::{
    Config as NetConfig, Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4,
};
use embassy_rp::clocks::RoscRng;
use embassy_time::{Duration, Timer};

use cyw43::{aligned_bytes, Control, JoinOptions};
use cyw43_pio::DEFAULT_CLOCK_DIVIDER;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0},
    pio::{InterruptHandler as PioInterruptHandler, Pio},
    Peri,
};
use static_cell::StaticCell;

/// Maximum WiFi connection attempts before giving up
const MAX_WIFI_RETRIES: u8 = 10;

/// Initial retry delay (1 second)
const INITIAL_RETRY_DELAY_MS: u64 = 1000;

/// WiFi configuration loaded from parameters
#[derive(Debug, Clone)]
pub struct WifiConfig {
    /// WiFi network SSID (max 32 chars)
    pub ssid: heapless::String<32>,
    /// WiFi password (WPA2, max 63 chars)
    pub password: heapless::String<63>,
    /// Use DHCP for IP configuration
    pub use_dhcp: bool,
    /// Static IP address (used if use_dhcp = false)
    pub static_ip: [u8; 4],
    /// Network mask (used if use_dhcp = false)
    pub netmask: [u8; 4],
    /// Gateway address (used if use_dhcp = false)
    pub gateway: [u8; 4],
}

impl WifiConfig {
    /// Create WiFi configuration from WiFi parameters
    ///
    /// # Arguments
    ///
    /// * `params` - WiFi parameters loaded from parameter store
    ///
    /// # Returns
    ///
    /// WiFi configuration ready for initialization
    pub fn from_params(params: &WifiParams) -> Self {
        Self {
            ssid: params.ssid.clone(),
            password: params.password.clone(),
            use_dhcp: params.use_dhcp,
            static_ip: params.static_ip,
            netmask: params.netmask,
            gateway: params.gateway,
        }
    }

    /// Check if WiFi is configured
    ///
    /// Returns false if SSID is empty (skip WiFi initialization).
    pub fn is_configured(&self) -> bool {
        !self.ssid.is_empty()
    }
}

impl Default for WifiConfig {
    fn default() -> Self {
        Self {
            ssid: heapless::String::new(),
            password: heapless::String::new(),
            use_dhcp: true,
            static_ip: [0, 0, 0, 0],
            netmask: [255, 255, 255, 0],
            gateway: [0, 0, 0, 0],
        }
    }
}

/// WiFi initialization error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "pico2_w", derive(defmt::Format))]
pub enum WifiError {
    /// WiFi not configured (empty SSID)
    NotConfigured,
    /// Connection failed after all retries
    ConnectionFailed,
    /// Invalid configuration
    InvalidConfig,
}

/// Initialize WiFi and network stack
///
/// # Arguments
///
/// * `spawner` - Embassy task spawner
/// * `config` - WiFi configuration
/// * `pin_23` - GPIO23 for WiFi power (PIN_23)
/// * `pin_24` - GPIO24 for WiFi DIO (PIN_24)
/// * `pin_25` - GPIO25 for WiFi CS (PIN_25)
/// * `pin_29` - GPIO29 for WiFi CLK (PIN_29)
/// * `pio0` - PIO0 peripheral for WiFi SPI
/// * `dma_ch0` - DMA channel 0 for WiFi SPI
///
/// # Returns
///
/// `Ok((&'static Stack, &'static Control))` on success containing network stack and WiFi control handles
///
/// # Errors
///
/// - `WifiError::NotConfigured` - Empty SSID in configuration
/// - `WifiError::ConnectionFailed` - Failed to connect after all retries
///
/// # Example
///
/// ```no_run
/// let (stack, control) = initialize_wifi(
///     spawner,
///     wifi_config,
///     p.PIN_23,
///     p.PIN_24,
///     p.PIN_25,
///     p.PIN_29,
///     p.PIO0,
///     p.DMA_CH0,
/// ).await?;
/// ```
#[allow(clippy::too_many_arguments)]
pub async fn initialize_wifi(
    spawner: Spawner,
    config: WifiConfig,
    pin_23: Peri<'static, PIN_23>,
    pin_24: Peri<'static, PIN_24>,
    pin_25: Peri<'static, PIN_25>,
    pin_29: Peri<'static, PIN_29>,
    pio0: Peri<'static, PIO0>,
    dma_ch0: Peri<'static, DMA_CH0>,
) -> Result<(&'static Stack<'static>, &'static mut Control<'static>), WifiError> {
    if !config.is_configured() {
        crate::log_info!("WiFi not configured (empty SSID), skipping WiFi initialization");
        return Err(WifiError::NotConfigured);
    }

    let mut rng = RoscRng;

    crate::log_info!("Initializing WiFi with SSID: {}", config.ssid.as_str());

    // 1. Load CYW43439 firmware (aligned for DMA)
    let fw = aligned_bytes!("../../../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../../../cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../../../../../cyw43-firmware/nvram_rp2040.bin");

    // 2. Initialize PIO for WiFi SPI communication
    let pwr = Output::new(pin_23, Level::Low);
    let cs = Output::new(pin_25, Level::High);
    let mut pio = Pio::new(pio0, PioIrqs);
    let spi = cyw43_pio::PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        pin_24, // DIO
        pin_29, // CLK
        dma_ch0,
    );

    // 4. Initialize CYW43 driver
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;

    // 5. Spawn WiFi driver task
    spawner.spawn(wifi_task(runner).unwrap());

    // Give WiFi driver task time to start
    Timer::after_millis(100).await;

    // 6. Initialize CLM (Country Locale Matrix) BEFORE network stack
    // This is critical - CLM must be loaded before network operations
    control.init(clm).await;

    // 7. Configure power management (embassy official recommendation)
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    // Log MAC address for debugging
    let mac = control.address().await;
    crate::log_debug!(
        "WiFi MAC address: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );

    // 8. Configure network stack
    let net_config = if config.use_dhcp {
        crate::log_info!("Configuring DHCP");
        NetConfig::dhcpv4(Default::default())
    } else {
        crate::log_info!(
            "Configuring static IP: {}.{}.{}.{}",
            config.static_ip[0],
            config.static_ip[1],
            config.static_ip[2],
            config.static_ip[3]
        );
        NetConfig::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(
                Ipv4Address::new(
                    config.static_ip[0],
                    config.static_ip[1],
                    config.static_ip[2],
                    config.static_ip[3],
                ),
                24, // /24 netmask
            ),
            gateway: Some(Ipv4Address::new(
                config.gateway[0],
                config.gateway[1],
                config.gateway[2],
                config.gateway[3],
            )),
            dns_servers: Default::default(),
        })
    };

    // Generate random seed
    let seed = rng.next_u64();

    // 7. Create network stack using StaticCell (following embassy official examples)
    static STACK_RESOURCES: StaticCell<StackResources<8>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        net_config,
        STACK_RESOURCES.init(StackResources::new()),
        seed,
    );

    // Store stack reference in static for global access
    static STACK: StaticCell<Stack<'static>> = StaticCell::new();
    let stack = STACK.init(stack);

    // 9. Spawn network stack task
    spawner.spawn(net_task(runner).unwrap());

    // Give the network stack task time to fully start
    // This ensures the stack is ready to handle DHCP requests
    crate::log_debug!("Waiting for network stack to initialize...");
    Timer::after_millis(500).await;

    // 10. Join WPA2 network with retry logic
    let mut retries = 0;
    loop {
        crate::log_info!(
            "Attempting to join WiFi network (attempt {}/{})",
            retries + 1,
            MAX_WIFI_RETRIES
        );

        let options = JoinOptions::new(config.password.as_bytes());
        match control.join(config.ssid.as_str(), options).await {
            Ok(_) => {
                crate::log_info!("WiFi connected successfully");
                break;
            }
            Err(_e) => {
                crate::log_warn!("WiFi connection failed");
                retries += 1;

                if retries >= MAX_WIFI_RETRIES {
                    crate::log_error!("WiFi connection failed after {} attempts", MAX_WIFI_RETRIES);
                    return Err(WifiError::ConnectionFailed);
                }

                // Wait before retry with exponential backoff
                let delay = get_retry_delay(retries - 1);
                crate::log_info!("Retrying in {} ms", delay.as_millis());
                Timer::after(delay).await;
            }
        }
    }

    // 11. Wait for network configuration (DHCP or static)
    if config.use_dhcp {
        // Give extra time for router DHCP server to be ready
        // On cold boot, routers may need time to initialize DHCP service
        // and learn the new MAC address
        crate::log_debug!("Waiting for router DHCP server to be ready (5 seconds)...");
        Timer::after_millis(5000).await;

        // Wait for link and DHCP using embassy's official wait methods
        crate::log_info!("Waiting for link up...");
        stack.wait_link_up().await;
        crate::log_info!("Link is up");

        crate::log_info!("Waiting for DHCP configuration...");
        stack.wait_config_up().await;

        if let Some(config) = stack.config_v4() {
            let ip = config.address.address().octets();
            crate::log_info!(
                "DHCP IP configured: {}.{}.{}.{}",
                ip[0],
                ip[1],
                ip[2],
                ip[3]
            );
        }
    } else {
        // Static IP でもリンク確立を待つ必要がある
        crate::log_info!("Waiting for link up (static IP)...");
        stack.wait_link_up().await;
        crate::log_info!("Static IP configured");
    }

    crate::log_info!("WiFi initialization complete");

    // SAFETY: We need to return a mutable reference to Control, but it's stored in a static.
    // This is safe because:
    // 1. Control is only created once in this function
    // 2. We're returning the only mutable reference to it
    // 3. The caller is expected to manage it appropriately
    static CONTROL: StaticCell<Control<'static>> = StaticCell::new();
    let control_ref = CONTROL.init(control);

    Ok((stack, control_ref))
}

/// Helper function to retry WiFi connection with exponential backoff
///
/// # Arguments
///
/// * `attempt` - Current attempt number (0-based)
///
/// # Returns
///
/// Delay duration in milliseconds
fn get_retry_delay(attempt: u8) -> Duration {
    let delay_ms = INITIAL_RETRY_DELAY_MS * (1 << attempt).min(16);
    Duration::from_millis(delay_ms)
}

bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

/// WiFi driver task
///
/// Runs the CYW43439 WiFi driver event loop.
/// Must be spawned on the executor for WiFi to function.
#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<
        'static,
        cyw43::SpiBus<Output<'static>, cyw43_pio::PioSpi<'static, PIO0, 0, DMA_CH0>>,
    >,
) -> ! {
    runner.run().await
}

/// Network stack task
///
/// Runs the embassy-net network stack event loop.
/// Must be spawned on the executor for network operations to function.
#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wifi_config_default() {
        let config = WifiConfig::default();
        assert!(!config.is_configured());
        assert!(config.use_dhcp);
    }

    #[test]
    fn test_wifi_config_is_configured() {
        let mut config = WifiConfig::default();
        assert!(!config.is_configured());

        config.ssid = heapless::String::from("TestNetwork");
        assert!(config.is_configured());
    }

    #[test]
    fn test_wifi_config_dhcp() {
        let mut config = WifiConfig::default();
        config.ssid = heapless::String::from("TestNetwork");
        config.password = heapless::String::from("password123");
        config.use_dhcp = true;

        assert!(config.is_configured());
        assert!(config.use_dhcp);
        assert_eq!(config.static_ip, [0, 0, 0, 0]); // Should be ignored with DHCP
    }

    #[test]
    fn test_wifi_config_static_ip() {
        let mut config = WifiConfig::default();
        config.ssid = heapless::String::from("TestNetwork");
        config.password = heapless::String::from("password123");
        config.use_dhcp = false;
        config.static_ip = [192, 168, 1, 100];
        config.netmask = [255, 255, 255, 0];
        config.gateway = [192, 168, 1, 1];

        assert!(config.is_configured());
        assert!(!config.use_dhcp);
        assert_eq!(config.static_ip, [192, 168, 1, 100]);
        assert_eq!(config.netmask, [255, 255, 255, 0]);
        assert_eq!(config.gateway, [192, 168, 1, 1]);
    }

    #[test]
    fn test_wifi_config_empty_ssid() {
        let config = WifiConfig::default();
        assert!(!config.is_configured());
        assert_eq!(config.ssid.as_str(), "");
    }

    #[test]
    fn test_wifi_config_max_length_ssid() {
        let mut config = WifiConfig::default();
        // Create 32-character SSID (max length)
        let long_ssid = "12345678901234567890123456789012";
        config.ssid = heapless::String::from(long_ssid);
        assert!(config.is_configured());
        assert_eq!(config.ssid.len(), 32);
    }

    #[test]
    fn test_wifi_config_max_length_password() {
        let mut config = WifiConfig::default();
        config.ssid = heapless::String::from("TestNetwork");
        // Create 63-character password (max length for WPA2)
        let long_password = "123456789012345678901234567890123456789012345678901234567890123";
        config.password = heapless::String::from(long_password);
        assert_eq!(config.password.len(), 63);
    }

    #[test]
    fn test_wifi_config_from_params_dhcp() {
        use crate::parameters::wifi::WifiParams;
        use heapless::String;

        let mut params = WifiParams::default();
        params.ssid = String::from("MyNetwork");
        params.password = String::from("mypass123");
        params.use_dhcp = true;

        let config = WifiConfig::from_params(&params);

        assert_eq!(config.ssid.as_str(), "MyNetwork");
        assert_eq!(config.password.as_str(), "mypass123");
        assert!(config.use_dhcp);
        assert_eq!(config.static_ip, [0, 0, 0, 0]);
    }

    #[test]
    fn test_wifi_config_from_params_static_ip() {
        use crate::parameters::wifi::WifiParams;
        use heapless::String;

        let mut params = WifiParams::default();
        params.ssid = String::from("MyNetwork");
        params.password = String::from("mypass123");
        params.use_dhcp = false;
        params.static_ip = [10, 0, 0, 50];
        params.netmask = [255, 255, 255, 0];
        params.gateway = [10, 0, 0, 1];

        let config = WifiConfig::from_params(&params);

        assert_eq!(config.ssid.as_str(), "MyNetwork");
        assert!(!config.use_dhcp);
        assert_eq!(config.static_ip, [10, 0, 0, 50]);
        assert_eq!(config.netmask, [255, 255, 255, 0]);
        assert_eq!(config.gateway, [10, 0, 0, 1]);
    }

    #[test]
    fn test_wifi_config_from_params_empty_ssid() {
        use crate::parameters::wifi::WifiParams;

        let params = WifiParams::default(); // Empty SSID

        let config = WifiConfig::from_params(&params);

        assert!(!config.is_configured());
        assert_eq!(config.ssid.as_str(), "");
    }

    #[test]
    fn test_retry_delay_exponential_backoff() {
        assert_eq!(get_retry_delay(0), Duration::from_millis(1000)); // 1s
        assert_eq!(get_retry_delay(1), Duration::from_millis(2000)); // 2s
        assert_eq!(get_retry_delay(2), Duration::from_millis(4000)); // 4s
        assert_eq!(get_retry_delay(3), Duration::from_millis(8000)); // 8s
        assert_eq!(get_retry_delay(4), Duration::from_millis(16000)); // 16s
        assert_eq!(get_retry_delay(5), Duration::from_millis(16000)); // Cap at 16s
    }

    #[test]
    fn test_retry_delay_large_attempt() {
        // Ensure delay caps at 16s even for large attempt numbers
        assert_eq!(get_retry_delay(100), Duration::from_millis(16000));
    }
}
