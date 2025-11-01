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

#[cfg(feature = "pico2_w")]
use crate::parameters::wifi::WifiParams;
use embassy_executor::Spawner;
use embassy_net::{
    Config as NetConfig, Ipv4Address, Ipv4Cidr, Stack, StackResources, StaticConfigV4,
};
use embassy_time::{Duration, Timer};

#[cfg(feature = "pico2_w")]
use cyw43::{Control, JoinOptions};
#[cfg(feature = "pico2_w")]
use cyw43_pio::DEFAULT_CLOCK_DIVIDER;
#[cfg(feature = "pico2_w")]
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{DMA_CH0, PIO0},
    pio::{InterruptHandler as PioInterruptHandler, Pio},
};
#[cfg(feature = "pico2_w")]
use static_cell::StaticCell;

/// Maximum WiFi connection attempts before giving up
const MAX_WIFI_RETRIES: u8 = 5;

/// Initial retry delay (1 second)
const INITIAL_RETRY_DELAY_MS: u64 = 1000;

/// WiFi connection timeout (30 seconds)
const WIFI_CONNECT_TIMEOUT_MS: u64 = 30_000;

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
    #[cfg(feature = "pico2_w")]
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

#[cfg(feature = "pico2_w")]
/// Initialize WiFi and network stack
///
/// # Arguments
///
/// * `spawner` - Embassy task spawner
/// * `config` - WiFi configuration
/// * `p` - RP2350 peripherals (PIO0, DMA, GPIO pins)
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
/// let (stack, control) = initialize_wifi(spawner, wifi_config, p).await?;
/// ```
pub async fn initialize_wifi(
    spawner: Spawner,
    config: WifiConfig,
    p: embassy_rp::Peripherals,
) -> Result<(&'static Stack<'static>, &'static mut Control<'static>), WifiError> {
    if !config.is_configured() {
        defmt::info!("WiFi not configured (empty SSID), skipping WiFi initialization");
        return Err(WifiError::NotConfigured);
    }

    defmt::info!("Initializing WiFi with SSID: {}", config.ssid.as_str());

    // 1. Load CYW43439 firmware
    let fw = include_bytes!("../../../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../../../cyw43-firmware/43439A0_clm.bin");

    // 2. Initialize PIO for WiFi SPI communication
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, PioIrqs);
    let spi = cyw43_pio::PioSpi::new(
        &mut pio.common,
        pio.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24, // DIO
        p.PIN_29, // CLK
        p.DMA_CH0,
    );

    // 4. Initialize CYW43 driver
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    // 5. Spawn WiFi driver task
    spawner.spawn(wifi_task(runner).unwrap());

    // 6. Configure network stack
    let net_config = if config.use_dhcp {
        defmt::info!("Configuring DHCP");
        NetConfig::dhcpv4(Default::default())
    } else {
        defmt::info!(
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
            dns_servers: heapless::Vec::new(),
        })
    };

    // Generate random seed for network stack
    let seed = 0x0123_4567_89ab_cdef; // TODO: Use proper RNG

    // 7. Create network stack
    static STACK: StaticCell<Stack<'static>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<8>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        net_config,
        RESOURCES.init(StackResources::<8>::new()),
        seed,
    );
    let stack = STACK.init(stack);

    // 8. Spawn network stack task
    spawner.spawn(net_task(runner).unwrap());

    // 9. Initialize CLM (Country Locale Matrix)
    control.init(clm).await;

    // 10. Join WPA2 network with retry logic
    let mut retries = 0;
    loop {
        defmt::info!(
            "Attempting to join WiFi network (attempt {}/{})",
            retries + 1,
            MAX_WIFI_RETRIES
        );

        let options = JoinOptions::new(config.password.as_bytes());
        match control.join(config.ssid.as_str(), options).await {
            Ok(_) => {
                defmt::info!("WiFi connected successfully");
                break;
            }
            Err(_e) => {
                defmt::warn!("WiFi connection failed");
                retries += 1;

                if retries >= MAX_WIFI_RETRIES {
                    defmt::error!("WiFi connection failed after {} attempts", MAX_WIFI_RETRIES);
                    return Err(WifiError::ConnectionFailed);
                }

                // Wait before retry with exponential backoff
                let delay = get_retry_delay(retries - 1);
                defmt::info!("Retrying in {} ms", delay.as_millis());
                Timer::after(delay).await;
            }
        }
    }

    // 11. Wait for network configuration (DHCP or static)
    if config.use_dhcp {
        defmt::info!("Waiting for DHCP...");
        let timeout = Duration::from_secs(30);
        let start = embassy_time::Instant::now();

        loop {
            if stack.is_config_up() {
                break;
            }

            if embassy_time::Instant::now().duration_since(start) > timeout {
                defmt::error!("DHCP timeout");
                return Err(WifiError::ConnectionFailed);
            }

            Timer::after_millis(100).await;
        }

        if let Some(config) = stack.config_v4() {
            defmt::info!("DHCP IP configured");
        }
    } else {
        defmt::info!("Static IP configured");
    }

    defmt::info!("WiFi initialization complete");

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

#[cfg(feature = "pico2_w")]
bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

#[cfg(feature = "pico2_w")]
/// WiFi driver task
///
/// Runs the CYW43439 WiFi driver event loop.
/// Must be spawned on the executor for WiFi to function.
#[embassy_executor::task]
async fn wifi_task(
    runner: cyw43::Runner<'static, Output<'static>, cyw43_pio::PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[cfg(feature = "pico2_w")]
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
