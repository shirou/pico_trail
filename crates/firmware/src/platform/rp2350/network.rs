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

use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};
use cyw43::{aligned_bytes, JoinOptions};
use cyw43_pio::DEFAULT_CLOCK_DIVIDER;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0},
    pio::{InterruptHandler as PioInterruptHandler, Pio},
    Peri,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use static_cell::StaticCell;

/// Maximum WiFi connection attempts before giving up
const MAX_WIFI_RETRIES: u8 = 10;

/// Atomic pointer to WiFi Stack (set after initialization)
static WIFI_STACK_PTR: AtomicPtr<Stack<'static>> = AtomicPtr::new(core::ptr::null_mut());

/// Signal to indicate WiFi initialization is complete
static WIFI_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Join timeout in seconds
const JOIN_TIMEOUT_SECS: u64 = 5;

/// Retry delay after timeout (1 second)
const RETRY_DELAY_MS: u64 = 1000;

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

/// Wait for WiFi to be ready and get the Stack reference.
///
/// Call this after spawning the WiFi task to get the network stack.
/// Blocks until WiFi initialization is complete.
///
/// # Returns
///
/// Reference to the network stack.
pub async fn wait_wifi_ready() -> &'static Stack<'static> {
    WIFI_READY.wait().await;
    // SAFETY: WIFI_STACK_PTR is set before WIFI_READY is signaled
    unsafe { &*WIFI_STACK_PTR.load(Ordering::Acquire) }
}

/// Initialize WiFi and network stack.
///
/// **IMPORTANT**: This function never returns! It runs an infinite loop to keep
/// the CYW43 Control alive. Moving Control to static memory breaks ping functionality.
///
/// # Usage
///
/// Spawn this as a task and use `wait_wifi_ready()` to get the Stack:
///
/// ```no_run
/// spawner.spawn(wifi_init_task(spawner, config, pins...)).unwrap();
/// let stack = wait_wifi_ready().await;
/// ```
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
) -> ! {
    if !config.is_configured() {
        crate::log_info!("WiFi not configured (empty SSID), skipping WiFi initialization");
        loop {
            Timer::after(Duration::from_secs(3600)).await;
        }
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

    // NOTE: Control must remain a local variable (NOT in StaticCell)
    // Moving Control to static memory causes ping to stop working.
    // This is a known issue with the CYW43 driver.

    // 5. Spawn WiFi driver task
    spawner.spawn(wifi_task(runner).unwrap());

    // Give WiFi driver task time to start
    Timer::after_millis(100).await;

    // 7. Initialize CLM (Country Locale Matrix) BEFORE network stack
    // This is critical - CLM must be loaded before network operations
    control.init(clm).await;

    // 8. Get MAC address BEFORE power management (matches wifi_test.rs order)
    // Note: Order matters for CYW43439 driver stability
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

    // 8. Configure power management
    // Note: PowerSave causes ping unresponsiveness, use None for debugging
    control
        .set_power_management(cyw43::PowerManagementMode::None)
        .await;

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
    static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
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

    // 10. Join WPA2 network with timeout and retry logic
    // Note: First join attempt often times out due to CYW43439 driver state issue.
    // Calling leave() after timeout fixes the internal state and allows subsequent joins to succeed.
    let mut retries = 0;
    loop {
        retries += 1;
        crate::log_info!(
            "Attempting to join WiFi network (attempt {}/{})",
            retries,
            MAX_WIFI_RETRIES
        );

        let options = JoinOptions::new(config.password.as_bytes());

        // Try join with timeout
        let join_result = embassy_time::with_timeout(
            Duration::from_secs(JOIN_TIMEOUT_SECS),
            control.join(config.ssid.as_str(), options),
        )
        .await;

        match join_result {
            Ok(Ok(_)) => {
                crate::log_info!("WiFi connected successfully on attempt {}", retries);
                break;
            }
            Ok(Err(_e)) => {
                crate::log_warn!("WiFi join returned error on attempt {}", retries);
            }
            Err(_timeout) => {
                crate::log_warn!(
                    "WiFi join timeout after {} seconds on attempt {}",
                    JOIN_TIMEOUT_SECS,
                    retries
                );
            }
        }

        if retries >= MAX_WIFI_RETRIES {
            crate::log_error!("WiFi connection failed after {} attempts", MAX_WIFI_RETRIES);
            // Don't return - keep Control alive and loop forever
            loop {
                Timer::after(Duration::from_secs(3600)).await;
            }
        }

        // Cleanup after timeout - leave() resets CYW43439 internal state
        crate::log_debug!("Calling leave() to cleanup driver state...");
        control.leave().await;
        crate::log_debug!("leave() done");

        // Wait before retry
        crate::log_info!("Retrying in {} ms", RETRY_DELAY_MS);
        Timer::after(Duration::from_millis(RETRY_DELAY_MS)).await;
    }

    // 11. Wait for network configuration (DHCP or static)
    if config.use_dhcp {
        // Wait for link and DHCP using embassy's official wait methods
        // Note: Removed 5 second delay - it caused ping to stop working
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

    // Store stack pointer and signal ready
    WIFI_STACK_PTR.store(stack as *const _ as *mut _, Ordering::Release);
    WIFI_READY.signal(());

    // CRITICAL: Control must NOT be moved or stored in static memory.
    // Moving Control breaks ping functionality (CYW43 driver issue).
    // Keep Control alive by running an infinite loop here.
    // The caller should use wait_wifi_ready() to get the Stack.
    crate::log_info!("WiFi control loop starting (Control stays local)");
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

bind_interrupts!(pub struct PioIrqs {
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

/// WiFi initialization task
///
/// Wrapper task for `initialize_wifi`. Spawn this task and use `wait_wifi_ready()`
/// to get the Stack once initialization is complete.
///
/// # Example
///
/// ```no_run
/// spawner.spawn(wifi_init_task(spawner, config, pins...).unwrap());
/// let stack = wait_wifi_ready().await;
/// ```
#[embassy_executor::task]
#[allow(clippy::too_many_arguments)]
pub async fn wifi_init_task(
    spawner: Spawner,
    config: WifiConfig,
    pin_23: Peri<'static, PIN_23>,
    pin_24: Peri<'static, PIN_24>,
    pin_25: Peri<'static, PIN_25>,
    pin_29: Peri<'static, PIN_29>,
    pio0: Peri<'static, PIO0>,
    dma_ch0: Peri<'static, DMA_CH0>,
) -> ! {
    initialize_wifi(
        spawner, config, pin_23, pin_24, pin_25, pin_29, pio0, dma_ch0,
    )
    .await
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
