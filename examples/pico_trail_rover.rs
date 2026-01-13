//! Pico Trail Rover Implementation
//!
//! Main rover implementation using MAVLink protocol over WiFi/UDP.
//!
//! # Features
//!
//! - MAVLink communication via UDP network transport
//! - WiFi configuration via parameter storage
//! - USB Serial debug output (when built with `usb_serial` feature)
//! - Command handling (ARM/DISARM, mode changes)
//! - Manual control with RC input processing (RC_CHANNELS)
//! - Telemetry streaming
//! - Mission handling
//!
//! # Hardware Setup
//!
//! ## WiFi Configuration
//!
//! WiFi credentials are stored in Flash parameters. Configure via MAVLink parameter protocol:
//!
//! 1. Connect via UART (115200 baud) initially for setup
//! 2. Set NET_SSID parameter to your WiFi network name
//! 3. Set NET_PASS parameter to your WiFi password
//! 4. Optionally configure NET_DHCP, NET_IP, NET_NETMASK, NET_GATEWAY
//! 5. Reboot device
//!
//! # Usage
//!
//! ```bash
//! # Build for RP2350 (default: defmt logging via probe-rs)
//! ./scripts/build-rp2350.sh --release pico_trail_rover
//!
//! # Build with USB Serial debug output (no probe-rs needed)
//! EXTRA_FEATURES="usb_serial" ./scripts/build-rp2350.sh --release pico_trail_rover
//!
//! # Flash to Pico 2 W
//! probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/pico_trail_rover
//!
//! # Or copy UF2 file (when using usb_serial feature)
//! # 1. Hold BOOTSEL while connecting USB
//! # 2. Copy target/pico_trail_rover.uf2 to mounted drive
//! # 3. Connect serial terminal: screen /dev/ttyACM0 115200
//! ```
//!
//! # Testing
//!
//! 1. Configure WiFi via UART connection (QGroundControl Parameters tab)
//! 2. Reboot device - should connect to WiFi automatically
//! 3. Open QGroundControl/Mission Planner UDP connection (port 14550)
//! 4. Verify heartbeat and telemetry appear
//! 5. Test ARM/DISARM commands
//! 6. Test manual control (configure joystick, arm vehicle, move joystick)
//! 7. Test RC timeout (stop sending RC_CHANNELS → verify fail-safe after 1s)

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_time::{Duration, Timer};

#[cfg(feature = "usb_serial")]
use core::ptr::addr_of_mut;
#[cfg(feature = "usb_serial")]
use embassy_rp::usb::{Driver, InterruptHandler};
#[cfg(feature = "usb_serial")]
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
#[cfg(feature = "usb_serial")]
use embassy_usb::{Builder, Config};

// Panic handler for USB serial builds
#[cfg(feature = "usb_serial")]
use panic_halt as _;

// Panic handler and defmt-rtt for non-USB builds
#[cfg(not(feature = "usb_serial"))]
use {defmt_rtt as _, panic_probe as _};

// Global allocator for Box/Vec support (required for mode manager)
#[cfg(feature = "pico2_w")]
use embedded_alloc::LlffHeap as Heap;

#[cfg(feature = "pico2_w")]
#[global_allocator]
static HEAP: Heap = Heap::empty();

// Heap size: 16 KB (sufficient for mode manager and small allocations)
#[cfg(feature = "pico2_w")]
const HEAP_SIZE: usize = 16 * 1024;

// Use global AHRS_STATE from pico_trail crate for sharing between tasks
// This is defined in pico_trail::subsystems::ahrs::AHRS_STATE

/// GPS position provider for FusedHeadingSource
///
/// Returns current GPS position from SYSTEM_STATE.
/// Used by FusedHeadingSource to get COG when moving.
#[cfg(feature = "pico2_w")]
fn get_gps_position() -> Option<GpsPosition> {
    use pico_trail::communication::mavlink::state::SYSTEM_STATE;
    critical_section::with(|cs| {
        let state = SYSTEM_STATE.borrow_ref(cs);
        state.gps_position
    })
}

// IRQ bindings: USB + ADC + UART0 + I2C0 when usb_serial is enabled, ADC + UART0 + I2C0 only otherwise
#[cfg(feature = "usb_serial")]
hal::bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<hal::peripherals::USB>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    UART0_IRQ => embassy_rp::uart::BufferedInterruptHandler<hal::peripherals::UART0>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<hal::peripherals::I2C0>;
});

#[cfg(all(feature = "pico2_w", not(feature = "usb_serial")))]
hal::bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    UART0_IRQ => embassy_rp::uart::BufferedInterruptHandler<hal::peripherals::UART0>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<hal::peripherals::I2C0>;
});

#[cfg(feature = "pico2_w")]
use pico_trail::{
    communication::mavlink::{
        dispatcher::MessageDispatcher,
        handlers::{
            command::CommandHandler, mission::MissionHandler, param::ParamHandler,
            rc_input::RcInputHandler, telemetry::TelemetryStreamer,
        },
        parser::MavlinkParser,
        state::BatteryAdcReader,
        transport::udp::UdpTransport,
        transport_router::TransportRouter,
        vehicle::GroundRover,
    },
    communication::shtp::ShtpI2c,
    core::traits::SharedState,
    devices::gps::GpsPosition,
    devices::imu::bno086::{Bno086DriverWithGpio, Bno086GpioConfig, EmbassyIntPin, EmbassyRstPin},
    libraries::{kinematics::DifferentialDrive, motor_driver::MotorGroup, RC_INPUT},
    parameters::wifi::WifiParams,
    platform::rp2350::{network::WifiConfig, Rp2350Flash},
    subsystems::ahrs::Bno086ExternalAhrs,
    subsystems::navigation::{
        FusedHeadingSource, HeadingSource, NavigationController, SimpleNavigationController,
        NAV_OUTPUT,
    },
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize heap for allocations (Box, Vec, etc.)
    #[cfg(feature = "pico2_w")]
    {
        static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
        unsafe {
            HEAP.init(
                core::ptr::addr_of_mut!(HEAP_MEM) as *mut u8 as usize,
                HEAP_SIZE,
            )
        }
    }

    pico_trail::log_info!("pico_trail Rover");
    pico_trail::log_info!("================");
    pico_trail::log_info!("");

    // Initialize Flash for parameter storage
    let mut flash = Rp2350Flash::new();

    // Initialize ParamHandler which loads/registers parameters
    let param_handler = ParamHandler::new(&mut flash);

    // Initialize WiFi and UDP transport
    #[cfg(feature = "pico2_w")]
    {
        // Initialize platform
        let p = hal::init(Default::default());

        // Initialize USB Serial (if feature enabled)
        #[cfg(feature = "usb_serial")]
        {
            pico_trail::log_info!("Initializing USB Serial for debug output...");

            let driver = Driver::new(p.USB, Irqs);

            let mut usb_config = Config::new(0x2e8a, 0x000a);
            usb_config.manufacturer = Some("Raspberry Pi");
            usb_config.product = Some("Pico Trail Rover");
            usb_config.serial_number = Some("12345678");
            usb_config.max_power = 100;
            usb_config.max_packet_size_0 = 64;

            static mut CONFIG_DESCRIPTOR: [u8; 256] = [0; 256];
            static mut BOS_DESCRIPTOR: [u8; 256] = [0; 256];
            static mut CONTROL_BUF: [u8; 64] = [0; 64];
            static mut MSOS_DESCRIPTOR: [u8; 256] = [0; 256];
            static mut STATE: State = State::new();

            let mut builder = Builder::new(
                driver,
                usb_config,
                unsafe { &mut *addr_of_mut!(CONFIG_DESCRIPTOR) },
                unsafe { &mut *addr_of_mut!(BOS_DESCRIPTOR) },
                unsafe { &mut *addr_of_mut!(MSOS_DESCRIPTOR) },
                unsafe { &mut *addr_of_mut!(CONTROL_BUF) },
            );

            let cdc_class =
                CdcAcmClass::new(&mut builder, unsafe { &mut *addr_of_mut!(STATE) }, 64);
            let usb = builder.build();

            // Spawn USB logger task
            spawner.spawn(pico_trail::core::logging::usb_logger_task(cdc_class).unwrap());

            // Spawn USB device task
            spawner.spawn(usb_task(usb).unwrap());

            pico_trail::log_info!("USB Serial initialized");
        }

        // Extract motor peripherals first (before WiFi initialization)
        use hal::pwm::{Config as PwmConfig, Pwm};
        let mut pwm_config = PwmConfig::default();
        pwm_config.top = 999; // 500 Hz @ 125 MHz (matches Freenove)
        pwm_config.divider = 250.into();

        pico_trail::log_info!("Initializing motors...");

        // Motor 1 (GPIO18/19) - Left Front
        let pwm1 = Pwm::new_output_ab(p.PWM_SLICE1, p.PIN_18, p.PIN_19, pwm_config.clone());
        let (output_a, output_b) = pwm1.split();
        let in2 = pico_trail::platform::rp2350::EmbassyPwmPin {
            output: core::cell::RefCell::new(output_a.expect("PWM channel A")),
        };
        let in1 = pico_trail::platform::rp2350::EmbassyPwmPin {
            output: core::cell::RefCell::new(output_b.expect("PWM channel B")),
        };
        let motor1 = pico_trail::libraries::motor_driver::HBridgeMotor::new(in1, in2);

        // Motor 2 (GPIO20/21) - Left Rear
        let pwm2 = Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_20, p.PIN_21, pwm_config.clone());
        let motor2 = pico_trail::platform::rp2350::init_motor_embassy(pwm2);

        // Motor 3 (GPIO6/7) - Right Rear
        let pwm3 = Pwm::new_output_ab(p.PWM_SLICE3, p.PIN_6, p.PIN_7, pwm_config.clone());
        let motor3 = pico_trail::platform::rp2350::init_motor_embassy(pwm3);

        // Motor 4 (GPIO8/9) - Right Front
        let pwm4 = Pwm::new_output_ab(p.PWM_SLICE4, p.PIN_8, p.PIN_9, pwm_config);
        let motor4 = pico_trail::platform::rp2350::init_motor_embassy(pwm4);

        let motors = [motor1, motor2, motor3, motor4];
        let motor_group = MotorGroup::new(motors);

        pico_trail::log_info!("Motors initialized");

        // Initialize UART0 for GPS (GPIO 0 TX, GPIO 1 RX) - Debug mode with buffering
        let mut uart0_config = embassy_rp::uart::Config::default();
        uart0_config.baudrate = 9600;

        static mut UART0_RX_BUF: [u8; 256] = [0; 256];
        static mut UART0_TX_BUF: [u8; 256] = [0; 256];

        let uart0 = embassy_rp::uart::BufferedUart::new(
            p.UART0,
            p.PIN_0, // TX (GPIO 0)
            p.PIN_1, // RX (GPIO 1)
            Irqs,
            unsafe { &mut *core::ptr::addr_of_mut!(UART0_TX_BUF) },
            unsafe { &mut *core::ptr::addr_of_mut!(UART0_RX_BUF) },
            uart0_config,
        );

        pico_trail::log_info!("UART0 initialized (GPIO 0/1, 9600 baud)");

        // GPS task will be spawned after WiFi connects (to avoid blocking DHCP)
        // Keep uart0 for later use

        // BNO086 External AHRS will be initialized after WiFi (GPIO 4 = SDA, GPIO 5 = SCL)
        // Keep I2C0 peripherals for later use
        let i2c0_peripheral = p.I2C0;
        let i2c0_scl = p.PIN_5;
        let i2c0_sda = p.PIN_4;

        // Load WiFi configuration from parameters
        let wifi_config = WifiParams::from_store(param_handler.store());
        if wifi_config.is_configured() {
            let config = WifiConfig {
                ssid: wifi_config.ssid.clone(),
                password: wifi_config.password.clone(),
                use_dhcp: wifi_config.use_dhcp,
                static_ip: wifi_config.static_ip,
                netmask: wifi_config.netmask,
                gateway: wifi_config.gateway,
            };
            pico_trail::log_info!("WiFi configuration loaded:");
            pico_trail::log_info!("  SSID: {}", config.ssid.as_str());
            pico_trail::log_info!("  DHCP: {}", config.use_dhcp);

            // Initialize WiFi with individual peripherals
            match pico_trail::platform::rp2350::network::initialize_wifi(
                spawner, config, p.PIN_23, p.PIN_24, p.PIN_25, p.PIN_29, p.PIO0, p.DMA_CH0,
            )
            .await
            {
                Ok((stack, _control)) => {
                    pico_trail::log_info!("WiFi connected");

                    // Wait for network to be ready
                    Timer::after(Duration::from_secs(2)).await;

                    // Initialize UDP transport
                    static mut RX_META: [embassy_net::udp::PacketMetadata; 4] =
                        [embassy_net::udp::PacketMetadata::EMPTY; 4];
                    static mut RX_BUFFER: [u8; 4096] = [0; 4096];
                    static mut TX_META: [embassy_net::udp::PacketMetadata; 4] =
                        [embassy_net::udp::PacketMetadata::EMPTY; 4];
                    static mut TX_BUFFER: [u8; 4096] = [0; 4096];

                    let udp_transport = UdpTransport::new(
                        stack,
                        14550,
                        unsafe { &mut *core::ptr::addr_of_mut!(RX_META) },
                        unsafe { &mut *core::ptr::addr_of_mut!(RX_BUFFER) },
                        unsafe { &mut *core::ptr::addr_of_mut!(TX_META) },
                        unsafe { &mut *core::ptr::addr_of_mut!(TX_BUFFER) },
                    );

                    pico_trail::log_info!("UDP transport initialized on port 14550");

                    // Create transport router with UDP only
                    let mut router = TransportRouter::new();
                    router.set_udp_transport(udp_transport);

                    pico_trail::log_info!("Transport router initialized with UDP");

                    // Initialize handlers
                    // Load system state with ARMING_CHECK parameter from parameter store
                    let state =
                        pico_trail::communication::mavlink::state::SystemState::from_param_store(
                            param_handler.store(),
                        );
                    // Initialize global SYSTEM_STATE
                    critical_section::with(|cs| {
                        *pico_trail::communication::mavlink::state::SYSTEM_STATE
                            .borrow_ref_mut(cs) = state;
                    });

                    let command_handler = CommandHandler::new();
                    let telemetry_streamer = TelemetryStreamer::new(1, 1);
                    let mission_handler = MissionHandler::new(1, 1);
                    let rc_input_handler = RcInputHandler::new();

                    // Create message dispatcher
                    let dispatcher = MessageDispatcher::<GroundRover>::new(
                        param_handler,
                        command_handler,
                        telemetry_streamer,
                        mission_handler,
                        rc_input_handler,
                    );

                    pico_trail::log_info!("Message dispatcher initialized");

                    // Initialize ADC for battery voltage monitoring (GPIO 26 = ADC0)
                    let adc =
                        embassy_rp::adc::Adc::new(p.ADC, Irqs, embassy_rp::adc::Config::default());

                    let adc_channel =
                        embassy_rp::adc::Channel::new_pin(p.PIN_26, embassy_rp::gpio::Pull::None);

                    let adc_reader = Rp2350AdcReader::from_parts(adc, adc_channel);
                    pico_trail::log_info!("ADC reader initialized (GPIO 26)");

                    // Spawn MAVLink task
                    spawner.spawn(rover_mavlink_task(router, dispatcher, adc_reader).unwrap());

                    // Spawn motor control task
                    spawner.spawn(motor_control_task(motor_group).unwrap());

                    // Spawn GPS task (after WiFi to avoid blocking DHCP)
                    spawner.spawn(gps_task(uart0).unwrap());

                    // Spawn navigation task
                    spawner.spawn(navigation_task().unwrap());

                    // =========================================================================
                    // BNO086 Initialization with GPIO Driver
                    // =========================================================================
                    // Uses Bno086DriverWithGpio for INT-driven reads and RST recovery
                    // Driver handles all GPIO operations internally

                    use embassy_rp::gpio::{Input, Level, Output, Pull};

                    // Pre-scan RST pulse to ensure BNO086 is in clean state
                    // This is critical after Pico soft reset (BNO086 may be in bad state)
                    // Uses Flex to properly release RST to high-Z (BNO086 RST must NOT be driven HIGH)
                    pico_trail::log_info!("=== BNO086 Pre-Reset ===");
                    {
                        use embassy_rp::gpio::Flex;
                        let mut rst = Flex::new(p.PIN_17);
                        rst.set_as_output();
                        rst.set_low();
                        Timer::after(Duration::from_millis(100)).await;
                        rst.set_as_input(); // Release to high-Z (not driven HIGH!)
                        drop(rst);
                        pico_trail::log_info!("  RST pulse complete, waiting for boot...");
                        Timer::after(Duration::from_millis(1000)).await; // Full boot time
                    }

                    // I2C Bus Recovery (SCL toggle only)
                    pico_trail::log_info!("=== I2C Bus Recovery ===");
                    {
                        let mut scl = Output::new(i2c0_scl, Level::High);
                        let sda_out = Output::new(i2c0_sda, Level::High);

                        Timer::after(Duration::from_millis(10)).await;

                        drop(sda_out);
                        let sda = Input::new(unsafe { hal::peripherals::PIN_4::steal() }, Pull::Up);
                        Timer::after(Duration::from_micros(100)).await;

                        if sda.is_low() {
                            pico_trail::log_warn!(
                                "  SDA stuck low, attempting SCL toggle recovery..."
                            );

                            // Toggle SCL up to 16 times to release stuck slave
                            for i in 0..16 {
                                scl.set_low();
                                Timer::after(Duration::from_micros(50)).await;
                                scl.set_high();
                                Timer::after(Duration::from_micros(50)).await;

                                if sda.is_high() {
                                    pico_trail::log_info!(
                                        "  SDA released after {} clock pulses",
                                        i + 1
                                    );
                                    break;
                                }
                            }

                            // Generate STOP condition
                            drop(sda);
                            let mut sda_out2 = Output::new(
                                unsafe { hal::peripherals::PIN_4::steal() },
                                Level::Low,
                            );
                            Timer::after(Duration::from_micros(50)).await;
                            scl.set_high();
                            Timer::after(Duration::from_micros(50)).await;
                            sda_out2.set_high();
                            Timer::after(Duration::from_micros(100)).await;

                            drop(sda_out2);
                            let sda_check =
                                Input::new(unsafe { hal::peripherals::PIN_4::steal() }, Pull::Up);
                            Timer::after(Duration::from_micros(100)).await;
                            if sda_check.is_high() {
                                pico_trail::log_info!("  Recovery complete, SDA is now HIGH");
                            } else {
                                pico_trail::log_warn!(
                                    "  SDA still LOW - may need power cycle (RST not used)"
                                );
                            }
                        } else {
                            pico_trail::log_info!("  SDA is high, bus OK");
                        }
                    }

                    // Small delay after recovery
                    Timer::after(Duration::from_millis(10)).await;

                    // Step 3: Initialize I2C at 50kHz (matching demo)
                    pico_trail::log_info!("Initializing I2C0 (GPIO4=SDA, GPIO5=SCL, 5kHz)...");
                    let pin_4 = unsafe { hal::peripherals::PIN_4::steal() };
                    let pin_5 = unsafe { hal::peripherals::PIN_5::steal() };

                    let i2c0 =
                        embassy_rp::i2c::I2c::new_async(i2c0_peripheral, pin_5, pin_4, Irqs, {
                            let mut config = embassy_rp::i2c::Config::default();
                            // 5kHz for maximum I2C stability with BNO086
                            // BNO086 has known I2C timing issues requiring very slow speeds
                            // and adequate delays between transactions
                            config.frequency = 5_000;
                            config
                        });

                    Timer::after(Duration::from_millis(100)).await;

                    // Step 4: Check for BNO086 at 0x4B (with retry)
                    let mut found_addr: Option<u8> = None;
                    use embedded_hal_async::i2c::I2c as I2cTrait;
                    let mut i2c0 = i2c0;

                    const BNO086_ADDR: u8 = 0x4B;
                    pico_trail::log_info!("=== BNO086 Check (0x{:02X}) ===", BNO086_ADDR);

                    for attempt in 1..=3_u8 {
                        let mut buf = [0u8; 1];
                        match embassy_time::with_timeout(
                            Duration::from_millis(100),
                            i2c0.read(BNO086_ADDR, &mut buf),
                        )
                        .await
                        {
                            Ok(Ok(_)) => {
                                pico_trail::log_info!("  Found BNO086 at 0x{:02X}", BNO086_ADDR);
                                found_addr = Some(BNO086_ADDR);
                                break;
                            }
                            Ok(Err(_)) => {
                                pico_trail::log_warn!("  Attempt {}/3: NACK", attempt);
                            }
                            Err(_) => {
                                pico_trail::log_warn!("  Attempt {}/3: timeout", attempt);
                            }
                        }

                        if attempt < 3 {
                            Timer::after(Duration::from_millis(200)).await;
                        }
                    }

                    if let Some(bno_addr) = found_addr {
                        pico_trail::log_info!("Using BNO086 at address 0x{:02X}", bno_addr);

                        // =========================================================================
                        // Create BNO086 driver with GPIO support (INT/RST pins)
                        // =========================================================================
                        pico_trail::log_info!(
                            "Initializing BNO086 with GPIO driver (INT-driven)..."
                        );

                        // Create SHTP I2C transport
                        let transport = ShtpI2c::new(i2c0, bno_addr);

                        // Create INT pin (GPIO22) - data ready signal
                        let int_gpio = Input::new(p.PIN_22, Pull::Up);
                        let int_pin = EmbassyIntPin::new(int_gpio);

                        // Create RST pin (GPIO17) - hardware reset (uses Flex for high-Z release)
                        // Note: PIN_17 was used for pre-scan reset, so we steal it here
                        use embassy_rp::gpio::Flex;
                        let rst_gpio = Flex::new(unsafe { hal::peripherals::PIN_17::steal() });
                        let rst_pin = EmbassyRstPin::new(rst_gpio);

                        // Create GPIO driver config
                        let config = Bno086GpioConfig::default();

                        // Create BNO086 driver with GPIO support
                        let mut driver =
                            Bno086DriverWithGpio::new(transport, int_pin, rst_pin, config);

                        // Initialize driver - handles RST pulse and INT detection internally
                        match driver.init().await {
                            Ok(()) => {
                                pico_trail::log_info!("BNO086 GPIO driver initialized!");

                                // Create AHRS wrapper and spawn task
                                let ahrs = Bno086ExternalAhrs::new(driver);
                                spawner.spawn(ahrs_task(ahrs).unwrap());
                                pico_trail::log_info!(
                                    "AHRS task spawned (BNO086 External AHRS, INT-driven)"
                                );
                            }
                            Err(e) => {
                                pico_trail::log_error!("BNO086 init failed: {:?}", e);
                                pico_trail::log_warn!("AHRS features will be disabled");
                            }
                        }
                    } else {
                        pico_trail::log_warn!("BNO086 not found on I2C bus!");
                        pico_trail::log_warn!("AHRS features will be disabled");
                        pico_trail::log_warn!("Check wiring: SDA=GPIO4, SCL=GPIO5");
                    }
                }
                Err(e) => {
                    pico_trail::log_warn!("WiFi initialization failed: {:?}", e);
                    pico_trail::log_warn!("Cannot continue without WiFi");
                    loop {
                        Timer::after(Duration::from_secs(60)).await;
                    }
                }
            }
        } else {
            pico_trail::log_info!("WiFi not configured");
            pico_trail::log_info!("Configure WiFi via MAVLink parameters and reboot:");
            pico_trail::log_info!("  NET_SSID - WiFi network name");
            pico_trail::log_info!("  NET_PASS - WiFi password");
            loop {
                Timer::after(Duration::from_secs(60)).await;
            }
        }
    }

    #[cfg(not(feature = "pico2_w"))]
    {
        pico_trail::log_error!("This example requires pico2_w feature");
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }
}

/// RP2350 ADC reader for battery voltage monitoring
///
/// Reads battery voltage from GPIO 26 (ADC0 channel) using embassy-rp ADC driver.
/// Implements 5-sample averaging per Freenove reference design for noise reduction.
///
/// # Usage
///
/// 1. Create reader with ADC peripheral and PIN_26
/// 2. Call `update_adc_value()` periodically (async) to read and average samples
/// 3. Call `read_battery_adc()` (sync) to get the most recent averaged value
///
/// # Example
///
/// ```rust
/// let mut adc_reader = Rp2350AdcReader::new(p.ADC, p.PIN_26);
///
/// // In async task:
/// adc_reader.update_adc_value().await;  // Read and average 5 samples
///
/// // Later (sync context):
/// let voltage = adc_reader.read_battery_adc();  // Get cached value
/// ```
#[cfg(feature = "pico2_w")]
struct Rp2350AdcReader<'a> {
    adc: embassy_rp::adc::Adc<'a, embassy_rp::adc::Async>,
    channel: embassy_rp::adc::Channel<'a>,
    last_value: core::cell::Cell<u16>,
}

#[cfg(feature = "pico2_w")]
impl<'a> Rp2350AdcReader<'a> {
    /// Create ADC reader from pre-initialized parts
    ///
    /// # Arguments
    ///
    /// * `adc` - Pre-initialized ADC peripheral
    /// * `channel` - Pre-configured ADC channel (GPIO 26)
    ///
    /// # Returns
    ///
    /// ADC reader ready for battery voltage monitoring
    pub fn from_parts(
        adc: embassy_rp::adc::Adc<'a, embassy_rp::adc::Async>,
        channel: embassy_rp::adc::Channel<'a>,
    ) -> Self {
        Self {
            adc,
            channel,
            last_value: core::cell::Cell::new(0),
        }
    }

    /// Update ADC value with 5-sample averaging
    ///
    /// Reads 5 samples from ADC0 (GPIO 26) and stores the average value.
    /// This reduces noise and provides more stable voltage readings.
    ///
    /// Call this method periodically (10 Hz recommended) from an async task.
    /// The averaged value is cached and returned by `read_battery_adc()`.
    ///
    /// # Timing
    ///
    /// Each ADC read takes ~2μs on RP2350.
    /// 5-sample averaging takes ~10μs total (well within 2ms requirement).
    pub async fn update_adc_value(&mut self) {
        // 5-sample averaging per Freenove reference design
        let mut sum: u32 = 0;
        for _ in 0..5 {
            match self.adc.read(&mut self.channel).await {
                Ok(value) => sum += value as u32,
                Err(_) => {
                    // ADC read error - use 0 as fallback
                    // This prevents telemetry from showing incorrect voltage
                    self.last_value.set(0);
                    return;
                }
            }
        }
        let avg = (sum / 5) as u16;
        self.last_value.set(avg);
    }
}

#[cfg(feature = "pico2_w")]
impl BatteryAdcReader for Rp2350AdcReader<'_> {
    /// Read the most recent averaged ADC value (synchronous)
    ///
    /// Returns the cached value from the last `update_adc_value()` call.
    /// This allows synchronous access to ADC data without blocking.
    ///
    /// # Returns
    ///
    /// Raw 12-bit ADC value (0-4095) representing voltage at ADC pin (0-3.3V range)
    fn read_battery_adc(&mut self) -> u16 {
        self.last_value.get()
    }
}

/// Simple byte buffer reader for MAVLink parsing
struct ByteReader<'a> {
    buffer: &'a [u8],
    position: usize,
}

impl<'a> ByteReader<'a> {
    fn new(buffer: &'a [u8]) -> Self {
        Self {
            buffer,
            position: 0,
        }
    }
}

impl<'a> embedded_io_async::ErrorType for ByteReader<'a> {
    type Error = core::convert::Infallible;
}

impl<'a> embedded_io_async::Read for ByteReader<'a> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let remaining = self.buffer.len() - self.position;
        if remaining == 0 {
            return Ok(0);
        }

        let to_read = buf.len().min(remaining);
        buf[..to_read].copy_from_slice(&self.buffer[self.position..self.position + to_read]);
        self.position += to_read;
        Ok(to_read)
    }
}

/// Main MAVLink task for rover operation
#[cfg(feature = "pico2_w")]
#[embassy_executor::task]
async fn rover_mavlink_task(
    mut router: TransportRouter<
        'static,
        embassy_rp::uart::BufferedUartRx,
        embassy_rp::uart::BufferedUartTx,
    >,
    mut dispatcher: MessageDispatcher<GroundRover>,
    mut adc_reader: Rp2350AdcReader<'static>,
) {
    use embassy_time::Instant;
    use mavlink::Message;

    pico_trail::log_info!("Rover MAVLink task started");

    let mut parser = MavlinkParser::new();
    let mut sequence: u8 = 0;
    let mut buf = [0u8; 512];

    // Battery update timing (10 Hz = 100ms interval)
    let mut last_battery_update = Instant::now();
    const BATTERY_UPDATE_INTERVAL: Duration = Duration::from_millis(100);

    loop {
        // Non-blocking receive with timeout
        let read_result = embassy_futures::select::select(
            Timer::after(Duration::from_millis(10)),
            router.receive_bytes(&mut buf),
        )
        .await;

        // Handle received messages
        match read_result {
            embassy_futures::select::Either::Second(Ok((n, _transport_id))) => {
                // Create reader for parsing - may contain multiple messages
                let mut reader = ByteReader::new(&buf[..n]);

                // Parse all messages in the buffer
                loop {
                    match parser.read_message(&mut reader).await {
                        Ok((header, msg)) => {
                            // Only log non-heartbeat messages
                            if msg.message_id() != 0 {
                                pico_trail::log_debug!("RX msgid={}", msg.message_id());
                            }

                            let timestamp_us = Instant::now().as_micros();

                            // Process RC input messages (async, updates global state)
                            dispatcher.process_rc_input(&msg, timestamp_us).await;

                            // Process navigation input messages (updates MISSION_STORAGE)
                            dispatcher.process_navigation_input(&msg).await;

                            // Dispatch message to appropriate handler
                            let responses = dispatcher.dispatch(&header, &msg, timestamp_us);

                            // Send all response messages
                            for response_msg in responses {
                                let response_header = mavlink::MavHeader {
                                    system_id: 1,
                                    component_id: 1,
                                    sequence,
                                };

                                let mut msg_buf = [0u8; 280];
                                let mut msg_cursor = &mut msg_buf[..];

                                if mavlink::write_v2_msg(
                                    &mut msg_cursor,
                                    response_header,
                                    &response_msg,
                                )
                                .is_ok()
                                {
                                    let len = 280 - msg_cursor.len();
                                    if router.send_bytes(&msg_buf[..len]).await.is_ok() {
                                        sequence = sequence.wrapping_add(1);
                                    }
                                }

                                // Add small delay between response messages to avoid overwhelming GCS
                                if matches!(
                                    response_msg,
                                    mavlink::common::MavMessage::PARAM_VALUE(_)
                                        | mavlink::common::MavMessage::MISSION_REQUEST_INT(_)
                                ) {
                                    Timer::after_millis(5).await;
                                }
                            }
                        }
                        Err(_e) => {
                            // End of buffer or parse error
                            //                            pico_trail::log_trace!("MAVLink parse finished/error: {:?}", e);
                            break;
                        }
                    }
                }
            }
            embassy_futures::select::Either::Second(Err(e)) => {
                pico_trail::log_warn!("Receive error: {:?}", e);
            }
            embassy_futures::select::Either::First(_) => {
                // Timeout - no message received
            }
        }

        // Update telemetry streams (get latest state from command handler)
        let timestamp_us = Instant::now().as_micros();

        // Update attitude from global AHRS state before sending telemetry
        critical_section::with(|cs| {
            let mut state =
                pico_trail::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs);
            state.update_attitude(&pico_trail::subsystems::ahrs::AHRS_STATE);
        });

        let current_state = dispatcher.command_handler().state();
        let telemetry_messages = dispatcher.update_telemetry(&current_state, timestamp_us);

        // Send telemetry messages
        for telem_msg in telemetry_messages {
            let header = mavlink::MavHeader {
                system_id: 1,
                component_id: 1,
                sequence,
            };

            let mut msg_buf = [0u8; 280];
            let mut cursor = &mut msg_buf[..];

            if mavlink::write_v2_msg(&mut cursor, header, &telem_msg).is_ok() {
                let len = 280 - cursor.len();
                if router.send_bytes(&msg_buf[..len]).await.is_ok() {
                    //                    pico_trail::log_trace!("TX telemetry ({} bytes to all transports)", len);
                    sequence = sequence.wrapping_add(1);
                }
            }
        }

        // Check for mission timeouts
        if let Some(mission_ack) = dispatcher.check_mission_timeout(timestamp_us) {
            pico_trail::log_warn!("SENDING MISSION_ACK due to timeout!");
            let header = mavlink::MavHeader {
                system_id: 1,
                component_id: 1,
                sequence,
            };

            let mut msg_buf = [0u8; 280];
            let mut cursor = &mut msg_buf[..];

            if mavlink::write_v2_msg(&mut cursor, header, &mission_ack).is_ok() {
                let len = 280 - cursor.len();
                if router.send_bytes(&msg_buf[..len]).await.is_ok() {
                    sequence = sequence.wrapping_add(1);
                }
            }
        }

        // Battery update (10 Hz timing check)
        if last_battery_update.elapsed() >= BATTERY_UPDATE_INTERVAL {
            // Read ADC value with 5-sample averaging (async)
            adc_reader.update_adc_value().await;

            // Update battery state in global SYSTEM_STATE with cached ADC value
            critical_section::with(|cs| {
                let mut state =
                    pico_trail::communication::mavlink::state::SYSTEM_STATE.borrow_ref_mut(cs);
                state.update_battery(&mut adc_reader);
            });
            last_battery_update = Instant::now();
        }

        Timer::after_millis(1).await;
    }
}

/// Motor control task
///
/// Reads input from RC (Manual mode) or Navigation Controller (Guided mode),
/// applies differential drive kinematics, and controls the 4-wheel drive motors.
/// Enforces armed state check.
#[cfg(feature = "pico2_w")]
#[embassy_executor::task]
async fn motor_control_task(
    mut motor_group: MotorGroup<
        pico_trail::libraries::motor_driver::HBridgeMotor<
            pico_trail::platform::rp2350::EmbassyPwmPin,
            pico_trail::platform::rp2350::EmbassyPwmPin,
        >,
    >,
) {
    use pico_trail::communication::mavlink::state::{FlightMode, SYSTEM_STATE};

    pico_trail::log_info!("Motor control task started");

    loop {
        // Get current mode and armed state
        let (mode, is_armed) = critical_section::with(|cs| {
            let state = SYSTEM_STATE.borrow_ref(cs);
            (state.mode, state.is_armed())
        });

        // Get steering/throttle based on mode
        let (steering, throttle, rc_status) = match mode {
            FlightMode::Manual => {
                // Manual mode: use RC input directly (blocking mutex with critical section)
                RC_INPUT.with(|rc| {
                    let steering = rc.channels[0]; // Channel 1 (index 0)
                    let throttle = rc.channels[2]; // Channel 3 (index 2)
                    (steering, throttle, Some(rc.status))
                })
            }
            FlightMode::Guided => {
                // Guided mode: use navigation controller output
                // Navigation output: steering is [-1, 1], throttle is [0, 1]
                // For bidirectional movement, we keep throttle as-is (forward only in Guided)
                NAV_OUTPUT.with(|nav_output| (nav_output.steering, nav_output.throttle, None))
                // No RC status in Guided mode
            }
            _ => {
                // Other modes (Stabilize, Loiter, Auto, RTL): not implemented yet
                // For now, use RC input as fallback (blocking mutex with critical section)
                RC_INPUT.with(|rc| (rc.channels[0], rc.channels[2], Some(rc.status)))
            }
        };

        // Apply differential drive kinematics
        // steering: -1.0 (left) to +1.0 (right)
        // throttle: -1.0 (reverse) to +1.0 (forward)
        let (left_speed, right_speed) = DifferentialDrive::mix(steering, throttle);

        // For 4WD: Left side = M1+M2, Right side = M3+M4
        let speeds = [
            left_speed,  // Motor 1 (Left Front)
            left_speed,  // Motor 2 (Left Rear)
            right_speed, // Motor 3 (Right Rear)
            right_speed, // Motor 4 (Right Front)
        ];

        // Only log when there's significant input or state change
        if is_armed && (steering.abs() > 0.05 || throttle.abs() > 0.05) {
            pico_trail::log_debug!(
                "Motor: mode={:?}, steering={}, throttle={}, L={}, R={}",
                mode,
                steering,
                throttle,
                left_speed,
                right_speed
            );
        }

        if !is_armed {
            // Stop all motors when disarmed (critical safety behavior)
            if motor_group.stop_all().is_err() {
                pico_trail::log_warn!("Failed to stop motors on disarm");
            }
        } else if let Err(e) = motor_group.set_group_speed(&speeds, is_armed) {
            pico_trail::log_warn!("Motor control error: {:?}", e);
        }

        // Check RC timeout (only in modes that use RC)
        if let Some(status) = rc_status {
            if status != pico_trail::libraries::RcStatus::Active {
                // RC lost or never connected - ensure motors stop
                if motor_group.stop_all().is_err() {
                    pico_trail::log_warn!("Failed to stop motors on RC timeout");
                }
            }
        }

        // 10 ms update rate (100 Hz)
        Timer::after_millis(10).await;
    }
}

/// USB device task
///
/// Runs the USB device event loop for USB Serial functionality.
#[cfg(feature = "usb_serial")]
#[embassy_executor::task]
async fn usb_task(
    mut usb: embassy_usb::UsbDevice<'static, Driver<'static, hal::peripherals::USB>>,
) {
    usb.run().await;
}

/// GPS polling task
///
/// Reads NMEA data from UART0 (GPS module) and updates global SYSTEM_STATE.
/// Uses BufferedUart wrapped in EmbassyBufferedUart adapter.
///
/// Initializes NEO-M8N with UBX commands following NeoGPS timing:
/// - 1 second startup delay for module stabilization
/// - 250ms delay between each UBX command
/// - Disables unnecessary NMEA messages (GLL, GSA, GSV, ZDA)
/// - Enables GGA, RMC, VTG on UART1
#[cfg(feature = "pico2_w")]
#[embassy_executor::task]
async fn gps_task(uart: embassy_rp::uart::BufferedUart) {
    use embassy_time::{Duration, Timer};
    use pico_trail::devices::gps::init::ublox::ubx_commands;
    use pico_trail::devices::gps::GpsDriver;
    use pico_trail::devices::gps_operation::GpsOperation;

    pico_trail::log_info!("GPS task started");

    // Create GPS driver with Embassy BufferedUart adapter
    let mut gps_uart = EmbassyBufferedUart::new(uart);

    // Wait for GPS module to stabilize after power-on (NeoGPS uses 1 second)
    // NEO-M8 needs time before accepting UBX commands reliably
    Timer::after(Duration::from_millis(1000)).await;

    // Configure u-blox NEO-M8N with delays between commands (NeoGPS: 250ms)
    // This follows the pattern from NeoGPS ubloxRate.ino
    const CMD_DELAY: Duration = Duration::from_millis(250);

    pico_trail::log_info!("GPS: Sending UBX configuration...");

    // Disable unnecessary NMEA messages first (reduces UART bandwidth)
    let _ = ubx_commands::send_ubx(&mut gps_uart, &ubx_commands::DISABLE_GLL);
    Timer::after(CMD_DELAY).await;

    let _ = ubx_commands::send_ubx(&mut gps_uart, &ubx_commands::DISABLE_GSA);
    Timer::after(CMD_DELAY).await;

    let _ = ubx_commands::send_ubx(&mut gps_uart, &ubx_commands::DISABLE_GSV);
    Timer::after(CMD_DELAY).await;

    let _ = ubx_commands::send_ubx(&mut gps_uart, &ubx_commands::DISABLE_ZDA);
    Timer::after(CMD_DELAY).await;

    // Enable required messages on UART1
    let _ = ubx_commands::send_ubx(&mut gps_uart, &ubx_commands::ENABLE_GGA);
    Timer::after(CMD_DELAY).await;

    let _ = ubx_commands::send_ubx(&mut gps_uart, &ubx_commands::ENABLE_RMC);
    Timer::after(CMD_DELAY).await;

    let _ = ubx_commands::send_ubx(&mut gps_uart, &ubx_commands::ENABLE_VTG);
    Timer::after(CMD_DELAY).await;

    pico_trail::log_info!("GPS UBX initialization complete (GGA/RMC/VTG enabled)");

    let gps = GpsDriver::new(gps_uart);
    let mut operation = GpsOperation::new(gps);

    // Run the GPS polling loop
    operation.poll_loop().await;
}

/// Navigation task
///
/// Reads GPS position from SYSTEM_STATE and navigation target from MISSION_STORAGE,
/// computes steering/throttle using SimpleNavigationController, and updates NAV_OUTPUT.
/// Runs at 50Hz (20ms interval).
///
/// Navigation is only active when MissionState is Running and mode is Guided or Auto.
/// Uses MISSION_STORAGE as the single source of truth for waypoint navigation.
#[cfg(feature = "pico2_w")]
#[embassy_executor::task]
async fn navigation_task() {
    use pico_trail::communication::mavlink::state::{FlightMode, SYSTEM_STATE};
    use pico_trail::core::mission::{
        advance_waypoint, complete_mission, get_current_target, get_mission_state,
        set_mission_state, set_single_waypoint, MissionState, Waypoint,
    };
    use pico_trail::subsystems::navigation::take_reposition_target;

    pico_trail::log_info!("Navigation task started");

    let mut controller = SimpleNavigationController::new();

    // Create fused heading source with AHRS + GPS COG
    // Uses GPS COG when moving (>1.0 m/s), AHRS yaw when stationary
    let heading_source = FusedHeadingSource::with_defaults(
        &pico_trail::subsystems::ahrs::AHRS_STATE,
        get_gps_position,
    );
    pico_trail::log_info!("Heading source initialized (AHRS + GPS COG, threshold=1.0 m/s)");

    loop {
        // Check for reposition command (from MAV_CMD_DO_REPOSITION / Fly Here)
        if let Some(reposition) = take_reposition_target() {
            pico_trail::log_info!(
                "Reposition target received: lat={}, lon={}",
                reposition.latitude,
                reposition.longitude
            );

            // Create waypoint from reposition target
            let waypoint = Waypoint {
                seq: 0,
                frame: 0,    // MAV_FRAME_GLOBAL
                command: 16, // MAV_CMD_NAV_WAYPOINT
                current: 1,
                autocontinue: 0,
                param1: 0.0,
                param2: 2.0, // WP_RADIUS default
                param3: 0.0,
                param4: 0.0,
                x: (reposition.latitude * 1e7) as i32,
                y: (reposition.longitude * 1e7) as i32,
                z: reposition.altitude.unwrap_or(0.0),
            };

            // Update MISSION_STORAGE (clear and add single waypoint)
            set_single_waypoint(waypoint);

            // Get current mode to check if we should start navigation
            let mode = critical_section::with(|cs| {
                let state = SYSTEM_STATE.borrow_ref(cs);
                state.mode
            });

            // If in GUIDED mode, set MissionState::Running
            if mode == FlightMode::Guided {
                set_mission_state(MissionState::Running);
                pico_trail::log_info!("GUIDED mode: Starting navigation to reposition target");
            }
        }

        // Get current mode and GPS from SYSTEM_STATE
        let (mode, gps_position) = critical_section::with(|cs| {
            let state = SYSTEM_STATE.borrow_ref(cs);
            (state.mode, state.gps_position)
        });

        // Get mission state
        let mission_state = get_mission_state();

        // Only run navigation when mission is running and in Guided or Auto mode
        let should_navigate = mission_state == MissionState::Running
            && (mode == FlightMode::Guided || mode == FlightMode::Auto);

        if should_navigate {
            // Get navigation target from MISSION_STORAGE (unified source)
            let target = get_current_target();

            if let (Some(current), Some(target)) = (gps_position, target) {
                // Get heading from fused source (AHRS when stationary, GPS COG when moving)
                let heading = heading_source
                    .get_heading()
                    .unwrap_or(current.course_over_ground.unwrap_or(0.0));

                // Compute navigation output
                let output = controller.update(&current, &target, heading, 0.02); // 20ms dt

                // Update global NAV_OUTPUT
                NAV_OUTPUT.with_mut(|nav_output| {
                    *nav_output = output;
                });

                // Handle waypoint arrival
                if output.at_target {
                    if mode == FlightMode::Guided {
                        // GUIDED mode: single waypoint, mission complete
                        complete_mission();
                        pico_trail::log_info!("GUIDED: Waypoint reached, mission completed");
                    } else if mode == FlightMode::Auto {
                        // AUTO mode: advance to next waypoint or complete mission
                        if advance_waypoint() {
                            pico_trail::log_info!("AUTO: Waypoint reached, advancing to next");
                        } else {
                            // Last waypoint reached - mission complete
                            complete_mission();
                            pico_trail::log_info!("AUTO: Mission completed, all waypoints reached");
                        }
                    }
                }

                // Log navigation status periodically (every ~1s = 50 iterations)
                static mut LOG_COUNTER: u32 = 0;
                unsafe {
                    LOG_COUNTER += 1;
                    if LOG_COUNTER >= 50 {
                        LOG_COUNTER = 0;
                        pico_trail::log_debug!(
                            "Nav: dist={}m, bearing={}°, steering={}, throttle={}, at_target={}",
                            output.distance_m,
                            output.bearing_deg,
                            output.steering,
                            output.throttle,
                            output.at_target
                        );
                    }
                }
            } else {
                // No GPS or no target - output zero
                NAV_OUTPUT.with_mut(|nav_output| {
                    *nav_output = pico_trail::subsystems::navigation::NavigationOutput::default();
                });
            }
        } else if mode == FlightMode::Guided || mode == FlightMode::Auto {
            // Mode is Guided/Auto but mission not running - zero output
            NAV_OUTPUT.with_mut(|nav_output| {
                *nav_output = pico_trail::subsystems::navigation::NavigationOutput::default();
            });
        }

        // 50 Hz update rate (20ms)
        Timer::after_millis(20).await;
    }
}

/// Embassy BufferedUart adapter for UartInterface trait
///
/// Wraps embassy_rp::uart::BufferedUart to implement the sync UartInterface.
/// Uses blocking read with timeout semantics.
#[cfg(feature = "pico2_w")]
struct EmbassyBufferedUart {
    uart: embassy_rp::uart::BufferedUart,
}

#[cfg(feature = "pico2_w")]
impl EmbassyBufferedUart {
    fn new(uart: embassy_rp::uart::BufferedUart) -> Self {
        Self { uart }
    }
}

#[cfg(feature = "pico2_w")]
impl pico_trail::platform::traits::UartInterface for EmbassyBufferedUart {
    fn read(&mut self, buffer: &mut [u8]) -> pico_trail::platform::Result<usize> {
        use embedded_hal_nb::serial::Read;
        // Use non-blocking read - returns WouldBlock if no data available
        // This prevents blocking the executor
        let mut count = 0;
        for byte in buffer.iter_mut() {
            match self.uart.read() {
                Ok(b) => {
                    *byte = b;
                    count += 1;
                }
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(e)) => {
                    // Handle UART errors based on type
                    // Overrun: RX FIFO overflowed - data was lost but we can continue reading
                    // Other errors (Break, Parity, Framing): indicate signal/config issues
                    match e {
                        embassy_rp::uart::Error::Overrun => {
                            // Overrun means the FIFO filled up - some data was lost
                            // but the remaining data in the buffer is still valid.
                            // Return what we have and let the next read continue.
                            if count > 0 {
                                break;
                            }
                            // No data yet, try to continue reading (error is cleared)
                            continue;
                        }
                        _ => {
                            // Other errors are more serious (signal issues)
                            let error_type = match e {
                                embassy_rp::uart::Error::Break => "Break",
                                embassy_rp::uart::Error::Parity => "Parity",
                                embassy_rp::uart::Error::Framing => "Framing",
                                _ => "Unknown",
                            };
                            pico_trail::log_debug!(
                                "UART read error: {}, count so far: {}",
                                error_type,
                                count
                            );
                            if count > 0 {
                                // Return what we have so far
                                break;
                            }
                            // No data read yet - return error
                            return Err(pico_trail::platform::PlatformError::Uart(
                                pico_trail::platform::error::UartError::ReadFailed,
                            ));
                        }
                    }
                }
            }
        }
        Ok(count)
    }

    fn write(&mut self, data: &[u8]) -> pico_trail::platform::Result<usize> {
        use embedded_io::Write;
        self.uart.write(data).map_err(|_| {
            pico_trail::platform::PlatformError::Uart(
                pico_trail::platform::error::UartError::WriteFailed,
            )
        })
    }

    fn flush(&mut self) -> pico_trail::platform::Result<()> {
        use embedded_io::Write;
        self.uart.flush().map_err(|_| {
            pico_trail::platform::PlatformError::Uart(
                pico_trail::platform::error::UartError::WriteFailed,
            )
        })
    }

    fn set_baud_rate(&mut self, _baud: u32) -> pico_trail::platform::Result<()> {
        // Embassy BufferedUart doesn't support runtime baud rate changes
        Err(pico_trail::platform::PlatformError::Uart(
            pico_trail::platform::error::UartError::InvalidBaudRate,
        ))
    }

    fn available(&self) -> bool {
        // Cannot check without mutable reference, assume data may be available
        true
    }
}

/// AHRS task for reading BNO086 attitude data at 100Hz
///
/// Reads quaternion orientation from the BNO086's on-chip sensor fusion and
/// updates AHRS_STATE. The BNO086 performs 9-axis fusion internally at 400Hz,
/// providing high-quality attitude estimation with minimal CPU overhead.
///
/// The AHRS yaw is used by FusedHeadingSource for navigation when stationary.
///
/// Uses Bno086DriverWithGpio for INT-driven reads and automatic RST recovery.
///
/// # Arguments
///
/// * `ahrs` - Initialized BNO086ExternalAhrs wrapper with GPIO driver
#[cfg(feature = "pico2_w")]
#[embassy_executor::task]
async fn ahrs_task(
    mut ahrs: Bno086ExternalAhrs<
        Bno086DriverWithGpio<
            ShtpI2c<embassy_rp::i2c::I2c<'static, hal::peripherals::I2C0, embassy_rp::i2c::Async>>,
            EmbassyIntPin<'static>,
            EmbassyRstPin<'static>,
        >,
    >,
) {
    use embassy_time::{Duration, Instant, Timer};
    use pico_trail::subsystems::ahrs::Ahrs;

    pico_trail::log_info!("AHRS task started (BNO086 External AHRS, 100Hz)");

    // Give BNO086 time to stabilize and start sending data
    pico_trail::log_info!("  Waiting 1s for BNO086 to stabilize...");
    Timer::after(Duration::from_secs(1)).await;

    let mut sample_count: u32 = 0;
    let mut error_count: u32 = 0;
    const CONVERGENCE_SAMPLES: u32 = 100; // 1 second at 100Hz (BNO086 converges fast)

    loop {
        // Read attitude from BNO086 via Ahrs trait
        match ahrs.get_attitude().await {
            Ok(state) => {
                sample_count += 1;

                // Update shared AHRS state (global from pico_trail crate)
                let timestamp_us = Instant::now().as_micros();
                pico_trail::subsystems::ahrs::AHRS_STATE.update_euler(
                    state.roll,
                    state.pitch,
                    state.yaw,
                    timestamp_us,
                );

                // Also update angular rates for telemetry
                {
                    let mut shared_state = pico_trail::subsystems::ahrs::AHRS_STATE.read();
                    shared_state.angular_rate = state.angular_rate;
                    pico_trail::subsystems::ahrs::AHRS_STATE.write(shared_state);
                }

                // Mark as healthy after convergence
                if sample_count == CONVERGENCE_SAMPLES {
                    pico_trail::subsystems::ahrs::AHRS_STATE.set_healthy(true);
                    pico_trail::log_info!("AHRS converged after {} samples", sample_count);
                }

                // Log every 100 samples (1 second at 100Hz)
                if sample_count % 100 == 0 {
                    pico_trail::log_info!(
                        "AHRS: yaw={} roll={} pitch={} healthy={}",
                        state.yaw.to_degrees(),
                        state.roll.to_degrees(),
                        state.pitch.to_degrees(),
                        ahrs.is_healthy()
                    );
                }
            }
            Err(e) => {
                error_count += 1;
                // Only log every 10th error to reduce spam
                if error_count <= 3 || error_count % 10 == 0 {
                    pico_trail::log_warn!("AHRS read error: {:?} (count={})", e, error_count);
                }
                // Longer delay after errors to let I2C bus recover
                Timer::after(Duration::from_millis(100)).await;
                continue;
            }
        }

        // BNO086 outputs at 100Hz, so ~10ms between reads
        // Slight delay to avoid polling too aggressively
        Timer::after(Duration::from_millis(5)).await;
    }
}
