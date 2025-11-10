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

#[cfg(feature = "usb_serial")]
hal::bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<hal::peripherals::USB>;
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
        transport::udp::UdpTransport,
        transport_router::TransportRouter,
    },
    libraries::{kinematics::DifferentialDrive, motor_driver::MotorGroup, RC_INPUT},
    parameters::wifi::WifiParams,
    platform::rp2350::{network::WifiConfig, Rp2350Flash},
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
                unsafe { &mut CONFIG_DESCRIPTOR },
                unsafe { &mut BOS_DESCRIPTOR },
                unsafe { &mut MSOS_DESCRIPTOR },
                unsafe { &mut CONTROL_BUF },
            );

            let cdc_class = CdcAcmClass::new(&mut builder, unsafe { &mut STATE }, 64);
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

        let wifi_params = WifiParams::from_store(param_handler.store());

        if wifi_params.is_configured() {
            pico_trail::log_info!("WiFi configured - initializing network...");

            let config = WifiConfig::from_params(&wifi_params);
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
                    let dispatcher = MessageDispatcher::new(
                        param_handler,
                        command_handler,
                        telemetry_streamer,
                        mission_handler,
                        rc_input_handler,
                    );

                    pico_trail::log_info!("Message dispatcher initialized");

                    // Spawn MAVLink task
                    spawner.spawn(rover_mavlink_task(router, dispatcher).unwrap());

                    // Spawn motor control task
                    spawner.spawn(motor_control_task(motor_group).unwrap());
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
    mut dispatcher: MessageDispatcher,
) {
    use embassy_time::Instant;
    use mavlink::Message;

    pico_trail::log_info!("Rover MAVLink task started");

    let mut parser = MavlinkParser::new();
    let mut sequence: u8 = 0;
    let mut buf = [0u8; 512];

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
                pico_trail::log_debug!("RX {} bytes", n);

                // Create reader for parsing - may contain multiple messages
                let mut reader = ByteReader::new(&buf[..n]);

                // Parse all messages in the buffer
                loop {
                    match parser.read_message(&mut reader).await {
                        Ok((header, msg)) => {
                            pico_trail::log_debug!(
                                "RX MAVLink message: sys={}, comp={}, seq={}, msgid={}",
                                header.system_id,
                                header.component_id,
                                header.sequence,
                                msg.message_id()
                            );

                            let timestamp_us = Instant::now().as_micros();

                            // Process RC input messages (async, updates global state)
                            dispatcher.process_rc_input(&msg, timestamp_us).await;

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
                                        pico_trail::log_trace!("TX {} bytes", len);
                                        sequence = sequence.wrapping_add(1);
                                    }
                                }

                                // Add small delay between response messages to avoid overwhelming GCS
                                if matches!(
                                    response_msg,
                                    mavlink::common::MavMessage::PARAM_VALUE(_)
                                ) {
                                    Timer::after_millis(5).await;
                                }
                            }
                        }
                        Err(e) => {
                            // End of buffer or parse error
                            pico_trail::log_trace!("MAVLink parse finished/error: {:?}", e);
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
                    pico_trail::log_trace!("TX telemetry ({} bytes to all transports)", len);
                    sequence = sequence.wrapping_add(1);
                }
            }
        }

        // Check for mission timeouts
        if let Some(mission_ack) = dispatcher.check_mission_timeout(timestamp_us) {
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

        Timer::after_millis(1).await;
    }
}

/// Motor control task
///
/// Reads RC input (steering/throttle), applies differential drive kinematics,
/// and controls the 4-wheel drive motors. Enforces armed state check.
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
    use pico_trail::communication::mavlink::state::SYSTEM_STATE;

    pico_trail::log_info!("Motor control task started");

    loop {
        // Read RC input
        let rc = RC_INPUT.lock().await;

        // Get steering (channel 1) and throttle (channel 3)
        // ArduPilot standard: Ch1=Roll/Steering, Ch3=Throttle
        let steering = rc.channels[0]; // Channel 1 (index 0)
        let throttle = rc.channels[2]; // Channel 3 (index 2)
        let rc_status = rc.status;

        drop(rc); // Release mutex

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

        // Set motor speeds (with armed check inside)
        let is_armed = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).is_armed());

        // Only log when there's significant input or state change
        if is_armed && (steering.abs() > 0.05 || throttle.abs() > 0.05) {
            pico_trail::log_debug!(
                "RC: steering={}, throttle={}, L={}, R={}, armed={}",
                steering,
                throttle,
                left_speed,
                right_speed,
                is_armed
            );
        }

        if let Err(e) = motor_group.set_group_speed(&speeds, is_armed) {
            // NotArmed is expected when system is disarmed - don't log as warning
            if !matches!(e, pico_trail::libraries::motor_driver::MotorError::NotArmed) {
                pico_trail::log_warn!("Motor control error: {:?}", e);
            }
        }

        // Check RC timeout
        if rc_status != pico_trail::libraries::RcStatus::Active {
            // RC lost or never connected - ensure motors stop
            if motor_group.stop_all().is_err() {
                pico_trail::log_warn!("Failed to stop motors on RC timeout");
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
