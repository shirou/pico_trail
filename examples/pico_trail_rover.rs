//! Pico Trail Rover Implementation
//!
//! Main rover implementation using MAVLink protocol over WiFi/UDP.
//!
//! # Features
//!
//! - MAVLink communication via UDP network transport
//! - WiFi configuration via parameter storage
//! - Command handling (ARM/DISARM, mode changes)
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
//! # Build for RP2350
//! ./scripts/build-rp2350.sh --release pico_trail_rover
//!
//! # Flash to Pico 2 W
//! probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/pico_trail_rover
//! ```
//!
//! # Testing
//!
//! 1. Configure WiFi via UART connection (QGroundControl Parameters tab)
//! 2. Reboot device - should connect to WiFi automatically
//! 3. Open QGroundControl UDP connection (port 14550)
//! 4. Verify heartbeat and telemetry appear
//! 5. Test ARM/DISARM commands

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_time::{Duration, Timer};
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

#[cfg(feature = "pico2_w")]
use pico_trail::{
    communication::mavlink::{
        dispatcher::MessageDispatcher,
        handlers::{
            command::CommandHandler, mission::MissionHandler, param::ParamHandler,
            telemetry::TelemetryStreamer,
        },
        parser::MavlinkParser,
        transport::udp::UdpTransport,
        transport_router::TransportRouter,
    },
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

    info!("pico_trail Rover");
    info!("================");
    info!("");

    // Initialize Flash for parameter storage
    let mut flash = Rp2350Flash::new();

    // Initialize ParamHandler which loads/registers parameters
    let param_handler = ParamHandler::new(&mut flash);

    // Initialize WiFi and UDP transport
    #[cfg(feature = "pico2_w")]
    {
        // Initialize platform
        let p = hal::init(Default::default());

        let wifi_params = WifiParams::from_store(param_handler.store());

        if wifi_params.is_configured() {
            info!("WiFi configured - initializing network...");

            let config = WifiConfig::from_params(&wifi_params);
            info!("  SSID: {}", config.ssid.as_str());
            info!("  DHCP: {}", config.use_dhcp);

            // Initialize WiFi (consumes p)
            match pico_trail::platform::rp2350::network::initialize_wifi(spawner, config, p).await {
                Ok((stack, _control)) => {
                    info!("WiFi connected");

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

                    info!("UDP transport initialized on port 14550");

                    // Create transport router with UDP only
                    let mut router = TransportRouter::new();
                    router.set_udp_transport(udp_transport);

                    info!("Transport router initialized with UDP");

                    // Initialize handlers
                    // Load system state with ARMING_CHECK parameter from parameter store
                    let state =
                        pico_trail::communication::mavlink::state::SystemState::from_param_store(
                            param_handler.store(),
                        );
                    let command_handler = CommandHandler::new(state);
                    let telemetry_streamer = TelemetryStreamer::new(1, 1);
                    let mission_handler = MissionHandler::new(1, 1);

                    // Create message dispatcher
                    let dispatcher = MessageDispatcher::new(
                        param_handler,
                        command_handler,
                        telemetry_streamer,
                        mission_handler,
                    );

                    info!("Message dispatcher initialized");

                    // Spawn MAVLink task
                    spawner.spawn(rover_mavlink_task(router, dispatcher).unwrap());
                }
                Err(e) => {
                    warn!("WiFi initialization failed: {:?}", e);
                    warn!("Cannot continue without WiFi");
                    loop {
                        Timer::after(Duration::from_secs(60)).await;
                    }
                }
            }
        } else {
            info!("WiFi not configured");
            info!("Configure WiFi via MAVLink parameters and reboot:");
            info!("  NET_SSID - WiFi network name");
            info!("  NET_PASS - WiFi password");
            loop {
                Timer::after(Duration::from_secs(60)).await;
            }
        }
    }

    #[cfg(not(feature = "pico2_w"))]
    {
        error!("This example requires pico2_w feature");
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

    info!("Rover MAVLink task started");

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
                defmt::trace!("RX {} bytes", n);

                // Create reader for parsing - may contain multiple messages
                let mut reader = ByteReader::new(&buf[..n]);

                // Parse all messages in the buffer
                loop {
                    match parser.read_message(&mut reader).await {
                        Ok((header, msg)) => {
                            defmt::debug!(
                                "RX MAVLink message: sys={}, comp={}, msgid={}",
                                header.system_id,
                                header.component_id,
                                msg.message_id()
                            );

                            let timestamp_us = Instant::now().as_micros();

                            // Dispatch message to appropriate handler
                            let responses = dispatcher.dispatch(&header, &msg, timestamp_us);
                            defmt::info!("Dispatch returned {} responses", responses.len());

                            // Send all response messages
                            for response_msg in responses {
                                defmt::info!(
                                    "Sending response: msgid={}",
                                    response_msg.message_id()
                                );

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
                                        defmt::trace!("TX {} bytes", len);
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
                            defmt::trace!("MAVLink parse finished/error: {:?}", e);
                            break;
                        }
                    }
                }
            }
            embassy_futures::select::Either::Second(Err(e)) => {
                defmt::warn!("Receive error: {:?}", e);
            }
            embassy_futures::select::Either::First(_) => {
                // Timeout - no message received
            }
        }

        // Update telemetry streams (get latest state from command handler)
        let timestamp_us = Instant::now().as_micros();
        let current_state = dispatcher.command_handler().state().clone();
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
                    defmt::trace!("TX telemetry ({} bytes to all transports)", len);
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
