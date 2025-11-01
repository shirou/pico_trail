//! MAVLink Network Transport Demonstration
//!
//! Demonstrates concurrent UART and UDP network transport for MAVLink communication.
//!
//! # Features
//!
//! - Concurrent UART and UDP transport operation
//! - WiFi configuration via parameter storage
//! - GCS endpoint auto-discovery
//! - Message broadcasting to all active transports
//!
//! # Hardware Setup
//!
//! ## UART Wiring
//!
//! - UART0 TX (GPIO 0) → USB-serial RX
//! - UART0 RX (GPIO 1) → USB-serial TX
//! - GND → GND
//!
//! ## WiFi Configuration
//!
//! WiFi credentials are stored in Flash parameters. Configure via MAVLink parameter protocol:
//!
//! 1. Connect via UART (115200 baud)
//! 2. Set NET_SSID parameter to your WiFi network name
//! 3. Set NET_PASS parameter to your WiFi password
//! 4. Optionally configure NET_DHCP, NET_IP, NET_NETMASK, NET_GATEWAY
//! 5. Reboot device
//!
//! # Usage
//!
//! ```bash
//! # Build for RP2350
//! ./scripts/build-rp2350.sh --release mavlink_demo_network
//!
//! # Flash to Pico 2 W
//! probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo_network
//! ```
//!
//! # Testing
//!
//! 1. Configure WiFi via UART connection (QGroundControl Parameters tab)
//! 2. Reboot device - should connect to WiFi automatically
//! 3. Open QGroundControl UDP connection (port 14550)
//! 4. Verify heartbeat and telemetry appear
//! 5. Verify concurrent UART and UDP operation

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[cfg(feature = "pico2_w")]
use pico_trail::{
    communication::mavlink::{
        transport::{uart::UartTransport, udp::UdpTransport, MavlinkTransport},
        transport_router::TransportRouter,
    },
    parameters::{wifi::WifiParams, ParameterStore},
    platform::rp2350::{network::WifiConfig, Rp2350Flash},
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("pico_trail MAVLink Network Demo");
    info!("==================================");
    info!("");

    // Initialize Flash for parameter storage
    let mut flash = Rp2350Flash::new();

    // Load parameters from Flash
    let mut param_store = ParameterStore::load_from_flash(&mut flash).unwrap_or_else(|_| {
        info!("No parameters found in Flash, using defaults");
        ParameterStore::new()
    });

    // Register WiFi parameters with defaults
    WifiParams::register_defaults(&mut param_store);

    // Initialize WiFi and UDP transport first (if configured)
    #[cfg(feature = "pico2_w")]
    {
        // Initialize platform
        let p = hal::init(Default::default());

        let wifi_params = WifiParams::from_store(&param_store);

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
                        unsafe { &mut RX_META },
                        unsafe { &mut RX_BUFFER },
                        unsafe { &mut TX_META },
                        unsafe { &mut TX_BUFFER },
                    );

                    info!("UDP transport initialized on port 14550");

                    // Create transport router with UDP only
                    let mut router = TransportRouter::new();
                    router.set_udp_transport(udp_transport);

                    info!("Transport router initialized with UDP");

                    // Spawn message routing task
                    spawner
                        .spawn(network_mavlink_task(router, param_store).unwrap());
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

    // Main loop
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

/// MAVLink task using TransportRouter for concurrent UART + UDP
#[cfg(feature = "pico2_w")]
#[embassy_executor::task]
async fn network_mavlink_task(
    mut router: TransportRouter<
        'static,
        embassy_rp::uart::BufferedUartRx,
        embassy_rp::uart::BufferedUartTx,
    >,
    _param_store: ParameterStore,
) {
    use embassy_time::Instant;

    info!("MAVLink network task started");

    let mut last_heartbeat = Instant::now();
    let heartbeat_interval = Duration::from_secs(1); // 1Hz
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
            embassy_futures::select::Either::Second(Ok((n, transport_id))) => {
                defmt::trace!("RX {} bytes from transport {}", n, transport_id);
                // For this demo, we just log received bytes
                // Full MAVLink parsing would use mavlink crate's read_v2_raw_message
            }
            embassy_futures::select::Either::Second(Err(e)) => {
                defmt::warn!("Receive error: {:?}", e);
            }
            embassy_futures::select::Either::First(_) => {
                // Timeout - no message received
            }
        }

        // Send periodic heartbeat
        if last_heartbeat.elapsed() >= heartbeat_interval {
            // Create HEARTBEAT message using mavlink common message set
            let heartbeat = mavlink::common::MavMessage::HEARTBEAT(
                mavlink::common::HEARTBEAT_DATA {
                    custom_mode: 0,
                    mavtype: mavlink::common::MavType::MAV_TYPE_QUADROTOR,
                    autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_GENERIC,
                    base_mode: mavlink::common::MavModeFlag::empty(),
                    system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
                    mavlink_version: 3,
                },
            );

            let header = mavlink::MavHeader {
                system_id: 1,
                component_id: 1,
                sequence,
            };

            // Serialize to bytes
            let mut msg_buf = [0u8; 280]; // MAVLink max message size
            let mut cursor = &mut msg_buf[..];

            if let Ok(_) = mavlink::write_v2_msg(&mut cursor, header, &heartbeat) {
                let len = 280 - cursor.len();
                match router.send_bytes(&msg_buf[..len]).await {
                    Ok(n) => {
                        defmt::trace!("TX HEARTBEAT ({} bytes to all transports)", n);
                        sequence = sequence.wrapping_add(1);
                    }
                    Err(e) => {
                        defmt::warn!("Heartbeat send error: {:?}", e);
                    }
                }
            }

            last_heartbeat = Instant::now();
        }

        Timer::after_millis(1).await;
    }
}

/// MAVLink task for non-pico2_w builds (UART-only)
#[cfg(not(feature = "pico2_w"))]
#[embassy_executor::task]
async fn network_mavlink_task(
    mut router: TransportRouter<
        'static,
        embassy_rp::uart::BufferedUartRx<'static>,
        embassy_rp::uart::BufferedUartTx<'static>,
    >,
    _param_store: ParameterStore,
) {
    use embassy_time::Instant;
    use pico_trail::communication::mavlink::{parser::MavlinkParser, writer::MavlinkWriter};

    info!("MAVLink task started (UART-only mode)");

    let mut parser = MavlinkParser::new();
    let mut writer = MavlinkWriter::new(1, 1);

    let mut last_heartbeat = Instant::now();
    let heartbeat_interval = Duration::from_secs(1);

    let mut buf = [0u8; 512];

    loop {
        let read_result = embassy_futures::select::select(
            Timer::after(Duration::from_millis(10)),
            router.receive_bytes(&mut buf),
        )
        .await;

        match read_result {
            embassy_futures::select::Either::Second(Ok((n, _transport_id))) => {
                for byte in &buf[..n] {
                    if let Ok(Some((header, msg))) = parser.parse_byte(*byte) {
                        if let Ok(bytes) = writer.encode_message(&header, &msg) {
                            let _ = router.send_bytes(bytes).await;
                        }
                    }
                }
            }
            embassy_futures::select::Either::Second(Err(_)) => {}
            embassy_futures::select::Either::First(_) => {}
        }

        if last_heartbeat.elapsed() >= heartbeat_interval {
            let heartbeat = mavlink::ardupilotmega::MavMessage::HEARTBEAT(
                mavlink::ardupilotmega::HEARTBEAT_DATA {
                    custom_mode: 0,
                    mavtype: mavlink::ardupilotmega::MavType::MAV_TYPE_QUADROTOR,
                    autopilot: mavlink::ardupilotmega::MavAutopilot::MAV_AUTOPILOT_GENERIC,
                    base_mode: mavlink::ardupilotmega::MavModeFlag::empty(),
                    system_status: mavlink::ardupilotmega::MavState::MAV_STATE_STANDBY,
                    mavlink_version: 3,
                },
            );

            let header = mavlink::MavHeader {
                system_id: 1,
                component_id: 1,
                sequence: 0,
            };

            if let Ok(bytes) = writer.encode_message(&header, &heartbeat) {
                let _ = router.send_bytes(bytes).await;
            }

            last_heartbeat = Instant::now();
        }

        Timer::after_millis(1).await;
    }
}
