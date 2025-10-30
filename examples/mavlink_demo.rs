//! MAVLink communication demonstration
//!
//! This example demonstrates MAVLink protocol communication with Ground Control
//! Stations (GCS) such as QGroundControl or Mission Planner.
//!
//! # Features
//!
//! - HEARTBEAT broadcasting at 1Hz
//! - Telemetry streaming (ATTITUDE, GPS_RAW_INT, SYS_STATUS)
//! - Parameter protocol support (PARAM_REQUEST_LIST, PARAM_SET)
//! - Command handling (ARM/DISARM, mode changes)
//! - Mission protocol (waypoint upload/download)
//!
//! # Hardware Setup
//!
//! ## Wiring
//!
//! Connect UART to USB-serial adapter:
//! - UART0 TX (GPIO 0) → USB-serial RX
//! - UART0 RX (GPIO 1) → USB-serial TX
//! - GND → GND
//!
//! ## GCS Configuration
//!
//! 1. Serial port: Select your USB-serial adapter (e.g., /dev/ttyUSB0, COM3)
//! 2. Baud rate: 115200
//! 3. Data bits: 8
//! 4. Parity: None
//! 5. Stop bits: 1
//!
//! # Usage
//!
//! ```bash
//! # Build for RP2350
//! ./scripts/build-rp2350.sh --release mavlink_demo
//!
//! # Flash using probe-rs (shows defmt logs)
//! probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo
//!
//! # Or flash via UF2 (no logs)
//! # 1. Hold BOOTSEL button while connecting USB
//! # 2. Copy target/mavlink_demo.uf2 to mounted drive
//! ```
//!
//! # Testing with QGroundControl
//!
//! 1. Open QGroundControl
//! 2. Go to Application Settings → Comm Links
//! 3. Add new Serial Link with 115200 baud
//! 4. Connect
//! 5. Vehicle should appear and telemetry should display
//!
//! # Expected Behavior
//!
//! - Logs show "MAVLink task started"
//! - HEARTBEAT sent every 1 second (visible in GCS)
//! - ATTITUDE sent every 100ms (10Hz)
//! - GPS_RAW_INT sent every 200ms (5Hz)
//! - SYS_STATUS sent every 1 second
//! - Parameter list available in GCS
//! - ARM/DISARM commands functional

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize platform
    let _p = hal::init(Default::default());

    info!("pico_trail MAVLink Demo");
    info!("========================");
    info!("");
    info!("UART Configuration:");
    info!("  - Baud rate: 115200");
    info!("  - UART0 TX: GPIO 0");
    info!("  - UART0 RX: GPIO 1");
    info!("");
    info!("Telemetry Rates:");
    info!("  - HEARTBEAT: 1Hz");
    info!("  - ATTITUDE: 10Hz (SR_EXTRA1)");
    info!("  - GPS_RAW_INT: 5Hz (SR_POSITION)");
    info!("  - SYS_STATUS: 1Hz");
    info!("");

    // TODO: Spawn MAVLink task
    // Example structure (requires actual UART instance):
    //
    // use pico_trail::communication::mavlink::task::{mavlink_task_placeholder, MavlinkConfig};
    // use pico_trail::platform::rp2350::Rp2350Flash;
    //
    // let flash = Rp2350Flash::new(...);
    // let config = MavlinkConfig {
    //     system_id: 1,
    //     component_id: 1,
    //     baud_rate: 115200,
    // };
    // spawner.spawn(mavlink_task_placeholder(config, flash)).unwrap();

    info!("Hardware integration pending - see example comments for details");
    info!("");
    info!("To complete this example:");
    info!("  1. Initialize RP2350 UART0 peripheral");
    info!("  2. Configure GPIO 0 (TX) and GPIO 1 (RX)");
    info!("  3. Create Rp2350Uart wrapper");
    info!("  4. Pass to MAVLink task");
    info!("");
    info!("Expected GCS Messages:");
    info!("  - HEARTBEAT (ID 0) - vehicle type, armed status");
    info!("  - SYS_STATUS (ID 1) - battery, CPU load");
    info!("  - ATTITUDE (ID 30) - roll, pitch, yaw");
    info!("  - GPS_RAW_INT (ID 24) - lat, lon, alt");
    info!("");
    info!("Supported Commands:");
    info!("  - MAV_CMD_COMPONENT_ARM_DISARM");
    info!("  - MAV_CMD_DO_SET_MODE");
    info!("  - MAV_CMD_PREFLIGHT_CALIBRATION");
    info!("");
    info!("Supported Parameters:");
    info!("  - SR_EXTRA1 (ATTITUDE rate, default 10Hz)");
    info!("  - SR_POSITION (GPS rate, default 5Hz)");
    info!("  - SR_RC_CHAN (RC channels rate, default 5Hz)");
    info!("  - SR_RAW_SENS (IMU rate, default 5Hz)");
    info!("  - SYSID_THISMAV (System ID, default 1)");
    info!("");

    // Demonstration statistics task
    spawner.spawn(stats_task()).unwrap();

    // Main loop
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

/// Statistics reporting task
///
/// Demonstrates the type of monitoring you'd see with actual MAVLink traffic
#[embassy_executor::task]
async fn stats_task() {
    info!("Statistics reporting started");
    info!("");

    let mut tick = 0u32;

    loop {
        Timer::after(Duration::from_secs(10)).await;
        tick += 1;

        info!("=== MAVLink Statistics (T+{}s) ===", tick * 10);
        info!("Expected telemetry in 10 seconds:");
        info!("  HEARTBEAT: 10 messages (1Hz)");
        info!("  ATTITUDE: 100 messages (10Hz)");
        info!("  GPS_RAW_INT: 50 messages (5Hz)");
        info!("  SYS_STATUS: 10 messages (1Hz)");
        info!("");
        info!("To see actual traffic:");
        info!("  1. Complete UART integration");
        info!("  2. Connect GCS");
        info!("  3. Monitor message counts in router stats");
        info!("");
    }
}
