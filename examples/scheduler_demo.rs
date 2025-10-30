//! Scheduler demonstration example
//!
//! This example demonstrates the Embassy-based task scheduler with multiple
//! tasks running at different rates. It shows:
//! - Task registration and execution
//! - Periodic task execution using Embassy Ticker
//! - Task execution time measurement
//! - CPU load monitoring
//! - Statistics collection and reporting
//!
//! # Hardware
//!
//! This example is designed for Raspberry Pi Pico 2 W (RP2350) but can be
//! adapted for other platforms.
//!
//! # Usage
//!
//! ```bash
//! cargo build --release --features pico2_w --example scheduler_demo
//! probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/scheduler_demo
//! ```

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use pico_trail::core::scheduler::monitor::monitor_task;
use pico_trail::core::scheduler::tasks::examples::{
    ahrs_task, control_task, imu_task, telemetry_task,
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize platform
    let _p = hal::init(Default::default());

    info!("pico_trail Scheduler Demo");
    info!("==========================");
    info!("");
    info!("Starting tasks:");
    info!("  - IMU task: 400Hz (2.5ms period)");
    info!("  - AHRS task: 100Hz (10ms period)");
    info!("  - Control task: 50Hz (20ms period)");
    info!("  - Telemetry task: 10Hz (100ms period)");
    info!("  - Monitor task: 1Hz (1s period)");
    info!("");

    // Spawn all tasks
    spawner.spawn(imu_task()).unwrap();
    spawner.spawn(ahrs_task()).unwrap();
    spawner.spawn(control_task()).unwrap();
    spawner.spawn(telemetry_task()).unwrap();
    spawner.spawn(monitor_task()).unwrap();

    info!("All tasks started successfully");
    info!("Monitor will report statistics every second");
    info!("");

    // Main loop - just keep alive
    loop {
        Timer::after(Duration::from_secs(10)).await;
    }
}
