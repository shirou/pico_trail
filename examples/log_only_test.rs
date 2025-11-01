//! Log Only Test - Minimal defmt logging test
//!
//! This example tests defmt-rtt functionality with minimal code.
//! No USB, no UART, no LED - just defmt logs via RTT.
//!
//! # Usage
//!
//! ```bash
//! cargo build --target thumbv8m.main-none-eabihf --features pico2_w --example log_only_test
//! sudo probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/debug/examples/log_only_test
//! ```

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize hardware (minimal setup)
    let _p = embassy_rp::init(Default::default());

    info!("========================================");
    info!("defmt-rtt Log Test");
    info!("========================================");
    info!("");
    info!("If you can see this message, probe-rs RTT is working correctly!");
    info!("");

    let mut counter = 0u32;

    loop {
        info!("Heartbeat #{}: System is running", counter);

        if counter % 5 == 0 {
            warn!("This is a WARNING message (counter={})", counter);
        }

        if counter % 10 == 0 {
            error!("This is an ERROR message (counter={})", counter);
            info!("--- Checkpoint: {} iterations completed ---", counter);
        }

        counter = counter.wrapping_add(1);

        // Wait 1 second
        Timer::after(Duration::from_secs(1)).await;
    }
}
