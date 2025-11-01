//! Debug Probe Test - Simple LED Blink with defmt Logging
//!
//! This example tests debug probe functionality with:
//! - Periodic LED blinking (GPIO 25)
//! - Regular defmt log output
//!
//! # Hardware Setup
//!
//! - LED connected to GPIO 25 (or use external LED with resistor)
//! - No additional hardware required
//!
//! # Usage
//!
//! ```bash
//! # Build
//! cargo build --target thumbv8m.main-none-eabihf --example debug_probe_test
//!
//! # Flash with debug probe
//! sudo probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/debug/examples/debug_probe_test
//! ```

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("=================================");
    info!("Debug Probe Test Starting");
    info!("=================================");
    info!("");

    // Initialize hardware
    let p = embassy_rp::init(Default::default());

    info!("Hardware initialized");

    // Configure GPIO 25 as output (LED pin on some Pico boards)
    let mut led = Output::new(p.PIN_25, Level::Low);

    info!("LED configured on GPIO 25");
    info!("Starting blink loop...");
    info!("");

    let mut counter = 0u32;

    loop {
        // LED ON
        led.set_high();
        info!("LED ON  - Counter: {}", counter);
        Timer::after(Duration::from_millis(500)).await;

        // LED OFF
        led.set_low();
        info!("LED OFF - Counter: {}", counter);
        Timer::after(Duration::from_millis(500)).await;

        counter = counter.wrapping_add(1);

        // Every 10 blinks, print additional info
        if counter % 10 == 0 {
            info!("--- Status: Running smoothly, {} blinks completed ---", counter);
        }
    }
}
