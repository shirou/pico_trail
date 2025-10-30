//! Simple LED blink test for RP2350
//!
//! This is a minimal test program to verify basic hardware functionality.
//! It blinks the onboard LED without using any complex features.
//!
//! # Hardware
//!
//! Raspberry Pi Pico 2 W - LED on GPIO25
//!
//! # Usage
//!
//! ```bash
//! ./scripts/build-rp2350.sh --release blink_test
//! # Copy target/blink_test.uf2 to Pico 2 W in BOOTSEL mode
//! ```

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // GPIO25 is the onboard LED on Pico 2 W
    let mut led = Output::new(p.PIN_25, Level::Low);

    // Blink forever: 1 second on, 1 second off
    loop {
        led.set_high();
        Timer::after_millis(1000).await;
        led.set_low();
        Timer::after_millis(1000).await;
    }
}
