//! Embassy-based time source implementation.
//!
//! This module provides the `EmbassyTime` implementation of the
//! `TimeSource` trait using Embassy's time driver.

use pico_trail_core::traits::TimeSource;

/// Embassy-based time source using the Embassy time driver.
///
/// This implementation uses `embassy_time::Instant` for high-resolution
/// timing on embedded targets with Embassy async runtime.
///
/// # Example
///
/// ```ignore
/// use pico_trail_firmware::platform::EmbassyTime;
/// use pico_trail_core::traits::TimeSource;
///
/// let time = EmbassyTime;
/// let now = time.now_us();
/// ```
#[derive(Clone, Copy, Default)]
pub struct EmbassyTime;

impl TimeSource for EmbassyTime {
    fn now_ms(&self) -> u64 {
        embassy_time::Instant::now().as_millis()
    }

    fn now_us(&self) -> u64 {
        embassy_time::Instant::now().as_micros()
    }
}
