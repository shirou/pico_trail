//! RP2350 Timer implementation
//!
//! This module provides timer and delay support for RP2350 using the `rp235x-hal` crate.

use crate::platform::{traits::TimerInterface, Result};
use rp235x_hal::timer::{Timer, TimerDevice};

/// RP2350 Timer implementation
///
/// Wraps the `rp235x-hal` timer to implement the `TimerInterface` trait.
///
/// # Note
///
/// The RP2350 timer is a 64-bit microsecond timer that provides accurate timing
/// and delay functionality.
pub struct Rp2350Timer<D: TimerDevice> {
    timer: Timer<D>,
}

impl<D: TimerDevice> Rp2350Timer<D> {
    /// Create a new RP2350 Timer instance
    ///
    /// # Arguments
    ///
    /// * `timer` - The HAL timer peripheral
    pub fn new(timer: Timer<D>) -> Self {
        Self { timer }
    }
}

impl<D: TimerDevice> TimerInterface for Rp2350Timer<D> {
    fn delay_us(&mut self, us: u32) -> Result<()> {
        use embedded_hal::blocking::delay::DelayUs;
        self.timer.delay_us(us);
        Ok(())
    }

    fn delay_ms(&mut self, ms: u32) -> Result<()> {
        use embedded_hal::blocking::delay::DelayMs;
        self.timer.delay_ms(ms);
        Ok(())
    }

    fn now_us(&self) -> u64 {
        self.timer.get_counter().ticks()
    }
}
