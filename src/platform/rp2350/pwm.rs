//! RP2350 PWM implementation
//!
//! This module provides PWM support for RP2350 using the `rp235x-hal` crate.

use crate::platform::{
    Result,
    error::{PlatformError, PwmError},
    traits::{PwmConfig, PwmInterface},
};
use rp235x_hal::pwm::{Channel, Slice, SliceId};

/// RP2350 PWM implementation
///
/// Wraps the `rp235x-hal` PWM slice to implement the `PwmInterface` trait.
pub struct Rp2350Pwm<S: SliceId, C: Channel> {
    slice: Slice<S, rp235x_hal::pwm::FreeRunning>,
    channel: core::marker::PhantomData<C>,
    duty_cycle: f32,
    frequency: u32,
    enabled: bool,
}

impl<S: SliceId, C: Channel> Rp2350Pwm<S, C> {
    /// Create a new RP2350 PWM instance
    ///
    /// # Arguments
    ///
    /// * `slice` - The HAL PWM slice
    /// * `config` - PWM configuration
    pub fn new(mut slice: Slice<S, rp235x_hal::pwm::FreeRunning>, config: PwmConfig) -> Self {
        // Set initial frequency and duty cycle
        let top = slice.get_top();

        let mut pwm = Self {
            slice,
            channel: core::marker::PhantomData,
            duty_cycle: config.duty_cycle,
            frequency: config.frequency,
            enabled: false,
        };

        // Apply configuration
        let _ = pwm.set_frequency(config.frequency);
        let _ = pwm.set_duty_cycle(config.duty_cycle);

        pwm
    }

    /// Calculate compare value from duty cycle
    fn duty_to_compare(&self, duty: f32) -> u16 {
        let top = self.slice.get_top();
        (duty * top as f32) as u16
    }
}

impl<S: SliceId, C: Channel> PwmInterface for Rp2350Pwm<S, C> {
    fn set_duty_cycle(&mut self, duty_cycle: f32) -> Result<()> {
        if !(0.0..=1.0).contains(&duty_cycle) {
            return Err(PlatformError::Pwm(PwmError::InvalidDutyCycle));
        }

        self.duty_cycle = duty_cycle;
        let compare = self.duty_to_compare(duty_cycle);

        self.slice.channel_b().set_duty_cycle(compare);

        Ok(())
    }

    fn duty_cycle(&self) -> f32 {
        self.duty_cycle
    }

    fn set_frequency(&mut self, frequency: u32) -> Result<()> {
        if frequency == 0 {
            return Err(PlatformError::Pwm(PwmError::InvalidFrequency));
        }

        self.frequency = frequency;

        // RP2350 system clock is typically 125 MHz
        const SYS_CLOCK: u32 = 125_000_000;

        // Calculate divider and top value
        // PWM frequency = SYS_CLOCK / (DIV * (TOP + 1))
        // We'll use a fixed top value and calculate the divider
        let top: u16 = 65535; // Maximum resolution
        let divider = SYS_CLOCK / (frequency * (top as u32 + 1));

        if divider > 255 {
            return Err(PlatformError::Pwm(PwmError::InvalidFrequency));
        }

        self.slice.set_div_int(divider as u8);
        self.slice.set_top(top);

        // Re-apply duty cycle with new top value
        let compare = self.duty_to_compare(self.duty_cycle);
        self.slice.channel_b().set_duty_cycle(compare);

        Ok(())
    }

    fn frequency(&self) -> u32 {
        self.frequency
    }

    fn enable(&mut self) {
        self.slice.enable();
        self.enabled = true;
    }

    fn disable(&mut self) {
        self.slice.disable();
        self.enabled = false;
    }

    fn is_enabled(&self) -> bool {
        self.enabled
    }
}
