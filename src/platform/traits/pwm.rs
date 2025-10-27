//! PWM interface trait
//!
//! This module defines the PWM output interface that platform implementations must provide.

use crate::platform::Result;

/// PWM configuration
#[derive(Debug, Clone, Copy)]
pub struct PwmConfig {
    /// PWM frequency in Hz
    pub frequency: u32,
    /// Initial duty cycle (0.0 = 0%, 1.0 = 100%)
    pub duty_cycle: f32,
}

impl Default for PwmConfig {
    fn default() -> Self {
        Self {
            frequency: 50, // 50 Hz for servos
            duty_cycle: 0.0,
        }
    }
}

/// PWM interface trait
///
/// Platform implementations must provide this interface for PWM output control.
///
/// # Safety Invariants
///
/// - PWM peripheral must be initialized before use
/// - Only one owner per PWM channel
/// - No concurrent access to the same PWM channel from multiple contexts
/// - Duty cycle must be in range [0.0, 1.0]
pub trait PwmInterface {
    /// Set PWM duty cycle
    ///
    /// # Arguments
    ///
    /// * `duty_cycle` - Duty cycle as a fraction (0.0 = 0%, 1.0 = 100%)
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Pwm(PwmError::InvalidDutyCycle)` if the duty cycle
    /// is outside the valid range [0.0, 1.0].
    fn set_duty_cycle(&mut self, duty_cycle: f32) -> Result<()>;

    /// Get current duty cycle
    ///
    /// Returns the current duty cycle as a fraction (0.0 = 0%, 1.0 = 100%).
    fn duty_cycle(&self) -> f32;

    /// Set PWM frequency
    ///
    /// # Arguments
    ///
    /// * `frequency` - Frequency in Hz
    ///
    /// # Errors
    ///
    /// Returns `PlatformError::Pwm(PwmError::InvalidFrequency)` if the frequency
    /// cannot be achieved with the current clock configuration.
    fn set_frequency(&mut self, frequency: u32) -> Result<()>;

    /// Get current frequency
    ///
    /// Returns the current PWM frequency in Hz.
    fn frequency(&self) -> u32;

    /// Enable PWM output
    fn enable(&mut self);

    /// Disable PWM output
    fn disable(&mut self);

    /// Check if PWM is enabled
    fn is_enabled(&self) -> bool;
}
