//! Servo and actuator abstraction for rover steering and throttle
//!
//! This module provides platform-agnostic types and functions for actuator control:
//! - Normalized commands (-1.0 to +1.0)
//! - PWM conversion (pulse width → duty cycle)
//! - Calibration support (min/neutral/max)
//!
//! # Design
//!
//! This module is pure `no_std` with no feature gates. Platform-specific
//! implementations (e.g., using Embassy PWM) belong in the firmware crate.
//!
//! ## Safety
//!
//! **CRITICAL**: Actuator implementations must enforce armed state check.
//! When disarmed, outputs should be overridden to neutral (0.0) regardless
//! of commanded value. This enforcement is the responsibility of the
//! platform-specific implementation.

/// Actuator interface for rover control
///
/// Provides normalized commands (-1.0 to +1.0) with safety enforcement.
pub trait ActuatorInterface {
    /// Set steering command
    ///
    /// # Arguments
    ///
    /// * `normalized` - Steering command (-1.0 left, 0.0 center, +1.0 right)
    ///
    /// # Safety
    ///
    /// Implementations must enforce armed check: outputs neutral if disarmed.
    fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str>;

    /// Set throttle command
    ///
    /// # Arguments
    ///
    /// * `normalized` - Throttle command (-1.0 reverse, 0.0 stop, +1.0 forward)
    ///
    /// # Safety
    ///
    /// Implementations must enforce armed check: outputs neutral if disarmed.
    fn set_throttle(&mut self, normalized: f32) -> Result<(), &'static str>;

    /// Get current steering value
    fn get_steering(&self) -> f32;

    /// Get current throttle value
    fn get_throttle(&self) -> f32;
}

/// Actuator calibration configuration
///
/// Defines PWM pulse widths for steering and throttle channels.
#[derive(Debug, Clone, Copy)]
pub struct ActuatorConfig {
    // Steering calibration (pulse width in μs)
    pub steering_min: u16,     // Default: 1000 (full left)
    pub steering_neutral: u16, // Default: 1500 (center)
    pub steering_max: u16,     // Default: 2000 (full right)

    // Throttle calibration (pulse width in μs)
    pub throttle_min: u16,     // Default: 1000 (full reverse)
    pub throttle_neutral: u16, // Default: 1500 (stop)
    pub throttle_max: u16,     // Default: 2000 (full forward)
}

impl Default for ActuatorConfig {
    fn default() -> Self {
        Self {
            steering_min: 1000,
            steering_neutral: 1500,
            steering_max: 2000,
            throttle_min: 1000,
            throttle_neutral: 1500,
            throttle_max: 2000,
        }
    }
}

/// Convert normalized value to PWM pulse width (microseconds)
///
/// # Arguments
///
/// * `normalized` - Normalized value (-1.0 to +1.0)
/// * `min` - Minimum pulse width (μs)
/// * `neutral` - Neutral pulse width (μs)
/// * `max` - Maximum pulse width (μs)
///
/// # Returns
///
/// Pulse width in microseconds
pub fn normalized_to_pulse(normalized: f32, min: u16, neutral: u16, max: u16) -> u16 {
    // Clamp to valid range
    let clamped = normalized.clamp(-1.0, 1.0);

    if clamped < 0.0 {
        // Negative: interpolate between min and neutral
        let range = (neutral - min) as f32;
        let offset = range * (-clamped);
        neutral - offset as u16
    } else {
        // Positive: interpolate between neutral and max
        let range = (max - neutral) as f32;
        let offset = range * clamped;
        neutral + offset as u16
    }
}

/// Convert pulse width to PWM duty cycle
///
/// For 50 Hz PWM (20 ms period):
/// - 1000 μs = 5.0% duty cycle
/// - 1500 μs = 7.5% duty cycle
/// - 2000 μs = 10.0% duty cycle
///
/// # Arguments
///
/// * `pulse_us` - Pulse width in microseconds
///
/// # Returns
///
/// Duty cycle (0.0 to 1.0)
pub fn pulse_to_duty_cycle(pulse_us: u16) -> f32 {
    // 50 Hz = 20,000 μs period
    const PERIOD_US: f32 = 20_000.0;
    pulse_us as f32 / PERIOD_US
}

/// Convert duty cycle to pulse width
///
/// Inverse of `pulse_to_duty_cycle`.
///
/// # Arguments
///
/// * `duty` - Duty cycle (0.0 to 1.0)
///
/// # Returns
///
/// Pulse width in microseconds
pub fn duty_cycle_to_pulse(duty: f32) -> u16 {
    const PERIOD_US: f32 = 20_000.0;
    (duty * PERIOD_US) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalized_to_pulse() {
        // Default config: 1000-1500-2000
        assert_eq!(normalized_to_pulse(-1.0, 1000, 1500, 2000), 1000);
        assert_eq!(normalized_to_pulse(0.0, 1000, 1500, 2000), 1500);
        assert_eq!(normalized_to_pulse(1.0, 1000, 1500, 2000), 2000);

        // Intermediate values
        assert_eq!(normalized_to_pulse(-0.5, 1000, 1500, 2000), 1250);
        assert_eq!(normalized_to_pulse(0.5, 1000, 1500, 2000), 1750);
    }

    #[test]
    fn test_pulse_to_duty_cycle() {
        // 50 Hz = 20,000 μs period
        assert!((pulse_to_duty_cycle(1000) - 0.05).abs() < 0.0001); // 5%
        assert!((pulse_to_duty_cycle(1500) - 0.075).abs() < 0.0001); // 7.5%
        assert!((pulse_to_duty_cycle(2000) - 0.10).abs() < 0.0001); // 10%
    }

    #[test]
    fn test_duty_cycle_to_pulse() {
        assert_eq!(duty_cycle_to_pulse(0.05), 1000);
        assert_eq!(duty_cycle_to_pulse(0.075), 1500);
        assert_eq!(duty_cycle_to_pulse(0.10), 2000);
    }

    #[test]
    fn test_actuator_config_default() {
        let config = ActuatorConfig::default();
        assert_eq!(config.steering_min, 1000);
        assert_eq!(config.steering_neutral, 1500);
        assert_eq!(config.steering_max, 2000);
        assert_eq!(config.throttle_min, 1000);
        assert_eq!(config.throttle_neutral, 1500);
        assert_eq!(config.throttle_max, 2000);
    }

    #[test]
    fn test_normalized_to_pulse_clamp() {
        // Values outside [-1.0, +1.0] should be clamped
        assert_eq!(normalized_to_pulse(-2.0, 1000, 1500, 2000), 1000);
        assert_eq!(normalized_to_pulse(2.0, 1000, 1500, 2000), 2000);
    }

    #[test]
    fn test_asymmetric_calibration() {
        // Asymmetric range: 1000-1400-2000 (neutral closer to min)
        assert_eq!(normalized_to_pulse(-1.0, 1000, 1400, 2000), 1000);
        assert_eq!(normalized_to_pulse(0.0, 1000, 1400, 2000), 1400);
        assert_eq!(normalized_to_pulse(1.0, 1000, 1400, 2000), 2000);

        // Check proportional interpolation
        assert_eq!(normalized_to_pulse(-0.5, 1000, 1400, 2000), 1200);
        assert_eq!(normalized_to_pulse(0.5, 1000, 1400, 2000), 1700);
    }
}
