//! Actuator abstraction for rover steering and throttle
//!
//! This module provides a safe abstraction layer between control modes and PWM hardware:
//! - Normalized commands (-1.0 to +1.0)
//! - Armed state enforcement (CRITICAL SAFETY)
//! - PWM conversion (pulse width → duty cycle)
//! - Calibration support (min/neutral/max)
//!
//! ## Safety
//!
//! **CRITICAL**: All actuator commands enforce armed state check.
//! When disarmed, outputs are overridden to neutral (0.0) regardless of commanded value.
//!
//! ## References
//!
//! - ADR-b8snw-actuator-abstraction-rover: Actuator design
//! - NFR-jng15-actuator-failsafe: Safety requirements

use crate::communication::mavlink::state::SystemState;
use crate::platform::traits::pwm::PwmInterface;

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
    /// Enforces armed check: outputs neutral if disarmed.
    fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str>;

    /// Set throttle command
    ///
    /// # Arguments
    ///
    /// * `normalized` - Throttle command (-1.0 reverse, 0.0 stop, +1.0 forward)
    ///
    /// # Safety
    ///
    /// Enforces armed check: outputs neutral if disarmed.
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

/// Actuator implementation for rover
///
/// Manages steering servo and throttle ESC with safety enforcement.
pub struct Actuators<'a> {
    steering_pwm: &'a mut dyn PwmInterface,
    throttle_pwm: &'a mut dyn PwmInterface,
    system_state: &'a SystemState,
    config: ActuatorConfig,
    current_steering: f32,
    current_throttle: f32,
}

impl<'a> Actuators<'a> {
    /// Create new actuators
    ///
    /// # Arguments
    ///
    /// * `steering_pwm` - PWM interface for steering servo
    /// * `throttle_pwm` - PWM interface for throttle ESC
    /// * `system_state` - System state (for armed check)
    /// * `config` - Actuator calibration configuration
    pub fn new(
        steering_pwm: &'a mut dyn PwmInterface,
        throttle_pwm: &'a mut dyn PwmInterface,
        system_state: &'a SystemState,
        config: ActuatorConfig,
    ) -> Self {
        Self {
            steering_pwm,
            throttle_pwm,
            system_state,
            config,
            current_steering: 0.0,
            current_throttle: 0.0,
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
    fn normalized_to_pulse(normalized: f32, min: u16, neutral: u16, max: u16) -> u16 {
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
    fn pulse_to_duty_cycle(pulse_us: u16) -> f32 {
        // 50 Hz = 20,000 μs period
        const PERIOD_US: f32 = 20_000.0;
        pulse_us as f32 / PERIOD_US
    }
}

impl<'a> ActuatorInterface for Actuators<'a> {
    fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str> {
        // SAFETY: Enforce armed check
        let value = if self.system_state.is_armed() {
            normalized
        } else {
            0.0 // Neutral when disarmed
        };

        // Store current value
        self.current_steering = value;

        // Convert to pulse width
        let pulse_us = Self::normalized_to_pulse(
            value,
            self.config.steering_min,
            self.config.steering_neutral,
            self.config.steering_max,
        );

        // Convert to duty cycle
        let duty = Self::pulse_to_duty_cycle(pulse_us);

        // Set PWM
        self.steering_pwm
            .set_duty_cycle(duty)
            .map_err(|_| "PWM error")?;

        Ok(())
    }

    fn set_throttle(&mut self, normalized: f32) -> Result<(), &'static str> {
        // SAFETY: Enforce armed check
        let value = if self.system_state.is_armed() {
            normalized
        } else {
            0.0 // Neutral when disarmed
        };

        // Store current value
        self.current_throttle = value;

        // Convert to pulse width
        let pulse_us = Self::normalized_to_pulse(
            value,
            self.config.throttle_min,
            self.config.throttle_neutral,
            self.config.throttle_max,
        );

        // Convert to duty cycle
        let duty = Self::pulse_to_duty_cycle(pulse_us);

        // Set PWM
        self.throttle_pwm
            .set_duty_cycle(duty)
            .map_err(|_| "PWM error")?;

        Ok(())
    }

    fn get_steering(&self) -> f32 {
        self.current_steering
    }

    fn get_throttle(&self) -> f32 {
        self.current_throttle
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::state::{ArmedState, SystemState};
    use crate::platform::traits::pwm::PwmInterface;
    use crate::platform::Result;

    // Mock PWM for testing
    struct MockPwm {
        duty_cycle: f32,
        frequency: u32,
        enabled: bool,
    }

    impl MockPwm {
        fn new() -> Self {
            Self {
                duty_cycle: 0.0,
                frequency: 50,
                enabled: true,
            }
        }
    }

    impl PwmInterface for MockPwm {
        fn set_duty_cycle(&mut self, duty_cycle: f32) -> Result<()> {
            if duty_cycle < 0.0 || duty_cycle > 1.0 {
                return Err(crate::platform::PlatformError::Pwm(
                    crate::platform::error::PwmError::InvalidDutyCycle,
                ));
            }
            self.duty_cycle = duty_cycle;
            Ok(())
        }

        fn duty_cycle(&self) -> f32 {
            self.duty_cycle
        }

        fn set_frequency(&mut self, frequency: u32) -> Result<()> {
            self.frequency = frequency;
            Ok(())
        }

        fn frequency(&self) -> u32 {
            self.frequency
        }

        fn enable(&mut self) {
            self.enabled = true;
        }

        fn disable(&mut self) {
            self.enabled = false;
        }

        fn is_enabled(&self) -> bool {
            self.enabled
        }
    }

    #[test]
    fn test_normalized_to_pulse() {
        // Default config: 1000-1500-2000
        assert_eq!(Actuators::normalized_to_pulse(-1.0, 1000, 1500, 2000), 1000);
        assert_eq!(Actuators::normalized_to_pulse(0.0, 1000, 1500, 2000), 1500);
        assert_eq!(Actuators::normalized_to_pulse(1.0, 1000, 1500, 2000), 2000);

        // Intermediate values
        assert_eq!(Actuators::normalized_to_pulse(-0.5, 1000, 1500, 2000), 1250);
        assert_eq!(Actuators::normalized_to_pulse(0.5, 1000, 1500, 2000), 1750);
    }

    #[test]
    fn test_pulse_to_duty_cycle() {
        // 50 Hz = 20,000 μs period
        assert!((Actuators::pulse_to_duty_cycle(1000) - 0.05).abs() < 0.0001); // 5%
        assert!((Actuators::pulse_to_duty_cycle(1500) - 0.075).abs() < 0.0001); // 7.5%
        assert!((Actuators::pulse_to_duty_cycle(2000) - 0.10).abs() < 0.0001); // 10%
    }

    #[test]
    fn test_armed_state_enforcement() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let system_state = SystemState::new(); // Disarmed by default
        let config = ActuatorConfig::default();

        {
            let mut actuators =
                Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);

            // CRITICAL TEST: Disarmed prevents motor output
            // Command full throttle while disarmed
            actuators.set_throttle(1.0).unwrap();
            assert_eq!(actuators.get_throttle(), 0.0);
        }

        // Verify neutral output (1500 μs = 7.5% duty)
        assert!((throttle_pwm.duty_cycle() - 0.075).abs() < 0.001);
    }

    #[test]
    fn test_armed_allows_output() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let config = ActuatorConfig::default();

        {
            let mut actuators =
                Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);

            // Command full throttle while armed
            actuators.set_throttle(1.0).unwrap();
            assert_eq!(actuators.get_throttle(), 1.0);
        }

        // Verify full output (2000 μs = 10% duty)
        assert!((throttle_pwm.duty_cycle() - 0.10).abs() < 0.001);
    }

    #[test]
    fn test_disarm_during_throttle() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let config = ActuatorConfig::default();

        // Test armed state
        {
            let mut system_state = SystemState::new();
            system_state.armed = ArmedState::Armed;
            let mut actuators =
                Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);

            actuators.set_throttle(1.0).unwrap();
        }
        // Verify output while armed (2000 μs = 10% duty)
        assert!((throttle_pwm.duty_cycle() - 0.10).abs() < 0.001);

        // Test disarmed state
        {
            let system_state = SystemState::new(); // Disarmed by default
            let mut actuators =
                Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);

            // Command throttle while disarmed (should be neutral)
            actuators.set_throttle(1.0).unwrap();
            assert_eq!(actuators.get_throttle(), 0.0);
        }

        // Verify immediate neutral output (1500 μs = 7.5% duty)
        assert!((throttle_pwm.duty_cycle() - 0.075).abs() < 0.001);
    }

    #[test]
    fn test_steering_commands() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let config = ActuatorConfig::default();

        {
            let mut actuators =
                Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);

            // Full left
            actuators.set_steering(-1.0).unwrap();
        }
        assert!((steering_pwm.duty_cycle() - 0.05).abs() < 0.001); // 1000 μs

        {
            let mut actuators =
                Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);

            // Center
            actuators.set_steering(0.0).unwrap();
        }
        assert!((steering_pwm.duty_cycle() - 0.075).abs() < 0.001); // 1500 μs

        {
            let mut actuators =
                Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);

            // Full right
            actuators.set_steering(1.0).unwrap();
        }
        assert!((steering_pwm.duty_cycle() - 0.10).abs() < 0.001); // 2000 μs
    }
}
