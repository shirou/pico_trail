//! Mock PWM implementation for testing

use crate::platform::{
    Result,
    error::{PlatformError, PwmError},
    traits::{PwmConfig, PwmInterface},
};

/// Mock PWM implementation
///
/// Tracks PWM state (duty cycle, frequency, enabled) for test verification.
#[derive(Debug)]
pub struct MockPwm {
    duty_cycle: f32,
    frequency: u32,
    enabled: bool,
}

impl MockPwm {
    /// Create a new mock PWM
    pub fn new(config: PwmConfig) -> Self {
        Self {
            duty_cycle: config.duty_cycle,
            frequency: config.frequency,
            enabled: false,
        }
    }
}

impl PwmInterface for MockPwm {
    fn set_duty_cycle(&mut self, duty_cycle: f32) -> Result<()> {
        if !(0.0..=1.0).contains(&duty_cycle) {
            return Err(PlatformError::Pwm(PwmError::InvalidDutyCycle));
        }
        self.duty_cycle = duty_cycle;
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_pwm_duty_cycle() {
        let mut pwm = MockPwm::new(PwmConfig::default());
        assert_eq!(pwm.duty_cycle(), 0.0);

        pwm.set_duty_cycle(0.5).unwrap();
        assert_eq!(pwm.duty_cycle(), 0.5);

        // Test invalid duty cycle
        assert!(pwm.set_duty_cycle(-0.1).is_err());
        assert!(pwm.set_duty_cycle(1.1).is_err());
    }

    #[test]
    fn test_mock_pwm_frequency() {
        let mut pwm = MockPwm::new(PwmConfig::default());
        assert_eq!(pwm.frequency(), 50);

        pwm.set_frequency(100).unwrap();
        assert_eq!(pwm.frequency(), 100);

        // Test invalid frequency
        assert!(pwm.set_frequency(0).is_err());
    }

    #[test]
    fn test_mock_pwm_enable() {
        let mut pwm = MockPwm::new(PwmConfig::default());
        assert!(!pwm.is_enabled());

        pwm.enable();
        assert!(pwm.is_enabled());

        pwm.disable();
        assert!(!pwm.is_enabled());
    }
}
