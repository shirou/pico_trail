//! Simulated PWM peripheral for SITL.
//!
//! Stores duty cycle and frequency, enabling the bridge to read
//! actuator commands from the autopilot's PWM outputs.

/// Simulated PWM channel with duty cycle tracking.
#[derive(Debug)]
pub struct SitlPwm {
    duty_cycle: f32,
    frequency: u32,
    enabled: bool,
    pin: u8,
}

impl SitlPwm {
    /// Create a new SITL PWM channel on the given pin.
    pub fn new(pin: u8, frequency: u32, duty_cycle: f32) -> Self {
        Self {
            duty_cycle: duty_cycle.clamp(0.0, 1.0),
            frequency,
            enabled: false,
            pin,
        }
    }

    /// Set duty cycle (0.0 = 0%, 1.0 = 100%).
    pub fn set_duty_cycle(&mut self, duty_cycle: f32) {
        self.duty_cycle = duty_cycle.clamp(0.0, 1.0);
    }

    /// Get current duty cycle.
    pub fn duty_cycle(&self) -> f32 {
        self.duty_cycle
    }

    /// Set PWM frequency in Hz.
    pub fn set_frequency(&mut self, frequency: u32) {
        self.frequency = frequency;
    }

    /// Get current frequency in Hz.
    pub fn frequency(&self) -> u32 {
        self.frequency
    }

    /// Enable PWM output.
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disable PWM output.
    pub fn disable(&mut self) {
        self.enabled = false;
    }

    /// Check if PWM is enabled.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Get the pin number.
    pub fn pin(&self) -> u8 {
        self.pin
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_duty_cycle() {
        let mut pwm = SitlPwm::new(0, 50, 0.0);
        assert_eq!(pwm.duty_cycle(), 0.0);

        pwm.set_duty_cycle(0.5);
        assert_eq!(pwm.duty_cycle(), 0.5);

        pwm.set_duty_cycle(1.0);
        assert_eq!(pwm.duty_cycle(), 1.0);
    }

    #[test]
    fn test_duty_cycle_clamping() {
        let mut pwm = SitlPwm::new(0, 50, 0.0);
        pwm.set_duty_cycle(1.5);
        assert_eq!(pwm.duty_cycle(), 1.0);

        pwm.set_duty_cycle(-0.5);
        assert_eq!(pwm.duty_cycle(), 0.0);
    }

    #[test]
    fn test_frequency() {
        let mut pwm = SitlPwm::new(0, 50, 0.0);
        assert_eq!(pwm.frequency(), 50);

        pwm.set_frequency(400);
        assert_eq!(pwm.frequency(), 400);
    }

    #[test]
    fn test_enable_disable() {
        let mut pwm = SitlPwm::new(0, 50, 0.0);
        assert!(!pwm.is_enabled());

        pwm.enable();
        assert!(pwm.is_enabled());

        pwm.disable();
        assert!(!pwm.is_enabled());
    }
}
