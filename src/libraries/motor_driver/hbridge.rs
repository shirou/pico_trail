//! H-bridge motor driver implementation
//!
//! This module implements motor control for H-bridge drivers like the DRV8837,
//! which use two PWM pins (IN1, IN2) to control motor direction and speed.
//!
//! ## DRV8837 Truth Table
//!
//! | IN1 | IN2 | Motor State                                |
//! |-----|-----|--------------------------------------------|
//! | 0   | 0   | Coast (High-Z, motor freewheels)           |
//! | PWM | 0   | Forward (speed = PWM duty cycle)           |
//! | 0   | PWM | Reverse (speed = PWM duty cycle)           |
//! | 1   | 1   | Brake (short brake, both terminals to GND) |
//!
//! ## References
//!
//! - [DRV8837 Datasheet](https://www.ti.com/product/DRV8837)

use core::sync::atomic::{AtomicU32, Ordering};

use super::{Motor, MotorError};

/// Counter for sampling motor speed logs (every 10th call)
static SET_SPEED_LOG_COUNTER: AtomicU32 = AtomicU32::new(0);

/// PWM pin abstraction for motor control
///
/// This trait defines the interface for controlling a single PWM output pin.
/// Platform-specific implementations wrap their HAL's PWM types.
pub trait PwmPin {
    /// Set PWM duty cycle as a fraction [0.0, 1.0]
    ///
    /// # Arguments
    ///
    /// * `duty` - Duty cycle as fraction (0.0 = 0%, 1.0 = 100%)
    ///
    /// # Errors
    ///
    /// Returns `MotorError::HardwareFault` if PWM hardware fails.
    fn set_duty(&mut self, duty: f32) -> Result<(), MotorError>;
}

/// DRV8837 H-bridge motor driver
///
/// Generic motor driver for H-bridges controlled by two PWM pins (IN1, IN2).
/// This implementation follows the DRV8837 truth table for 2-pin PWM control.
///
/// # Type Parameters
///
/// * `IN1` - PWM pin type for IN1 (first H-bridge input)
/// * `IN2` - PWM pin type for IN2 (second H-bridge input)
///
/// # Performance
///
/// All methods are marked `#[inline]` for zero-cost abstraction. With LTO enabled,
/// the trait methods compile directly to GPIO writes with no function call overhead.
pub struct HBridgeMotor<IN1, IN2>
where
    IN1: PwmPin,
    IN2: PwmPin,
{
    in1: IN1,
    in2: IN2,
}

impl<IN1, IN2> HBridgeMotor<IN1, IN2>
where
    IN1: PwmPin,
    IN2: PwmPin,
{
    /// Create new H-bridge motor with initialized PWM pins
    ///
    /// # Arguments
    ///
    /// * `in1` - PWM pin for H-bridge IN1 input
    /// * `in2` - PWM pin for H-bridge IN2 input
    ///
    /// # Safety
    ///
    /// The PWM pins must be properly initialized before passing to this constructor.
    /// The motor will be in coast state (both pins LOW) after creation.
    pub fn new(in1: IN1, in2: IN2) -> Self {
        Self { in1, in2 }
    }
}

impl<IN1, IN2> Motor for HBridgeMotor<IN1, IN2>
where
    IN1: PwmPin,
    IN2: PwmPin,
{
    /// Set motor speed and direction using DRV8837 truth table
    ///
    /// # DRV8837 Control Logic
    ///
    /// - Forward (speed > 0): IN1=PWM, IN2=LOW
    /// - Reverse (speed < 0): IN1=LOW, IN2=PWM
    /// - Coast (speed = 0): IN1=LOW, IN2=LOW
    ///
    /// # Errors
    ///
    /// Returns `MotorError::InvalidSpeed` if speed is outside [-1.0, +1.0].
    #[inline]
    fn set_speed(&mut self, speed: f32) -> Result<(), MotorError> {
        // Validate speed range
        if !(-1.0..=1.0).contains(&speed) {
            return Err(MotorError::InvalidSpeed);
        }

        // Sample logging: only log every 10th call to reduce noise
        let count = SET_SPEED_LOG_COUNTER.fetch_add(1, Ordering::Relaxed);
        if count.is_multiple_of(100) {
            crate::log_debug!("Motor set_speed: {}", speed);
        }

        // DRV8837 truth table implementation
        if speed > 0.0 {
            // Forward: IN1=PWM, IN2=LOW
            self.in1.set_duty(speed)?;
            self.in2.set_duty(0.0)?;
        } else if speed < 0.0 {
            // Reverse: IN1=LOW, IN2=PWM
            self.in1.set_duty(0.0)?;
            self.in2.set_duty(speed.abs())?;
        } else {
            // Coast: IN1=LOW, IN2=LOW
            self.in1.set_duty(0.0)?;
            self.in2.set_duty(0.0)?;
        }
        Ok(())
    }

    /// Stop motor (coast mode - both pins LOW)
    ///
    /// Motor enters high-impedance state and freewheels to a stop.
    #[inline]
    fn stop(&mut self) -> Result<(), MotorError> {
        // Coast: IN1=LOW, IN2=LOW (High-Z)
        self.in1.set_duty(0.0)?;
        self.in2.set_duty(0.0)?;
        Ok(())
    }

    /// Brake motor (short brake - both pins HIGH)
    ///
    /// Motor actively resists rotation for faster stopping.
    /// Both H-bridge outputs are driven to the same state, creating a short brake.
    #[inline]
    fn brake(&mut self) -> Result<(), MotorError> {
        // Brake: IN1=HIGH, IN2=HIGH (short brake)
        self.in1.set_duty(1.0)?;
        self.in2.set_duty(1.0)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mock PWM pin for testing (no hardware dependencies)
    #[derive(Debug)]
    struct MockPwmPin {
        duty: f32,
    }

    impl MockPwmPin {
        fn new() -> Self {
            Self { duty: 0.0 }
        }

        fn get_duty(&self) -> f32 {
            self.duty
        }
    }

    impl PwmPin for MockPwmPin {
        fn set_duty(&mut self, duty: f32) -> Result<(), MotorError> {
            self.duty = duty;
            Ok(())
        }
    }

    #[test]
    fn test_hbridge_motor_forward() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Set forward speed
        assert!(motor.set_speed(0.75).is_ok());

        // Verify IN1=75%, IN2=0% (forward)
        assert_eq!(motor.in1.get_duty(), 0.75);
        assert_eq!(motor.in2.get_duty(), 0.0);
    }

    #[test]
    fn test_hbridge_motor_reverse() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Set reverse speed
        assert!(motor.set_speed(-0.5).is_ok());

        // Verify IN1=0%, IN2=50% (reverse)
        assert_eq!(motor.in1.get_duty(), 0.0);
        assert_eq!(motor.in2.get_duty(), 0.5);
    }

    #[test]
    fn test_hbridge_motor_coast() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Set zero speed (coast)
        assert!(motor.set_speed(0.0).is_ok());

        // Verify IN1=0%, IN2=0% (coast)
        assert_eq!(motor.in1.get_duty(), 0.0);
        assert_eq!(motor.in2.get_duty(), 0.0);
    }

    #[test]
    fn test_hbridge_motor_stop() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Stop motor
        assert!(motor.stop().is_ok());

        // Verify IN1=0%, IN2=0% (coast)
        assert_eq!(motor.in1.get_duty(), 0.0);
        assert_eq!(motor.in2.get_duty(), 0.0);
    }

    #[test]
    fn test_hbridge_motor_brake() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Brake motor
        assert!(motor.brake().is_ok());

        // Verify IN1=100%, IN2=100% (short brake)
        assert_eq!(motor.in1.get_duty(), 1.0);
        assert_eq!(motor.in2.get_duty(), 1.0);
    }

    #[test]
    fn test_invalid_speed() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Test invalid speeds
        assert_eq!(motor.set_speed(1.5), Err(MotorError::InvalidSpeed));
        assert_eq!(motor.set_speed(-1.5), Err(MotorError::InvalidSpeed));
        assert_eq!(motor.set_speed(2.0), Err(MotorError::InvalidSpeed));
        assert_eq!(motor.set_speed(-2.0), Err(MotorError::InvalidSpeed));
    }

    #[test]
    fn test_valid_speed_boundaries() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Test boundary values
        assert!(motor.set_speed(1.0).is_ok());
        assert_eq!(motor.in1.get_duty(), 1.0); // Forward: IN1=PWM, IN2=LOW

        assert!(motor.set_speed(-1.0).is_ok());
        assert_eq!(motor.in2.get_duty(), 1.0); // Reverse: IN1=LOW, IN2=PWM
    }
}
