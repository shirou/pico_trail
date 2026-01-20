//! Motor driver abstraction
//!
//! This module provides a zero-cost abstraction for controlling DC motors via H-bridge drivers,
//! with support for forward/reverse motion, braking, and speed control.
//!
//! # Features
//!
//! - Platform-independent `Motor` trait for generic motor control
//! - H-bridge motor driver implementation (DRV8837 2-pin PWM control)
//! - Motor groups for coordinating multiple motors (differential drive)
//! - Zero-cost abstraction with inline trait methods
//!
//! # Design
//!
//! This module is pure `no_std` with no feature gates. Platform-specific
//! PWM implementations belong in the firmware crate. Armed state checking
//! is the responsibility of the caller (via `MotorGroup::set_group_speed`).

/// Motor control error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorError {
    /// System not armed - motors cannot run
    NotArmed,
    /// Speed value outside [-1.0, +1.0] range
    InvalidSpeed,
    /// Hardware PWM channel unavailable or initialization failed
    HardwareFault,
}

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

/// Motor control trait (platform-independent)
///
/// This trait defines the interface for controlling a single motor with variable speed
/// in forward and reverse directions, plus stop (coast) and brake operations.
///
/// Speed values are normalized to [-1.0, +1.0]:
/// - `+1.0` = full forward
/// - `0.0` = stopped
/// - `-1.0` = full reverse
pub trait Motor {
    /// Set motor speed and direction
    ///
    /// # Arguments
    ///
    /// * `speed` - Motor speed in range [-1.0, +1.0]
    ///   - Positive values: forward motion
    ///   - Negative values: reverse motion
    ///   - Zero: stop (coast)
    ///
    /// # Errors
    ///
    /// Returns `MotorError::InvalidSpeed` if speed is outside [-1.0, +1.0] range.
    /// Returns `MotorError::HardwareFault` if PWM hardware fails.
    fn set_speed(&mut self, speed: f32) -> Result<(), MotorError>;

    /// Stop motor (coast mode - high-Z state)
    ///
    /// Motor freewheels to a stop. This is the lowest-power state.
    ///
    /// # Errors
    ///
    /// Returns `MotorError::HardwareFault` if PWM hardware fails.
    fn stop(&mut self) -> Result<(), MotorError>;

    /// Brake motor (short brake - active braking)
    ///
    /// Motor actively resists rotation. This provides faster stopping than coast.
    ///
    /// # Errors
    ///
    /// Returns `MotorError::HardwareFault` if PWM hardware fails.
    fn brake(&mut self) -> Result<(), MotorError>;
}

/// DRV8837 H-bridge motor driver
///
/// Generic motor driver for H-bridges controlled by two PWM pins (IN1, IN2).
/// This implementation follows the DRV8837 truth table for 2-pin PWM control.
///
/// # DRV8837 Truth Table
///
/// | IN1 | IN2 | Motor State                                |
/// |-----|-----|--------------------------------------------|
/// | 0   | 0   | Coast (High-Z, motor freewheels)           |
/// | PWM | 0   | Forward (speed = PWM duty cycle)           |
/// | 0   | PWM | Reverse (speed = PWM duty cycle)           |
/// | 1   | 1   | Brake (short brake, both terminals to GND) |
///
/// # Type Parameters
///
/// * `IN1` - PWM pin type for IN1 (first H-bridge input)
/// * `IN2` - PWM pin type for IN2 (second H-bridge input)
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

    /// Get reference to IN1 pin (for testing)
    #[cfg(test)]
    pub fn in1(&self) -> &IN1 {
        &self.in1
    }

    /// Get reference to IN2 pin (for testing)
    #[cfg(test)]
    pub fn in2(&self) -> &IN2 {
        &self.in2
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
    #[inline]
    fn set_speed(&mut self, speed: f32) -> Result<(), MotorError> {
        // Validate speed range
        if !(-1.0..=1.0).contains(&speed) {
            return Err(MotorError::InvalidSpeed);
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
    #[inline]
    fn stop(&mut self) -> Result<(), MotorError> {
        // Coast: IN1=LOW, IN2=LOW (High-Z)
        self.in1.set_duty(0.0)?;
        self.in2.set_duty(0.0)?;
        Ok(())
    }

    /// Brake motor (short brake - both pins HIGH)
    #[inline]
    fn brake(&mut self) -> Result<(), MotorError> {
        // Brake: IN1=HIGH, IN2=HIGH (short brake)
        self.in1.set_duty(1.0)?;
        self.in2.set_duty(1.0)?;
        Ok(())
    }
}

/// Motor group for coordinating multiple motors (e.g., left/right sides in differential drive)
///
/// This struct manages a group of motors and ensures armed state enforcement before
/// allowing motor operations. It's designed for coordinating motor groups in differential
/// drive configurations (e.g., Freenove 4WD Car with 4 motors).
pub struct MotorGroup<M: Motor> {
    motors: [M; 4],
}

impl<M: Motor> MotorGroup<M> {
    /// Create a new motor group with 4 motors
    ///
    /// # Arguments
    ///
    /// * `motors` - Array of 4 motors (typically: [M1, M2, M3, M4] for 4WD)
    pub fn new(motors: [M; 4]) -> Self {
        Self { motors }
    }

    /// Set speeds for all motors in the group
    ///
    /// # Arguments
    ///
    /// * `speeds` - Array of 4 speed values in range [-1.0, +1.0]
    /// * `is_armed` - Whether the system is armed
    ///
    /// # Errors
    ///
    /// Returns `MotorError::NotArmed` if `is_armed` is false.
    /// Returns `MotorError::InvalidSpeed` if any speed is outside [-1.0, +1.0].
    /// Returns `MotorError::HardwareFault` if motor hardware fails.
    #[inline]
    pub fn set_group_speed(&mut self, speeds: &[f32; 4], is_armed: bool) -> Result<(), MotorError> {
        if !is_armed {
            return Err(MotorError::NotArmed);
        }

        for (motor, &speed) in self.motors.iter_mut().zip(speeds.iter()) {
            motor.set_speed(speed)?;
        }
        Ok(())
    }

    /// Stop all motors (coast mode)
    #[inline]
    pub fn stop_all(&mut self) -> Result<(), MotorError> {
        for motor in self.motors.iter_mut() {
            motor.stop()?;
        }
        Ok(())
    }

    /// Brake all motors (active braking)
    #[inline]
    pub fn brake_all(&mut self) -> Result<(), MotorError> {
        for motor in self.motors.iter_mut() {
            motor.brake()?;
        }
        Ok(())
    }

    /// Get immutable reference to a specific motor
    pub fn get_motor(&self, index: usize) -> Option<&M> {
        self.motors.get(index)
    }

    /// Get mutable reference to a specific motor
    pub fn get_motor_mut(&mut self, index: usize) -> Option<&mut M> {
        self.motors.get_mut(index)
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

    // Mock motor for testing MotorGroup
    #[derive(Debug, Clone)]
    struct MockMotor {
        speed: f32,
        stopped: bool,
        braked: bool,
    }

    impl MockMotor {
        fn new() -> Self {
            Self {
                speed: 0.0,
                stopped: false,
                braked: false,
            }
        }
    }

    impl Motor for MockMotor {
        fn set_speed(&mut self, speed: f32) -> Result<(), MotorError> {
            if !(-1.0..=1.0).contains(&speed) {
                return Err(MotorError::InvalidSpeed);
            }
            self.speed = speed;
            self.stopped = false;
            self.braked = false;
            Ok(())
        }

        fn stop(&mut self) -> Result<(), MotorError> {
            self.speed = 0.0;
            self.stopped = true;
            self.braked = false;
            Ok(())
        }

        fn brake(&mut self) -> Result<(), MotorError> {
            self.speed = 0.0;
            self.stopped = false;
            self.braked = true;
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
        assert_eq!(motor.in1().get_duty(), 0.75);
        assert_eq!(motor.in2().get_duty(), 0.0);
    }

    #[test]
    fn test_hbridge_motor_reverse() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Set reverse speed
        assert!(motor.set_speed(-0.5).is_ok());

        // Verify IN1=0%, IN2=50% (reverse)
        assert_eq!(motor.in1().get_duty(), 0.0);
        assert_eq!(motor.in2().get_duty(), 0.5);
    }

    #[test]
    fn test_hbridge_motor_coast() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Set zero speed (coast)
        assert!(motor.set_speed(0.0).is_ok());

        // Verify IN1=0%, IN2=0% (coast)
        assert_eq!(motor.in1().get_duty(), 0.0);
        assert_eq!(motor.in2().get_duty(), 0.0);
    }

    #[test]
    fn test_hbridge_motor_stop() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Stop motor
        assert!(motor.stop().is_ok());

        // Verify IN1=0%, IN2=0% (coast)
        assert_eq!(motor.in1().get_duty(), 0.0);
        assert_eq!(motor.in2().get_duty(), 0.0);
    }

    #[test]
    fn test_hbridge_motor_brake() {
        let in1 = MockPwmPin::new();
        let in2 = MockPwmPin::new();
        let mut motor = HBridgeMotor::new(in1, in2);

        // Brake motor
        assert!(motor.brake().is_ok());

        // Verify IN1=100%, IN2=100% (short brake)
        assert_eq!(motor.in1().get_duty(), 1.0);
        assert_eq!(motor.in2().get_duty(), 1.0);
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
        assert_eq!(motor.in1().get_duty(), 1.0); // Forward: IN1=PWM, IN2=LOW

        assert!(motor.set_speed(-1.0).is_ok());
        assert_eq!(motor.in2().get_duty(), 1.0); // Reverse: IN1=LOW, IN2=PWM
    }

    #[test]
    fn test_motor_group_armed_check() {
        let motors = [
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
        ];
        let mut group = MotorGroup::new(motors);

        // Test disarmed (should fail)
        let speeds = [0.5, 0.5, 0.5, 0.5];
        let result = group.set_group_speed(&speeds, false);
        assert_eq!(result, Err(MotorError::NotArmed));

        // Verify motors didn't move when disarmed
        for i in 0..4 {
            assert_eq!(group.get_motor(i).unwrap().speed, 0.0);
        }

        // Test armed (should succeed)
        let result = group.set_group_speed(&speeds, true);
        assert!(result.is_ok());

        // Verify all motors received the speed
        for i in 0..4 {
            assert_eq!(group.get_motor(i).unwrap().speed, 0.5);
        }
    }

    #[test]
    fn test_motor_group_set_speeds() {
        let motors = [
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
        ];
        let mut group = MotorGroup::new(motors);

        // Set different speeds for each motor
        let speeds = [0.5, -0.5, 0.75, -0.25];
        let result = group.set_group_speed(&speeds, true);
        assert!(result.is_ok());

        // Verify each motor has correct speed
        assert_eq!(group.get_motor(0).unwrap().speed, 0.5);
        assert_eq!(group.get_motor(1).unwrap().speed, -0.5);
        assert_eq!(group.get_motor(2).unwrap().speed, 0.75);
        assert_eq!(group.get_motor(3).unwrap().speed, -0.25);
    }

    #[test]
    fn test_motor_group_invalid_speed() {
        let motors = [
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
        ];
        let mut group = MotorGroup::new(motors);

        // Test invalid speed (> 1.0)
        let speeds = [1.5, 0.5, 0.5, 0.5];
        let result = group.set_group_speed(&speeds, true);
        assert_eq!(result, Err(MotorError::InvalidSpeed));

        // Test invalid speed (< -1.0)
        let speeds = [-1.5, 0.5, 0.5, 0.5];
        let result = group.set_group_speed(&speeds, true);
        assert_eq!(result, Err(MotorError::InvalidSpeed));
    }

    #[test]
    fn test_motor_group_stop_all() {
        let motors = [
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
        ];
        let mut group = MotorGroup::new(motors);

        // Set motors to some speed first
        let speeds = [0.5, 0.5, 0.5, 0.5];
        group.set_group_speed(&speeds, true).unwrap();

        // Stop all motors
        let result = group.stop_all();
        assert!(result.is_ok());

        // Verify all motors are stopped
        for i in 0..4 {
            let motor = group.get_motor(i).unwrap();
            assert_eq!(motor.speed, 0.0);
            assert!(motor.stopped);
            assert!(!motor.braked);
        }
    }

    #[test]
    fn test_motor_group_brake_all() {
        let motors = [
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
        ];
        let mut group = MotorGroup::new(motors);

        // Set motors to some speed first
        let speeds = [0.5, 0.5, 0.5, 0.5];
        group.set_group_speed(&speeds, true).unwrap();

        // Brake all motors
        let result = group.brake_all();
        assert!(result.is_ok());

        // Verify all motors are braked
        for i in 0..4 {
            let motor = group.get_motor(i).unwrap();
            assert_eq!(motor.speed, 0.0);
            assert!(!motor.stopped);
            assert!(motor.braked);
        }
    }

    #[test]
    fn test_motor_group_get_motor() {
        let motors = [
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
        ];
        let group = MotorGroup::new(motors);

        // Test valid indices
        assert!(group.get_motor(0).is_some());
        assert!(group.get_motor(3).is_some());

        // Test invalid index
        assert!(group.get_motor(4).is_none());
    }

    #[test]
    fn test_motor_group_get_motor_mut() {
        let motors = [
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
            MockMotor::new(),
        ];
        let mut group = MotorGroup::new(motors);

        // Modify a motor directly
        if let Some(motor) = group.get_motor_mut(0) {
            motor.set_speed(0.75).unwrap();
        }

        // Verify the change
        assert_eq!(group.get_motor(0).unwrap().speed, 0.75);
    }
}
