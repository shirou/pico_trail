//! Motor driver abstraction
//!
//! This module provides a zero-cost abstraction for controlling DC motors via H-bridge drivers,
//! with support for forward/reverse motion, braking, and speed control.
//!
//! ## Features
//!
//! - Platform-independent `Motor` trait for generic motor control
//! - H-bridge motor driver implementation (DRV8837 2-pin PWM control)
//! - Motor groups for coordinating multiple motors (differential drive)
//! - Armed state enforcement (motors disabled when system is disarmed)
//! - Zero-cost abstraction with inline trait methods
//!
//! ## Example - Single Motor
//!
//! ```no_run
//! use motor_driver::{Motor, HBridgeMotor};
//!
//! // Create H-bridge motor with PWM pins
//! let mut motor = HBridgeMotor::new(pwm_in1, pwm_in2);
//!
//! // Set motor speed (forward)
//! motor.set_speed(0.75)?; // 75% forward
//!
//! // Reverse
//! motor.set_speed(-0.5)?; // 50% reverse
//!
//! // Stop (coast)
//! motor.stop()?;
//!
//! // Brake (short brake)
//! motor.brake()?;
//! ```
//!
//! ## Example - Motor Group (Differential Drive)
//!
//! ```no_run
//! use pico_trail::libraries::motor_driver::{MotorGroup, Motor};
//!
//! // Initialize 4 motors for Freenove 4WD Car
//! // (motor initialization code is platform-specific, see examples/)
//! let motors = [motor1, motor2, motor3, motor4];
//! let mut motor_group = MotorGroup::new(motors);
//!
//! // Control all motors (checks armed state)
//! let is_armed = system_state.is_armed();
//! let speeds = [0.5, 0.5, 0.5, 0.5]; // All motors 50% forward
//! motor_group.set_group_speed(&speeds, is_armed)?;
//!
//! // Differential drive: left motors forward, right motors reverse (turn right)
//! let speeds = [0.5, 0.5, -0.5, -0.5]; // Left: M1+M2 forward, Right: M3+M4 reverse
//! motor_group.set_group_speed(&speeds, is_armed)?;
//!
//! // Emergency stop
//! motor_group.brake_all()?;
//! ```

pub mod hbridge;

// Re-export main types
pub use hbridge::{HBridgeMotor, PwmPin};

/// Motor control error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "pico2_w", derive(defmt::Format))]
pub enum MotorError {
    /// System not armed - motors cannot run
    NotArmed,
    /// Speed value outside [-1.0, +1.0] range
    InvalidSpeed,
    /// Hardware PWM channel unavailable or initialization failed
    HardwareFault,
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
///
/// ## Safety
///
/// Motor implementations must enforce armed state checking. Motors cannot run when
/// the system is disarmed (enforced via `MotorError::NotArmed`).
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
    /// Returns `MotorError::NotArmed` if system is not armed.
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

/// Motor group for coordinating multiple motors (e.g., left/right sides in differential drive)
///
/// This struct manages a group of motors and ensures armed state enforcement before
/// allowing motor operations. It's designed for coordinating motor groups in differential
/// drive configurations (e.g., Freenove 4WD Car with 4 motors).
///
/// # Example
///
/// ```no_run
/// use motor_driver::{MotorGroup, HBridgeMotor};
///
/// // Create motors (M1, M2, M3, M4)
/// let motors = [motor1, motor2, motor3, motor4];
/// let mut motor_group = MotorGroup::new(motors);
///
/// // Set speeds for all motors (with armed check)
/// let is_armed = true;
/// let speeds = [0.5, 0.5, 0.5, 0.5]; // All motors 50% forward
/// motor_group.set_group_speed(&speeds, is_armed)?;
///
/// // Stop all motors
/// motor_group.stop_all()?;
///
/// // Brake all motors
/// motor_group.brake_all()?;
/// ```
pub struct MotorGroup<M: Motor> {
    motors: [M; 4],
}

impl<M: Motor> MotorGroup<M> {
    /// Create a new motor group with 4 motors
    ///
    /// # Arguments
    ///
    /// * `motors` - Array of 4 motors (typically: [M1, M2, M3, M4] for 4WD)
    ///
    /// # Example
    ///
    /// ```no_run
    /// let motors = [motor1, motor2, motor3, motor4];
    /// let motor_group = MotorGroup::new(motors);
    /// ```
    pub fn new(motors: [M; 4]) -> Self {
        Self { motors }
    }

    /// Set speeds for all motors in the group
    ///
    /// # Arguments
    ///
    /// * `speeds` - Array of 4 speed values in range [-1.0, +1.0]
    /// * `is_armed` - Whether the system is armed (from SystemState::is_armed())
    ///
    /// # Errors
    ///
    /// Returns `MotorError::NotArmed` if `is_armed` is false.
    /// Returns `MotorError::InvalidSpeed` if any speed is outside [-1.0, +1.0].
    /// Returns `MotorError::HardwareFault` if motor hardware fails.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use motor_driver::MotorGroup;
    ///
    /// let is_armed = system_state.is_armed();
    /// let speeds = [0.5, 0.5, 0.5, 0.5]; // All motors forward
    /// motor_group.set_group_speed(&speeds, is_armed)?;
    /// ```
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
    ///
    /// All motors will freewheel to a stop.
    ///
    /// # Errors
    ///
    /// Returns `MotorError::HardwareFault` if any motor hardware fails.
    #[inline]
    pub fn stop_all(&mut self) -> Result<(), MotorError> {
        for motor in self.motors.iter_mut() {
            motor.stop()?;
        }
        Ok(())
    }

    /// Brake all motors (active braking)
    ///
    /// All motors will actively resist rotation for faster stopping.
    ///
    /// # Errors
    ///
    /// Returns `MotorError::HardwareFault` if any motor hardware fails.
    #[inline]
    pub fn brake_all(&mut self) -> Result<(), MotorError> {
        for motor in self.motors.iter_mut() {
            motor.brake()?;
        }
        Ok(())
    }

    /// Get immutable reference to a specific motor
    ///
    /// # Arguments
    ///
    /// * `index` - Motor index (0-3)
    ///
    /// # Returns
    ///
    /// Reference to the motor at the specified index, or None if index is out of bounds
    pub fn get_motor(&self, index: usize) -> Option<&M> {
        self.motors.get(index)
    }

    /// Get mutable reference to a specific motor
    ///
    /// # Arguments
    ///
    /// * `index` - Motor index (0-3)
    ///
    /// # Returns
    ///
    /// Mutable reference to the motor at the specified index, or None if index is out of bounds
    pub fn get_motor_mut(&mut self, index: usize) -> Option<&mut M> {
        self.motors.get_mut(index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mock motor for testing (no hardware dependencies)
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
