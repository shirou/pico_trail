//! Motor driver abstraction (firmware wrapper)
//!
//! This module re-exports core motor types and provides platform-specific
//! H-bridge implementation with logging support.
//!
//! ## Features
//!
//! - Platform-independent `Motor` trait for generic motor control
//! - H-bridge motor driver implementation (DRV8837 2-pin PWM control) with logging
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

// Re-export core motor types
pub use pico_trail_core::motor::{Motor, MotorError, MotorGroup, PwmPin};

// Re-export firmware-specific HBridgeMotor with logging
pub use hbridge::HBridgeMotor;

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
