//! Hold Mode
//!
//! Simplest control mode: zeros all actuator outputs unconditionally.
//! Serves as the primary failsafe fallback mode.

extern crate alloc;
use alloc::boxed::Box;

use crate::servo::ActuatorInterface;

use super::traits::Mode;

/// Hold Mode
///
/// Zeros all actuator outputs on enter, every update, and exit.
/// Has no external dependencies and always succeeds entry.
pub struct HoldMode {
    actuators: Box<dyn ActuatorInterface>,
}

impl HoldMode {
    /// Create new Hold mode
    ///
    /// # Arguments
    ///
    /// * `actuators` - Actuator interface for steering and throttle
    pub fn new(actuators: Box<dyn ActuatorInterface>) -> Self {
        Self { actuators }
    }
}

impl Mode for HoldMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Entering Hold mode");
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }

    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Exiting Hold mode");
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Hold"
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;

    struct MockActuator {
        steering: f32,
        throttle: f32,
    }

    impl MockActuator {
        fn new() -> Self {
            Self {
                steering: 0.0,
                throttle: 0.0,
            }
        }
    }

    impl ActuatorInterface for MockActuator {
        fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str> {
            self.steering = normalized;
            Ok(())
        }
        fn set_throttle(&mut self, normalized: f32) -> Result<(), &'static str> {
            self.throttle = normalized;
            Ok(())
        }
        fn get_steering(&self) -> f32 {
            self.steering
        }
        fn get_throttle(&self) -> f32 {
            self.throttle
        }
    }

    #[test]
    fn test_hold_mode_enter_exit() {
        let mut mode = HoldMode::new(Box::new(MockActuator::new()));

        assert!(mode.enter().is_ok());
        assert!(
            mode.actuators.get_steering().abs() < 0.001,
            "Enter should zero steering"
        );
        assert!(
            mode.actuators.get_throttle().abs() < 0.001,
            "Enter should zero throttle"
        );

        assert!(mode.exit().is_ok());
        assert!(
            mode.actuators.get_steering().abs() < 0.001,
            "Exit should zero steering"
        );
        assert!(
            mode.actuators.get_throttle().abs() < 0.001,
            "Exit should zero throttle"
        );
    }

    #[test]
    fn test_hold_mode_update_zeros_actuators() {
        let mut mock = MockActuator::new();
        mock.steering = 0.5;
        mock.throttle = 0.8;
        let mut mode = HoldMode::new(Box::new(mock));

        mode.update(0.02).unwrap();

        assert!(
            mode.actuators.get_steering().abs() < 0.001,
            "Update should zero steering, got {}",
            mode.actuators.get_steering()
        );
        assert!(
            mode.actuators.get_throttle().abs() < 0.001,
            "Update should zero throttle, got {}",
            mode.actuators.get_throttle()
        );
    }

    #[test]
    fn test_hold_mode_name() {
        let mode = HoldMode::new(Box::new(MockActuator::new()));
        assert_eq!(mode.name(), "Hold");
    }

    #[test]
    fn test_hold_mode_size() {
        assert!(
            core::mem::size_of::<HoldMode>() <= 32,
            "HoldMode should be <= 32 bytes, got {}",
            core::mem::size_of::<HoldMode>()
        );
    }
}
