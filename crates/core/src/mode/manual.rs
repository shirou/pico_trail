//! Manual Mode
//!
//! Direct RC control with no stabilization or autonomous behavior.
//!
//! ## Behavior
//!
//! - Read RC inputs (channel 1 = steering, channel 3 = throttle)
//! - Pass through directly to actuators
//! - Fail-safe to neutral on RC timeout
//! - Actuators neutral when disarmed (enforced by actuator layer)
//!
//! ## Safety
//!
//! - Multi-layer safety:
//!   1. RC timeout detection (1 second) -> neutral outputs
//!   2. Armed state check (actuator layer) -> neutral when disarmed
//! - No autonomous behavior - pilot in full control
//!
//! ## References
//!
//! - FR-uk0us-manual-mode: Manual mode requirements
//! - ADR-w9zpl-control-mode-architecture: Mode architecture
//! - ArduPilot Manual Mode: https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_manual.cpp

extern crate alloc;
use alloc::boxed::Box;

use crate::servo::ActuatorInterface;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

use super::Mode;

/// Manual Mode
///
/// Provides direct RC control with no stabilization.
pub struct ManualMode {
    rc_input: &'static Mutex<CriticalSectionRawMutex, crate::rc::RcInput>,
    actuators: Box<dyn ActuatorInterface>,
}

impl ManualMode {
    /// Create new Manual mode
    ///
    /// # Arguments
    ///
    /// * `rc_input` - RC input state (global RC_INPUT)
    /// * `actuators` - Actuator interface for steering and throttle
    pub fn new(
        rc_input: &'static Mutex<CriticalSectionRawMutex, crate::rc::RcInput>,
        actuators: Box<dyn ActuatorInterface>,
    ) -> Self {
        Self {
            rc_input,
            actuators,
        }
    }
}

impl Mode for ManualMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Entering Manual mode");
        Ok(())
    }

    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        // This function is async because we need to lock the RC input mutex
        // Embassy's async runtime will handle this
        embassy_futures::block_on(self.update_async())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Exiting Manual mode");

        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Manual"
    }
}

impl ManualMode {
    /// Async update implementation
    ///
    /// This is separated from update() to handle the async RC input lock.
    async fn update_async(&mut self) -> Result<(), &'static str> {
        // Lock RC input briefly
        let rc = self.rc_input.lock().await;

        // Check RC timeout
        if rc.is_lost() {
            // Fail-safe: neutral outputs
            crate::log_warn!("RC lost in Manual mode, neutral outputs");
            drop(rc); // Release lock before calling actuators
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        }

        // Read RC channels (1-indexed)
        let steering = rc.get_channel(1); // Channel 1: steering
        let throttle = rc.get_channel(3); // Channel 3: throttle
        drop(rc); // Release lock

        // Direct pass-through to actuators
        // Note: Actuator layer enforces armed check
        self.actuators.set_steering(steering)?;
        self.actuators.set_throttle(throttle)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate alloc;

    use super::*;
    use crate::rc::{RcInput, RcStatus};

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

    fn leak_mutex(rc: RcInput) -> &'static Mutex<CriticalSectionRawMutex, RcInput> {
        alloc::boxed::Box::leak(alloc::boxed::Box::new(Mutex::new(rc)))
    }

    #[test]
    fn test_manual_mode_enter_exit() {
        let rc_mutex = leak_mutex(RcInput::new());
        let mut mode = ManualMode::new(rc_mutex, Box::new(MockActuator::new()));

        assert_eq!(mode.name(), "Manual");
        assert!(mode.enter().is_ok());
        assert!(mode.exit().is_ok());
    }

    #[test]
    fn test_manual_mode_rc_passthrough() {
        let mut rc = RcInput::new();
        rc.channels[0] = 0.5; // Channel 1: steering
        rc.channels[2] = 0.8; // Channel 3: throttle
        rc.status = RcStatus::Active;
        let rc_mutex = leak_mutex(rc);

        let mut mode = ManualMode::new(rc_mutex, Box::new(MockActuator::new()));
        mode.enter().unwrap();
        mode.update(0.02).unwrap();

        assert!(
            (mode.actuators.get_steering() - 0.5).abs() < 0.001,
            "Steering should be 0.5, got {}",
            mode.actuators.get_steering()
        );
        assert!(
            (mode.actuators.get_throttle() - 0.8).abs() < 0.001,
            "Throttle should be 0.8, got {}",
            mode.actuators.get_throttle()
        );
    }

    #[test]
    fn test_manual_mode_rc_lost_failsafe() {
        // RcInput defaults to NeverConnected, which is_lost() returns true for
        let rc_mutex = leak_mutex(RcInput::new());

        let mut mock = MockActuator::new();
        mock.steering = 0.5;
        mock.throttle = 0.8;
        let mut mode = ManualMode::new(rc_mutex, Box::new(mock));
        mode.enter().unwrap();
        mode.update(0.02).unwrap();

        assert!(
            mode.actuators.get_steering().abs() < 0.001,
            "Steering should be neutral on RC lost"
        );
        assert!(
            mode.actuators.get_throttle().abs() < 0.001,
            "Throttle should be neutral on RC lost"
        );
    }

    #[test]
    fn test_manual_mode_exit_neutralizes_actuators() {
        let rc_mutex = leak_mutex(RcInput::new());

        let mut mock = MockActuator::new();
        mock.steering = 1.0;
        mock.throttle = 1.0;
        let mut mode = ManualMode::new(rc_mutex, Box::new(mock));
        mode.exit().unwrap();

        assert!(
            mode.actuators.get_steering().abs() < 0.001,
            "Exit should neutralize steering"
        );
        assert!(
            mode.actuators.get_throttle().abs() < 0.001,
            "Exit should neutralize throttle"
        );
    }
}
