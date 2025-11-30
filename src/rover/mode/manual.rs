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
//!   1. RC timeout detection (1 second) → neutral outputs
//!   2. Armed state check (actuator layer) → neutral when disarmed
//! - No autonomous behavior - pilot in full control
//!
//! ## References
//!
//! - FR-uk0us-manual-mode: Manual mode requirements
//! - ADR-w9zpl-control-mode-architecture: Mode architecture
//! - ArduPilot Manual Mode: https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_manual.cpp

use crate::libraries::ActuatorInterface;

#[cfg(feature = "embassy")]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(feature = "embassy")]
use embassy_sync::mutex::Mutex;

use super::Mode;

/// Manual Mode
///
/// Provides direct RC control with no stabilization.
pub struct ManualMode<'a> {
    #[cfg(feature = "embassy")]
    rc_input: &'static Mutex<CriticalSectionRawMutex, crate::libraries::RcInput>,
    actuators: &'a mut dyn ActuatorInterface,
}

impl<'a> ManualMode<'a> {
    /// Create new Manual mode
    ///
    /// # Arguments
    ///
    /// * `rc_input` - RC input state (global RC_INPUT)
    /// * `actuators` - Actuator interface for steering and throttle
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// let manual_mode = ManualMode::new(&RC_INPUT, actuators);
    /// ```
    #[cfg(feature = "embassy")]
    pub fn new(
        rc_input: &'static Mutex<CriticalSectionRawMutex, crate::libraries::RcInput>,
        actuators: &'a mut dyn ActuatorInterface,
    ) -> Self {
        Self {
            rc_input,
            actuators,
        }
    }

    /// Create new Manual mode (host tests)
    #[cfg(not(feature = "embassy"))]
    pub fn new(actuators: &'a mut dyn ActuatorInterface) -> Self {
        Self { actuators }
    }
}

impl<'a> Mode for ManualMode<'a> {
    fn enter(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Entering Manual mode");
        Ok(())
    }

    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        #[cfg(feature = "embassy")]
        {
            // This function is async because we need to lock the RC input mutex
            // Embassy's async runtime will handle this
            embassy_futures::block_on(self.update_async())
        }

        #[cfg(not(feature = "embassy"))]
        {
            // Host tests: no RC input, just return Ok
            Ok(())
        }
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

#[cfg(feature = "embassy")]
impl<'a> ManualMode<'a> {
    /// Async update implementation (embedded only)
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
    use super::*;
    use crate::communication::mavlink::state::{ArmedState, SystemState};
    use crate::libraries::{ActuatorConfig, ActuatorInterface, Actuators};
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
            if !(0.0..=1.0).contains(&duty_cycle) {
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
    fn test_manual_mode_creation() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let config = ActuatorConfig::default();

        let mut actuators =
            Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);
        let manual_mode = ManualMode::new(&mut actuators);

        assert_eq!(manual_mode.name(), "Manual");
    }

    #[test]
    fn test_manual_mode_enter() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let config = ActuatorConfig::default();

        let mut actuators =
            Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);
        let mut manual_mode = ManualMode::new(&mut actuators);

        let result = manual_mode.enter();
        assert!(result.is_ok());
    }

    #[test]
    fn test_manual_mode_exit() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let config = ActuatorConfig::default();

        let mut actuators =
            Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);
        let mut manual_mode = ManualMode::new(&mut actuators);

        // Enter and then exit
        manual_mode.enter().unwrap();
        let result = manual_mode.exit();

        assert!(result.is_ok());
        // Verify neutral outputs (1500 μs = 7.5% duty)
        assert!((steering_pwm.duty_cycle() - 0.075).abs() < 0.001);
        assert!((throttle_pwm.duty_cycle() - 0.075).abs() < 0.001);
    }

    #[test]
    fn test_manual_mode_disarmed_safety() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let system_state = SystemState::new(); // Disarmed by default
        let config = ActuatorConfig::default();

        let mut actuators =
            Actuators::new(&mut steering_pwm, &mut throttle_pwm, &system_state, config);

        // Command actuators while disarmed
        actuators.set_steering(1.0).unwrap();
        actuators.set_throttle(1.0).unwrap();

        // Verify neutral outputs (1500 μs = 7.5% duty)
        // This tests the actuator layer safety, which Manual mode relies on
        assert!((steering_pwm.duty_cycle() - 0.075).abs() < 0.001);
        assert!((throttle_pwm.duty_cycle() - 0.075).abs() < 0.001);
    }
}
