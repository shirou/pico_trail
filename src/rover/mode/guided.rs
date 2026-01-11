//! Guided Mode
//!
//! Single-waypoint navigation mode that navigates to targets set via GCS commands.
//!
//! # Behavior
//!
//! - On entry: Validate GPS fix and heading source
//! - Each update: Navigate to target waypoint from MISSION_STORAGE
//! - On arrival: Stop at target and set mission complete
//! - On GPS loss: Return error to trigger Hold mode transition
//!
//! # Target Sources
//!
//! Targets are set via:
//! - SET_POSITION_TARGET_GLOBAL_INT MAVLink message
//! - MAV_CMD_DO_REPOSITION command
//!
//! These handlers update MISSION_STORAGE with a single waypoint.
//!
//! # References
//!
//! - FR-jpmdj-unified-waypoint-navigation: Waypoint navigation requirements
//! - ADR-2hs12-unified-waypoint-navigation: Architecture decision
//! - ArduPilot Guided Mode: https://ardupilot.org/rover/docs/guided-mode.html

use super::Mode;
#[cfg(feature = "embassy")]
use crate::core::mission::{
    complete_mission, get_current_target, get_mission_state, set_mission_state, MissionState,
};
#[cfg(feature = "embassy")]
use crate::devices::gps::GpsFixType;
#[cfg(feature = "embassy")]
use crate::devices::gps::GpsPosition;
use crate::libraries::ActuatorInterface;
use crate::subsystems::navigation::{NavigationController, SimpleNavigationController};

/// Guided Mode state
#[derive(Clone, Copy, Debug, Default)]
#[allow(dead_code)]
struct GuidedState {
    /// Flag indicating if navigation is active
    navigation_active: bool,
    /// Flag indicating if target has been reached
    at_target: bool,
}

/// Guided Mode
///
/// Provides single-waypoint navigation for GCS-controlled movement.
/// Navigates to targets set via SET_POSITION_TARGET_GLOBAL_INT or DO_REPOSITION.
pub struct GuidedMode<'a> {
    /// Actuator interface for steering and throttle
    actuators: &'a mut dyn ActuatorInterface,
    /// Navigation controller for path following
    nav_controller: SimpleNavigationController,
    /// Guided state
    state: Option<GuidedState>,
    /// GPS position provider function
    #[cfg(feature = "embassy")]
    gps_provider: fn() -> Option<GpsPosition>,
    /// Heading provider function (returns heading in degrees, 0-360)
    #[cfg(feature = "embassy")]
    heading_provider: fn() -> Option<f32>,
}

#[allow(dead_code)]
impl<'a> GuidedMode<'a> {
    /// Create new Guided mode
    ///
    /// # Arguments
    ///
    /// * `actuators` - Actuator interface for steering and throttle
    /// * `gps_provider` - Function that returns current GPS position
    /// * `heading_provider` - Function that returns current heading (degrees, 0-360)
    #[cfg(feature = "embassy")]
    pub fn new(
        actuators: &'a mut dyn ActuatorInterface,
        gps_provider: fn() -> Option<GpsPosition>,
        heading_provider: fn() -> Option<f32>,
    ) -> Self {
        Self {
            actuators,
            nav_controller: SimpleNavigationController::new(),
            state: None,
            gps_provider,
            heading_provider,
        }
    }

    /// Create new Guided mode (host tests)
    #[cfg(not(feature = "embassy"))]
    pub fn new(actuators: &'a mut dyn ActuatorInterface) -> Self {
        Self {
            actuators,
            nav_controller: SimpleNavigationController::new(),
            state: None,
        }
    }

    /// Check if Guided mode can be entered
    ///
    /// Validates:
    /// - GPS fix is available and valid (3D fix)
    ///
    /// # Returns
    ///
    /// Ok(()) if entry is allowed, Err with reason otherwise
    #[cfg(feature = "embassy")]
    pub fn can_enter(gps_provider: fn() -> Option<GpsPosition>) -> Result<(), &'static str> {
        // Check GPS fix
        let gps = gps_provider().ok_or("Guided requires GPS fix")?;
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("Guided requires 3D GPS fix");
        }

        Ok(())
    }

    /// Check if navigation is active
    pub fn is_navigation_active(&self) -> bool {
        self.state
            .as_ref()
            .map(|s| s.navigation_active)
            .unwrap_or(false)
    }

    /// Check if target has been reached
    pub fn is_at_target(&self) -> bool {
        self.state.as_ref().map(|s| s.at_target).unwrap_or(false)
    }
}

impl<'a> Mode for GuidedMode<'a> {
    fn enter(&mut self) -> Result<(), &'static str> {
        #[cfg(feature = "embassy")]
        {
            // Validate GPS fix
            let gps = (self.gps_provider)().ok_or("No GPS fix")?;
            if gps.fix_type < GpsFixType::Fix3D {
                return Err("Guided requires 3D GPS fix");
            }

            // Initialize state
            self.state = Some(GuidedState {
                navigation_active: false,
                at_target: false,
            });

            // Reset navigation controller
            self.nav_controller.reset();

            crate::log_info!("Guided mode entered");
        }

        #[cfg(not(feature = "embassy"))]
        {
            crate::log_info!("Guided mode entered (host test)");
        }

        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        #[cfg(feature = "embassy")]
        {
            let state = self.state.as_mut().ok_or("Guided not initialized")?;

            // Check mission state - only navigate if mission is running
            let mission_state = get_mission_state();
            if mission_state != MissionState::Running {
                // No active mission - stop
                state.navigation_active = false;
                self.actuators.set_steering(0.0)?;
                self.actuators.set_throttle(0.0)?;
                return Ok(());
            }

            // Get current GPS position
            let gps = (self.gps_provider)().ok_or("GPS lost during Guided")?;

            // Validate GPS fix
            if gps.fix_type < GpsFixType::Fix3D {
                return Err("GPS fix lost during Guided");
            }

            // Get target from mission storage
            let target = match get_current_target() {
                Some(t) => t,
                None => {
                    // No target - stop
                    state.navigation_active = false;
                    self.actuators.set_steering(0.0)?;
                    self.actuators.set_throttle(0.0)?;
                    return Ok(());
                }
            };

            // Get heading from heading provider (fallback to GPS COG)
            let heading = (self.heading_provider)()
                .or(gps.course_over_ground)
                .ok_or("No heading available for Guided")?;

            // Navigation is active
            state.navigation_active = true;

            // Navigate to target
            let output = self.nav_controller.update(&gps, &target, heading, dt);

            // Check for arrival
            if output.at_target {
                state.at_target = true;
                complete_mission();
                crate::log_info!("Guided: target reached");
                self.actuators.set_steering(0.0)?;
                self.actuators.set_throttle(0.0)?;
            } else {
                state.at_target = false;
                self.actuators.set_steering(output.steering)?;
                self.actuators.set_throttle(output.throttle)?;
            }
        }

        #[cfg(not(feature = "embassy"))]
        {
            let _ = dt;
            // Host tests: no-op
        }

        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Exiting Guided mode");

        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;

        // Clear state
        self.state = None;
        self.nav_controller.reset();

        #[cfg(feature = "embassy")]
        {
            // Stop mission if it was running
            let mission_state = get_mission_state();
            if mission_state == MissionState::Running {
                set_mission_state(MissionState::Idle);
            }
        }

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Guided"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::state::{ArmedState, SystemState};
    use crate::libraries::{ActuatorConfig, Actuators};
    use crate::platform::traits::pwm::PwmInterface;
    use crate::platform::Result;

    // Mock PWM for testing
    struct MockPwm {
        duty_cycle: f32,
    }

    impl MockPwm {
        fn new() -> Self {
            Self { duty_cycle: 0.0 }
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

        fn set_frequency(&mut self, _frequency: u32) -> Result<()> {
            Ok(())
        }

        fn frequency(&self) -> u32 {
            50
        }

        fn enable(&mut self) {}
        fn disable(&mut self) {}

        fn is_enabled(&self) -> bool {
            true
        }
    }

    // ========== GuidedMode Tests ==========

    #[test]
    fn test_guided_mode_creation() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );
        let guided_mode = GuidedMode::new(&mut actuators);

        assert_eq!(guided_mode.name(), "Guided");
    }

    #[test]
    fn test_guided_mode_enter_exit() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );
        let mut guided_mode = GuidedMode::new(&mut actuators);

        // Enter should succeed (host test mode)
        assert!(guided_mode.enter().is_ok());
        assert!(guided_mode.exit().is_ok());
    }

    #[test]
    fn test_guided_mode_update() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );
        let mut guided_mode = GuidedMode::new(&mut actuators);

        // Enter first
        assert!(guided_mode.enter().is_ok());

        // Update should succeed (host test mode - no-op)
        assert!(guided_mode.update(0.02).is_ok());
    }

    #[test]
    fn test_guided_navigation_state_default() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );
        let guided_mode = GuidedMode::new(&mut actuators);

        // Before entering, navigation should not be active
        assert!(!guided_mode.is_navigation_active());
        assert!(!guided_mode.is_at_target());
    }

    #[test]
    fn test_guided_state_default() {
        let state = GuidedState::default();
        assert!(!state.navigation_active);
        assert!(!state.at_target);
    }

    // ========== State Transition Tests ==========

    #[test]
    fn test_guided_mode_lifecycle() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );
        let mut guided_mode = GuidedMode::new(&mut actuators);

        // Initial state: not active
        assert!(!guided_mode.is_navigation_active());
        assert!(!guided_mode.is_at_target());

        // Enter mode
        assert!(guided_mode.enter().is_ok());

        // Update multiple times (host test - no-op but should not error)
        for _ in 0..10 {
            assert!(guided_mode.update(0.02).is_ok());
        }

        // Exit mode
        assert!(guided_mode.exit().is_ok());

        // After exit: state should be cleared
        assert!(!guided_mode.is_navigation_active());
        assert!(!guided_mode.is_at_target());
    }

    #[test]
    fn test_guided_mode_double_enter() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );
        let mut guided_mode = GuidedMode::new(&mut actuators);

        // Enter twice should both succeed (host test mode)
        assert!(guided_mode.enter().is_ok());
        assert!(guided_mode.enter().is_ok());

        assert!(guided_mode.exit().is_ok());
    }

    #[test]
    fn test_guided_mode_double_exit() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );
        let mut guided_mode = GuidedMode::new(&mut actuators);

        assert!(guided_mode.enter().is_ok());

        // Exit twice should both succeed
        assert!(guided_mode.exit().is_ok());
        assert!(guided_mode.exit().is_ok());
    }

    #[test]
    fn test_guided_mode_update_without_enter() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );
        let mut guided_mode = GuidedMode::new(&mut actuators);

        // Update without enter (host test mode - should succeed as no-op)
        assert!(guided_mode.update(0.02).is_ok());
    }

    #[test]
    fn test_guided_mode_name_constant() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );
        let guided_mode = GuidedMode::new(&mut actuators);

        // Name should always be "Guided"
        assert_eq!(guided_mode.name(), "Guided");
    }

    #[test]
    fn test_guided_mode_multiple_update_cycles() {
        let mut steering_pwm = MockPwm::new();
        let mut throttle_pwm = MockPwm::new();
        let mut system_state = SystemState::new();
        system_state.armed = ArmedState::Armed;
        let actuator_config = ActuatorConfig::default();

        let mut actuators = Actuators::new(
            &mut steering_pwm,
            &mut throttle_pwm,
            &system_state,
            actuator_config,
        );
        let mut guided_mode = GuidedMode::new(&mut actuators);

        assert!(guided_mode.enter().is_ok());

        // Simulate 50Hz updates for 1 second (50 updates)
        for i in 0..50 {
            let result = guided_mode.update(0.02);
            assert!(result.is_ok(), "Update {} failed", i);
        }

        assert!(guided_mode.exit().is_ok());
    }
}
