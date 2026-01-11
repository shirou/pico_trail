//! Auto Mode
//!
//! Mission-based autonomous navigation mode that executes waypoint missions.
//!
//! # Behavior
//!
//! - On entry: Validate GPS fix and mission loaded
//! - Each update: Navigate to current waypoint, advance on arrival
//! - On mission complete: Set mission state to Completed and stop
//! - On GPS loss: Return error to trigger Hold mode transition
//!
//! # Waypoint Progression
//!
//! Auto mode uses the mission storage to navigate through waypoints sequentially.
//! When a waypoint is reached:
//! 1. Check autocontinue flag
//! 2. If autocontinue=1: Advance to next waypoint
//! 3. If at last waypoint or autocontinue=0: Complete mission
//!
//! # References
//!
//! - FR-jm7mj-auto-mode-mission-execution: Auto mode requirements
//! - ADR-h3k9f-heading-source-integration: Heading source integration
//! - ArduPilot Auto Mode: https://ardupilot.org/rover/docs/auto-mode.html

use super::Mode;
#[cfg(feature = "embassy")]
use crate::core::mission::{
    advance_waypoint, complete_mission, get_current_target, get_mission_state, has_waypoints,
    start_mission_from_beginning, MissionState,
};
#[cfg(feature = "embassy")]
use crate::devices::gps::GpsFixType;
#[cfg(feature = "embassy")]
use crate::devices::gps::GpsPosition;
use crate::libraries::ActuatorInterface;
use crate::subsystems::navigation::{NavigationController, SimpleNavigationController};

/// Auto Mode state
#[derive(Clone, Copy, Debug, Default)]
#[allow(dead_code)]
struct AutoState {
    /// Flag indicating if navigation is active
    navigation_active: bool,
    /// Flag indicating if mission is complete
    mission_complete: bool,
    /// Current waypoint index (for logging)
    current_wp_index: u16,
}

/// Auto Mode
///
/// Provides mission-based autonomous navigation through uploaded waypoints.
/// Waypoints are uploaded via MAVLink mission protocol before entering Auto mode.
pub struct AutoMode<'a> {
    /// Actuator interface for steering and throttle
    actuators: &'a mut dyn ActuatorInterface,
    /// Navigation controller for path following
    nav_controller: SimpleNavigationController,
    /// Auto state
    state: Option<AutoState>,
    /// GPS position provider function
    #[cfg(feature = "embassy")]
    gps_provider: fn() -> Option<GpsPosition>,
    /// Heading provider function (returns heading in degrees, 0-360)
    #[cfg(feature = "embassy")]
    heading_provider: fn() -> Option<f32>,
}

#[allow(dead_code)]
impl<'a> AutoMode<'a> {
    /// Create new Auto mode
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

    /// Create new Auto mode (host tests)
    #[cfg(not(feature = "embassy"))]
    pub fn new(actuators: &'a mut dyn ActuatorInterface) -> Self {
        Self {
            actuators,
            nav_controller: SimpleNavigationController::new(),
            state: None,
        }
    }

    /// Check if Auto mode can be entered
    ///
    /// Validates:
    /// - GPS fix is available and valid (3D fix)
    /// - Mission is loaded with waypoints
    ///
    /// # Returns
    ///
    /// Ok(()) if entry is allowed, Err with reason otherwise
    #[cfg(feature = "embassy")]
    pub fn can_enter(gps_provider: fn() -> Option<GpsPosition>) -> Result<(), &'static str> {
        // Check GPS fix
        let gps = gps_provider().ok_or("Auto requires GPS fix")?;
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("Auto requires 3D GPS fix");
        }

        // Check mission loaded
        if !has_waypoints() {
            return Err("Auto requires mission loaded");
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

    /// Check if mission is complete
    pub fn is_mission_complete(&self) -> bool {
        self.state
            .as_ref()
            .map(|s| s.mission_complete)
            .unwrap_or(false)
    }

    /// Get current waypoint index
    pub fn current_waypoint_index(&self) -> u16 {
        self.state.as_ref().map(|s| s.current_wp_index).unwrap_or(0)
    }
}

impl<'a> Mode for AutoMode<'a> {
    fn enter(&mut self) -> Result<(), &'static str> {
        #[cfg(feature = "embassy")]
        {
            // Validate GPS fix
            let gps = (self.gps_provider)().ok_or("No GPS fix")?;
            if gps.fix_type < GpsFixType::Fix3D {
                return Err("Auto requires 3D GPS fix");
            }

            // Validate mission loaded
            if !has_waypoints() {
                return Err("Auto requires mission loaded");
            }

            // Start mission from beginning
            if !start_mission_from_beginning() {
                return Err("Failed to start mission");
            }

            // Initialize state
            self.state = Some(AutoState {
                navigation_active: false,
                mission_complete: false,
                current_wp_index: 0,
            });

            // Reset navigation controller
            self.nav_controller.reset();

            crate::log_info!("Auto mode entered");
        }

        #[cfg(not(feature = "embassy"))]
        {
            crate::log_info!("Auto mode entered (host test)");
        }

        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        #[cfg(feature = "embassy")]
        {
            let state = self.state.as_mut().ok_or("Auto not initialized")?;

            // Check mission state
            let mission_state = get_mission_state();
            if mission_state == MissionState::Completed {
                // Mission already completed - stop
                state.navigation_active = false;
                state.mission_complete = true;
                self.actuators.set_steering(0.0)?;
                self.actuators.set_throttle(0.0)?;
                return Ok(());
            }

            if mission_state != MissionState::Running {
                // Mission not running - stop
                state.navigation_active = false;
                self.actuators.set_steering(0.0)?;
                self.actuators.set_throttle(0.0)?;
                return Ok(());
            }

            // Get current GPS position
            let gps = (self.gps_provider)().ok_or("GPS lost during Auto")?;

            // Validate GPS fix
            if gps.fix_type < GpsFixType::Fix3D {
                return Err("GPS fix lost during Auto");
            }

            // Get target from mission storage
            let target = match get_current_target() {
                Some(t) => t,
                None => {
                    // No more waypoints - mission complete
                    state.navigation_active = false;
                    state.mission_complete = true;
                    complete_mission();
                    self.actuators.set_steering(0.0)?;
                    self.actuators.set_throttle(0.0)?;
                    crate::log_info!("Auto: mission complete (no more waypoints)");
                    return Ok(());
                }
            };

            // Get heading from heading provider (fallback to GPS COG)
            let heading = (self.heading_provider)()
                .or(gps.course_over_ground)
                .ok_or("No heading available for Auto")?;

            // Navigation is active
            state.navigation_active = true;

            // Navigate to target
            let output = self.nav_controller.update(&gps, &target, heading, dt);

            // Check for waypoint arrival
            if output.at_target {
                crate::log_info!("Auto: waypoint {} reached", state.current_wp_index);

                // Try to advance to next waypoint
                if advance_waypoint() {
                    // Advanced to next waypoint
                    state.current_wp_index += 1;
                    self.nav_controller.reset();
                    crate::log_info!("Auto: advancing to waypoint {}", state.current_wp_index);
                } else {
                    // No more waypoints or autocontinue disabled - mission complete
                    state.mission_complete = true;
                    complete_mission();
                    self.actuators.set_steering(0.0)?;
                    self.actuators.set_throttle(0.0)?;
                    crate::log_info!("Auto: mission complete");
                    return Ok(());
                }
            }

            // Apply navigation output
            self.actuators.set_steering(output.steering)?;
            self.actuators.set_throttle(output.throttle)?;
        }

        #[cfg(not(feature = "embassy"))]
        {
            let _ = dt;
            // Host tests: no-op
        }

        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Exiting Auto mode");

        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;

        // Clear state (preserve mission state for potential resume)
        self.state = None;
        self.nav_controller.reset();

        #[cfg(feature = "embassy")]
        {
            // Note: We do NOT reset mission state on exit
            // This allows resuming the mission later
            // User can clear mission with MISSION_CLEAR_ALL
        }

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Auto"
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

    // ========== AutoMode Tests ==========

    #[test]
    fn test_auto_mode_creation() {
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
        let auto_mode = AutoMode::new(&mut actuators);

        assert_eq!(auto_mode.name(), "Auto");
    }

    #[test]
    fn test_auto_mode_enter_exit() {
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
        let mut auto_mode = AutoMode::new(&mut actuators);

        // Enter should succeed (host test mode)
        assert!(auto_mode.enter().is_ok());
        assert!(auto_mode.exit().is_ok());
    }

    #[test]
    fn test_auto_mode_update() {
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
        let mut auto_mode = AutoMode::new(&mut actuators);

        // Enter first
        assert!(auto_mode.enter().is_ok());

        // Update should succeed (host test mode - no-op)
        assert!(auto_mode.update(0.02).is_ok());
    }

    #[test]
    fn test_auto_navigation_state_default() {
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
        let auto_mode = AutoMode::new(&mut actuators);

        // Before entering, navigation should not be active
        assert!(!auto_mode.is_navigation_active());
        assert!(!auto_mode.is_mission_complete());
        assert_eq!(auto_mode.current_waypoint_index(), 0);
    }

    #[test]
    fn test_auto_state_default() {
        let state = AutoState::default();
        assert!(!state.navigation_active);
        assert!(!state.mission_complete);
        assert_eq!(state.current_wp_index, 0);
    }

    // ========== State Transition Tests ==========

    #[test]
    fn test_auto_mode_lifecycle() {
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
        let mut auto_mode = AutoMode::new(&mut actuators);

        // Initial state: not active
        assert!(!auto_mode.is_navigation_active());
        assert!(!auto_mode.is_mission_complete());
        assert_eq!(auto_mode.current_waypoint_index(), 0);

        // Enter mode
        assert!(auto_mode.enter().is_ok());

        // Update multiple times (host test - no-op but should not error)
        for _ in 0..10 {
            assert!(auto_mode.update(0.02).is_ok());
        }

        // Exit mode
        assert!(auto_mode.exit().is_ok());

        // After exit: state should be cleared
        assert!(!auto_mode.is_navigation_active());
        assert!(!auto_mode.is_mission_complete());
        assert_eq!(auto_mode.current_waypoint_index(), 0);
    }

    #[test]
    fn test_auto_mode_double_enter() {
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
        let mut auto_mode = AutoMode::new(&mut actuators);

        // Enter twice should both succeed (host test mode)
        assert!(auto_mode.enter().is_ok());
        assert!(auto_mode.enter().is_ok());

        assert!(auto_mode.exit().is_ok());
    }

    #[test]
    fn test_auto_mode_double_exit() {
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
        let mut auto_mode = AutoMode::new(&mut actuators);

        assert!(auto_mode.enter().is_ok());

        // Exit twice should both succeed
        assert!(auto_mode.exit().is_ok());
        assert!(auto_mode.exit().is_ok());
    }

    #[test]
    fn test_auto_mode_update_without_enter() {
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
        let mut auto_mode = AutoMode::new(&mut actuators);

        // Update without enter (host test mode - should succeed as no-op)
        assert!(auto_mode.update(0.02).is_ok());
    }

    #[test]
    fn test_auto_mode_name_constant() {
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
        let auto_mode = AutoMode::new(&mut actuators);

        // Name should always be "Auto"
        assert_eq!(auto_mode.name(), "Auto");
    }

    #[test]
    fn test_auto_mode_multiple_update_cycles() {
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
        let mut auto_mode = AutoMode::new(&mut actuators);

        assert!(auto_mode.enter().is_ok());

        // Simulate 50Hz updates for 1 second (50 updates)
        for i in 0..50 {
            let result = auto_mode.update(0.02);
            assert!(result.is_ok(), "Update {} failed", i);
        }

        assert!(auto_mode.exit().is_ok());
    }

    #[test]
    fn test_auto_state_clone() {
        let state = AutoState {
            navigation_active: true,
            mission_complete: false,
            current_wp_index: 5,
        };
        let cloned = state;

        assert_eq!(state.navigation_active, cloned.navigation_active);
        assert_eq!(state.mission_complete, cloned.mission_complete);
        assert_eq!(state.current_wp_index, cloned.current_wp_index);
    }

    #[test]
    fn test_auto_state_debug() {
        let state = AutoState {
            navigation_active: true,
            mission_complete: true,
            current_wp_index: 10,
        };

        let debug_str = format!("{:?}", state);
        assert!(debug_str.contains("AutoState"));
        assert!(debug_str.contains("navigation_active: true"));
        assert!(debug_str.contains("mission_complete: true"));
        assert!(debug_str.contains("current_wp_index: 10"));
    }

    // ========== Waypoint Tracking Tests ==========

    #[test]
    fn test_auto_mode_initial_waypoint_index() {
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
        let auto_mode = AutoMode::new(&mut actuators);

        // Initial waypoint index should be 0
        assert_eq!(auto_mode.current_waypoint_index(), 0);
    }

    #[test]
    fn test_auto_mode_mission_state_methods() {
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
        let auto_mode = AutoMode::new(&mut actuators);

        // All state methods should work before enter
        assert!(!auto_mode.is_navigation_active());
        assert!(!auto_mode.is_mission_complete());
        assert_eq!(auto_mode.current_waypoint_index(), 0);
    }
}
