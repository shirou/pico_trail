//! SmartRTL (Smart Return to Launch) Mode
//!
//! Safe return to home by retracing the recorded flight path in reverse.
//!
//! # Behavior
//!
//! - On entry: Load waypoints from PathRecorder in reverse order
//! - Each update: Navigate to current waypoint, advance on arrival
//! - On completion: Stop at home position
//! - On GPS loss: Return error to trigger Hold mode transition
//!
//! # Advantages over Direct RTL
//!
//! - Follows a known-safe path (the path already traveled)
//! - Avoids obstacles that may be between current position and home
//! - Suitable for complex environments
//!
//! # Fallback
//!
//! If no recorded path is available, SmartRTL entry fails and the system
//! should fall back to direct RTL mode.
//!
//! # References
//!
//! - FR-nywcm-smartrtl-path-recording: Path recording requirements
//! - FR-8dug4-smartrtl-return-navigation: Return navigation requirements
//! - FR-hibx1-smartrtl-rtl-fallback: RTL fallback requirements
//! - ArduPilot SmartRTL: https://ardupilot.org/rover/docs/smartrtl-mode.html

use super::Mode;
use crate::devices::gps::GpsFixType;
use crate::devices::gps::GpsPosition;
use crate::libraries::ActuatorInterface;
use crate::subsystems::navigation::PositionTarget;
use crate::subsystems::navigation::{NavigationController, PathPoint, SimpleNavigationController};

/// Maximum waypoints to load from path recorder
const MAX_WAYPOINTS: usize = 300;

/// SmartRTL Mode state
#[derive(Clone, Copy, Debug, Default)]
struct SmartRtlState {
    /// Current waypoint index
    waypoint_idx: usize,
    /// Total number of waypoints
    total_waypoints: usize,
    /// Completion flag
    completed: bool,
}

/// SmartRTL Mode
///
/// Provides safe return to home by retracing the recorded path.
pub struct SmartRtlMode<'a> {
    /// Actuator interface for steering and throttle
    actuators: &'a mut dyn ActuatorInterface,
    /// Navigation controller for path following
    nav_controller: SimpleNavigationController,
    /// SmartRTL state
    state: Option<SmartRtlState>,
    /// Waypoints loaded from path recorder (reverse order)
    waypoints: [PathPoint; MAX_WAYPOINTS],
    /// Number of waypoints loaded
    waypoint_count: usize,
    /// GPS position provider function
    gps_provider: fn() -> Option<GpsPosition>,
    /// Path recorder provider function (fills buffer, returns count)
    path_provider: fn(&mut [PathPoint]) -> usize,
    /// Home position provider function
    home_provider: fn() -> Option<(f32, f32)>,
    /// Heading provider function (returns heading in degrees, 0-360)
    heading_provider: fn() -> Option<f32>,
}

impl<'a> SmartRtlMode<'a> {
    /// Create new SmartRTL mode
    ///
    /// # Arguments
    ///
    /// * `actuators` - Actuator interface for steering and throttle
    /// * `gps_provider` - Function that returns current GPS position
    /// * `path_provider` - Function that fills buffer with path points and returns count
    /// * `home_provider` - Function that returns home position (lat, lon)
    /// * `heading_provider` - Function that returns current heading (degrees, 0-360)
    pub fn new(
        actuators: &'a mut dyn ActuatorInterface,
        gps_provider: fn() -> Option<GpsPosition>,
        path_provider: fn(&mut [PathPoint]) -> usize,
        home_provider: fn() -> Option<(f32, f32)>,
        heading_provider: fn() -> Option<f32>,
    ) -> Self {
        Self {
            actuators,
            nav_controller: SimpleNavigationController::new(),
            state: None,
            waypoints: [PathPoint::default(); MAX_WAYPOINTS],
            waypoint_count: 0,
            gps_provider,
            path_provider,
            home_provider,
            heading_provider,
        }
    }

    /// Check if SmartRTL mode can be entered
    ///
    /// Validates:
    /// - GPS fix is available and valid (3D fix)
    /// - Home position is set
    /// - Recorded path is available (has at least one point)
    ///
    /// # Returns
    ///
    /// Ok(()) if entry is allowed, Err with reason otherwise
    pub fn can_enter(
        gps_provider: fn() -> Option<GpsPosition>,
        path_count_provider: fn() -> usize,
        home_provider: fn() -> Option<(f32, f32)>,
    ) -> Result<(), &'static str> {
        // Check GPS fix
        let gps = gps_provider().ok_or("SmartRTL requires GPS fix")?;
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("SmartRTL requires 3D GPS fix");
        }

        // Check home position
        home_provider().ok_or("SmartRTL requires home position")?;

        // Check path availability
        if path_count_provider() == 0 {
            return Err("SmartRTL requires recorded path");
        }

        Ok(())
    }

    /// Check if navigation is complete
    pub fn is_completed(&self) -> bool {
        self.state.as_ref().map(|s| s.completed).unwrap_or(false)
    }

    /// Get current waypoint index
    pub fn current_waypoint(&self) -> usize {
        self.state.as_ref().map(|s| s.waypoint_idx).unwrap_or(0)
    }

    /// Get total waypoint count
    pub fn total_waypoints(&self) -> usize {
        self.state.as_ref().map(|s| s.total_waypoints).unwrap_or(0)
    }

    /// Load waypoints from path provider
    fn load_waypoints(&mut self) -> usize {
        let count = (self.path_provider)(&mut self.waypoints);
        self.waypoint_count = count;
        count
    }
}

impl<'a> Mode for SmartRtlMode<'a> {
    fn enter(&mut self) -> Result<(), &'static str> {
        // Validate GPS fix
        let gps = (self.gps_provider)().ok_or("No GPS fix")?;
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("SmartRTL requires 3D GPS fix");
        }

        // Validate home position
        (self.home_provider)().ok_or("Home position not set")?;

        // Load waypoints from path recorder
        let waypoint_count = self.load_waypoints();
        if waypoint_count == 0 {
            return Err("No recorded path available");
        }

        // Initialize state
        self.state = Some(SmartRtlState {
            waypoint_idx: 0,
            total_waypoints: waypoint_count,
            completed: false,
        });

        // Reset navigation controller
        self.nav_controller.reset();

        crate::log_info!("SmartRTL: navigating {} waypoints", waypoint_count);

        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        let state = self.state.as_mut().ok_or("SmartRTL not initialized")?;

        // Check if already completed
        if state.completed {
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        }

        // Get current GPS position
        let gps = (self.gps_provider)().ok_or("GPS lost during SmartRTL")?;

        // Validate GPS fix
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("GPS fix lost during SmartRTL");
        }

        // Get heading from heading provider (fallback to GPS COG)
        let heading = (self.heading_provider)()
            .or(gps.course_over_ground)
            .ok_or("No heading available for SmartRTL")?;

        // Get current waypoint
        let waypoint = &self.waypoints[state.waypoint_idx];
        let target = PositionTarget::new(waypoint.latitude, waypoint.longitude);

        // Navigate to waypoint
        let output = self
            .nav_controller
            .update(gps.latitude, gps.longitude, &target, heading, dt);

        // Check for waypoint arrival
        if output.at_target {
            state.waypoint_idx += 1;

            // Check if all waypoints visited
            if state.waypoint_idx >= state.total_waypoints {
                state.completed = true;
                crate::log_info!("SmartRTL: arrived at home");
                self.actuators.set_steering(0.0)?;
                self.actuators.set_throttle(0.0)?;
            } else {
                crate::log_debug!(
                    "SmartRTL: waypoint {}/{}",
                    state.waypoint_idx,
                    state.total_waypoints
                );
                self.nav_controller.reset();
            }
        } else {
            self.actuators.set_steering(output.steering)?;
            self.actuators.set_throttle(output.throttle)?;
        }

        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Exiting SmartRTL mode");

        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;

        // Clear state
        self.state = None;
        self.waypoint_count = 0;
        self.nav_controller.reset();

        Ok(())
    }

    fn name(&self) -> &'static str {
        "SmartRTL"
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

    // ========== SmartRtlMode Tests ==========

    #[test]
    fn test_smartrtl_mode_creation() {
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
        let smartrtl_mode = SmartRtlMode::new(&mut actuators);

        assert_eq!(smartrtl_mode.name(), "SmartRTL");
    }

    #[test]
    fn test_smartrtl_mode_enter_exit() {
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
        let mut smartrtl_mode = SmartRtlMode::new(&mut actuators);

        // Enter should succeed (host test mode)
        assert!(smartrtl_mode.enter().is_ok());
        assert!(smartrtl_mode.exit().is_ok());
    }

    #[test]
    fn test_smartrtl_mode_update() {
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
        let mut smartrtl_mode = SmartRtlMode::new(&mut actuators);

        // Enter first
        assert!(smartrtl_mode.enter().is_ok());

        // Update should succeed (host test mode - no-op)
        assert!(smartrtl_mode.update(0.02).is_ok());
    }

    #[test]
    fn test_smartrtl_is_completed_default() {
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
        let smartrtl_mode = SmartRtlMode::new(&mut actuators);

        // Should not be completed before entering
        assert!(!smartrtl_mode.is_completed());
    }

    #[test]
    fn test_smartrtl_waypoint_tracking() {
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
        let smartrtl_mode = SmartRtlMode::new(&mut actuators);

        // Before entering, should have 0 waypoints
        assert_eq!(smartrtl_mode.current_waypoint(), 0);
        assert_eq!(smartrtl_mode.total_waypoints(), 0);
    }

    #[test]
    fn test_smartrtl_state_default() {
        let state = SmartRtlState::default();
        assert_eq!(state.waypoint_idx, 0);
        assert_eq!(state.total_waypoints, 0);
        assert!(!state.completed);
    }
}
