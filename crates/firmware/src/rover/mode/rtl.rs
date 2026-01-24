//! RTL (Return to Launch) Mode
//!
//! Direct navigation to home position for emergency return.
//!
//! # Behavior
//!
//! - On entry: Validate GPS fix and home position
//! - Each update: Navigate directly toward home position
//! - On arrival: Stop within WP_RADIUS of home
//! - On GPS loss: Return error to trigger Hold mode transition
//!
//! # Usage
//!
//! RTL is typically selected when SmartRTL is not available (no recorded path).
//! It provides a direct, straight-line path to home which may not be safe
//! in obstacle-rich environments.
//!
//! # References
//!
//! - FR-lia7r-rtl-navigate-home: RTL navigation requirements
//! - FR-bwqq7-rtl-entry-validation: Entry validation requirements
//! - FR-zlza4-rtl-arrival-stop: Arrival detection requirements
//! - FR-crqdd-rtl-gps-loss-handling: GPS loss handling requirements
//! - ArduPilot Rover RTL: https://ardupilot.org/rover/docs/rtl-mode.html

use super::Mode;
use crate::devices::gps::GpsFixType;
use crate::devices::gps::GpsPosition;
use crate::libraries::ActuatorInterface;
use crate::subsystems::navigation::PositionTarget;
use crate::subsystems::navigation::{NavigationController, SimpleNavigationController};

/// RTL Mode state
#[derive(Clone, Copy, Debug)]
struct RtlState {
    /// Home position target
    target_lat: f32,
    target_lon: f32,
    /// Arrival flag
    arrived: bool,
}

impl Default for RtlState {
    fn default() -> Self {
        Self {
            target_lat: 0.0,
            target_lon: 0.0,
            arrived: false,
        }
    }
}

/// RTL Mode
///
/// Provides direct navigation to home position.
/// This is the fallback mode when SmartRTL is not available.
pub struct RtlMode<'a> {
    /// Actuator interface for steering and throttle
    actuators: &'a mut dyn ActuatorInterface,
    /// Navigation controller for path following
    nav_controller: SimpleNavigationController,
    /// RTL state (set on mode entry)
    state: Option<RtlState>,
    /// GPS position provider function
    gps_provider: fn() -> Option<GpsPosition>,
    /// Home position provider function
    home_provider: fn() -> Option<(f32, f32)>,
    /// Heading provider function (returns heading in degrees, 0-360)
    heading_provider: fn() -> Option<f32>,
}

impl<'a> RtlMode<'a> {
    /// Create new RTL mode
    ///
    /// # Arguments
    ///
    /// * `actuators` - Actuator interface for steering and throttle
    /// * `gps_provider` - Function that returns current GPS position
    /// * `home_provider` - Function that returns home position (lat, lon)
    /// * `heading_provider` - Function that returns current heading (degrees, 0-360)
    pub fn new(
        actuators: &'a mut dyn ActuatorInterface,
        gps_provider: fn() -> Option<GpsPosition>,
        home_provider: fn() -> Option<(f32, f32)>,
        heading_provider: fn() -> Option<f32>,
    ) -> Self {
        Self {
            actuators,
            nav_controller: SimpleNavigationController::new(),
            state: None,
            gps_provider,
            home_provider,
            heading_provider,
        }
    }

    /// Check if RTL mode can be entered
    ///
    /// Validates:
    /// - GPS fix is available and valid (3D fix)
    /// - Home position is set
    ///
    /// # Returns
    ///
    /// Ok(()) if entry is allowed, Err with reason otherwise
    pub fn can_enter(
        gps_provider: fn() -> Option<GpsPosition>,
        home_provider: fn() -> Option<(f32, f32)>,
    ) -> Result<(), &'static str> {
        // Check GPS fix
        let gps = gps_provider().ok_or("RTL requires GPS fix")?;
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("RTL requires 3D GPS fix");
        }

        // Check home position
        home_provider().ok_or("RTL requires home position")?;

        Ok(())
    }

    /// Check if vehicle has arrived at home
    pub fn has_arrived(&self) -> bool {
        self.state.as_ref().map(|s| s.arrived).unwrap_or(false)
    }
}

impl<'a> Mode for RtlMode<'a> {
    fn enter(&mut self) -> Result<(), &'static str> {
        // Validate GPS fix
        let gps = (self.gps_provider)().ok_or("No GPS fix")?;
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("RTL requires 3D GPS fix");
        }

        // Get home position
        let (home_lat, home_lon) = (self.home_provider)().ok_or("Home position not set")?;

        // Initialize state
        self.state = Some(RtlState {
            target_lat: home_lat,
            target_lon: home_lon,
            arrived: false,
        });

        // Reset navigation controller
        self.nav_controller.reset();

        crate::log_info!("RTL: navigating to home ({}, {})", home_lat, home_lon);

        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        let state = self.state.as_mut().ok_or("RTL not initialized")?;

        // Check if already arrived
        if state.arrived {
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        }

        // Get current GPS position
        let gps = (self.gps_provider)().ok_or("GPS lost during RTL")?;

        // Validate GPS fix
        if gps.fix_type < GpsFixType::Fix3D {
            // GPS degraded - return error to trigger Hold mode
            return Err("GPS fix lost during RTL");
        }

        // Get heading from heading provider (fallback to GPS COG)
        let heading = (self.heading_provider)()
            .or(gps.course_over_ground)
            .ok_or("No heading available for RTL")?;

        // Create target
        let target = PositionTarget::new(state.target_lat, state.target_lon);

        // Navigate to target
        let output = self.nav_controller.update(&gps, &target, heading, dt);

        // Check for arrival
        if output.at_target {
            state.arrived = true;
            crate::log_info!("RTL: arrived at home");
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
        } else {
            self.actuators.set_steering(output.steering)?;
            self.actuators.set_throttle(output.throttle)?;
        }

        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Exiting RTL mode");

        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;

        // Clear state
        self.state = None;
        self.nav_controller.reset();

        Ok(())
    }

    fn name(&self) -> &'static str {
        "RTL"
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

    // ========== RtlMode Tests ==========

    #[test]
    fn test_rtl_mode_creation() {
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
        let rtl_mode = RtlMode::new(&mut actuators);

        assert_eq!(rtl_mode.name(), "RTL");
    }

    #[test]
    fn test_rtl_mode_enter_exit() {
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
        let mut rtl_mode = RtlMode::new(&mut actuators);

        // Enter should succeed (host test mode)
        assert!(rtl_mode.enter().is_ok());
        assert!(rtl_mode.exit().is_ok());
    }

    #[test]
    fn test_rtl_mode_update() {
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
        let mut rtl_mode = RtlMode::new(&mut actuators);

        // Enter first
        assert!(rtl_mode.enter().is_ok());

        // Update should succeed (host test mode - no-op)
        assert!(rtl_mode.update(0.02).is_ok());
    }

    #[test]
    fn test_rtl_has_arrived_default() {
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
        let rtl_mode = RtlMode::new(&mut actuators);

        // Should not be arrived before entering
        assert!(!rtl_mode.has_arrived());
    }

    #[test]
    fn test_rtl_state_default() {
        let state = RtlState::default();
        assert_eq!(state.target_lat, 0.0);
        assert_eq!(state.target_lon, 0.0);
        assert!(!state.arrived);
    }
}
