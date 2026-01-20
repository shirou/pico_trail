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
use crate::core::mission::{
    complete_mission, get_current_target, get_mission_state, set_mission_state, MissionState,
};
use crate::devices::gps::GpsFixType;
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
    gps_provider: fn() -> Option<GpsPosition>,
    /// Heading provider function (returns heading in degrees, 0-360)
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

    /// Check if Guided mode can be entered
    ///
    /// Validates:
    /// - GPS fix is available and valid (3D fix)
    ///
    /// # Returns
    ///
    /// Ok(()) if entry is allowed, Err with reason otherwise
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
        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
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

        // Stop mission if it was running
        let mission_state = get_mission_state();
        if mission_state == MissionState::Running {
            set_mission_state(MissionState::Idle);
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

    #[test]
    fn test_guided_state_default() {
        let state = GuidedState::default();
        assert!(!state.navigation_active);
        assert!(!state.at_target);
    }
}
