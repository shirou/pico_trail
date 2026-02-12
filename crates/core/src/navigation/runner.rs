//! Navigation Runner
//!
//! Shared navigation step logic used by both firmware and SITL.
//! Checks mode/mission state, computes navigation output, and handles waypoint arrival.
//!
//! ## Usage
//!
//! Both firmware's `navigation_task` and SITL's main loop call `NavigationRunner::step()`
//! each tick. The caller provides GPS/heading via function pointers and handles
//! actuator output and logging.

extern crate alloc;

use crate::autopilot::state::{FlightMode, SYSTEM_STATE};
use crate::mission::{advance_waypoint, complete_mission, get_current_target, get_mission_state};
use crate::navigation::controller::{NavigationController, SimpleNavigationController};
use crate::navigation::{GpsFixType, GpsPosition, NavigationOutput};

/// Navigation runner that encapsulates the shared navigation loop logic.
pub struct NavigationRunner {
    controller: SimpleNavigationController,
}

impl NavigationRunner {
    /// Create a new navigation runner.
    pub fn new() -> Self {
        Self {
            controller: SimpleNavigationController::new(),
        }
    }

    /// Execute one navigation step.
    ///
    /// Checks SYSTEM_STATE mode and MissionState, gets GPS/heading/target,
    /// computes navigation output, and handles waypoint arrival.
    ///
    /// Returns `Some(NavigationOutput)` when actively navigating,
    /// `None` when navigation is not active (wrong mode, mission not running, no GPS, etc).
    ///
    /// The caller is responsible for applying the output to actuators and logging.
    pub fn step(
        &mut self,
        gps_provider: fn() -> Option<GpsPosition>,
        heading_provider: fn() -> Option<f32>,
        dt: f32,
        yaw_rate: Option<f32>,
    ) -> Option<NavigationOutput> {
        // Check mode and mission state
        let mode = critical_section::with(|cs| SYSTEM_STATE.borrow_ref(cs).mode);
        let mission_state = get_mission_state();

        let should_navigate = mission_state == crate::mission::MissionState::Running
            && (mode == FlightMode::Guided || mode == FlightMode::Auto);

        if !should_navigate {
            return None;
        }

        // Get GPS position
        let gps = gps_provider()?;
        if gps.fix_type < GpsFixType::Fix3D {
            return None;
        }

        // Get navigation target from mission storage
        let target = get_current_target()?;

        // Get heading (prefer provider, fallback to GPS COG)
        let heading = heading_provider().or(gps.course_over_ground)?;

        // Compute navigation output
        let output = self
            .controller
            .update(gps.latitude, gps.longitude, &target, heading, dt, yaw_rate);

        // Handle waypoint arrival
        if output.at_target {
            if mode == FlightMode::Guided {
                complete_mission();
                crate::log_info!("Navigation: Guided target reached");
            } else if mode == FlightMode::Auto {
                if advance_waypoint() {
                    crate::log_info!("Navigation: Auto waypoint reached, advancing");
                } else {
                    complete_mission();
                    crate::log_info!("Navigation: Auto mission completed");
                }
            }
            return Some(NavigationOutput {
                steering: 0.0,
                throttle: 0.0,
                ..output
            });
        }

        Some(output)
    }

    /// Get a reference to the internal navigation controller.
    pub fn controller(&self) -> &SimpleNavigationController {
        &self.controller
    }

    /// Reset the navigation controller state.
    pub fn reset(&mut self) {
        use crate::navigation::controller::NavigationController;
        self.controller.reset();
    }
}

impl Default for NavigationRunner {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mission::{set_mission_state, set_single_waypoint, MissionState, Waypoint};
    use crate::navigation::GpsPosition;
    use crate::traits::sync::SharedState;

    fn reset_state() {
        critical_section::with(|cs| {
            let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
            *state = crate::autopilot::state::SystemState::new();
        });
        set_mission_state(MissionState::Idle);
    }

    fn gps_tokyo() -> Option<GpsPosition> {
        Some(GpsPosition {
            latitude: 35.6762,
            longitude: 139.6503,
            altitude: 10.0,
            speed: 1.0,
            course_over_ground: Some(90.0),
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        })
    }

    fn heading_north() -> Option<f32> {
        Some(0.0)
    }

    fn no_gps() -> Option<GpsPosition> {
        None
    }

    #[test]
    #[serial_test::serial]
    fn test_step_returns_none_when_idle() {
        reset_state();
        let mut runner = NavigationRunner::new();
        assert!(runner.step(gps_tokyo, heading_north, 0.02, None).is_none());
    }

    #[test]
    #[serial_test::serial]
    fn test_step_returns_none_when_no_gps() {
        reset_state();
        critical_section::with(|cs| {
            SYSTEM_STATE.borrow_ref_mut(cs).mode = FlightMode::Guided;
        });
        set_mission_state(MissionState::Running);
        let mut runner = NavigationRunner::new();
        assert!(runner.step(no_gps, heading_north, 0.02, None).is_none());
    }

    #[test]
    #[serial_test::serial]
    fn test_step_returns_output_when_navigating() {
        reset_state();
        critical_section::with(|cs| {
            SYSTEM_STATE.borrow_ref_mut(cs).mode = FlightMode::Guided;
        });
        set_mission_state(MissionState::Running);

        // Set a target slightly north of current position
        let waypoint = Waypoint {
            seq: 0,
            frame: 0,
            command: 16,
            current: 1,
            autocontinue: 0,
            param1: 0.0,
            param2: 2.0,
            param3: 0.0,
            param4: 0.0,
            x: 356770_i32, // ~35.6770
            y: 1396503_i32,
            z: 0.0,
        };
        set_single_waypoint(waypoint);

        let mut runner = NavigationRunner::new();
        let output = runner.step(gps_tokyo, heading_north, 0.02, None);
        assert!(output.is_some());
    }
}
