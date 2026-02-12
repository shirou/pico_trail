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

extern crate alloc;
use alloc::boxed::Box;

use super::Mode;
use crate::navigation::controller::{NavigationController, SimpleNavigationController};
use crate::navigation::PositionTarget;
use crate::navigation::{GpsFixType, GpsPosition};
use crate::servo::ActuatorInterface;

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
pub struct RtlMode {
    /// Actuator interface for steering and throttle
    actuators: Box<dyn ActuatorInterface>,
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

impl RtlMode {
    /// Create new RTL mode
    ///
    /// # Arguments
    ///
    /// * `actuators` - Actuator interface for steering and throttle
    /// * `gps_provider` - Function that returns current GPS position
    /// * `home_provider` - Function that returns home position (lat, lon)
    /// * `heading_provider` - Function that returns current heading (degrees, 0-360)
    pub fn new(
        actuators: Box<dyn ActuatorInterface>,
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
    pub fn can_enter(
        gps_provider: fn() -> Option<GpsPosition>,
        home_provider: fn() -> Option<(f32, f32)>,
    ) -> Result<(), &'static str> {
        let gps = gps_provider().ok_or("RTL requires GPS fix")?;
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("RTL requires 3D GPS fix");
        }
        home_provider().ok_or("RTL requires home position")?;
        Ok(())
    }

    /// Check if vehicle has arrived at home
    pub fn has_arrived(&self) -> bool {
        self.state.as_ref().map(|s| s.arrived).unwrap_or(false)
    }
}

impl Mode for RtlMode {
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
            return Err("GPS fix lost during RTL");
        }

        // Get heading from heading provider (fallback to GPS COG)
        let heading = (self.heading_provider)()
            .or(gps.course_over_ground)
            .ok_or("No heading available for RTL")?;

        // Create target
        let target = PositionTarget::new(state.target_lat, state.target_lon);

        // Navigate to target
        let output =
            self.nav_controller
                .update(gps.latitude, gps.longitude, &target, heading, dt, None);

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

    #[test]
    fn test_rtl_state_default() {
        let state = RtlState::default();
        assert_eq!(state.target_lat, 0.0);
        assert_eq!(state.target_lon, 0.0);
        assert!(!state.arrived);
    }
}
