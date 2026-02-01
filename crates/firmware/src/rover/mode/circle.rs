//! Circle Mode
//!
//! Autonomous circular orbit around a fixed center point.
//!
//! ## Behavior
//!
//! - On entry: Calculate center point (CIRC_RADIUS ahead in heading direction)
//! - Each update: Generate look-ahead target point on circle perimeter
//! - Delegate path following to L1 navigation controller
//! - Support stationary mode (CIRC_RADIUS = 0)
//!
//! ## Hybrid Approach (ADR-897ov)
//!
//! Circle mode uses a continuous circle generator feeding look-ahead targets
//! to the existing L1 navigation controller, providing smooth motion while
//! reusing proven navigation infrastructure.
//!
//! ## References
//!
//! - FR-khjpl-circle-mode-implementation: Circle mode requirements
//! - ADR-897ov-circle-mode-path-generation: Architecture decision
//! - ArduPilot Circle Mode: https://ardupilot.org/rover/docs/circle-mode.html

use super::Mode;
use crate::devices::gps::GpsFixType;
use crate::devices::gps::GpsPosition;
use crate::libraries::ActuatorInterface;
use crate::subsystems::navigation::{
    calculate_bearing, offset_position, NavigationController, PositionTarget,
    SimpleNavigationController,
};

// CircleDirection is defined in core and re-exported via crate::parameters::circle
pub use pico_trail_core::parameters::circle::CircleDirection;

/// Circle mode state
#[derive(Clone, Copy, Debug)]
struct CircleState {
    /// Center point of the circle (calculated on mode entry)
    center_lat: f32,
    center_lon: f32,
    /// Circle radius in meters
    radius: f32,
    /// Target speed in m/s
    speed: f32,
    /// Orbit direction
    direction: CircleDirection,
}

impl Default for CircleState {
    fn default() -> Self {
        Self {
            center_lat: 0.0,
            center_lon: 0.0,
            radius: 20.0,
            speed: 2.0,
            direction: CircleDirection::Clockwise,
        }
    }
}

/// Circle mode configuration (ArduPilot-compatible parameters)
#[derive(Clone, Debug)]
pub struct CircleConfig {
    /// Circle radius in meters (CIRC_RADIUS)
    pub radius: f32,
    /// Target speed in m/s (CIRC_SPEED)
    pub speed: f32,
    /// Orbit direction (CIRC_DIR: 0=CW, 1=CCW)
    pub direction: CircleDirection,
}

impl Default for CircleConfig {
    fn default() -> Self {
        Self {
            radius: 20.0,
            speed: 2.0,
            direction: CircleDirection::Clockwise,
        }
    }
}

impl CircleConfig {
    /// Create configuration from CircleParams
    ///
    /// # Arguments
    ///
    /// * `params` - Circle parameters loaded from parameter store
    ///
    /// # Returns
    ///
    /// Circle configuration with values from parameters
    pub fn from_params(params: &crate::parameters::CircleParams) -> Self {
        Self {
            radius: params.radius,
            speed: params.speed,
            direction: params.direction,
        }
    }

    /// Create configuration from parameter store
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to read from
    ///
    /// # Returns
    ///
    /// Circle configuration with values from store or defaults
    pub fn from_store(store: &crate::parameters::ParameterStore) -> Self {
        let params = crate::parameters::CircleParams::from_store(store);
        Self::from_params(&params)
    }
}

/// Circle Mode
///
/// Provides autonomous circular orbit around a center point.
pub struct CircleMode<'a> {
    /// Actuator interface for steering and throttle
    actuators: &'a mut dyn ActuatorInterface,
    /// Navigation controller for path following
    nav_controller: SimpleNavigationController,
    /// Circle state (set on mode entry)
    state: Option<CircleState>,
    /// Configuration (loaded on mode entry)
    config: CircleConfig,
    /// GPS position provider function
    gps_provider: fn() -> Option<GpsPosition>,
    /// Heading provider function (AHRS or GPS COG)
    heading_provider: fn() -> Option<f32>,
}

impl<'a> CircleMode<'a> {
    /// Look-ahead time in seconds for target calculation
    const LOOK_AHEAD_TIME: f32 = 1.5;

    /// Create new Circle mode
    ///
    /// # Arguments
    ///
    /// * `actuators` - Actuator interface for steering and throttle
    /// * `config` - Circle mode configuration
    /// * `gps_provider` - Function that returns current GPS position
    /// * `heading_provider` - Function that returns current heading in degrees
    pub fn new(
        actuators: &'a mut dyn ActuatorInterface,
        config: CircleConfig,
        gps_provider: fn() -> Option<GpsPosition>,
        heading_provider: fn() -> Option<f32>,
    ) -> Self {
        Self {
            actuators,
            nav_controller: SimpleNavigationController::new(),
            state: None,
            config,
            gps_provider,
            heading_provider,
        }
    }

    /// Calculate the next target point on the circle perimeter
    ///
    /// Uses look-ahead time to generate a target point ahead of the vehicle's
    /// current position on the circle, providing smooth path following.
    fn calculate_target(&self, current: &GpsPosition) -> Option<PositionTarget> {
        let state = self.state.as_ref()?;

        // 1. Calculate current angle from center to vehicle
        let current_angle = calculate_bearing(
            state.center_lat,
            state.center_lon,
            current.latitude,
            current.longitude,
        );

        // 2. Calculate angular velocity: ω = v / r (rad/s)
        let angular_velocity = state.speed / state.radius;
        let angular_velocity_deg = angular_velocity.to_degrees();

        // 3. Calculate look-ahead angle
        let look_ahead_angle = angular_velocity_deg * Self::LOOK_AHEAD_TIME;

        // 4. Calculate target angle based on direction
        let target_angle = match state.direction {
            CircleDirection::Clockwise => current_angle + look_ahead_angle,
            CircleDirection::CounterClockwise => current_angle - look_ahead_angle,
        };

        // 5. Calculate target point on circle perimeter
        let (target_lat, target_lon) = offset_position(
            state.center_lat,
            state.center_lon,
            state.radius,
            target_angle,
        );

        Some(PositionTarget::new(target_lat, target_lon))
    }

    /// Calculate center point based on current position and heading
    fn calculate_center(
        current_lat: f32,
        current_lon: f32,
        heading: f32,
        radius: f32,
    ) -> (f32, f32) {
        // Center is CIRC_RADIUS meters ahead in heading direction
        offset_position(current_lat, current_lon, radius, heading)
    }
}

impl<'a> Mode for CircleMode<'a> {
    fn enter(&mut self) -> Result<(), &'static str> {
        // Get current GPS position
        let gps = (self.gps_provider)().ok_or("No GPS fix")?;

        // Validate GPS fix
        if gps.fix_type < GpsFixType::Fix3D {
            return Err("GPS fix insufficient for Circle mode");
        }

        // Get heading (from AHRS or GPS COG)
        let heading = (self.heading_provider)()
            .or(gps.course_over_ground)
            .ok_or("No heading available")?;

        // Handle stationary mode (radius = 0)
        if self.config.radius <= 0.0 {
            self.state = Some(CircleState {
                center_lat: gps.latitude,
                center_lon: gps.longitude,
                radius: 0.0,
                speed: 0.0,
                direction: self.config.direction,
            });
            crate::log_info!("Circle mode entered (stationary)");
            return Ok(());
        }

        // Calculate center point
        let (center_lat, center_lon) =
            Self::calculate_center(gps.latitude, gps.longitude, heading, self.config.radius);

        // Store circle state
        self.state = Some(CircleState {
            center_lat,
            center_lon,
            radius: self.config.radius,
            speed: self.config.speed,
            direction: self.config.direction,
        });

        crate::log_info!("Circle mode entered");
        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        let state = self.state.as_ref().ok_or("Circle mode not initialized")?;

        // Handle stationary mode
        if state.radius <= 0.0 {
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        }

        // Get current GPS position
        let gps = (self.gps_provider)().ok_or("GPS lost")?;

        // Get heading from heading provider (fallback to GPS COG)
        let heading = (self.heading_provider)()
            .or(gps.course_over_ground)
            .ok_or("No heading available for Circle mode")?;

        // Calculate target point on circle
        let target = self
            .calculate_target(&gps)
            .ok_or("Failed to calculate target")?;

        // Delegate to navigation controller
        let output =
            self.nav_controller
                .update(gps.latitude, gps.longitude, &target, heading, dt, None);

        // Apply commands to actuators
        self.actuators.set_steering(output.steering)?;
        self.actuators.set_throttle(output.throttle)?;

        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Exiting Circle mode");

        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;

        // Clear state
        self.state = None;
        self.nav_controller.reset();

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Circle"
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::subsystems::navigation::calculate_distance;

    // ========== CircleDirection Tests ==========

    #[test]
    fn test_circle_direction_default() {
        let dir = CircleDirection::default();
        assert_eq!(dir, CircleDirection::Clockwise);
    }

    #[test]
    fn test_circle_direction_from_i8() {
        assert_eq!(CircleDirection::from(0), CircleDirection::Clockwise);
        assert_eq!(CircleDirection::from(1), CircleDirection::CounterClockwise);
        assert_eq!(CircleDirection::from(2), CircleDirection::CounterClockwise);
    }

    // ========== CircleConfig Tests ==========

    #[test]
    fn test_circle_config_default() {
        let config = CircleConfig::default();
        assert!((config.radius - 20.0).abs() < 0.001);
        assert!((config.speed - 2.0).abs() < 0.001);
        assert_eq!(config.direction, CircleDirection::Clockwise);
    }

    #[test]
    fn test_circle_config_from_params() {
        use crate::parameters::CircleParams;

        let params = CircleParams {
            radius: 50.0,
            speed: 3.5,
            direction: CircleDirection::CounterClockwise,
        };

        let config = CircleConfig::from_params(&params);
        assert!((config.radius - 50.0).abs() < 0.001);
        assert!((config.speed - 3.5).abs() < 0.001);
        assert_eq!(config.direction, CircleDirection::CounterClockwise);
    }

    #[test]
    fn test_circle_config_from_store() {
        use crate::parameters::{CircleParams, ParamValue, ParameterStore};

        let mut store = ParameterStore::new();
        CircleParams::register_defaults(&mut store).unwrap();

        // Use default values from store
        let config = CircleConfig::from_store(&store);
        assert!((config.radius - 20.0).abs() < 0.001);
        assert!((config.speed - 2.0).abs() < 0.001);
        assert_eq!(config.direction, CircleDirection::Clockwise);

        // Override with custom values
        store.set("CIRC_RADIUS", ParamValue::Float(75.0)).unwrap();
        store.set("CIRC_SPEED", ParamValue::Float(4.0)).unwrap();
        store.set("CIRC_DIR", ParamValue::Int(1)).unwrap();

        let config = CircleConfig::from_store(&store);
        assert!((config.radius - 75.0).abs() < 0.001);
        assert!((config.speed - 4.0).abs() < 0.001);
        assert_eq!(config.direction, CircleDirection::CounterClockwise);
    }

    // ========== CircleState Tests ==========

    #[test]
    fn test_circle_state_default() {
        let state = CircleState::default();
        assert!((state.radius - 20.0).abs() < 0.001);
        assert!((state.speed - 2.0).abs() < 0.001);
        assert_eq!(state.direction, CircleDirection::Clockwise);
    }

    // ========== Center Point Calculation Tests ==========

    #[test]
    fn test_calculate_center_north() {
        // Vehicle at origin, facing north, radius 20m
        let (center_lat, center_lon) = CircleMode::calculate_center(0.0, 0.0, 0.0, 20.0);

        // Center should be ~20m north
        let distance = calculate_distance(0.0, 0.0, center_lat, center_lon);
        assert!(
            (distance - 20.0).abs() < 1.0,
            "Expected ~20m, got {}m",
            distance
        );
        assert!(center_lat > 0.0, "Center should be north of origin");
    }

    #[test]
    fn test_calculate_center_east() {
        // Vehicle at origin, facing east, radius 20m
        let (center_lat, center_lon) = CircleMode::calculate_center(0.0, 0.0, 90.0, 20.0);

        let distance = calculate_distance(0.0, 0.0, center_lat, center_lon);
        assert!(
            (distance - 20.0).abs() < 1.0,
            "Expected ~20m, got {}m",
            distance
        );
        assert!(center_lon > 0.0, "Center should be east of origin");
    }

    #[test]
    fn test_calculate_center_tokyo() {
        // Vehicle in Tokyo, facing northeast (45°), radius 50m
        let start_lat = 35.6762;
        let start_lon = 139.6503;
        let (center_lat, center_lon) =
            CircleMode::calculate_center(start_lat, start_lon, 45.0, 50.0);

        let distance = calculate_distance(start_lat, start_lon, center_lat, center_lon);
        assert!(
            (distance - 50.0).abs() < 1.0,
            "Expected ~50m, got {}m",
            distance
        );
    }
}
