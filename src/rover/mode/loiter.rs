//! Rover Loiter Mode
//!
//! Position holding mode for ground rovers with two behavior types:
//!
//! - **Type 0** (Stop): Simply stops the motors and records the loiter position.
//!   The rover will remain stationary but does not actively correct for drift.
//!
//! - **Type 1** (Active Hold): Monitors drift from the loiter position and
//!   actively navigates back when the rover drifts outside LOIT_RADIUS.
//!   Uses hysteresis to prevent oscillation at the radius boundary.
//!
//! ## Parameters
//!
//! - `LOIT_TYPE`: Loiter behavior type (0=stop, 1=active hold)
//! - `LOIT_RADIUS`: Drift threshold in meters before correction (Type 1)
//!
//! ## References
//!
//! - FR-aw3h3-rover-loiter-mode: Rover loiter mode requirements
//! - ADR-8icsq-vehicle-type-separation: Vehicle type separation architecture
//! - ADR-w9zpl-control-mode-architecture: Mode trait pattern
//! - ArduPilot Loiter Mode: https://ardupilot.org/rover/docs/loiter-mode.html

#[cfg(feature = "rover")]
use super::Mode;
#[cfg(all(feature = "rover", feature = "embassy"))]
use crate::devices::gps::GpsFixType;
#[cfg(feature = "rover")]
use crate::devices::gps::GpsPosition;
#[cfg(feature = "rover")]
use crate::libraries::ActuatorInterface;
#[cfg(feature = "rover")]
use crate::subsystems::navigation::{
    calculate_distance, offset_position, NavigationController, PositionTarget,
    SimpleNavigationController,
};

/// Hysteresis factor for drift detection (prevents oscillation at radius boundary)
///
/// When correcting, stop when distance < LOIT_RADIUS * HYSTERESIS_FACTOR.
/// This creates a deadband between "start correcting" and "stop correcting" thresholds.
#[cfg(feature = "rover")]
const HYSTERESIS_FACTOR: f32 = 0.8;

/// Low speed threshold for loiter point calculation
///
/// If rover is moving slower than this, use current position as loiter point.
/// Otherwise, project a stop point ahead based on deceleration.
#[cfg(feature = "rover")]
const LOW_SPEED_THRESHOLD: f32 = 0.5;

/// Maximum projection distance for loiter point calculation
///
/// Limits how far ahead the loiter point can be projected when entering
/// loiter mode at high speed.
#[cfg(feature = "rover")]
const MAX_PROJECTION: f32 = 50.0;

/// Default maximum deceleration for stop point projection (m/s^2)
#[cfg(feature = "rover")]
const DEFAULT_MAX_DECEL: f32 = 1.0;

/// Loiter mode state
#[derive(Clone, Copy, Debug)]
pub struct LoiterState {
    /// Recorded loiter position (latitude in degrees)
    pub loiter_lat: f32,
    /// Recorded loiter position (longitude in degrees)
    pub loiter_lon: f32,
    /// Type of loiter behavior (0=stop, 1=active hold)
    pub loiter_type: u8,
    /// Acceptable drift radius in meters
    pub radius: f32,
    /// True if currently correcting position (Type 1 only)
    pub is_correcting: bool,
}

impl Default for LoiterState {
    fn default() -> Self {
        Self {
            loiter_lat: 0.0,
            loiter_lon: 0.0,
            loiter_type: 0,
            radius: 2.0,
            is_correcting: false,
        }
    }
}

/// Rover Loiter Mode
///
/// Provides position holding with optional active correction.
#[cfg(feature = "rover")]
pub struct RoverLoiter<'a> {
    /// Actuator interface for steering and throttle
    actuators: &'a mut dyn ActuatorInterface,
    /// Navigation controller for position correction (Type 1)
    nav_controller: SimpleNavigationController,
    /// Loiter state (set on mode entry)
    state: Option<LoiterState>,
    /// GPS position provider function
    #[cfg(feature = "embassy")]
    gps_provider: fn() -> Option<GpsPosition>,
    /// Heading provider function (returns heading in degrees, 0-360)
    #[cfg(feature = "embassy")]
    heading_provider: fn() -> Option<f32>,
}

#[cfg(feature = "rover")]
#[allow(dead_code)]
impl<'a> RoverLoiter<'a> {
    /// Create new Loiter mode
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

    /// Create new Loiter mode (host tests)
    #[cfg(not(feature = "embassy"))]
    pub fn new(actuators: &'a mut dyn ActuatorInterface) -> Self {
        Self {
            actuators,
            nav_controller: SimpleNavigationController::new(),
            state: None,
        }
    }

    /// Calculate the loiter point based on current position and velocity
    ///
    /// If moving slowly (< 0.5 m/s), uses current position.
    /// If moving, projects a stop point based on v^2/(2*a) formula.
    fn calculate_loiter_point(gps: &GpsPosition) -> (f32, f32) {
        let speed = gps.speed;

        // Low speed - use current position
        if speed < LOW_SPEED_THRESHOLD {
            return (gps.latitude, gps.longitude);
        }

        // Project stop point based on deceleration
        let stop_distance = (speed * speed) / (2.0 * DEFAULT_MAX_DECEL);

        // Cap projection to reasonable distance
        let stop_distance = stop_distance.min(MAX_PROJECTION);

        // Get heading (course over ground)
        let heading = gps.course_over_ground.unwrap_or(0.0);

        // Project in direction of travel
        offset_position(gps.latitude, gps.longitude, stop_distance, heading)
    }

    /// Update for Type 0 (stop motors)
    fn update_type0(&mut self) -> Result<(), &'static str> {
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }

    /// Update for Type 1 (active position correction)
    #[cfg(feature = "embassy")]
    fn update_type1(&mut self, dt: f32) -> Result<(), &'static str> {
        let state = self.state.as_mut().ok_or("Loiter not initialized")?;

        // Get current GPS position
        let gps = (self.gps_provider)();

        // Check for GPS validity
        let gps = match gps {
            Some(g) if g.fix_type >= GpsFixType::Fix3D => g,
            _ => {
                // GPS lost - degrade to Type 0
                crate::log_warn!("Loiter: GPS lost, degrading to Type 0");
                return self.update_type0();
            }
        };

        // Calculate distance to loiter point
        let distance = calculate_distance(
            gps.latitude,
            gps.longitude,
            state.loiter_lat,
            state.loiter_lon,
        );

        // Hysteresis state machine
        if state.is_correcting {
            // Stop correcting when well within radius
            if distance < state.radius * HYSTERESIS_FACTOR {
                state.is_correcting = false;
                crate::log_debug!("Loiter: correction complete");
            }
        } else {
            // Start correcting when outside radius
            if distance > state.radius {
                state.is_correcting = true;
                crate::log_debug!("Loiter: drift detected, correcting");
            }
        }

        if state.is_correcting {
            // Get heading from heading provider (fallback to GPS COG)
            let heading = (self.heading_provider)()
                .or(gps.course_over_ground)
                .ok_or("No heading available for Loiter correction")?;

            // Navigate back to loiter point
            let target = PositionTarget::new(state.loiter_lat, state.loiter_lon);
            let output = self.nav_controller.update(&gps, &target, heading, dt);

            self.actuators.set_steering(output.steering)?;
            self.actuators.set_throttle(output.throttle)?;
        } else {
            // Within radius - stop
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
        }

        Ok(())
    }
}

#[cfg(feature = "rover")]
impl<'a> Mode for RoverLoiter<'a> {
    fn enter(&mut self) -> Result<(), &'static str> {
        #[cfg(feature = "embassy")]
        {
            // Get current GPS position
            let gps = (self.gps_provider)().ok_or("No GPS fix")?;

            // Validate GPS fix
            if gps.fix_type < GpsFixType::Fix3D {
                return Err("Loiter requires GPS fix");
            }

            // Load parameters
            let params = crate::parameters::LoiterParams::default();
            let loit_type = params.loit_type;
            let loit_radius = params.loit_radius;

            // Calculate loiter point
            let (loiter_lat, loiter_lon) = Self::calculate_loiter_point(&gps);

            // Store loiter state
            self.state = Some(LoiterState {
                loiter_lat,
                loiter_lon,
                loiter_type: loit_type,
                radius: loit_radius,
                is_correcting: false,
            });

            crate::log_info!(
                "Loiter: entered Type {} at ({}, {}), radius={}m",
                loit_type,
                loiter_lat,
                loiter_lon,
                loit_radius
            );
        }

        #[cfg(not(feature = "embassy"))]
        {
            crate::log_info!("Loiter mode entered (host test)");
        }

        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        #[cfg(feature = "embassy")]
        {
            let state = self.state.as_ref().ok_or("Loiter not initialized")?;

            match state.loiter_type {
                0 => self.update_type0(),
                1 => self.update_type1(dt),
                _ => self.update_type0(), // Default to stop
            }
        }

        #[cfg(not(feature = "embassy"))]
        {
            let _ = dt;
            // Host tests: no-op
            Ok(())
        }
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("Exiting Loiter mode");

        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;

        // Clear state
        self.state = None;
        self.nav_controller.reset();

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Loiter"
    }
}

#[cfg(all(test, feature = "rover"))]
mod tests {
    use super::*;
    use crate::communication::mavlink::state::{ArmedState, SystemState};
    use crate::devices::gps::{GpsFixType, GpsPosition};
    use crate::libraries::{ActuatorConfig, Actuators};
    use crate::platform::traits::pwm::PwmInterface;
    use crate::platform::Result;
    use crate::subsystems::navigation::calculate_distance;

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

    // ========== LoiterState Tests ==========

    #[test]
    fn test_loiter_state_default() {
        let state = LoiterState::default();
        assert_eq!(state.loiter_type, 0);
        assert!((state.radius - 2.0).abs() < 0.001);
        assert!(!state.is_correcting);
    }

    // ========== Loiter Point Calculation Tests ==========

    #[test]
    fn test_loiter_point_at_rest() {
        // When stopped, loiter point should be current position
        let gps = GpsPosition {
            latitude: 35.6762,
            longitude: 139.6503,
            altitude: 0.0,
            speed: 0.0,
            course_over_ground: None,
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        };

        let (lat, lon) = RoverLoiter::calculate_loiter_point(&gps);
        assert!((lat - 35.6762).abs() < 0.0001);
        assert!((lon - 139.6503).abs() < 0.0001);
    }

    #[test]
    fn test_loiter_point_low_speed() {
        // At low speed (< 0.5 m/s), use current position
        let gps = GpsPosition {
            latitude: 35.6762,
            longitude: 139.6503,
            altitude: 0.0,
            speed: 0.3,
            course_over_ground: Some(0.0),
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        };

        let (lat, lon) = RoverLoiter::calculate_loiter_point(&gps);
        assert!((lat - 35.6762).abs() < 0.0001);
        assert!((lon - 139.6503).abs() < 0.0001);
    }

    #[test]
    fn test_loiter_point_moving_north() {
        // Moving north at 5 m/s
        // Stop distance = v^2 / (2*a) = 25 / 2 = 12.5m
        let gps = GpsPosition {
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            speed: 5.0,
            course_over_ground: Some(0.0), // Heading north
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        };

        let (lat, lon) = RoverLoiter::calculate_loiter_point(&gps);

        // Should be north of current position
        assert!(lat > 0.0, "Loiter point should be north");

        // Verify distance is approximately 12.5m
        let distance = calculate_distance(0.0, 0.0, lat, lon);
        assert!(
            (distance - 12.5).abs() < 1.0,
            "Expected ~12.5m, got {}m",
            distance
        );
    }

    #[test]
    fn test_loiter_point_moving_east() {
        // Moving east at 5 m/s
        let gps = GpsPosition {
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            speed: 5.0,
            course_over_ground: Some(90.0), // Heading east
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        };

        let (lat, lon) = RoverLoiter::calculate_loiter_point(&gps);

        // Should be east of current position
        assert!(lon > 0.0, "Loiter point should be east");
    }

    #[test]
    fn test_loiter_point_projection_capped() {
        // Moving very fast (100 m/s) - should be capped to MAX_PROJECTION
        let gps = GpsPosition {
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            speed: 100.0,
            course_over_ground: Some(0.0),
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        };

        let (lat, lon) = RoverLoiter::calculate_loiter_point(&gps);

        // Verify distance is capped to MAX_PROJECTION (50m)
        let distance = calculate_distance(0.0, 0.0, lat, lon);
        assert!(
            (distance - MAX_PROJECTION).abs() < 1.0,
            "Expected ~50m, got {}m",
            distance
        );
    }

    // ========== Hysteresis Tests ==========

    #[test]
    fn test_hysteresis_prevents_oscillation() {
        let mut state = LoiterState {
            loiter_lat: 0.0,
            loiter_lon: 0.0,
            loiter_type: 1,
            radius: 2.0,
            is_correcting: true,
        };

        // Distance at 1.7m (85% of radius) - should still be correcting
        // due to hysteresis factor of 0.8 (stop at 1.6m)
        let distance = 1.7;
        if distance < state.radius * HYSTERESIS_FACTOR {
            state.is_correcting = false;
        }
        assert!(
            state.is_correcting,
            "Should still be correcting at 1.7m (threshold is 1.6m)"
        );

        // Distance at 1.5m - should stop correcting
        let distance = 1.5;
        if distance < state.radius * HYSTERESIS_FACTOR {
            state.is_correcting = false;
        }
        assert!(!state.is_correcting, "Should stop correcting at 1.5m");
    }

    #[test]
    fn test_hysteresis_start_correcting() {
        let mut state = LoiterState {
            loiter_lat: 0.0,
            loiter_lon: 0.0,
            loiter_type: 1,
            radius: 2.0,
            is_correcting: false,
        };

        // Distance at 1.9m - should not start correcting yet
        let distance = 1.9;
        if distance > state.radius {
            state.is_correcting = true;
        }
        assert!(
            !state.is_correcting,
            "Should not start correcting at 1.9m (threshold is 2.0m)"
        );

        // Distance at 2.1m - should start correcting
        let distance = 2.1;
        if distance > state.radius {
            state.is_correcting = true;
        }
        assert!(state.is_correcting, "Should start correcting at 2.1m");
    }

    // ========== RoverLoiter Tests ==========

    #[test]
    fn test_rover_loiter_creation() {
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
        let loiter_mode = RoverLoiter::new(&mut actuators);

        assert_eq!(loiter_mode.name(), "Loiter");
    }

    #[test]
    fn test_rover_loiter_enter_exit() {
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
        let mut loiter_mode = RoverLoiter::new(&mut actuators);

        // Enter should succeed (host test mode)
        assert!(loiter_mode.enter().is_ok());
        assert!(loiter_mode.exit().is_ok());
    }

    #[test]
    fn test_rover_loiter_update() {
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
        let mut loiter_mode = RoverLoiter::new(&mut actuators);

        // Enter first
        assert!(loiter_mode.enter().is_ok());

        // Update should succeed (host test mode)
        assert!(loiter_mode.update(0.02).is_ok());
    }
}
