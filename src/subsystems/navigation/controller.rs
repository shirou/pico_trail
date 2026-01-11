//! Navigation controller implementations
//!
//! This module provides the `NavigationController` trait and implementations
//! for converting position targets into steering and throttle commands.

use crate::devices::gps::GpsPosition;

use super::geo::{calculate_bearing, calculate_distance, wrap_180};
use super::types::{NavigationOutput, PositionTarget, SimpleNavConfig};

/// Navigation controller trait for position-based navigation
///
/// Implementations convert a current position and target position into
/// steering and throttle commands for vehicle control.
pub trait NavigationController {
    /// Update navigation state and calculate commands
    ///
    /// Called at control loop rate (typically 50Hz) but may only
    /// recalculate when GPS updates (1-10Hz).
    ///
    /// # Arguments
    /// * `current` - Current GPS position
    /// * `target` - Target position to navigate to
    /// * `heading` - Current heading in degrees (0-360, 0 = North)
    /// * `_dt` - Time delta since last update (seconds)
    ///
    /// # Returns
    /// Navigation output with steering, throttle, and status
    fn update(
        &mut self,
        current: &GpsPosition,
        target: &PositionTarget,
        heading: f32,
        _dt: f32,
    ) -> NavigationOutput;

    /// Reset controller state
    ///
    /// Called on mode change or target change to clear any accumulated state.
    fn reset(&mut self);
}

/// Simple bearing-based navigation controller (L1-lite)
///
/// This controller calculates steering based on heading error (bearing to target
/// minus current heading) and throttle based on distance to target with
/// approach slowdown.
///
/// # Steering Calculation
/// - Heading error = bearing_to_target - current_heading (wrapped to ±180°)
/// - Steering = heading_error / max_heading_error (clamped to ±1.0)
///
/// # Throttle Calculation
/// - At target (distance < wp_radius): throttle = 0.0
/// - In approach zone (distance < approach_dist): throttle scales linearly
/// - Far from target: throttle = 1.0
pub struct SimpleNavigationController {
    config: SimpleNavConfig,
}

impl SimpleNavigationController {
    /// Create a new SimpleNavigationController with default configuration
    pub fn new() -> Self {
        Self {
            config: SimpleNavConfig::default(),
        }
    }

    /// Create a new SimpleNavigationController with custom configuration
    pub fn with_config(config: SimpleNavConfig) -> Self {
        Self { config }
    }

    /// Get the current configuration
    pub fn config(&self) -> &SimpleNavConfig {
        &self.config
    }

    /// Calculate throttle based on distance to target
    fn calculate_throttle(&self, distance: f32) -> f32 {
        if distance < self.config.wp_radius {
            // At target - stop
            0.0
        } else if distance < self.config.approach_dist {
            // In approach zone - scale throttle
            let ratio = (distance - self.config.wp_radius)
                / (self.config.approach_dist - self.config.wp_radius);
            // Linear interpolation from min_approach_throttle to 1.0
            self.config.min_approach_throttle + ratio * (1.0 - self.config.min_approach_throttle)
        } else {
            // Far from target - full throttle
            1.0
        }
    }

    /// Calculate steering based on heading error
    fn calculate_steering(&self, heading_error: f32) -> f32 {
        let steering = heading_error / self.config.max_heading_error;
        steering.clamp(-1.0, 1.0)
    }
}

impl Default for SimpleNavigationController {
    fn default() -> Self {
        Self::new()
    }
}

impl NavigationController for SimpleNavigationController {
    fn update(
        &mut self,
        current: &GpsPosition,
        target: &PositionTarget,
        heading: f32,
        _dt: f32,
    ) -> NavigationOutput {
        // Calculate bearing and distance to target
        let bearing = calculate_bearing(
            current.latitude,
            current.longitude,
            target.latitude,
            target.longitude,
        );
        let distance = calculate_distance(
            current.latitude,
            current.longitude,
            target.latitude,
            target.longitude,
        );

        // Calculate heading error (wrapped to ±180°)
        let heading_error = wrap_180(bearing - heading);

        // Check if at target
        let at_target = distance < self.config.wp_radius;

        // Calculate steering and throttle
        let mut steering = self.calculate_steering(heading_error);
        let throttle = self.calculate_throttle(distance);

        // Safety: ensure outputs are in valid ranges and handle NaN
        steering = sanitize_output(steering, -1.0, 1.0, 0.0);
        let throttle = sanitize_output(throttle, 0.0, 1.0, 0.0);

        NavigationOutput {
            steering,
            throttle,
            distance_m: distance,
            bearing_deg: bearing,
            heading_error_deg: heading_error,
            at_target,
        }
    }

    fn reset(&mut self) {
        // SimpleNavigationController has no accumulated state to reset
    }
}

/// Sanitize output value, handling NaN and infinity
fn sanitize_output(value: f32, min: f32, max: f32, default: f32) -> f32 {
    if value.is_nan() || value.is_infinite() {
        default
    } else {
        value.clamp(min, max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::gps::GpsFixType;

    fn make_gps_position(lat: f32, lon: f32) -> GpsPosition {
        GpsPosition {
            latitude: lat,
            longitude: lon,
            altitude: 0.0,
            speed: 5.0,
            course_over_ground: None, // Not used anymore
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        }
    }

    // ========== At Target Tests ==========

    #[test]
    fn test_at_target_within_radius() {
        let mut controller = SimpleNavigationController::new();
        // Target is ~1m away (within 2m wp_radius)
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(0.000009, 0.0); // ~1m north
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);
        assert!(
            output.at_target,
            "Should be at target when within wp_radius"
        );
        assert!(
            output.throttle < 0.01,
            "Throttle should be ~0 at target, got {}",
            output.throttle
        );
    }

    #[test]
    fn test_at_target_outside_radius() {
        let mut controller = SimpleNavigationController::new();
        // Target is ~100m away (outside 2m wp_radius)
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(0.0009, 0.0); // ~100m north
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);
        assert!(
            !output.at_target,
            "Should not be at target when outside wp_radius"
        );
    }

    // ========== Steering Tests ==========

    #[test]
    fn test_steering_target_ahead() {
        let mut controller = SimpleNavigationController::new();
        // Vehicle facing north (0°), target is north
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(1.0, 0.0); // North
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);
        assert!(
            output.steering.abs() < 0.1,
            "Steering should be ~0 when target ahead, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_target_to_left() {
        let mut controller = SimpleNavigationController::new();
        // Vehicle facing north (0°), target is west (bearing 270°)
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(0.0, -1.0); // West
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);
        assert!(
            output.steering < -0.5,
            "Steering should be negative (left) when target to left, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_target_to_right() {
        let mut controller = SimpleNavigationController::new();
        // Vehicle facing north (0°), target is east (bearing 90°)
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(0.0, 1.0); // East
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);
        assert!(
            output.steering > 0.5,
            "Steering should be positive (right) when target to right, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_target_behind() {
        let mut controller = SimpleNavigationController::new();
        // Vehicle facing north (0°), target is south (bearing 180°)
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(-1.0, 0.0); // South
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);
        // Should steer maximally (either left or right based on wrap)
        assert!(
            output.steering.abs() > 0.9,
            "Steering should be maximal when target behind, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_clamped_to_range() {
        let mut controller = SimpleNavigationController::new();
        // Vehicle facing east (90°), target is west (bearing 270°)
        // Heading error = 180°, should be clamped
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(0.0, -1.0); // West
        let heading = 90.0;

        let output = controller.update(&current, &target, heading, 0.02);
        assert!(
            output.steering >= -1.0 && output.steering <= 1.0,
            "Steering should be clamped to [-1, 1], got {}",
            output.steering
        );
    }

    // ========== Throttle Tests ==========

    #[test]
    fn test_throttle_far_from_target() {
        let mut controller = SimpleNavigationController::new();
        // Target is 100m away (far from approach_dist of 10m)
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(0.0009, 0.0); // ~100m north
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);
        assert!(
            (output.throttle - 1.0).abs() < 0.01,
            "Throttle should be 1.0 far from target, got {}",
            output.throttle
        );
    }

    #[test]
    fn test_throttle_in_approach_zone() {
        let mut controller = SimpleNavigationController::new();
        // Target is ~5m away (in approach zone: between 2m and 10m)
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(0.000045, 0.0); // ~5m north
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);
        assert!(
            output.throttle > 0.2 && output.throttle < 1.0,
            "Throttle should be reduced in approach zone, got {}",
            output.throttle
        );
    }

    #[test]
    fn test_throttle_at_target() {
        let mut controller = SimpleNavigationController::new();
        // Target is ~1m away (at target: within 2m)
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(0.000009, 0.0); // ~1m north
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);
        assert!(
            output.throttle < 0.01,
            "Throttle should be 0 at target, got {}",
            output.throttle
        );
    }

    // ========== Output Range Tests ==========

    #[test]
    fn test_output_ranges_always_valid() {
        let mut controller = SimpleNavigationController::new();

        // Test various positions (lat1, lon1, lat2, lon2, heading)
        let test_cases = [
            (0.0, 0.0, 0.0, 0.0, 0.0),    // Same position
            (0.0, 0.0, 1.0, 1.0, 45.0),   // Northeast target
            (0.0, 0.0, -1.0, -1.0, 0.0),  // Southwest target, facing north
            (89.0, 0.0, 89.0, 1.0, 90.0), // High latitude
        ];

        for (lat1, lon1, lat2, lon2, heading) in test_cases {
            let current = make_gps_position(lat1, lon1);
            let target = PositionTarget::new(lat2, lon2);
            let output = controller.update(&current, &target, heading, 0.02);

            assert!(
                output.steering >= -1.0 && output.steering <= 1.0,
                "Steering out of range: {} for case {:?}",
                output.steering,
                (lat1, lon1, lat2, lon2)
            );
            assert!(
                output.throttle >= 0.0 && output.throttle <= 1.0,
                "Throttle out of range: {} for case {:?}",
                output.throttle,
                (lat1, lon1, lat2, lon2)
            );
        }
    }

    // ========== Edge Cases ==========

    #[test]
    fn test_same_position() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(35.6762, 139.6503);
        let target = PositionTarget::new(35.6762, 139.6503);
        let heading = 45.0;

        let output = controller.update(&current, &target, heading, 0.02);
        assert!(output.at_target, "Should be at target when same position");
        assert!(output.distance_m < 1.0, "Distance should be ~0");
    }

    #[test]
    fn test_reset_does_not_crash() {
        let mut controller = SimpleNavigationController::new();
        controller.reset(); // Should not panic
    }

    #[test]
    fn test_custom_config() {
        let config = SimpleNavConfig {
            wp_radius: 5.0,
            approach_dist: 20.0,
            max_heading_error: 45.0,
            min_approach_throttle: 0.3,
        };
        let controller = SimpleNavigationController::with_config(config);

        assert!((controller.config().wp_radius - 5.0).abs() < 0.001);
        assert!((controller.config().approach_dist - 20.0).abs() < 0.001);
    }

    // ========== Additional Edge Cases ==========

    #[test]
    fn test_very_large_distance() {
        let mut controller = SimpleNavigationController::new();
        // Tokyo to New York: approximately 10,800km
        let current = make_gps_position(35.6762, 139.6503);
        let target = PositionTarget::new(40.7128, -74.0060);
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);

        // Should still produce valid outputs
        assert!(
            output.steering >= -1.0 && output.steering <= 1.0,
            "Steering out of range for large distance: {}",
            output.steering
        );
        assert!(
            output.throttle >= 0.0 && output.throttle <= 1.0,
            "Throttle out of range for large distance: {}",
            output.throttle
        );
        assert!(
            !output.at_target,
            "Should not be at target for large distance"
        );
        // Distance should be roughly 10,800 km
        assert!(
            output.distance_m > 10_000_000.0 && output.distance_m < 12_000_000.0,
            "Distance should be ~10,800km, got {}km",
            output.distance_m / 1000.0
        );
    }

    #[test]
    fn test_near_poles() {
        let mut controller = SimpleNavigationController::new();
        // Near north pole
        let current = make_gps_position(89.0, 0.0);
        let target = PositionTarget::new(89.5, 180.0);
        let heading = 0.0;

        let output = controller.update(&current, &target, heading, 0.02);

        // Should still produce valid outputs
        assert!(
            output.steering >= -1.0 && output.steering <= 1.0,
            "Steering out of range near poles: {}",
            output.steering
        );
        assert!(
            output.throttle >= 0.0 && output.throttle <= 1.0,
            "Throttle out of range near poles: {}",
            output.throttle
        );
    }

    #[test]
    fn test_date_line_crossing() {
        let mut controller = SimpleNavigationController::new();
        // Crossing the international date line (180° longitude)
        let current = make_gps_position(0.0, 179.0); // Near date line
        let target = PositionTarget::new(0.0, -179.0); // Just past date line
        let heading = 90.0; // Facing east

        let output = controller.update(&current, &target, heading, 0.02);

        // Should still produce valid outputs
        assert!(
            output.steering >= -1.0 && output.steering <= 1.0,
            "Steering out of range crossing date line: {}",
            output.steering
        );
        assert!(
            output.throttle >= 0.0 && output.throttle <= 1.0,
            "Throttle out of range crossing date line: {}",
            output.throttle
        );
        // Distance should be small (about 222km for 2 degrees at equator)
        assert!(
            output.distance_m < 250_000.0,
            "Distance should be small when crossing date line, got {}km",
            output.distance_m / 1000.0
        );
    }

    // ========== Pseudo-Property Tests (manual range verification) ==========

    #[test]
    fn test_steering_range_for_all_headings() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(1.0, 0.0); // North

        // Test steering for all possible heading values
        for heading in (0..360).step_by(10) {
            let output = controller.update(&current, &target, heading as f32, 0.02);

            assert!(
                output.steering >= -1.0 && output.steering <= 1.0,
                "Steering {} out of range for heading {}°",
                output.steering,
                heading
            );
        }
    }

    #[test]
    fn test_steering_range_for_all_bearings() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);
        let heading = 0.0;

        // Test steering for targets in all directions
        for bearing in (0..360).step_by(15) {
            let rad = (bearing as f32) * core::f32::consts::PI / 180.0;
            let lat = libm::sinf(rad);
            let lon = libm::cosf(rad);

            let target = PositionTarget::new(lat, lon);
            let output = controller.update(&current, &target, heading, 0.02);

            assert!(
                output.steering >= -1.0 && output.steering <= 1.0,
                "Steering {} out of range for bearing {}°",
                output.steering,
                bearing
            );
        }
    }

    #[test]
    fn test_throttle_range_for_all_distances() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);
        let heading = 0.0;

        // Test throttle for various distances (0.1m to 1000km)
        let distances_deg = [
            0.000001, // ~0.1m
            0.00001,  // ~1m
            0.0001,   // ~10m
            0.001,    // ~100m
            0.01,     // ~1km
            0.1,      // ~10km
            1.0,      // ~100km
            10.0,     // ~1000km
        ];

        for lat_offset in distances_deg {
            let target = PositionTarget::new(lat_offset, 0.0);
            let output = controller.update(&current, &target, heading, 0.02);

            assert!(
                output.throttle >= 0.0 && output.throttle <= 1.0,
                "Throttle {} out of range for distance ~{}m",
                output.throttle,
                output.distance_m
            );
        }
    }

    // ========== Heading Input Edge Cases ==========

    #[test]
    fn test_heading_at_zero() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(1.0, 0.0); // North (bearing 0°)

        let output = controller.update(&current, &target, 0.0, 0.02);

        // Heading error should be ~0 (target is north, heading is north)
        assert!(
            output.heading_error_deg.abs() < 1.0,
            "Heading error should be ~0, got {}",
            output.heading_error_deg
        );
        assert!(
            output.steering.abs() < 0.1,
            "Steering should be ~0, got {}",
            output.steering
        );
    }

    #[test]
    fn test_heading_at_360() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(1.0, 0.0); // North (bearing 0°)

        // Heading 360° should be equivalent to 0°
        let output = controller.update(&current, &target, 360.0, 0.02);

        // Heading error should be ~0 (360° = 0°)
        assert!(
            output.heading_error_deg.abs() < 1.0,
            "Heading error should be ~0 for 360°, got {}",
            output.heading_error_deg
        );
    }

    #[test]
    fn test_heading_wraparound_positive() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(1.0, 0.0); // North (bearing 0°)

        // Heading 350°, target bearing 10°: shortest turn is +20° (right)
        let output = controller.update(&current, &target, 350.0, 0.02);

        // Heading error should be positive (turn right)
        // bearing(0°) - heading(350°) = -350° → wrapped to +10°
        assert!(
            output.heading_error_deg > 0.0,
            "Heading error should be positive for turn right, got {}",
            output.heading_error_deg
        );
        assert!(
            output.steering > 0.0,
            "Steering should be positive for turn right, got {}",
            output.steering
        );
    }

    #[test]
    fn test_heading_wraparound_negative() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(0.0, -1.0); // West (bearing ~270°)

        // Heading 280°, target bearing 270°: shortest turn is -10° (left)
        let output = controller.update(&current, &target, 280.0, 0.02);

        // Heading error should be negative (turn left)
        assert!(
            output.heading_error_deg < 0.0,
            "Heading error should be negative for turn left, got {}",
            output.heading_error_deg
        );
        assert!(
            output.steering < 0.0,
            "Steering should be negative for turn left, got {}",
            output.steering
        );
    }

    #[test]
    fn test_heading_exactly_opposite() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(1.0, 0.0); // North (bearing 0°)

        // Heading exactly 180° opposite to target
        let output = controller.update(&current, &target, 180.0, 0.02);

        // Heading error should be ±180° (maximum error)
        assert!(
            output.heading_error_deg.abs() > 170.0,
            "Heading error should be ~180° when opposite, got {}",
            output.heading_error_deg
        );
        // Steering should be maximal
        assert!(
            output.steering.abs() > 0.9,
            "Steering should be maximal when opposite, got {}",
            output.steering
        );
    }

    #[test]
    fn test_heading_various_dt_values() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);
        let target = PositionTarget::new(1.0, 0.0);
        let heading = 45.0;

        // dt should not affect simple navigation controller (no PID integration)
        let dt_values = [0.001, 0.02, 0.1, 1.0];
        let mut outputs = Vec::new();

        for dt in dt_values {
            outputs.push(controller.update(&current, &target, heading, dt));
        }

        // All outputs should be the same (dt-independent)
        for output in &outputs {
            assert!(
                (output.steering - outputs[0].steering).abs() < 0.001,
                "Steering should be dt-independent"
            );
            assert!(
                (output.throttle - outputs[0].throttle).abs() < 0.001,
                "Throttle should be dt-independent"
            );
        }
    }

    #[test]
    fn test_bearing_output_range() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);

        // Test bearing output for targets in all directions
        let test_cases = [
            (1.0, 0.0, 0.0),     // North
            (1.0, 1.0, 45.0),    // Northeast (approximate)
            (0.0, 1.0, 90.0),    // East
            (-1.0, 1.0, 135.0),  // Southeast (approximate)
            (-1.0, 0.0, 180.0),  // South
            (-1.0, -1.0, 225.0), // Southwest (approximate)
            (0.0, -1.0, 270.0),  // West
            (1.0, -1.0, 315.0),  // Northwest (approximate)
        ];

        for (lat, lon, expected_bearing) in test_cases {
            let target = PositionTarget::new(lat, lon);
            let output = controller.update(&current, &target, 0.0, 0.02);

            // Bearing should be in range [0, 360)
            assert!(
                output.bearing_deg >= 0.0 && output.bearing_deg < 360.0,
                "Bearing {} out of range for target ({}, {})",
                output.bearing_deg,
                lat,
                lon
            );

            // Bearing should be approximately correct (within 10° due to Earth curvature)
            let bearing_diff = (output.bearing_deg - expected_bearing).abs();
            let bearing_diff = if bearing_diff > 180.0 {
                360.0 - bearing_diff
            } else {
                bearing_diff
            };
            assert!(
                bearing_diff < 15.0,
                "Bearing {} not close to expected {} for target ({}, {})",
                output.bearing_deg,
                expected_bearing,
                lat,
                lon
            );
        }
    }

    #[test]
    fn test_distance_output() {
        let mut controller = SimpleNavigationController::new();
        let current = make_gps_position(0.0, 0.0);

        // 1 degree of latitude ≈ 111km
        let target = PositionTarget::new(1.0, 0.0);
        let output = controller.update(&current, &target, 0.0, 0.02);

        // Distance should be approximately 111km (110-112km acceptable)
        assert!(
            output.distance_m > 110_000.0 && output.distance_m < 112_000.0,
            "Distance should be ~111km, got {}km",
            output.distance_m / 1000.0
        );
    }
}
