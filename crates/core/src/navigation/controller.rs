//! Navigation controller implementations
//!
//! This module provides the `NavigationController` trait and implementations
//! for converting position targets into steering and throttle commands.

use crate::navigation::geo::{calculate_bearing, calculate_distance, wrap_180};
use crate::navigation::types::{NavigationOutput, PositionTarget, SimpleNavConfig};

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
    /// * `current_lat` - Current latitude in degrees
    /// * `current_lon` - Current longitude in degrees
    /// * `target` - Target position to navigate to
    /// * `heading` - Current heading in degrees (0-360, 0 = North)
    /// * `dt` - Time delta since last update (seconds)
    ///
    /// # Returns
    /// Navigation output with steering, throttle, and status
    fn update(
        &mut self,
        current_lat: f32,
        current_lon: f32,
        target: &PositionTarget,
        heading: f32,
        dt: f32,
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
/// - Heading error = bearing_to_target - current_heading (wrapped to +/-180)
/// - Steering = heading_error / max_heading_error (clamped to +/-1.0)
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
        current_lat: f32,
        current_lon: f32,
        target: &PositionTarget,
        heading: f32,
        _dt: f32,
    ) -> NavigationOutput {
        // Calculate bearing and distance to target
        let bearing =
            calculate_bearing(current_lat, current_lon, target.latitude, target.longitude);
        let distance =
            calculate_distance(current_lat, current_lon, target.latitude, target.longitude);

        // Calculate heading error (wrapped to +/-180)
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

    fn make_position(lat: f32, lon: f32) -> (f32, f32) {
        (lat, lon)
    }

    // ========== At Target Tests ==========

    #[test]
    fn test_at_target_within_radius() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(0.000009, 0.0); // ~1m north
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02);
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
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(0.0009, 0.0); // ~100m north
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02);
        assert!(
            !output.at_target,
            "Should not be at target when outside wp_radius"
        );
    }

    // ========== Steering Tests ==========

    #[test]
    fn test_steering_target_ahead() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(1.0, 0.0); // North
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02);
        assert!(
            output.steering.abs() < 0.1,
            "Steering should be ~0 when target ahead, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_target_to_left() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(0.0, -1.0); // West
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02);
        assert!(
            output.steering < -0.5,
            "Steering should be negative (left) when target to left, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_target_to_right() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(0.0, 1.0); // East
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02);
        assert!(
            output.steering > 0.5,
            "Steering should be positive (right) when target to right, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_target_behind() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(-1.0, 0.0); // South
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02);
        assert!(
            output.steering.abs() > 0.9,
            "Steering should be maximal when target behind, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_clamped_to_range() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(0.0, -1.0); // West
        let heading = 90.0;

        let output = controller.update(lat, lon, &target, heading, 0.02);
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
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(0.0009, 0.0); // ~100m north
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02);
        assert!(
            (output.throttle - 1.0).abs() < 0.01,
            "Throttle should be 1.0 far from target, got {}",
            output.throttle
        );
    }

    #[test]
    fn test_throttle_in_approach_zone() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(0.000045, 0.0); // ~5m north
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02);
        assert!(
            output.throttle > 0.2 && output.throttle < 1.0,
            "Throttle should be reduced in approach zone, got {}",
            output.throttle
        );
    }

    #[test]
    fn test_throttle_at_target() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(0.000009, 0.0); // ~1m north
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02);
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

        let test_cases = [
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 1.0, 45.0),
            (0.0, 0.0, -1.0, -1.0, 0.0),
            (89.0, 0.0, 89.0, 1.0, 90.0),
        ];

        for (lat1, lon1, lat2, lon2, heading) in test_cases {
            let target = PositionTarget::new(lat2, lon2);
            let output = controller.update(lat1, lon1, &target, heading, 0.02);

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
        let target = PositionTarget::new(35.6762, 139.6503);
        let heading = 45.0;

        let output = controller.update(35.6762, 139.6503, &target, heading, 0.02);
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

    #[test]
    fn test_distance_output() {
        let mut controller = SimpleNavigationController::new();

        // 1 degree of latitude ~ 111km
        let target = PositionTarget::new(1.0, 0.0);
        let output = controller.update(0.0, 0.0, &target, 0.0, 0.02);

        assert!(
            output.distance_m > 110_000.0 && output.distance_m < 112_000.0,
            "Distance should be ~111km, got {}km",
            output.distance_m / 1000.0
        );
    }
}
