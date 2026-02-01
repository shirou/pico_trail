//! Navigation controller implementations
//!
//! This module provides the `NavigationController` trait and implementations
//! for converting position targets into steering and throttle commands.

use crate::navigation::geo::{calculate_bearing, calculate_distance, wrap_180};
use crate::navigation::heading_filter::HeadingFilter;
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
    /// * `yaw_rate_dps` - Optional gyroscope yaw rate in degrees/second
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
        yaw_rate_dps: Option<f32>,
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
/// - PD controller: steering = P(error) + D(error_rate), with slew rate limiting
///
/// # Throttle Calculation
/// - At target (distance < wp_radius): throttle = 0.0
/// - In approach zone (distance < approach_dist): throttle scales linearly
/// - Far from target: throttle = 1.0
/// - Scaled by heading error: reduced to 0 when |heading error| >= throttle_heading_error_max
pub struct SimpleNavigationController {
    config: SimpleNavConfig,
    prev_heading_error: f32,
    prev_steering: f32,
    heading_filter: HeadingFilter,
}

impl SimpleNavigationController {
    /// Create a new SimpleNavigationController with default configuration
    pub fn new() -> Self {
        let config = SimpleNavConfig::default();
        let heading_filter = HeadingFilter::new(config.heading_filter_alpha);
        Self {
            config,
            prev_heading_error: 0.0,
            prev_steering: 0.0,
            heading_filter,
        }
    }

    /// Create a new SimpleNavigationController with custom configuration
    pub fn with_config(config: SimpleNavConfig) -> Self {
        let heading_filter = HeadingFilter::new(config.heading_filter_alpha);
        Self {
            config,
            prev_heading_error: 0.0,
            prev_steering: 0.0,
            heading_filter,
        }
    }

    /// Get the current configuration
    pub fn config(&self) -> &SimpleNavConfig {
        &self.config
    }

    /// Calculate throttle based on distance to target, scaled by heading error
    fn calculate_throttle(&self, distance: f32, heading_error_abs: f32) -> f32 {
        let distance_throttle = if distance < self.config.wp_radius {
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
        };

        // Heading error scaling: full throttle at 0 error, 0 at max degrees
        let heading_scale = if heading_error_abs >= self.config.throttle_heading_error_max {
            0.0
        } else {
            1.0 - (heading_error_abs / self.config.throttle_heading_error_max)
        };

        (distance_throttle * heading_scale).clamp(0.0, 1.0)
    }

    /// Calculate steering using PD controller with slew rate limiting
    fn calculate_steering(
        &mut self,
        heading_error: f32,
        dt: f32,
        yaw_rate_dps: Option<f32>,
    ) -> f32 {
        // Proportional term
        let p_term = heading_error / self.config.max_heading_error;

        // Derivative term: prefer gyroscope yaw rate over differenced heading error
        let d_term = if let Some(yaw_rate) = yaw_rate_dps {
            // Gyroscope provides clean yaw rate directly.
            // Negative because positive yaw rate (turning right) should reduce
            // positive steering (already turning right = less correction needed).
            -yaw_rate * self.config.steering_d_gain
        } else if dt > 0.0 {
            // Fallback: differentiate heading error
            let error_rate = (heading_error - self.prev_heading_error) / dt;
            error_rate * self.config.steering_d_gain
        } else {
            0.0
        };
        self.prev_heading_error = heading_error;

        // PD output
        let raw_steering = (p_term + d_term).clamp(-1.0, 1.0);

        // Slew rate limiting
        let limited_steering = if self.config.max_steering_rate > 0.0 && dt > 0.0 {
            let max_change = self.config.max_steering_rate * dt;
            let delta = (raw_steering - self.prev_steering).clamp(-max_change, max_change);
            self.prev_steering + delta
        } else {
            raw_steering
        };
        self.prev_steering = limited_steering;

        limited_steering.clamp(-1.0, 1.0)
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
        dt: f32,
        yaw_rate_dps: Option<f32>,
    ) -> NavigationOutput {
        // Apply heading EMA filter to smooth vibration-induced noise
        let filtered_heading = self.heading_filter.apply(heading);

        // Calculate bearing and distance to target
        let bearing =
            calculate_bearing(current_lat, current_lon, target.latitude, target.longitude);
        let distance =
            calculate_distance(current_lat, current_lon, target.latitude, target.longitude);

        // Calculate heading error (wrapped to +/-180)
        let heading_error = wrap_180(bearing - filtered_heading);

        // Check if at target
        let at_target = distance < self.config.wp_radius;

        // Calculate steering (PD + slew rate) and throttle (distance * heading error)
        let mut steering = self.calculate_steering(heading_error, dt, yaw_rate_dps);
        let throttle = self.calculate_throttle(distance, heading_error.abs());

        // Spin-in-place speed limit: cap steering when throttle is near zero
        if throttle < self.config.spin_throttle_threshold {
            steering = steering.clamp(
                -self.config.max_spin_steering,
                self.config.max_spin_steering,
            );
        }

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
        self.prev_heading_error = 0.0;
        self.prev_steering = 0.0;
        self.heading_filter.reset();
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

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
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

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
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

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
        assert!(
            output.steering.abs() < 0.1,
            "Steering should be ~0 when target ahead, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_target_to_left() {
        let config = SimpleNavConfig {
            max_steering_rate: 0.0, // disable slew rate for directional test
            max_spin_steering: 1.0, // disable spin cap
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(0.0, -1.0); // West
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
        assert!(
            output.steering < -0.5,
            "Steering should be negative (left) when target to left, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_target_to_right() {
        let config = SimpleNavConfig {
            max_steering_rate: 0.0, // disable slew rate for directional test
            max_spin_steering: 1.0, // disable spin cap
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(0.0, 1.0); // East
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
        assert!(
            output.steering > 0.5,
            "Steering should be positive (right) when target to right, got {}",
            output.steering
        );
    }

    #[test]
    fn test_steering_target_behind() {
        let config = SimpleNavConfig {
            max_steering_rate: 0.0, // disable slew rate for directional test
            max_spin_steering: 1.0, // disable spin cap
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);
        let (lat, lon) = make_position(0.0, 0.0);
        let target = PositionTarget::new(-1.0, 0.0); // South
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
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

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
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

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
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

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
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

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
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
            let output = controller.update(lat1, lon1, &target, heading, 0.02, None);

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

        let output = controller.update(35.6762, 139.6503, &target, heading, 0.02, None);
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
            steering_d_gain: 0.01,
            max_steering_rate: 3.0,
            throttle_heading_error_max: 60.0,
            max_spin_steering: 0.5,
            spin_throttle_threshold: 0.15,
            heading_filter_alpha: 0.5,
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
        let output = controller.update(0.0, 0.0, &target, 0.0, 0.02, None);

        assert!(
            output.distance_m > 110_000.0 && output.distance_m < 112_000.0,
            "Distance should be ~111km, got {}km",
            output.distance_m / 1000.0
        );
    }

    // ========== PD Steering Tests ==========

    #[test]
    fn test_d_term_opposes_increasing_error() {
        // When error increases, D-term should add to steering correction.
        // When error decreases, D-term should reduce steering correction.
        // Use very small D-gain to avoid clamping at +/-1.0.
        let config = SimpleNavConfig {
            steering_d_gain: 0.0001,
            max_steering_rate: 0.0,    // disable slew rate for this test
            heading_filter_alpha: 1.0, // disable heading filter for this test
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);

        // Warm up: set prev_heading_error to 10
        let target_north = PositionTarget::new(0.0009, 0.0); // ~100m north
        controller.update(0.0, 0.0, &target_north, 350.0, 0.02, None); // error ~10°

        // Call with error increasing (10 → 20): D-term adds to P-term
        let output_increasing = controller.update(0.0, 0.0, &target_north, 340.0, 0.02, None); // error ~20°

        // Reset and set prev to 30
        controller.reset();
        controller.update(0.0, 0.0, &target_north, 330.0, 0.02, None); // error ~30°

        // Call with error decreasing (30 → 20): D-term subtracts from P-term
        let output_decreasing = controller.update(0.0, 0.0, &target_north, 340.0, 0.02, None); // error ~20°

        // Same ~20° error, but increasing vs decreasing: increasing should produce more steering
        assert!(
            output_increasing.steering > output_decreasing.steering,
            "Increasing error should produce more steering than decreasing: {} vs {}",
            output_increasing.steering,
            output_decreasing.steering
        );
    }

    #[test]
    fn test_slew_rate_limits_steering_change() {
        let config = SimpleNavConfig {
            max_steering_rate: 1.0, // max 1.0/s change
            steering_d_gain: 0.0,   // disable D-term for cleaner test
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);
        let dt = 0.02; // 50Hz

        // First update with 0 heading error (steering ~0)
        let target_north = PositionTarget::new(1.0, 0.0);
        controller.update(0.0, 0.0, &target_north, 0.0, dt, None);

        // Sudden large heading error demanding full steering
        let target_west = PositionTarget::new(0.0, -1.0);
        let output = controller.update(0.0, 0.0, &target_west, 0.0, dt, None);

        // max_change = 1.0 * 0.02 = 0.02 per step from ~0
        let max_expected = 1.0 * dt;
        assert!(
            output.steering.abs() <= max_expected + 0.01,
            "Steering change should be limited by slew rate, got {}",
            output.steering
        );
    }

    #[test]
    fn test_reset_clears_derivative_state() {
        let mut controller = SimpleNavigationController::new();

        // Build up some state
        let target = PositionTarget::new(0.0, 1.0); // East
        controller.update(0.0, 0.0, &target, 0.0, 0.02, None);

        // Reset
        controller.reset();

        // After reset, prev_heading_error and prev_steering should be 0
        // First update after reset should behave like fresh controller
        let target_north = PositionTarget::new(1.0, 0.0);
        let output = controller.update(0.0, 0.0, &target_north, 0.0, 0.02, None);
        assert!(
            output.steering.abs() < 0.1,
            "After reset, steering for target ahead should be ~0, got {}",
            output.steering
        );
    }

    #[test]
    fn test_dt_zero_disables_d_term() {
        let config = SimpleNavConfig {
            steering_d_gain: 0.01,
            max_steering_rate: 0.0,
            max_spin_steering: 1.0, // disable spin cap for this test
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);

        // With dt=0, D-term should be 0 (safe fallback)
        let target = PositionTarget::new(0.0, 1.0); // East (90 degrees heading error)
        let output = controller.update(0.0, 0.0, &target, 0.0, 0.0, None);

        // Should only have P-term: ~90/90 = 1.0
        assert!(
            output.steering > 0.9,
            "With dt=0, should still have P-term steering, got {}",
            output.steering
        );
    }

    // ========== Throttle Heading Error Tests ==========

    #[test]
    fn test_throttle_zero_at_90_degree_error() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        // Target east, heading north -> 90 degree error
        let target = PositionTarget::new(0.0, 1.0); // East
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
        assert!(
            output.throttle < 0.05,
            "Throttle should be ~0 at 90 degree heading error, got {}",
            output.throttle
        );
    }

    #[test]
    fn test_throttle_full_at_zero_error() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        // Target north, heading north -> 0 error, far away
        let target = PositionTarget::new(0.0009, 0.0); // ~100m north
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
        assert!(
            (output.throttle - 1.0).abs() < 0.01,
            "Throttle should be 1.0 at 0 heading error far from target, got {}",
            output.throttle
        );
    }

    #[test]
    fn test_throttle_scales_with_heading_error() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        // Target slightly east of north (~45 degree bearing)
        let target = PositionTarget::new(0.001, 0.001);
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
        // Heading error ~45 degrees, throttle should be ~50% of distance-based throttle
        assert!(
            output.throttle > 0.2 && output.throttle < 0.8,
            "Throttle should be partially reduced at ~45 degree error, got {}",
            output.throttle
        );
    }

    #[test]
    fn test_throttle_zero_at_180_degree_error() {
        let mut controller = SimpleNavigationController::new();
        let (lat, lon) = make_position(0.0, 0.0);
        // Target south, heading north -> 180 degree error
        let target = PositionTarget::new(-0.001, 0.0); // South
        let heading = 0.0;

        let output = controller.update(lat, lon, &target, heading, 0.02, None);
        assert!(
            output.throttle < 0.01,
            "Throttle should be 0 at 180 degree heading error, got {}",
            output.throttle
        );
    }

    #[test]
    fn test_spin_cap_limits_steering_when_throttle_low() {
        let config = SimpleNavConfig {
            max_steering_rate: 0.0, // disable slew rate to isolate spin cap
            max_spin_steering: 0.3,
            spin_throttle_threshold: 0.1,
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);

        // Target east, heading north -> 90° error, throttle = 0 (at throttle_heading_error_max)
        let target = PositionTarget::new(0.0, 1.0);
        let output = controller.update(0.0, 0.0, &target, 0.0, 0.02, None);

        assert!(
            output.steering.abs() <= 0.3 + 0.001,
            "Steering should be capped to max_spin_steering when throttle < threshold, got {}",
            output.steering
        );
    }

    #[test]
    fn test_spin_cap_does_not_limit_when_throttle_sufficient() {
        let config = SimpleNavConfig {
            max_steering_rate: 0.0, // disable slew rate to isolate spin cap
            max_spin_steering: 0.3,
            spin_throttle_threshold: 0.1,
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);

        // Target slightly east, heading north -> small error, throttle near 1.0
        let target = PositionTarget::new(0.001, 0.0003); // ~17° bearing, high throttle
        let output = controller.update(0.0, 0.0, &target, 0.0, 0.02, None);

        // Throttle should be above threshold (small heading error -> high throttle)
        assert!(
            output.throttle >= 0.1,
            "Throttle should be above spin threshold, got {}",
            output.throttle
        );
        // Steering should NOT be capped
        // (heading error is small so steering won't exceed 0.3 anyway,
        // but the cap logic should not be the limiting factor)
    }

    #[test]
    fn test_spin_cap_disabled_with_max_spin_steering_one() {
        let config = SimpleNavConfig {
            max_steering_rate: 0.0,
            max_spin_steering: 1.0, // effectively disables spin cap
            spin_throttle_threshold: 0.1,
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);

        // Target east, heading north -> 90° error, throttle = 0
        let target = PositionTarget::new(0.0, 1.0);
        let output = controller.update(0.0, 0.0, &target, 0.0, 0.02, None);

        // With max_spin_steering=1.0, steering should reach full P-term output
        assert!(
            output.steering > 0.9,
            "Steering should not be capped with max_spin_steering=1.0, got {}",
            output.steering
        );
    }

    // ========== Gyroscope Yaw Rate D-Term Tests ==========

    #[test]
    fn test_d_term_uses_yaw_rate_when_provided() {
        let config = SimpleNavConfig {
            steering_d_gain: 0.05,
            max_steering_rate: 0.0,
            max_spin_steering: 1.0,
            heading_filter_alpha: 1.0,
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);

        // Target north, heading slightly west -> small positive heading error
        let target = PositionTarget::new(0.0009, 0.0); // ~100m north
                                                       // First call to establish prev_heading_error
        controller.update(0.0, 0.0, &target, 350.0, 0.02, None); // error ~10°

        // With positive yaw_rate (turning right), D-term should be negative
        // reducing the positive steering correction
        let output_with_gyro = controller.update(0.0, 0.0, &target, 350.0, 0.02, Some(30.0));

        // Reset and do the same without gyro (error stays the same => D=0 from differencing)
        controller.reset();
        controller.update(0.0, 0.0, &target, 350.0, 0.02, None);
        let output_without_gyro = controller.update(0.0, 0.0, &target, 350.0, 0.02, None);

        // Gyro D-term: -30.0 * 0.05 = -1.5, which will reduce steering
        // Without gyro, error doesn't change so D-term is 0
        assert!(
            output_with_gyro.steering < output_without_gyro.steering,
            "Positive yaw rate should reduce positive steering: with_gyro={} < without_gyro={}",
            output_with_gyro.steering,
            output_without_gyro.steering
        );
    }

    #[test]
    fn test_d_term_falls_back_to_differenced_heading_without_gyro() {
        let config = SimpleNavConfig {
            steering_d_gain: 0.001,
            max_steering_rate: 0.0,
            heading_filter_alpha: 1.0,
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);

        let target = PositionTarget::new(0.0009, 0.0);
        // First call: error ~10°
        controller.update(0.0, 0.0, &target, 350.0, 0.02, None);
        // Second call: error ~20° (increasing)
        let output = controller.update(0.0, 0.0, &target, 340.0, 0.02, None);

        // With increasing error, D-term from differencing should add to steering
        // P-term alone for ~20° error: 20/90 ≈ 0.222
        // D-term = (20-10)/0.02 * 0.001 = 0.5
        // Total should be > P-term alone
        assert!(
            output.steering > 0.22,
            "D-term from differenced heading should add to steering, got {}",
            output.steering
        );
    }

    #[test]
    fn test_d_term_gyro_meaningful_at_normal_rates() {
        let config = SimpleNavConfig {
            steering_d_gain: 0.05,
            max_steering_rate: 0.0,
            max_spin_steering: 1.0,
            heading_filter_alpha: 1.0,
            ..SimpleNavConfig::default()
        };
        let mut controller = SimpleNavigationController::with_config(config);

        let target = PositionTarget::new(0.0009, 0.0);
        // At 30 deg/s yaw rate with gain 0.05: D-term = -30 * 0.05 = -1.5
        // This should produce meaningful damping (not negligible)
        let output = controller.update(0.0, 0.0, &target, 350.0, 0.02, Some(30.0));

        // P-term for ~10° error: 10/90 ≈ 0.111
        // D-term: -30 * 0.05 = -1.5
        // PD: 0.111 - 1.5 = -1.389, clamped to -1.0
        // The gyro D-term should overwhelm the P-term at 30 deg/s
        assert!(
            output.steering < 0.0,
            "At 30 deg/s yaw rate, D-term should produce negative steering (damping), got {}",
            output.steering
        );
    }
}
