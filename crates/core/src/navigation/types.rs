//! Navigation type definitions
//!
//! This module contains core types used by the navigation subsystem:
//! - `PositionTarget`: Target position for navigation
//! - `NavigationOutput`: Output from navigation controller
//! - `SimpleNavConfig`: Configuration for simple navigation controller

/// Target position for navigation
#[derive(Clone, Copy, Debug, Default)]
pub struct PositionTarget {
    /// Latitude in degrees (-90 to +90)
    pub latitude: f32,
    /// Longitude in degrees (-180 to +180)
    pub longitude: f32,
    /// Optional altitude in meters (not used in Phase 1)
    pub altitude: Option<f32>,
}

impl PositionTarget {
    /// Create a new position target
    pub fn new(latitude: f32, longitude: f32) -> Self {
        Self {
            latitude,
            longitude,
            altitude: None,
        }
    }

    /// Create a new position target with altitude
    pub fn with_altitude(latitude: f32, longitude: f32, altitude: f32) -> Self {
        Self {
            latitude,
            longitude,
            altitude: Some(altitude),
        }
    }
}

/// Output from navigation controller
#[derive(Clone, Copy, Debug, Default)]
pub struct NavigationOutput {
    /// Steering command: -1.0 (full left) to +1.0 (full right)
    pub steering: f32,
    /// Throttle command: 0.0 (stop) to 1.0 (full)
    pub throttle: f32,
    /// Distance to target in meters
    pub distance_m: f32,
    /// Bearing to target in degrees (0-360, true north)
    pub bearing_deg: f32,
    /// Heading error in degrees (-180 to +180)
    pub heading_error_deg: f32,
    /// True if vehicle is within WP_RADIUS of target
    pub at_target: bool,
}

/// Configuration for simple navigation controller
#[derive(Clone, Debug)]
pub struct SimpleNavConfig {
    /// Waypoint acceptance radius in meters (ArduPilot: WP_RADIUS)
    pub wp_radius: f32,
    /// Approach distance to start slowing (meters)
    pub approach_dist: f32,
    /// Heading error for full steering deflection (degrees)
    pub max_heading_error: f32,
    /// Minimum throttle during approach (0.0-1.0)
    pub min_approach_throttle: f32,
    /// D-gain for steering PD controller (ArduPilot: ATC_STR_RAT_D)
    pub steering_d_gain: f32,
    /// Maximum steering change per second (slew rate, 0 = unlimited)
    pub max_steering_rate: f32,
    /// Heading error (degrees) at which throttle reaches zero
    pub throttle_heading_error_max: f32,
    /// Maximum steering magnitude when throttle is near zero (0.0-1.0)
    pub max_spin_steering: f32,
    /// Throttle threshold below which spin-in-place limiting applies
    pub spin_throttle_threshold: f32,
    /// EMA filter alpha for heading smoothing (0.0 = max smoothing, 1.0 = no filter)
    pub heading_filter_alpha: f32,
}

impl Default for SimpleNavConfig {
    fn default() -> Self {
        Self {
            wp_radius: 2.0,
            approach_dist: 10.0,
            max_heading_error: 90.0,
            min_approach_throttle: 0.2,
            steering_d_gain: 0.05,
            max_steering_rate: 2.0,
            throttle_heading_error_max: 90.0,
            max_spin_steering: 0.3,
            spin_throttle_threshold: 0.1,
            heading_filter_alpha: 0.3,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_position_target_new() {
        let target = PositionTarget::new(35.6762, 139.6503);
        assert!((target.latitude - 35.6762).abs() < 0.0001);
        assert!((target.longitude - 139.6503).abs() < 0.0001);
        assert!(target.altitude.is_none());
    }

    #[test]
    fn test_position_target_with_altitude() {
        let target = PositionTarget::with_altitude(35.6762, 139.6503, 100.0);
        assert!((target.latitude - 35.6762).abs() < 0.0001);
        assert!((target.longitude - 139.6503).abs() < 0.0001);
        assert!((target.altitude.unwrap() - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_navigation_output_default() {
        let output = NavigationOutput::default();
        assert!((output.steering - 0.0).abs() < 0.001);
        assert!((output.throttle - 0.0).abs() < 0.001);
        assert!((output.distance_m - 0.0).abs() < 0.001);
        assert!(!output.at_target);
    }

    #[test]
    fn test_simple_nav_config_default() {
        let config = SimpleNavConfig::default();
        assert!((config.wp_radius - 2.0).abs() < 0.001);
        assert!((config.approach_dist - 10.0).abs() < 0.001);
        assert!((config.max_heading_error - 90.0).abs() < 0.001);
        assert!((config.min_approach_throttle - 0.2).abs() < 0.001);
        assert!((config.steering_d_gain - 0.05).abs() < 0.001);
        assert!((config.max_steering_rate - 2.0).abs() < 0.001);
        assert!((config.throttle_heading_error_max - 90.0).abs() < 0.001);
        assert!((config.max_spin_steering - 0.3).abs() < 0.001);
        assert!((config.spin_throttle_threshold - 0.1).abs() < 0.001);
        assert!((config.heading_filter_alpha - 0.3).abs() < 0.001);
    }
}
