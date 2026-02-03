//! Navigation Controller Parameter Definitions
//!
//! Defines navigation controller parameters following ArduPilot standards.
//! Maps all `SimpleNavConfig` fields to parameter store entries for
//! runtime configuration via MAVLink GCS.
//!
//! # Parameters
//!
//! - `WP_RADIUS` - Waypoint acceptance radius in meters (**visible in GCS**)
//! - `WP_APPR_DIST` - Approach distance to start slowing (**visible in GCS**)
//! - `ATC_HDG_ERR` - Heading error for full steering deflection (**visible in GCS**)
//! - `WP_APPR_THR` - Minimum throttle during approach (**visible in GCS**)
//! - `ATC_STR_RAT_D` - D-gain for steering PD controller (**visible in GCS**)
//! - `ATC_STR_SMAX` - Maximum steering change per second (**visible in GCS**)
//! - `ATC_THR_HERR` - Heading error at which throttle reaches zero (**visible in GCS**)
//! - `ATC_SPIN_MAX` - Maximum steering when throttle near zero (**visible in GCS**)
//! - `ATC_SPIN_THR` - Throttle threshold for spin limiting (**visible in GCS**)
//! - `ATC_HDG_FILT` - EMA filter alpha for heading smoothing (**visible in GCS**)
//! - `WP_PIVOT_ANGLE` - Heading error threshold for pivot turns (**visible in GCS**)
//! - `WP_ARC_THR` - Minimum throttle during arc turns (**visible in GCS**)
//!
//! # ArduPilot Compatibility
//!
//! Standard ArduPilot parameters: `WP_RADIUS`, `ATC_STR_RAT_D`, `WP_PIVOT_ANGLE`.
//! Custom parameters follow ArduPilot naming conventions for GCS compatibility.

use super::error::ParameterError;
use super::storage::{ParamFlags, ParamValue, ParameterStore};
use crate::navigation::SimpleNavConfig;

// --- Defaults ---

const DEFAULT_WP_RADIUS: f32 = 2.0;
const DEFAULT_APPROACH_DIST: f32 = 10.0;
const DEFAULT_MAX_HEADING_ERROR: f32 = 90.0;
const DEFAULT_MIN_APPROACH_THROTTLE: f32 = 0.2;
const DEFAULT_STEERING_D_GAIN: f32 = 0.05;
const DEFAULT_MAX_STEERING_RATE: f32 = 2.0;
const DEFAULT_THROTTLE_HEADING_ERROR_MAX: f32 = 90.0;
const DEFAULT_MAX_SPIN_STEERING: f32 = 0.3;
const DEFAULT_SPIN_THROTTLE_THRESHOLD: f32 = 0.1;
const DEFAULT_HEADING_FILTER_ALPHA: f32 = 0.3;
const DEFAULT_PIVOT_TURN_ANGLE: f32 = 60.0;
const DEFAULT_ARC_TURN_MIN_THROTTLE: f32 = 0.15;

// --- Ranges ---

const MIN_WP_RADIUS: f32 = 0.5;
const MAX_WP_RADIUS: f32 = 100.0;

const MIN_APPROACH_DIST: f32 = 1.0;
const MAX_APPROACH_DIST: f32 = 200.0;

const MIN_HEADING_ERROR: f32 = 10.0;
const MAX_HEADING_ERROR: f32 = 180.0;

const MIN_THROTTLE: f32 = 0.0;
const MAX_THROTTLE: f32 = 1.0;

const MIN_D_GAIN: f32 = 0.0;
const MAX_D_GAIN: f32 = 1.0;

const MIN_STEERING_RATE: f32 = 0.0;
const MAX_STEERING_RATE: f32 = 10.0;

const MIN_PIVOT_ANGLE: f32 = 0.0;
const MAX_PIVOT_ANGLE: f32 = 180.0;

/// Navigation controller parameters loaded from parameter store
#[derive(Debug, Clone)]
pub struct NavigationParams {
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
    /// Heading error threshold for pivot turns (degrees, ArduPilot: WP_PIVOT_ANGLE)
    pub pivot_turn_angle: f32,
    /// Minimum throttle during arc turns (0.0-1.0)
    pub arc_turn_min_throttle: f32,
}

impl Default for NavigationParams {
    fn default() -> Self {
        Self {
            wp_radius: DEFAULT_WP_RADIUS,
            approach_dist: DEFAULT_APPROACH_DIST,
            max_heading_error: DEFAULT_MAX_HEADING_ERROR,
            min_approach_throttle: DEFAULT_MIN_APPROACH_THROTTLE,
            steering_d_gain: DEFAULT_STEERING_D_GAIN,
            max_steering_rate: DEFAULT_MAX_STEERING_RATE,
            throttle_heading_error_max: DEFAULT_THROTTLE_HEADING_ERROR_MAX,
            max_spin_steering: DEFAULT_MAX_SPIN_STEERING,
            spin_throttle_threshold: DEFAULT_SPIN_THROTTLE_THRESHOLD,
            heading_filter_alpha: DEFAULT_HEADING_FILTER_ALPHA,
            pivot_turn_angle: DEFAULT_PIVOT_TURN_ANGLE,
            arc_turn_min_throttle: DEFAULT_ARC_TURN_MIN_THROTTLE,
        }
    }
}

impl NavigationParams {
    /// Register navigation parameters with default values
    pub fn register_defaults(store: &mut ParameterStore) -> Result<(), ParameterError> {
        store.register(
            "WP_RADIUS",
            ParamValue::Float(DEFAULT_WP_RADIUS),
            ParamFlags::empty(),
        )?;
        store.register(
            "WP_APPR_DIST",
            ParamValue::Float(DEFAULT_APPROACH_DIST),
            ParamFlags::empty(),
        )?;
        store.register(
            "ATC_HDG_ERR",
            ParamValue::Float(DEFAULT_MAX_HEADING_ERROR),
            ParamFlags::empty(),
        )?;
        store.register(
            "WP_APPR_THR",
            ParamValue::Float(DEFAULT_MIN_APPROACH_THROTTLE),
            ParamFlags::empty(),
        )?;
        store.register(
            "ATC_STR_RAT_D",
            ParamValue::Float(DEFAULT_STEERING_D_GAIN),
            ParamFlags::empty(),
        )?;
        store.register(
            "ATC_STR_SMAX",
            ParamValue::Float(DEFAULT_MAX_STEERING_RATE),
            ParamFlags::empty(),
        )?;
        store.register(
            "ATC_THR_HERR",
            ParamValue::Float(DEFAULT_THROTTLE_HEADING_ERROR_MAX),
            ParamFlags::empty(),
        )?;
        store.register(
            "ATC_SPIN_MAX",
            ParamValue::Float(DEFAULT_MAX_SPIN_STEERING),
            ParamFlags::empty(),
        )?;
        store.register(
            "ATC_SPIN_THR",
            ParamValue::Float(DEFAULT_SPIN_THROTTLE_THRESHOLD),
            ParamFlags::empty(),
        )?;
        store.register(
            "ATC_HDG_FILT",
            ParamValue::Float(DEFAULT_HEADING_FILTER_ALPHA),
            ParamFlags::empty(),
        )?;
        store.register(
            "WP_PIVOT_ANGLE",
            ParamValue::Float(DEFAULT_PIVOT_TURN_ANGLE),
            ParamFlags::empty(),
        )?;
        store.register(
            "WP_ARC_THR",
            ParamValue::Float(DEFAULT_ARC_TURN_MIN_THROTTLE),
            ParamFlags::empty(),
        )?;

        Ok(())
    }

    /// Load navigation parameters from parameter store
    pub fn from_store(store: &ParameterStore) -> Self {
        Self {
            wp_radius: load_float(
                store,
                "WP_RADIUS",
                DEFAULT_WP_RADIUS,
                MIN_WP_RADIUS,
                MAX_WP_RADIUS,
            ),
            approach_dist: load_float(
                store,
                "WP_APPR_DIST",
                DEFAULT_APPROACH_DIST,
                MIN_APPROACH_DIST,
                MAX_APPROACH_DIST,
            ),
            max_heading_error: load_float(
                store,
                "ATC_HDG_ERR",
                DEFAULT_MAX_HEADING_ERROR,
                MIN_HEADING_ERROR,
                MAX_HEADING_ERROR,
            ),
            min_approach_throttle: load_float(
                store,
                "WP_APPR_THR",
                DEFAULT_MIN_APPROACH_THROTTLE,
                MIN_THROTTLE,
                MAX_THROTTLE,
            ),
            steering_d_gain: load_float(
                store,
                "ATC_STR_RAT_D",
                DEFAULT_STEERING_D_GAIN,
                MIN_D_GAIN,
                MAX_D_GAIN,
            ),
            max_steering_rate: load_float(
                store,
                "ATC_STR_SMAX",
                DEFAULT_MAX_STEERING_RATE,
                MIN_STEERING_RATE,
                MAX_STEERING_RATE,
            ),
            throttle_heading_error_max: load_float(
                store,
                "ATC_THR_HERR",
                DEFAULT_THROTTLE_HEADING_ERROR_MAX,
                MIN_HEADING_ERROR,
                MAX_HEADING_ERROR,
            ),
            max_spin_steering: load_float(
                store,
                "ATC_SPIN_MAX",
                DEFAULT_MAX_SPIN_STEERING,
                MIN_THROTTLE,
                MAX_THROTTLE,
            ),
            spin_throttle_threshold: load_float(
                store,
                "ATC_SPIN_THR",
                DEFAULT_SPIN_THROTTLE_THRESHOLD,
                MIN_THROTTLE,
                MAX_THROTTLE,
            ),
            heading_filter_alpha: load_float(
                store,
                "ATC_HDG_FILT",
                DEFAULT_HEADING_FILTER_ALPHA,
                MIN_THROTTLE,
                MAX_THROTTLE,
            ),
            pivot_turn_angle: load_float(
                store,
                "WP_PIVOT_ANGLE",
                DEFAULT_PIVOT_TURN_ANGLE,
                MIN_PIVOT_ANGLE,
                MAX_PIVOT_ANGLE,
            ),
            arc_turn_min_throttle: load_float(
                store,
                "WP_ARC_THR",
                DEFAULT_ARC_TURN_MIN_THROTTLE,
                MIN_THROTTLE,
                MAX_THROTTLE,
            ),
        }
    }

    /// Convert to `SimpleNavConfig` for use by `SimpleNavigationController`
    pub fn to_nav_config(&self) -> SimpleNavConfig {
        SimpleNavConfig {
            wp_radius: self.wp_radius,
            approach_dist: self.approach_dist,
            max_heading_error: self.max_heading_error,
            min_approach_throttle: self.min_approach_throttle,
            steering_d_gain: self.steering_d_gain,
            max_steering_rate: self.max_steering_rate,
            throttle_heading_error_max: self.throttle_heading_error_max,
            max_spin_steering: self.max_spin_steering,
            spin_throttle_threshold: self.spin_throttle_threshold,
            heading_filter_alpha: self.heading_filter_alpha,
            pivot_turn_angle: self.pivot_turn_angle,
            arc_turn_min_throttle: self.arc_turn_min_throttle,
        }
    }

    /// Validate navigation parameters
    pub fn is_valid(&self) -> bool {
        // Range checks
        if self.wp_radius < MIN_WP_RADIUS || self.wp_radius > MAX_WP_RADIUS {
            return false;
        }
        if self.approach_dist < MIN_APPROACH_DIST || self.approach_dist > MAX_APPROACH_DIST {
            return false;
        }
        if self.max_heading_error < MIN_HEADING_ERROR || self.max_heading_error > MAX_HEADING_ERROR
        {
            return false;
        }
        if self.min_approach_throttle < MIN_THROTTLE || self.min_approach_throttle > MAX_THROTTLE {
            return false;
        }
        if self.steering_d_gain < MIN_D_GAIN || self.steering_d_gain > MAX_D_GAIN {
            return false;
        }
        if self.max_steering_rate < MIN_STEERING_RATE || self.max_steering_rate > MAX_STEERING_RATE
        {
            return false;
        }
        if self.throttle_heading_error_max < MIN_HEADING_ERROR
            || self.throttle_heading_error_max > MAX_HEADING_ERROR
        {
            return false;
        }
        if self.max_spin_steering < MIN_THROTTLE || self.max_spin_steering > MAX_THROTTLE {
            return false;
        }
        if self.spin_throttle_threshold < MIN_THROTTLE
            || self.spin_throttle_threshold > MAX_THROTTLE
        {
            return false;
        }
        if self.heading_filter_alpha < MIN_THROTTLE || self.heading_filter_alpha > MAX_THROTTLE {
            return false;
        }
        if self.pivot_turn_angle < MIN_PIVOT_ANGLE || self.pivot_turn_angle > MAX_PIVOT_ANGLE {
            return false;
        }
        if self.arc_turn_min_throttle < MIN_THROTTLE || self.arc_turn_min_throttle > MAX_THROTTLE {
            return false;
        }

        // Consistency: approach_dist must be greater than wp_radius
        if self.approach_dist <= self.wp_radius {
            return false;
        }

        true
    }
}

/// Load a float parameter from store with clamping
fn load_float(store: &ParameterStore, name: &str, default: f32, min: f32, max: f32) -> f32 {
    match store.get(name) {
        Some(ParamValue::Float(v)) => v.clamp(min, max),
        Some(ParamValue::Int(v)) => (*v as f32).clamp(min, max),
        _ => default,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_navigation_params_defaults() {
        let params = NavigationParams::default();

        assert!((params.wp_radius - 2.0).abs() < 0.001);
        assert!((params.approach_dist - 10.0).abs() < 0.001);
        assert!((params.max_heading_error - 90.0).abs() < 0.001);
        assert!((params.min_approach_throttle - 0.2).abs() < 0.001);
        assert!((params.steering_d_gain - 0.05).abs() < 0.001);
        assert!((params.max_steering_rate - 2.0).abs() < 0.001);
        assert!((params.throttle_heading_error_max - 90.0).abs() < 0.001);
        assert!((params.max_spin_steering - 0.3).abs() < 0.001);
        assert!((params.spin_throttle_threshold - 0.1).abs() < 0.001);
        assert!((params.heading_filter_alpha - 0.3).abs() < 0.001);
        assert!((params.pivot_turn_angle - 60.0).abs() < 0.001);
        assert!((params.arc_turn_min_throttle - 0.15).abs() < 0.001);
        assert!(params.is_valid());
    }

    #[test]
    fn test_register_defaults_populates_all_12() {
        let mut store = ParameterStore::new();
        NavigationParams::register_defaults(&mut store).unwrap();

        // Verify all 12 parameters are registered
        assert!(store.get("WP_RADIUS").is_some());
        assert!(store.get("WP_APPR_DIST").is_some());
        assert!(store.get("ATC_HDG_ERR").is_some());
        assert!(store.get("WP_APPR_THR").is_some());
        assert!(store.get("ATC_STR_RAT_D").is_some());
        assert!(store.get("ATC_STR_SMAX").is_some());
        assert!(store.get("ATC_THR_HERR").is_some());
        assert!(store.get("ATC_SPIN_MAX").is_some());
        assert!(store.get("ATC_SPIN_THR").is_some());
        assert!(store.get("ATC_HDG_FILT").is_some());
        assert!(store.get("WP_PIVOT_ANGLE").is_some());
        assert!(store.get("WP_ARC_THR").is_some());
    }

    #[test]
    fn test_from_store_reads_defaults() {
        let mut store = ParameterStore::new();
        NavigationParams::register_defaults(&mut store).unwrap();

        let params = NavigationParams::from_store(&store);
        assert!((params.wp_radius - 2.0).abs() < 0.001);
        assert!((params.approach_dist - 10.0).abs() < 0.001);
        assert!((params.pivot_turn_angle - 60.0).abs() < 0.001);
        assert!((params.arc_turn_min_throttle - 0.15).abs() < 0.001);
    }

    #[test]
    fn test_from_store_reads_custom_values() {
        let mut store = ParameterStore::new();
        NavigationParams::register_defaults(&mut store).unwrap();

        store.set("WP_RADIUS", ParamValue::Float(5.0)).unwrap();
        store.set("WP_APPR_DIST", ParamValue::Float(20.0)).unwrap();
        store
            .set("WP_PIVOT_ANGLE", ParamValue::Float(45.0))
            .unwrap();
        store.set("WP_ARC_THR", ParamValue::Float(0.25)).unwrap();

        let params = NavigationParams::from_store(&store);
        assert!((params.wp_radius - 5.0).abs() < 0.001);
        assert!((params.approach_dist - 20.0).abs() < 0.001);
        assert!((params.pivot_turn_angle - 45.0).abs() < 0.001);
        assert!((params.arc_turn_min_throttle - 0.25).abs() < 0.001);
    }

    #[test]
    fn test_out_of_range_values_clamped() {
        let mut store = ParameterStore::new();
        NavigationParams::register_defaults(&mut store).unwrap();

        // WP_RADIUS below min
        store.set("WP_RADIUS", ParamValue::Float(0.1)).unwrap();
        let params = NavigationParams::from_store(&store);
        assert!((params.wp_radius - MIN_WP_RADIUS).abs() < 0.001);

        // WP_RADIUS above max
        store.set("WP_RADIUS", ParamValue::Float(200.0)).unwrap();
        let params = NavigationParams::from_store(&store);
        assert!((params.wp_radius - MAX_WP_RADIUS).abs() < 0.001);

        // WP_PIVOT_ANGLE below min
        store
            .set("WP_PIVOT_ANGLE", ParamValue::Float(-10.0))
            .unwrap();
        let params = NavigationParams::from_store(&store);
        assert!((params.pivot_turn_angle - MIN_PIVOT_ANGLE).abs() < 0.001);

        // WP_PIVOT_ANGLE above max
        store
            .set("WP_PIVOT_ANGLE", ParamValue::Float(200.0))
            .unwrap();
        let params = NavigationParams::from_store(&store);
        assert!((params.pivot_turn_angle - MAX_PIVOT_ANGLE).abs() < 0.001);

        // ATC_STR_SMAX above max
        store.set("ATC_STR_SMAX", ParamValue::Float(15.0)).unwrap();
        let params = NavigationParams::from_store(&store);
        assert!((params.max_steering_rate - MAX_STEERING_RATE).abs() < 0.001);
    }

    #[test]
    fn test_to_nav_config_produces_matching_config() {
        let params = NavigationParams {
            wp_radius: 3.0,
            approach_dist: 15.0,
            max_heading_error: 80.0,
            min_approach_throttle: 0.25,
            steering_d_gain: 0.1,
            max_steering_rate: 3.0,
            throttle_heading_error_max: 85.0,
            max_spin_steering: 0.4,
            spin_throttle_threshold: 0.15,
            heading_filter_alpha: 0.5,
            pivot_turn_angle: 45.0,
            arc_turn_min_throttle: 0.2,
        };

        let config = params.to_nav_config();
        assert!((config.wp_radius - 3.0).abs() < 0.001);
        assert!((config.approach_dist - 15.0).abs() < 0.001);
        assert!((config.max_heading_error - 80.0).abs() < 0.001);
        assert!((config.min_approach_throttle - 0.25).abs() < 0.001);
        assert!((config.steering_d_gain - 0.1).abs() < 0.001);
        assert!((config.max_steering_rate - 3.0).abs() < 0.001);
        assert!((config.throttle_heading_error_max - 85.0).abs() < 0.001);
        assert!((config.max_spin_steering - 0.4).abs() < 0.001);
        assert!((config.spin_throttle_threshold - 0.15).abs() < 0.001);
        assert!((config.heading_filter_alpha - 0.5).abs() < 0.001);
        assert!((config.pivot_turn_angle - 45.0).abs() < 0.001);
        assert!((config.arc_turn_min_throttle - 0.2).abs() < 0.001);
    }

    #[test]
    fn test_is_valid_accepts_valid_params() {
        let params = NavigationParams::default();
        assert!(params.is_valid());

        // Custom valid params
        let params = NavigationParams {
            wp_radius: 5.0,
            approach_dist: 20.0,
            max_heading_error: 45.0,
            min_approach_throttle: 0.3,
            steering_d_gain: 0.1,
            max_steering_rate: 5.0,
            throttle_heading_error_max: 90.0,
            max_spin_steering: 0.5,
            spin_throttle_threshold: 0.2,
            heading_filter_alpha: 0.5,
            pivot_turn_angle: 45.0,
            arc_turn_min_throttle: 0.2,
        };
        assert!(params.is_valid());
    }

    #[test]
    fn test_is_valid_rejects_invalid_params() {
        // approach_dist <= wp_radius
        let params = NavigationParams {
            wp_radius: 10.0,
            approach_dist: 5.0,
            ..NavigationParams::default()
        };
        assert!(!params.is_valid());

        // approach_dist == wp_radius
        let params = NavigationParams {
            wp_radius: 5.0,
            approach_dist: 5.0,
            ..NavigationParams::default()
        };
        assert!(!params.is_valid());

        // wp_radius out of range
        let params = NavigationParams {
            wp_radius: 0.1,
            ..NavigationParams::default()
        };
        assert!(!params.is_valid());

        // pivot_turn_angle out of range
        let params = NavigationParams {
            pivot_turn_angle: -1.0,
            ..NavigationParams::default()
        };
        assert!(!params.is_valid());

        // arc_turn_min_throttle out of range
        let params = NavigationParams {
            arc_turn_min_throttle: 1.5,
            ..NavigationParams::default()
        };
        assert!(!params.is_valid());
    }
}
