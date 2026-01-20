//! Compass Parameter Definitions
//!
//! Defines compass calibration parameters following ArduPilot standards.
//!
//! # Parameters
//!
//! - `COMPASS_OFS_X` - Compass X-axis offset
//! - `COMPASS_OFS_Y` - Compass Y-axis offset
//! - `COMPASS_OFS_Z` - Compass Z-axis offset
//!
//! # Purpose
//!
//! These parameters are required by Mission Planner to enable the compass
//! calibration UI. The actual calibration is performed by BNO086's internal
//! calibration system, but Mission Planner checks for these parameters to
//! determine if the autopilot supports compass configuration.
//!
//! # ArduPilot Compatibility
//!
//! These parameters match ArduPilot's standard compass parameters:
//! - https://ardupilot.org/rover/docs/parameters.html#compass-ofs-x-compass-offsets-in-milligauss-on-the-x-axis

use super::storage::{ParamFlags, ParamValue, ParameterStore};
use crate::platform::Result;

/// Compass calibration configuration
#[derive(Debug, Clone, Copy)]
pub struct CompassParams {
    /// X-axis offset in milligauss
    pub ofs_x: f32,
    /// Y-axis offset in milligauss
    pub ofs_y: f32,
    /// Z-axis offset in milligauss
    pub ofs_z: f32,
}

impl Default for CompassParams {
    fn default() -> Self {
        Self {
            ofs_x: 0.0,
            ofs_y: 0.0,
            ofs_z: 0.0,
        }
    }
}

impl CompassParams {
    /// Register compass parameters with default values
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to register into
    ///
    /// # Returns
    ///
    /// Ok if all parameters registered successfully
    pub fn register_defaults(store: &mut ParameterStore) -> Result<()> {
        // COMPASS_OFS_X - X-axis offset (required by Mission Planner)
        let _ = store.register("COMPASS_OFS_X", ParamValue::Float(0.0), ParamFlags::empty());

        // COMPASS_OFS_Y - Y-axis offset
        let _ = store.register("COMPASS_OFS_Y", ParamValue::Float(0.0), ParamFlags::empty());

        // COMPASS_OFS_Z - Z-axis offset
        let _ = store.register("COMPASS_OFS_Z", ParamValue::Float(0.0), ParamFlags::empty());

        Ok(())
    }

    /// Load compass configuration from parameter store
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to load from
    ///
    /// # Returns
    ///
    /// Compass configuration with values from store or defaults
    pub fn from_store(store: &ParameterStore) -> Self {
        Self {
            ofs_x: match store.get("COMPASS_OFS_X") {
                Some(ParamValue::Float(v)) => *v,
                _ => 0.0,
            },
            ofs_y: match store.get("COMPASS_OFS_Y") {
                Some(ParamValue::Float(v)) => *v,
                _ => 0.0,
            },
            ofs_z: match store.get("COMPASS_OFS_Z") {
                Some(ParamValue::Float(v)) => *v,
                _ => 0.0,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compass_defaults() {
        let params = CompassParams::default();
        assert_eq!(params.ofs_x, 0.0);
        assert_eq!(params.ofs_y, 0.0);
        assert_eq!(params.ofs_z, 0.0);
    }

    #[test]
    fn test_register_defaults() {
        let mut store = ParameterStore::default();
        CompassParams::register_defaults(&mut store).unwrap();

        assert!(store.get("COMPASS_OFS_X").is_some());
        assert!(store.get("COMPASS_OFS_Y").is_some());
        assert!(store.get("COMPASS_OFS_Z").is_some());
    }

    #[test]
    fn test_from_store() {
        let mut store = ParameterStore::default();
        CompassParams::register_defaults(&mut store).unwrap();

        // Set custom values
        store.set("COMPASS_OFS_X", ParamValue::Float(10.5)).unwrap();
        store.set("COMPASS_OFS_Y", ParamValue::Float(-5.0)).unwrap();
        store.set("COMPASS_OFS_Z", ParamValue::Float(3.2)).unwrap();

        let params = CompassParams::from_store(&store);
        assert_eq!(params.ofs_x, 10.5);
        assert_eq!(params.ofs_y, -5.0);
        assert_eq!(params.ofs_z, 3.2);
    }
}
