//! Loiter Mode Parameter Definitions
//!
//! Defines loiter mode parameters following ArduPilot standards.
//!
//! # Parameters
//!
//! - `LOIT_TYPE` - Loiter behavior type (0=stop, 1=active hold) (**visible in GCS**)
//! - `LOIT_RADIUS` - Acceptable drift radius in meters (**visible in GCS**)
//!
//! # ArduPilot Compatibility
//!
//! These parameters match ArduPilot Rover's standard loiter mode parameters:
//! - https://ardupilot.org/rover/docs/loiter-mode.html
//! - https://ardupilot.org/rover/docs/parameters.html

use super::error::ParameterError;
use super::storage::{ParamFlags, ParamValue, ParameterStore};

/// Default loiter type (0 = stop motors)
const DEFAULT_TYPE: i32 = 0;

/// Default loiter radius in meters
const DEFAULT_RADIUS: f32 = 2.0;

/// Minimum loiter radius in meters
const MIN_RADIUS: f32 = 0.5;

/// Maximum loiter radius in meters
const MAX_RADIUS: f32 = 100.0;

/// Loiter mode parameters loaded from parameter store
#[derive(Debug, Clone)]
pub struct LoiterParams {
    /// Loiter behavior type (0=stop, 1=active hold)
    pub loit_type: u8,
    /// Acceptable drift radius in meters before correction
    pub loit_radius: f32,
}

impl Default for LoiterParams {
    fn default() -> Self {
        Self {
            loit_type: DEFAULT_TYPE as u8,
            loit_radius: DEFAULT_RADIUS,
        }
    }
}

impl LoiterParams {
    /// Register loiter parameters with default values
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to register parameters in
    ///
    /// # Returns
    ///
    /// Ok if all parameters registered successfully
    pub fn register_defaults(store: &mut ParameterStore) -> Result<(), ParameterError> {
        // LOIT_TYPE - Default to 0 (stop motors)
        store.register(
            "LOIT_TYPE",
            ParamValue::Int(DEFAULT_TYPE),
            ParamFlags::empty(),
        )?;

        // LOIT_RADIUS - Default to 2.0 meters
        store.register(
            "LOIT_RADIUS",
            ParamValue::Float(DEFAULT_RADIUS),
            ParamFlags::empty(),
        )?;

        Ok(())
    }

    /// Load loiter parameters from parameter store
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to read from
    ///
    /// # Returns
    ///
    /// Loiter parameters with values from store or defaults
    pub fn from_store(store: &ParameterStore) -> Self {
        let loit_type = match store.get("LOIT_TYPE") {
            Some(ParamValue::Int(v)) => (*v as u8).min(1),
            Some(ParamValue::Float(v)) => (*v as u8).min(1),
            _ => DEFAULT_TYPE as u8,
        };

        let loit_radius = match store.get("LOIT_RADIUS") {
            Some(ParamValue::Float(v)) => v.clamp(MIN_RADIUS, MAX_RADIUS),
            Some(ParamValue::Int(v)) => (*v as f32).clamp(MIN_RADIUS, MAX_RADIUS),
            _ => DEFAULT_RADIUS,
        };

        Self {
            loit_type,
            loit_radius,
        }
    }

    /// Validate loiter parameters
    ///
    /// # Returns
    ///
    /// true if parameters are valid
    pub fn is_valid(&self) -> bool {
        // Type must be 0 or 1
        if self.loit_type > 1 {
            return false;
        }

        // Radius must be within range
        if self.loit_radius < MIN_RADIUS || self.loit_radius > MAX_RADIUS {
            return false;
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_loiter_params_defaults() {
        let params = LoiterParams::default();

        assert_eq!(params.loit_type, 0);
        assert!((params.loit_radius - 2.0).abs() < 0.001);
        assert!(params.is_valid());
    }

    #[test]
    fn test_loiter_params_from_store() {
        let mut store = ParameterStore::new();
        LoiterParams::register_defaults(&mut store).unwrap();

        let params = LoiterParams::from_store(&store);
        assert_eq!(params.loit_type, 0);
        assert!((params.loit_radius - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_loiter_params_from_store_custom() {
        let mut store = ParameterStore::new();
        LoiterParams::register_defaults(&mut store).unwrap();

        // Override with custom values
        store.set("LOIT_TYPE", ParamValue::Int(1)).unwrap();
        store.set("LOIT_RADIUS", ParamValue::Float(5.0)).unwrap();

        let params = LoiterParams::from_store(&store);
        assert_eq!(params.loit_type, 1);
        assert!((params.loit_radius - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_loiter_params_clamp_type() {
        let mut store = ParameterStore::new();
        LoiterParams::register_defaults(&mut store).unwrap();

        // Invalid type (> 1) should be clamped to 1
        store.set("LOIT_TYPE", ParamValue::Int(5)).unwrap();

        let params = LoiterParams::from_store(&store);
        assert_eq!(params.loit_type, 1);
    }

    #[test]
    fn test_loiter_params_clamp_radius() {
        let mut store = ParameterStore::new();
        LoiterParams::register_defaults(&mut store).unwrap();

        // Radius too small should be clamped to MIN_RADIUS
        store.set("LOIT_RADIUS", ParamValue::Float(0.1)).unwrap();
        let params = LoiterParams::from_store(&store);
        assert!((params.loit_radius - MIN_RADIUS).abs() < 0.001);

        // Radius too large should be clamped to MAX_RADIUS
        store.set("LOIT_RADIUS", ParamValue::Float(200.0)).unwrap();
        let params = LoiterParams::from_store(&store);
        assert!((params.loit_radius - MAX_RADIUS).abs() < 0.001);
    }

    #[test]
    fn test_loiter_params_validation() {
        // Valid params - Type 0
        let params = LoiterParams {
            loit_type: 0,
            loit_radius: 2.0,
        };
        assert!(params.is_valid());

        // Valid params - Type 1
        let params = LoiterParams {
            loit_type: 1,
            loit_radius: 5.0,
        };
        assert!(params.is_valid());

        // Invalid type
        let params = LoiterParams {
            loit_type: 2,
            loit_radius: 2.0,
        };
        assert!(!params.is_valid());

        // Radius too small
        let params = LoiterParams {
            loit_type: 0,
            loit_radius: 0.3,
        };
        assert!(!params.is_valid());

        // Radius too large
        let params = LoiterParams {
            loit_type: 0,
            loit_radius: 150.0,
        };
        assert!(!params.is_valid());
    }
}
