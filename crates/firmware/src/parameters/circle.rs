//! Circle Mode Parameter Definitions
//!
//! Defines circle mode parameters following ArduPilot standards.
//!
//! # Parameters
//!
//! - `CIRC_RADIUS` - Circle radius in meters (**visible in GCS**)
//! - `CIRC_SPEED` - Circle target speed in m/s (**visible in GCS**)
//! - `CIRC_DIR` - Circle direction (0=CW, 1=CCW) (**visible in GCS**)
//!
//! # ArduPilot Compatibility
//!
//! These parameters match ArduPilot Rover's standard circle mode parameters:
//! - https://ardupilot.org/rover/docs/circle-mode.html
//! - https://ardupilot.org/rover/docs/parameters.html

use super::storage::{ParamFlags, ParamValue, ParameterStore};
use crate::platform::Result;
use crate::rover::mode::circle::CircleDirection;

/// Default circle radius in meters
const DEFAULT_RADIUS: f32 = 20.0;

/// Default circle speed in m/s
const DEFAULT_SPEED: f32 = 2.0;

/// Default circle direction (0 = CW)
const DEFAULT_DIR: i32 = 0;

/// Circle mode parameters loaded from parameter store
#[derive(Debug, Clone)]
pub struct CircleParams {
    /// Circle radius in meters (0 = stationary)
    pub radius: f32,
    /// Target speed in m/s
    pub speed: f32,
    /// Orbit direction (0=CW, 1=CCW)
    pub direction: CircleDirection,
}

impl Default for CircleParams {
    fn default() -> Self {
        Self {
            radius: DEFAULT_RADIUS,
            speed: DEFAULT_SPEED,
            direction: CircleDirection::Clockwise,
        }
    }
}

impl CircleParams {
    /// Register circle parameters with default values
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to register parameters in
    ///
    /// # Returns
    ///
    /// Ok if all parameters registered successfully
    pub fn register_defaults(store: &mut ParameterStore) -> Result<()> {
        // CIRC_RADIUS - Default to 20.0 meters
        store.register(
            "CIRC_RADIUS",
            ParamValue::Float(DEFAULT_RADIUS),
            ParamFlags::empty(),
        )?;

        // CIRC_SPEED - Default to 2.0 m/s
        store.register(
            "CIRC_SPEED",
            ParamValue::Float(DEFAULT_SPEED),
            ParamFlags::empty(),
        )?;

        // CIRC_DIR - Default to 0 (Clockwise)
        store.register(
            "CIRC_DIR",
            ParamValue::Int(DEFAULT_DIR),
            ParamFlags::empty(),
        )?;

        Ok(())
    }

    /// Load circle parameters from parameter store
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to read from
    ///
    /// # Returns
    ///
    /// Circle parameters with values from store or defaults
    pub fn from_store(store: &ParameterStore) -> Self {
        let radius = match store.get("CIRC_RADIUS") {
            Some(ParamValue::Float(v)) => *v,
            Some(ParamValue::Int(v)) => *v as f32,
            _ => DEFAULT_RADIUS,
        };

        let speed = match store.get("CIRC_SPEED") {
            Some(ParamValue::Float(v)) => *v,
            Some(ParamValue::Int(v)) => *v as f32,
            _ => DEFAULT_SPEED,
        };

        let direction = match store.get("CIRC_DIR") {
            Some(ParamValue::Int(v)) => CircleDirection::from(*v as i8),
            Some(ParamValue::Float(v)) => CircleDirection::from(*v as i8),
            _ => CircleDirection::Clockwise,
        };

        Self {
            radius,
            speed,
            direction,
        }
    }

    /// Validate circle parameters
    ///
    /// # Returns
    ///
    /// true if parameters are valid
    pub fn is_valid(&self) -> bool {
        // Radius must be non-negative (0 = stationary mode)
        if self.radius < 0.0 {
            return false;
        }

        // Radius should not exceed 1000m (reasonable limit)
        if self.radius > 1000.0 {
            return false;
        }

        // Speed must be non-negative
        if self.speed < 0.0 {
            return false;
        }

        // Speed should not exceed 10 m/s (reasonable limit for rover)
        if self.speed > 10.0 {
            return false;
        }

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_circle_params_defaults() {
        let params = CircleParams::default();

        assert!((params.radius - 20.0).abs() < 0.001);
        assert!((params.speed - 2.0).abs() < 0.001);
        assert_eq!(params.direction, CircleDirection::Clockwise);
        assert!(params.is_valid());
    }

    #[test]
    fn test_circle_params_from_store() {
        let mut store = ParameterStore::new();
        CircleParams::register_defaults(&mut store).unwrap();

        let params = CircleParams::from_store(&store);
        assert!((params.radius - 20.0).abs() < 0.001);
        assert!((params.speed - 2.0).abs() < 0.001);
        assert_eq!(params.direction, CircleDirection::Clockwise);
    }

    #[test]
    fn test_circle_params_from_store_custom() {
        let mut store = ParameterStore::new();
        CircleParams::register_defaults(&mut store).unwrap();

        // Override with custom values
        store.set("CIRC_RADIUS", ParamValue::Float(50.0)).unwrap();
        store.set("CIRC_SPEED", ParamValue::Float(3.0)).unwrap();
        store.set("CIRC_DIR", ParamValue::Int(1)).unwrap();

        let params = CircleParams::from_store(&store);
        assert!((params.radius - 50.0).abs() < 0.001);
        assert!((params.speed - 3.0).abs() < 0.001);
        assert_eq!(params.direction, CircleDirection::CounterClockwise);
    }

    #[test]
    fn test_circle_params_validation() {
        // Valid params
        let params = CircleParams {
            radius: 20.0,
            speed: 2.0,
            direction: CircleDirection::Clockwise,
        };
        assert!(params.is_valid());

        // Stationary mode (radius = 0) is valid
        let params = CircleParams {
            radius: 0.0,
            speed: 0.0,
            direction: CircleDirection::Clockwise,
        };
        assert!(params.is_valid());

        // Negative radius is invalid
        let params = CircleParams {
            radius: -1.0,
            speed: 2.0,
            direction: CircleDirection::Clockwise,
        };
        assert!(!params.is_valid());

        // Radius too large is invalid
        let params = CircleParams {
            radius: 1001.0,
            speed: 2.0,
            direction: CircleDirection::Clockwise,
        };
        assert!(!params.is_valid());

        // Negative speed is invalid
        let params = CircleParams {
            radius: 20.0,
            speed: -1.0,
            direction: CircleDirection::Clockwise,
        };
        assert!(!params.is_valid());

        // Speed too high is invalid
        let params = CircleParams {
            radius: 20.0,
            speed: 11.0,
            direction: CircleDirection::Clockwise,
        };
        assert!(!params.is_valid());
    }
}
