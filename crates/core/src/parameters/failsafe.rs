//! Failsafe Parameter Definitions
//!
//! Defines failsafe-related parameters following ArduPilot standards.
//!
//! # Parameters
//!
//! - `FS_ACTION` - Default failsafe action (**visible in GCS**)
//! - `FS_TIMEOUT` - GCS heartbeat loss timeout (seconds, **visible in GCS**)
//! - `FS_GCS_ENABLE` - Enable GCS heartbeat loss failsafe (**visible in GCS**)
//!
//! # ArduPilot Compatibility
//!
//! These parameters match ArduPilot's standard failsafe parameters:
//! - https://ardupilot.org/rover/docs/parameters.html#fs-action
//! - https://ardupilot.org/rover/docs/parameters.html#fs-timeout
//! - https://ardupilot.org/rover/docs/parameters.html#fs-gcs-enable

use super::error::ParameterError;
use super::storage::{ParamFlags, ParamValue, ParameterStore};

/// Failsafe actions (ArduPilot compatible)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailsafeAction {
    /// No action
    None = 0,
    /// Hold position
    Hold = 1,
    /// Return to launch
    RTL = 2,
    /// Disarm
    Disarm = 3,
}

/// Failsafe parameters loaded from parameter store
#[derive(Debug, Clone)]
pub struct FailsafeParams {
    /// Default failsafe action
    pub action: u8,
    /// GCS heartbeat loss timeout (seconds)
    pub timeout: f32,
    /// Enable GCS heartbeat loss failsafe
    pub gcs_enable: bool,
}

impl FailsafeParams {
    /// Register failsafe parameters with default values
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to register parameters in
    ///
    /// # Returns
    ///
    /// Ok if all parameters registered successfully
    pub fn register_defaults(store: &mut ParameterStore) -> Result<(), ParameterError> {
        // FS_ACTION - Default to Hold (1)
        store.register(
            "FS_ACTION",
            ParamValue::Int(FailsafeAction::Hold as i32),
            ParamFlags::empty(),
        )?;

        // FS_TIMEOUT - Default to 5 seconds
        store.register("FS_TIMEOUT", ParamValue::Float(5.0), ParamFlags::empty())?;

        // FS_GCS_ENABLE - Default to enabled (1)
        store.register("FS_GCS_ENABLE", ParamValue::Int(1), ParamFlags::empty())?;

        Ok(())
    }

    /// Load failsafe parameters from parameter store
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to read from
    ///
    /// # Returns
    ///
    /// Failsafe parameters with values from store or defaults
    pub fn from_store(store: &ParameterStore) -> Self {
        let action = match store.get("FS_ACTION") {
            Some(ParamValue::Int(v)) => *v as u8,
            Some(ParamValue::Float(v)) => *v as u8,
            _ => FailsafeAction::Hold as u8,
        };

        let timeout = match store.get("FS_TIMEOUT") {
            Some(ParamValue::Float(v)) => *v,
            Some(ParamValue::Int(v)) => *v as f32,
            _ => 5.0,
        };

        let gcs_enable = match store.get("FS_GCS_ENABLE") {
            Some(ParamValue::Int(v)) => *v != 0,
            Some(ParamValue::Float(v)) => *v != 0.0,
            _ => true,
        };

        Self {
            action,
            timeout,
            gcs_enable,
        }
    }

    /// Check if failsafe configuration is valid
    pub fn is_configured(&self) -> bool {
        self.timeout > 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_failsafe_params_defaults() {
        let params = FailsafeParams {
            action: FailsafeAction::Hold as u8,
            timeout: 5.0,
            gcs_enable: true,
        };

        assert!(params.is_configured());
        assert!(params.gcs_enable);
    }

    #[test]
    fn test_register_defaults() {
        let mut store = ParameterStore::new();
        FailsafeParams::register_defaults(&mut store).unwrap();

        assert!(store.get("FS_ACTION").is_some());
        assert!(store.get("FS_TIMEOUT").is_some());
        assert!(store.get("FS_GCS_ENABLE").is_some());
    }

    #[test]
    fn test_from_store_defaults() {
        let mut store = ParameterStore::new();
        FailsafeParams::register_defaults(&mut store).unwrap();

        let params = FailsafeParams::from_store(&store);
        assert_eq!(params.action, FailsafeAction::Hold as u8);
        assert!((params.timeout - 5.0).abs() < f32::EPSILON);
        assert!(params.gcs_enable);
    }

    #[test]
    fn test_from_store_custom_values() {
        let mut store = ParameterStore::new();
        FailsafeParams::register_defaults(&mut store).unwrap();

        store
            .set("FS_ACTION", ParamValue::Int(FailsafeAction::RTL as i32))
            .unwrap();
        store.set("FS_TIMEOUT", ParamValue::Float(10.0)).unwrap();
        store.set("FS_GCS_ENABLE", ParamValue::Int(0)).unwrap();

        let params = FailsafeParams::from_store(&store);
        assert_eq!(params.action, FailsafeAction::RTL as u8);
        assert!((params.timeout - 10.0).abs() < f32::EPSILON);
        assert!(!params.gcs_enable);
    }
}
