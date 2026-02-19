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
///
/// Numbering matches ArduPilot Rover FS_ACTION:
///   0 = None, 1 = RTL, 2 = Hold, 3 = Disarm (SmartRTL omitted)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailsafeAction {
    /// No action
    None = 0,
    /// Return to launch
    RTL = 1,
    /// Hold position
    Hold = 2,
    /// Disarm
    Disarm = 3,
}

/// Failsafe parameters loaded from parameter store
#[derive(Debug, Clone)]
pub struct FailsafeParams {
    /// Default failsafe action (FS_ACTION)
    pub action: u8,
    /// Persistence timeout in seconds (FS_TIMEOUT, default 1.5s)
    pub timeout: f32,
    /// GCS heartbeat detection timeout in seconds (FS_GCS_TIMEOUT, default 5.0s)
    pub gcs_timeout: f32,
    /// Enable GCS heartbeat loss failsafe (FS_GCS_ENABLE)
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
        // FS_ACTION - Default to Hold (2, ArduPilot numbering)
        store.register(
            "FS_ACTION",
            ParamValue::Int(FailsafeAction::Hold as i32),
            ParamFlags::empty(),
        )?;

        // FS_TIMEOUT - Persistence timeout (default 1.5s)
        store.register("FS_TIMEOUT", ParamValue::Float(1.5), ParamFlags::empty())?;

        // FS_GCS_TIMEOUT - GCS heartbeat detection timeout (default 5.0s)
        store.register(
            "FS_GCS_TIMEOUT",
            ParamValue::Float(5.0),
            ParamFlags::empty(),
        )?;

        // FS_GCS_ENABLE - Default to enabled (1)
        store.register("FS_GCS_ENABLE", ParamValue::Int(1), ParamFlags::empty())?;

        Ok(())
    }

    /// Load failsafe parameters from parameter store
    ///
    /// Note: The FailsafeAction enum was reordered to match ArduPilot (Hold: 1→2,
    /// RTL: 2→1). Previously stored FS_ACTION values will be interpreted with the
    /// new numbering. Pre-release only; no migration logic needed.
    pub fn from_store(store: &ParameterStore) -> Self {
        let action = match store.get("FS_ACTION") {
            Some(ParamValue::Int(v)) => *v as u8,
            Some(ParamValue::Float(v)) => *v as u8,
            _ => FailsafeAction::Hold as u8,
        };

        let timeout = match store.get("FS_TIMEOUT") {
            Some(ParamValue::Float(v)) => *v,
            Some(ParamValue::Int(v)) => *v as f32,
            _ => 1.5,
        };

        let gcs_timeout = match store.get("FS_GCS_TIMEOUT") {
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
            gcs_timeout,
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
    fn test_failsafe_action_ardupilot_numbering() {
        assert_eq!(FailsafeAction::None as u8, 0);
        assert_eq!(FailsafeAction::RTL as u8, 1);
        assert_eq!(FailsafeAction::Hold as u8, 2);
        assert_eq!(FailsafeAction::Disarm as u8, 3);
    }

    #[test]
    fn test_failsafe_params_defaults() {
        let params = FailsafeParams {
            action: FailsafeAction::Hold as u8,
            timeout: 1.5,
            gcs_timeout: 5.0,
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
        assert!(store.get("FS_GCS_TIMEOUT").is_some());
        assert!(store.get("FS_GCS_ENABLE").is_some());
    }

    #[test]
    fn test_from_store_defaults() {
        let mut store = ParameterStore::new();
        FailsafeParams::register_defaults(&mut store).unwrap();

        let params = FailsafeParams::from_store(&store);
        assert_eq!(params.action, FailsafeAction::Hold as u8);
        assert!((params.timeout - 1.5).abs() < f32::EPSILON);
        assert!((params.gcs_timeout - 5.0).abs() < f32::EPSILON);
        assert!(params.gcs_enable);
    }

    #[test]
    fn test_from_store_custom_values() {
        let mut store = ParameterStore::new();
        FailsafeParams::register_defaults(&mut store).unwrap();

        store
            .set("FS_ACTION", ParamValue::Int(FailsafeAction::RTL as i32))
            .unwrap();
        store.set("FS_TIMEOUT", ParamValue::Float(3.0)).unwrap();
        store
            .set("FS_GCS_TIMEOUT", ParamValue::Float(10.0))
            .unwrap();
        store.set("FS_GCS_ENABLE", ParamValue::Int(0)).unwrap();

        let params = FailsafeParams::from_store(&store);
        assert_eq!(params.action, FailsafeAction::RTL as u8);
        assert!((params.timeout - 3.0).abs() < f32::EPSILON);
        assert!((params.gcs_timeout - 10.0).abs() < f32::EPSILON);
        assert!(!params.gcs_enable);
    }

    #[test]
    fn test_gcs_timeout_parameter() {
        let mut store = ParameterStore::new();
        FailsafeParams::register_defaults(&mut store).unwrap();

        // Default
        let params = FailsafeParams::from_store(&store);
        assert!((params.gcs_timeout - 5.0).abs() < f32::EPSILON);

        // Custom
        store.set("FS_GCS_TIMEOUT", ParamValue::Float(8.0)).unwrap();
        let params = FailsafeParams::from_store(&store);
        assert!((params.gcs_timeout - 8.0).abs() < f32::EPSILON);
    }
}
