//! Fence Parameter Definitions
//!
//! Defines geofence-related parameters following ArduPilot standards.
//!
//! # Parameters
//!
//! - `FENCE_AUTOENABLE` - Auto-enable fence on arm (**visible in GCS**)
//! - `FENCE_ACTION` - Action when fence breached (**visible in GCS**)
//!
//! # ArduPilot Compatibility
//!
//! These parameters match ArduPilot's standard fence parameters:
//! - https://ardupilot.org/rover/docs/parameters.html#fence-autoenable
//! - https://ardupilot.org/rover/docs/parameters.html#fence-action

use super::storage::{ParamFlags, ParamValue, ParameterStore};
use crate::platform::Result;

/// Fence auto-enable modes (ArduPilot compatible)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FenceAutoEnable {
    /// Disabled
    Disabled = 0,
    /// Enable on arm
    EnableOnArm = 1,
    /// Enable on arm and fly
    EnableOnArmAndFly = 2,
}

/// Fence breach actions (ArduPilot compatible)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FenceAction {
    /// Report only
    ReportOnly = 0,
    /// Hold position
    Hold = 1,
    /// Return to launch
    RTL = 2,
    /// Disarm
    Disarm = 3,
}

/// Fence parameters loaded from parameter store
#[derive(Debug, Clone)]
pub struct FenceParams {
    /// Auto-enable fence on arm
    pub auto_enable: u8,
    /// Action when fence breached
    pub action: u8,
}

impl FenceParams {
    /// Register fence parameters with default values
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to register parameters in
    ///
    /// # Returns
    ///
    /// Ok if all parameters registered successfully
    pub fn register_defaults(store: &mut ParameterStore) -> Result<()> {
        // FENCE_AUTOENABLE - Default to EnableOnArm (1)
        store.register(
            "FENCE_AUTOENABLE",
            ParamValue::Int(FenceAutoEnable::EnableOnArm as i32),
            ParamFlags::empty(),
        )?;

        // FENCE_ACTION - Default to RTL (2)
        store.register(
            "FENCE_ACTION",
            ParamValue::Int(FenceAction::RTL as i32),
            ParamFlags::empty(),
        )?;

        Ok(())
    }

    /// Load fence parameters from parameter store
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to read from
    ///
    /// # Returns
    ///
    /// Fence parameters with values from store or defaults
    pub fn from_store(store: &ParameterStore) -> Self {
        let auto_enable = match store.get("FENCE_AUTOENABLE") {
            Some(ParamValue::Int(v)) => *v as u8,
            Some(ParamValue::Float(v)) => *v as u8,
            _ => FenceAutoEnable::EnableOnArm as u8,
        };

        let action = match store.get("FENCE_ACTION") {
            Some(ParamValue::Int(v)) => *v as u8,
            Some(ParamValue::Float(v)) => *v as u8,
            _ => FenceAction::RTL as u8,
        };

        Self {
            auto_enable,
            action,
        }
    }

    /// Check if fence should be auto-enabled
    pub fn should_auto_enable(&self) -> bool {
        self.auto_enable != FenceAutoEnable::Disabled as u8
    }

    /// Check if fence configuration is valid
    pub fn is_configured(&self) -> bool {
        true // Always valid
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fence_params_defaults() {
        let params = FenceParams {
            auto_enable: FenceAutoEnable::EnableOnArm as u8,
            action: FenceAction::RTL as u8,
        };

        assert!(params.should_auto_enable());
        assert!(params.is_configured());
    }
}
