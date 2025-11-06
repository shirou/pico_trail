//! Battery Parameter Definitions
//!
//! Defines battery monitoring parameters following ArduPilot standards.
//!
//! # Parameters
//!
//! - `BATT_ARM_VOLT` - Minimum voltage to arm (V, **visible in GCS**)
//! - `BATT_CRT_VOLT` - Critical voltage for failsafe (V, **visible in GCS**)
//! - `BATT_FS_CRT_ACT` - Action when critical threshold reached (**visible in GCS**)
//!
//! # ArduPilot Compatibility
//!
//! These parameters match ArduPilot's standard battery parameters:
//! - https://ardupilot.org/rover/docs/parameters.html#batt-arm-volt
//! - https://ardupilot.org/rover/docs/parameters.html#batt-crt-volt
//! - https://ardupilot.org/rover/docs/parameters.html#batt-fs-crt-act

use super::storage::{ParamFlags, ParamValue, ParameterStore};
use crate::platform::Result;

/// Battery failsafe actions (ArduPilot compatible)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryFailsafeAction {
    /// No action
    None = 0,
    /// Land immediately
    Land = 1,
    /// Return to launch
    RTL = 2,
    /// Disarm
    Disarm = 3,
}

/// Battery parameters loaded from parameter store
#[derive(Debug, Clone)]
pub struct BatteryParams {
    /// Minimum voltage to arm (V)
    pub arm_voltage: f32,
    /// Critical voltage for failsafe (V)
    pub critical_voltage: f32,
    /// Action when critical threshold reached
    pub critical_action: u8,
}

impl BatteryParams {
    /// Register battery parameters with default values
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to register parameters in
    ///
    /// # Returns
    ///
    /// Ok if all parameters registered successfully
    pub fn register_defaults(store: &mut ParameterStore) -> Result<()> {
        // BATT_ARM_VOLT - Default to 10.5V (3S LiPo minimum)
        store.register(
            "BATT_ARM_VOLT",
            ParamValue::Float(10.5),
            ParamFlags::empty(),
        )?;

        // BATT_CRT_VOLT - Default to 10.0V (3S LiPo critical)
        store.register(
            "BATT_CRT_VOLT",
            ParamValue::Float(10.0),
            ParamFlags::empty(),
        )?;

        // BATT_FS_CRT_ACT - Default to Land (1)
        store.register(
            "BATT_FS_CRT_ACT",
            ParamValue::Int(BatteryFailsafeAction::Land as i32),
            ParamFlags::empty(),
        )?;

        Ok(())
    }

    /// Load battery parameters from parameter store
    ///
    /// # Arguments
    ///
    /// * `store` - Parameter store to read from
    ///
    /// # Returns
    ///
    /// Battery parameters with values from store or defaults
    pub fn from_store(store: &ParameterStore) -> Self {
        let arm_voltage = match store.get("BATT_ARM_VOLT") {
            Some(ParamValue::Float(v)) => *v,
            Some(ParamValue::Int(v)) => *v as f32,
            _ => 10.5,
        };

        let critical_voltage = match store.get("BATT_CRT_VOLT") {
            Some(ParamValue::Float(v)) => *v,
            Some(ParamValue::Int(v)) => *v as f32,
            _ => 10.0,
        };

        let critical_action = match store.get("BATT_FS_CRT_ACT") {
            Some(ParamValue::Int(v)) => *v as u8,
            Some(ParamValue::Float(v)) => *v as u8,
            _ => BatteryFailsafeAction::Land as u8,
        };

        Self {
            arm_voltage,
            critical_voltage,
            critical_action,
        }
    }

    /// Check if battery configuration is valid
    pub fn is_configured(&self) -> bool {
        self.arm_voltage > 0.0 && self.critical_voltage > 0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_battery_params_defaults() {
        let params = BatteryParams {
            arm_voltage: 10.5,
            critical_voltage: 10.0,
            critical_action: BatteryFailsafeAction::Land as u8,
        };

        assert!(params.is_configured());
        assert!(params.arm_voltage > params.critical_voltage);
    }
}
