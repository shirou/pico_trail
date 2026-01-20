//! Battery Parameter Definitions
//!
//! Defines battery monitoring parameters following ArduPilot standards.
//!
//! # Parameters
//!
//! - `BATT_ARM_VOLT` - Minimum voltage to arm (V, **visible in GCS**)
//! - `BATT_CRT_VOLT` - Critical voltage for failsafe (V, **visible in GCS**)
//! - `BATT_FS_CRT_ACT` - Action when critical threshold reached (**visible in GCS**)
//! - `BATT_VOLT_MULT` - Voltage multiplier for ADC conversion (default 3.95, **visible in GCS**)
//!
//! # ArduPilot Compatibility
//!
//! These parameters match ArduPilot's standard battery parameters:
//! - https://ardupilot.org/rover/docs/parameters.html#batt-arm-volt
//! - https://ardupilot.org/rover/docs/parameters.html#batt-crt-volt
//! - https://ardupilot.org/rover/docs/parameters.html#batt-fs-crt-act
//! - https://ardupilot.org/rover/docs/parameters.html#batt-volt-mult

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
    /// Voltage multiplier for ADC conversion (voltage divider coefficient)
    pub volt_mult: f32,
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
        // BATT_ARM_VOLT - Default to 5.0V (minimum voltage to arm)
        store.register("BATT_ARM_VOLT", ParamValue::Float(5.0), ParamFlags::empty())?;

        // BATT_CRT_VOLT - Default to 3.0V (critical voltage for failsafe)
        store.register("BATT_CRT_VOLT", ParamValue::Float(3.0), ParamFlags::empty())?;

        // BATT_FS_CRT_ACT - Default to Land (1)
        store.register(
            "BATT_FS_CRT_ACT",
            ParamValue::Int(BatteryFailsafeAction::Land as i32),
            ParamFlags::empty(),
        )?;

        // BATT_VOLT_MULT - Default to 3.95 (Freenove voltage divider coefficient)
        store.register(
            "BATT_VOLT_MULT",
            ParamValue::Float(3.95),
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
            _ => 5.0,
        };

        let critical_voltage = match store.get("BATT_CRT_VOLT") {
            Some(ParamValue::Float(v)) => *v,
            Some(ParamValue::Int(v)) => *v as f32,
            _ => 3.0,
        };

        let critical_action = match store.get("BATT_FS_CRT_ACT") {
            Some(ParamValue::Int(v)) => *v as u8,
            Some(ParamValue::Float(v)) => *v as u8,
            _ => BatteryFailsafeAction::Land as u8,
        };

        let volt_mult = match store.get("BATT_VOLT_MULT") {
            Some(ParamValue::Float(v)) => *v,
            Some(ParamValue::Int(v)) => *v as f32,
            _ => 3.95,
        };

        Self {
            arm_voltage,
            critical_voltage,
            critical_action,
            volt_mult,
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
            arm_voltage: 5.0,
            critical_voltage: 3.0,
            critical_action: BatteryFailsafeAction::Land as u8,
            volt_mult: 3.95,
        };

        assert!(params.is_configured());
        assert!(params.arm_voltage > params.critical_voltage);
        assert_eq!(params.volt_mult, 3.95);
    }
}
