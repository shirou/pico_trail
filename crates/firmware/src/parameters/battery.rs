//! Battery Parameter Definitions
//!
//! Defines battery monitoring parameters following ArduPilot standards.
//!
//! # Parameters
//!
//! - `BATT_ARM_VOLT` - Minimum voltage to arm (V, **visible in GCS**)
//! - `BATT_LOW_VOLT` - Low voltage warning threshold (V, 0.0 = disabled, **visible in GCS**)
//! - `BATT_CRT_VOLT` - Critical voltage for failsafe (V, **visible in GCS**)
//! - `BATT_FS_LOW_ACT` - Action when low threshold reached (**visible in GCS**)
//! - `BATT_FS_CRT_ACT` - Action when critical threshold reached (**visible in GCS**)
//! - `BATT_VOLT_MULT` - Voltage multiplier for ADC conversion (default 3.95, **visible in GCS**)
//!
//! # ArduPilot Compatibility
//!
//! These parameters match ArduPilot's standard battery parameters:
//! - https://ardupilot.org/rover/docs/parameters.html#batt-arm-volt
//! - https://ardupilot.org/rover/docs/parameters.html#batt-low-volt
//! - https://ardupilot.org/rover/docs/parameters.html#batt-crt-volt
//! - https://ardupilot.org/rover/docs/parameters.html#batt-fs-low-act
//! - https://ardupilot.org/rover/docs/parameters.html#batt-fs-crt-act
//! - https://ardupilot.org/rover/docs/parameters.html#batt-volt-mult

use super::storage::{ParamFlags, ParamValue, ParameterStore};
use crate::platform::Result;

/// Battery failsafe actions (ArduPilot compatible)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryFailsafeAction {
    /// No action
    None = 0,
    /// Hold position (stop motors)
    Hold = 1,
    /// Return to launch
    RTL = 2,
    /// Disarm
    Disarm = 3,
}

impl BatteryFailsafeAction {
    /// Convert from u8 value to BatteryFailsafeAction
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => Self::None,
            1 => Self::Hold,
            2 => Self::RTL,
            3 => Self::Disarm,
            _ => Self::Hold, // Default to Hold for unknown values
        }
    }
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
    /// Low voltage warning threshold (V, 0.0 = disabled)
    pub low_voltage: f32,
    /// Action when low threshold reached
    pub low_action: u8,
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

        // BATT_FS_CRT_ACT - Default to Hold (1)
        store.register(
            "BATT_FS_CRT_ACT",
            ParamValue::Int(BatteryFailsafeAction::Hold as i32),
            ParamFlags::empty(),
        )?;

        // BATT_LOW_VOLT - Default to 0.0V (disabled)
        store.register("BATT_LOW_VOLT", ParamValue::Float(0.0), ParamFlags::empty())?;

        // BATT_FS_LOW_ACT - Default to None (0)
        store.register(
            "BATT_FS_LOW_ACT",
            ParamValue::Int(BatteryFailsafeAction::None as i32),
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
            _ => BatteryFailsafeAction::Hold as u8,
        };

        let low_voltage = match store.get("BATT_LOW_VOLT") {
            Some(ParamValue::Float(v)) => *v,
            Some(ParamValue::Int(v)) => *v as f32,
            _ => 0.0,
        };

        let low_action = match store.get("BATT_FS_LOW_ACT") {
            Some(ParamValue::Int(v)) => *v as u8,
            Some(ParamValue::Float(v)) => *v as u8,
            _ => BatteryFailsafeAction::None as u8,
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
            low_voltage,
            low_action,
            volt_mult,
        }
    }

    /// Check if battery configuration is valid
    ///
    /// Returns true if arm and critical voltages are set, and if low voltage
    /// is enabled, it must be greater than critical voltage.
    pub fn is_configured(&self) -> bool {
        let basic = self.arm_voltage > 0.0 && self.critical_voltage > 0.0;
        let low_valid = self.low_voltage == 0.0 || self.low_voltage > self.critical_voltage;
        basic && low_valid
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
            critical_action: BatteryFailsafeAction::Hold as u8,
            low_voltage: 0.0,
            low_action: BatteryFailsafeAction::None as u8,
            volt_mult: 3.95,
        };

        assert!(params.is_configured());
        assert!(params.arm_voltage > params.critical_voltage);
        assert_eq!(params.volt_mult, 3.95);
    }

    #[test]
    fn test_battery_failsafe_action_from_u8() {
        assert_eq!(
            BatteryFailsafeAction::from_u8(0),
            BatteryFailsafeAction::None
        );
        assert_eq!(
            BatteryFailsafeAction::from_u8(1),
            BatteryFailsafeAction::Hold
        );
        assert_eq!(
            BatteryFailsafeAction::from_u8(2),
            BatteryFailsafeAction::RTL
        );
        assert_eq!(
            BatteryFailsafeAction::from_u8(3),
            BatteryFailsafeAction::Disarm
        );
        assert_eq!(
            BatteryFailsafeAction::from_u8(255),
            BatteryFailsafeAction::Hold
        );
    }

    #[test]
    fn test_battery_params_low_voltage_validation() {
        // LOW disabled (0.0) → valid
        let params = BatteryParams {
            arm_voltage: 5.0,
            critical_voltage: 3.0,
            critical_action: BatteryFailsafeAction::Hold as u8,
            low_voltage: 0.0,
            low_action: BatteryFailsafeAction::None as u8,
            volt_mult: 3.95,
        };
        assert!(params.is_configured());

        // LOW > CRT → valid
        let params = BatteryParams {
            arm_voltage: 5.0,
            critical_voltage: 3.0,
            critical_action: BatteryFailsafeAction::Hold as u8,
            low_voltage: 3.5,
            low_action: BatteryFailsafeAction::Hold as u8,
            volt_mult: 3.95,
        };
        assert!(params.is_configured());

        // LOW <= CRT → invalid
        let params = BatteryParams {
            arm_voltage: 5.0,
            critical_voltage: 3.0,
            critical_action: BatteryFailsafeAction::Hold as u8,
            low_voltage: 2.5,
            low_action: BatteryFailsafeAction::Hold as u8,
            volt_mult: 3.95,
        };
        assert!(!params.is_configured());
    }
}
