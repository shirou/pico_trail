//! MAVLink Loop Runner
//!
//! Shared battery failsafe logic for the MAVLink loop used by both firmware and SITL.
//! Handles arm transition detection, failsafe config reload, and action mapping.
//!
//! ## Usage
//!
//! The caller (firmware or SITL) creates a `MavlinkLoopRunner` and calls
//! `check_battery()` at 10 Hz. The returned `BatteryAction` tells the caller
//! what to do (set mode, disarm, or nothing).

use crate::autopilot::battery::{BatteryFailsafeChecker, BatteryFailsafeConfig};
use crate::autopilot::state::FlightMode;
use crate::parameters::battery::{BatteryFailsafeAction, BatteryParams};
use crate::parameters::ParameterStore;

/// Action to take after a battery failsafe check.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryAction {
    /// No action needed
    None,
    /// Set vehicle mode (e.g., Hold or RTL)
    SetMode(FlightMode),
    /// Force disarm
    Disarm,
}

/// MAVLink loop runner that encapsulates battery failsafe logic.
///
/// Tracks arm transitions to reset sticky failsafe flags and reload
/// config from the parameter store on each arm event.
pub struct MavlinkLoopRunner {
    battery_failsafe: BatteryFailsafeChecker,
    was_armed: bool,
}

impl MavlinkLoopRunner {
    /// Create a new runner with initial battery failsafe config.
    pub fn new(config: BatteryFailsafeConfig) -> Self {
        Self {
            battery_failsafe: BatteryFailsafeChecker::new(config),
            was_armed: false,
        }
    }

    /// Check battery voltage and return action if failsafe triggered.
    ///
    /// Called once per battery update interval (typically 10 Hz).
    ///
    /// On arm transition (disarmed → armed), reloads battery config from
    /// `param_store` and resets sticky failsafe flags.
    ///
    /// # Returns
    ///
    /// - `BatteryAction::None` - No failsafe triggered
    /// - `BatteryAction::SetMode(Hold)` - Hold action triggered
    /// - `BatteryAction::SetMode(Rtl)` - RTL action triggered
    /// - `BatteryAction::Disarm` - Disarm action triggered
    pub fn check_battery(
        &mut self,
        voltage: f32,
        is_armed: bool,
        param_store: &ParameterStore,
    ) -> BatteryAction {
        // On arm transition: reset sticky flags and reload config
        if is_armed && !self.was_armed {
            let fresh_params = BatteryParams::from_store(param_store);
            self.battery_failsafe
                .reset_with_config(BatteryFailsafeConfig::from_params(&fresh_params));
        }
        self.was_armed = is_armed;

        // Check battery failsafe
        if let Some(event) = self.battery_failsafe.check(voltage, is_armed) {
            return match event.action {
                BatteryFailsafeAction::None => BatteryAction::None,
                BatteryFailsafeAction::Hold => BatteryAction::SetMode(FlightMode::Hold),
                BatteryFailsafeAction::RTL => BatteryAction::SetMode(FlightMode::Rtl),
                BatteryFailsafeAction::Disarm => BatteryAction::Disarm,
            };
        }

        BatteryAction::None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_param_store_with_actions(
        low_volt: f32,
        crt_volt: f32,
        low_action: BatteryFailsafeAction,
        crt_action: BatteryFailsafeAction,
    ) -> ParameterStore {
        use crate::parameters::storage::ParamFlags;
        use crate::parameters::ParamValue;
        let mut store = ParameterStore::new();
        let flags = ParamFlags::empty();
        store
            .register("BATT_LOW_VOLT", ParamValue::Float(low_volt), flags)
            .unwrap();
        store
            .register("BATT_CRT_VOLT", ParamValue::Float(crt_volt), flags)
            .unwrap();
        store
            .register("BATT_FS_LOW_ACT", ParamValue::Int(low_action as i32), flags)
            .unwrap();
        store
            .register("BATT_FS_CRT_ACT", ParamValue::Int(crt_action as i32), flags)
            .unwrap();
        store
            .register("BATT_ARM_VOLT", ParamValue::Float(5.0), flags)
            .unwrap();
        store
            .register("BATT_VOLT_MULT", ParamValue::Float(3.95), flags)
            .unwrap();
        store
    }

    fn make_param_store(low_volt: f32, crt_volt: f32) -> ParameterStore {
        use crate::parameters::storage::ParamFlags;
        use crate::parameters::ParamValue;
        let mut store = ParameterStore::new();
        let flags = ParamFlags::empty();
        store
            .register("BATT_LOW_VOLT", ParamValue::Float(low_volt), flags)
            .unwrap();
        store
            .register("BATT_CRT_VOLT", ParamValue::Float(crt_volt), flags)
            .unwrap();
        store
            .register(
                "BATT_FS_LOW_ACT",
                ParamValue::Int(BatteryFailsafeAction::Hold as i32),
                flags,
            )
            .unwrap();
        store
            .register(
                "BATT_FS_CRT_ACT",
                ParamValue::Int(BatteryFailsafeAction::Disarm as i32),
                flags,
            )
            .unwrap();
        // BatteryParams::from_store also reads these with defaults
        store
            .register("BATT_ARM_VOLT", ParamValue::Float(5.0), flags)
            .unwrap();
        store
            .register("BATT_VOLT_MULT", ParamValue::Float(3.95), flags)
            .unwrap();
        store
    }

    fn default_config() -> BatteryFailsafeConfig {
        BatteryFailsafeConfig {
            low_voltage: 3.5,
            low_action: BatteryFailsafeAction::Hold,
            critical_voltage: 3.0,
            critical_action: BatteryFailsafeAction::Disarm,
        }
    }

    #[test]
    fn test_normal_voltage_no_action() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = MavlinkLoopRunner::new(default_config());
        assert_eq!(runner.check_battery(4.0, true, &store), BatteryAction::None);
    }

    #[test]
    fn test_critical_voltage_disarm() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = MavlinkLoopRunner::new(default_config());
        assert_eq!(
            runner.check_battery(2.5, true, &store),
            BatteryAction::Disarm
        );
    }

    #[test]
    fn test_low_voltage_hold() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = MavlinkLoopRunner::new(default_config());

        // 100 samples below LOW threshold to trigger hysteresis
        for _ in 0..99 {
            assert_eq!(runner.check_battery(3.3, true, &store), BatteryAction::None);
        }
        assert_eq!(
            runner.check_battery(3.3, true, &store),
            BatteryAction::SetMode(FlightMode::Hold)
        );
    }

    #[test]
    fn test_arm_transition_resets_failsafe() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = MavlinkLoopRunner::new(default_config());

        // Trigger critical
        assert_eq!(
            runner.check_battery(2.5, true, &store),
            BatteryAction::Disarm
        );

        // Sticky: no more events
        assert_eq!(runner.check_battery(2.5, true, &store), BatteryAction::None);

        // Disarm
        assert_eq!(
            runner.check_battery(2.5, false, &store),
            BatteryAction::None
        );

        // Re-arm: triggers reset_with_config, critical fires again
        assert_eq!(
            runner.check_battery(2.5, true, &store),
            BatteryAction::Disarm
        );
    }

    #[test]
    fn test_arm_transition_reloads_config() {
        // Store has critical threshold 3.0V (same as initial config)
        let store = make_param_store(3.5, 3.0);
        let mut runner = MavlinkLoopRunner::new(default_config());

        // First arm: config reloaded from store (critical=3.0V), 2.5V triggers
        assert_eq!(
            runner.check_battery(2.5, true, &store),
            BatteryAction::Disarm
        );

        // Disarm
        runner.check_battery(2.5, false, &store);

        // Change store to have higher critical threshold (2.0V)
        let store_v2 = make_param_store_with_actions(
            3.5,
            2.0,
            BatteryFailsafeAction::Hold,
            BatteryFailsafeAction::Disarm,
        );

        // Re-arm: config reloaded from new store (critical now 2.0V)
        // 2.5V no longer triggers critical (above 2.0V threshold)
        assert_eq!(
            runner.check_battery(2.5, true, &store_v2),
            BatteryAction::None
        );
    }

    #[test]
    fn test_disarmed_no_action() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = MavlinkLoopRunner::new(default_config());
        for _ in 0..200 {
            assert_eq!(
                runner.check_battery(2.5, false, &store),
                BatteryAction::None
            );
        }
    }

    #[test]
    fn test_rtl_action() {
        let store = make_param_store_with_actions(
            3.5,
            3.0,
            BatteryFailsafeAction::RTL,
            BatteryFailsafeAction::RTL,
        );
        let config = BatteryFailsafeConfig {
            low_voltage: 3.5,
            low_action: BatteryFailsafeAction::RTL,
            critical_voltage: 3.0,
            critical_action: BatteryFailsafeAction::RTL,
        };
        let mut runner = MavlinkLoopRunner::new(config);

        assert_eq!(
            runner.check_battery(2.5, true, &store),
            BatteryAction::SetMode(FlightMode::Rtl)
        );
    }
}
