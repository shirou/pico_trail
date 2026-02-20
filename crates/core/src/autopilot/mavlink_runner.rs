//! MAVLink Loop Runner
//!
//! Shared failsafe logic for the MAVLink loop used by both firmware and SITL.
//! Handles arm transition detection, failsafe config reload, and action mapping
//! for battery and GCS communication lost failsafes.
//!
//! ## Usage
//!
//! The caller (firmware or SITL) creates a `MavlinkLoopRunner` and calls
//! `check_battery()` at 10 Hz and `check_gcs_failsafe()` at 10 Hz.
//! The returned actions tell the caller what to do.

use crate::autopilot::battery::{BatteryFailsafeChecker, BatteryFailsafeConfig};
use crate::autopilot::gcs_failsafe::{GcsFailsafeAction, GcsFailsafeChecker, GcsFailsafeConfig};
use crate::autopilot::state::FlightMode;
use crate::parameters::battery::{BatteryFailsafeAction, BatteryParams};
use crate::parameters::failsafe::FailsafeParams;
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

/// MAVLink loop runner that encapsulates battery and GCS failsafe logic.
///
/// Tracks arm transitions to reset sticky failsafe flags and reload
/// config from the parameter store on each arm event.
pub struct MavlinkLoopRunner {
    battery_failsafe: BatteryFailsafeChecker,
    gcs_failsafe: GcsFailsafeChecker,
    was_armed: bool,
}

impl MavlinkLoopRunner {
    /// Create a new runner with initial battery and GCS failsafe configs.
    pub fn new(battery_config: BatteryFailsafeConfig, gcs_config: GcsFailsafeConfig) -> Self {
        Self {
            battery_failsafe: BatteryFailsafeChecker::new(battery_config),
            gcs_failsafe: GcsFailsafeChecker::new(gcs_config),
            was_armed: false,
        }
    }

    /// Handle arm transition (disarmed → armed): reload configs and reset failsafes.
    ///
    /// Called internally by both `check_battery` and `check_gcs_failsafe` so that
    /// either check alone is sufficient to trigger the arm-transition reset.
    /// The reset is idempotent per transition: only the first call in a given
    /// arm cycle performs the reload.
    fn handle_arm_transition(&mut self, is_armed: bool, param_store: &ParameterStore) {
        if is_armed && !self.was_armed {
            let fresh_params = BatteryParams::from_store(param_store);
            self.battery_failsafe
                .reset_with_config(BatteryFailsafeConfig::from_params(&fresh_params));

            let failsafe_params = FailsafeParams::from_store(param_store);
            self.gcs_failsafe
                .reset_with_config(GcsFailsafeConfig::from_params(&failsafe_params));
        }
        self.was_armed = is_armed;
    }

    /// Check battery voltage and return action if failsafe triggered.
    ///
    /// Called once per battery update interval (typically 10 Hz).
    ///
    /// On arm transition (disarmed → armed), reloads all failsafe configs from
    /// `param_store` and resets sticky failsafe flags.
    pub fn check_battery(
        &mut self,
        voltage: f32,
        is_armed: bool,
        param_store: &ParameterStore,
    ) -> BatteryAction {
        self.handle_arm_transition(is_armed, param_store);

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

    /// Check GCS heartbeat and return action if failsafe triggered.
    ///
    /// Called at ~10 Hz from the main control loop. On arm transition, reloads
    /// failsafe config from `param_store` (independently of `check_battery`).
    pub fn check_gcs_failsafe(
        &mut self,
        last_heartbeat_us: u64,
        heartbeat_count: u32,
        current_time_us: u64,
        is_armed: bool,
        current_mode: FlightMode,
        param_store: &ParameterStore,
    ) -> GcsFailsafeAction {
        self.handle_arm_transition(is_armed, param_store);

        self.gcs_failsafe.check(
            last_heartbeat_us,
            heartbeat_count,
            current_time_us,
            is_armed,
            current_mode,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parameters::failsafe::FailsafeAction;

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
        FailsafeParams::register_defaults(&mut store).unwrap();
        store
    }

    fn make_param_store(low_volt: f32, crt_volt: f32) -> ParameterStore {
        make_param_store_with_actions(
            low_volt,
            crt_volt,
            BatteryFailsafeAction::Hold,
            BatteryFailsafeAction::Disarm,
        )
    }

    fn default_battery_config() -> BatteryFailsafeConfig {
        BatteryFailsafeConfig {
            low_voltage: 3.5,
            low_action: BatteryFailsafeAction::Hold,
            critical_voltage: 3.0,
            critical_action: BatteryFailsafeAction::Disarm,
        }
    }

    fn default_gcs_config() -> GcsFailsafeConfig {
        GcsFailsafeConfig {
            enabled: true,
            gcs_timeout_us: 5_000_000,
            persistence_us: 1_500_000,
            action: FailsafeAction::Hold,
        }
    }

    fn new_runner() -> MavlinkLoopRunner {
        MavlinkLoopRunner::new(default_battery_config(), default_gcs_config())
    }

    // --- Battery failsafe tests ---

    #[test]
    fn test_normal_voltage_no_action() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = new_runner();
        assert_eq!(runner.check_battery(4.0, true, &store), BatteryAction::None);
    }

    #[test]
    fn test_critical_voltage_disarm() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = new_runner();
        assert_eq!(
            runner.check_battery(2.5, true, &store),
            BatteryAction::Disarm
        );
    }

    #[test]
    fn test_low_voltage_hold() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = new_runner();

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
        let mut runner = new_runner();

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
        let mut runner = new_runner();

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
        let mut runner = new_runner();
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
        let battery_config = BatteryFailsafeConfig {
            low_voltage: 3.5,
            low_action: BatteryFailsafeAction::RTL,
            critical_voltage: 3.0,
            critical_action: BatteryFailsafeAction::RTL,
        };
        let mut runner = MavlinkLoopRunner::new(battery_config, default_gcs_config());

        assert_eq!(
            runner.check_battery(2.5, true, &store),
            BatteryAction::SetMode(FlightMode::Rtl)
        );
    }

    // --- GCS failsafe runner tests ---

    const GCS_TIMEOUT_US: u64 = 5_000_000;
    const PERSISTENCE_US: u64 = 1_500_000;

    /// Run two-stage GCS failsafe detection through the runner
    fn trigger_gcs(
        runner: &mut MavlinkLoopRunner,
        last_hb: u64,
        hb_count: u32,
        store: &ParameterStore,
    ) -> GcsFailsafeAction {
        // Stage 1: detect
        runner.check_gcs_failsafe(
            last_hb,
            hb_count,
            last_hb + GCS_TIMEOUT_US,
            true,
            FlightMode::Manual,
            store,
        );
        // Stage 2: persist
        runner.check_gcs_failsafe(
            last_hb,
            hb_count,
            last_hb + GCS_TIMEOUT_US + PERSISTENCE_US,
            true,
            FlightMode::Manual,
            store,
        )
    }

    #[test]
    fn test_gcs_normal_heartbeats_no_action() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = new_runner();
        let action =
            runner.check_gcs_failsafe(1_000_000, 10, 2_000_000, true, FlightMode::Manual, &store);
        assert_eq!(action, GcsFailsafeAction::None);
    }

    #[test]
    fn test_gcs_timeout_triggers_hold() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = new_runner();
        let action = trigger_gcs(&mut runner, 0, 10, &store);
        assert_eq!(action, GcsFailsafeAction::SetMode(FlightMode::Hold));
    }

    #[test]
    fn test_gcs_arm_transition_resets() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = new_runner();

        // Trigger GCS failsafe (two-stage)
        trigger_gcs(&mut runner, 0, 10, &store);

        // Disarm and re-arm (via check_gcs_failsafe which now handles arm transition)
        runner.check_gcs_failsafe(0, 10, 0, false, FlightMode::Manual, &store);
        runner.check_gcs_failsafe(0, 10, 0, true, FlightMode::Manual, &store);

        // GCS failsafe was reset on arm transition
        // Single check at past-timeout: condition detected but persistence not yet met
        let action = runner.check_gcs_failsafe(
            0,
            10,
            GCS_TIMEOUT_US + PERSISTENCE_US + 1_000_000,
            true,
            FlightMode::Manual,
            &store,
        );
        // Condition just re-detected, persistence not yet met
        assert_eq!(action, GcsFailsafeAction::None);
    }

    #[test]
    fn test_gcs_config_reload_on_arm() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = new_runner();

        // Arm via check_gcs_failsafe (triggers config reload independently)
        runner.check_gcs_failsafe(1_000_000, 10, 1_000_000, true, FlightMode::Manual, &store);

        // GCS config was reloaded from store defaults (5.0s timeout, 1.5s persistence)
        let action = trigger_gcs(&mut runner, 0, 10, &store);
        assert_eq!(action, GcsFailsafeAction::SetMode(FlightMode::Hold));
    }

    #[test]
    fn test_gcs_arm_transition_via_gcs_check_only() {
        let store = make_param_store(3.5, 3.0);
        let mut runner = new_runner();

        // Trigger GCS failsafe
        trigger_gcs(&mut runner, 0, 10, &store);

        // Disarm and re-arm ONLY via check_gcs_failsafe (no check_battery calls)
        runner.check_gcs_failsafe(0, 10, 0, false, FlightMode::Manual, &store);
        runner.check_gcs_failsafe(0, 10, 0, true, FlightMode::Manual, &store);

        // Should have reset: trigger again requires full two-stage detection
        let action = runner.check_gcs_failsafe(
            0,
            10,
            GCS_TIMEOUT_US + PERSISTENCE_US + 1_000_000,
            true,
            FlightMode::Manual,
            &store,
        );
        assert_eq!(action, GcsFailsafeAction::None);
    }
}
