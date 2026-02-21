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
use crate::autopilot::state::{FlightMode, HomePosition};
use crate::navigation::{GpsFixType, GpsPosition};
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

    /// Check if home should be auto-set on first GPS 3D fix.
    ///
    /// Returns true when home is not yet set and GPS has at least a 3D fix.
    /// Called on each control loop iteration.
    pub fn check_home_auto_set(
        &self,
        has_home: bool,
        gps_position: Option<&GpsPosition>,
        gps_fix_type: GpsFixType,
    ) -> bool {
        !has_home && gps_position.is_some() && gps_fix_type >= GpsFixType::Fix3D
    }

    /// Check if home should be updated while disarmed.
    ///
    /// Returns true when home is unlocked and the current GPS position
    /// has moved more than 0.5m from the current home. Called at 1 Hz while disarmed.
    pub fn check_home_update(
        &self,
        home: &HomePosition,
        gps: &GpsPosition,
        home_locked: bool,
    ) -> bool {
        if home_locked {
            return false;
        }
        let distance =
            calculate_distance_flat(home.latitude, home.longitude, gps.latitude, gps.longitude);
        distance >= DISTANCE_HOME_MINCHANGE
    }
}

/// Minimum distance in meters for disarmed home update (matches ArduPilot).
const DISTANCE_HOME_MINCHANGE: f32 = 0.5;

/// Earth radius in meters for flat-earth distance approximation.
const EARTH_RADIUS: f32 = 6_371_000.0;

/// Flat-earth distance approximation between two lat/lon positions.
///
/// Sufficient for the 0.5m home update threshold. Uses equirectangular projection.
fn calculate_distance_flat(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> f32 {
    const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
    let dlat = (lat2 - lat1) * DEG_TO_RAD * EARTH_RADIUS;
    let dlon = (lon2 - lon1) * DEG_TO_RAD * EARTH_RADIUS * libm::cosf(lat1 * DEG_TO_RAD);
    libm::sqrtf(dlat * dlat + dlon * dlon)
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

    // --- Home management tests ---

    use crate::autopilot::state::HomePosition;
    use crate::navigation::{GpsFixType, GpsPosition};

    fn make_gps(lat: f32, lon: f32, alt: f32) -> GpsPosition {
        GpsPosition {
            latitude: lat,
            longitude: lon,
            altitude: alt,
            speed: 0.0,
            course_over_ground: None,
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        }
    }

    #[test]
    fn test_home_auto_set_no_home_with_3d_fix() {
        let runner = new_runner();
        let gps = make_gps(35.6812, 139.7671, 40.0);
        assert!(runner.check_home_auto_set(false, Some(&gps), GpsFixType::Fix3D));
    }

    #[test]
    fn test_home_auto_set_already_has_home() {
        let runner = new_runner();
        let gps = make_gps(35.6812, 139.7671, 40.0);
        assert!(!runner.check_home_auto_set(true, Some(&gps), GpsFixType::Fix3D));
    }

    #[test]
    fn test_home_auto_set_no_gps() {
        let runner = new_runner();
        assert!(!runner.check_home_auto_set(false, None, GpsFixType::NoFix));
    }

    #[test]
    fn test_home_auto_set_2d_fix_insufficient() {
        let runner = new_runner();
        let gps = make_gps(35.6812, 139.7671, 0.0);
        assert!(!runner.check_home_auto_set(false, Some(&gps), GpsFixType::Fix2D));
    }

    #[test]
    fn test_home_auto_set_fires_only_once() {
        let runner = new_runner();
        let gps = make_gps(35.6812, 139.7671, 40.0);
        // First call: no home, returns true
        assert!(runner.check_home_auto_set(false, Some(&gps), GpsFixType::Fix3D));
        // After setting home (has_home=true), returns false
        assert!(!runner.check_home_auto_set(true, Some(&gps), GpsFixType::Fix3D));
    }

    #[test]
    fn test_home_update_moved_enough_unlocked() {
        let runner = new_runner();
        let home = HomePosition::new(35.6812, 139.7671, 40.0);
        // ~111m north
        let gps = make_gps(35.6822, 139.7671, 40.0);
        assert!(runner.check_home_update(&home, &gps, false));
    }

    #[test]
    fn test_home_update_locked_skips() {
        let runner = new_runner();
        let home = HomePosition::new(35.6812, 139.7671, 40.0);
        let gps = make_gps(35.6822, 139.7671, 40.0);
        assert!(!runner.check_home_update(&home, &gps, true));
    }

    #[test]
    fn test_home_update_below_threshold() {
        let runner = new_runner();
        let home = HomePosition::new(35.6812, 139.7671, 40.0);
        // Very small movement (~0.1m)
        let gps = make_gps(35.681201, 139.7671, 40.0);
        assert!(!runner.check_home_update(&home, &gps, false));
    }

    #[test]
    fn test_distance_calculation_accuracy() {
        // 1 degree latitude ≈ 111,195 m at equator
        let dist = calculate_distance_flat(0.0, 0.0, 1.0, 0.0);
        assert!((dist - 111_195.0).abs() < 100.0, "dist={dist}");

        // At 45°N, 1 degree longitude ≈ 78,847 m (f32 equirectangular gives ~78,627)
        let dist_lon = calculate_distance_flat(45.0, 0.0, 45.0, 1.0);
        assert!((dist_lon - 78_627.0).abs() < 500.0, "dist_lon={dist_lon}");
    }
}
