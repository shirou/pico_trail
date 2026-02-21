//! System State Management
//!
//! Re-exports autopilot state types from `pico_trail_core` and provides
//! firmware-specific extensions (arming, parameter loading, AHRS integration)
//! via the `SystemStateExt` trait.
//!
//! # Thread Safety
//!
//! System state is designed to be accessed from multiple tasks:
//! - Command handler updates armed state
//! - Telemetry task reads state for streaming
//! - GPS driver updates position data
//! - AHRS/Control loops update mode and status
//!
//! Use appropriate synchronization primitives (Mutex, critical sections)
//! when accessing state from multiple tasks.

// Re-export all types from core
pub use pico_trail_core::autopilot::state::{
    ArmedState, AttitudeState, BatteryState, FlightMode, HomePosition, SystemState, SYSTEM_STATE,
};
pub use pico_trail_core::navigation::GpsPosition;

/// Firmware-specific extensions for SystemState
///
/// These methods depend on firmware-specific modules (parameters, AHRS)
/// that are not in the core crate. Arm/disarm methods are now inherent
/// on SystemState in core.
pub trait SystemStateExt {
    /// Create a new system state with configuration loaded from parameter store
    fn from_param_store(param_store: &crate::parameters::storage::ParameterStore) -> Self;

    /// Update attitude state from AHRS
    fn update_attitude(&mut self, ahrs_state: &crate::subsystems::ahrs::SharedAhrsState);
}

impl SystemStateExt for SystemState {
    fn from_param_store(param_store: &crate::parameters::storage::ParameterStore) -> Self {
        use crate::parameters::arming::ArmingParams;
        use crate::parameters::battery::BatteryParams;
        use crate::parameters::compass::CompassParams;

        let arming_params = ArmingParams::from_store(param_store);
        let battery_params = BatteryParams::from_store(param_store);
        let compass_params = CompassParams::from_store(param_store);

        Self {
            armed: ArmedState::Disarmed,
            mode: FlightMode::Manual,
            battery: BatteryState::placeholder(),
            attitude: AttitudeState::placeholder(),
            gps_position: None,
            gps_timestamp_us: 0,
            home_position: None,
            home_locked: false,
            uptime_us: 0,
            cpu_load: 0.0,
            arming_checks: arming_params.check_bitmask as u16,
            battery_volt_mult: battery_params.volt_mult,
            battery_arm_volt: battery_params.arm_voltage,
            compass_yaw_offset: compass_params.declination,
        }
    }

    fn update_attitude(&mut self, ahrs_state: &crate::subsystems::ahrs::SharedAhrsState) {
        let state = ahrs_state.read();
        self.attitude.roll = state.roll;
        self.attitude.pitch = state.pitch;
        self.attitude.yaw = state.yaw;
        self.attitude.rollspeed = state.angular_rate.x;
        self.attitude.pitchspeed = state.angular_rate.y;
        self.attitude.yawspeed = state.angular_rate.z;
        self.attitude.timestamp_us = state.timestamp_us;
        self.attitude.healthy = state.healthy;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_armed_state_default() {
        let state = ArmedState::default();
        assert_eq!(state, ArmedState::Disarmed);
    }

    #[test]
    fn test_flight_mode_conversion() {
        assert_eq!(FlightMode::Manual.to_custom_mode(), 0);
        assert_eq!(FlightMode::Stabilize.to_custom_mode(), 1);
        assert_eq!(FlightMode::Hold.to_custom_mode(), 4);
        assert_eq!(FlightMode::Loiter.to_custom_mode(), 5);
        assert_eq!(FlightMode::Circle.to_custom_mode(), 7);
        assert_eq!(FlightMode::Auto.to_custom_mode(), 10);
        assert_eq!(FlightMode::Rtl.to_custom_mode(), 11);
        assert_eq!(FlightMode::SmartRtl.to_custom_mode(), 12);
        assert_eq!(FlightMode::Guided.to_custom_mode(), 15);

        assert_eq!(FlightMode::from_custom_mode(0), Some(FlightMode::Manual));
        assert_eq!(FlightMode::from_custom_mode(4), Some(FlightMode::Hold));
        assert_eq!(FlightMode::from_custom_mode(7), Some(FlightMode::Circle));
        assert_eq!(FlightMode::from_custom_mode(10), Some(FlightMode::Auto));
        assert_eq!(FlightMode::from_custom_mode(12), Some(FlightMode::SmartRtl));
        assert_eq!(FlightMode::from_custom_mode(15), Some(FlightMode::Guided));
        assert_eq!(FlightMode::from_custom_mode(99), None);
    }

    #[test]
    fn test_flight_mode_to_base_mode_flags() {
        use mavlink::common::MavModeFlag;

        let custom_enabled = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED.bits();

        let flags = FlightMode::Manual.to_base_mode_flags();
        assert!(
            flags & custom_enabled != 0,
            "Manual should have CUSTOM_MODE_ENABLED"
        );
        assert!(
            flags & MavModeFlag::MAV_MODE_FLAG_MANUAL_INPUT_ENABLED.bits() != 0,
            "Manual should have MANUAL_INPUT_ENABLED"
        );

        let flags = FlightMode::Guided.to_base_mode_flags();
        assert!(
            flags & custom_enabled != 0,
            "Guided should have CUSTOM_MODE_ENABLED"
        );
        assert!(
            flags & MavModeFlag::MAV_MODE_FLAG_GUIDED_ENABLED.bits() != 0,
            "Guided should have GUIDED_ENABLED"
        );
        assert!(
            flags & MavModeFlag::MAV_MODE_FLAG_STABILIZE_ENABLED.bits() != 0,
            "Guided should have STABILIZE_ENABLED"
        );

        let flags = FlightMode::Auto.to_base_mode_flags();
        assert!(
            flags & custom_enabled != 0,
            "Auto should have CUSTOM_MODE_ENABLED"
        );
        assert!(
            flags & MavModeFlag::MAV_MODE_FLAG_AUTO_ENABLED.bits() != 0,
            "Auto should have AUTO_ENABLED"
        );
    }

    #[test]
    fn test_battery_critical() {
        let mut battery = BatteryState {
            voltage: 12.0,
            ..Default::default()
        };
        assert!(!battery.is_critical());

        battery.voltage = 9.0;
        assert!(battery.is_critical());
    }

    #[test]
    fn test_voltage_from_adc_zero() {
        assert_eq!(BatteryState::voltage_from_adc(0, 3.95), 0.0);
    }

    #[test]
    fn test_voltage_from_adc_max() {
        let voltage = BatteryState::voltage_from_adc(4095, 3.95);
        assert!((voltage - 13.035).abs() < 0.01);
    }

    #[test]
    fn test_voltage_from_adc_typical() {
        let voltage = BatteryState::voltage_from_adc(3000, 3.95);
        assert!((voltage - 9.549).abs() < 0.01);
    }

    #[test]
    fn test_voltage_from_adc_different_multipliers() {
        let adc = 3000_u16;
        let v1 = BatteryState::voltage_from_adc(adc, 3.5);
        let v2 = BatteryState::voltage_from_adc(adc, 4.0);
        let v3 = BatteryState::voltage_from_adc(adc, 4.5);
        assert!(v1 < v2);
        assert!(v2 < v3);
    }

    #[test]
    fn test_system_state_arm_disarm() {
        let mut state = SystemState::new();
        assert!(!state.is_armed());

        state.battery.voltage = 12.0;
        assert!(state.arm().is_ok());
        assert!(state.is_armed());

        assert!(state.arm().is_err());

        assert!(state.disarm().is_ok());
        assert!(!state.is_armed());

        assert!(state.disarm().is_err());
    }

    #[test]
    fn test_arm_with_low_battery() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0;

        let result = state.arm();
        assert!(result.is_err());

        if let Err(crate::core::arming::ArmingError::CheckFailed { category, .. }) = result {
            assert_eq!(category, crate::core::arming::CheckCategory::Battery);
        } else {
            panic!("Expected CheckFailed error with Battery category");
        }

        assert!(!state.is_armed());
    }

    #[test]
    fn test_arm_forced_bypasses_checks() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0;

        let result = state.arm_forced();
        assert!(result.is_ok());
        assert!(state.is_armed());
    }

    #[test]
    fn test_arm_forced_already_armed() {
        let mut state = SystemState::new();
        state.battery.voltage = 12.0;

        assert!(state.arm().is_ok());

        let result = state.arm_forced();
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            crate::core::arming::ArmingError::AlreadyArmed
        );
    }

    #[test]
    fn test_disarm_forced_bypasses_checks() {
        let mut state = SystemState::new();
        state.battery.voltage = 12.0;

        assert!(state.arm().is_ok());
        assert!(state.is_armed());

        let result = state.disarm_forced();
        assert!(result.is_ok());
        assert!(!state.is_armed());
    }

    #[test]
    fn test_disarm_forced_not_armed() {
        let mut state = SystemState::new();

        let result = state.disarm_forced();
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            crate::core::arming::DisarmError::NotArmed
        );
    }

    #[test]
    fn test_mode_change() {
        let mut state = SystemState::new();
        assert_eq!(state.mode, FlightMode::Manual);

        assert!(state.set_mode(FlightMode::Auto).is_ok());
        assert_eq!(state.mode, FlightMode::Auto);
    }

    #[test]
    fn test_battery_update() {
        let mut state = SystemState::new();
        state.battery_volt_mult = 3.95;

        state.update_battery(3000);

        assert!((state.battery.voltage - 9.549).abs() < 0.01);
        assert!(state.battery.remaining_percent > 0);
        assert!(state.battery.remaining_percent < 30);
    }

    #[test]
    fn test_estimate_remaining_full() {
        assert_eq!(BatteryState::estimate_remaining_percent(12.6), 100);
    }

    #[test]
    fn test_estimate_remaining_empty() {
        assert_eq!(BatteryState::estimate_remaining_percent(9.0), 0);
    }

    #[test]
    fn test_estimate_remaining_mid() {
        assert_eq!(BatteryState::estimate_remaining_percent(10.8), 50);
    }

    #[test]
    fn test_estimate_remaining_nominal() {
        assert_eq!(BatteryState::estimate_remaining_percent(11.1), 58);
    }

    #[test]
    fn test_estimate_remaining_low() {
        assert_eq!(BatteryState::estimate_remaining_percent(10.5), 41);
    }

    #[test]
    fn test_estimate_remaining_above_full() {
        assert_eq!(BatteryState::estimate_remaining_percent(13.0), 100);
    }

    #[test]
    fn test_estimate_remaining_3s_below_empty() {
        assert_eq!(BatteryState::estimate_remaining_percent(8.8), 0);
    }

    #[test]
    fn test_estimate_remaining_2s_full() {
        assert_eq!(BatteryState::estimate_remaining_percent(8.4), 100);
    }

    #[test]
    fn test_estimate_remaining_2s_empty() {
        assert_eq!(BatteryState::estimate_remaining_percent(6.0), 0);
    }

    #[test]
    fn test_estimate_remaining_2s_mid() {
        assert_eq!(BatteryState::estimate_remaining_percent(7.2), 50);
    }

    #[test]
    fn test_estimate_remaining_2s_nominal() {
        assert_eq!(BatteryState::estimate_remaining_percent(7.4), 58);
    }

    #[test]
    fn test_estimate_remaining_2s_typical() {
        assert_eq!(BatteryState::estimate_remaining_percent(7.827), 76);
    }

    #[test]
    fn test_estimate_remaining_2s_above_full() {
        assert_eq!(BatteryState::estimate_remaining_percent(8.6), 100);
    }

    #[test]
    fn test_estimate_remaining_2s_below_empty() {
        assert_eq!(BatteryState::estimate_remaining_percent(5.5), 0);
    }

    #[test]
    fn test_from_param_store_compass_yaw_offset_default() {
        use crate::parameters::storage::ParameterStore;

        let mut store = ParameterStore::default();
        crate::parameters::compass::CompassParams::register_defaults(&mut store).unwrap();

        let state = SystemState::from_param_store(&store);
        assert_eq!(state.compass_yaw_offset, 0.0);
    }

    #[test]
    fn test_from_param_store_compass_yaw_offset_calibrated() {
        use crate::parameters::storage::{ParamValue, ParameterStore};

        let mut store = ParameterStore::default();
        crate::parameters::compass::CompassParams::register_defaults(&mut store).unwrap();

        store.set("COMPASS_DEC", ParamValue::Float(0.126)).unwrap();

        let state = SystemState::from_param_store(&store);
        assert!((state.compass_yaw_offset - 0.126).abs() < f32::EPSILON);
    }
}
