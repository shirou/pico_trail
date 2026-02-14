//! Autopilot state types
//!
//! Core autopilot state types shared between firmware and SITL:
//! - `ArmedState`: Vehicle arming state
//! - `FlightMode`: Flight mode for rover/boat
//! - `BatteryState`: Battery voltage and capacity
//! - `AttitudeState`: Vehicle attitude from AHRS
//! - `HomePosition`: Home position for RTL
//! - `SystemState`: Aggregate system state for MAVLink and command handling

use crate::navigation::GpsPosition;

/// MAVLink MAV_MODE_FLAG bit values (from MAVLink specification)
mod mav_mode_flag {
    pub const CUSTOM_MODE_ENABLED: u8 = 1;
    pub const AUTO_ENABLED: u8 = 4;
    pub const GUIDED_ENABLED: u8 = 8;
    pub const STABILIZE_ENABLED: u8 = 16;
    pub const MANUAL_INPUT_ENABLED: u8 = 64;
}

/// Vehicle arming state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmedState {
    /// Vehicle is disarmed (motors off, safe)
    Disarmed,
    /// Vehicle is armed (motors can spin, caution)
    Armed,
}

impl Default for ArmedState {
    fn default() -> Self {
        Self::Disarmed
    }
}

/// Flight mode for rover/boat
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlightMode {
    /// Manual mode (direct RC control)
    Manual,
    /// Stabilize mode (heading hold)
    Stabilize,
    /// Hold mode (stop and hold position)
    Hold,
    /// Loiter mode (position hold)
    Loiter,
    /// Circle mode (orbit around a point)
    Circle,
    /// Auto mode (following waypoints)
    Auto,
    /// Return to launch
    Rtl,
    /// Smart return to launch (retrace path)
    SmartRtl,
    /// Guided mode (navigate to target position)
    Guided,
}

impl Default for FlightMode {
    fn default() -> Self {
        Self::Manual
    }
}

impl FlightMode {
    /// Get human-readable name for the flight mode
    ///
    /// Returns a short uppercase name suitable for STATUSTEXT messages.
    pub fn as_str(self) -> &'static str {
        match self {
            FlightMode::Manual => "MANUAL",
            FlightMode::Stabilize => "STABILIZE",
            FlightMode::Hold => "HOLD",
            FlightMode::Loiter => "LOITER",
            FlightMode::Circle => "CIRCLE",
            FlightMode::Auto => "AUTO",
            FlightMode::Guided => "GUIDED",
            FlightMode::Rtl => "RTL",
            FlightMode::SmartRtl => "SMARTRTL",
        }
    }

    /// Convert to MAVLink custom mode number
    ///
    /// This mapping follows ArduPilot's mode numbering for rovers.
    /// See: <https://ardupilot.org/rover/docs/rover-features.html#flight-modes>
    pub fn to_custom_mode(self) -> u32 {
        match self {
            FlightMode::Manual => 0,
            FlightMode::Stabilize => 1,
            FlightMode::Hold => 4,
            FlightMode::Loiter => 5,
            FlightMode::Circle => 7,
            FlightMode::Auto => 10,
            FlightMode::Rtl => 11,
            FlightMode::SmartRtl => 12,
            FlightMode::Guided => 15,
        }
    }

    /// Convert from MAVLink custom mode number
    ///
    /// Returns None if mode number is unrecognized.
    /// See: <https://ardupilot.org/rover/docs/rover-features.html#flight-modes>
    pub fn from_custom_mode(mode: u32) -> Option<Self> {
        match mode {
            0 => Some(FlightMode::Manual),
            1 => Some(FlightMode::Stabilize),
            4 => Some(FlightMode::Hold),
            5 => Some(FlightMode::Loiter),
            7 => Some(FlightMode::Circle),
            10 => Some(FlightMode::Auto),
            11 => Some(FlightMode::Rtl),
            12 => Some(FlightMode::SmartRtl),
            15 => Some(FlightMode::Guided),
            _ => None,
        }
    }

    /// Convert to MAVLink base_mode flags
    ///
    /// Returns the appropriate MAV_MODE_FLAG bitmask for this flight mode.
    /// The armed flag should be OR'd separately based on arm state.
    ///
    /// Key flags:
    /// - CUSTOM_MODE_ENABLED (1): Always set to indicate custom_mode field is valid
    /// - MANUAL_INPUT_ENABLED (64): RC input is being used
    /// - STABILIZE_ENABLED (16): Attitude stabilization active
    /// - GUIDED_ENABLED (8): Guided mode (external position commands)
    /// - AUTO_ENABLED (4): Autonomous mode (waypoint following)
    pub fn to_base_mode_flags(self) -> u8 {
        // Always set CUSTOM_MODE_ENABLED so GCS reads custom_mode field
        let mut flags = mav_mode_flag::CUSTOM_MODE_ENABLED;

        match self {
            FlightMode::Manual => {
                flags |= mav_mode_flag::MANUAL_INPUT_ENABLED;
            }
            FlightMode::Stabilize => {
                flags |= mav_mode_flag::MANUAL_INPUT_ENABLED;
                flags |= mav_mode_flag::STABILIZE_ENABLED;
            }
            FlightMode::Hold => {
                flags |= mav_mode_flag::STABILIZE_ENABLED;
                flags |= mav_mode_flag::GUIDED_ENABLED;
            }
            FlightMode::Loiter => {
                flags |= mav_mode_flag::STABILIZE_ENABLED;
                flags |= mav_mode_flag::GUIDED_ENABLED;
            }
            FlightMode::Circle => {
                flags |= mav_mode_flag::STABILIZE_ENABLED;
                flags |= mav_mode_flag::AUTO_ENABLED;
            }
            FlightMode::Auto => {
                flags |= mav_mode_flag::STABILIZE_ENABLED;
                flags |= mav_mode_flag::AUTO_ENABLED;
            }
            FlightMode::Guided => {
                flags |= mav_mode_flag::STABILIZE_ENABLED;
                flags |= mav_mode_flag::GUIDED_ENABLED;
            }
            FlightMode::Rtl => {
                flags |= mav_mode_flag::STABILIZE_ENABLED;
                flags |= mav_mode_flag::AUTO_ENABLED;
            }
            FlightMode::SmartRtl => {
                flags |= mav_mode_flag::STABILIZE_ENABLED;
                flags |= mav_mode_flag::AUTO_ENABLED;
            }
        }

        flags
    }
}

/// Home position for RTL and mission planning
///
/// Stores the home location that the vehicle will return to in RTL mode.
/// Can be set manually via MAV_CMD_DO_SET_HOME or automatically on first GPS fix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HomePosition {
    /// Latitude in degrees (-90 to +90)
    pub latitude: f32,
    /// Longitude in degrees (-180 to +180)
    pub longitude: f32,
    /// Altitude in meters above sea level
    pub altitude: f32,
}

impl HomePosition {
    /// Create a new home position
    pub const fn new(latitude: f32, longitude: f32, altitude: f32) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
        }
    }

    /// Create home position from COMMAND_INT parameters
    ///
    /// COMMAND_INT uses scaled integers for lat/lon:
    /// - x: latitude in degrees * 10^7
    /// - y: longitude in degrees * 10^7
    /// - z: altitude in meters
    pub fn from_command_int(x: i32, y: i32, z: f32) -> Self {
        Self {
            latitude: x as f32 / 1e7,
            longitude: y as f32 / 1e7,
            altitude: z,
        }
    }

    /// Create home position from current GPS position
    pub fn from_gps(gps: &GpsPosition) -> Self {
        Self {
            latitude: gps.latitude,
            longitude: gps.longitude,
            altitude: gps.altitude,
        }
    }
}

/// Attitude state from AHRS
#[derive(Debug, Clone, Copy, Default)]
pub struct AttitudeState {
    /// Roll angle in radians
    pub roll: f32,
    /// Pitch angle in radians
    pub pitch: f32,
    /// Yaw angle in radians (heading, 0 = north)
    pub yaw: f32,
    /// Roll angular rate in rad/s
    pub rollspeed: f32,
    /// Pitch angular rate in rad/s
    pub pitchspeed: f32,
    /// Yaw angular rate in rad/s
    pub yawspeed: f32,
    /// Timestamp when attitude was last updated (microseconds since boot)
    pub timestamp_us: u64,
    /// True if AHRS data is valid and healthy
    pub healthy: bool,
}

impl AttitudeState {
    /// Create attitude state with placeholder values (const fn for static initialization)
    pub const fn placeholder() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
            timestamp_us: 0,
            healthy: false,
        }
    }

    /// Check if attitude data is fresh (updated within threshold)
    ///
    /// # Arguments
    ///
    /// * `current_time_us` - Current system uptime in microseconds
    /// * `threshold_us` - Maximum age for data to be considered fresh (default: 500ms)
    pub fn is_fresh(&self, current_time_us: u64, threshold_us: u64) -> bool {
        if !self.healthy {
            return false;
        }
        current_time_us.saturating_sub(self.timestamp_us) < threshold_us
    }
}

/// Battery state
#[derive(Debug, Clone, Copy, Default)]
pub struct BatteryState {
    /// Battery voltage (volts)
    pub voltage: f32,
    /// Battery current (amperes, positive = discharging)
    pub current: f32,
    /// Battery remaining capacity (percentage, 0-100)
    pub remaining_percent: u8,
}

impl BatteryState {
    /// Create battery state with placeholder values
    pub const fn placeholder() -> Self {
        Self {
            voltage: 12.0,
            current: 0.0,
            remaining_percent: 100,
        }
    }

    /// Check if battery voltage is critically low
    ///
    /// Returns true if voltage is below 10.0V (typical for 3S LiPo minimum).
    pub fn is_critical(&self) -> bool {
        self.voltage < 10.0
    }

    /// Convert ADC value to battery voltage
    ///
    /// Converts a raw 12-bit ADC reading to battery voltage using the voltage
    /// divider multiplier (BATT_VOLT_MULT parameter).
    ///
    /// # Formula
    ///
    /// ```text
    /// voltage = (adc / 4095.0 * 3.3) * mult
    /// ```
    ///
    /// # Arguments
    ///
    /// * `adc` - Raw ADC value (0-4095)
    /// * `mult` - Voltage multiplier from BATT_VOLT_MULT parameter
    pub fn voltage_from_adc(adc: u16, mult: f32) -> f32 {
        (adc as f32 / 4095.0 * 3.3) * mult
    }

    /// Estimate battery remaining percentage from voltage
    ///
    /// Automatically detects battery cell count (2S or 3S) from voltage range.
    ///
    /// - **2S LiPo**: 6.0V (empty) to 8.4V (full)
    /// - **3S LiPo**: 9.0V (empty) to 12.6V (full)
    ///
    /// Detection threshold: voltage >= 8.7V → 3S, otherwise 2S.
    pub fn estimate_remaining_percent(voltage: f32) -> u8 {
        const DETECTION_THRESHOLD: f32 = 8.7;

        let (voltage_full, voltage_empty) = if voltage >= DETECTION_THRESHOLD {
            (12.6, 9.0) // 3S LiPo
        } else {
            (8.4, 6.0) // 2S LiPo
        };

        if voltage >= voltage_full {
            100
        } else if voltage <= voltage_empty {
            0
        } else {
            let percent = (voltage - voltage_empty) / (voltage_full - voltage_empty) * 100.0;
            percent.clamp(0.0, 100.0) as u8
        }
    }
}

/// System state for autopilot
///
/// Contains all system state information needed for MAVLink communication
/// and command handling.
#[derive(Debug, Clone, Copy)]
pub struct SystemState {
    /// Vehicle arming state
    pub armed: ArmedState,
    /// Current flight mode
    pub mode: FlightMode,
    /// Battery state
    pub battery: BatteryState,
    /// Attitude state from AHRS
    pub attitude: AttitudeState,
    /// GPS position (None if no fix or stale)
    pub gps_position: Option<GpsPosition>,
    /// GPS timestamp (microseconds since boot when position was updated)
    pub gps_timestamp_us: u64,
    /// Home position for RTL (None if not set)
    pub home_position: Option<HomePosition>,
    /// System uptime (microseconds since boot)
    pub uptime_us: u64,
    /// CPU load (percentage, 0.0-100.0)
    pub cpu_load: f32,
    /// Enabled pre-arm check categories bitmask (ARMING_CHECK parameter)
    /// Default: 0xFFFF (all checks enabled)
    pub arming_checks: u16,
    /// Battery voltage multiplier for ADC conversion (BATT_VOLT_MULT parameter)
    /// Default: 3.95 (Freenove voltage divider coefficient)
    pub battery_volt_mult: f32,
    /// Minimum battery voltage to arm (BATT_ARM_VOLT parameter)
    /// Default: 10.5V. Set to 0 to disable battery voltage check.
    pub battery_arm_volt: f32,
    /// Compass yaw offset in radians (from Large Vehicle MagCal)
    /// Applied to AHRS yaw when reporting heading in telemetry.
    /// Set by MAV_CMD_FIXED_MAG_CAL_YAW command.
    pub compass_yaw_offset: f32,
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            armed: ArmedState::Disarmed,
            mode: FlightMode::Manual,
            battery: BatteryState::placeholder(),
            attitude: AttitudeState::placeholder(),
            gps_position: None,
            gps_timestamp_us: 0,
            home_position: None,
            uptime_us: 0,
            cpu_load: 0.0,
            arming_checks: 0xFFFF,
            battery_volt_mult: 3.95,
            battery_arm_volt: 10.5,
            compass_yaw_offset: 0.0,
        }
    }
}

impl SystemState {
    /// Create a new system state with default values (const fn for static initialization)
    pub const fn init() -> Self {
        Self {
            armed: ArmedState::Disarmed,
            mode: FlightMode::Manual,
            battery: BatteryState::placeholder(),
            attitude: AttitudeState::placeholder(),
            gps_position: None,
            gps_timestamp_us: 0,
            home_position: None,
            uptime_us: 0,
            cpu_load: 0.0,
            arming_checks: 0xFFFF,
            battery_volt_mult: 3.95,
            battery_arm_volt: 10.5,
            compass_yaw_offset: 0.0,
        }
    }

    /// Create a new system state with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Check if vehicle is armed
    pub fn is_armed(&self) -> bool {
        self.armed == ArmedState::Armed
    }

    /// Set flight mode
    ///
    /// Returns Ok if mode change is successful, or Err with reason if denied.
    pub fn set_mode(&mut self, mode: FlightMode) -> Result<(), &'static str> {
        self.mode = mode;
        Ok(())
    }

    /// Update battery state from ADC reading
    ///
    /// Converts a raw ADC value to battery voltage using the BATT_VOLT_MULT parameter
    /// stored in battery_volt_mult field, then estimates remaining percentage.
    ///
    /// # Arguments
    ///
    /// * `adc_value` - Raw 12-bit ADC value (0-4095) from battery voltage pin
    pub fn update_battery(&mut self, adc_value: u16) {
        let voltage = BatteryState::voltage_from_adc(adc_value, self.battery_volt_mult);
        self.battery.voltage = voltage;
        self.battery.remaining_percent = BatteryState::estimate_remaining_percent(voltage);
    }

    /// Update battery state from a direct voltage reading (e.g., from SITL).
    ///
    /// Unlike `update_battery()` which takes a raw ADC value, this method
    /// accepts voltage directly — suitable for simulators that provide
    /// pre-converted battery voltage.
    ///
    /// # Arguments
    ///
    /// * `voltage` - Battery voltage in volts
    pub fn update_battery_voltage(&mut self, voltage: f32) {
        self.battery.voltage = voltage;
        self.battery.remaining_percent = BatteryState::estimate_remaining_percent(voltage);
    }

    /// Update system uptime
    pub fn update_uptime(&mut self, uptime_us: u64) {
        self.uptime_us = uptime_us;
    }

    /// Update GPS position and timestamp
    ///
    /// Called by GPS driver after successful NMEA parse.
    pub fn update_gps(&mut self, position: GpsPosition, timestamp_us: u64) {
        self.gps_position = Some(position);
        self.gps_timestamp_us = timestamp_us;
    }

    /// Update attitude state directly from values
    ///
    /// Alternative to firmware-specific `update_attitude()` for cases where
    /// SharedAhrsState is not used (e.g., SITL).
    #[allow(clippy::too_many_arguments)]
    pub fn update_attitude_direct(
        &mut self,
        roll: f32,
        pitch: f32,
        yaw: f32,
        rollspeed: f32,
        pitchspeed: f32,
        yawspeed: f32,
        timestamp_us: u64,
    ) {
        self.attitude.roll = roll;
        self.attitude.pitch = pitch;
        self.attitude.yaw = yaw;
        self.attitude.rollspeed = rollspeed;
        self.attitude.pitchspeed = pitchspeed;
        self.attitude.yawspeed = yawspeed;
        self.attitude.timestamp_us = timestamp_us;
        self.attitude.healthy = true;
    }

    /// Check if GPS data is fresh (updated within threshold)
    ///
    /// GPS data is considered stale if it hasn't been updated within
    /// the specified threshold. Default threshold is 1 second (1_000_000 us).
    pub fn is_gps_fresh(&self, current_time_us: u64, threshold_us: u64) -> bool {
        if self.gps_position.is_none() {
            return false;
        }
        current_time_us.saturating_sub(self.gps_timestamp_us) < threshold_us
    }

    /// Set home position directly
    ///
    /// Used when MAV_CMD_DO_SET_HOME specifies a location (param1 = 0).
    pub fn set_home(&mut self, home: HomePosition) {
        self.home_position = Some(home);
    }

    /// Set home position to current GPS location
    ///
    /// Used when MAV_CMD_DO_SET_HOME requests current location (param1 = 1).
    pub fn set_home_to_current(&mut self) -> Result<(), &'static str> {
        if let Some(gps) = self.gps_position {
            self.home_position = Some(HomePosition::from_gps(&gps));
            Ok(())
        } else {
            Err("No GPS fix available")
        }
    }

    /// Check if home position is set
    pub fn has_home(&self) -> bool {
        self.home_position.is_some()
    }

    /// Arm the vehicle with pre-arm checks and post-arm initialization
    pub fn arm(&mut self) -> Result<(), crate::arming::ArmingError> {
        use crate::arming::{create_default_checker, ArmMethod, PostArmInitializer};

        let checker = create_default_checker(self.arming_checks);
        checker.run_checks(self)?;

        self.armed = ArmedState::Armed;

        let mut initializer = PostArmInitializer::new(self.arming_checks);
        initializer.execute(self, ArmMethod::GcsCommand)?;

        Ok(())
    }

    /// Force-arm the vehicle (bypasses pre-arm checks)
    pub fn arm_forced(&mut self) -> Result<(), crate::arming::ArmingError> {
        use crate::arming::{ArmMethod, ArmingError, PostArmInitializer};

        if self.is_armed() {
            return Err(ArmingError::AlreadyArmed);
        }

        self.armed = ArmedState::Armed;

        let mut initializer = PostArmInitializer::new(self.arming_checks);
        initializer.execute(self, ArmMethod::GcsCommand)?;

        Ok(())
    }

    /// Disarm the vehicle with pre-disarm validation
    pub fn disarm(&mut self) -> Result<(), crate::arming::DisarmError> {
        use crate::arming::{DisarmMethod, DisarmReason, DisarmValidator, PostDisarmCleanup};

        let validator = DisarmValidator::new();
        validator.validate(self, DisarmMethod::GcsCommand, false)?;

        self.armed = ArmedState::Disarmed;

        let mut cleanup = PostDisarmCleanup::new(false);
        cleanup
            .execute(self, DisarmMethod::GcsCommand, DisarmReason::UserRequest)
            .map_err(|_| crate::arming::DisarmError::ValidationFailed {
                reason: "Cleanup failed",
            })?;

        Ok(())
    }

    /// Force-disarm the vehicle (bypasses pre-disarm validation)
    pub fn disarm_forced(&mut self) -> Result<(), crate::arming::DisarmError> {
        use crate::arming::{DisarmMethod, DisarmReason, PostDisarmCleanup};

        if !self.is_armed() {
            return Err(crate::arming::DisarmError::NotArmed);
        }

        self.armed = ArmedState::Disarmed;

        let mut cleanup = PostDisarmCleanup::new(false);
        cleanup
            .execute(self, DisarmMethod::GcsCommand, DisarmReason::UserRequest)
            .map_err(|_| crate::arming::DisarmError::ValidationFailed {
                reason: "Cleanup failed",
            })?;

        Ok(())
    }

    /// Sync parameter-based fields from parameter store
    ///
    /// Reads ARMING_CHECK, BATT_VOLT_MULT, and BATT_ARM_VOLT from the store
    /// and updates the corresponding SystemState fields.
    pub fn sync_from_params(&mut self, param_store: &crate::parameters::ParameterStore) {
        use crate::parameters::ParamValue;

        if let Some(ParamValue::Int(v)) = param_store.get("ARMING_CHECK") {
            self.arming_checks = *v as u16;
        }
        if let Some(ParamValue::Float(v)) = param_store.get("BATT_VOLT_MULT") {
            self.battery_volt_mult = *v;
        }
        if let Some(ParamValue::Float(v)) = param_store.get("BATT_ARM_VOLT") {
            self.battery_arm_volt = *v;
        }
    }
}

/// Global system state (protected by Mutex for multi-task access)
///
/// This allows both the command handler and motor control task to share
/// the same system state, ensuring ARM status changes are visible across tasks.
pub static SYSTEM_STATE: critical_section::Mutex<core::cell::RefCell<SystemState>> =
    critical_section::Mutex::new(core::cell::RefCell::new(SystemState::init()));

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::GpsFixType;

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
    fn test_flight_mode_base_mode_flags() {
        // All modes should have CUSTOM_MODE_ENABLED set
        let custom_enabled = mav_mode_flag::CUSTOM_MODE_ENABLED;

        let flags = FlightMode::Manual.to_base_mode_flags();
        assert!(flags & custom_enabled != 0);
        assert!(flags & mav_mode_flag::MANUAL_INPUT_ENABLED != 0);

        let flags = FlightMode::Guided.to_base_mode_flags();
        assert!(flags & custom_enabled != 0);
        assert!(flags & mav_mode_flag::GUIDED_ENABLED != 0);
        assert!(flags & mav_mode_flag::STABILIZE_ENABLED != 0);

        let flags = FlightMode::Auto.to_base_mode_flags();
        assert!(flags & custom_enabled != 0);
        assert!(flags & mav_mode_flag::AUTO_ENABLED != 0);

        let flags = FlightMode::Circle.to_base_mode_flags();
        assert!(flags & custom_enabled != 0);
        assert!(flags & mav_mode_flag::STABILIZE_ENABLED != 0);
        assert!(flags & mav_mode_flag::AUTO_ENABLED != 0);

        let flags = FlightMode::SmartRtl.to_base_mode_flags();
        assert!(flags & custom_enabled != 0);
        assert!(flags & mav_mode_flag::STABILIZE_ENABLED != 0);
        assert!(flags & mav_mode_flag::AUTO_ENABLED != 0);
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
    fn test_voltage_from_adc() {
        assert_eq!(BatteryState::voltage_from_adc(0, 3.95), 0.0);

        let voltage = BatteryState::voltage_from_adc(4095, 3.95);
        assert!((voltage - 13.035).abs() < 0.01);

        let voltage = BatteryState::voltage_from_adc(3000, 3.95);
        assert!((voltage - 9.549).abs() < 0.01);
    }

    #[test]
    fn test_estimate_remaining_percent() {
        // 3S LiPo
        assert_eq!(BatteryState::estimate_remaining_percent(12.6), 100);
        assert_eq!(BatteryState::estimate_remaining_percent(9.0), 0);
        assert_eq!(BatteryState::estimate_remaining_percent(10.8), 50);
        assert_eq!(BatteryState::estimate_remaining_percent(13.0), 100);

        // 2S LiPo
        assert_eq!(BatteryState::estimate_remaining_percent(8.4), 100);
        assert_eq!(BatteryState::estimate_remaining_percent(6.0), 0);
        assert_eq!(BatteryState::estimate_remaining_percent(7.2), 50);
    }

    #[test]
    fn test_system_state_default() {
        let state = SystemState::new();
        assert!(!state.is_armed());
        assert_eq!(state.mode, FlightMode::Manual);
        assert!(!state.has_home());
    }

    #[test]
    fn test_mode_change() {
        let mut state = SystemState::new();
        assert_eq!(state.mode, FlightMode::Manual);

        assert!(state.set_mode(FlightMode::Auto).is_ok());
        assert_eq!(state.mode, FlightMode::Auto);
    }

    #[test]
    fn test_gps_position_creation() {
        let pos = GpsPosition {
            latitude: 35.6762,
            longitude: 139.6503,
            altitude: 40.0,
            speed: 1.5,
            course_over_ground: Some(45.0),
            fix_type: GpsFixType::Fix3D,
            satellites: 12,
        };
        assert!((pos.latitude - 35.6762).abs() < 0.0001);
        assert_eq!(pos.fix_type, GpsFixType::Fix3D);
    }

    #[test]
    fn test_home_position_from_gps() {
        let gps = GpsPosition {
            latitude: 35.6762,
            longitude: 139.6503,
            altitude: 40.0,
            speed: 0.0,
            course_over_ground: None,
            fix_type: GpsFixType::Fix3D,
            satellites: 10,
        };
        let home = HomePosition::from_gps(&gps);
        assert!((home.latitude - 35.6762).abs() < 0.0001);
        assert!((home.longitude - 139.6503).abs() < 0.0001);
        assert!((home.altitude - 40.0).abs() < 0.1);
    }

    #[test]
    fn test_set_home_to_current() {
        let mut state = SystemState::new();
        assert!(state.set_home_to_current().is_err());

        state.update_gps(
            GpsPosition {
                latitude: 35.6762,
                longitude: 139.6503,
                altitude: 40.0,
                speed: 0.0,
                course_over_ground: None,
                fix_type: GpsFixType::Fix3D,
                satellites: 10,
            },
            1_000_000,
        );
        assert!(state.set_home_to_current().is_ok());
        assert!(state.has_home());
    }

    #[test]
    fn test_attitude_freshness() {
        let mut attitude = AttitudeState::placeholder();
        assert!(!attitude.is_fresh(1_000_000, 500_000));

        attitude.healthy = true;
        attitude.timestamp_us = 900_000;
        assert!(attitude.is_fresh(1_000_000, 500_000));
        assert!(!attitude.is_fresh(2_000_000, 500_000));
    }
}
