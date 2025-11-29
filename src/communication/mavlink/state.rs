//! System State Management
//!
//! Manages autopilot system state for MAVLink telemetry and command handling.
//!
//! # State Components
//!
//! - **Armed State**: Vehicle arming status (disarmed/armed)
//! - **Flight Mode**: Current flight mode (Stabilize, Loiter, Auto, etc.)
//! - **Battery**: Battery voltage and current
//! - **GPS**: GPS position and timestamp
//! - **System Status**: Overall system health and status
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

use crate::devices::gps::GpsPosition;

#[cfg(feature = "defmt")]
use defmt::warn;

/// Trait for reading battery ADC values
///
/// This trait provides a minimal interface for battery voltage monitoring,
/// allowing different implementations (hardware ADC, stub, mock) to be used
/// with SystemState::update_battery().
pub trait BatteryAdcReader {
    /// Read raw battery ADC value
    ///
    /// # Returns
    ///
    /// Raw 12-bit ADC value (0-4095) representing voltage at ADC pin (0-3.3V range)
    fn read_battery_adc(&mut self) -> u16;
}

/// Blanket implementation for Platform trait
///
/// Any type implementing Platform automatically implements BatteryAdcReader
impl<P: crate::platform::traits::Platform> BatteryAdcReader for P {
    fn read_battery_adc(&mut self) -> u16 {
        crate::platform::traits::Platform::read_battery_adc(self)
    }
}

// Stub macro when defmt is not available
#[cfg(not(feature = "defmt"))]
macro_rules! warn {
    ($($arg:tt)*) => {{}};
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
    /// Loiter mode (position hold)
    Loiter,
    /// Auto mode (following waypoints)
    Auto,
    /// Return to launch
    Rtl,
}

impl Default for FlightMode {
    fn default() -> Self {
        Self::Manual
    }
}

impl FlightMode {
    /// Convert to MAVLink custom mode number
    ///
    /// This mapping follows ArduPilot's mode numbering for rovers.
    pub fn to_custom_mode(self) -> u32 {
        match self {
            FlightMode::Manual => 0,
            FlightMode::Stabilize => 1,
            FlightMode::Loiter => 2,
            FlightMode::Auto => 3,
            FlightMode::Rtl => 4,
        }
    }

    /// Convert from MAVLink custom mode number
    ///
    /// Returns None if mode number is unrecognized.
    pub fn from_custom_mode(mode: u32) -> Option<Self> {
        match mode {
            0 => Some(FlightMode::Manual),
            1 => Some(FlightMode::Stabilize),
            2 => Some(FlightMode::Loiter),
            3 => Some(FlightMode::Auto),
            4 => Some(FlightMode::Rtl),
            _ => None,
        }
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
    /// Where:
    /// - `adc`: Raw ADC value (0-4095 for 12-bit ADC)
    /// - `4095.0`: Maximum ADC count (12-bit)
    /// - `3.3`: ADC reference voltage (V)
    /// - `mult`: Voltage divider coefficient (BATT_VOLT_MULT, typically 3.95 for Freenove)
    ///
    /// # Arguments
    ///
    /// * `adc` - Raw ADC value (0-4095)
    /// * `mult` - Voltage multiplier from BATT_VOLT_MULT parameter
    ///
    /// # Returns
    ///
    /// Battery voltage in volts
    ///
    /// # Examples
    ///
    /// ```
    /// # use pico_trail::communication::mavlink::state::BatteryState;
    /// // ADC reading of 3000 with multiplier 3.95
    /// let voltage = BatteryState::voltage_from_adc(3000, 3.95);
    /// assert!((voltage - 9.549).abs() < 0.01);
    /// ```
    pub fn voltage_from_adc(adc: u16, mult: f32) -> f32 {
        (adc as f32 / 4095.0 * 3.3) * mult
    }

    /// Estimate battery remaining percentage from voltage
    ///
    /// Estimates remaining battery capacity based on voltage using linear interpolation.
    /// Automatically detects battery cell count (2S or 3S) from voltage range.
    ///
    /// # Supported Battery Types
    ///
    /// - **2S LiPo**: 6.0V (empty) to 8.4V (full)
    ///   - Full charge: 8.4V (4.2V per cell) = 100%
    ///   - Nominal: 7.4V (3.7V per cell) = ~50%
    ///   - Empty: 6.0V (3.0V per cell) = 0%
    ///
    /// - **3S LiPo**: 9.0V (empty) to 12.6V (full)
    ///   - Full charge: 12.6V (4.2V per cell) = 100%
    ///   - Nominal: 11.1V (3.7V per cell) = ~50%
    ///   - Empty: 9.0V (3.0V per cell) = 0%
    ///
    /// # Detection Logic
    ///
    /// - Voltage >= 8.7V → 3S battery (9.0V-12.6V range)
    /// - Voltage < 8.7V → 2S battery (6.0V-8.4V range)
    ///
    /// # Arguments
    ///
    /// * `voltage` - Battery voltage in volts
    ///
    /// # Returns
    ///
    /// Estimated remaining capacity as percentage (0-100)
    ///
    /// # Examples
    ///
    /// ```
    /// # use pico_trail::communication::mavlink::state::BatteryState;
    /// // 3S LiPo examples
    /// assert_eq!(BatteryState::estimate_remaining_percent(12.6), 100);
    /// assert_eq!(BatteryState::estimate_remaining_percent(9.0), 0);
    /// assert_eq!(BatteryState::estimate_remaining_percent(10.8), 50);
    ///
    /// // 2S LiPo examples
    /// assert_eq!(BatteryState::estimate_remaining_percent(8.4), 100);
    /// assert_eq!(BatteryState::estimate_remaining_percent(6.0), 0);
    /// assert_eq!(BatteryState::estimate_remaining_percent(7.2), 50);
    /// ```
    pub fn estimate_remaining_percent(voltage: f32) -> u8 {
        // Detect battery type from voltage range
        // Threshold at 8.7V (midpoint between 2S max 8.4V and 3S min 9.0V)
        const DETECTION_THRESHOLD: f32 = 8.7;

        let (voltage_full, voltage_empty) = if voltage >= DETECTION_THRESHOLD {
            // 3S LiPo battery
            (12.6, 9.0) // 4.2V × 3 cells, 3.0V × 3 cells
        } else {
            // 2S LiPo battery
            (8.4, 6.0) // 4.2V × 2 cells, 3.0V × 2 cells
        };

        if voltage >= voltage_full {
            100
        } else if voltage <= voltage_empty {
            0
        } else {
            // Linear interpolation: percent = (voltage - empty) / (full - empty) * 100
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
    /// GPS position (None if no fix or stale)
    pub gps_position: Option<GpsPosition>,
    /// GPS timestamp (microseconds since boot when position was updated)
    pub gps_timestamp_us: u64,
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
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            armed: ArmedState::Disarmed,
            mode: FlightMode::Manual,
            battery: BatteryState::placeholder(),
            gps_position: None,
            gps_timestamp_us: 0,
            uptime_us: 0,
            cpu_load: 0.0,
            arming_checks: 0xFFFF, // All checks enabled by default
            battery_volt_mult: 3.95,
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
            gps_position: None,
            gps_timestamp_us: 0,
            uptime_us: 0,
            cpu_load: 0.0,
            arming_checks: 0xFFFF, // All checks enabled by default
            battery_volt_mult: 3.95,
        }
    }

    /// Create a new system state with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a new system state with configuration loaded from parameter store
    ///
    /// Initializes SystemState with all configurable parameters from the parameter store.
    /// Runtime state fields (armed, uptime_us, cpu_load) are set to default values.
    ///
    /// Currently loaded parameters:
    /// - ARMING_CHECK - Pre-arm check bitmask (stored in arming_checks field)
    /// - BATT_VOLT_MULT - Battery voltage multiplier (stored in battery_volt_mult field)
    ///
    /// Future parameters (not yet implemented):
    /// - INITIAL_MODE - Initial flight mode
    /// - Other battery configuration (BATT_CRT_VOLT, BATT_ARM_VOLT)
    /// - Other system configuration parameters
    ///
    /// # Arguments
    ///
    /// * `param_store` - Parameter store to read configuration from
    ///
    /// # Returns
    ///
    /// SystemState with configurable fields loaded from parameter store
    pub fn from_param_store(param_store: &crate::parameters::storage::ParameterStore) -> Self {
        use crate::parameters::arming::ArmingParams;
        use crate::parameters::battery::BatteryParams;

        let arming_params = ArmingParams::from_store(param_store);
        let battery_params = BatteryParams::from_store(param_store);

        Self {
            armed: ArmedState::Disarmed,
            mode: FlightMode::Manual,
            battery: BatteryState::placeholder(),
            gps_position: None,
            gps_timestamp_us: 0,
            uptime_us: 0,
            cpu_load: 0.0,
            arming_checks: arming_params.check_bitmask as u16,
            battery_volt_mult: battery_params.volt_mult,
        }
    }

    /// Check if vehicle is armed
    pub fn is_armed(&self) -> bool {
        self.armed == ArmedState::Armed
    }

    /// Arm the vehicle
    ///
    /// Executes pre-arm checks and post-arm initialization sequence.
    ///
    /// Returns Ok if arming is successful, or Err with reason if denied.
    ///
    /// # Pre-arm Checks
    ///
    /// Validates system health before allowing arming. Checks are controlled
    /// by the `arming_checks` bitmask (ARMING_CHECK parameter).
    ///
    /// # Post-arm Initialization
    ///
    /// After successful arming:
    /// - Records arm timestamp
    /// - Logs arm event
    /// - Initializes actuators (placeholder)
    /// - Notifies subsystems (placeholder)
    /// - Enables geofence if configured (placeholder)
    pub fn arm(&mut self) -> Result<(), crate::core::arming::ArmingError> {
        use crate::core::arming::{create_default_checker, ArmMethod, PostArmInitializer};

        // Run pre-arm checks
        let checker = create_default_checker(self.arming_checks);
        checker.run_checks(self)?;

        // Set armed state
        self.armed = ArmedState::Armed;

        // Execute post-arm initialization
        let mut initializer = PostArmInitializer::new(self.arming_checks);
        initializer.execute(self, ArmMethod::GcsCommand)?;

        Ok(())
    }

    /// Force-arm the vehicle (bypasses pre-arm checks)
    ///
    /// **WARNING**: This bypasses all safety checks and should only be used
    /// in emergency situations or for bench testing. This action is logged
    /// with WARNING severity for audit trail.
    ///
    /// # Arguments
    ///
    /// None - force-arm always bypasses all checks regardless of ARMING_CHECK parameter
    ///
    /// # Returns
    ///
    /// Ok if arming is successful, or Err if already armed or initialization fails.
    ///
    /// # Audit Trail
    ///
    /// All force-arm operations are logged with:
    /// - WARNING severity
    /// - "FORCE ARM" audit marker
    /// - Timestamp and uptime
    pub fn arm_forced(&mut self) -> Result<(), crate::core::arming::ArmingError> {
        use crate::core::arming::{ArmMethod, ArmingError, PostArmInitializer};

        // Only check if already armed
        if self.is_armed() {
            return Err(ArmingError::AlreadyArmed);
        }

        // Log force-arm with WARNING for audit trail
        warn!("FORCE ARM: Bypassing all pre-arm checks (param2=21196)");

        // Set armed state
        self.armed = ArmedState::Armed;

        // Execute post-arm initialization
        let mut initializer = PostArmInitializer::new(self.arming_checks);
        initializer.execute(self, ArmMethod::GcsCommand)?;

        Ok(())
    }

    /// Disarm the vehicle
    ///
    /// Executes pre-disarm validation and post-disarm cleanup sequence.
    ///
    /// Returns Ok if disarming is successful, or Err with reason if denied.
    ///
    /// # Pre-disarm Validation
    ///
    /// Validates safety conditions before allowing disarm:
    /// - Throttle low (< 10%)
    /// - Velocity safe (< 0.5 m/s for RC stick disarm)
    ///
    /// # Post-disarm Cleanup
    ///
    /// After successful disarm:
    /// - Logs disarm event
    /// - Verifies actuators neutral (placeholder)
    /// - Notifies subsystems (placeholder)
    /// - Disables geofence if auto-enabled (placeholder)
    /// - Persists configuration changes (placeholder)
    pub fn disarm(&mut self) -> Result<(), crate::core::arming::DisarmError> {
        use crate::core::arming::{DisarmMethod, DisarmReason, DisarmValidator, PostDisarmCleanup};

        // Run pre-disarm validation
        let validator = DisarmValidator::new();
        validator.validate(self, DisarmMethod::GcsCommand, false)?;

        // Set disarmed state
        self.armed = ArmedState::Disarmed;

        // Execute post-disarm cleanup
        let mut cleanup = PostDisarmCleanup::new(false); // TODO: Track FENCE_AUTOENABLE
        cleanup
            .execute(self, DisarmMethod::GcsCommand, DisarmReason::UserRequest)
            .map_err(|_| crate::core::arming::DisarmError::ValidationFailed {
                reason: "Cleanup failed",
            })?;

        Ok(())
    }

    /// Force-disarm the vehicle (bypasses pre-disarm validation)
    ///
    /// **WARNING**: This bypasses all safety checks and should only be used
    /// in emergency situations. This action is logged with WARNING severity
    /// for audit trail.
    ///
    /// # Arguments
    ///
    /// None - force-disarm always bypasses all checks
    ///
    /// # Returns
    ///
    /// Ok if disarming is successful, or Err if not armed or cleanup fails.
    ///
    /// # Audit Trail
    ///
    /// All force-disarm operations are logged with:
    /// - WARNING severity
    /// - "FORCE DISARM" audit marker
    /// - Timestamp and uptime
    pub fn disarm_forced(&mut self) -> Result<(), crate::core::arming::DisarmError> {
        use crate::core::arming::{DisarmMethod, DisarmReason, PostDisarmCleanup};

        // Only check if armed
        if !self.is_armed() {
            return Err(crate::core::arming::DisarmError::NotArmed);
        }

        // Log force-disarm with WARNING for audit trail
        warn!("FORCE DISARM: Bypassing all pre-disarm validation (param2=21196)");

        // Set disarmed state
        self.armed = ArmedState::Disarmed;

        // Execute post-disarm cleanup (still need to clean up even with forced disarm)
        let mut cleanup = PostDisarmCleanup::new(false); // TODO: Track FENCE_AUTOENABLE
        cleanup
            .execute(self, DisarmMethod::GcsCommand, DisarmReason::UserRequest)
            .map_err(|_| crate::core::arming::DisarmError::ValidationFailed {
                reason: "Cleanup failed",
            })?;

        Ok(())
    }

    /// Set flight mode
    ///
    /// Returns Ok if mode change is successful, or Err with reason if denied.
    pub fn set_mode(&mut self, mode: FlightMode) -> Result<(), &'static str> {
        // Mode change restrictions can be added here
        // For now, allow all mode changes

        self.mode = mode;
        Ok(())
    }

    /// Update battery state from ADC reading
    ///
    /// Reads raw ADC value from the ADC reader and converts it to battery voltage
    /// using the BATT_VOLT_MULT parameter stored in battery_volt_mult field.
    /// Also estimates remaining battery percentage based on voltage.
    ///
    /// # Arguments
    ///
    /// * `adc_reader` - Any type implementing BatteryAdcReader trait
    ///
    /// # Notes
    ///
    /// - Reads GPIO 26 (ADC0) via BatteryAdcReader::read_battery_adc()
    /// - Converts using BatteryState::voltage_from_adc()
    /// - Estimates remaining percentage using BatteryState::estimate_remaining_percent()
    /// - Current is not updated (no current sensor available)
    /// - On ADC read failure (returns 0), voltage is set to 0.0 (caller should handle gracefully)
    pub fn update_battery<R: BatteryAdcReader>(&mut self, adc_reader: &mut R) {
        let adc_value = adc_reader.read_battery_adc();
        let voltage = BatteryState::voltage_from_adc(adc_value, self.battery_volt_mult);
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
    ///
    /// # Arguments
    ///
    /// * `position` - GPS position from NMEA parser
    /// * `timestamp_us` - Current system uptime in microseconds
    pub fn update_gps(&mut self, position: GpsPosition, timestamp_us: u64) {
        self.gps_position = Some(position);
        self.gps_timestamp_us = timestamp_us;
    }

    /// Check if GPS data is fresh (updated within threshold)
    ///
    /// GPS data is considered stale if it hasn't been updated within
    /// the specified threshold. Default threshold is 1 second (1_000_000 us).
    ///
    /// # Arguments
    ///
    /// * `current_time_us` - Current system uptime in microseconds
    /// * `threshold_us` - Maximum age for GPS data to be considered fresh
    ///
    /// # Returns
    ///
    /// true if GPS position exists and was updated within threshold
    pub fn is_gps_fresh(&self, current_time_us: u64, threshold_us: u64) -> bool {
        if self.gps_position.is_none() {
            return false;
        }
        current_time_us.saturating_sub(self.gps_timestamp_us) < threshold_us
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

    #[test]
    fn test_armed_state_default() {
        let state = ArmedState::default();
        assert_eq!(state, ArmedState::Disarmed);
    }

    #[test]
    fn test_flight_mode_conversion() {
        assert_eq!(FlightMode::Manual.to_custom_mode(), 0);
        assert_eq!(FlightMode::Auto.to_custom_mode(), 3);
        assert_eq!(FlightMode::from_custom_mode(0), Some(FlightMode::Manual));
        assert_eq!(FlightMode::from_custom_mode(3), Some(FlightMode::Auto));
        assert_eq!(FlightMode::from_custom_mode(99), None);
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
        let voltage = BatteryState::voltage_from_adc(0, 3.95);
        assert_eq!(voltage, 0.0);
    }

    #[test]
    fn test_voltage_from_adc_max() {
        let voltage = BatteryState::voltage_from_adc(4095, 3.95);
        // 4095 / 4095.0 * 3.3 * 3.95 = 13.035
        assert!((voltage - 13.035).abs() < 0.01);
    }

    #[test]
    fn test_voltage_from_adc_typical() {
        // Typical reading: 3000 ADC counts
        let voltage = BatteryState::voltage_from_adc(3000, 3.95);
        // 3000 / 4095.0 * 3.3 * 3.95 = 9.549
        assert!((voltage - 9.549).abs() < 0.01);
    }

    #[test]
    fn test_voltage_from_adc_different_multipliers() {
        let adc = 3000_u16;

        // Test with different multipliers
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

        // Arm with good battery
        state.battery.voltage = 12.0;
        assert!(state.arm().is_ok());
        assert!(state.is_armed());

        // Can't arm again
        assert!(state.arm().is_err());

        // Disarm
        assert!(state.disarm().is_ok());
        assert!(!state.is_armed());

        // Can't disarm again
        assert!(state.disarm().is_err());
    }

    #[test]
    fn test_arm_with_low_battery() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0; // Critical

        let result = state.arm();
        assert!(result.is_err());

        // Check that it's a battery check failure
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
        state.battery.voltage = 9.0; // Critical - would fail normal arm

        // Force-arm should succeed even with low battery
        let result = state.arm_forced();
        assert!(result.is_ok());
        assert!(state.is_armed());
    }

    #[test]
    fn test_arm_forced_already_armed() {
        let mut state = SystemState::new();
        state.battery.voltage = 12.0;

        // First arm succeeds
        assert!(state.arm().is_ok());

        // Second force-arm fails (already armed)
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

        // Arm first
        assert!(state.arm().is_ok());
        assert!(state.is_armed());

        // Force-disarm should succeed
        let result = state.disarm_forced();
        assert!(result.is_ok());
        assert!(!state.is_armed());
    }

    #[test]
    fn test_disarm_forced_not_armed() {
        let mut state = SystemState::new();

        // Force-disarm when not armed should fail
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
        use crate::platform::mock::MockPlatform;

        let mut platform = MockPlatform::new();
        platform.set_battery_adc_value(3000); // Typical ADC reading

        let mut state = SystemState::new();
        state.battery_volt_mult = 3.95;

        state.update_battery(&mut platform);

        // 3000 / 4095.0 * 3.3 * 3.95 = 9.549V
        assert!((state.battery.voltage - 9.549).abs() < 0.01);
        // At 9.549V, remaining should be ~15%
        assert!(state.battery.remaining_percent > 0);
        assert!(state.battery.remaining_percent < 30);
    }

    #[test]
    fn test_estimate_remaining_full() {
        let remaining = BatteryState::estimate_remaining_percent(12.6);
        assert_eq!(remaining, 100);
    }

    #[test]
    fn test_estimate_remaining_empty() {
        let remaining = BatteryState::estimate_remaining_percent(9.0);
        assert_eq!(remaining, 0);
    }

    #[test]
    fn test_estimate_remaining_mid() {
        // 10.8V should be exactly 50%
        // (10.8 - 9.0) / (12.6 - 9.0) * 100 = 1.8 / 3.6 * 100 = 50
        let remaining = BatteryState::estimate_remaining_percent(10.8);
        assert_eq!(remaining, 50);
    }

    #[test]
    fn test_estimate_remaining_nominal() {
        // 11.1V (3.7V per cell, nominal voltage)
        let remaining = BatteryState::estimate_remaining_percent(11.1);
        // (11.1 - 9.0) / 3.6 * 100 = 58.33%
        assert_eq!(remaining, 58);
    }

    #[test]
    fn test_estimate_remaining_low() {
        // 10.5V (3.5V per cell, low voltage warning)
        let remaining = BatteryState::estimate_remaining_percent(10.5);
        // (10.5 - 9.0) / 3.6 * 100 = 41.66%
        assert_eq!(remaining, 41);
    }

    #[test]
    fn test_estimate_remaining_above_full() {
        // Values above full should clamp to 100
        let remaining = BatteryState::estimate_remaining_percent(13.0);
        assert_eq!(remaining, 100);
    }

    #[test]
    fn test_estimate_remaining_3s_below_empty() {
        // 3S battery: Values below 9.0V empty should clamp to 0
        let remaining = BatteryState::estimate_remaining_percent(8.8);
        assert_eq!(remaining, 0);
    }

    // 2S LiPo battery tests
    #[test]
    fn test_estimate_remaining_2s_full() {
        // 2S battery: 8.4V (4.2V per cell) = 100%
        let remaining = BatteryState::estimate_remaining_percent(8.4);
        assert_eq!(remaining, 100);
    }

    #[test]
    fn test_estimate_remaining_2s_empty() {
        // 2S battery: 6.0V (3.0V per cell) = 0%
        let remaining = BatteryState::estimate_remaining_percent(6.0);
        assert_eq!(remaining, 0);
    }

    #[test]
    fn test_estimate_remaining_2s_mid() {
        // 2S battery: 7.2V should be exactly 50%
        // (7.2 - 6.0) / (8.4 - 6.0) * 100 = 1.2 / 2.4 * 100 = 50
        let remaining = BatteryState::estimate_remaining_percent(7.2);
        assert_eq!(remaining, 50);
    }

    #[test]
    fn test_estimate_remaining_2s_nominal() {
        // 2S battery: 7.4V (3.7V per cell, nominal voltage)
        let remaining = BatteryState::estimate_remaining_percent(7.4);
        // (7.4 - 6.0) / 2.4 * 100 = 58.33%
        assert_eq!(remaining, 58);
    }

    #[test]
    fn test_estimate_remaining_2s_typical() {
        // 2S battery: 7.827V (typical reading from user's battery)
        let remaining = BatteryState::estimate_remaining_percent(7.827);
        // (7.827 - 6.0) / 2.4 * 100 = 76.125%
        assert_eq!(remaining, 76);
    }

    #[test]
    fn test_estimate_remaining_2s_above_full() {
        // 2S battery: Values above 8.4V full should clamp to 100
        let remaining = BatteryState::estimate_remaining_percent(8.6);
        assert_eq!(remaining, 100);
    }

    #[test]
    fn test_estimate_remaining_2s_below_empty() {
        // 2S battery: Values below 6.0V empty should clamp to 0
        let remaining = BatteryState::estimate_remaining_percent(5.5);
        assert_eq!(remaining, 0);
    }
}
