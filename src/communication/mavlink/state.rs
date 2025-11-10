//! System State Management
//!
//! Manages autopilot system state for MAVLink telemetry and command handling.
//!
//! # State Components
//!
//! - **Armed State**: Vehicle arming status (disarmed/armed)
//! - **Flight Mode**: Current flight mode (Stabilize, Loiter, Auto, etc.)
//! - **Battery**: Battery voltage and current
//! - **System Status**: Overall system health and status
//!
//! # Thread Safety
//!
//! System state is designed to be accessed from multiple tasks:
//! - Command handler updates armed state
//! - Telemetry task reads state for streaming
//! - AHRS/Control loops update mode and status
//!
//! Use appropriate synchronization primitives (Mutex, critical sections)
//! when accessing state from multiple tasks.

#[cfg(feature = "defmt")]
use defmt::warn;

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
    /// System uptime (microseconds since boot)
    pub uptime_us: u64,
    /// CPU load (percentage, 0.0-100.0)
    pub cpu_load: f32,
    /// Enabled pre-arm check categories bitmask (ARMING_CHECK parameter)
    /// Default: 0xFFFF (all checks enabled)
    pub arming_checks: u16,
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            armed: ArmedState::Disarmed,
            mode: FlightMode::Manual,
            battery: BatteryState::placeholder(),
            uptime_us: 0,
            cpu_load: 0.0,
            arming_checks: 0xFFFF, // All checks enabled by default
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
            uptime_us: 0,
            cpu_load: 0.0,
            arming_checks: 0xFFFF, // All checks enabled by default
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
    ///
    /// Future parameters (not yet implemented):
    /// - INITIAL_MODE - Initial flight mode
    /// - BATT_* - Battery configuration
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
        let arming_params = ArmingParams::from_store(param_store);

        Self {
            armed: ArmedState::Disarmed,
            mode: FlightMode::Manual,
            battery: BatteryState::placeholder(),
            uptime_us: 0,
            cpu_load: 0.0,
            arming_checks: arming_params.check_bitmask as u16,
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

    /// Update battery state
    pub fn update_battery(&mut self, voltage: f32, current: f32, remaining_percent: u8) {
        self.battery.voltage = voltage;
        self.battery.current = current;
        self.battery.remaining_percent = remaining_percent;
    }

    /// Update system uptime
    pub fn update_uptime(&mut self, uptime_us: u64) {
        self.uptime_us = uptime_us;
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
        let mut state = SystemState::new();
        state.update_battery(11.5, 2.5, 75);
        assert_eq!(state.battery.voltage, 11.5);
        assert_eq!(state.battery.current, 2.5);
        assert_eq!(state.battery.remaining_percent, 75);
    }
}
