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
    pub fn placeholder() -> Self {
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
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            armed: ArmedState::Disarmed,
            mode: FlightMode::Manual,
            battery: BatteryState::placeholder(),
            uptime_us: 0,
            cpu_load: 0.0,
        }
    }
}

impl SystemState {
    /// Create a new system state with default values
    pub fn new() -> Self {
        Self::default()
    }

    /// Check if vehicle is armed
    pub fn is_armed(&self) -> bool {
        self.armed == ArmedState::Armed
    }

    /// Arm the vehicle
    ///
    /// Returns Ok if arming is successful, or Err with reason if denied.
    pub fn arm(&mut self) -> Result<(), &'static str> {
        if self.is_armed() {
            return Err("Already armed");
        }

        if self.battery.is_critical() {
            return Err("Battery voltage too low");
        }

        self.armed = ArmedState::Armed;
        Ok(())
    }

    /// Disarm the vehicle
    ///
    /// Returns Ok if disarming is successful, or Err with reason if denied.
    pub fn disarm(&mut self) -> Result<(), &'static str> {
        if !self.is_armed() {
            return Err("Already disarmed");
        }

        self.armed = ArmedState::Disarmed;
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
        let mut battery = BatteryState::default();
        battery.voltage = 12.0;
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
        assert_eq!(result.unwrap_err(), "Battery voltage too low");
        assert!(!state.is_armed());
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
