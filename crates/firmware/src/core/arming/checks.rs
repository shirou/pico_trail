//! Pre-Arm Checks
//!
//! Defines the PreArmCheck trait and built-in checks for system validation
//! before arming.

use crate::communication::mavlink::state::SystemState;
use pico_trail_core::arming::{ArmingError, CheckCategory};

/// Result of a pre-arm check execution
pub type CheckResult = Result<(), ArmingError>;

/// Pre-arm check trait
///
/// Implement this trait to create custom pre-arm checks that validate
/// system health before allowing arming.
///
/// # Example
///
/// ```ignore
/// struct MyCustomCheck;
///
/// impl PreArmCheck for MyCustomCheck {
///     fn check(&self, state: &SystemState) -> CheckResult {
///         if some_condition {
///             Ok(())
///         } else {
///             Err(ArmingError::CheckFailed {
///                 reason: "Custom check failed",
///                 category: CheckCategory::System,
///             })
///         }
///     }
///
///     fn name(&self) -> &'static str {
///         "My Custom Check"
///     }
///
///     fn category(&self) -> CheckCategory {
///         CheckCategory::System
///     }
/// }
/// ```
pub trait PreArmCheck {
    /// Execute the check
    ///
    /// Returns Ok if the check passes, or Err with reason if it fails.
    fn check(&self, state: &SystemState) -> CheckResult;

    /// Get the check name for logging and error reporting
    fn name(&self) -> &'static str;

    /// Get the check category for selective enabling via ARMING_CHECK
    fn category(&self) -> CheckCategory;
}

/// Arming checker that orchestrates pre-arm check execution
///
/// Manages a collection of pre-arm checks and executes them based on
/// the ARMING_CHECK parameter bitmask.
pub struct ArmingChecker {
    /// Registered checks
    checks: heapless::Vec<&'static dyn PreArmCheck, 16>,
    /// Enabled categories bitmask (from ARMING_CHECK parameter)
    enabled_categories: u16,
}

impl ArmingChecker {
    /// Create a new arming checker with the given enabled categories bitmask
    ///
    /// # Arguments
    ///
    /// * `enabled_categories` - Bitmask of enabled check categories (ARMING_CHECK parameter)
    pub fn new(enabled_categories: u16) -> Self {
        Self {
            checks: heapless::Vec::new(),
            enabled_categories,
        }
    }

    /// Register a pre-arm check
    ///
    /// Returns Err if the maximum number of checks (16) has been reached.
    pub fn register(&mut self, check: &'static dyn PreArmCheck) -> Result<(), &'static str> {
        self.checks
            .push(check)
            .map_err(|_| "Maximum number of checks (16) reached")
    }

    /// Run all enabled pre-arm checks
    ///
    /// Executes checks sequentially, short-circuiting on first failure.
    /// Only runs checks whose category is enabled in the bitmask.
    ///
    /// Returns Ok if all enabled checks pass, or Err with the first failure.
    pub fn run_checks(&self, state: &SystemState) -> CheckResult {
        // If ARMING_CHECK is 0, all checks are disabled (bench testing only)
        if self.enabled_categories == 0 {
            crate::log_warn!("ARMING_CHECK=0: All pre-arm checks disabled (bench testing only)");
            return Ok(());
        }

        crate::log_info!(
            "Running pre-arm checks (enabled: 0x{:04X})",
            self.enabled_categories
        );

        let mut _executed_count = 0;
        for check in &self.checks {
            // Skip checks that are not enabled
            if !check.category().is_enabled(self.enabled_categories) {
                crate::log_debug!("Pre-arm check skipped (disabled): {}", check.name());
                continue;
            }

            // Execute check
            crate::log_info!("Pre-arm check: {}", check.name());
            check.check(state)?;
            _executed_count += 1;
        }

        crate::log_info!("All pre-arm checks passed ({} executed)", _executed_count);
        Ok(())
    }

    /// Get the list of enabled check categories as a comma-separated string
    pub fn enabled_categories_str(&self) -> heapless::String<128> {
        use core::fmt::Write;
        let mut s = heapless::String::new();

        if self.enabled_categories == 0 {
            let _ = write!(s, "None");
            return s;
        }

        if self.enabled_categories == 0xFFFF {
            let _ = write!(s, "All");
            return s;
        }

        let mut first = true;
        for category in [
            CheckCategory::Barometer,
            CheckCategory::Compass,
            CheckCategory::Gps,
            CheckCategory::Ins,
            CheckCategory::Parameters,
            CheckCategory::RcChannels,
            CheckCategory::Board,
            CheckCategory::Battery,
            CheckCategory::Airspeed,
            CheckCategory::Logging,
            CheckCategory::Switch,
            CheckCategory::GpsCfg,
            CheckCategory::System,
        ] {
            if category.is_enabled(self.enabled_categories) {
                if !first {
                    let _ = write!(s, ", ");
                }
                let _ = write!(s, "{}", category);
                first = false;
            }
        }

        s
    }
}

// ============================================================================
// Built-in Pre-Arm Checks
// ============================================================================

/// Battery voltage check
///
/// Verifies that battery voltage is above the minimum arming threshold.
/// Uses the BATT_ARM_VOLT parameter from SystemState.
///
/// If BATT_ARM_VOLT is 0, the battery voltage check is disabled.
pub struct BatteryVoltageCheck;

impl PreArmCheck for BatteryVoltageCheck {
    fn check(&self, state: &SystemState) -> CheckResult {
        // If BATT_ARM_VOLT is 0, skip the battery voltage check
        if state.battery_arm_volt == 0.0 {
            crate::log_debug!("voltage check disabled (BATT_ARM_VOLT=0)");
            return Ok(());
        }

        // Check if battery voltage is below the arming threshold
        if state.battery.voltage < state.battery_arm_volt {
            crate::log_warn!(
                "voltage {}V below minimum {}V",
                state.battery.voltage,
                state.battery_arm_volt
            );
            Err(ArmingError::CheckFailed {
                reason: "voltage below BATT_ARM_VOLT",
                category: CheckCategory::Battery,
            })
        } else {
            Ok(())
        }
    }

    fn name(&self) -> &'static str {
        "Battery Voltage"
    }

    fn category(&self) -> CheckCategory {
        CheckCategory::Battery
    }
}

/// System state check
///
/// Verifies that the system is in a valid state for arming.
/// Currently checks that the vehicle is not already armed.
pub struct SystemStateCheck;

impl PreArmCheck for SystemStateCheck {
    fn check(&self, state: &SystemState) -> CheckResult {
        if state.is_armed() {
            Err(ArmingError::AlreadyArmed)
        } else {
            Ok(())
        }
    }

    fn name(&self) -> &'static str {
        "System State"
    }

    fn category(&self) -> CheckCategory {
        CheckCategory::System
    }
}

// Static instances of built-in checks for registration
static BATTERY_CHECK: BatteryVoltageCheck = BatteryVoltageCheck;
static SYSTEM_CHECK: SystemStateCheck = SystemStateCheck;

/// Create a default arming checker with built-in checks registered
///
/// # Arguments
///
/// * `enabled_categories` - Bitmask of enabled check categories (ARMING_CHECK parameter)
///
/// # Returns
///
/// An ArmingChecker with all built-in checks registered.
pub fn create_default_checker(enabled_categories: u16) -> ArmingChecker {
    let mut checker = ArmingChecker::new(enabled_categories);

    // Register built-in checks
    // Ignore registration errors since we control the count
    let _ = checker.register(&BATTERY_CHECK);
    let _ = checker.register(&SYSTEM_CHECK);

    checker
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::state::ArmedState;

    #[test]
    fn test_battery_check_passes() {
        let mut state = SystemState::new();
        state.battery.voltage = 12.0;
        state.battery_arm_volt = 10.5; // Default threshold

        let check = BatteryVoltageCheck;
        assert!(check.check(&state).is_ok());
    }

    #[test]
    fn test_battery_check_fails_low_voltage() {
        let mut state = SystemState::new();
        state.battery.voltage = 4.0; // Below BATT_ARM_VOLT threshold
        state.battery_arm_volt = 5.0;

        let check = BatteryVoltageCheck;
        let result = check.check(&state);
        assert!(result.is_err());

        if let Err(ArmingError::CheckFailed { reason, category }) = result {
            assert_eq!(category, CheckCategory::Battery);
            assert!(reason.contains("voltage") || reason.contains("BATT_ARM_VOLT"));
        } else {
            panic!("Expected CheckFailed error");
        }
    }

    #[test]
    fn test_battery_check_disabled_when_zero() {
        let mut state = SystemState::new();
        state.battery.voltage = 2.0; // Very low voltage
        state.battery_arm_volt = 0.0; // Disabled

        let check = BatteryVoltageCheck;
        // Should pass even with very low voltage because check is disabled
        assert!(check.check(&state).is_ok());
    }

    #[test]
    fn test_battery_check_uses_custom_threshold() {
        let mut state = SystemState::new();
        state.battery_arm_volt = 8.0; // Custom low threshold

        // Voltage below custom threshold - should fail
        state.battery.voltage = 7.5;
        let check = BatteryVoltageCheck;
        assert!(check.check(&state).is_err());

        // Voltage above custom threshold - should pass
        state.battery.voltage = 8.5;
        assert!(check.check(&state).is_ok());
    }

    #[test]
    fn test_system_state_check_passes() {
        let state = SystemState::new(); // Default is disarmed

        let check = SystemStateCheck;
        assert!(check.check(&state).is_ok());
    }

    #[test]
    fn test_system_state_check_fails_already_armed() {
        let mut state = SystemState::new();
        state.armed = ArmedState::Armed;

        let check = SystemStateCheck;
        let result = check.check(&state);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), ArmingError::AlreadyArmed);
    }

    #[test]
    fn test_arming_checker_runs_enabled_checks() {
        let mut state = SystemState::new();
        state.battery.voltage = 12.0;

        // Enable Battery + System checks
        let bitmask = 0x1080; // Battery (0x80) + System (0x1000)
        let checker = create_default_checker(bitmask);

        assert!(checker.run_checks(&state).is_ok());
    }

    #[test]
    fn test_arming_checker_skips_disabled_checks() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0; // Would fail battery check

        // Enable only System check (skip Battery)
        let bitmask = 0x1000; // System only
        let checker = create_default_checker(bitmask);

        // Should pass because battery check is disabled
        assert!(checker.run_checks(&state).is_ok());
    }

    #[test]
    fn test_arming_checker_fails_on_first_failure() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0; // Battery check will fail

        // Enable Battery + System checks
        let bitmask = 0x1080;
        let checker = create_default_checker(bitmask);

        let result = checker.run_checks(&state);
        assert!(result.is_err());

        if let Err(ArmingError::CheckFailed { category, .. }) = result {
            assert_eq!(category, CheckCategory::Battery);
        } else {
            panic!("Expected CheckFailed error");
        }
    }

    #[test]
    fn test_arming_checker_all_disabled() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0; // Would fail if checks were enabled

        // Disable all checks (ARMING_CHECK=0)
        let checker = create_default_checker(0);

        // Should pass with warning
        assert!(checker.run_checks(&state).is_ok());
    }

    #[test]
    fn test_enabled_categories_str() {
        let checker = create_default_checker(0x0000);
        assert_eq!(checker.enabled_categories_str(), "None");

        let checker = create_default_checker(0xFFFF);
        assert_eq!(checker.enabled_categories_str(), "All");

        let checker = create_default_checker(0x1080); // Battery + System
        let s = checker.enabled_categories_str();
        assert!(s.contains("Battery"));
        assert!(s.contains("System"));
    }
}
