//! Pre-Arm Checks
//!
//! Defines the PreArmCheck trait and built-in checks for system validation
//! before arming.

use super::error::{ArmingError, CheckCategory};
use crate::communication::mavlink::state::SystemState;

#[cfg(feature = "defmt")]
use defmt::{debug, info, warn};

// Stub macros when defmt is not available
#[cfg(not(feature = "defmt"))]
macro_rules! warn {
    ($($arg:tt)*) => {{}};
}

#[cfg(not(feature = "defmt"))]
macro_rules! info {
    ($($arg:tt)*) => {{}};
}

#[cfg(not(feature = "defmt"))]
macro_rules! debug {
    ($($arg:tt)*) => {{}};
}

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
    #[allow(unused_variables)] // executed_count used in defmt macro (may be stubbed out)
    pub fn run_checks(&self, state: &SystemState) -> CheckResult {
        // If ARMING_CHECK is 0, all checks are disabled (bench testing only)
        if self.enabled_categories == 0 {
            warn!("ARMING_CHECK=0: All pre-arm checks disabled (bench testing only)");
            return Ok(());
        }

        info!(
            "Running pre-arm checks (enabled: 0x{:04X})",
            self.enabled_categories
        );

        let mut executed_count = 0;
        for check in &self.checks {
            // Skip checks that are not enabled
            if !check.category().is_enabled(self.enabled_categories) {
                debug!("Pre-arm check skipped (disabled): {}", check.name());
                continue;
            }

            // Execute check
            info!("Pre-arm check: {}", check.name());
            check.check(state)?;
            executed_count += 1;
        }

        info!("All pre-arm checks passed ({} executed)", executed_count);
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
/// Currently uses the hardcoded critical threshold (10.0V).
///
/// In the future, this will check against BATT_ARM_VOLT parameter.
pub struct BatteryVoltageCheck;

impl PreArmCheck for BatteryVoltageCheck {
    fn check(&self, state: &SystemState) -> CheckResult {
        if state.battery.is_critical() {
            Err(ArmingError::CheckFailed {
                reason: "Battery voltage below minimum (< 10.0V)",
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

        let check = BatteryVoltageCheck;
        assert!(check.check(&state).is_ok());
    }

    #[test]
    fn test_battery_check_fails_low_voltage() {
        let mut state = SystemState::new();
        state.battery.voltage = 9.0; // Below critical (10.0V)

        let check = BatteryVoltageCheck;
        let result = check.check(&state);
        assert!(result.is_err());

        if let Err(ArmingError::CheckFailed { reason, category }) = result {
            assert_eq!(category, CheckCategory::Battery);
            assert!(reason.contains("voltage"));
        } else {
            panic!("Expected CheckFailed error");
        }
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
