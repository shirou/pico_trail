//! Pre-Disarm Validation
//!
//! Provides safety checks before allowing vehicle disarm.

use super::error::DisarmError;
use crate::communication::mavlink::state::SystemState;

/// Method used to disarm the vehicle
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DisarmMethod {
    /// Disarmed via RC stick command
    RcStick,
    /// Disarmed via GCS MAVLink command
    GcsCommand,
    /// Disarmed by failsafe action
    Failsafe,
}

impl core::fmt::Display for DisarmMethod {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DisarmMethod::RcStick => write!(f, "RC stick"),
            DisarmMethod::GcsCommand => write!(f, "GCS command"),
            DisarmMethod::Failsafe => write!(f, "failsafe"),
        }
    }
}

/// Reason for disarming
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DisarmReason {
    /// User requested disarm
    UserRequest,
    /// Failsafe triggered disarm
    Failsafe,
    /// Auto-land completed
    AutoLand,
}

impl core::fmt::Display for DisarmReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DisarmReason::UserRequest => write!(f, "user request"),
            DisarmReason::Failsafe => write!(f, "failsafe"),
            DisarmReason::AutoLand => write!(f, "auto-land"),
        }
    }
}

/// Disarm validator with method-specific rules
///
/// Validates safety conditions before allowing disarm, with different
/// strictness levels based on disarm method:
/// - **RC stick**: Strictest (throttle low, velocity safe)
/// - **GCS command**: Lenient (throttle low only)
/// - **Failsafe**: No checks (emergency disarm)
pub struct DisarmValidator {
    /// Throttle threshold for safe disarm (percentage, 0.0-100.0)
    throttle_threshold: f32,
    /// Maximum velocity for safe disarm (m/s)
    max_velocity: f32,
}

impl DisarmValidator {
    /// Create a new disarm validator with default thresholds
    ///
    /// # Default Thresholds
    ///
    /// - Throttle: 10% (safe for disarm when below this)
    /// - Velocity: 0.5 m/s (vehicle nearly stopped)
    pub fn new() -> Self {
        Self {
            throttle_threshold: 10.0,
            max_velocity: 0.5,
        }
    }

    /// Create a new disarm validator with custom thresholds
    ///
    /// # Arguments
    ///
    /// * `throttle_threshold` - Maximum throttle percentage for safe disarm (0.0-100.0)
    /// * `max_velocity` - Maximum velocity for safe disarm (m/s)
    pub fn with_thresholds(throttle_threshold: f32, max_velocity: f32) -> Self {
        Self {
            throttle_threshold,
            max_velocity,
        }
    }

    /// Validate disarm request
    ///
    /// # Arguments
    ///
    /// * `state` - Current system state
    /// * `method` - Method used to initiate disarm
    /// * `forced` - If true, bypass all validation checks
    ///
    /// # Returns
    ///
    /// Ok if disarm is safe, or Err with specific reason if validation fails.
    ///
    /// # Validation Rules
    ///
    /// - **Forced disarm**: Always allowed (no checks)
    /// - **Failsafe disarm**: Always allowed (emergency situation)
    /// - **RC stick disarm**: Requires throttle low AND velocity safe
    /// - **GCS command disarm**: Requires throttle low only (lenient for remote ops)
    pub fn validate(
        &self,
        state: &SystemState,
        method: DisarmMethod,
        forced: bool,
    ) -> Result<(), DisarmError> {
        // Check armed state
        if !state.is_armed() {
            return Err(DisarmError::NotArmed);
        }

        // Forced disarm bypasses all checks
        if forced {
            crate::log_warn!("FORCED DISARM: Bypassing all pre-disarm validation");
            return Ok(());
        }

        // Failsafe disarm bypasses all checks (emergency situation)
        #[allow(clippy::collapsible_if)]
        if method == DisarmMethod::Failsafe {
            return Ok(());
        }

        // Validate throttle (all methods except failsafe)
        // TODO: When actuator system is implemented, read actual throttle
        // For now, use placeholder validation
        let current_throttle = 0.0; // Placeholder: assume throttle at 0%

        if current_throttle > self.throttle_threshold {
            return Err(DisarmError::ThrottleNotLow {
                current: current_throttle,
            });
        }

        // Validate velocity (RC stick only - strictest check)
        if method == DisarmMethod::RcStick {
            // TODO: When AHRS/EKF system is implemented, read actual ground speed
            // For now, use placeholder validation
            let current_velocity = 0.0; // Placeholder: assume vehicle stopped

            if current_velocity > self.max_velocity {
                return Err(DisarmError::VelocityTooHigh {
                    current: current_velocity,
                    max: self.max_velocity,
                });
            }
        }

        Ok(())
    }
}

impl Default for DisarmValidator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::state::ArmedState;

    #[test]
    fn test_disarm_validator_new() {
        let validator = DisarmValidator::new();
        assert_eq!(validator.throttle_threshold, 10.0);
        assert_eq!(validator.max_velocity, 0.5);
    }

    #[test]
    fn test_disarm_validator_with_thresholds() {
        let validator = DisarmValidator::with_thresholds(15.0, 1.0);
        assert_eq!(validator.throttle_threshold, 15.0);
        assert_eq!(validator.max_velocity, 1.0);
    }

    #[test]
    fn test_validate_not_armed() {
        let validator = DisarmValidator::new();
        let state = SystemState::new(); // Default is disarmed

        let result = validator.validate(&state, DisarmMethod::GcsCommand, false);
        assert_eq!(result.unwrap_err(), DisarmError::NotArmed);
    }

    #[test]
    fn test_validate_forced_disarm() {
        let validator = DisarmValidator::new();
        let mut state = SystemState::new();
        state.armed = ArmedState::Armed;

        // Forced disarm should bypass all checks
        let result = validator.validate(&state, DisarmMethod::GcsCommand, true);
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_failsafe_disarm() {
        let validator = DisarmValidator::new();
        let mut state = SystemState::new();
        state.armed = ArmedState::Armed;

        // Failsafe disarm should bypass all checks
        let result = validator.validate(&state, DisarmMethod::Failsafe, false);
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_gcs_command_disarm() {
        let validator = DisarmValidator::new();
        let mut state = SystemState::new();
        state.armed = ArmedState::Armed;

        // GCS command disarm with low throttle should succeed
        let result = validator.validate(&state, DisarmMethod::GcsCommand, false);
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_rc_stick_disarm() {
        let validator = DisarmValidator::new();
        let mut state = SystemState::new();
        state.armed = ArmedState::Armed;

        // RC stick disarm with low throttle and velocity should succeed
        let result = validator.validate(&state, DisarmMethod::RcStick, false);
        assert!(result.is_ok());
    }

    #[test]
    fn test_disarm_method_display() {
        assert_eq!(format!("{}", DisarmMethod::RcStick), "RC stick");
        assert_eq!(format!("{}", DisarmMethod::GcsCommand), "GCS command");
        assert_eq!(format!("{}", DisarmMethod::Failsafe), "failsafe");
    }

    #[test]
    fn test_disarm_reason_display() {
        assert_eq!(format!("{}", DisarmReason::UserRequest), "user request");
        assert_eq!(format!("{}", DisarmReason::Failsafe), "failsafe");
        assert_eq!(format!("{}", DisarmReason::AutoLand), "auto-land");
    }
}
