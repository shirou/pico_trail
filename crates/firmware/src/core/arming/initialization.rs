//! Post-Arm Initialization
//!
//! Handles initialization sequence after successful arming.

use super::error::ArmingError;
use crate::communication::mavlink::state::SystemState;

/// Method used to arm the vehicle
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ArmMethod {
    /// Armed via RC stick command
    RcStick,
    /// Armed via GCS MAVLink command
    GcsCommand,
}

impl core::fmt::Display for ArmMethod {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ArmMethod::RcStick => write!(f, "RC stick"),
            ArmMethod::GcsCommand => write!(f, "GCS command"),
        }
    }
}

/// Post-arm initializer
///
/// Executes initialization sequence after successful pre-arm checks
/// and armed state transition.
pub struct PostArmInitializer {
    /// Timestamp when armed (None if disarmed)
    arm_timestamp_ms: Option<u64>,
    /// Enabled check categories bitmask
    enabled_checks: u16,
}

impl PostArmInitializer {
    /// Create a new post-arm initializer
    ///
    /// # Arguments
    ///
    /// * `enabled_checks` - Bitmask of enabled check categories (ARMING_CHECK parameter)
    pub fn new(enabled_checks: u16) -> Self {
        Self {
            arm_timestamp_ms: None,
            enabled_checks,
        }
    }

    /// Execute post-arm initialization sequence
    ///
    /// # Arguments
    ///
    /// * `state` - System state (must be armed)
    /// * `method` - Method used to arm the vehicle
    ///
    /// # Returns
    ///
    /// Ok if initialization completes successfully, or Err if any step fails.
    ///
    /// # Initialization Steps
    ///
    /// 1. Record arm timestamp
    /// 2. Log arm event with method
    /// 3. Initialize actuators (placeholder - will call context.actuators.enter_armed_state())
    /// 4. Notify subsystems (placeholder - will notify monitoring, failsafe, mode_manager)
    /// 5. Enable geofence if configured (placeholder - will check FENCE_AUTOENABLE)
    /// 6. Warn if checks disabled
    pub fn execute(&mut self, state: &SystemState, _method: ArmMethod) -> Result<(), ArmingError> {
        // Verify armed state
        if !state.is_armed() {
            return Err(ArmingError::InitializationFailed {
                subsystem: "state verification",
            });
        }

        // 1. Record arm timestamp
        self.arm_timestamp_ms = Some(state.uptime_us / 1000);

        // 2. Log arm event
        crate::log_info!("Vehicle armed via {}", _method);

        // 3. Initialize actuators
        // TODO: When actuator subsystem is implemented, call:
        // context.actuators.enter_armed_state()?;
        crate::log_debug!("Actuators initialization (placeholder)");

        // 4. Notify subsystems
        // TODO: When subsystems are implemented, notify:
        // - monitoring: Start armed state monitoring
        // - failsafe: Enable armed-state failsafes
        // - mode_manager: Update mode restrictions
        crate::log_debug!("Subsystem notification (placeholder)");

        // 5. Enable geofence if configured
        // TODO: When fence subsystem is implemented, check FENCE_AUTOENABLE:
        // if context.params.get("FENCE_AUTOENABLE") == 1 {
        //     context.fence.enable()?;
        // }
        crate::log_debug!("Geofence auto-enable (placeholder)");

        // 6. Warn if checks disabled
        match self.enabled_checks {
            0 => crate::log_warn!(
                "ARMED WITH ALL PRE-ARM CHECKS DISABLED (ARMING_CHECK=0) - BENCH TESTING ONLY"
            ),
            0xFFFF => {}
            _ => crate::log_warn!(
                "Armed with some pre-arm checks disabled (ARMING_CHECK=0x{:04X})",
                self.enabled_checks
            ),
        }

        crate::log_info!("Post-arm initialization complete");
        Ok(())
    }

    /// Get the arm timestamp (milliseconds since boot)
    ///
    /// Returns None if vehicle is not armed or has never been armed.
    pub fn arm_timestamp_ms(&self) -> Option<u64> {
        self.arm_timestamp_ms
    }

    /// Get the arm duration (milliseconds)
    ///
    /// Returns None if vehicle is not armed, or Some(duration) if armed.
    pub fn arm_duration_ms(&self, current_uptime_us: u64) -> Option<u64> {
        self.arm_timestamp_ms
            .map(|ts| (current_uptime_us / 1000).saturating_sub(ts))
    }

    /// Clear arm timestamp (called on disarm)
    pub fn clear_timestamp(&mut self) {
        self.arm_timestamp_ms = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::state::{ArmedState, SystemState};

    #[test]
    fn test_post_arm_init_success() {
        let mut state = SystemState::new();
        state.armed = ArmedState::Armed;
        state.uptime_us = 5_000_000; // 5 seconds

        let mut initializer = PostArmInitializer::new(0xFFFF);
        let result = initializer.execute(&state, ArmMethod::GcsCommand);

        assert!(result.is_ok());
        assert_eq!(initializer.arm_timestamp_ms(), Some(5000));
    }

    #[test]
    fn test_post_arm_init_fails_if_not_armed() {
        let state = SystemState::new(); // Disarmed by default

        let mut initializer = PostArmInitializer::new(0xFFFF);
        let result = initializer.execute(&state, ArmMethod::GcsCommand);

        assert!(result.is_err());
        if let Err(ArmingError::InitializationFailed { subsystem }) = result {
            assert_eq!(subsystem, "state verification");
        } else {
            panic!("Expected InitializationFailed error");
        }
    }

    #[test]
    fn test_arm_duration() {
        let mut state = SystemState::new();
        state.armed = ArmedState::Armed;
        state.uptime_us = 5_000_000; // Armed at 5s

        let mut initializer = PostArmInitializer::new(0xFFFF);
        let _ = initializer.execute(&state, ArmMethod::GcsCommand);

        // Check duration after 3 more seconds
        let duration = initializer.arm_duration_ms(8_000_000); // Now at 8s
        assert_eq!(duration, Some(3000)); // 3 seconds armed
    }

    #[test]
    fn test_clear_timestamp() {
        let mut state = SystemState::new();
        state.armed = ArmedState::Armed;
        state.uptime_us = 5_000_000;

        let mut initializer = PostArmInitializer::new(0xFFFF);
        let _ = initializer.execute(&state, ArmMethod::GcsCommand);

        assert!(initializer.arm_timestamp_ms().is_some());

        initializer.clear_timestamp();
        assert!(initializer.arm_timestamp_ms().is_none());
    }

    #[test]
    fn test_arm_method_display() {
        assert_eq!(format!("{}", ArmMethod::RcStick), "RC stick");
        assert_eq!(format!("{}", ArmMethod::GcsCommand), "GCS command");
    }
}
