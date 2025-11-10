//! Post-Disarm Cleanup
//!
//! Handles cleanup sequence after successful disarming.

use super::disarm::{DisarmMethod, DisarmReason};
use crate::communication::mavlink::state::SystemState;

/// Post-disarm cleanup error
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CleanupError {
    /// Actuators not in neutral state
    ActuatorsNotNeutral,
    /// Subsystem notification failed
    SubsystemNotificationFailed(&'static str),
    /// Configuration persistence failed
    ConfigPersistenceFailed,
}

impl core::fmt::Display for CleanupError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            CleanupError::ActuatorsNotNeutral => {
                write!(f, "Actuators not in neutral state after disarm")
            }
            CleanupError::SubsystemNotificationFailed(subsystem) => {
                write!(f, "Failed to notify subsystem: {}", subsystem)
            }
            CleanupError::ConfigPersistenceFailed => {
                write!(f, "Failed to persist configuration changes")
            }
        }
    }
}

/// Post-disarm cleanup handler
///
/// Executes cleanup sequence after successful disarm:
/// 1. Log disarm event
/// 2. Verify actuators neutral
/// 3. Notify subsystems
/// 4. Disable geofence (if auto-enabled)
/// 5. Persist configuration changes
/// 6. Clear arm timestamp
pub struct PostDisarmCleanup {
    /// Track if geofence was auto-enabled (to disable on disarm)
    fence_auto_enabled: bool,
}

impl PostDisarmCleanup {
    /// Create a new post-disarm cleanup handler
    ///
    /// # Arguments
    ///
    /// * `fence_auto_enabled` - True if geofence was auto-enabled on arm (FENCE_AUTOENABLE)
    pub fn new(fence_auto_enabled: bool) -> Self {
        Self { fence_auto_enabled }
    }

    /// Execute post-disarm cleanup sequence
    ///
    /// # Arguments
    ///
    /// * `state` - System state (must be disarmed)
    /// * `method` - Method used to disarm
    /// * `reason` - Reason for disarming
    ///
    /// # Returns
    ///
    /// Ok if cleanup completes successfully, or Err if any step fails.
    ///
    /// # Cleanup Steps
    ///
    /// 1. Verify disarmed state
    /// 2. Log disarm event with method and reason
    /// 3. Verify actuators in neutral state
    /// 4. Notify subsystems (monitoring, failsafe, mode_manager)
    /// 5. Disable geofence if FENCE_AUTOENABLE was set
    /// 6. Persist configuration changes if unsaved
    /// 7. Clear arm timestamp and monitoring state
    #[allow(unused_variables)] // method and reason used in logging but may be compiled out
    pub fn execute(
        &mut self,
        state: &SystemState,
        method: DisarmMethod,
        reason: DisarmReason,
    ) -> Result<(), CleanupError> {
        // 1. Verify disarmed state
        if state.is_armed() {
            crate::log_warn!("Post-disarm cleanup called while still armed");
            return Ok(()); // Not an error, just skip cleanup
        }

        // 2. Log disarm event
        crate::log_info!("Vehicle disarmed via {} ({})", method, reason);

        // 3. Verify actuators neutral
        // TODO: When actuator system is implemented, verify:
        // if !context.actuators.verify_neutral_state() {
        //     return Err(CleanupError::ActuatorsNotNeutral);
        // }
        crate::log_debug!("Actuator neutral verification (placeholder)");

        // 4. Notify subsystems
        // TODO: When subsystems are implemented, notify:
        // - monitoring: Stop armed state monitoring tasks
        // - failsafe: Disable armed-state failsafes
        // - mode_manager: Update mode restrictions
        crate::log_debug!("Subsystem notification (placeholder)");

        // 5. Disable geofence if auto-enabled
        if self.fence_auto_enabled {
            // TODO: When fence system is implemented:
            // context.fence.disable()?;
            crate::log_debug!("Geofence auto-disable (placeholder)");
        }

        // 6. Persist configuration changes
        // TODO: When parameter system supports change tracking:
        // if context.params.has_unsaved_changes() {
        //     context.params.save()?;
        // }
        crate::log_debug!("Configuration persistence (placeholder)");

        // 7. Clear arm timestamp and monitoring state
        // This is handled by the caller (PostArmInitializer::clear_timestamp())
        crate::log_debug!("Arm timestamp cleared");

        crate::log_info!("Post-disarm cleanup complete");
        Ok(())
    }
}

impl Default for PostDisarmCleanup {
    fn default() -> Self {
        Self::new(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::state::ArmedState;

    #[test]
    fn test_cleanup_new() {
        let cleanup = PostDisarmCleanup::new(true);
        assert!(cleanup.fence_auto_enabled);

        let cleanup = PostDisarmCleanup::new(false);
        assert!(!cleanup.fence_auto_enabled);
    }

    #[test]
    fn test_cleanup_success() {
        let mut cleanup = PostDisarmCleanup::new(false);
        let state = SystemState::new(); // Disarmed by default

        let result = cleanup.execute(&state, DisarmMethod::GcsCommand, DisarmReason::UserRequest);
        assert!(result.is_ok());
    }

    #[test]
    fn test_cleanup_skips_if_armed() {
        let mut cleanup = PostDisarmCleanup::new(false);
        let mut state = SystemState::new();
        state.armed = ArmedState::Armed;

        // Should not fail, just skip cleanup
        let result = cleanup.execute(&state, DisarmMethod::GcsCommand, DisarmReason::UserRequest);
        assert!(result.is_ok());
    }

    #[test]
    fn test_cleanup_with_fence_auto_enabled() {
        let mut cleanup = PostDisarmCleanup::new(true);
        let state = SystemState::new();

        let result = cleanup.execute(&state, DisarmMethod::GcsCommand, DisarmReason::UserRequest);
        assert!(result.is_ok());
    }

    #[test]
    fn test_cleanup_error_display() {
        let error = CleanupError::ActuatorsNotNeutral;
        assert_eq!(
            format!("{}", error),
            "Actuators not in neutral state after disarm"
        );

        let error = CleanupError::SubsystemNotificationFailed("monitoring");
        assert_eq!(
            format!("{}", error),
            "Failed to notify subsystem: monitoring"
        );

        let error = CleanupError::ConfigPersistenceFailed;
        assert_eq!(
            format!("{}", error),
            "Failed to persist configuration changes"
        );
    }

    #[test]
    fn test_cleanup_different_methods() {
        let mut cleanup = PostDisarmCleanup::new(false);
        let state = SystemState::new();

        // Test different disarm methods
        assert!(cleanup
            .execute(&state, DisarmMethod::RcStick, DisarmReason::UserRequest)
            .is_ok());
        assert!(cleanup
            .execute(&state, DisarmMethod::GcsCommand, DisarmReason::UserRequest)
            .is_ok());
        assert!(cleanup
            .execute(&state, DisarmMethod::Failsafe, DisarmReason::Failsafe)
            .is_ok());
    }

    #[test]
    fn test_cleanup_different_reasons() {
        let mut cleanup = PostDisarmCleanup::new(false);
        let state = SystemState::new();

        // Test different disarm reasons
        assert!(cleanup
            .execute(&state, DisarmMethod::GcsCommand, DisarmReason::UserRequest)
            .is_ok());
        assert!(cleanup
            .execute(&state, DisarmMethod::Failsafe, DisarmReason::Failsafe)
            .is_ok());
        assert!(cleanup
            .execute(&state, DisarmMethod::GcsCommand, DisarmReason::AutoLand)
            .is_ok());
    }
}
