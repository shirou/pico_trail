//! GCS Communication Lost Failsafe Checker
//!
//! Two-stage timeout detection for GCS heartbeat loss:
//! - **Stage 1 (detection)**: Heartbeat age exceeds `FS_GCS_TIMEOUT` (default 5.0s)
//! - **Stage 2 (persistence)**: Condition persists for `FS_TIMEOUT` (default 1.5s)
//!
//! Follows the `BatteryFailsafeChecker` pattern: pure logic with no async or
//! embassy dependencies, fully testable on host.

use crate::autopilot::state::FlightMode;
use crate::parameters::failsafe::{FailsafeAction, FailsafeParams};

/// Action returned by GCS failsafe check
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GcsFailsafeAction {
    /// No action needed
    None,
    /// Set vehicle mode (Hold or RTL)
    SetMode(FlightMode),
    /// Failsafe activated but no mode change needed (already in Hold/RTL/SmartRTL)
    Activated,
    /// Failsafe cleared (heartbeat resumed)
    Cleared,
}

impl GcsFailsafeAction {
    /// Returns a STATUSTEXT message string for this action, or `None` if no notification needed.
    ///
    /// Used by both firmware and SITL callers to avoid duplicating notification logic.
    pub fn status_text(&self) -> Option<&'static str> {
        match self {
            GcsFailsafeAction::None => Option::None,
            GcsFailsafeAction::SetMode(FlightMode::Hold) => Some("Failsafe: GCS Lost - Hold"),
            GcsFailsafeAction::SetMode(FlightMode::Rtl) => Some("Failsafe: GCS Lost - RTL"),
            GcsFailsafeAction::SetMode(_) => Some("Failsafe: GCS Lost"),
            GcsFailsafeAction::Activated => Some("Failsafe: GCS Lost - already in safe mode"),
            GcsFailsafeAction::Cleared => Some("GCS Failsafe Cleared"),
        }
    }

    /// Whether this action is a warning (activation) vs info (cleared).
    ///
    /// Returns `true` for SetMode/Activated (warnings), `false` for Cleared (info).
    /// Returns `None` for None (no notification).
    pub fn is_warning(&self) -> Option<bool> {
        match self {
            GcsFailsafeAction::None => Option::None,
            GcsFailsafeAction::SetMode(_) | GcsFailsafeAction::Activated => Some(true),
            GcsFailsafeAction::Cleared => Some(false),
        }
    }
}

/// Configuration for GCS failsafe (pre-converted to microseconds)
#[derive(Debug, Clone)]
pub struct GcsFailsafeConfig {
    /// Whether GCS failsafe is enabled
    pub enabled: bool,
    /// GCS heartbeat detection timeout in microseconds (FS_GCS_TIMEOUT)
    pub gcs_timeout_us: u64,
    /// Persistence timeout in microseconds (FS_TIMEOUT)
    pub persistence_us: u64,
    /// Failsafe action to take
    pub action: FailsafeAction,
}

impl GcsFailsafeConfig {
    /// Build config from failsafe parameters.
    ///
    /// Maps `FailsafeAction::Disarm` to `Hold` because disarming on GCS loss
    /// while in motion is dangerous. Disarm is retained in the enum for battery
    /// critical failsafe only.
    pub fn from_params(params: &FailsafeParams) -> Self {
        let raw_action = match params.action {
            0 => FailsafeAction::None,
            1 => FailsafeAction::RTL,
            2 => FailsafeAction::Hold,
            3 => FailsafeAction::Hold, // Disarm → Hold (safe fallback for GCS)
            _ => FailsafeAction::Hold, // Invalid → Hold (safe default)
        };

        // Guard against negative or NaN values: clamp to 0.0 before conversion
        let gcs_timeout_secs = if params.gcs_timeout.is_finite() && params.gcs_timeout > 0.0 {
            params.gcs_timeout
        } else {
            5.0 // safe default
        };
        let persistence_secs = if params.timeout.is_finite() && params.timeout > 0.0 {
            params.timeout
        } else {
            1.5 // safe default
        };

        Self {
            enabled: params.gcs_enable,
            gcs_timeout_us: (gcs_timeout_secs * 1_000_000.0) as u64,
            persistence_us: (persistence_secs * 1_000_000.0) as u64,
            action: raw_action,
        }
    }
}

/// Stateful GCS failsafe checker with two-stage timeout detection
pub struct GcsFailsafeChecker {
    config: GcsFailsafeConfig,
    /// Whether the GCS loss condition has been detected (stage 1)
    condition_detected: bool,
    /// Timestamp when condition was first detected
    condition_start_us: u64,
    /// Whether failsafe action has been executed (sticky until cleared)
    failsafe_active: bool,
}

impl GcsFailsafeChecker {
    /// Create a new checker from configuration
    pub fn new(config: GcsFailsafeConfig) -> Self {
        Self {
            config,
            condition_detected: false,
            condition_start_us: 0,
            failsafe_active: false,
        }
    }

    /// Check GCS heartbeat status and return action if failsafe triggered.
    ///
    /// Two-stage detection:
    /// 1. Heartbeat age >= `gcs_timeout_us`: condition detected
    /// 2. Condition persists for `persistence_us`: failsafe fires
    ///
    /// Guards (return None immediately):
    /// - Not armed
    /// - GCS failsafe disabled
    /// - No heartbeat ever received (`heartbeat_count == 0`)
    ///
    /// Mode exemption: If vehicle is already in Hold/RTL/SmartRTL, returns
    /// `Activated` instead of `SetMode` so the caller can send STATUSTEXT
    /// without changing mode.
    pub fn check(
        &mut self,
        last_heartbeat_us: u64,
        heartbeat_count: u32,
        current_time_us: u64,
        is_armed: bool,
        current_mode: FlightMode,
    ) -> GcsFailsafeAction {
        // Guard: not armed - clear condition tracking
        if !is_armed {
            self.condition_detected = false;
            return GcsFailsafeAction::None;
        }

        // Guard: disabled
        if !self.config.enabled {
            return GcsFailsafeAction::None;
        }

        // Guard: never-seen (no heartbeat ever received)
        if heartbeat_count == 0 {
            return GcsFailsafeAction::None;
        }

        let heartbeat_age = current_time_us.saturating_sub(last_heartbeat_us);

        // Heartbeat is within threshold - check for recovery
        if heartbeat_age < self.config.gcs_timeout_us {
            self.condition_detected = false;
            if self.failsafe_active {
                self.failsafe_active = false;
                return GcsFailsafeAction::Cleared;
            }
            return GcsFailsafeAction::None;
        }

        // Stage 1: condition detected
        if !self.condition_detected {
            self.condition_detected = true;
            self.condition_start_us = current_time_us;
        }

        // Stage 2: check persistence
        let condition_duration = current_time_us.saturating_sub(self.condition_start_us);
        if condition_duration >= self.config.persistence_us && !self.failsafe_active {
            // FS_ACTION=None means failsafe is disabled at the action level;
            // do not set failsafe_active so recovery won't emit a spurious Cleared.
            if self.config.action == FailsafeAction::None {
                return GcsFailsafeAction::None;
            }

            self.failsafe_active = true;

            // Mode exemption: already in a safe mode
            if matches!(
                current_mode,
                FlightMode::Hold | FlightMode::Rtl | FlightMode::SmartRtl
            ) {
                return GcsFailsafeAction::Activated;
            }

            // Map action to flight mode
            return match self.config.action {
                FailsafeAction::None => unreachable!(), // handled above
                FailsafeAction::RTL => GcsFailsafeAction::SetMode(FlightMode::Rtl),
                FailsafeAction::Hold => GcsFailsafeAction::SetMode(FlightMode::Hold),
                FailsafeAction::Disarm => GcsFailsafeAction::SetMode(FlightMode::Hold),
            };
        }

        GcsFailsafeAction::None
    }

    /// Reset all failsafe state (used on disarm/rearm cycle)
    pub fn reset(&mut self) {
        self.condition_detected = false;
        self.condition_start_us = 0;
        self.failsafe_active = false;
    }

    /// Reset state and reload configuration
    ///
    /// Called on arm transition to clear sticky flags and pick up any
    /// parameter changes made while disarmed.
    pub fn reset_with_config(&mut self, config: GcsFailsafeConfig) {
        self.config = config;
        self.reset();
    }

    /// Whether the failsafe is currently active
    pub fn is_active(&self) -> bool {
        self.failsafe_active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GCS_TIMEOUT_US: u64 = 5_000_000; // 5s
    const PERSISTENCE_US: u64 = 1_500_000; // 1.5s
    const TOTAL_TIMEOUT_US: u64 = GCS_TIMEOUT_US + PERSISTENCE_US; // 6.5s

    fn default_config() -> GcsFailsafeConfig {
        GcsFailsafeConfig {
            enabled: true,
            gcs_timeout_us: GCS_TIMEOUT_US,
            persistence_us: PERSISTENCE_US,
            action: FailsafeAction::Hold,
        }
    }

    fn rtl_config() -> GcsFailsafeConfig {
        GcsFailsafeConfig {
            action: FailsafeAction::RTL,
            ..default_config()
        }
    }

    /// Advance a checker through the two-stage detection until failsafe fires.
    /// Returns the action from the final (triggering) check.
    fn trigger_failsafe(
        checker: &mut GcsFailsafeChecker,
        last_hb: u64,
        hb_count: u32,
        mode: FlightMode,
    ) -> GcsFailsafeAction {
        // Stage 1: detect condition at gcs_timeout
        let detect_time = last_hb + GCS_TIMEOUT_US;
        checker.check(last_hb, hb_count, detect_time, true, mode);
        // Stage 2: persistence elapsed
        let trigger_time = detect_time + PERSISTENCE_US;
        checker.check(last_hb, hb_count, trigger_time, true, mode)
    }

    #[test]
    fn test_normal_heartbeats_no_trigger() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        // Heartbeat at t=0, check at t=1s - well within timeout
        let action = checker.check(0, 10, 1_000_000, true, FlightMode::Manual);
        assert_eq!(action, GcsFailsafeAction::None);
    }

    #[test]
    fn test_two_stage_timeout_triggers() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        let last_hb = 0;
        let hb_count = 10;

        // t = 4s: within GCS timeout, no trigger
        assert_eq!(
            checker.check(last_hb, hb_count, 4_000_000, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );

        // t = 5s: GCS timeout reached, stage 1 detected, but persistence not met
        assert_eq!(
            checker.check(last_hb, hb_count, 5_000_000, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );

        // t = 6s: 1s into persistence (need 1.5s), still waiting
        assert_eq!(
            checker.check(last_hb, hb_count, 6_000_000, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );

        // t = 6.5s: persistence met, failsafe fires
        assert_eq!(
            checker.check(
                last_hb,
                hb_count,
                TOTAL_TIMEOUT_US,
                true,
                FlightMode::Manual
            ),
            GcsFailsafeAction::SetMode(FlightMode::Hold),
        );
    }

    #[test]
    fn test_condition_clears_before_persistence() {
        let mut checker = GcsFailsafeChecker::new(default_config());

        // t = 5s: condition detected
        assert_eq!(
            checker.check(0, 10, 5_000_000, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );

        // t = 5.5s: heartbeat resumes (age < gcs_timeout)
        assert_eq!(
            checker.check(5_500_000, 11, 5_500_000, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );

        // t = 11s from new heartbeat: would be timeout from original, but not from new
        assert_eq!(
            checker.check(5_500_000, 11, 10_000_000, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );
    }

    #[test]
    fn test_armed_guard_disarmed_never_triggers() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        // Way past timeout, but not armed
        assert_eq!(
            checker.check(0, 10, 100_000_000, false, FlightMode::Manual),
            GcsFailsafeAction::None,
        );
    }

    #[test]
    fn test_never_seen_guard() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        // heartbeat_count == 0: GCS never connected
        assert_eq!(
            checker.check(0, 0, 100_000_000, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );
    }

    #[test]
    fn test_disabled_guard() {
        let config = GcsFailsafeConfig {
            enabled: false,
            ..default_config()
        };
        let mut checker = GcsFailsafeChecker::new(config);
        assert_eq!(
            checker.check(0, 10, 100_000_000, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );
    }

    #[test]
    fn test_mode_exemption_hold() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::Hold);
        assert_eq!(action, GcsFailsafeAction::Activated);
    }

    #[test]
    fn test_mode_exemption_rtl() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::Rtl);
        assert_eq!(action, GcsFailsafeAction::Activated);
    }

    #[test]
    fn test_mode_exemption_smart_rtl() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::SmartRtl);
        assert_eq!(action, GcsFailsafeAction::Activated);
    }

    #[test]
    fn test_mode_exemption_then_cleared() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        // Trigger in Hold mode (Activated)
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::Hold);
        assert_eq!(action, GcsFailsafeAction::Activated);
        // Heartbeat resumes
        let resume_time = TOTAL_TIMEOUT_US + 1_000_000;
        assert_eq!(
            checker.check(resume_time, 11, resume_time, true, FlightMode::Hold),
            GcsFailsafeAction::Cleared,
        );
    }

    #[test]
    fn test_sticky_trigger_fires_once() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        // First trigger via two-stage
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::Manual);
        assert_eq!(action, GcsFailsafeAction::SetMode(FlightMode::Hold));
        // Second check: sticky, no repeat
        assert_eq!(
            checker.check(0, 10, TOTAL_TIMEOUT_US + 1_000_000, true, FlightMode::Hold),
            GcsFailsafeAction::None,
        );
    }

    #[test]
    fn test_recovery_returns_cleared() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        // Trigger
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::Manual);
        assert_eq!(action, GcsFailsafeAction::SetMode(FlightMode::Hold));
        // Heartbeat resumes
        let resume_time = TOTAL_TIMEOUT_US + 500_000;
        assert_eq!(
            checker.check(resume_time, 11, resume_time, true, FlightMode::Hold),
            GcsFailsafeAction::Cleared,
        );
        // After clear, no more events
        assert_eq!(
            checker.check(
                resume_time,
                11,
                resume_time + 100_000,
                true,
                FlightMode::Hold
            ),
            GcsFailsafeAction::None,
        );
    }

    #[test]
    fn test_retrigger_after_recovery() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        // First trigger
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::Manual);
        assert_eq!(action, GcsFailsafeAction::SetMode(FlightMode::Hold));
        // Recovery
        let resume = TOTAL_TIMEOUT_US + 500_000;
        assert_eq!(
            checker.check(resume, 11, resume, true, FlightMode::Hold),
            GcsFailsafeAction::Cleared,
        );
        // New loss: trigger again via two-stage
        let action2 = trigger_failsafe(&mut checker, resume, 11, FlightMode::Manual);
        assert_eq!(action2, GcsFailsafeAction::SetMode(FlightMode::Hold));
    }

    #[test]
    fn test_arm_transition_reset() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        // Trigger
        trigger_failsafe(&mut checker, 0, 10, FlightMode::Manual);
        assert!(checker.is_active());
        // Reset
        checker.reset();
        assert!(!checker.is_active());
        // After reset, two-stage needed again
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::Manual);
        assert_eq!(action, GcsFailsafeAction::SetMode(FlightMode::Hold));
    }

    #[test]
    fn test_rapid_arm_disarm_cycle() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        // Arm, condition starts detecting
        checker.check(0, 10, 5_500_000, true, FlightMode::Manual);
        // Disarm - should clear condition tracking
        checker.check(0, 10, 6_000_000, false, FlightMode::Manual);
        // Re-arm with reset
        checker.reset();
        // Should not fire immediately (persistence timer reset)
        assert_eq!(
            checker.check(0, 10, 6_000_000, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );
    }

    #[test]
    fn test_disarm_action_fallback_to_hold() {
        let params = FailsafeParams {
            action: FailsafeAction::Disarm as u8,
            timeout: 1.5,
            gcs_timeout: 5.0,
            gcs_enable: true,
        };
        let config = GcsFailsafeConfig::from_params(&params);
        assert_eq!(config.action, FailsafeAction::Hold);

        let mut checker = GcsFailsafeChecker::new(config);
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::Manual);
        assert_eq!(action, GcsFailsafeAction::SetMode(FlightMode::Hold));
    }

    #[test]
    fn test_clock_non_monotonicity() {
        let mut checker = GcsFailsafeChecker::new(default_config());
        // current_time < last_heartbeat (clock went backwards)
        // saturating_sub returns 0, so no timeout detected
        assert_eq!(
            checker.check(10_000_000, 10, 5_000_000, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );
    }

    #[test]
    fn test_rtl_action() {
        let mut checker = GcsFailsafeChecker::new(rtl_config());
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::Manual);
        assert_eq!(action, GcsFailsafeAction::SetMode(FlightMode::Rtl));
    }

    #[test]
    fn test_none_action() {
        let config = GcsFailsafeConfig {
            action: FailsafeAction::None,
            ..default_config()
        };
        let mut checker = GcsFailsafeChecker::new(config);
        // With action=None, even after timeout, no mode change
        let action = trigger_failsafe(&mut checker, 0, 10, FlightMode::Manual);
        assert_eq!(action, GcsFailsafeAction::None);
    }

    #[test]
    fn test_none_action_no_stuck_failsafe_active() {
        let config = GcsFailsafeConfig {
            action: FailsafeAction::None,
            ..default_config()
        };
        let mut checker = GcsFailsafeChecker::new(config);
        // Trigger with action=None
        trigger_failsafe(&mut checker, 0, 10, FlightMode::Manual);
        // failsafe_active should NOT be set when action=None
        assert!(!checker.is_active());
        // Recovery should return None (not Cleared) since failsafe was never activated
        let resume_time = TOTAL_TIMEOUT_US + 1_000_000;
        assert_eq!(
            checker.check(resume_time, 11, resume_time, true, FlightMode::Manual),
            GcsFailsafeAction::None,
        );
    }

    #[test]
    fn test_from_params_default() {
        let params = FailsafeParams {
            action: FailsafeAction::Hold as u8,
            timeout: 1.5,
            gcs_timeout: 5.0,
            gcs_enable: true,
        };
        let config = GcsFailsafeConfig::from_params(&params);
        assert!(config.enabled);
        assert_eq!(config.gcs_timeout_us, 5_000_000);
        assert_eq!(config.persistence_us, 1_500_000);
        assert_eq!(config.action, FailsafeAction::Hold);
    }

    #[test]
    fn test_from_params_invalid_action() {
        let params = FailsafeParams {
            action: 99, // invalid
            timeout: 1.5,
            gcs_timeout: 5.0,
            gcs_enable: true,
        };
        let config = GcsFailsafeConfig::from_params(&params);
        // Invalid action falls back to Hold
        assert_eq!(config.action, FailsafeAction::Hold);
    }

    #[test]
    fn test_from_params_negative_timeout_uses_default() {
        let params = FailsafeParams {
            action: FailsafeAction::Hold as u8,
            timeout: -1.0,
            gcs_timeout: -5.0,
            gcs_enable: true,
        };
        let config = GcsFailsafeConfig::from_params(&params);
        assert_eq!(config.gcs_timeout_us, 5_000_000); // default 5.0s
        assert_eq!(config.persistence_us, 1_500_000); // default 1.5s
    }

    #[test]
    fn test_from_params_nan_timeout_uses_default() {
        let params = FailsafeParams {
            action: FailsafeAction::Hold as u8,
            timeout: f32::NAN,
            gcs_timeout: f32::NAN,
            gcs_enable: true,
        };
        let config = GcsFailsafeConfig::from_params(&params);
        assert_eq!(config.gcs_timeout_us, 5_000_000);
        assert_eq!(config.persistence_us, 1_500_000);
    }

    #[test]
    fn test_from_params_infinity_timeout_uses_default() {
        let params = FailsafeParams {
            action: FailsafeAction::Hold as u8,
            timeout: f32::INFINITY,
            gcs_timeout: f32::NEG_INFINITY,
            gcs_enable: true,
        };
        let config = GcsFailsafeConfig::from_params(&params);
        assert_eq!(config.gcs_timeout_us, 5_000_000);
        assert_eq!(config.persistence_us, 1_500_000);
    }

    #[test]
    fn test_status_text_set_mode_hold() {
        let action = GcsFailsafeAction::SetMode(FlightMode::Hold);
        assert_eq!(action.status_text(), Some("Failsafe: GCS Lost - Hold"));
        assert_eq!(action.is_warning(), Some(true));
    }

    #[test]
    fn test_status_text_set_mode_rtl() {
        let action = GcsFailsafeAction::SetMode(FlightMode::Rtl);
        assert_eq!(action.status_text(), Some("Failsafe: GCS Lost - RTL"));
        assert_eq!(action.is_warning(), Some(true));
    }

    #[test]
    fn test_status_text_activated() {
        let action = GcsFailsafeAction::Activated;
        assert_eq!(
            action.status_text(),
            Some("Failsafe: GCS Lost - already in safe mode")
        );
        assert_eq!(action.is_warning(), Some(true));
    }

    #[test]
    fn test_status_text_cleared() {
        let action = GcsFailsafeAction::Cleared;
        assert_eq!(action.status_text(), Some("GCS Failsafe Cleared"));
        assert_eq!(action.is_warning(), Some(false));
    }

    #[test]
    fn test_status_text_none() {
        let action = GcsFailsafeAction::None;
        assert_eq!(action.status_text(), Option::None);
        assert_eq!(action.is_warning(), Option::None);
    }
}
