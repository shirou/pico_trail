//! Battery Failsafe Checker
//!
//! Voltage-based battery failsafe with two stages:
//! - **LOW**: Warning with configurable action, requires sustained voltage drop (hysteresis)
//! - **CRITICAL**: Immediate action, no hysteresis
//!
//! Pure logic module with no async or embassy dependencies. Fully testable on host.

use crate::parameters::battery::{BatteryFailsafeAction, BatteryParams};

/// Battery failsafe severity level
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryFailsafeLevel {
    /// Low voltage warning (sustained threshold breach)
    Low,
    /// Critical voltage (immediate action required)
    Critical,
}

/// Event emitted when a failsafe threshold is breached
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryFailsafeEvent {
    pub level: BatteryFailsafeLevel,
    pub action: BatteryFailsafeAction,
}

/// Configuration for battery failsafe thresholds and actions
#[derive(Debug, Clone)]
pub struct BatteryFailsafeConfig {
    /// Low voltage threshold (V), 0.0 = disabled
    pub low_voltage: f32,
    /// Action when low threshold is reached
    pub low_action: BatteryFailsafeAction,
    /// Critical voltage threshold (V), 0.0 = disabled
    pub critical_voltage: f32,
    /// Action when critical threshold is reached
    pub critical_action: BatteryFailsafeAction,
}

impl BatteryFailsafeConfig {
    /// Build config from BatteryParams
    pub fn from_params(params: &BatteryParams) -> Self {
        Self {
            low_voltage: params.low_voltage,
            low_action: BatteryFailsafeAction::from_u8(params.low_action),
            critical_voltage: params.critical_voltage,
            critical_action: BatteryFailsafeAction::from_u8(params.critical_action),
        }
    }
}

/// Number of consecutive samples below LOW threshold required to trigger (10s at 10Hz)
const LOW_HYSTERESIS_COUNT: u32 = 100;

/// Stateful battery failsafe checker with hysteresis
pub struct BatteryFailsafeChecker {
    config: BatteryFailsafeConfig,
    /// Counter for consecutive samples below LOW threshold
    low_count: u32,
    /// Whether LOW failsafe has already triggered (sticky)
    low_triggered: bool,
    /// Whether CRITICAL failsafe has already triggered (sticky)
    critical_triggered: bool,
}

impl BatteryFailsafeChecker {
    /// Create a new checker from configuration
    pub fn new(config: BatteryFailsafeConfig) -> Self {
        Self {
            config,
            low_count: 0,
            low_triggered: false,
            critical_triggered: false,
        }
    }

    /// Check voltage against failsafe thresholds
    ///
    /// Called at 10 Hz from the battery update loop.
    ///
    /// Returns `Some(event)` on first trigger of each level, `None` otherwise.
    /// Once triggered, each level is sticky until `reset()` is called.
    ///
    /// Skips checks when:
    /// - `is_armed` is false (resets hysteresis counter)
    /// - `voltage` is 0.0 (uninitialized ADC)
    /// - threshold is 0.0 (disabled)
    pub fn check(&mut self, voltage: f32, is_armed: bool) -> Option<BatteryFailsafeEvent> {
        // Skip when disarmed - reset hysteresis counter
        if !is_armed {
            self.low_count = 0;
            return None;
        }

        // Skip uninitialized voltage reading
        if voltage == 0.0 {
            return None;
        }

        // CRITICAL check first (immediate, no hysteresis)
        if !self.critical_triggered
            && self.config.critical_voltage > 0.0
            && voltage < self.config.critical_voltage
        {
            self.critical_triggered = true;
            return Some(BatteryFailsafeEvent {
                level: BatteryFailsafeLevel::Critical,
                action: self.config.critical_action,
            });
        }

        // LOW check with hysteresis
        if !self.low_triggered && self.config.low_voltage > 0.0 {
            if voltage < self.config.low_voltage {
                self.low_count += 1;
                if self.low_count >= LOW_HYSTERESIS_COUNT {
                    self.low_triggered = true;
                    return Some(BatteryFailsafeEvent {
                        level: BatteryFailsafeLevel::Low,
                        action: self.config.low_action,
                    });
                }
            } else {
                // Voltage recovered - reset counter
                self.low_count = 0;
            }
        }

        None
    }

    /// Reset all failsafe state (used on disarm/rearm cycle)
    pub fn reset(&mut self) {
        self.low_count = 0;
        self.low_triggered = false;
        self.critical_triggered = false;
    }

    /// Reset state and reload configuration
    ///
    /// Called on arm transition to clear sticky flags and pick up any
    /// parameter changes made while disarmed.
    pub fn reset_with_config(&mut self, config: BatteryFailsafeConfig) {
        self.config = config;
        self.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> BatteryFailsafeConfig {
        BatteryFailsafeConfig {
            low_voltage: 3.5,
            low_action: BatteryFailsafeAction::Hold,
            critical_voltage: 3.0,
            critical_action: BatteryFailsafeAction::Hold,
        }
    }

    #[test]
    fn test_normal_voltage_no_trigger() {
        let mut checker = BatteryFailsafeChecker::new(default_config());
        for _ in 0..200 {
            assert_eq!(checker.check(4.0, true), None);
        }
    }

    #[test]
    fn test_critical_triggers_immediately() {
        let mut checker = BatteryFailsafeChecker::new(default_config());
        let event = checker.check(2.5, true);
        assert_eq!(
            event,
            Some(BatteryFailsafeEvent {
                level: BatteryFailsafeLevel::Critical,
                action: BatteryFailsafeAction::Hold,
            })
        );
    }

    #[test]
    fn test_low_requires_hysteresis() {
        let mut checker = BatteryFailsafeChecker::new(default_config());

        // 99 samples below LOW → no trigger yet
        for _ in 0..99 {
            assert_eq!(checker.check(3.3, true), None);
        }

        // 100th sample triggers
        let event = checker.check(3.3, true);
        assert_eq!(
            event,
            Some(BatteryFailsafeEvent {
                level: BatteryFailsafeLevel::Low,
                action: BatteryFailsafeAction::Hold,
            })
        );
    }

    #[test]
    fn test_recovery_resets_hysteresis() {
        let mut checker = BatteryFailsafeChecker::new(default_config());

        // 50 samples below threshold
        for _ in 0..50 {
            assert_eq!(checker.check(3.3, true), None);
        }

        // Voltage recovers
        assert_eq!(checker.check(4.0, true), None);

        // Another 99 samples below → no trigger (counter was reset)
        for _ in 0..99 {
            assert_eq!(checker.check(3.3, true), None);
        }

        // 100th consecutive sample triggers
        let event = checker.check(3.3, true);
        assert_eq!(
            event,
            Some(BatteryFailsafeEvent {
                level: BatteryFailsafeLevel::Low,
                action: BatteryFailsafeAction::Hold,
            })
        );
    }

    #[test]
    fn test_critical_takes_priority_over_low() {
        let mut checker = BatteryFailsafeChecker::new(default_config());

        // Voltage below both thresholds → critical fires first
        let event = checker.check(2.5, true);
        assert_eq!(
            event,
            Some(BatteryFailsafeEvent {
                level: BatteryFailsafeLevel::Critical,
                action: BatteryFailsafeAction::Hold,
            })
        );
    }

    #[test]
    fn test_triggers_only_once_sticky() {
        let mut checker = BatteryFailsafeChecker::new(default_config());

        // First critical trigger
        assert!(checker.check(2.5, true).is_some());

        // Subsequent checks → no more events
        for _ in 0..10 {
            assert_eq!(checker.check(2.5, true), None);
        }
    }

    #[test]
    fn test_low_triggers_only_once_sticky() {
        let mut checker = BatteryFailsafeChecker::new(default_config());

        // Trigger LOW after 100 samples
        for _ in 0..100 {
            checker.check(3.3, true);
        }

        // Subsequent checks → no more events
        for _ in 0..50 {
            assert_eq!(checker.check(3.3, true), None);
        }
    }

    #[test]
    fn test_reset_clears_state() {
        let mut checker = BatteryFailsafeChecker::new(default_config());

        // Trigger critical
        assert!(checker.check(2.5, true).is_some());

        // Reset
        checker.reset();

        // Critical fires again
        let event = checker.check(2.5, true);
        assert_eq!(
            event,
            Some(BatteryFailsafeEvent {
                level: BatteryFailsafeLevel::Critical,
                action: BatteryFailsafeAction::Hold,
            })
        );
    }

    #[test]
    fn test_disarmed_no_trigger() {
        let mut checker = BatteryFailsafeChecker::new(default_config());

        // Even with critical voltage, disarmed → no trigger
        for _ in 0..200 {
            assert_eq!(checker.check(2.5, false), None);
        }
    }

    #[test]
    fn test_zero_threshold_disabled() {
        let config = BatteryFailsafeConfig {
            low_voltage: 0.0,
            low_action: BatteryFailsafeAction::Hold,
            critical_voltage: 0.0,
            critical_action: BatteryFailsafeAction::Hold,
        };
        let mut checker = BatteryFailsafeChecker::new(config);

        for _ in 0..200 {
            assert_eq!(checker.check(1.0, true), None);
        }
    }

    #[test]
    fn test_zero_voltage_no_trigger() {
        let mut checker = BatteryFailsafeChecker::new(default_config());
        for _ in 0..200 {
            assert_eq!(checker.check(0.0, true), None);
        }
    }

    #[test]
    fn test_different_actions_per_level() {
        let config = BatteryFailsafeConfig {
            low_voltage: 3.5,
            low_action: BatteryFailsafeAction::RTL,
            critical_voltage: 3.0,
            critical_action: BatteryFailsafeAction::Disarm,
        };
        let mut checker = BatteryFailsafeChecker::new(config);

        // Trigger LOW (100 samples at 3.3V)
        for _ in 0..100 {
            checker.check(3.3, true);
        }
        // LOW was triggered on the 100th sample in the loop above;
        // verify it won't re-fire
        assert_eq!(checker.check(3.3, true), None);

        // Reset and trigger critical
        checker.reset();
        let event = checker.check(2.5, true);
        assert_eq!(
            event,
            Some(BatteryFailsafeEvent {
                level: BatteryFailsafeLevel::Critical,
                action: BatteryFailsafeAction::Disarm,
            })
        );
    }

    #[test]
    fn test_disarm_resets_hysteresis_counter() {
        let mut checker = BatteryFailsafeChecker::new(default_config());

        // 50 samples below LOW while armed
        for _ in 0..50 {
            checker.check(3.3, true);
        }

        // Disarm → resets counter
        checker.check(3.3, false);

        // Re-arm: need full 100 samples again
        for _ in 0..99 {
            assert_eq!(checker.check(3.3, true), None);
        }

        // 100th triggers
        assert!(checker.check(3.3, true).is_some());
    }

    #[test]
    fn test_reset_with_config_clears_state_and_updates_config() {
        let mut checker = BatteryFailsafeChecker::new(default_config());

        // Trigger critical
        assert!(checker.check(2.5, true).is_some());

        // Reset with new config (different critical threshold)
        let new_config = BatteryFailsafeConfig {
            low_voltage: 3.8,
            low_action: BatteryFailsafeAction::RTL,
            critical_voltage: 2.0,
            critical_action: BatteryFailsafeAction::Disarm,
        };
        checker.reset_with_config(new_config);

        // Old critical (2.5V) no longer triggers with new threshold (2.0V)
        assert_eq!(checker.check(2.5, true), None);

        // New critical threshold triggers
        let event = checker.check(1.5, true);
        assert_eq!(
            event,
            Some(BatteryFailsafeEvent {
                level: BatteryFailsafeLevel::Critical,
                action: BatteryFailsafeAction::Disarm,
            })
        );
    }
}
