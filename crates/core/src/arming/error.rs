//! Arming System Error Types
//!
//! Defines error types for arming and disarming operations.

use core::fmt;

/// Errors that can occur during arming operations
#[derive(Debug, Clone, PartialEq)]
pub enum ArmingError {
    /// Pre-arm check failed
    CheckFailed {
        /// Human-readable reason for failure
        reason: &'static str,
        /// Check category that failed
        category: CheckCategory,
    },
    /// Initialization step failed after successful pre-arm checks
    InitializationFailed {
        /// Subsystem that failed to initialize
        subsystem: &'static str,
    },
    /// Vehicle is already armed
    AlreadyArmed,
}

impl fmt::Display for ArmingError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ArmingError::CheckFailed { reason, category } => {
                write!(f, "Pre-arm check failed ({}): {}", category, reason)
            }
            ArmingError::InitializationFailed { subsystem } => {
                write!(f, "Post-arm initialization failed: {}", subsystem)
            }
            ArmingError::AlreadyArmed => write!(f, "Vehicle is already armed"),
        }
    }
}

/// Errors that can occur during disarming operations
#[derive(Debug, Clone, PartialEq)]
pub enum DisarmError {
    /// Pre-disarm validation failed
    ValidationFailed {
        /// Human-readable reason for failure
        reason: &'static str,
    },
    /// Vehicle is not armed
    NotArmed,
    /// Throttle is too high to safely disarm
    ThrottleNotLow {
        /// Current throttle percentage (0.0-100.0)
        current: f32,
    },
    /// Vehicle velocity is too high to safely disarm
    VelocityTooHigh {
        /// Current velocity (m/s)
        current: f32,
        /// Maximum allowed velocity (m/s)
        max: f32,
    },
}

impl fmt::Display for DisarmError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            DisarmError::ValidationFailed { reason } => {
                write!(f, "Disarm validation failed: {}", reason)
            }
            DisarmError::NotArmed => write!(f, "Vehicle is not armed"),
            DisarmError::ThrottleNotLow { current } => {
                write!(
                    f,
                    "Cannot disarm: Throttle at {:.1}% (must be < 10%)",
                    current
                )
            }
            DisarmError::VelocityTooHigh { current, max } => {
                write!(
                    f,
                    "Cannot disarm: Ground speed {:.2} m/s exceeds {:.2} m/s maximum",
                    current, max
                )
            }
        }
    }
}

/// Pre-arm check categories (ArduPilot ARMING_CHECK bitmask values)
///
/// These categories allow selective enabling/disabling of pre-arm checks
/// via the ARMING_CHECK parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum CheckCategory {
    /// All checks (bitmask: 0xFFFF)
    All = 0xFFFF,
    /// Barometer check (bitmask: 0x0001)
    Barometer = 0x0001,
    /// Compass check (bitmask: 0x0002)
    Compass = 0x0002,
    /// GPS check (bitmask: 0x0004)
    Gps = 0x0004,
    /// IMU/INS check (bitmask: 0x0008)
    Ins = 0x0008,
    /// Parameters check (bitmask: 0x0010)
    Parameters = 0x0010,
    /// RC channels check (bitmask: 0x0020)
    RcChannels = 0x0020,
    /// Board voltage check (bitmask: 0x0040)
    Board = 0x0040,
    /// Battery check (bitmask: 0x0080)
    Battery = 0x0080,
    /// Airspeed check (bitmask: 0x0100)
    Airspeed = 0x0100,
    /// Logging check (bitmask: 0x0200)
    Logging = 0x0200,
    /// Safety switch check (bitmask: 0x0400)
    Switch = 0x0400,
    /// GPS configuration check (bitmask: 0x0800)
    GpsCfg = 0x0800,
    /// System check (bitmask: 0x1000)
    System = 0x1000,
}

impl CheckCategory {
    /// Check if this category is enabled in the given bitmask
    pub fn is_enabled(self, bitmask: u16) -> bool {
        (bitmask & (self as u16)) != 0
    }
}

impl fmt::Display for CheckCategory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CheckCategory::All => write!(f, "All"),
            CheckCategory::Barometer => write!(f, "Barometer"),
            CheckCategory::Compass => write!(f, "Compass"),
            CheckCategory::Gps => write!(f, "GPS"),
            CheckCategory::Ins => write!(f, "IMU/INS"),
            CheckCategory::Parameters => write!(f, "Parameters"),
            CheckCategory::RcChannels => write!(f, "RC Channels"),
            CheckCategory::Board => write!(f, "Board"),
            CheckCategory::Battery => write!(f, "Battery"),
            CheckCategory::Airspeed => write!(f, "Airspeed"),
            CheckCategory::Logging => write!(f, "Logging"),
            CheckCategory::Switch => write!(f, "Safety Switch"),
            CheckCategory::GpsCfg => write!(f, "GPS Config"),
            CheckCategory::System => write!(f, "System"),
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::format;

    use super::*;

    #[test]
    fn test_check_category_is_enabled() {
        let bitmask = 0x00A8; // Battery (0x80) + Ins (0x08) + RcChannels (0x20)

        assert!(CheckCategory::Battery.is_enabled(bitmask));
        assert!(CheckCategory::Ins.is_enabled(bitmask));
        assert!(CheckCategory::RcChannels.is_enabled(bitmask));

        assert!(!CheckCategory::Gps.is_enabled(bitmask));
        assert!(!CheckCategory::Compass.is_enabled(bitmask));
    }

    #[test]
    fn test_check_category_all_enabled() {
        let bitmask = 0xFFFF; // All checks enabled
        assert!(CheckCategory::Battery.is_enabled(bitmask));
        assert!(CheckCategory::Gps.is_enabled(bitmask));
        assert!(CheckCategory::Ins.is_enabled(bitmask));
    }

    #[test]
    fn test_check_category_none_enabled() {
        let bitmask = 0x0000; // All checks disabled
        assert!(!CheckCategory::Battery.is_enabled(bitmask));
        assert!(!CheckCategory::Gps.is_enabled(bitmask));
        assert!(!CheckCategory::Ins.is_enabled(bitmask));
    }

    #[test]
    fn test_arming_error_display() {
        let error = ArmingError::CheckFailed {
            reason: "No RC signal received",
            category: CheckCategory::RcChannels,
        };
        assert_eq!(
            format!("{}", error),
            "Pre-arm check failed (RC Channels): No RC signal received"
        );

        let error = ArmingError::AlreadyArmed;
        assert_eq!(format!("{}", error), "Vehicle is already armed");
    }

    #[test]
    fn test_disarm_error_display() {
        let error = DisarmError::ThrottleNotLow { current: 45.0 };
        assert_eq!(
            format!("{}", error),
            "Cannot disarm: Throttle at 45.0% (must be < 10%)"
        );

        let error = DisarmError::VelocityTooHigh {
            current: 1.2,
            max: 0.5,
        };
        assert_eq!(
            format!("{}", error),
            "Cannot disarm: Ground speed 1.20 m/s exceeds 0.50 m/s maximum"
        );
    }
}
