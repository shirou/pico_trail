//! Arming system types and logic
//!
//! This module provides arming-related types, error definitions,
//! pre-arm checks, post-arm initialization, disarm validation,
//! and post-disarm cleanup.
//!
//! Platform-specific arming tasks and async monitoring are in the firmware crate.

pub mod checks;
pub mod cleanup;
pub mod disarm;
pub mod error;
pub mod initialization;

pub use checks::{create_default_checker, CheckResult};
pub use checks::{ArmingChecker, BatteryVoltageCheck, PreArmCheck, SystemStateCheck};
pub use cleanup::{CleanupError, PostDisarmCleanup};
pub use disarm::{DisarmMethod, DisarmReason, DisarmValidator};
pub use error::{ArmingError, CheckCategory, DisarmError};
pub use initialization::{ArmMethod, PostArmInitializer};
