//! Arming System
//!
//! Provides comprehensive arming safety system with pre-arm checks,
//! post-arm initialization, armed state monitoring, and pre-disarm validation.
//!
//! # Architecture
//!
//! The arming system consists of five subsystems:
//! - **Pre-Arm Checks**: Validate system health before allowing arming
//! - **Post-Arm Initialization**: Initialize subsystems after successful arming
//! - **Armed State Monitoring**: Continuous health monitoring during armed operation
//! - **Pre-Disarm Validation**: Safety checks before allowing disarm
//! - **Post-Disarm Cleanup**: Cleanup and shutdown after disarming
//!
//! # Usage
//!
//! ```ignore
//! use pico_trail::core::arming::{ArmingChecker, CheckCategory};
//!
//! // Create checker with enabled categories from ARMING_CHECK parameter
//! let mut checker = ArmingChecker::new(0x00A8); // Battery + INS + RC
//!
//! // Run all enabled pre-arm checks
//! match checker.run_checks(&context) {
//!     Ok(()) => {
//!         // Checks passed, safe to arm
//!         system_state.armed = ArmedState::Armed;
//!     }
//!     Err(e) => {
//!         // Check failed, report reason to user
//!         log::warn!("Arming denied: {}", e);
//!     }
//! }
//! ```

pub mod checks;
pub mod cleanup;
pub mod disarm;
pub mod error;
pub mod initialization;
pub mod monitoring;
pub mod tasks;

pub use checks::{create_default_checker, ArmingChecker, CheckResult, PreArmCheck};
pub use cleanup::{CleanupError, PostDisarmCleanup};
pub use disarm::{DisarmMethod, DisarmReason, DisarmValidator};
pub use error::{ArmingError, CheckCategory, DisarmError};
pub use initialization::{ArmMethod, PostArmInitializer};
pub use monitoring::{
    ArmedStateMonitor, EkfStatus, FailsafeReason, FenceStatus, SensorHealthFlags,
};

// Embassy async task integration
pub use tasks::{monitoring_loop_fast, monitoring_loop_medium, monitoring_loop_slow};
