//! Arming System
//!
//! Re-exports core arming logic from `pico_trail_core::arming` and provides
//! firmware-specific monitoring tasks and async integration.

// Re-export core arming modules
pub use pico_trail_core::arming::checks;
pub use pico_trail_core::arming::cleanup;
pub use pico_trail_core::arming::disarm;
pub use pico_trail_core::arming::initialization;

// Firmware-specific modules
pub mod monitoring;
pub mod tasks;

// Re-export commonly used types from core
pub use pico_trail_core::arming::{
    create_default_checker, ArmMethod, ArmingChecker, ArmingError, BatteryVoltageCheck,
    CheckCategory, CheckResult, CleanupError, DisarmError, DisarmMethod, DisarmReason,
    DisarmValidator, PostArmInitializer, PostDisarmCleanup, PreArmCheck, SystemStateCheck,
};

pub use monitoring::{
    ArmedStateMonitor, EkfStatus, FailsafeReason, FenceStatus, SensorHealthFlags,
};

// Embassy async task integration
pub use tasks::{monitoring_loop_fast, monitoring_loop_medium, monitoring_loop_slow};
