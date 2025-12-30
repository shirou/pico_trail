//! Task implementations for scheduler
//!
//! This module contains Embassy async tasks for various subsystems:
//! - Example tasks demonstrating scheduler usage
//! - Control loop task (50 Hz mode execution, vehicle-agnostic)
//!
//! Each task follows the pattern of:
//! 1. Using Ticker for periodic execution
//! 2. Measuring execution time
//! 3. Logging errors and status updates
//!
//! ## Platform Location
//!
//! The actual task implementations live in `src/platform/rp2350/tasks/`
//! because they require Embassy runtime (`#[embassy_executor::task]`).
//! This module re-exports them for API compatibility.

// Re-export tasks from platform module
#[cfg(feature = "pico2_w")]
pub use crate::platform::rp2350::tasks::control::control_loop_task;

#[cfg(feature = "pico2_w")]
pub use crate::platform::rp2350::tasks::examples::{
    ahrs_task, control_task, imu_task, telemetry_task,
};
