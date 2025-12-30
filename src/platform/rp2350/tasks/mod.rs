//! RP2350 Platform Embassy Tasks
//!
//! This module contains Embassy async tasks that are platform-specific and require
//! the `pico2_w` feature. These tasks use `#[embassy_executor::task]` attribute
//! which requires the Embassy runtime available only on embedded targets.
//!
//! ## Available Tasks
//!
//! - `control_loop_task` - 50 Hz vehicle control loop
//! - `monitor_task` - 1 Hz scheduler monitoring
//! - `mavlink_task` - MAVLink communication task
//! - Example tasks (imu_task, ahrs_task, control_task, telemetry_task)
//!
//! ## Re-exports
//!
//! These tasks are re-exported from `crate::core::scheduler::tasks` for
//! API compatibility with existing code.

pub mod control;
pub mod examples;
pub mod mavlink;
pub mod monitor;

// Re-export task functions
pub use control::control_loop_task;
pub use examples::{ahrs_task, control_task, imu_task, telemetry_task};
pub use mavlink::mavlink_task;
pub use monitor::monitor_task;
