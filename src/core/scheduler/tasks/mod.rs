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

#[cfg(feature = "pico2_w")]
pub mod examples;

#[cfg(feature = "pico2_w")]
pub mod control;

// Re-export control loop task for convenience
#[cfg(feature = "pico2_w")]
pub use control::control_loop_task;
