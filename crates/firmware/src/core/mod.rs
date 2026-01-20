//! Core autopilot functionality
//!
//! This module contains the core components of the pico_trail autopilot system,
//! including the task scheduler and other fundamental infrastructure.
//!
//! This module provides firmware-specific core code and re-exports
//! pure business logic from pico_trail_core.

// Firmware-specific modules
pub mod log_buffer;
pub mod log_router;
pub mod logging;

// Firmware-specific arming module (contains Embassy-specific tasks)
pub mod arming;

// Firmware-specific mission module (contains Embassy mutex wrappers)
pub mod mission;

// Firmware-specific parameters module (contains saver and other platform code)
pub mod parameters;

// Firmware-specific scheduler module
pub mod scheduler;

// Firmware-specific traits module
pub mod traits;

// Re-export pure types from pico_trail_core for compatibility
// This allows code to use crate::core::X where X is from the core library
pub use pico_trail_core::kinematics;
pub use pico_trail_core::mode;
pub use pico_trail_core::motor;
pub use pico_trail_core::navigation;
pub use pico_trail_core::rc;
pub use pico_trail_core::servo;
