//! Core autopilot functionality
//!
//! This module contains the core components of the pico_trail autopilot system,
//! including the task scheduler and other fundamental infrastructure.

pub mod arming;
pub mod log_buffer;
pub mod log_router;
pub mod logging;
pub mod mission;
pub mod parameters;
pub mod scheduler;
