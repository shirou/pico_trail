//! RC input state and processing (firmware wrapper)
//!
//! This module re-exports core RC types and the global RC_INPUT state.
//! All pure RC logic is in `pico_trail_core::rc`.
//!
//! ## References
//!
//! - ADR-ea7fw-rc-input-processing: RC input design
//! - FR-993xy-rc-channels-processing: RC requirements

// Re-export all core RC types and functions
pub use pico_trail_core::rc::{
    normalize_channel, normalize_pwm_channel, normalize_pwm_channel_inverted, RcInput, RcStatus,
    RC_INPUT, RC_TIMEOUT_US,
};
