//! RC input state and processing (firmware wrapper)
//!
//! This module re-exports core RC types and adds Embassy-specific global state.
//! All pure RC logic is in `pico_trail_core::rc`.
//!
//! ## References
//!
//! - ADR-ea7fw-rc-input-processing: RC input design
//! - FR-993xy-rc-channels-processing: RC requirements

use crate::core::traits::EmbassyState;

// Re-export all core RC types and functions
pub use pico_trail_core::rc::{
    normalize_channel, normalize_pwm_channel, normalize_pwm_channel_inverted, RcInput, RcStatus,
    RC_TIMEOUT_US,
};

/// Global RC input (protected by EmbassyState)
///
/// Uses blocking mutex with critical sections for interrupt-safe access.
/// Access via `SharedState` trait methods: `.with()` for read, `.with_mut()` for write.
pub static RC_INPUT: EmbassyState<RcInput> = EmbassyState::new(RcInput::new());
