//! Core traits for platform-agnostic autopilot functionality.
//!
//! This module provides trait abstractions that decouple core autopilot logic
//! from platform-specific implementations (Embassy, etc.).
//!
//! # Design
//!
//! - Trait definitions are pure and have no feature gates
//! - Mock implementations are always available for host testing
//! - Embassy implementations are behind the `embassy` feature flag
//! - Platform implementations (Embassy tasks) live in the firmware crate

pub mod sync;
pub mod time;

#[cfg(feature = "embassy")]
pub use sync::EmbassyState;
pub use sync::{MockState, SharedState};
pub use time::{MockTime, TimeSource};
