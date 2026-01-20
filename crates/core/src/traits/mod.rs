//! Core traits for platform-agnostic autopilot functionality.
//!
//! This module provides trait abstractions that decouple core autopilot logic
//! from platform-specific implementations (Embassy, etc.).
//!
//! # Design
//!
//! - Trait definitions are pure and have no feature gates
//! - Mock implementations are always available for host testing
//! - Platform implementations (Embassy) live in the firmware crate

pub mod time;

pub use time::{MockTime, TimeSource};
