//! Arming system types and logic
//!
//! This module provides arming-related types and error definitions.
//! Platform-specific arming tasks and async checks are in the firmware crate.

pub mod error;

pub use error::{ArmingError, CheckCategory, DisarmError};
