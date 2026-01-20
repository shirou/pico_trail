//! Mode State Machine Types and Utilities
//!
//! This module provides pure types and utilities for control mode state machines.
//! Platform-specific implementations (Embassy, ActuatorInterface) remain in the
//! firmware crate.
//!
//! # Contents
//!
//! - `Mode` trait definition (platform-agnostic interface)
//! - State types for each mode (AutoState, GuidedState, RtlState)
//! - Navigation calculation utilities (haversine distance/bearing)
//!
//! # References
//!
//! - ADR-w9zpl-control-mode-architecture: Trait-based mode architecture
//! - FR-sp3at-control-modes: Mode requirements

mod nav;
mod state;
mod traits;

pub use nav::{haversine_distance_bearing, normalize_angle};
pub use state::{AutoState, GuidedState, RtlState};
pub use traits::Mode;
