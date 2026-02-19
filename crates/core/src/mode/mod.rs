//! Mode State Machine Types and Implementations
//!
//! This module provides control mode trait, state types, and mode implementations
//! for rover vehicles. All modes implement the `Mode` trait with enter/update/exit
//! lifecycle hooks.
//!
//! # Contents
//!
//! - `Mode` trait definition (platform-agnostic interface)
//! - State types for each mode (AutoState, GuidedState, RtlState)
//! - Navigation calculation utilities (haversine distance/bearing)
//! - Mode implementations (Manual, Auto, Guided, RTL, Loiter, Circle, SmartRTL)
//! - ModeExecutor for mode lifecycle management
//!
//! # References
//!
//! - ADR-w9zpl-control-mode-architecture: Trait-based mode architecture
//! - FR-sp3at-control-modes: Mode requirements

mod nav;
mod state;
mod traits;

// Mode implementations (no embassy dependency)
pub mod circle;
pub mod hold;
pub mod loiter;
pub mod rtl;
pub mod smartrtl;

// Embassy-gated modes (depend on mission globals or embassy-sync)
#[cfg(feature = "embassy")]
pub mod auto;
#[cfg(feature = "embassy")]
pub mod guided;
#[cfg(feature = "embassy")]
pub mod manual;
#[cfg(feature = "embassy")]
pub mod mode_executor;

pub use nav::{haversine_distance_bearing, normalize_angle};
pub use state::{AutoState, GuidedState, RtlState};
pub use traits::Mode;

// Re-export mode types (non-embassy)
pub use circle::{CircleConfig, CircleMode};
pub use hold::HoldMode;
pub use loiter::{LoiterState, RoverLoiter};
pub use rtl::RtlMode;
pub use smartrtl::SmartRtlMode;

// Embassy-gated re-exports
#[cfg(feature = "embassy")]
pub use auto::AutoMode;
#[cfg(feature = "embassy")]
pub use guided::GuidedMode;
#[cfg(feature = "embassy")]
pub use manual::ManualMode;
#[cfg(feature = "embassy")]
pub use mode_executor::ModeExecutor;
