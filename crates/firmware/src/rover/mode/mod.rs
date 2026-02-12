//! Rover control modes
//!
//! This module re-exports control mode implementations from the core crate.
//! Following ArduPilot's architecture, each mode implements the `Mode` trait
//! with enter/update/exit lifecycle hooks.
//!
//! ## Available Modes
//!
//! - **Manual**: Direct RC control (no stabilization)
//! - **Hold**: Stop in place
//! - **Auto**: Follow waypoint mission
//! - **RTL**: Return to launch
//! - **Guided**: Accept real-time commands from GCS
//! - **Loiter**: Position hold with optional active correction
//! - **Circle**: Autonomous circular orbit
//! - **SmartRTL**: Smart return to launch via recorded path
//!
//! ## References
//!
//! - ADR-w9zpl-control-mode-architecture: Trait-based mode architecture
//! - FR-sp3at-control-modes: Mode requirements
//! - ArduPilot Rover modes: https://ardupilot.org/rover/docs/rover-control-modes.html

// Re-export all mode implementations from core
pub use pico_trail_core::mode::Mode;

// Non-embassy modes (always available)
pub use pico_trail_core::mode::circle::{CircleConfig, CircleMode};
pub use pico_trail_core::mode::loiter::{LoiterState, RoverLoiter};
pub use pico_trail_core::mode::rtl::RtlMode;
pub use pico_trail_core::mode::smartrtl::SmartRtlMode;

// Re-export CircleDirection from parameters
pub use pico_trail_core::parameters::circle::CircleDirection;

// Embassy-gated modes
pub use pico_trail_core::mode::auto::AutoMode;
pub use pico_trail_core::mode::guided::GuidedMode;
pub use pico_trail_core::mode::manual::ManualMode;
