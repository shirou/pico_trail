//! Rover vehicle implementation
//!
//! This module contains rover-specific control logic, following ArduPilot's
//! vehicle-specific architecture (Rover/, ArduCopter/, ArduPlane/).
//!
//! ## Modules
//!
//! - `mode`: Control mode implementations (Manual, Hold, Auto, RTL, Guided)
//!
//! ## References
//!
//! - ArduPilot Rover: https://github.com/ArduPilot/ardupilot/tree/master/Rover
//! - FR-sp3at-control-modes: Rover mode requirements

pub mod mode;

// Re-export Mode trait for convenience
pub use mode::Mode;
