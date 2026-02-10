//! Rover vehicle implementation
//!
//! This module contains rover-specific control logic, following ArduPilot's
//! vehicle-specific architecture (Rover/, ArduCopter/, ArduPlane/).
//!
//! ## Modules
//!
//! - `mode`: Control mode implementations (re-exported from core)
//! - `mode_manager`: Mode lifecycle management (re-exported from core as ModeManager)
//!
//! ## References
//!
//! - ArduPilot Rover: https://github.com/ArduPilot/ardupilot/tree/master/Rover
//! - FR-sp3at-control-modes: Rover mode requirements

pub mod mode;
pub mod mode_manager;

// Re-export commonly used types
pub use mode::Mode;
pub use mode_manager::ModeManager;
