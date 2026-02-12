//! pico_trail_core - Pure no_std business logic for pico_trail autopilot
//!
//! This crate contains platform-agnostic algorithms and types
//! that can be tested on host without any feature flags or embassy dependencies.
//!
//! # Design Principles
//!
//! - **Pure no_std**: No std library dependencies
//! - **Trait abstractions**: Platform services injected via traits
//! - **Embassy feature**: Optional `embassy` feature enables embassy-sync
//!   based implementations (EmbassyState, etc.)
//!
//! # Modules
//!
//! - [`traits`]: Platform-agnostic trait abstractions (TimeSource)
//! - [`kinematics`]: Vehicle motion control algorithms
//! - [`parameters`]: Parameter block types and CRC utilities
//! - [`arming`]: Arming system error types and check categories
//! - [`navigation`]: Navigation types (PositionTarget, NavigationOutput)
//! - [`scheduler`]: Task scheduler types and statistics
//! - [`mission`]: Mission waypoint storage and types
//! - [`mode`]: Mode trait, state types, and navigation utilities
//! - [`rc`]: RC input state and channel normalization
//! - [`servo`]: Actuator abstraction for steering and throttle
//! - [`motor`]: Motor driver abstraction for DC motors
//! - [`autopilot`]: Autopilot state types (SystemState, FlightMode, VehicleType)
//! - [`ahrs`]: AHRS calibration types and utilities

#![no_std]

// serial_test macro expansion requires std to be linked at crate root
#[cfg(test)]
extern crate std;

// Logging macros (must be declared before other modules that use them)
#[macro_use]
pub mod logging;

pub mod ahrs;
pub mod arming;
pub mod autopilot;
pub mod communication;
pub mod kinematics;
pub mod mission;
pub mod mode;
pub mod motor;
pub mod navigation;
pub mod parameters;
pub mod rc;
pub mod scheduler;
pub mod servo;
pub mod traits;
