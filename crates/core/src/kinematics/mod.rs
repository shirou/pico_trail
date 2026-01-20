//! Kinematics library for vehicle motion control
//!
//! This module provides pure, platform-independent kinematics conversions for various
//! vehicle drive systems. All implementations are no_std compatible and have zero
//! dependencies on platform-specific code, hardware abstractions, or system state.

pub mod differential_drive;

pub use differential_drive::DifferentialDrive;
