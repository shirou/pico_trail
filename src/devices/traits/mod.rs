//! Device traits
//!
//! This module contains hardware-independent trait definitions for device drivers.
//! These traits enable:
//! - Unit testing with mock implementations
//! - Sensor independence for higher-level subsystems
//! - Future hardware upgrades without algorithm changes

pub mod imu;
pub mod quaternion;

pub use imu::{ImuCalibration, ImuError, ImuReading, ImuSensor};
pub use quaternion::{QuaternionError, QuaternionReading, QuaternionSensor};
