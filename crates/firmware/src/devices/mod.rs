//! Device drivers
//!
//! This module contains device drivers that use platform abstraction traits,
//! demonstrating how to write hardware-independent drivers.
//!
//! ## Modules
//!
//! - `gps`: GPS receiver driver
//! - `gps_operation`: GPS operation management
//! - `imu`: IMU sensor drivers (MPU-9250, mock)
//! - `traits`: Device trait definitions (ImuSensor, etc.)

pub mod gps;
pub mod gps_operation;
pub mod imu;
pub mod traits;
