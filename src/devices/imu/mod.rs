//! IMU Drivers
//!
//! This module contains IMU sensor drivers implementing the `ImuSensor` trait.
//!
//! ## Available Drivers
//!
//! - `mock`: Mock IMU for testing (always available)
//! - `Mpu9250Driver`: MPU-9250 9-axis IMU driver (requires `pico2_w` feature, in platform module)
//! - `Icm20948Driver`: ICM-20948 9-axis IMU driver (requires `pico2_w` feature, in platform module)
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::devices::imu::MockImu;
//! use pico_trail::devices::traits::ImuSensor;
//!
//! let mut imu = MockImu::with_default_reading();
//! let reading = imu.read_all().await?;
//! ```

pub mod mock;

pub use mock::MockImu;

// Platform-specific IMU drivers are in crate::platform::rp2350::devices::imu
// Re-export modules and types for convenience
#[cfg(feature = "pico2_w")]
pub use crate::platform::rp2350::devices::imu::{icm20948, mpu9250, Icm20948Driver, Mpu9250Driver};
