//! IMU Drivers
//!
//! This module contains IMU sensor drivers implementing the `ImuSensor` trait.
//!
//! ## Available Drivers
//!
//! - `mock`: Mock IMU for testing (always available)
//! - `mpu9250`: MPU-9250 9-axis IMU driver (requires `pico2_w` feature)
//! - `icm20948`: ICM-20948 9-axis IMU driver (requires `pico2_w` feature)
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

#[cfg(feature = "pico2_w")]
pub mod icm20948;

#[cfg(feature = "pico2_w")]
pub mod mpu9250;

pub use mock::MockImu;

#[cfg(feature = "pico2_w")]
pub use icm20948::Icm20948Driver;

#[cfg(feature = "pico2_w")]
pub use mpu9250::Mpu9250Driver;
