//! IMU Drivers
//!
//! This module contains IMU sensor drivers implementing the `ImuSensor` trait.
//!
//! ## Available Drivers
//!
//! - `mock`: Mock IMU for testing (always available)
//! - `Mpu9250Driver`: MPU-9250 9-axis IMU driver (requires `embassy` feature)
//! - `Icm20948Driver`: ICM-20948 9-axis IMU driver (requires `embassy` feature)
//!
//! Hardware drivers use `embedded_hal_async::i2c::I2c` trait and are platform-agnostic.
//! They require the `embassy` feature for async I2C and time support.
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::devices::imu::{Mpu9250Driver, Mpu9250Config};
//! use pico_trail::devices::traits::ImuSensor;
//!
//! let mut driver = Mpu9250Driver::new(i2c, Mpu9250Config::default()).await?;
//! let reading = driver.read_all().await?;
//! ```

#[cfg(feature = "embassy")]
pub mod icm20948;
pub mod mock;
#[cfg(feature = "embassy")]
pub mod mpu9250;

#[cfg(feature = "embassy")]
pub use icm20948::Icm20948Driver;
pub use mock::MockImu;
#[cfg(feature = "embassy")]
pub use mpu9250::Mpu9250Driver;
