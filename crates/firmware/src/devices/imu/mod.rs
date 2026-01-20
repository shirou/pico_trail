//! IMU Drivers
//!
//! This module contains IMU sensor drivers for attitude estimation.
//!
//! ## Available Drivers
//!
//! ### Raw IMU Sensors (implement `ImuSensor` trait)
//!
//! - `mock`: Mock IMU for testing (always available)
//! - `Mpu9250Driver`: MPU-9250 9-axis IMU driver
//! - `Icm20948Driver`: ICM-20948 9-axis IMU driver
//!
//! ### Quaternion Sensors (implement `QuaternionSensor` trait)
//!
//! - `Bno086Driver`: BNO086 with on-chip sensor fusion
//!
//! Hardware drivers use `embedded_hal_async::i2c::I2c` trait and are platform-agnostic.
//!
//! ## Usage
//!
//! ### Raw IMU (requires external EKF)
//!
//! ```ignore
//! use pico_trail::devices::imu::{Mpu9250Driver, Mpu9250Config};
//! use pico_trail::devices::traits::ImuSensor;
//!
//! let mut driver = Mpu9250Driver::new(i2c, Mpu9250Config::default()).await?;
//! let reading = driver.read_all().await?;
//! ```
//!
//! ### Quaternion Sensor (on-chip fusion)
//!
//! ```ignore
//! use pico_trail::devices::imu::bno086::{Bno086Driver, Bno086Config};
//! use pico_trail::communication::shtp::ShtpI2c;
//! use pico_trail::devices::traits::QuaternionSensor;
//!
//! let transport = ShtpI2c::new(i2c, 0x4A);
//! let mut driver = Bno086Driver::new(transport, Bno086Config::default());
//! driver.init().await?;
//! let reading = driver.read_quaternion().await?;
//! ```

pub mod bno086;
pub mod icm20948;
pub mod mock;
pub mod mpu9250;

pub use bno086::{Bno086Config, Bno086Driver};
pub use icm20948::Icm20948Driver;
pub use mock::MockImu;
pub use mpu9250::Mpu9250Driver;
