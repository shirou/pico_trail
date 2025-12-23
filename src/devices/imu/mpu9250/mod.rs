//! MPU-9250 9-Axis IMU Driver
//!
//! I2C driver for the InvenSense MPU-9250 IMU with integrated AK8963 magnetometer.
//!
//! ## Requirements
//!
//! - FR-oqxl8: MPU-9250 I2C Driver
//! - ADR-t5cq4: MPU-9250 I2C Driver Architecture
//!
//! ## Features
//!
//! - 3-axis gyroscope: ±250, ±500, ±1000, ±2000 °/s
//! - 3-axis accelerometer: ±2, ±4, ±8, ±16 g
//! - 3-axis magnetometer (AK8963): ±4912 µT
//! - I2C @ 400kHz
//! - 400Hz sampling rate
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::devices::imu::Mpu9250Driver;
//! use pico_trail::devices::traits::ImuSensor;
//!
//! let mut driver = Mpu9250Driver::new(i2c, Mpu9250Config::default()).await?;
//! let reading = driver.read_all().await?;
//! ```

mod config;
mod driver;
mod registers;

pub use config::{AccelRange, DlpfConfig, GyroRange, MagMode, Mpu9250Config};
pub use driver::Mpu9250Driver;
