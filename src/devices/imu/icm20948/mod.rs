//! ICM-20948 9-Axis IMU Driver
//!
//! I2C driver for the TDK InvenSense ICM-20948 IMU with integrated AK09916 magnetometer.
//!
//! ## Requirements
//!
//! - FR-slm3x: ICM-20948 I2C Driver
//! - ADR-t5cq4: MPU-9250 I2C Driver Architecture (applies to ICM-20948)
//!
//! ## Features
//!
//! - 3-axis gyroscope: ±250, ±500, ±1000, ±2000 °/s
//! - 3-axis accelerometer: ±2, ±4, ±8, ±16 g
//! - 3-axis magnetometer (AK09916): ±4912 µT
//! - 4-bank register architecture with bank switching
//! - I2C @ 400kHz
//! - 400Hz sampling rate
//!
//! ## Usage
//!
//! ```ignore
//! use pico_trail::devices::imu::Icm20948Driver;
//! use pico_trail::devices::traits::ImuSensor;
//!
//! let mut driver = Icm20948Driver::new_initialized(i2c, Icm20948Config::default()).await?;
//! let reading = driver.read_all().await?;
//! ```

mod config;
mod driver;
mod registers;

pub use config::{
    AccelDlpfConfig, AccelRange, GyroDlpfConfig, GyroRange, Icm20948Config, MagMode, RegisterBank,
};
pub use driver::Icm20948Driver;
