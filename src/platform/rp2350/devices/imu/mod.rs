//! RP2350 IMU Drivers
//!
//! Platform-specific IMU driver implementations using embassy-rp I2C.
//!
//! These drivers implement the `ImuSensor` trait from `crate::devices::traits`.

pub mod icm20948;
pub mod mpu9250;

pub use icm20948::Icm20948Driver;
pub use mpu9250::Mpu9250Driver;
