//! RP2350 Device Drivers
//!
//! Platform-specific device driver implementations for RP2350.
//!
//! Note: IMU drivers (MPU-9250, ICM-20948) have been moved to `crate::devices::imu`
//! as they are platform-agnostic and use `embedded_hal_async::i2c::I2c` trait.
