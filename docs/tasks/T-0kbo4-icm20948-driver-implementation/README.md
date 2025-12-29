# T-0kbo4 ICM-20948 I2C Driver Implementation

## Metadata

- Type: Task
- Status: Cancelled

## Cancellation Note

**Reason**: ICM-20948 chip was counterfeit - magnetometer (AK09916) not functional.

The purchased ICM-20948 breakout board turned out to be counterfeit. While the gyroscope and accelerometer work, the integrated AK09916 magnetometer does not respond correctly, making the 9-axis functionality unusable.

**Source code retained**: The driver implementation in `src/devices/imu/icm20948/` is kept as-is for potential future use with genuine ICM-20948 hardware.

## Links

- Related Analyses:
  - [AN-t47be-mpu9250-imu-and-ekf-integration](../../analysis/AN-t47be-mpu9250-imu-and-ekf-integration.md)
- Related Requirements:
  - [FR-slm3x-icm20948-i2c-driver](../../requirements/FR-slm3x-icm20948-i2c-driver.md)
  - [FR-z1fdo-imu-sensor-trait](../../requirements/FR-z1fdo-imu-sensor-trait.md)
  - [FR-soukr-imu-calibration-interface](../../requirements/FR-soukr-imu-calibration-interface.md)
  - [NFR-ulsja-imu-i2c-read-latency](../../requirements/NFR-ulsja-imu-i2c-read-latency.md)
  - [NFR-wkrr5-imu-read-reliability](../../requirements/NFR-wkrr5-imu-read-reliability.md)
- Related ADRs:
  - [ADR-t5cq4-mpu9250-i2c-driver-architecture](../../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)
- Associated Design Document:
  - [T-0kbo4-icm20948-driver-implementation-design](./design.md)
- Associated Plan Document:
  - [T-0kbo4-icm20948-driver-implementation-plan](./plan.md)
- Supersedes Task:
  - [T-kx79g-mpu9250-driver-implementation](../T-kx79g-mpu9250-driver-implementation/README.md) (Cancelled)

## Summary

Implement an ICM-20948 9-axis IMU driver using I2C communication, providing the `ImuSensor` trait interface for the EKF AHRS subsystem. The driver reads gyroscope, accelerometer, and magnetometer data at 400Hz, applies calibration corrections, and exposes sensor health status. ICM-20948 is the primary sensor (TDK InvenSense successor to MPU-9250).

## Scope

- In scope:
  - ICM-20948 register definitions and I2C communication
  - Register bank switching (4-bank architecture)
  - AK09916 magnetometer integration via I2C bypass mode
  - `ImuSensor` trait implementation (reuse existing trait definition)
  - Unit tests with mock I2C for host testing
  - Embassy task for 400Hz sampling
- Out of scope:
  - EKF algorithm implementation (separate task T-p8w8f)
  - SPI interface support (I2C only per ADR decision)
  - ICM-20948 DMP (Digital Motion Processor) usage
  - Temperature calibration (future enhancement)
  - MPU-9250 driver (existing code in `src/devices/imu/mpu9250/`, retained)

## Success Metrics

- **Sensor Initialization**: WHO_AM_I verification passes (ICM-20948: 0xEA, AK09916: 0x09)
- **Sampling Rate**: Sustained 400Hz ± 5Hz over 10-second window
- **I2C Latency**: Full 9-axis read completes in < 1.5ms (95th percentile)
- **Reliability**: < 1 read error per 1000 samples under normal operation
- **Unit Conversion**: Gyro (rad/s), Accel (m/s²), Mag (µT) with correct scaling
- **Calibration**: Offset/scale corrections applied correctly to raw data
- **Platform Support**: Works on both RP2040 and RP2350 without modification
- **Code Quality**: All tests pass, no clippy warnings, embedded build succeeds

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../../templates/README.md#task-template-taskmd) in the templates README.
