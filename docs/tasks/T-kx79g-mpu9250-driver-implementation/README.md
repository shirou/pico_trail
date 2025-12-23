# T-kx79g MPU-9250 I2C Driver Implementation

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-t47be-mpu9250-imu-and-ekf-integration](../../analysis/AN-t47be-mpu9250-imu-and-ekf-integration.md)
- Related Requirements:
  - [FR-oqxl8-mpu9250-i2c-driver](../../requirements/FR-oqxl8-mpu9250-i2c-driver.md)
  - [FR-z1fdo-imu-sensor-trait](../../requirements/FR-z1fdo-imu-sensor-trait.md)
  - [FR-soukr-imu-calibration-interface](../../requirements/FR-soukr-imu-calibration-interface.md)
  - [NFR-ulsja-imu-i2c-read-latency](../../requirements/NFR-ulsja-imu-i2c-read-latency.md)
  - [NFR-wkrr5-imu-read-reliability](../../requirements/NFR-wkrr5-imu-read-reliability.md)
- Related ADRs:
  - [ADR-t5cq4-mpu9250-i2c-driver-architecture](../../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)
- Associated Design Document:
  - [T-kx79g-mpu9250-driver-implementation-design](./design.md)
- Associated Plan Document:
  - [T-kx79g-mpu9250-driver-implementation-plan](./plan.md)

## Summary

Implement an MPU-9250 9-axis IMU driver using I2C communication, providing the `ImuSensor` trait interface for the EKF AHRS subsystem. The driver reads gyroscope, accelerometer, and magnetometer data at 400Hz, applies calibration corrections, and exposes sensor health status.

## Scope

- In scope:
  - MPU-9250 register definitions and I2C communication
  - AK8963 magnetometer integration via I2C bypass mode
  - `ImuSensor` trait definition and implementation
  - `ImuReading`, `ImuCalibration`, `ImuError` data structures
  - Calibration data loading and application
  - Unit tests with mock I2C for host testing
  - Embassy task for 400Hz sampling
- Out of scope:
  - EKF algorithm implementation (separate task T-p8w8f)
  - SPI interface support (I2C only per ADR decision)
  - MPU-9250 DMP (Digital Motion Processor) usage
  - Temperature calibration (future enhancement)
  - ICM-20948 support (future sensor upgrade)

## Success Metrics

- **Sensor Initialization**: WHO_AM_I verification passes (MPU-9250: 0x71, AK8963: 0x48)
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
