# T-00023 MPU-9250 I2C Driver Implementation

## Metadata

- Type: Task
- Status: Cancelled

## Change History

- **2025-01-XX**: Status changed to Cancelled. MPU-9250 hardware unavailable; ICM-20948 selected as primary sensor. See [T-00024-icm20948-driver-implementation](../T-00024-icm20948-driver-implementation/README.md) for ICM-20948 implementation. Existing MPU-9250 code in `src/devices/imu/mpu9250/` retained for users with existing hardware.

## Links

- Related Analyses:
  - [AN-00027-mpu9250-imu-and-ekf-integration](../../analysis/AN-00027-mpu9250-imu-and-ekf-integration.md)
- Related Requirements:
  - [FR-00103-mpu9250-i2c-driver](../../requirements/FR-00103-mpu9250-i2c-driver.md)
  - [FR-00101-imu-sensor-trait](../../requirements/FR-00101-imu-sensor-trait.md)
  - [FR-00100-imu-calibration-interface](../../requirements/FR-00100-imu-calibration-interface.md)
  - [NFR-00075-imu-i2c-read-latency](../../requirements/NFR-00075-imu-i2c-read-latency.md)
  - [NFR-00076-imu-read-reliability](../../requirements/NFR-00076-imu-read-reliability.md)
- Related ADRs:
  - [ADR-00026-mpu9250-i2c-driver-architecture](../../adr/ADR-00026-mpu9250-i2c-driver-architecture.md)
- Associated Design Document:
  - [T-00023-mpu9250-driver-implementation-design](./design.md)
- Associated Plan Document:
  - [T-00023-mpu9250-driver-implementation-plan](./plan.md)

## Summary

Implement an MPU-9250 9-axis IMU driver using I2C communication, providing the `ImuSensor` trait interface for the EKF AHRS subsystem. The driver reads gyroscope, accelerometer, and magnetometer data at 400Hz, applies calibration corrections, and exposes sensor health status.

## Scope

**Note: This task is CANCELLED. See ICM-20948 task for active implementation.**

- In scope (was):
  - MPU-9250 register definitions and I2C communication
  - AK8963 magnetometer integration via I2C bypass mode
  - `ImuSensor` trait definition and implementation
  - `ImuReading`, `ImuCalibration`, `ImuError` data structures
  - Calibration data loading and application
  - Unit tests with mock I2C for host testing
  - Embassy task for 400Hz sampling
- Out of scope:
  - EKF algorithm implementation (separate task T-00022)
  - SPI interface support (I2C only per ADR decision)
  - MPU-9250 DMP (Digital Motion Processor) usage
  - Temperature calibration (future enhancement)
  - ~~ICM-20948 support~~ → Now primary sensor, see separate task

## Success Metrics

- **Sensor Initialization**: WHO_AM_I verification passes (MPU-9250: 0x71, AK8963: 0x48)
- **Sampling Rate**: Sustained 400Hz ± 5Hz over 10-second window
- **I2C Latency**: Full 9-axis read completes in < 1.5ms (95th percentile)
- **Reliability**: < 1 read error per 1000 samples under normal operation
- **Unit Conversion**: Gyro (rad/s), Accel (m/s²), Mag (µT) with correct scaling
- **Calibration**: Offset/scale corrections applied correctly to raw data
- **Platform Support**: Works on both RP2040 and RP2350 without modification
- **Code Quality**: All tests pass, no clippy warnings, embedded build succeeds
