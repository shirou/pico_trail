# T-00026 IMU HAL Refactor

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-00030-imu-driver-hal-abstraction](../../analysis/AN-00030-imu-driver-hal-abstraction.md)
- Related Requirements:
  - [FR-00107-imu-hal-abstraction](../../requirements/FR-00107-imu-hal-abstraction.md)
  - [NFR-00078-imu-driver-location](../../requirements/NFR-00078-imu-driver-location.md)
- Related ADRs:
  - [ADR-00028-imu-embedded-hal-async](../../adr/ADR-00028-imu-embedded-hal-async.md)
- Associated Design Document:
  - [T-00026-imu-hal-refactor-design](design.md)
- Associated Plan Document:
  - [T-00026-imu-hal-refactor-plan](plan.md)

## Summary

Refactor IMU drivers (MPU9250, ICM20948) to use `embedded_hal_async::i2c::I2c` trait instead of `embassy_rp` concrete types, and move drivers from `platform/rp2350/devices/imu/` to `devices/imu/`.

## Scope

- In scope:
  - Refactor `Mpu9250Driver` to use generic I2C trait
  - Refactor `Icm20948Driver` to use generic I2C trait
  - Move driver files from `platform/rp2350/devices/imu/` to `devices/imu/`
  - Update all imports and re-exports
  - Ensure embedded build and host tests pass
- Out of scope:
  - Adding new IMU chip support
  - SPI interface support
  - Performance optimizations

## Success Metrics

- Zero `embassy_rp` imports in IMU driver modules
- Zero `pico2_w` feature gates in IMU driver modules
- All existing unit tests pass
- Embedded build succeeds
- Drivers compile with mock I2C for host tests
