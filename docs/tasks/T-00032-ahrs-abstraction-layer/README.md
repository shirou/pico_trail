# T-00032 AHRS Abstraction Layer Implementation

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-00028-bno086-imu-integration](../../analysis/AN-00028-bno086-imu-integration.md)
  - [AN-00030-imu-driver-hal-abstraction](../../analysis/AN-00030-imu-driver-hal-abstraction.md)
- Related Requirements:
  - [FR-00001-ahrs-attitude-estimation](../../requirements/FR-00001-ahrs-attitude-estimation.md)
  - [FR-00101-imu-sensor-trait](../../requirements/FR-00101-imu-sensor-trait.md)
  - [FR-00107-imu-hal-abstraction](../../requirements/FR-00107-imu-hal-abstraction.md)
  - [NFR-00002-imu-sampling-rate](../../requirements/NFR-00002-imu-sampling-rate.md)
- Related ADRs:
  - [ADR-00032-ahrs-abstraction-architecture](../../adr/ADR-00032-ahrs-abstraction-architecture.md)
  - [ADR-00025-ekf-ahrs-implementation](../../adr/ADR-00025-ekf-ahrs-implementation.md)
- Associated Design Document:
  - [T-00032-ahrs-abstraction-layer-design](./design.md)
- Associated Plan Document:
  - [T-00032-ahrs-abstraction-layer-plan](./plan.md)
- Prerequisite Tasks:
  - [T-00031-bno086-driver-implementation](../T-00031-bno086-driver-implementation/README.md)
- Dependent Tasks:
  - [T-00022-ekf-ahrs-implementation](../T-00022-ekf-ahrs-implementation/README.md)

## Summary

Implement a unified AHRS abstraction layer that provides a common `Ahrs` trait for flight control, supporting both External AHRS (BNO086 on-chip fusion) and Software AHRS (EKF for raw IMUs). This enables flight control code to remain sensor-agnostic while optimizing resource usage based on sensor capabilities.

## Scope

- In scope:
  - `Ahrs` trait definition with `get_attitude()`, `is_healthy()`, `ahrs_type()` methods
  - `AhrsState` struct with quaternion, angular rates, acceleration, and timestamp
  - `ExternalAhrs` implementation wrapping BNO086's `QuaternionSensor`
  - `RawImu` trait for future raw IMU sensor support
  - Module structure under `src/ahrs/` following ADR-00032 architecture
  - Unit tests for trait implementations
- Out of scope:
  - `SoftwareAhrs` EKF implementation (handled by T-00022)
  - ICM-42688 raw IMU driver (future task)
  - GPS velocity fusion for heading
  - MAVLink attitude message integration (existing infrastructure)

## Success Metrics

- **Abstraction**: Flight control code compiles and runs with `Ahrs` trait without knowing sensor type
- **BNO086 Integration**: `Bno086ExternalAhrs` produces valid `AhrsState` from existing driver
- **Zero Overhead**: BNO086 users incur no CPU cost from unused EKF code paths
- **Extensibility**: Adding new AHRS source requires only implementing `Ahrs` trait
- **Code Quality**: All tests pass, no clippy warnings, embedded build succeeds
