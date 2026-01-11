# T-7khm3 AHRS Abstraction Layer Implementation

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-srhcj-bno086-imu-integration](../../analysis/AN-srhcj-bno086-imu-integration.md)
  - [AN-aruub-imu-driver-hal-abstraction](../../analysis/AN-aruub-imu-driver-hal-abstraction.md)
- Related Requirements:
  - [FR-eyuh8-ahrs-attitude-estimation](../../requirements/FR-eyuh8-ahrs-attitude-estimation.md)
  - [FR-z1fdo-imu-sensor-trait](../../requirements/FR-z1fdo-imu-sensor-trait.md)
  - [FR-svawa-imu-hal-abstraction](../../requirements/FR-svawa-imu-hal-abstraction.md)
  - [NFR-3wlo1-imu-sampling-rate](../../requirements/NFR-3wlo1-imu-sampling-rate.md)
- Related ADRs:
  - [ADR-nzvfy-ahrs-abstraction-architecture](../../adr/ADR-nzvfy-ahrs-abstraction-architecture.md)
  - [ADR-ymkzt-ekf-ahrs-implementation](../../adr/ADR-ymkzt-ekf-ahrs-implementation.md)
- Associated Design Document:
  - [T-7khm3-ahrs-abstraction-layer-design](./design.md)
- Associated Plan Document:
  - [T-7khm3-ahrs-abstraction-layer-plan](./plan.md)
- Prerequisite Tasks:
  - [T-x8mq2-bno086-driver-implementation](../T-x8mq2-bno086-driver-implementation/README.md)
- Dependent Tasks:
  - [T-p8w8f-ekf-ahrs-implementation](../T-p8w8f-ekf-ahrs-implementation/README.md)

## Summary

Implement a unified AHRS abstraction layer that provides a common `Ahrs` trait for flight control, supporting both External AHRS (BNO086 on-chip fusion) and Software AHRS (EKF for raw IMUs). This enables flight control code to remain sensor-agnostic while optimizing resource usage based on sensor capabilities.

## Scope

- In scope:
  - `Ahrs` trait definition with `get_attitude()`, `is_healthy()`, `ahrs_type()` methods
  - `AhrsState` struct with quaternion, angular rates, acceleration, and timestamp
  - `ExternalAhrs` implementation wrapping BNO086's `QuaternionSensor`
  - `RawImu` trait for future raw IMU sensor support
  - Module structure under `src/ahrs/` following ADR-nzvfy architecture
  - Unit tests for trait implementations
- Out of scope:
  - `SoftwareAhrs` EKF implementation (handled by T-p8w8f)
  - ICM-42688 raw IMU driver (future task)
  - GPS velocity fusion for heading
  - MAVLink attitude message integration (existing infrastructure)

## Success Metrics

- **Abstraction**: Flight control code compiles and runs with `Ahrs` trait without knowing sensor type
- **BNO086 Integration**: `Bno086ExternalAhrs` produces valid `AhrsState` from existing driver
- **Zero Overhead**: BNO086 users incur no CPU cost from unused EKF code paths
- **Extensibility**: Adding new AHRS source requires only implementing `Ahrs` trait
- **Code Quality**: All tests pass, no clippy warnings, embedded build succeeds

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../../templates/README.md#task-template-taskmd) in the templates README.
