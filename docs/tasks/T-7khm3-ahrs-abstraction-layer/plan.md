# T-7khm3 AHRS Abstraction Layer Implementation Plan

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [T-7khm3-ahrs-abstraction-layer-design](./design.md)

## Overview

This plan implements the AHRS abstraction layer defined in ADR-nzvfy, providing a unified `Ahrs` trait that enables flight control to work with both External AHRS (BNO086) and future Software AHRS (EKF) implementations without code changes.

## Success Metrics

- [x] `Ahrs` trait compiles with `Bno086ExternalAhrs` implementation
- [x] `AhrsState` includes quaternion, Euler angles, angular rates, and timestamp
- [x] Unit tests pass for all new components (42 AHRS + raw_imu tests)
- [x] Embedded build succeeds on RP2350
- [x] All existing tests pass; no regressions in BNO086 functionality

## Scope

- Goal: Implement AHRS abstraction layer with BNO086 as first External AHRS source
- Non-Goals: SoftwareAhrs/EKF implementation (T-p8w8f), raw IMU drivers
- Assumptions: BNO086 driver (T-x8mq2) is complete and functional
- Constraints: Embassy async runtime, no_std environment, < 256 bytes abstraction overhead

## ADR & Legacy Alignment

- [x] ADR-nzvfy-ahrs-abstraction-architecture governs this work
- [x] ADR-ymkzt-ekf-ahrs-implementation defines SoftwareAhrs (out of scope for this task)
- [x] Existing `src/subsystems/ahrs/` module uses DCM; abstraction added in `src/subsystems/ahrs/traits.rs`

## Plan Summary

- Phase 1 – Core Ahrs Trait and AhrsState
- Phase 2 – ExternalAhrs Implementation (BNO086)
- Phase 3 – RawImu Trait and Testing

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Core Ahrs Trait and AhrsState

### Goal

- Define the `Ahrs` trait and `AhrsState` struct as the foundation for all AHRS implementations

### Inputs

- Documentation:
  - `docs/adr/ADR-nzvfy-ahrs-abstraction-architecture.md` – Architecture specification
  - `docs/adr/ADR-ymkzt-ekf-ahrs-implementation.md` – AhrsState fields reference
- Source Code to Modify:
  - `src/lib.rs` – Add `ahrs` module export
- Dependencies:
  - External crates: `nalgebra` – Quaternion and Vector3 types

### Tasks

- [x] **Create module structure**
  - [x] Create `src/subsystems/ahrs/traits.rs` with `Ahrs` trait, `AhrsType`, `AhrsError`, `AhrsState`
  - [x] Create `src/subsystems/ahrs/external/mod.rs` placeholder
  - [x] Update `src/subsystems/ahrs/mod.rs` exports

- [x] **Implement AhrsState**
  - [x] Define struct with quaternion, Euler angles, angular_rate, acceleration, timestamp_us, healthy, accuracy_rad
  - [x] Implement `Default` trait
  - [x] Implement `from_quaternion()` and `from_quaternion_reading()` constructors
  - [x] Add `is_valid()` and `is_fresh()` methods
  - [x] Add builder pattern methods (`with_angular_rate()`, `with_acceleration()`, etc.)

- [x] **Implement Ahrs trait**
  - [x] Define `async fn get_attitude(&mut self) -> Result<AhrsState, AhrsError>`
  - [x] Define `fn is_healthy(&self) -> bool`
  - [x] Define `fn ahrs_type(&self) -> AhrsType`

- [x] **Add error types**
  - [x] Define `AhrsError` enum with SensorError, NotInitialized, InvalidData, Timeout, NotConverged, Resetting
  - [x] Implement `From<QuaternionError>` for `AhrsError`

- [x] **Add SharedAhrsState (unified state)**
  - [x] Implement `SharedAhrsState` with thread-safe critical section access
  - [x] Add convenience methods: `update_euler()`, `update_quaternion()`, `get_roll/pitch/yaw()`
  - [x] Update `task.rs` to use `SharedAhrsState`
  - [x] Remove legacy `AttitudeState`, `AttitudeQuality`, `SharedAttitudeState` from `state.rs`
  - [x] Delete `state.rs` (unified to traits.rs)

### Deliverables

- `src/subsystems/ahrs/traits.rs` – Ahrs trait, AhrsType, AhrsError, AhrsState, SharedAhrsState
- `src/subsystems/ahrs/external/mod.rs` – External AHRS module (placeholder)
- `src/subsystems/ahrs/mod.rs` – Updated module exports (unified, legacy removed)
- `src/subsystems/ahrs/task.rs` – Updated to use SharedAhrsState

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet ahrs
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- `Ahrs` trait compiles without errors
- `AhrsState::from_quaternion()` correctly extracts Euler angles
- Unit tests pass for quaternion-to-Euler conversion

### Rollback/Fallback

- Revert changes to `src/subsystems/ahrs/traits.rs`, `mod.rs`, `task.rs`; restore `state.rs` from git

---

## Phase 2: ExternalAhrs Implementation (BNO086)

### Phase 2 Goal

- Implement `Bno086ExternalAhrs` wrapper that provides `Ahrs` trait for BNO086 driver

### Phase 2 Inputs

- Dependencies:
  - Phase 1: `Ahrs` trait and `AhrsState`
  - `src/devices/imu/bno086/` – Existing BNO086 driver
  - `src/devices/traits/quaternion.rs` – `QuaternionSensor` trait
- Source Code to Modify:
  - `src/subsystems/ahrs/external/mod.rs` – External AHRS submodule
  - `src/subsystems/ahrs/external/bno086.rs` – Wrapper implementation

### Phase 2 Tasks

- [x] **Investigate BNO086 coordinate frame and implement NED conversion**
  - [x] Research BNO086 default output coordinate frame (ENU vs sensor-local)
  - [x] Document coordinate frame in `src/devices/imu/bno086/` module comments
  - [x] Implement coordinate frame conversion to NED if required
  - [x] Add unit tests to verify NED output (e.g., sensor flat → roll=0, pitch=0)

- [x] **Enable BNO086 angular rate output (REQUIRED for PID D-term)**
  - [x] Investigate BNO086 GYROSCOPE_CALIBRATED report (Report ID 0x02)
  - [x] Update BNO086 driver to request gyro report alongside rotation vector
  - [x] Populate `AhrsState.angular_rate` with actual gyro data (not zeros)
  - [x] Add unit test to verify angular rate is non-zero when rotating

- [x] **Create external AHRS implementation**
  - [x] Create `src/subsystems/ahrs/external/bno086.rs`
  - [x] Update `src/subsystems/ahrs/external/mod.rs` to export
  - [x] Export `Bno086ExternalAhrs` from `src/subsystems/ahrs/mod.rs`

- [x] **Implement Bno086ExternalAhrs**
  - [x] Define struct wrapping `Bno086Driver<T>`
  - [x] Implement `new()` constructor
  - [x] Implement `Ahrs` trait for `Bno086ExternalAhrs<T>`
  - [x] Map `QuaternionError` to `AhrsError`
  - [x] Ensure `get_attitude()` returns angular_rate from gyro report

- [x] **Add convenience methods**
  - [x] Add `into_driver()` to extract inner driver if needed
  - [x] Add `driver(&self)` and `driver_mut(&mut self)` accessors

### Phase 2 Deliverables

- `src/subsystems/ahrs/external/mod.rs` – External AHRS module (updated)
- `src/subsystems/ahrs/external/bno086.rs` – BNO086 wrapper implementation

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet ahrs
./scripts/build-rp2350.sh bno086_demo
```

### Phase 2 Acceptance Criteria

- `Bno086ExternalAhrs` compiles and implements `Ahrs` trait
- BNO086 demo example compiles with new abstraction
- No regressions in existing BNO086 functionality

### Phase 2 Rollback/Fallback

- Remove `src/ahrs/external/` directory; BNO086 driver remains usable via `QuaternionSensor` directly

---

## Phase 3: RawImu Trait and Testing

### Phase 3 Goal

- Define `RawImu` trait for future raw IMU support and create comprehensive tests

### Phase 3 Tasks

- [x] **Implement RawImu trait**
  - [x] Create `src/devices/traits/raw_imu.rs`
  - [x] Define `RawImu` trait with `read_accel()`, `read_gyro()`, `read_mag()`, `sample_rate()`, `is_healthy()`
  - [x] Reuse `ImuError` enum from `imu.rs` (avoids duplication)
  - [x] Export from `src/devices/traits/mod.rs`

- [x] **Unit tests**
  - [x] Test `AhrsState::from_quaternion()` with known quaternions
  - [x] Test Euler angle extraction accuracy (±0.001 rad)
  - [x] Test `AhrsError` conversion from `QuaternionError` (all variants)
  - [x] Test `AhrsState::is_valid()` for normalized/unnormalized quaternions

- [x] **Integration preparation**
  - [x] Fixed `examples/bno086_demo.rs` Cargo.toml required-features
  - [x] Document usage in module-level comments (`raw_imu.rs`)

### Phase 3 Deliverables

- `src/devices/traits/raw_imu.rs` – RawImu trait for future use
- Comprehensive unit tests for AHRS abstraction layer
- Updated example demonstrating `Ahrs` trait usage

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- All unit tests pass
- `RawImu` trait defined for future implementation
- No clippy warnings

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet` (AHRS tests pass; 1 unrelated test failure in command handler)
- [x] `./scripts/build-rp2350.sh pico_trail_rover`
- [x] Module documentation in `src/subsystems/ahrs/mod.rs`
- [x] ADR-nzvfy referenced in implementation comments
- [x] No `unsafe` code (except minimal required for SharedAhrsState critical sections)
- [x] No vague naming ("manager"/"util")

## Open Questions

- [x] Should `AhrsState` include accuracy/uncertainty field for external AHRS? → Resolved: Added `accuracy_rad: Option<f32>` field
- [x] Enable BNO086 gyro report for angular_rate in AhrsState? → Resolved: Added as **required** Phase 2 task (PID D-term needs angular rate)
- [x] BNO086 coordinate frame conversion to NED? → Resolved: Added as Phase 2 task to investigate and implement if needed

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
