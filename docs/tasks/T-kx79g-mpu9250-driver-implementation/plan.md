# T-kx79g MPU-9250 I2C Driver Implementation

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [T-kx79g-mpu9250-driver-implementation-design](./design.md)

## Overview

Implement an MPU-9250 9-axis IMU driver with I2C communication, providing the `ImuSensor` trait interface for EKF AHRS consumption. The driver supports 400Hz sampling, calibration integration, and works on both RP2040 and RP2350 platforms.

## Success Metrics

- [ ] MPU-9250 WHO_AM_I verification passes (0x71)
- [ ] AK8963 magnetometer access via bypass mode (WHO_AM_I: 0x48)
- [ ] Full 9-axis read latency < 1.5ms (95th percentile)
- [ ] Sustained 400Hz sampling with < 1 error per 1000 samples
- [ ] All unit tests pass; embedded build succeeds on RP2350

## Scope

- Goal: Complete MPU-9250 driver implementing `ImuSensor` trait with calibration support
- Non-Goals: EKF implementation, SPI support, DMP usage, temperature calibration
- Assumptions: I2C bus available at 400kHz, Embassy async runtime, parameter system exists
- Constraints: no_std, no heap allocation, Embassy async, I2C @ 400kHz

## ADR & Legacy Alignment

- [ ] Confirm ADR-t5cq4-mpu9250-i2c-driver-architecture is referenced and followed
- [ ] Mark T-qwvco-bmi088-imu-driver-implementation as superseded/deprecated

## Plan Summary

- Phase 1 - Core trait definitions and module structure
- Phase 2 - MPU-9250 driver implementation (gyro + accel)
- Phase 3 - AK8963 magnetometer integration
- Phase 4 - Calibration and Embassy task integration
- Phase 5 - Testing and validation

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Core Trait Definitions and Module Structure

### Goal

- Define `ImuSensor` trait, data structures, and establish module hierarchy

### Inputs

- Documentation:
  - [FR-z1fdo-imu-sensor-trait](../../requirements/FR-z1fdo-imu-sensor-trait.md)
  - [ADR-t5cq4-mpu9250-i2c-driver-architecture](../../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)
- Source Code to Modify:
  - `src/devices/mod.rs` - Add imu module export
- Dependencies:
  - External crates: `nalgebra` (vectors), `embedded-hal-async` (I2C trait)

### Tasks

- [x] **Module structure**
  - [x] Create `src/devices/traits/mod.rs` with imu export
  - [x] Create `src/devices/traits/imu.rs` with trait and types
  - [x] Create `src/devices/imu/mod.rs` with submodule exports
  - [x] Update `src/devices/mod.rs` to export new modules
- [x] **ImuSensor trait**
  - [x] Define `ImuSensor` trait with async methods
  - [x] Define `ImuReading` struct with all fields
  - [x] Define `ImuCalibration` struct with Default impl
  - [x] Define `ImuError` enum with all variants
- [x] **Mock implementation**
  - [x] Create `src/devices/imu/mock.rs`
  - [x] Implement `MockImu` with preset readings
  - [x] Implement `ImuSensor` for `MockImu`

### Deliverables

- `src/devices/traits/imu.rs` with trait and data structures
- `src/devices/imu/mock.rs` with mock implementation
- Module exports configured in `mod.rs` files

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- `ImuSensor` trait compiles with async methods
- `MockImu` implements trait and can be instantiated in tests
- Host tests pass; embedded build succeeds

### Rollback/Fallback

- Revert new files; no existing functionality affected

---

## Phase 2: MPU-9250 Driver Implementation (Gyro + Accel)

### Phase 2 Goal

- Implement MPU-9250 I2C driver for gyroscope and accelerometer reading

### Phase 2 Inputs

- Dependencies:
  - Phase 1: `ImuSensor` trait, `ImuReading`, `ImuError`
  - External crates: `embassy-rp` (I2C), `embedded-hal-async`
- Source Code to Modify:
  - `src/devices/imu/mpu9250/` - New driver module

### Phase 2 Tasks

- [x] **Register definitions**
  - [x] Create `src/devices/imu/mpu9250/registers.rs`
  - [x] Define MPU-9250 register addresses (WHO_AM_I, PWR_MGMT, CONFIG, etc.)
  - [x] Define configuration constants (ranges, DLPF values)
- [x] **Configuration structs**
  - [x] Create `src/devices/imu/mpu9250/config.rs`
  - [x] Define `Mpu9250Config` with gyro/accel range, DLPF settings
  - [x] Define `GyroRange`, `AccelRange`, `DlpfConfig` enums
  - [x] Implement Default for `Mpu9250Config`
- [x] **Core driver**
  - [x] Create `src/devices/imu/mpu9250/driver.rs`
  - [x] Implement `Mpu9250Driver` struct with I2C handle
  - [x] Implement `new()` with WHO_AM_I verification
  - [x] Implement device reset and configuration sequence
  - [x] Implement gyro/accel read (14 bytes from ACCEL_XOUT_H)
  - [x] Implement unit conversion (raw to rad/s, m/s²)
- [x] **Module exports**
  - [x] Create `src/devices/imu/mpu9250/mod.rs`
  - [x] Re-export driver and config types
  - [x] Update `src/devices/imu/mod.rs`

### Phase 2 Deliverables

- Complete MPU-9250 driver for gyro/accel reading
- Unit conversion functions with correct scaling
- Initialization sequence with error handling

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::imu
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- Driver compiles on both host and embedded target
- WHO_AM_I check implemented with error handling
- Gyro/accel read returns calibration-ready values

### Phase 2 Rollback/Fallback

- Driver incomplete but does not break existing code

---

## Phase 3: AK8963 Magnetometer Integration

### Phase 3 Goal

- Enable magnetometer reading via I2C bypass mode

### Phase 3 Inputs

- Dependencies:
  - Phase 2: MPU-9250 driver with I2C access
- Source Code to Modify:
  - `src/devices/imu/mpu9250/registers.rs` - AK8963 registers
  - `src/devices/imu/mpu9250/driver.rs` - Mag read methods

### Phase 3 Tasks

- [x] **AK8963 registers**
  - [x] Add AK8963 register definitions to registers.rs
  - [x] Define magnetometer configuration constants
- [x] **Bypass mode setup**
  - [x] Enable I2C bypass in INT_PIN_CFG register
  - [x] Verify AK8963 WHO_AM_I (0x48)
  - [x] Configure AK8963 for continuous 100Hz mode
- [x] **Magnetometer reading**
  - [x] Implement mag read (7 bytes from HXL)
  - [x] Check DRDY status and overflow
  - [x] Implement unit conversion (raw to µT)
- [x] **Combined read**
  - [x] Implement `read_all()` combining gyro+accel+mag
  - [x] Timestamp readings with microsecond precision

### Phase 3 Deliverables

- AK8963 magnetometer accessible and reading
- Combined 9-axis reading with correct units
- Proper DRDY handling for magnetometer

### Phase 3 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::imu
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- Magnetometer WHO_AM_I verification passes
- Full 9-axis read completes successfully
- Unit conversions produce reasonable values

### Phase 3 Rollback/Fallback

- Magnetometer can be disabled; gyro/accel still functional

---

## Phase 4: Calibration and Embassy Task Integration

### Phase 4 Goal

- Integrate calibration system and create Embassy task for 400Hz sampling

### Phase 4 Inputs

- Dependencies:
  - Phase 3: Complete 9-axis reading
  - Existing: Parameter system for calibration storage
- Source Code to Modify:
  - `src/devices/imu/mpu9250/driver.rs` - Calibration application
  - `src/subsystems/ahrs/mod.rs` - Embassy task

### Phase 4 Tasks

- [x] **Calibration application**
  - [x] Implement `set_calibration()` method
  - [x] Apply gyro bias in `read_gyro()`
  - [x] Apply accel offset/scale in `read_accel()`
  - [x] Apply mag hard/soft iron in `read_mag()`
  - [x] Apply all calibrations in `read_all()`
- [x] **Health tracking**
  - [x] Implement `is_healthy()` method
  - [x] Track consecutive error count
  - [ ] Detect stuck sensor values (deferred - future enhancement)
  - [x] Log warnings for degraded health
- [x] **ImuSensor trait implementation**
  - [x] Implement `ImuSensor` for `Mpu9250Driver`
  - [x] Verify all trait methods compile
- [x] **Embassy task**
  - [x] Create `run_imu_task` function in AHRS module
  - [x] Set up 400Hz ticker (2.5ms period)
  - [x] Read IMU data each cycle
  - [x] Pass data to callback (placeholder for T-p8w8f EKF)
  - [x] Log latency statistics periodically

### Phase 4 Deliverables

- Full `ImuSensor` implementation for MPU-9250
- Calibration application for all 9 axes
- Embassy task ready for 400Hz operation

### Phase 4 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::imu
cargo test --lib --quiet subsystems::ahrs
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 4 Acceptance Criteria

- `ImuSensor` trait fully implemented
- Calibration correctly modifies output values
- Embassy task compiles and starts without error

### Phase 4 Rollback/Fallback

- Task can be disabled; driver remains testable

---

## Phase 5: Testing and Validation

### Phase 5 Goal

- Comprehensive unit tests and validation of all driver functionality

### Phase 5 Tasks

- [ ] **Unit tests**
  - [ ] Test `Mpu9250Config` defaults and ranges
  - [ ] Test unit conversion math (gyro/accel/mag)
  - [ ] Test calibration application
  - [ ] Test `ImuCalibration` default values
  - [ ] Test `MockImu` reading sequence
- [ ] **Integration preparation**
  - [ ] Create example for hardware testing (if needed)
  - [ ] Document test procedure for hardware validation
- [ ] **Documentation**
  - [ ] Add module documentation with examples
  - [ ] Update `docs/architecture.md` with IMU module

### Phase 5 Deliverables

- Comprehensive unit test suite
- Documentation for IMU driver module
- Hardware test procedure documented

### Phase 5 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
bun scripts/trace-status.ts --check
bun format && bun lint
```

### Phase 5 Acceptance Criteria

- All unit tests pass
- Documentation complete
- Traceability matrix updated

---

## Definition of Done

- [ ] `cargo check`
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] Architecture documentation updated
- [ ] ADR referenced and followed
- [ ] Error messages actionable and in English
- [ ] No `unsafe` and no vague naming

## Open Questions

- [ ] Should we implement I2C DMA for lower CPU overhead? → Method: Benchmark CPU usage in Phase 4, add DMA if >20%
- [ ] Is bypass mode or master mode better for AK8963? → Decision: Start with bypass (simpler), evaluate in Phase 3

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
