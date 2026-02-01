# T-00009 BMI088 IMU Driver Implementation

## Metadata

- Type: Implementation Plan
- Status: Deprecated
  <!-- Deprecated: BMI088 replaced by MPU-9250. See AN-00027-mpu9250-imu-and-ekf-integration. -->

## Links

- Associated Design Document:
  - [design.md](./design.md)

## Overview

Implement BMI088-based IMU driver with SPI communication, interrupt-driven sampling at 400Hz, and trait-based abstraction for AHRS integration. The implementation follows a three-phase approach: driver foundation, sensor reading, and AHRS integration.

## Success Metrics

- [ ] Sampling rate: 400Hz ± 5Hz sustained over 10-second window
- [ ] Jitter: Standard deviation < 1ms measured via timestamps
- [ ] Latency: Sensor read completes in < 0.5ms
- [ ] CPU overhead: < 10% on RP2040 @ 133MHz
- [ ] Reliability: < 1 read error per 1000 samples
- [ ] AHRS integration: ±2° roll/pitch accuracy achieved
- [ ] All existing tests pass; no regressions in AHRS subsystem

## Scope

- Goal: Provide 400Hz gyro/accel data to AHRS with <1ms jitter for accurate attitude estimation
- Non-Goals:
  - Magnetometer integration (separate task)
  - Hardware FIFO buffering (deferred to Phase 2)
  - DMA optimization (deferred to Phase 2)
  - Temperature compensation (evaluate in Phase 2)
- Assumptions:
  - Platform abstraction layer provides `SpiInterface` trait (ADR-00003)
  - AHRS DCM algorithm is implemented and ready (T-00005)
  - Embassy async runtime is available
  - Calibration data will be stored in parameter system (future work)
- Constraints:
  - No heap allocation (embedded environment)
  - Real-time 400Hz sampling must not be interrupted
  - RP2040 has no FPU (software floating-point)
  - Must work on both RP2040 and RP2350

## ADR & Legacy Alignment

- [ ] Confirm ADR-00006-imu-driver-architecture is referenced and approved
- [ ] Note: No legacy IMU driver code exists; this is initial implementation
- [ ] Ensure `SpiInterface` trait from ADR-00003 is used for platform abstraction
- [ ] Align with AHRS requirements from T-00005 (400Hz data feed)

## Plan Summary

- Phase 1 – Driver Foundation (register definitions, SPI communication, chip ID verification)
- Phase 2 – Sensor Reading (gyro/accel data reading, interrupt handling, error retry logic)
- Phase 3 – AHRS Integration (trait implementation, calibration, testing, documentation)

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: Driver Foundation

### Goal

Establish BMI088 driver scaffolding with register definitions, SPI communication, and chip ID verification. Verify hardware communication before implementing data reading logic.

### Inputs

- Documentation:
  - [BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
  - [BMI088 Application Note](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bmi088-an000.pdf)
  - [ADR-00006-imu-driver-architecture](../../adr/ADR-00006-imu-driver-architecture.md)
- Source Code to Modify:
  - Create `src/devices/traits/imu.rs` - `ImuSensor` trait definition
  - Create `src/devices/imu/bmi088/mod.rs` - Public exports
  - Create `src/devices/imu/bmi088/registers.rs` - Register definitions
  - Create `src/devices/imu/bmi088/config.rs` - Configuration structs
  - Create `src/devices/imu/bmi088/driver.rs` - Driver skeleton
- Dependencies:
  - Internal: `src/platform/` - `SpiInterface` trait
  - External crates: `embassy-rp`, `embassy-time`, `nalgebra`, `defmt`

### Tasks

- [ ] **Module structure setup**
  - [ ] Create `src/devices/traits/imu.rs` with `ImuSensor` trait definition
  - [ ] Create `src/devices/imu/bmi088/` module directory
  - [ ] Create `src/devices/imu/bmi088/mod.rs` with public exports
  - [ ] Update `src/devices/mod.rs` to include `imu` module
- [ ] **Register definitions**
  - [ ] Define gyro register addresses in `src/devices/imu/bmi088/registers.rs`
  - [ ] Define accel register addresses
  - [ ] Define bit masks for configuration registers
  - [ ] Add chip ID constants (0x0F gyro, 0x1E accel)
- [ ] **Configuration structures**
  - [ ] Define `GyroRange` enum (±125, ±250, ±500, ±1000, ±2000 °/s) in `config.rs`
  - [ ] Define `GyroOdr` enum (100, 200, 400, 1000, 2000 Hz)
  - [ ] Define `AccelRange` enum (±3, ±6, ±12, ±24 g)
  - [ ] Define `AccelOdr` enum (12.5, 25, 50, 100, 200, 400, 800, 1600 Hz)
  - [ ] Define `Bmi088Config` struct with default values
- [ ] **Driver skeleton and chip ID verification**
  - [ ] Define `Bmi088Driver` struct in `driver.rs` (SPI bus, CS pins, config)
  - [ ] Implement `new()` constructor
  - [ ] Implement SPI read/write helper methods
  - [ ] Implement `read_gyro_chip_id()` method
  - [ ] Implement `read_accel_chip_id()` method
  - [ ] Add basic error handling with `ImuError` enum

### Deliverables

- Module structure with trait, registers, config, and driver skeleton
- BMI088 register definitions for gyro and accel
- Configuration enums and default values
- Chip ID verification methods (0x0F gyro, 0x1E accel)

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::imu
```

### Acceptance Criteria (Phase Gate)

- Trait definition compiles without errors
- Register definitions are complete and documented
- Configuration structs have sensible defaults
- Driver skeleton compiles and links
- Chip ID verification methods are implemented (not yet tested on hardware)

### Rollback/Fallback

- Delete `src/devices/imu/` directory and revert module exports if fundamental design issues discovered
- Consult ArduPilot BMI088 driver implementation for reference

---

## Phase 2: Sensor Reading

### Phase 2 Goal

Implement gyro and accel data reading, interrupt handling, and error retry logic. Achieve 400Hz sampling with <1ms jitter.

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Driver foundation, register definitions, chip ID verification
  - Platform: `GpioInterface` trait for interrupt configuration
- Source Code to Modify:
  - `src/devices/imu/bmi088/driver.rs` - Add data reading methods
  - `src/devices/imu/bmi088/config.rs` - Add sensor configuration
- Documentation:
  - BMI088 datasheet sections on data reading and interrupt configuration

### Phase 2 Tasks

- [ ] **Sensor configuration**
  - [ ] Implement `configure()` method to set gyro range, ODR, and filters
  - [ ] Implement accel configuration (range, ODR, filters)
  - [ ] Implement interrupt configuration (data-ready on INT1/INT2, rising edge)
  - [ ] Verify configuration via register readback
- [ ] **Data reading methods**
  - [ ] Implement `read_gyro_raw()` (6-byte SPI burst read from 0x02-0x07)
  - [ ] Implement `read_accel_raw()` (6-byte SPI burst read from 0x12-0x17)
  - [ ] Implement raw-to-physical conversion (ADC → rad/s for gyro, m/s² for accel)
  - [ ] Implement `read_gyro()` method (raw read + conversion)
  - [ ] Implement `read_accel()` method (raw read + conversion)
  - [ ] Add combined `read_gyro_accel()` for efficiency
- [ ] **Interrupt handling**
  - [ ] Configure GPIO interrupts for data-ready signals (INT1 gyro, INT2 accel)
  - [ ] Implement interrupt-driven sampling task (Embassy async)
  - [ ] Measure sampling rate and jitter (timestamp tracking)
- [ ] **Error handling and retry logic**
  - [ ] Implement retry with exponential backoff (max 3 retries, 100µs/200µs/400µs)
  - [ ] Return `ImuError::SpiError` after max retries
  - [ ] Add sensor reset on persistent failure (> 10 consecutive errors)
  - [ ] Add defmt logging for errors (ERROR, WARN, INFO levels)

### Phase 2 Deliverables

- Sensor configuration methods for gyro and accel
- Raw and calibrated data reading methods
- Interrupt-driven sampling at 400Hz
- Error retry logic with exponential backoff
- Sampling rate and jitter measurement

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::imu
# Hardware test: Verify 400Hz sampling and <1ms jitter on Pico 2 W
./scripts/build-rp2350.sh imu_test
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/imu_test
```

### Phase 2 Acceptance Criteria

- Sensor configuration succeeds and registers are verified
- Gyro and accel data reading returns valid physical units
- Interrupt-driven sampling achieves 400Hz ± 5Hz
- Jitter standard deviation < 1ms measured over 10 seconds
- Error retry logic handles transient SPI failures
- No panics or crashes during 60-second continuous operation

### Phase 2 Rollback/Fallback

- If interrupt-driven sampling fails, fall back to polling at 400Hz (higher CPU overhead)
- If jitter exceeds 1ms, increase SPI clock speed or use DMA

---

## Phase 3: AHRS Integration

### Phase 3 Goal

Implement `ImuSensor` trait, integrate with AHRS, add calibration support, create mock implementation for testing, and finalize documentation.

### Phase 3 Tasks

- [ ] **Trait implementation**
  - [ ] Implement `ImuSensor` trait for `Bmi088Driver`
  - [ ] Implement `read_gyro()` returning `Result<Vector3<f32>, ImuError>`
  - [ ] Implement `read_accel()` returning `Result<Vector3<f32>, ImuError>`
  - [ ] Implement `read_mag()` returning `Ok(None)` (no magnetometer yet)
  - [ ] Implement `configure()` accepting `ImuConfig`
  - [ ] Implement `self_test()` method
- [ ] **Calibration integration**
  - [ ] Define `CalibrationData` struct (gyro bias, accel offset/scale)
  - [ ] Implement `set_gyro_bias()` method
  - [ ] Implement `set_accel_calibration()` method
  - [ ] Apply calibration in `read_gyro()` and `read_accel()` methods
  - [ ] Add placeholder for loading calibration from parameter system
- [ ] **Mock implementation for testing**
  - [ ] Create `src/devices/imu/mock.rs` with `MockImu` struct
  - [ ] Implement `ImuSensor` trait for `MockImu`
  - [ ] Support static and dynamic sensor data (configurable via constructor)
  - [ ] Add unit test demonstrating AHRS integration with `MockImu`
- [ ] **AHRS integration**
  - [ ] Update AHRS task (T-00005) to use `ImuSensor` trait
  - [ ] Test AHRS accuracy with real BMI088 data (±2° roll/pitch target)
  - [ ] Verify 100Hz AHRS update rate with 400Hz IMU data
  - [ ] Measure CPU overhead on RP2040 (< 10% target)
- [ ] **Documentation and logging**
  - [ ] Add module-level documentation for `src/devices/imu/bmi088/`
  - [ ] Document `ImuSensor` trait in `src/devices/traits/imu.rs`
  - [ ] Add defmt logging for initialization, errors, and performance metrics
  - [ ] Create wiring diagram for BMI088 connections (SPI, CS, interrupts)

### Phase 3 Deliverables

- Complete `ImuSensor` trait implementation for BMI088
- Calibration support (gyro bias, accel offset/scale)
- Mock IMU implementation for unit testing
- AHRS integration with ±2° accuracy demonstrated
- Comprehensive documentation and logging

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
# Hardware test: Verify AHRS accuracy with real BMI088 data
./scripts/build-rp2350.sh ahrs_imu_test
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/ahrs_imu_test
# Markdown documentation
bun format
bun lint
```

### Phase 3 Acceptance Criteria

- `ImuSensor` trait fully implemented and tested
- Calibration applied correctly (verified via static test data)
- `MockImu` enables AHRS unit testing without hardware
- AHRS achieves ±2° roll/pitch accuracy with real BMI088 data
- CPU overhead < 10% on RP2040 @ 133MHz
- All documentation complete and formatted correctly

---

## Definition of Done

- [ ] `cargo check`
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] Hardware tests pass on Pico 2 W (400Hz sampling, <1ms jitter, ±2° AHRS accuracy)
- [ ] `bun format` and `bun lint` pass for documentation
- [ ] Module documentation added for all public items
- [ ] Wiring diagram created for BMI088 connections
- [ ] Error messages actionable and in English
- [ ] No `unsafe` code
- [ ] No vague naming (no "manager"/"util")
- [ ] Platform verification completed (RP2040 and RP2350)

## Open Questions

- [ ] Should we implement hardware FIFO buffering for robustness? → Next step: Prototype in Phase 2 after basic driver works, measure benefits
- [ ] What DMA strategy is optimal for RP2040? → Method: Benchmark DMA vs interrupt-driven SPI in Phase 2
- [ ] Do we need gyro temperature compensation? → Next step: Measure temperature drift on hardware, add if > 0.01°/s/°C
