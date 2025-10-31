# T-49k7n AHRS DCM Implementation

## Metadata

- Type: Implementation Plan
- Status: Phase 1 In Progress

## Links

- Associated Design Document:
  - [T-49k7n-ahrs-dcm-implementation-design](design.md)

## Overview

Implement Direction Cosine Matrix (DCM) based AHRS for real-time attitude estimation by fusing gyroscope, accelerometer, and magnetometer data. This implementation provides roll, pitch, and heading estimates at 100Hz for navigation and control subsystems.

## Success Metrics

- [ ] Roll and pitch accuracy within ±2 degrees during static conditions
- [ ] Heading accuracy within ±5 degrees with calibrated magnetometer
- [ ] AHRS update rate at 100Hz minimum (< 10ms per cycle)
- [ ] Convergence to correct attitude within 5 seconds
- [ ] Memory footprint < 2 KB RAM for DCM state
- [ ] All existing tests pass; no regressions in IMU and task scheduler

## Scope

- Goal: Deliver functional DCM-based AHRS meeting FR-eyuh8 accuracy and performance requirements
- Non-Goals:
  - EKF implementation (deferred to Phase 2 per ADR-6twis)
  - GPS velocity integration for heading aid (future enhancement)
  - Advanced magnetometer calibration UI (use manual calibration initially)
- Assumptions:
  - IMU drivers (BMI088) are functional and provide data at 400Hz
  - Task scheduler (FR-5inw2) allocates 100Hz execution slot
  - Parameter system (T-ex2h7) handles calibration persistence
- Constraints:
  - Pico W: No FPU, software floating-point math only
  - Pico 2 W: Hardware FPU available
  - CPU budget: < 10ms per update cycle
  - Memory budget: < 10 KB RAM total for AHRS subsystem

## ADR & Legacy Alignment

- [ ] ADR-6twis (AHRS Algorithm Selection) mandates DCM for initial implementation
- [ ] FR-eyuh8 (AHRS Attitude Estimation) specifies accuracy and performance requirements
- [ ] NFR-3wlo1 (IMU Sampling Rate) ensures 400Hz IMU data available for 100Hz AHRS updates
- [ ] No legacy AHRS code exists - greenfield implementation

## Plan Summary

- Phase 1 – Core DCM Algorithm (matrix operations, gyro integration, normalization)
- Phase 2 – Sensor Fusion & Corrections (accelerometer/magnetometer corrections, calibration)
- Phase 3 – AHRS Task Integration (Embassy task, shared state, performance profiling)

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. Annotate deferred or skipped items with strike-through and reason.

---

## Phase 1: Core DCM Algorithm

### Goal

Implement DCM state structure, matrix operations, gyro integration, and normalization. Establish unit test framework for algorithm correctness.

### Inputs

- Documentation:
  - `/docs/adr/ADR-6twis-ahrs-algorithm-selection.md` – Algorithm choice rationale
  - `/docs/requirements/FR-eyuh8-ahrs-attitude-estimation.md` – Requirements
  - ArduPilot DCM reference: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_AHRS>
- Source Code to Modify:
  - Create `/src/subsystems/ahrs/` module structure
  - Create `/src/subsystems/ahrs/dcm.rs` – Core DCM algorithm
- Dependencies:
  - External crates: `nalgebra` (matrix operations), `micromath` (fast trig approximations)
  - Internal: None (standalone module initially)

### Tasks

- [x] **Module scaffolding**
  - [x] Create `src/subsystems/ahrs/` directory
  - [x] Add `mod.rs` with public exports
  - [x] Add `dcm.rs` stub with placeholder structs
  - [x] Update `src/subsystems/mod.rs` to include `ahrs` module
- [x] **DCM state structures**
  - [x] Define `DcmState` struct (dcm_matrix, gyro_bias, omega_p, omega_i)
  - [x] Define `DcmConfig` struct (kp_roll_pitch, ki_roll_pitch, kp_yaw, ki_yaw)
  - [x] Implement `Default` trait for `DcmConfig` with tuning gains from ADR-6twis
  - [x] Implement `Dcm::new(config)` constructor (initialize with identity matrix)
- [x] **Gyro integration**
  - [x] Implement `rotation_matrix(omega: Vector3<f32>, dt: f32) -> Matrix3<f32>` helper
  - [x] Implement gyro integration: `dcm_matrix *= rotation_matrix(omega, dt)`
  - [x] Add unit test: Pure rotation around Z-axis, verify yaw changes correctly
- [x] **Matrix normalization**
  - [x] Implement `orthonormalize(matrix: Matrix3<f32>) -> Matrix3<f32>` (Gram-Schmidt)
  - [x] Add determinant check (warn if `|det(M) - 1.0| > 0.1`)
  - [x] Add unit test: Verify orthogonality preserved after multiple integrations
- [x] **Euler angle extraction**
  - [x] Implement `Dcm::get_euler_angles() -> (f32, f32, f32)` (roll, pitch, yaw)
  - [x] Handle gimbal lock at ±90° pitch (documented limitation)
  - [x] Add unit test: Known DCM matrices → expected Euler angles

### Deliverables

- `src/subsystems/ahrs/dcm.rs` with core DCM algorithm (gyro integration + normalization)
- Unit tests covering gyro integration, normalization, and Euler extraction
- Compiles successfully with `nalgebra` and `micromath` dependencies

### Verification

```bash
# Build and checks
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
# Unit tests for DCM module
cargo test --lib --quiet ahrs::dcm
```

### Acceptance Criteria (Phase Gate)

- `DcmState` and `DcmConfig` structures defined and documented
- Gyro integration produces correct rotation matrices (unit test passes)
- Matrix normalization maintains orthogonality (determinant ≈ 1.0)
- Euler angle extraction handles all quadrants and gimbal lock edge cases
- All unit tests pass; clippy reports no warnings

### Rollback/Fallback

- If `nalgebra` overhead too high on Pico W, switch to manual 3x3 matrix implementation
- If gimbal lock unavoidable with Euler angles, switch to quaternion representation internally

---

## Phase 2: Sensor Fusion & Corrections

### Phase 2 Goal

Implement accelerometer and magnetometer corrections using PI controller. Add calibration data structures and persistence integration.

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Core DCM algorithm with gyro integration
  - Parameter system (T-ex2h7) for calibration persistence
- Source Code to Modify:
  - `/src/subsystems/ahrs/dcm.rs` – Add `update()` and `update_with_mag()` methods
  - Create `/src/subsystems/ahrs/calibration.rs` – Calibration data structure

### Phase 2 Tasks

- [ ] **Accelerometer correction**
  - [ ] Implement `Dcm::update(gyro, accel, dt)` method
  - [ ] Compute accel error: `error = accel.cross(&accel_ref)`
  - [ ] Apply PI correction: `omega_p = error * kp`, `omega_i += error * ki * dt`
  - [ ] Add unit test: Static pitch/roll recovery from tilted initial state
- [ ] **Magnetometer correction**
  - [ ] Implement `Dcm::update_with_mag(mag)` method
  - [ ] Compute heading error from magnetometer
  - [ ] Apply yaw correction to `omega_p.z` and `omega_i.z`
  - [ ] Add unit test: Heading correction from known magnetic field vector
- [ ] **Calibration structures**
  - [ ] Define `CalibrationData` struct (accel_offset, accel_scale, mag_offset, mag_scale, gyro_bias)
  - [ ] Implement `apply_accel_calibration(raw: Vector3) -> Vector3`
  - [ ] Implement `apply_mag_calibration(raw: Vector3) -> Vector3`
  - [ ] Add unit test: Apply known offsets/scales, verify output
- [ ] **Calibration persistence**
  - [ ] Integrate with parameter system (load calibration on startup)
  - [ ] Add parameter definitions for accel/mag offsets and scales
  - [ ] Implement `load_calibration() -> Result<CalibrationData>`
  - [ ] Add fallback: Use default identity calibration if load fails
- [ ] **Gyro bias estimation**
  - [ ] Implement initialization routine: Accumulate gyro samples for 1 second, compute mean
  - [ ] Store bias in `gyro_bias` field, subtract from raw gyro readings
  - [ ] Add unit test: Bias estimation from synthetic noisy gyro data

### Phase 2 Deliverables

- Full DCM update cycle with accel/mag corrections
- Calibration data structures and parameter integration
- Unit tests covering sensor fusion correctness

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet ahrs
```

### Phase 2 Acceptance Criteria

- `Dcm::update()` fuses gyro and accel data with PI correction
- `Dcm::update_with_mag()` corrects heading using magnetometer
- Calibration data loads from parameter system successfully
- All unit tests pass (accel correction, mag correction, calibration)

### Phase 2 Rollback/Fallback

- If PI gains cause instability, fall back to complementary filter approach (fixed alpha blend)
- If parameter load fails, use uncalibrated mode with warning flag

---

## Phase 3: AHRS Task Integration

### Phase 3 Goal

Integrate DCM algorithm into Embassy async task, publish attitude to shared state, validate performance on hardware, and document usage.

### Phase 3 Inputs

- Dependencies:
  - Phase 2: Complete DCM with sensor fusion
  - Task scheduler (FR-5inw2) with 100Hz slot allocation
  - IMU driver (BMI088) providing gyro/accel data
- Source Code to Modify:
  - Create `/src/subsystems/ahrs/task.rs` – Embassy async task
  - Create `/src/subsystems/ahrs/state.rs` – Shared attitude state

### Phase 3 Tasks

- [ ] **Shared attitude state**
  - [ ] Define `AttitudeState` struct (roll, pitch, yaw, angular_rates, timestamp, quality)
  - [ ] Implement thread-safe access (use `embassy_sync::mutex::Mutex` or atomic)
  - [ ] Add getter methods: `get_roll()`, `get_pitch()`, `get_yaw()`
- [ ] **AHRS task implementation**
  - [ ] Create `ahrs_task()` async function
  - [ ] Initialize DCM and calibration on startup
  - [ ] Main loop: Read IMU at 100Hz, call `Dcm::update()`, publish to shared state
  - [ ] Magnetometer loop: Read mag at 10Hz, call `Dcm::update_with_mag()`
  - [ ] Add convergence detection: Set `quality.converged` flag after 5 seconds
- [ ] **Task scheduler integration**
  - [ ] Register AHRS task with task scheduler (100Hz priority slot)
  - [ ] Verify task execution via `defmt` logs
  - [ ] Measure cycle time with `embassy_time::Instant`, log if > 8ms
- [ ] **Performance profiling**
  - [ ] Build for Pico W (RP2040): `./scripts/build-rp2350.sh ahrs_demo`
  - [ ] Flash and run on hardware: `probe-rs run --chip RP2350 target/.../ahrs_demo`
  - [ ] Capture cycle time statistics (min/max/avg)
  - [ ] Verify < 10ms cycle time on Pico W, < 5ms on Pico 2 W
- [ ] **Integration tests**
  - [ ] Add test under `tests/ahrs_integration.rs`
  - [ ] Test scenario: AHRS task running in Embassy executor, verify attitude updates
  - [ ] Test convergence: Start with random orientation, verify convergence within 5s
  - [ ] Test graceful degradation: Disable magnetometer, verify gyro-only mode
- [ ] **Documentation**
  - [ ] Add `docs/subsystems/ahrs.md` with usage guide
  - [ ] Document calibration procedure (6-position accel, mag sphere fitting)
  - [ ] Document tuning parameters (kp/ki gains)
  - [ ] Update `docs/architecture.md` with AHRS subsystem section

### Phase 3 Deliverables

- Functional AHRS task running on hardware at 100Hz
- Shared attitude state accessible to navigation subsystem
- Performance validated on both Pico W and Pico 2 W
- User documentation for calibration and tuning

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
# Integration tests
cargo test --quiet --test ahrs_integration
# Hardware validation
./scripts/build-rp2350.sh ahrs_demo
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/ahrs_demo
```

### Phase 3 Acceptance Criteria

- AHRS task executes at 100Hz on hardware without timing violations
- Attitude updates visible in shared state (verified via defmt logs)
- Roll/pitch accuracy within ±2° during static test
- Heading accuracy within ±5° with calibrated magnetometer
- Convergence time < 5 seconds from startup
- Integration tests pass on host system

---

## Definition of Done

- [ ] `cargo check`
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] Integration tests pass: `cargo test --quiet --test ahrs_integration`
- [ ] Hardware validation on Pico 2 W (performance and accuracy verified)
- [ ] `docs/subsystems/ahrs.md` created with calibration and tuning guide
- [ ] `docs/architecture.md` updated with AHRS subsystem description
- [ ] `docs/traceability.md` updated with T-49k7n links
- [ ] Error messages in English; no `unsafe` code; no "manager"/"util" naming
- [ ] Memory profiling confirms < 2 KB RAM usage for DCM state

## Open Questions

- [ ] Should we support runtime gain tuning via parameters? → Next step: Implement basic parameters first, defer runtime tuning to user feedback phase
- [ ] What fallback behavior if IMU read timeout occurs? → Method: Log error and hold previous attitude estimate for up to 500ms, then reset DCM state
- [ ] Is Gram-Schmidt normalization sufficient or should we use SVD for robustness? → Method: Profile Gram-Schmidt performance; if determinant drift observed, upgrade to SVD

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
