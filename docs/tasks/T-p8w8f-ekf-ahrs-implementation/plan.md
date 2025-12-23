# T-p8w8f EKF AHRS Implementation

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [T-p8w8f-ekf-ahrs-implementation-design](./design.md)

## Overview

Implement a 7-state quaternion-based Extended Kalman Filter for attitude estimation, replacing DCM as the primary AHRS algorithm. The EKF provides accurate roll/pitch/yaw estimates with automatic gyroscope bias compensation and exposes attitude data through a global `ATTITUDE_STATE` interface.

## Success Metrics

- [ ] Roll/pitch accuracy within ±2° under static conditions
- [ ] Heading accuracy within ±5° with calibrated magnetometer
- [ ] EKF 100Hz update completes in < 10ms on RP2040
- [ ] Memory usage < 10KB for EKF state and covariance
- [ ] Convergence within 5 seconds of startup
- [ ] All unit tests pass; embedded build succeeds on RP2350

## Scope

- Goal: Complete 7-state quaternion EKF with global attitude state interface
- Non-Goals: GPS fusion, adaptive noise, DCM removal, navigation EKF
- Assumptions: IMU driver (T-kx79g) provides calibrated data at 400Hz
- Constraints: no_std, no heap, Embassy async, RP2040 no FPU

## ADR & Legacy Alignment

- [ ] Confirm ADR-ymkzt-ekf-ahrs-implementation is referenced and followed
- [ ] Note: ADR-6twis (AHRS algorithm selection) is superseded by ADR-ymkzt
- [ ] Mark T-49k7n (DCM implementation) as superseded but keep code for reference

## Plan Summary

- Phase 1 - Quaternion math utilities and state structure
- Phase 2 - EKF prediction step (gyro integration)
- Phase 3 - EKF update step (accel and mag corrections)
- Phase 4 - Global attitude state and Embassy integration
- Phase 5 - MAVLink integration and testing

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Quaternion Math and State Structure

### Goal

- Implement quaternion math utilities and define EKF state structure

### Inputs

- Documentation:
  - [FR-3f2cn-quaternion-ekf-ahrs](../../requirements/FR-3f2cn-quaternion-ekf-ahrs.md)
  - [ADR-ymkzt-ekf-ahrs-implementation](../../adr/ADR-ymkzt-ekf-ahrs-implementation.md)
- Source Code to Modify:
  - `src/subsystems/ahrs/` - New EKF module
- Dependencies:
  - External crates: `micromath` (trig functions)

### Tasks

- [ ] **Quaternion utilities**
  - [ ] Create `src/subsystems/ahrs/quaternion.rs`
  - [ ] Implement quaternion struct with \[w, x, y, z] storage
  - [ ] Implement `normalize()` function
  - [ ] Implement quaternion multiplication
  - [ ] Implement quaternion derivative from angular rate
  - [ ] Implement rotation of vector by quaternion
  - [ ] Implement quaternion to Euler conversion
  - [ ] Implement quaternion from Euler (for initialization)
- [ ] **EKF state structure**
  - [ ] Create `src/subsystems/ahrs/ekf.rs`
  - [ ] Define `AhrsEkf` struct with state vector and covariance
  - [ ] Define `EkfConfig` with noise parameters
  - [ ] Implement `new()` with default initial state
  - [ ] Implement getter methods for quaternion, bias, covariance
- [ ] **Matrix utilities**
  - [ ] Implement 3x3 matrix operations (multiply, transpose, invert)
  - [ ] Implement 7x7 covariance operations (or use flat array approach)
  - [ ] Avoid heap allocation; use fixed-size arrays

### Deliverables

- Complete quaternion math library
- EKF struct with state and covariance storage
- Matrix utilities for Kalman operations

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet subsystems::ahrs
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- Quaternion operations compile and pass unit tests
- EKF struct can be instantiated with default state
- Quaternion-to-Euler conversion produces correct angles

### Rollback/Fallback

- New files only; no impact on existing AHRS code

---

## Phase 2: EKF Prediction Step

### Phase 2 Goal

- Implement gyroscope-based state prediction at 400Hz

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Quaternion math, EKF struct
- Source Code to Modify:
  - `src/subsystems/ahrs/ekf.rs` - Add prediction method

### Phase 2 Tasks

- [ ] **Prediction implementation**
  - [ ] Implement `predict(gyro: [f32; 3], dt: f32)` method
  - [ ] Subtract gyro bias from measurement
  - [ ] Compute quaternion derivative using corrected gyro
  - [ ] Integrate quaternion with first-order Euler
  - [ ] Normalize quaternion after integration
- [ ] **Covariance propagation**
  - [ ] Compute state transition Jacobian F
  - [ ] Propagate covariance: P = F × P × F^T + Q × dt
  - [ ] Ensure covariance remains symmetric positive definite
- [ ] **Gyro bias dynamics**
  - [ ] Model bias as random walk (identity dynamics)
  - [ ] Add process noise for bias states

### Phase 2 Deliverables

- Complete prediction step implementation
- Covariance propagation with correct Jacobians
- Unit tests for prediction with known inputs

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet subsystems::ahrs::ekf
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- Prediction integrates quaternion correctly
- Covariance grows appropriately with time
- No quaternion denormalization

### Phase 2 Rollback/Fallback

- Prediction incomplete but doesn't affect other code

---

## Phase 3: EKF Update Step

### Phase 3 Goal

- Implement accelerometer and magnetometer measurement updates

### Phase 3 Inputs

- Dependencies:
  - Phase 2: Prediction step with covariance
- Source Code to Modify:
  - `src/subsystems/ahrs/ekf.rs` - Add update methods

### Phase 3 Tasks

- [ ] **Accelerometer update**
  - [ ] Implement `update_accel(accel: [f32; 3])` method
  - [ ] Normalize accelerometer to unit vector
  - [ ] Predict gravity direction from quaternion
  - [ ] Compute innovation (measured - predicted)
  - [ ] Compute measurement Jacobian H (3x7)
  - [ ] Compute Kalman gain K
  - [ ] Apply state correction
  - [ ] Update covariance: P = (I - K × H) × P
  - [ ] Re-normalize quaternion
- [ ] **Magnetometer update**
  - [ ] Implement `update_mag(mag: [f32; 3])` method
  - [ ] Normalize magnetometer to unit vector
  - [ ] Predict magnetic field from quaternion and reference
  - [ ] Compute heading error (horizontal plane)
  - [ ] Compute measurement Jacobian for heading
  - [ ] Apply Kalman correction for yaw only
- [ ] **Outlier rejection**
  - [ ] Reject accel updates when |accel| far from 1g
  - [ ] Reject mag updates when |mag| abnormal
  - [ ] Log rejected measurements

### Phase 3 Deliverables

- Complete accelerometer update implementation
- Complete magnetometer update implementation
- Outlier rejection for robustness

### Phase 3 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet subsystems::ahrs::ekf
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- Accel update corrects roll/pitch error
- Mag update corrects heading error
- Filter converges to correct attitude from wrong initial state

### Phase 3 Rollback/Fallback

- Updates can be disabled; prediction still works

---

## Phase 4: Global Attitude State and Embassy Integration

### Phase 4 Goal

- Create global `ATTITUDE_STATE` and integrate with Embassy async runtime

### Phase 4 Inputs

- Dependencies:
  - Phase 3: Complete EKF implementation
  - T-kx79g: IMU driver with `ImuSensor` trait
- Source Code to Modify:
  - `src/subsystems/ahrs/state.rs` - New file
  - `src/subsystems/ahrs/mod.rs` - Embassy task

### Phase 4 Tasks

- [ ] **AttitudeState structure**
  - [ ] Create `src/subsystems/ahrs/state.rs`
  - [ ] Define `AttitudeState` struct with all fields
  - [ ] Implement `Default` trait
  - [ ] Implement helper methods (is_stale, etc.)
- [ ] **Global state**
  - [ ] Declare `ATTITUDE_STATE` static with mutex
  - [ ] Implement update method for EKF to publish
  - [ ] Implement read methods for consumers
- [ ] **Health monitoring**
  - [ ] Monitor covariance trace for divergence
  - [ ] Set healthy flag based on convergence
  - [ ] Bound gyro bias estimates
  - [ ] Log warnings for health issues
- [ ] **Embassy task**
  - [ ] Create `ahrs_task` in `mod.rs`
  - [ ] Initialize EKF with config
  - [ ] Set up 400Hz ticker
  - [ ] Read IMU data via `ImuSensor` trait
  - [ ] Run prediction every sample
  - [ ] Run accel update every 4th sample (100Hz)
  - [ ] Run mag update every 40th sample (10Hz)
  - [ ] Update global state every 4th sample

### Phase 4 Deliverables

- Global `ATTITUDE_STATE` accessible by all subsystems
- Embassy task for continuous EKF operation
- Health monitoring and divergence detection

### Phase 4 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet subsystems::ahrs
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 4 Acceptance Criteria

- `ATTITUDE_STATE` accessible from other modules
- Embassy task compiles and starts without error
- Health status correctly reflects filter state

### Phase 4 Rollback/Fallback

- Task can be disabled; DCM still available as fallback

---

## Phase 5: MAVLink Integration and Testing

### Phase 5 Goal

- Implement MAVLink attitude messages and comprehensive testing

### Phase 5 Tasks

- [ ] **MAVLink messages**
  - [ ] Create/update `src/communication/mavlink/handlers/attitude.rs`
  - [ ] Implement `create_attitude_message()` (ID 30)
  - [ ] Implement `create_attitude_quaternion_message()` (ID 31)
  - [ ] Register handler with dispatcher for attitude requests
- [ ] **Telemetry integration**
  - [ ] Add attitude to periodic telemetry at 10Hz
  - [ ] Include gyro bias in debug telemetry
- [ ] **Unit tests**
  - [ ] Test quaternion math edge cases
  - [ ] Test Euler conversion at gimbal lock angles
  - [ ] Test EKF convergence with simulated data
  - [ ] Test gyro bias estimation convergence
  - [ ] Test outlier rejection
- [ ] **Documentation**
  - [ ] Update `docs/architecture.md` with AHRS subsystem
  - [ ] Add EKF tuning notes

### Phase 5 Deliverables

- MAVLink attitude messages working
- Comprehensive test suite
- Updated documentation

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

- MAVLink attitude messages display in GCS
- All unit tests pass
- Documentation updated

---

## Definition of Done

- [ ] `cargo check`
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] Architecture documentation updated
- [ ] ADR-ymkzt referenced and followed
- [ ] Error messages actionable and in English
- [ ] No `unsafe` and no vague naming

## Open Questions

- [ ] What Q values work best for rover/boat dynamics? → Method: Tune on hardware in Phase 5
- [ ] Should mag update correct full attitude or heading only? → Decision: Heading only initially
- [ ] Is adaptive process noise needed? → Method: Evaluate fixed noise first

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
