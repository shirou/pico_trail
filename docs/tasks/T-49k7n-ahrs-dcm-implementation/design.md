# T-49k7n AHRS DCM Implementation

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-49k7n-ahrs-dcm-implementation-plan](plan.md)

## Overview

Implement an Attitude and Heading Reference System (AHRS) using Direction Cosine Matrix (DCM) algorithm to provide accurate real-time attitude estimation (roll, pitch, heading) by fusing gyroscope, accelerometer, and magnetometer data. This implementation satisfies FR-eyuh8 and follows the architecture decision documented in ADR-6twis.

## Success Metrics

- [ ] Roll and pitch accuracy within ±2 degrees during static conditions
- [ ] Heading accuracy within ±5 degrees with calibrated magnetometer
- [ ] AHRS update rate at 100Hz minimum
- [ ] Convergence to correct attitude within 5 seconds of system startup
- [ ] Memory footprint < 2 KB RAM for DCM state
- [ ] Update cycle completes within 10ms (100Hz requirement)
- [ ] Zero regressions in existing IMU and task scheduler functionality

## Background and Current State

- Context: AHRS is a fundamental subsystem for autonomous navigation and control. It transforms raw sensor data into attitude estimates that navigation algorithms consume.
- Current behavior: IMU drivers exist (BMI088, placeholder for MPU series), but no sensor fusion or attitude estimation capability. Task scheduler (FR-5inw2) provides 100Hz execution slots.
- Pain points: Without AHRS, navigation algorithms cannot determine vehicle orientation, preventing waypoint navigation (FR-333ym).
- Constraints:
  - Pico W (RP2040): No FPU, software floating-point math only
  - Pico 2 W (RP2350): Hardware FPU available
  - Memory budget: < 10 KB RAM for AHRS subsystem
  - CPU budget: < 10ms per update (100Hz rate)
- Related ADRs: [ADR-6twis-ahrs-algorithm-selection](../../adr/ADR-6twis-ahrs-algorithm-selection.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                    Task Scheduler (100Hz)                    │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          v
┌─────────────────────────────────────────────────────────────┐
│                      AHRS Subsystem                          │
│  ┌──────────────┐   ┌──────────────┐   ┌───────────────┐   │
│  │  IMU Reader  │   │  Mag Reader  │   │  Calibration  │   │
│  │  (gyro+acc)  │   │  (optional)  │   │    Manager    │   │
│  └──────┬───────┘   └──────┬───────┘   └───────┬───────┘   │
│         │                  │                    │            │
│         v                  v                    v            │
│  ┌────────────────────────────────────────────────────────┐ │
│  │               DCM Algorithm Core                       │ │
│  │  - Matrix integration & normalization                  │ │
│  │  - Accel/mag error correction (PI controller)          │ │
│  │  - Gyro bias estimation                                │ │
│  └────────────────────┬───────────────────────────────────┘ │
│                       │                                      │
│                       v                                      │
│  ┌────────────────────────────────────────────────────────┐ │
│  │           Attitude Output (roll, pitch, yaw)           │ │
│  └────────────────────┬───────────────────────────────────┘ │
└────────────────────────┼──────────────────────────────────-─┘
                         │
                         v
┌─────────────────────────────────────────────────────────────┐
│          Navigation & Control Subsystems                     │
└─────────────────────────────────────────────────────────────┘
```

### Components

**Core DCM Module** (`src/subsystems/ahrs/dcm.rs`):

- `DcmState`: Rotation matrix (3x3), gyro bias (3D), PI correction terms
- `DcmConfig`: Tuning gains (kp_roll_pitch, ki_roll_pitch, kp_yaw, ki_yaw)
- `Dcm::update()`: Main update cycle - integrate gyro, normalize matrix, apply corrections
- `Dcm::update_with_mag()`: Optional magnetometer heading correction
- `Dcm::get_euler_angles()`: Extract roll, pitch, yaw from DCM matrix

**AHRS Task** (`src/subsystems/ahrs/task.rs`):

- `AhrsTask`: Embassy async task running at 100Hz via task scheduler
- Reads IMU data (gyro + accel) at 100Hz
- Reads magnetometer at 10Hz (if available)
- Calls `Dcm::update()` and publishes attitude to shared state

**Calibration Module** (`src/subsystems/ahrs/calibration.rs`):

- Accelerometer offset/scale calibration (6-position calibration)
- Magnetometer offset/scale calibration (sphere/ellipsoid fitting)
- Gyro bias estimation during initialization
- Calibration data persistence (via parameter system from T-ex2h7)

**Shared Attitude State** (`src/subsystems/ahrs/state.rs`):

- `AttitudeState`: Thread-safe shared state (Mutex or atomic)
- Fields: roll, pitch, yaw (f32), angular_rates (Vector3), timestamp, quality_flag
- Consumers: Navigation subsystem, MAVLink reporting

### Data Flow

1. Task Scheduler triggers AHRS task at 100Hz
2. AHRS task reads IMU data (gyro rad/s, accel m/s²) from device queue
3. Apply calibration offsets/scales to raw sensor data
4. Call `Dcm::update(gyro, accel, dt)`:
   - Integrate gyro to update DCM matrix
   - Normalize matrix (prevent drift)
   - Compute accel error vector (cross product)
   - Apply PI correction to omega
5. Every 10th cycle (10Hz), call `Dcm::update_with_mag(mag)` if magnetometer available
6. Extract Euler angles from DCM matrix
7. Publish attitude to shared state (atomic write or mutex)
8. Navigation subsystem reads shared state as needed

### Data Models and Types

```rust
/// Direction Cosine Matrix state
pub struct DcmState {
    /// 3x3 rotation matrix (body to earth frame)
    dcm_matrix: Matrix3<f32>,
    /// Estimated gyro bias (rad/s)
    gyro_bias: Vector3<f32>,
    /// Proportional correction term
    omega_p: Vector3<f32>,
    /// Integral correction term
    omega_i: Vector3<f32>,
}

/// DCM algorithm configuration
pub struct DcmConfig {
    /// P gain for roll/pitch correction
    kp_roll_pitch: f32,  // Default: 0.2
    /// I gain for roll/pitch correction
    ki_roll_pitch: f32,  // Default: 0.0005
    /// P gain for yaw correction
    kp_yaw: f32,         // Default: 1.0
    /// I gain for yaw correction
    ki_yaw: f32,         // Default: 0.00005
}

/// Published attitude estimate
pub struct AttitudeState {
    /// Roll angle (radians)
    roll: f32,
    /// Pitch angle (radians)
    pitch: f32,
    /// Yaw/heading angle (radians, 0=north)
    yaw: f32,
    /// Angular rates (rad/s)
    angular_rates: Vector3<f32>,
    /// Timestamp (microseconds)
    timestamp_us: u64,
    /// Quality flags (bit field: accel_valid, mag_valid, converged)
    quality: u8,
}

/// Calibration data (persisted via parameter system)
pub struct CalibrationData {
    /// Accel offsets (m/s²)
    accel_offset: Vector3<f32>,
    /// Accel scale factors
    accel_scale: Vector3<f32>,
    /// Mag offsets (µT)
    mag_offset: Vector3<f32>,
    /// Mag scale factors
    mag_scale: Vector3<f32>,
    /// Gyro bias (rad/s)
    gyro_bias: Vector3<f32>,
}
```

### Error Handling

- English error messages for all failure cases
- Error types:
  - `AhrsError::ImuReadTimeout` - IMU data not available within deadline
  - `AhrsError::MagNotCalibrated` - Magnetometer used before calibration
  - `AhrsError::ConvergenceFailed` - Attitude failed to converge within 5s
  - `AhrsError::MatrixInvalid` - DCM matrix became non-orthogonal (numerical instability)
- Use `defmt` logging for diagnostics (requires defmt feature)
- Graceful degradation: Continue with gyro-only if accel/mag unavailable (set quality flags)
- Watchdog: Reset DCM state if matrix determinant deviates from 1.0 by > 0.1

### Security Considerations

- N/A - Embedded subsystem, no external inputs or network access
- Calibration data integrity: Use CRC when loading from flash (handled by parameter system)

### Performance Considerations

- Hot path: `Dcm::update()` called at 100Hz, must complete in < 10ms
- Matrix operations: Use `nalgebra` with `no_std` feature
  - Pico W: Software float math, optimize for minimal allocations
  - Pico 2 W: Hardware FPU accelerates `f32` operations
- Trigonometric functions: Use `micromath::F32Ext` for fast approximations (atan2, asin, sin, cos)
- Avoid heap allocations in update loop - all state pre-allocated
- Profiling: Measure cycle time with Embassy's `time::Instant`, log if > 8ms (warning threshold)

### Platform Considerations

#### Pico W (RP2040)

- No FPU: Software floating-point via libm
- Expected DCM update time: 5-8ms at 100Hz
- May need to tune convergence gains for stability
- Use `cortex_m::asm::nop()` for cycle counting if profiling needed

#### Pico 2 W (RP2350)

- Hardware FPU: Accelerates `f32` matrix math and trig functions
- Expected DCM update time: 2-4ms at 100Hz
- FPU enables headroom for future EKF upgrade (Phase 2)

#### Cross-Platform

- Use conditional compilation for platform-specific optimizations:

  ```rust
  #[cfg(feature = "rp2040")]
  const DEFAULT_KP: f32 = 0.15;  // Lower gain for stability on soft-float

  #[cfg(feature = "rp2350")]
  const DEFAULT_KP: f32 = 0.2;   // Higher gain feasible with FPU
  ```

- Abstract attitude interface: `pub trait AttitudeProvider` for future EKF swap

## Alternatives Considered

1. **Madgwick/Mahony Filter**
   - Pros: Simpler than DCM, minimal memory (1 KB)
   - Cons: Lower accuracy (±3-5°), no magnetometer hard/soft iron correction
   - Decision: Rejected in ADR-6twis due to insufficient accuracy

2. **Simplified EKF (6-9 states)**
   - Pros: Better accuracy (±1-2°), explicit covariance modeling
   - Cons: Higher complexity, 4x memory usage, requires FPU for acceptable performance
   - Decision: Deferred to Phase 2 upgrade (Pico 2 W only)

3. **Full EKF (15-24 states)**
   - Pros: Maximum accuracy and robustness
   - Cons: Too memory/CPU intensive for Pico W, overkill for rover/boat navigation
   - Decision: Out of scope for embedded constraints

## Decision Rationale

DCM was chosen because:

- Proven track record in ArduPilot (10+ years production use)
- Meets accuracy requirements (±2° roll/pitch, ±5° heading)
- Works efficiently on Pico W without FPU
- Low memory footprint (2 KB) leaves headroom for other features
- Faster to implement and validate than EKF

Per ADR-6twis, EKF can be added later as a compile-time feature for Pico 2 W if field testing reveals DCM accuracy is insufficient.

## Migration and Compatibility

- Backward compatibility: N/A - New feature
- Forward compatibility: Design abstraction (`AttitudeProvider` trait) allows future EKF swap without breaking consumers
- Rollout plan:
  - Phase 1: DCM implementation for both platforms
  - Phase 2 (future): Optional EKF upgrade via `--features ahrs-ekf` for Pico 2 W

## Testing Strategy

### Unit Tests

- Place tests in `src/subsystems/ahrs/tests.rs` with `#[cfg(test)]`
- Test cases:
  - DCM matrix initialization (identity matrix)
  - Gyro integration with zero bias (pure rotation)
  - Accel correction (static roll/pitch recovery)
  - Magnetometer heading correction
  - Matrix normalization (orthogonality preserved)
  - Edge cases: Gimbal lock at ±90° pitch, 360° wrap-around
- Mock IMU data with known trajectories, assert Euler angles within tolerance

### Integration Tests

- Add tests under `tests/ahrs_integration.rs`
- Test scenarios:
  - Full AHRS task running in Embassy executor (100Hz loop)
  - Calibration load/save via parameter system
  - Graceful degradation when magnetometer unavailable
  - Convergence time from random initial state to correct attitude
- Use recorded IMU data from ArduPilot MAVLog for validation

### External API Parsing

- N/A - No external APIs consumed

### Performance & Benchmarks

- Criterion benchmark in `benches/dcm_update.rs`:
  - Measure `Dcm::update()` cycle time on Pico W (RP2040) and Pico 2 W (RP2350)
  - Assert: Pico W < 10ms, Pico 2 W < 5ms
- Embedded profiling: Log cycle times with `defmt::info!("AHRS cycle: {}us", elapsed_us)`

## Documentation Impact

- Update `docs/architecture.md`: Add AHRS subsystem section
- Add `docs/subsystems/ahrs.md`: User guide for calibration and tuning
- Update `docs/traceability.md`: Link T-49k7n to FR-eyuh8 and ADR-6twis
- Add calibration procedure to user documentation

## External References

- ArduPilot DCM Implementation: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_AHRS>
- DCM Tutorial (Starlino): <http://www.starlino.com/dcm_tutorial.html>
- Direction Cosine Matrix IMU: <https://www.instructables.com/id/Guide-to-gyro-and-accelerometer-with-Arduino-incl/>

## Open Questions

- [ ] Should we implement online gyro bias estimation or only during initialization? → Method: Profile CPU overhead, decide based on Pico W performance
- [ ] What default tuning gains (kp/ki) work best across different IMU sensors? → Next step: Empirical tuning with BMI088, document in calibration guide
- [ ] Is 10Hz magnetometer update rate sufficient, or should we increase to 50Hz? → Method: Test heading accuracy with moving platform, adjust if drift observed

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
