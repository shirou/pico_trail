# T-00022 EKF AHRS Implementation

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-00022-ekf-ahrs-implementation-plan](./plan.md)

## Overview

This design implements a 7-state quaternion-based Extended Kalman Filter (EKF) for attitude estimation in the pico_trail autopilot. The EKF fuses gyroscope, accelerometer, and magnetometer data to provide accurate roll, pitch, and yaw estimates with automatic gyroscope bias compensation. The implementation supersedes the existing DCM algorithm, providing better accuracy, explicit uncertainty estimation, and gimbal-lock-free operation through quaternion representation.

## Success Metrics

- [ ] Roll/pitch accuracy within ±2° under static conditions
- [ ] Heading accuracy within ±5° with calibrated magnetometer
- [ ] EKF 100Hz update completes in < 10ms on RP2040
- [ ] Memory usage < 10KB for EKF state and covariance
- [ ] Attitude convergence within 5 seconds of startup

## Background and Current State

- Context: AHRS subsystem provides attitude for navigation and control; GPS already uses global state pattern
- Current behavior: DCM algorithm implemented (T-00005) but superseded by EKF per ADR-00025
- Pain points: DCM lacks explicit covariance, separate bias compensation needed
- Constraints: no_std, no heap, Embassy async, RP2040 no FPU, RP2350 with FPU
- Related ADRs: [ADR-00025-ekf-ahrs-implementation](../../adr/ADR-00025-ekf-ahrs-implementation.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                     AHRS Subsystem                              │
│                                                                 │
│  ┌─────────────┐     ┌──────────────────────────────────────┐   │
│  │ IMU Driver  │────▶│           AhrsEkf                    │   │
│  │ (ImuSensor) │     │                                      │   │
│  │  @400Hz     │     │  State: x = [q0,q1,q2,q3,bx,by,bz]   │   │
│  └─────────────┘     │                                      │   │
│                      │  ┌────────────┐  ┌─────────────────┐ │   │
│                      │  │ Prediction │  │    Updates      │ │   │
│                      │  │   @400Hz   │  │ Accel: @100Hz   │ │   │
│                      │  │   (gyro)   │  │ Mag:   @10Hz    │ │   │
│                      │  └────────────┘  └─────────────────┘ │   │
│                      │                                      │   │
│                      │  Covariance: P [7x7]                 │   │
│                      └──────────────┬───────────────────────┘   │
│                                     │                           │
│                                     ▼                           │
│                      ┌──────────────────────────────────────┐   │
│                      │         ATTITUDE_STATE               │   │
│                      │  (Mutex<CriticalSectionRawMutex>)    │   │
│                      │                                      │   │
│                      │  - quaternion, euler angles          │   │
│                      │  - angular rates                     │   │
│                      │  - gyro bias estimate                │   │
│                      │  - health status, timestamp          │   │
│                      └──────────────┬───────────────────────┘   │
│                                     │                           │
│              ┌──────────────────────┼──────────────────────┐    │
│              ▼                      ▼                      ▼    │
│   ┌──────────────────┐   ┌──────────────────┐   ┌────────────┐  │
│   │   Navigation     │   │    Control       │   │ Telemetry  │  │
│   │   Controller     │   │    Loops         │   │  (MAVLink) │  │
│   └──────────────────┘   └──────────────────┘   └────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### Components

**`src/subsystems/ahrs/ekf.rs`**:

- `AhrsEkf` struct: 7-state EKF with covariance
- `predict()`: State propagation with gyroscope
- `update_accel()`: Gravity reference correction
- `update_mag()`: Heading reference correction
- `EkfConfig`: Noise parameters and tuning

**`src/subsystems/ahrs/state.rs`**:

- `AttitudeState` struct: Shared attitude data
- `ATTITUDE_STATE` global: Mutex-protected static
- Quaternion-to-Euler conversion

**`src/subsystems/ahrs/quaternion.rs`**:

- Quaternion math utilities
- Rotation operations
- Normalization and derivative

**`src/communication/mavlink/handlers/attitude.rs`**:

- `create_attitude_message()`: ATTITUDE (ID 30)
- `create_attitude_quaternion_message()`: ATTITUDE_QUATERNION (ID 31)

### Data Flow

1. **IMU Read (400Hz)**:
   - IMU driver provides calibrated gyro, accel, mag
   - Each sample triggers EKF prediction

2. **Prediction (400Hz)**:
   - Subtract gyro bias from measurement
   - Compute quaternion derivative
   - Integrate quaternion with first-order Euler
   - Normalize quaternion
   - Propagate covariance: P = F × P × F^T + Q × dt

3. **Accelerometer Update (100Hz)**:
   - Normalize accelerometer to unit vector
   - Predict gravity direction from quaternion
   - Compute innovation (measurement - prediction)
   - Compute Kalman gain
   - Apply state correction
   - Update covariance

4. **Magnetometer Update (10Hz)**:
   - Normalize magnetometer to unit vector
   - Predict magnetic field from quaternion and reference
   - Compute heading error
   - Apply Kalman correction for yaw

5. **State Publication (100Hz)**:
   - Convert quaternion to Euler angles
   - Update global `ATTITUDE_STATE`
   - Include gyro bias and health status

### Data Models and Types

```rust
/// 7-state EKF for AHRS
pub struct AhrsEkf {
    /// State: [q0, q1, q2, q3, bx, by, bz]
    x: [f32; 7],

    /// Covariance matrix P [7x7] stored as flat array
    P: [f32; 49],

    /// Process noise Q diagonal
    Q: [f32; 7],

    /// Accel measurement noise R [3x3]
    R_accel: [f32; 9],

    /// Mag measurement noise R [3x3]
    R_mag: [f32; 9],

    /// Reference magnetic field (NED frame)
    mag_ref: [f32; 3],

    /// Last update timestamp
    last_update_us: u64,
}

/// Global attitude state
#[derive(Clone, Copy, Default)]
pub struct AttitudeState {
    /// Quaternion [w, x, y, z]
    pub quaternion: [f32; 4],

    /// Euler angles (radians)
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,

    /// Angular rates (rad/s)
    pub roll_rate: f32,
    pub pitch_rate: f32,
    pub yaw_rate: f32,

    /// Gyro bias estimate (rad/s)
    pub gyro_bias: [f32; 3],

    /// Covariance diagonal (uncertainty)
    pub covariance: [f32; 7],

    /// EKF health status
    pub healthy: bool,

    /// Timestamp (ms)
    pub timestamp_ms: u32,
}

/// EKF configuration
pub struct EkfConfig {
    /// Quaternion process noise
    pub q_quaternion: f32,
    /// Gyro bias process noise
    pub q_gyro_bias: f32,
    /// Accelerometer measurement noise
    pub r_accel: f32,
    /// Magnetometer measurement noise
    pub r_mag: f32,
    /// Reference magnetic declination (radians)
    pub mag_declination: f32,
}
```

### Error Handling

- **Quaternion denormalization**: Re-normalize after every update
- **Covariance growth**: Bound diagonal elements, reset if too large
- **Innovation outliers**: Reject accel updates during high acceleration
- **EKF divergence**: Monitor trace(P), mark unhealthy if exceeds threshold

### Security Considerations

- N/A - No external input processing

### Performance Considerations

- **Matrix Operations**: Use flat arrays to avoid nalgebra overhead on RP2040
- **FPU Optimization**: Conditional compilation for RP2350 FPU
- **Trig Functions**: Use `micromath` for fast approximations on RP2040
- **Memory**: Fixed buffers, no heap allocation (\~600 bytes for matrices)
- **Update Rate**: 100Hz EKF update, 400Hz prediction

### Platform Considerations

#### RP2040 (Pico W)

- No FPU: Software floating-point for matrix math
- Use `micromath` for fast trig approximations
- May need to reduce update rate to 50Hz if CPU-bound
- Memory: 264KB RAM sufficient (\~8KB for EKF)

#### RP2350 (Pico 2 W)

- Hardware FPU: 3-5x speedup for matrix operations
- 100Hz update easily achievable
- Memory: 520KB RAM provides ample headroom

#### Filesystem

- N/A

## Alternatives Considered

1. **DCM Algorithm (T-00005)**
   - Pros: Already implemented, simpler, lower memory
   - Cons: No explicit covariance, separate bias handling
   - Decision: EKF supersedes DCM for better accuracy

2. **Madgwick/Mahony Filter**
   - Pros: Very simple, fast, low memory
   - Cons: Lower accuracy, no bias estimation
   - Decision: EKF provides required accuracy

3. **Full Navigation EKF (15+ states)**
   - Pros: Position/velocity integration
   - Cons: Too complex for AHRS-only application
   - Decision: Defer to future navigation enhancement

Decision Rationale:

- 7-state EKF provides optimal balance of accuracy and complexity
- Quaternion representation avoids gimbal lock
- Explicit gyro bias estimation essential for long-term stability

## Migration and Compatibility

- Backward compatibility: DCM code kept for reference but not used
- Rollout: EKF becomes primary AHRS; DCM deprecated
- Deprecation: ADR-00001 (AHRS algorithm selection) superseded

## Testing Strategy

### Unit Tests

- Test quaternion math (normalization, derivative, rotation)
- Test Euler conversion (edge cases at ±90° pitch)
- Test prediction step (gyro integration)
- Test update step (gravity/mag correction)
- Test with simulated IMU sequences

### Integration Tests

- Verify attitude convergence with known initial orientation
- Verify gyro bias estimation with synthetic bias
- Verify MAVLink message generation

### External API Parsing (if applicable)

- N/A

### Performance & Benchmarks (if applicable)

- Measure EKF update time on RP2040 and RP2350
- Profile memory usage with embedded profiler

## Documentation Impact

- Update `docs/architecture.md` with AHRS subsystem
- Add EKF tuning guide for calibration

## External References

- [Quaternion EKF for AHRS (MDPI)](https://www.mdpi.com/1424-8220/20/14/4055)
- [ArduPilot EKF Documentation](https://ardupilot.org/dev/docs/extended-kalman-filter.html)
- [Extended Kalman Filter Tutorial (Welch & Bishop)](https://www.cs.unc.edu/~welch/kalman/kalmanIntro.html)

## Open Questions

- [ ] What Q values work best for rover/boat dynamics? → Method: Tune on hardware
- [ ] Should magnetometer update only correct heading? → Decision: Heading only initially
- [ ] Is adaptive process noise needed? → Method: Evaluate fixed noise first

## Appendix

### EKF State Transition Matrix

For quaternion propagation:

```text
F = ∂f/∂x where f is the state transition function

F_qq = I + 0.5 * dt * Ω(ω)  (quaternion block)
F_qb = -0.5 * dt * Ξ(q)     (quaternion-bias coupling)
F_bb = I                     (bias random walk)

where Ω(ω) is the skew-symmetric matrix from angular rate
and Ξ(q) is the quaternion multiplication matrix
```

### Measurement Jacobians

Accelerometer (gravity reference):

```text
H_accel = ∂h/∂x where h predicts gravity in body frame

h(q) = R(q)^T * [0, 0, 1]^T  (gravity in NED rotated to body)
```

Magnetometer (heading reference):

```text
H_mag = ∂h/∂x where h predicts magnetic field in body frame

h(q) = R(q)^T * mag_ref
```

### Default Tuning Parameters

```rust
impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            q_quaternion: 1e-6,     // Quaternion process noise
            q_gyro_bias: 1e-8,      // Gyro bias random walk
            r_accel: 0.5,           // Accel measurement noise (m/s²)²
            r_mag: 0.5,             // Mag measurement noise (µT)²
            mag_declination: 0.0,   // Set per location
        }
    }
}
```
