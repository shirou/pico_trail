# FR-3f2cn Quaternion EKF AHRS

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-eyuh8-ahrs-attitude-estimation](FR-eyuh8-ahrs-attitude-estimation.md)
  - [FR-z1fdo-imu-sensor-trait](FR-z1fdo-imu-sensor-trait.md)
  - [FR-oqxl8-mpu9250-i2c-driver](FR-oqxl8-mpu9250-i2c-driver.md)
  - [FR-soukr-imu-calibration-interface](FR-soukr-imu-calibration-interface.md)
  - [NFR-3wlo1-imu-sampling-rate](NFR-3wlo1-imu-sampling-rate.md)
- Dependent Requirements:
  - [FR-1sel5-ekf-gyro-bias-estimation](FR-1sel5-ekf-gyro-bias-estimation.md)
  - [FR-0azz0-attitude-state-interface](FR-0azz0-attitude-state-interface.md)
  - [NFR-ax9yx-ekf-update-performance](NFR-ax9yx-ekf-update-performance.md)
  - [NFR-35otr-ekf-memory-overhead](NFR-35otr-ekf-memory-overhead.md)
- Related ADRs:
  - [ADR-ymkzt-ekf-ahrs-implementation](../adr/ADR-ymkzt-ekf-ahrs-implementation.md)
- Related Tasks:
  - [T-p8w8f-ekf-ahrs-implementation](../tasks/T-p8w8f-ekf-ahrs-implementation/README.md)

## Requirement Statement

The system shall implement a 7-state quaternion-based Extended Kalman Filter (EKF) for attitude estimation, using quaternion representation for orientation and including gyroscope bias as estimated states.

## Rationale

A 7-state quaternion EKF provides optimal attitude estimation:

- **Quaternion (4 states)**: Avoids gimbal lock at ±90° pitch inherent in Euler angles
- **Gyro Bias (3 states)**: Explicitly estimates gyro drift for long-term stability
- **Covariance Matrix**: Provides uncertainty estimates for health monitoring
- **Optimal Fusion**: Kalman filter optimally weights sensor measurements by uncertainty
- **ArduPilot Alignment**: ArduPilot uses EKF for all modern vehicles

The 7-state formulation balances accuracy and computational cost, avoiding the complexity of full 15+ state navigation EKF while providing superior accuracy to simpler complementary filters.

## User Story (if applicable)

As an autopilot system, I want accurate attitude estimation via quaternion EKF, so that navigation and control algorithms receive reliable orientation data free from gimbal lock and gyro drift.

## Acceptance Criteria

- [ ] EKF state vector contains 7 elements: quaternion (4) + gyro bias (3)
- [ ] Quaternion maintained as unit quaternion (normalized after each update)
- [ ] Prediction step runs at 400Hz using gyroscope data
- [ ] Accelerometer update runs at 100Hz (gravity reference)
- [ ] Magnetometer update runs at 10Hz (heading reference)
- [ ] State covariance matrix (7x7) maintained for uncertainty estimation
- [ ] Roll/pitch accuracy within ±2° under static conditions
- [ ] Heading accuracy within ±5° with calibrated magnetometer
- [ ] Convergence to correct attitude within 5 seconds of startup
- [ ] No gimbal lock at any attitude (tested at ±85° pitch)

## Technical Details (if applicable)

### Functional Requirement Details

**State Vector:**

```rust
/// 7-state EKF for AHRS
/// x = [q0, q1, q2, q3, bx, by, bz]
/// - q0..q3: Unit quaternion (w, x, y, z) representing attitude
/// - bx..bz: Gyroscope bias in rad/s (body frame)
pub struct AhrsEkf {
    /// State vector [7]
    x: Vector7<f32>,

    /// State covariance matrix [7x7]
    P: Matrix7<f32>,

    /// Process noise covariance [7x7]
    Q: Matrix7<f32>,

    /// Accelerometer measurement noise [3x3]
    R_accel: Matrix3<f32>,

    /// Magnetometer measurement noise [3x3]
    R_mag: Matrix3<f32>,

    /// Reference magnetic field vector (NED frame)
    mag_ref: Vector3<f32>,
}
```

**Update Rates:**

| Operation         | Rate  | Samples      | Purpose                              |
| ----------------- | ----- | ------------ | ------------------------------------ |
| Prediction (gyro) | 400Hz | Every sample | Propagate attitude with angular rate |
| Accel update      | 100Hz | Every 4th    | Correct roll/pitch with gravity      |
| Mag update        | 10Hz  | Every 40th   | Correct heading with magnetic field  |

**Prediction Step (400Hz):**

```rust
pub fn predict(&mut self, gyro: Vector3<f32>, dt: f32) {
    // Bias-corrected angular rate
    let omega = gyro - self.gyro_bias();

    // Quaternion derivative: q_dot = 0.5 * q ⊗ [0, omega]
    let q = self.quaternion();
    let q_dot = q.derivative(omega);

    // First-order integration
    let q_new = (q + q_dot * dt).normalize();
    self.set_quaternion(q_new);

    // Propagate covariance: P = F * P * F^T + Q * dt
    let F = self.state_jacobian(omega, dt);
    self.P = F * self.P * F.transpose() + self.Q * dt;
}
```

**Accelerometer Update (100Hz):**

Uses accelerometer as gravity reference for roll/pitch correction:

1. Normalize accelerometer reading (assume stationary = gravity only)
2. Predict gravity direction in body frame from current quaternion
3. Compute innovation (measured - predicted)
4. Apply Kalman update

**Magnetometer Update (10Hz):**

Uses magnetometer for heading correction:

1. Normalize magnetometer reading
2. Predict magnetic field in body frame from quaternion and reference
3. Compute heading error (horizontal plane only)
4. Apply Kalman update

**Quaternion Normalization:**

After each update, quaternion must be re-normalized:

```rust
fn normalize_quaternion(&mut self) {
    let q = self.quaternion();
    let norm = q.norm();
    if norm > 0.0 {
        self.set_quaternion(q / norm);
    }
}
```

## Platform Considerations

### Pico W (RP2040)

- No FPU: Software floating-point for matrix operations
- Use `micromath` for fast trig approximations
- May need to reduce update rate to 50Hz if CPU-bound

### Pico 2 W (RP2350)

- Hardware FPU: 3-5x speedup for matrix operations
- 100Hz update easily achievable
- Can use full precision trigonometry

### Cross-Platform

- Conditional compilation for FPU optimizations
- Same algorithm on both platforms
- Use `nalgebra` with `no_std` feature for matrix math

## Risks & Mitigation

| Risk                        | Impact | Likelihood | Mitigation                                   | Validation                          |
| --------------------------- | ------ | ---------- | -------------------------------------------- | ----------------------------------- |
| EKF divergence              | High   | Low        | Monitor covariance trace, reset if too large | Test 1-hour continuous operation    |
| Quaternion denormalization  | High   | Low        | Normalize after every update                 | Assert quaternion norm ≈ 1.0        |
| Gimbal lock near ±90° pitch | High   | Low        | Quaternion inherently avoids this            | Test at extreme attitudes           |
| Poor convergence            | Medium | Medium     | Tune initial covariance and process noise    | Measure convergence time on startup |

## Implementation Notes

**Module Location:**

```
src/subsystems/ahrs/
├── mod.rs              # Public exports
├── ekf.rs              # AhrsEkf implementation
├── quaternion.rs       # Quaternion math utilities
└── state.rs            # AttitudeState definition
```

**Initial State:**

```rust
impl AhrsEkf {
    pub fn new(config: EkfConfig) -> Self {
        Self {
            // Initial quaternion: identity (level, north-facing)
            x: Vector7::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),

            // Large initial covariance (uncertain)
            P: Matrix7::from_diagonal(&Vector7::new(
                0.1, 0.1, 0.1, 0.1,  // quaternion
                0.01, 0.01, 0.01,    // gyro bias
            )),

            Q: config.process_noise,
            R_accel: config.accel_noise,
            R_mag: config.mag_noise,
            mag_ref: config.mag_reference,
        }
    }
}
```

**Process Noise Tuning:**

Process noise Q balances trust in gyro vs corrections:

- Higher Q: Faster response, more noise
- Lower Q: Smoother, slower response

Typical starting values:

- Quaternion: 1e-6 to 1e-4 (per sample)
- Gyro bias: 1e-8 to 1e-6 (slow drift)

## External References

- [Quaternion EKF for AHRS (MDPI)](https://www.mdpi.com/1424-8220/20/14/4055)
- [ArduPilot EKF Documentation](https://ardupilot.org/dev/docs/extended-kalman-filter.html)
- [Extended Kalman Filter Tutorial](https://www.cs.unc.edu/~welch/kalman/kalmanIntro.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
