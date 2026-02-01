# ADR-00025 EKF AHRS Implementation

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-00001-ahrs-attitude-estimation](../requirements/FR-00001-ahrs-attitude-estimation.md)
  - [NFR-00002-imu-sampling-rate](../requirements/NFR-00002-imu-sampling-rate.md)
  - [FR-00104-quaternion-ekf-ahrs](../requirements/FR-00104-quaternion-ekf-ahrs.md)
  - [FR-00099-ekf-gyro-bias-estimation](../requirements/FR-00099-ekf-gyro-bias-estimation.md)
  - [FR-00098-attitude-state-interface](../requirements/FR-00098-attitude-state-interface.md)
  - [NFR-00074-ekf-update-performance](../requirements/NFR-00074-ekf-update-performance.md)
  - [NFR-00073-ekf-memory-overhead](../requirements/NFR-00073-ekf-memory-overhead.md)
- Supersedes ADRs:
  - [ADR-00001-ahrs-algorithm-selection](../adr/ADR-00001-ahrs-algorithm-selection.md)
- Related Analyses:
  - [AN-00027-mpu9250-imu-and-ekf-integration](../analysis/AN-00027-mpu9250-imu-and-ekf-integration.md)
- Related ADRs:
  - [ADR-00026-mpu9250-i2c-driver-architecture](../adr/ADR-00026-mpu9250-i2c-driver-architecture.md)
- Related Tasks:
  - [T-00022-ekf-ahrs-implementation](../tasks/T-00022-ekf-ahrs-implementation/README.md)
  - [T-00032-ahrs-abstraction-layer](../tasks/T-00032-ahrs-abstraction-layer/README.md)

## Context

The pico_trail autopilot requires accurate attitude estimation (roll, pitch, yaw) for navigation and control. The previous ADR (ADR-00001) selected DCM as the primary algorithm with EKF as an optional upgrade. This decision supersedes that approach, selecting EKF as the primary implementation.

**Problem Statement:**

- Provide accurate attitude estimation (±2° roll/pitch, ±5° heading)
- Fuse data from gyroscope, accelerometer, and magnetometer
- Estimate and compensate gyro bias for long-term stability
- Provide quaternion representation to avoid gimbal lock
- Share attitude state with navigation, control, and telemetry subsystems

**Constraints:**

- RP2040: No FPU, limited RAM (264KB)
- RP2350: Hardware FPU, more RAM (520KB)
- Must run at 100Hz minimum (10ms update period)
- Memory budget: < 10KB for EKF state and covariance
- Embassy async runtime

**Assumptions:**

- IMU provides calibrated 9-axis data at 400Hz
- EKF update runs at 100Hz (every 4th IMU sample)
- Magnetometer fusion at 10Hz (every 10th EKF update)
- GPS velocity available for heading aid (future enhancement)

**Forces in Tension:**

1. **Accuracy vs Complexity**: EKF more accurate but complex; DCM simpler but less accurate
2. **Memory vs Performance**: Full covariance matrix uses more RAM
3. **FPU Dependency**: EKF benefits from FPU but must work without it

## Success Metrics

- **Accuracy**: Roll/pitch within ±2°, heading within ±5° (static conditions)
- **Convergence**: Correct attitude within 5 seconds of startup
- **Update Rate**: EKF completes 100Hz update in < 10ms on RP2040
- **Memory**: EKF state + covariance < 10KB RAM
- **Stability**: No divergence during 1-hour operation
- **Gyro Bias**: Estimated bias converges within 2% of true value

## Decision

**We will implement a 7-state quaternion-based Extended Kalman Filter (EKF) for attitude estimation, replacing the DCM algorithm as the primary AHRS implementation.**

### Core Decisions

1. **Algorithm**: 7-state EKF with quaternion attitude
   - State: \[q0, q1, q2, q3, bx, by, bz] (quaternion + gyro bias)
   - Avoids gimbal lock inherent in Euler angle representations

2. **Update Rates**:
   - Prediction: 400Hz (every IMU sample)
   - Accel correction: 100Hz (every 4th sample)
   - Mag correction: 10Hz (every 40th sample)

3. **State Management**: Global `ATTITUDE_STATE`
   - Follows GPS state pattern for consistency
   - Protected by critical section mutex
   - Accessible by navigation, control, telemetry

4. **MAVLink Integration**:
   - Send `ATTITUDE` message at 10Hz
   - Send `ATTITUDE_QUATERNION` at 10Hz (optional)

5. **DCM Fallback**: Remove DCM as primary; keep as reference only
   - EKF provides superior accuracy and explicit uncertainty
   - DCM code may remain for comparison testing

### Decision Drivers

1. **Accuracy**: EKF provides ±1-2° accuracy vs DCM's ±2-3°
2. **Gyro Bias Estimation**: EKF explicitly estimates gyro bias
3. **Covariance**: EKF provides uncertainty estimates for health monitoring
4. **Quaternion**: Avoids gimbal lock at extreme attitudes
5. **ArduPilot Alignment**: ArduPilot uses EKF for all modern vehicles

### Considered Options

**Option A: 7-State Quaternion EKF** (Selected)

- States: Quaternion (4) + Gyro bias (3)
- Pros: Explicit bias estimation, uncertainty, no gimbal lock
- Cons: Higher complexity, more memory (\~8KB)

**Option B: DCM with PI Corrections** (Previous decision)

- Pros: Simpler, lower memory (\~2KB), works well without FPU
- Cons: No explicit covariance, separate bias compensation needed

**Option C: Madgwick/Mahony Filter**

- Pros: Very simple, fast, low memory
- Cons: Lower accuracy, no bias estimation, limited sensor fusion

**Option D: 15+ State Navigation EKF**

- States: Quaternion + Gyro bias + Position + Velocity + Accel bias
- Pros: Full navigation solution
- Cons: Too complex for AHRS-only, excessive memory/CPU

### Option Analysis

- **Option A** — Best accuracy and explicit uncertainty | Higher complexity acceptable on modern MCUs
- **Option B** — Proven but less accurate | DCM already implemented but superseded
- **Option C** — Too simplistic for autonomous navigation | Insufficient accuracy
- **Option D** — Overkill for AHRS-only | Defer to navigation integration

## Rationale

**Why EKF over DCM?**

1. **Gyro Bias Estimation**: EKF explicitly estimates and compensates gyro bias as part of the filter state. DCM requires separate bias estimation logic.

2. **Covariance/Uncertainty**: EKF maintains covariance matrix, enabling health monitoring and sensor fusion quality assessment.

3. **Quaternion Representation**: Quaternions avoid gimbal lock at ±90° pitch, which is critical for accurate attitude in all orientations.

4. **Optimal Fusion**: EKF provides mathematically optimal sensor fusion weighted by measurement uncertainty.

5. **Industry Standard**: ArduPilot, PX4, and most modern autopilots use EKF-based AHRS.

**Why 7 States?**

- **Quaternion (4)**: Minimum representation for attitude without gimbal lock
- **Gyro Bias (3)**: Essential for long-term stability (gyros drift over time)
- **Not Accel Bias**: Accelerometer bias less critical; can be calibrated offline
- **Not Mag Bias**: Magnetometer calibration handles hard/soft iron

**Trade-offs Accepted:**

- **Complexity**: EKF more complex than DCM but well-understood algorithm
- **Memory**: \~8KB vs \~2KB, acceptable on both RP2040 and RP2350
- **CPU**: EKF heavier than DCM but achievable at 100Hz even without FPU

## Consequences

### Positive

- **Better Accuracy**: ±1-2° roll/pitch vs DCM's ±2-3°
- **Gyro Bias Tracking**: Automatic compensation for temperature drift
- **Health Monitoring**: Covariance diagonal indicates filter health
- **No Gimbal Lock**: Quaternion representation stable at all attitudes
- **Uncertainty Estimates**: Can weight attitude quality in navigation
- **MAVLink Compatible**: Standard attitude messages work directly

### Negative

- **Higher Complexity**: EKF implementation more complex than DCM
- **More Memory**: \~8KB vs \~2KB for DCM
- **CPU Intensive**: Matrix operations heavier than DCM
- **Tuning Required**: Process/measurement noise covariances need tuning
- **FPU Benefit**: Runs slower on RP2040 without FPU

### Neutral

- **DCM Superseded**: Existing DCM code may be kept for reference
- **Same IMU Interface**: Both EKF and DCM use same `ImuSensor` trait

## Implementation Notes

### EKF State Definition

```rust
/// 7-state EKF for AHRS
pub struct AhrsEkf {
    /// State vector: [q0, q1, q2, q3, bx, by, bz]
    /// - q0..q3: Unit quaternion (attitude)
    /// - bx..bz: Gyro bias (rad/s)
    x: Vector7<f32>,

    /// State covariance matrix (7x7)
    P: Matrix7<f32>,

    /// Process noise covariance
    Q: Matrix7<f32>,

    /// Accelerometer measurement noise
    R_accel: Matrix3<f32>,

    /// Magnetometer measurement noise
    R_mag: Matrix3<f32>,

    /// Reference magnetic field vector (NED frame)
    mag_ref: Vector3<f32>,

    /// Timestamp of last update
    last_update_us: u64,
}
```

### Prediction Step (400Hz)

```rust
impl AhrsEkf {
    /// Propagate state with gyroscope measurement
    pub fn predict(&mut self, gyro: Vector3<f32>, dt: f32) {
        // Get current quaternion and bias
        let q = self.quaternion();
        let bias = self.gyro_bias();

        // Corrected angular rate
        let omega = gyro - bias;

        // Quaternion derivative: q_dot = 0.5 * q ⊗ [0, omega]
        let q_dot = 0.5 * q * Quaternion::from_imag(omega);

        // Integrate quaternion
        let q_new = (q + q_dot * dt).normalize();

        // Update state
        self.set_quaternion(q_new);

        // Propagate covariance: P = F * P * F^T + Q
        let F = self.state_transition_jacobian(omega, dt);
        self.P = F * self.P * F.transpose() + self.Q * dt;
    }
}
```

### Accelerometer Update (100Hz)

```rust
impl AhrsEkf {
    /// Update with accelerometer measurement (gravity reference)
    pub fn update_accel(&mut self, accel: Vector3<f32>) {
        // Normalize accelerometer (assume stationary -> gravity only)
        let accel_norm = accel.normalize();

        // Predicted gravity in body frame from current quaternion
        let q = self.quaternion();
        let g_pred = q.inverse_transform_vector(&Vector3::z());

        // Innovation (measurement - prediction)
        let y = accel_norm - g_pred;

        // Measurement Jacobian H (3x7)
        let H = self.accel_measurement_jacobian();

        // Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
        let S = H * self.P * H.transpose() + self.R_accel;
        let K = self.P * H.transpose() * S.try_inverse().unwrap_or(Matrix3::identity());

        // State update
        let dx = K * y;
        self.apply_state_correction(dx);

        // Covariance update: P = (I - K * H) * P
        let I_KH = Matrix7::identity() - K * H;
        self.P = I_KH * self.P;

        // Re-normalize quaternion
        self.normalize_quaternion();
    }
}
```

### Magnetometer Update (10Hz)

```rust
impl AhrsEkf {
    /// Update with magnetometer measurement (heading reference)
    pub fn update_mag(&mut self, mag: Vector3<f32>) {
        // Normalize magnetometer reading
        let mag_norm = mag.normalize();

        // Predicted magnetic field in body frame
        let q = self.quaternion();
        let mag_pred = q.inverse_transform_vector(&self.mag_ref);

        // Only use horizontal component for heading correction
        // Project to horizontal plane in body frame
        let y = heading_error(mag_norm, mag_pred);

        // Measurement Jacobian (simplified for heading only)
        let H = self.mag_measurement_jacobian();

        // Kalman gain and update (similar to accel)
        let S = H * self.P * H.transpose() + self.R_mag;
        let K = self.P * H.transpose() * S.try_inverse().unwrap_or(Matrix3::identity());

        let dx = K * y;
        self.apply_state_correction(dx);

        self.P = (Matrix7::identity() - K * H) * self.P;
        self.normalize_quaternion();
    }
}
```

### Global Attitude State

```rust
/// Global attitude state accessible by all subsystems
pub struct AttitudeState {
    /// Quaternion representation
    pub quaternion: Quaternion<f32>,

    /// Euler angles (roll, pitch, yaw in radians)
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,

    /// Angular rates from gyro (rad/s)
    pub roll_rate: f32,
    pub pitch_rate: f32,
    pub yaw_rate: f32,

    /// Estimated gyro bias (rad/s)
    pub gyro_bias: Vector3<f32>,

    /// EKF covariance diagonal (uncertainty)
    pub covariance: [f32; 7],

    /// Filter health status
    pub healthy: bool,

    /// Timestamp of last update (ms)
    pub timestamp_ms: u32,
}

pub static ATTITUDE_STATE: Mutex<CriticalSectionRawMutex, AttitudeState> =
    Mutex::new(AttitudeState::new());
```

### MAVLink Integration

```rust
/// Generate ATTITUDE message (ID 30)
pub fn create_attitude_message(state: &AttitudeState, boot_time_ms: u32) -> MavlinkMessage {
    mavlink::common::ATTITUDE {
        time_boot_ms: boot_time_ms,
        roll: state.roll,
        pitch: state.pitch,
        yaw: state.yaw,
        rollspeed: state.roll_rate,
        pitchspeed: state.pitch_rate,
        yawspeed: state.yaw_rate,
    }
}

/// Generate ATTITUDE_QUATERNION message (ID 31)
pub fn create_attitude_quaternion_message(
    state: &AttitudeState,
    boot_time_ms: u32
) -> MavlinkMessage {
    let q = state.quaternion;
    mavlink::common::ATTITUDE_QUATERNION {
        time_boot_ms: boot_time_ms,
        q1: q.w,
        q2: q.i,
        q3: q.j,
        q4: q.k,
        rollspeed: state.roll_rate,
        pitchspeed: state.pitch_rate,
        yawspeed: state.yaw_rate,
        repr_offset_q: [0.0, 0.0, 0.0, 0.0], // No offset
    }
}
```

### EKF Task

```rust
#[embassy_executor::task]
pub async fn ekf_task(mut imu: impl ImuSensor) {
    let mut ekf = AhrsEkf::new(EkfConfig::default());
    let mut sample_count: u32 = 0;
    let mut ticker = Ticker::every(Duration::from_micros(2500)); // 400Hz

    loop {
        ticker.next().await;

        // Read IMU data
        let reading = match imu.read_all().await {
            Ok(r) => r,
            Err(e) => {
                log_warn!("IMU read error: {:?}", e);
                continue;
            }
        };

        // Prediction step (every sample, 400Hz)
        let dt = 0.0025; // 2.5ms
        ekf.predict(reading.gyro, dt);

        sample_count += 1;

        // Accelerometer update (every 4th sample, 100Hz)
        if sample_count % 4 == 0 {
            ekf.update_accel(reading.accel);
        }

        // Magnetometer update (every 40th sample, 10Hz)
        if sample_count % 40 == 0 {
            ekf.update_mag(reading.mag);
        }

        // Update global state (every 4th sample, 100Hz)
        if sample_count % 4 == 0 {
            let state = ekf.get_attitude_state(reading.gyro);
            ATTITUDE_STATE.lock().await.update(state);
        }
    }
}
```

## Platform Considerations

### RP2040 (Pico W)

- **No FPU**: Software floating-point for matrix operations
- **Optimization**: Use `micromath` for fast trig approximations
- **Memory**: 264KB RAM sufficient for 7-state EKF (\~8KB)
- **CPU**: May need to reduce to 50Hz update if CPU-bound

### RP2350 (Pico 2 W)

- **Hardware FPU**: Faster matrix operations (3-5x speedup)
- **Memory**: 520KB RAM provides ample headroom
- **CPU**: 100Hz update easily achievable

### Cross-Platform

- Conditional compilation for FPU optimizations
- Same EKF algorithm on both platforms
- Platform-specific tuning if needed

## Monitoring & Logging

```rust
// ERROR: Critical failures
log_error!("EKF divergence detected: trace(P)={}", trace);

// WARN: Health issues
log_warn!("EKF covariance growing: P[0,0]={}", p00);
log_warn!("Accelerometer rejected: |a|={} m/s²", accel_mag);

// INFO: Operational events
log_info!("EKF initialized, converging...");
log_info!("EKF converged: roll={:.1}° pitch={:.1}°", roll_deg, pitch_deg);

// DEBUG: Detailed diagnostics
log_debug!("EKF update: dt={}ms, bias=[{:.4}, {:.4}, {:.4}]", dt_ms, bx, by, bz);

// TRACE: High-volume data
log_trace!("EKF state: q=[{:.3}, {:.3}, {:.3}, {:.3}]", q0, q1, q2, q3);
```

## Open Questions

- [ ] What process noise (Q) values work best for rover/boat dynamics? → Method: Tune on hardware
- [ ] Should we add GPS velocity fusion for heading when moving? → Decision: Defer to navigation task
- [ ] How to detect and handle EKF divergence? → Method: Monitor trace(P), reset if too large
- [ ] Should magnetometer update only correct heading or full attitude? → Decision: Heading only initially
- [ ] Is adaptive process noise needed for varying dynamics? → Method: Evaluate fixed noise first

## External References

- [Extended Kalman Filter Tutorial](https://www.cs.unc.edu/~welch/kalman/kalmanIntro.html)
- [Quaternion EKF for AHRS (MDPI)](https://www.mdpi.com/1424-8220/20/14/4055)
- [ArduPilot EKF Documentation](https://ardupilot.org/dev/docs/extended-kalman-filter.html)
- [STM32 MPU9250 EKF Implementation](https://github.com/suhetao/stm32f4_mpu9250)
- [Madgwick AHRS Paper](http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
