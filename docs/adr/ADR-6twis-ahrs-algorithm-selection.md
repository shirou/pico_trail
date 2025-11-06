# ADR-6twis AHRS Algorithm Selection: DCM with Optional EKF Upgrade

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-eyuh8-ahrs-attitude-estimation](../requirements/FR-eyuh8-ahrs-attitude-estimation.md)
  - [NFR-z2iuk-memory-limits](../requirements/NFR-z2iuk-memory-limits.md)
  - [NFR-3wlo1-imu-sampling-rate](../requirements/NFR-3wlo1-imu-sampling-rate.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-49k7n-ahrs-dcm-implementation](../tasks/T-49k7n-ahrs-dcm-implementation/README.md)
  - [T-qwvco-bmi088-imu-driver-implementation](../tasks/T-qwvco-bmi088-imu-driver-implementation/README.md)

## Context

The autopilot requires accurate attitude estimation (roll, pitch, heading) for navigation and control. The Attitude and Heading Reference System (AHRS) fuses data from multiple sensors:

- **Gyroscope**: Accurate short-term rotation, but drifts over time
- **Accelerometer**: Gravity reference for roll/pitch, noisy during motion
- **Magnetometer**: Heading reference, susceptible to magnetic interference
- **GPS**: Velocity-based heading aid when moving

### Problem

We need an AHRS algorithm that:

- Provides ±2° roll/pitch accuracy, ±5° heading accuracy
- Runs at 100Hz on resource-constrained microcontrollers
- Works on both Pico W (no FPU, 264 KB RAM) and Pico 2 W (FPU, 520 KB RAM)
- Handles sensor noise and temporary failures gracefully

### Constraints

- **Pico W**: No FPU (software floating-point), limited RAM (264 KB)
- **Pico 2 W**: Hardware FPU, more RAM (520 KB)
- **CPU Budget**: AHRS update must complete within 10ms (100Hz rate)
- **Memory Budget**: < 10 KB RAM for AHRS state

### Prior Art

- **ArduPilot**: Uses DCM or EKF (15-state), configurable per vehicle
- **PX4**: Uses EKF2 (24-state), heavyweight but highly accurate
- **Embedded Systems**: DCM, complementary filter, Madgwick filter, simplified EKF

## Success Metrics

- **Accuracy**: Roll/pitch within ±2°, heading within ±5° (with calibrated mag)
- **Convergence**: Correct attitude within 5 seconds of startup
- **CPU**: AHRS update completes within 10ms (100Hz rate)
- **Memory**: AHRS state < 10 KB RAM
- **Robustness**: Handles temporary sensor failures (e.g., mag interference)

## Decision

**We will implement Direction Cosine Matrix (DCM) as the primary AHRS algorithm, with an optional Extended Kalman Filter (EKF) upgrade for Pico 2 W as a future enhancement.**

### Phased Approach

1. **Phase 1 (Initial)**: Implement DCM for both Pico W and Pico 2 W
   - Proven algorithm, lower complexity, faster to validate
   - Meets accuracy requirements (±2° roll/pitch, ±5° heading)
   - Works well on Pico W (no FPU required)

2. **Phase 2 (Optional)**: Add simplified EKF for Pico 2 W
   - Better accuracy (±1° roll/pitch, ±3° heading)
   - Leverages hardware FPU for matrix operations
   - Compile-time feature flag: `--features ahrs-ekf`

### Decision Drivers

1. **Development Velocity**: DCM is simpler to implement and test than EKF
2. **Pico W Compatibility**: DCM works well without FPU, EKF is CPU-intensive
3. **Memory Efficiency**: DCM uses \~2 KB RAM vs EKF \~8 KB RAM
4. **Proven Track Record**: DCM is battle-tested in ArduPilot for 10+ years
5. **Phased Risk**: Start with simpler DCM, add EKF if accuracy insufficient

### Considered Options

- **Option A: DCM (Direction Cosine Matrix)** ⭐ Selected for Phase 1
- **Option B: Simplified EKF (6-9 states)** - Phase 2 upgrade
- **Option C: Madgwick/Mahony Filter** - Considered but rejected

### Option Analysis

**Option A: DCM (Direction Cosine Matrix)**

- **Pros**:
  - Simple, well-understood algorithm
  - Low memory footprint (\~2 KB RAM)
  - Works efficiently without FPU
  - Proven in ArduPilot (10+ years production use)
  - Fast implementation and testing
- **Cons**:
  - Lower accuracy than EKF (±2-3° vs ±1-2°)
  - Less robust to sensor noise
  - No explicit covariance estimation
- **Estimated Performance**: ±2° roll/pitch, ±5° heading, \~5ms update time

**Option B: Simplified EKF (6-9 states)**

- **Pros**:
  - Better accuracy (±1-2° roll/pitch, ±3-5° heading)
  - Explicit covariance and sensor uncertainty modeling
  - More robust to sensor noise
  - FPU accelerates matrix operations on Pico 2 W
- **Cons**:
  - Higher complexity (slower to implement and validate)
  - More memory (\~8 KB RAM for state and covariance)
  - Requires FPU for acceptable performance (not ideal for Pico W)
  - Higher CPU usage (\~8ms update time on Pico 2 W)
- **Estimated Performance**: ±1° roll/pitch, ±3° heading, \~8ms update time (with FPU)

**Option C: Madgwick/Mahony Filter**

- **Pros**:
  - Very simple complementary filter
  - Minimal memory (\~1 KB RAM)
  - Fast execution (\~2ms update time)
- **Cons**:
  - Lower accuracy (±3-5° roll/pitch, ±10° heading)
  - No magnetometer hard/soft iron correction
  - Limited to IMU + mag fusion (no GPS velocity integration)
  - Not industry-standard (less ArduPilot/Pixhawk compatibility)
- **Decision**: Rejected - accuracy insufficient for autonomous navigation

## Rationale

DCM was chosen over EKF and Madgwick for the following reasons:

1. **Development Velocity**: DCM is simpler than EKF, allowing faster initial implementation and validation. We can validate navigation and control with DCM, then upgrade to EKF if needed.

2. **Pico W Compatibility**: DCM performs acceptably without an FPU, making it suitable for both Pico W and Pico 2 W. EKF would be too slow on Pico W's Cortex-M0+.

3. **Proven Track Record**: DCM has been used successfully in ArduPilot for over 10 years, demonstrating it meets accuracy requirements for rovers and boats.

4. **Memory Efficiency**: DCM uses \~2 KB RAM vs EKF's \~8 KB, leaving more headroom for other subsystems.

5. **Phased Risk**: Starting with DCM reduces implementation risk. If accuracy is insufficient, we can add EKF as a Phase 2 upgrade for Pico 2 W.

### Why Not EKF Immediately

EKF provides better accuracy (±1° vs ±2°) but:

- Higher implementation complexity (more time to validate)
- Requires FPU for acceptable performance (not ideal for Pico W)
- Uses 4x more RAM than DCM (\~8 KB vs \~2 KB)
- The additional accuracy may not be necessary for rover/boat navigation

**Decision**: We prioritize faster time-to-working-system with DCM. If field testing reveals DCM accuracy is insufficient, we can implement EKF as a compile-time feature for Pico 2 W.

## Consequences

### Positive

- **Faster Development**: DCM is simpler to implement and test than EKF
- **Pico W Support**: DCM works well without FPU, enabling full feature parity on Pico W
- **Memory Efficient**: \~2 KB RAM leaves more headroom for other features
- **Proven Reliability**: DCM has 10+ years production use in ArduPilot
- **Future Upgrade Path**: Can add EKF later if needed without breaking existing code

### Negative

- **Lower Accuracy**: DCM provides ±2-3° accuracy vs EKF's ±1-2° (may be acceptable for rovers/boats)
- **Less Robust to Noise**: DCM has less sophisticated noise handling than EKF
- **No Covariance**: DCM doesn't provide uncertainty estimates (useful for sensor fusion)

### Neutral

- **ArduPilot Compatibility**: Both DCM and EKF are used in ArduPilot, so either choice is compatible

## Implementation Notes

### DCM Algorithm Structure

```rust
pub struct Dcm {
    dcm_matrix: Matrix3<f32>,     // Direction cosine matrix
    gyro_bias: Vector3<f32>,      // Gyro bias estimate
    omega_p: Vector3<f32>,        // Proportional correction
    omega_i: Vector3<f32>,        // Integral correction
    kp_roll_pitch: f32,           // P gain for roll/pitch
    ki_roll_pitch: f32,           // I gain for roll/pitch
}

impl Dcm {
    pub fn update(&mut self, gyro: Vector3<f32>, accel: Vector3<f32>, dt: f32) {
        // 1. Integrate gyro to update DCM matrix
        let omega = gyro - self.gyro_bias + self.omega_p + self.omega_i;
        self.dcm_matrix = self.dcm_matrix * rotation_matrix(omega, dt);

        // 2. Normalize DCM matrix (prevent drift)
        self.dcm_matrix = orthonormalize(self.dcm_matrix);

        // 3. Error correction from accelerometer
        let accel_ref = self.dcm_matrix * Vector3::new(0.0, 0.0, 1.0);
        let error = accel.cross(&accel_ref);

        // 4. Proportional and integral corrections
        self.omega_p = error * self.kp_roll_pitch;
        self.omega_i += error * self.ki_roll_pitch * dt;
    }

    pub fn get_euler_angles(&self) -> (f32, f32, f32) {
        // Extract roll, pitch, yaw from DCM matrix
        let roll = self.dcm_matrix[(2, 1)].atan2(self.dcm_matrix[(2, 2)]);
        let pitch = (-self.dcm_matrix[(2, 0)]).asin();
        let yaw = self.dcm_matrix[(1, 0)].atan2(self.dcm_matrix[(0, 0)]);
        (roll, pitch, yaw)
    }
}
```

### Magnetometer Integration

```rust
pub fn update_with_mag(&mut self, mag: Vector3<f32>) {
    // Correct heading using magnetometer
    let mag_body = self.dcm_matrix * mag;
    let mag_heading = mag_body.y.atan2(mag_body.x);

    let (_, _, yaw) = self.get_euler_angles();
    let heading_error = normalize_angle(mag_heading - yaw);

    // Apply heading correction
    self.omega_p.z += heading_error * self.kp_yaw;
    self.omega_i.z += heading_error * self.ki_yaw * dt;
}
```

### Future EKF Upgrade (Phase 2)

```rust
// Compile-time feature flag
#[cfg(feature = "ahrs-ekf")]
pub type Ahrs = SimplifiedEkf;

#[cfg(not(feature = "ahrs-ekf"))]
pub type Ahrs = Dcm;
```

## Platform Considerations

- **Pico W (RP2040)**: Use DCM (software float math acceptable at 100Hz)
- **Pico 2 W (RP2350)**: Use DCM initially, optionally enable EKF with `--features ahrs-ekf`
- **Cross-Platform**: Both DCM and EKF abstracted behind `Ahrs` trait

## Open Questions

- [ ] Is DCM accuracy (±2-3°) sufficient for waypoint navigation? → Method: Field test with DCM, measure cross-track error
- [ ] Should we implement EKF immediately or defer to Phase 2? → Decision: Implement DCM first, add EKF only if needed
- [ ] Can we optimize DCM for better accuracy without moving to full EKF? → Method: Experiment with gain tuning and filtering

## External References

- ArduPilot DCM Implementation: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_AHRS>
- DCM Tutorial: <http://www.starlino.com/dcm_tutorial.html>
- Simplified EKF for AHRS: <https://www.mdpi.com/1424-8220/15/8/19302>

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
