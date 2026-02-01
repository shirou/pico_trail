# FR-00099 EKF Gyro Bias Estimation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00104-quaternion-ekf-ahrs](FR-00104-quaternion-ekf-ahrs.md)
- Dependent Requirements: None
- Related ADRs:
  - [ADR-00025-ekf-ahrs-implementation](../adr/ADR-00025-ekf-ahrs-implementation.md)
- Related Tasks:
  - [T-00022-ekf-ahrs-implementation](../tasks/T-00022-ekf-ahrs-implementation/README.md)

## Requirement Statement

The EKF shall continuously estimate gyroscope bias as part of the filter state, automatically compensating for gyro drift due to temperature changes and sensor aging.

## Rationale

Gyroscope bias is a major source of attitude error:

- **MEMS Gyros**: Exhibit bias that changes with temperature (\~0.03°/s/°C)
- **Power Cycling**: Bias changes on each power-up
- **Long-term Drift**: Bias drifts slowly over time
- **Integration Error**: Uncompensated bias causes roll/pitch/yaw drift

By including gyro bias in the EKF state vector, the filter continuously estimates and compensates for bias, providing stable long-term attitude estimation without manual recalibration.

## User Story (if applicable)

As an autopilot system, I want gyro bias to be automatically estimated and compensated, so that attitude estimation remains accurate over hours of operation without manual recalibration.

## Acceptance Criteria

- [ ] Gyro bias estimated as 3 states (bx, by, bz) in EKF state vector
- [ ] Bias subtracted from gyro measurements in prediction step
- [ ] Bias estimate converges within 2% of true value within 60 seconds
- [ ] Bias estimate tracks temperature-induced drift
- [ ] Bias estimate accessible via `get_gyro_bias()` method
- [ ] Bias estimate included in attitude state for telemetry
- [ ] No attitude drift > 1°/hour due to uncompensated bias

## Technical Details (if applicable)

### Functional Requirement Details

**Bias in State Vector:**

```rust
/// State vector: x = [q0, q1, q2, q3, bx, by, bz]
///                    └── quaternion ──┘ └─ gyro bias ─┘
impl AhrsEkf {
    /// Get current gyro bias estimate (rad/s)
    pub fn gyro_bias(&self) -> Vector3<f32> {
        Vector3::new(self.x[4], self.x[5], self.x[6])
    }

    /// Get bias uncertainty (diagonal of covariance)
    pub fn gyro_bias_variance(&self) -> Vector3<f32> {
        Vector3::new(self.P[(4, 4)], self.P[(5, 5)], self.P[(6, 6)])
    }
}
```

**Bias Compensation in Prediction:**

```rust
pub fn predict(&mut self, gyro: Vector3<f32>, dt: f32) {
    // Get current bias estimate
    let bias = self.gyro_bias();

    // Compensate gyro measurement
    let omega = gyro - bias;

    // Use compensated rate for quaternion propagation
    let q = self.quaternion();
    let q_dot = 0.5 * q * Quaternion::from_imag(omega);
    // ...
}
```

**Bias Dynamics Model:**

Gyro bias modeled as random walk:

```rust
// State transition matrix F (7x7)
// Bias states have identity dynamics (slowly varying)
// F[4:7, 4:7] = I (bias persists between updates)
// Process noise Q[4:7, 4:7] models random walk
```

**Process Noise for Bias:**

```rust
// Typical gyro bias random walk: 0.0001 rad/s/√Hz
// At 400Hz: σ² = (0.0001)² / 400 = 2.5e-11 per sample
const GYRO_BIAS_NOISE: f32 = 2.5e-11;
```

**Observability:**

Gyro bias is observable through accelerometer/magnetometer corrections:

- When vehicle is stationary, accel correction observes roll/pitch bias
- When vehicle moves, mag correction observes yaw bias
- Continuous updates keep bias estimate accurate

**Convergence Criteria:**

```rust
/// Check if bias estimate has converged
pub fn bias_converged(&self) -> bool {
    let variance = self.gyro_bias_variance();
    // Converged when variance < threshold (e.g., 0.001 rad²/s²)
    variance.iter().all(|v| *v < 0.001)
}
```

## Platform Considerations

### Cross-Platform

- Same bias estimation on both RP2040 and RP2350
- No platform-specific considerations for this feature

## Risks & Mitigation

| Risk                                   | Impact | Likelihood | Mitigation                                  | Validation                        |
| -------------------------------------- | ------ | ---------- | ------------------------------------------- | --------------------------------- |
| Bias estimate diverges                 | High   | Low        | Bound bias to reasonable range (±0.1 rad/s) | Monitor bias over time            |
| Slow convergence                       | Medium | Medium     | Initialize bias from parameter, tune Q      | Measure convergence time          |
| Temperature drift faster than tracking | Medium | Medium     | Higher process noise for bias               | Test over temperature range       |
| Bias confused with attitude            | Medium | Low        | Proper observability from corrections       | Test static and moving conditions |

## Implementation Notes

**Bias Bounds:**

```rust
const MAX_GYRO_BIAS: f32 = 0.1; // rad/s (about 6°/s)

fn clamp_bias(&mut self) {
    for i in 4..7 {
        self.x[i] = self.x[i].clamp(-MAX_GYRO_BIAS, MAX_GYRO_BIAS);
    }
}
```

**Initialization Options:**

1. **Zero initialization**: Start with zero bias, let EKF estimate
2. **Parameter initialization**: Load last known bias from parameters
3. **Static calibration**: Average gyro output during stationary startup

```rust
impl AhrsEkf {
    pub fn set_initial_bias(&mut self, bias: Vector3<f32>) {
        self.x[4] = bias.x;
        self.x[5] = bias.y;
        self.x[6] = bias.z;
    }
}
```

**Telemetry:**

Bias estimate should be included in AHRS telemetry for monitoring:

- Include in `ATTITUDE` debug extension
- Log bias values periodically

## External References

- [Gyroscope Bias Estimation (ResearchGate)](https://www.researchgate.net/publication/224312142_Gyroscope_Bias_Estimation)
- [ArduPilot Gyro Bias](https://ardupilot.org/dev/docs/extended-kalman-filter.html#gyro-bias)
