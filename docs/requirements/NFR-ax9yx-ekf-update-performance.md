# NFR-ax9yx EKF Update Performance

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-3f2cn-quaternion-ekf-ahrs](FR-3f2cn-quaternion-ekf-ahrs.md)
- Dependent Requirements: None
- Related ADRs:
  - [ADR-ymkzt-ekf-ahrs-implementation](../adr/ADR-ymkzt-ekf-ahrs-implementation.md)
- Related Tasks:
  - [T-p8w8f-ekf-ahrs-implementation](../tasks/T-p8w8f-ekf-ahrs-implementation/README.md)

## Requirement Statement

The EKF 100Hz update cycle (prediction + correction) shall complete within 10ms on RP2040 and 5ms on RP2350, ensuring adequate headroom for other system tasks.

## Rationale

At 100Hz update rate (10ms period), the EKF must complete within the period to avoid timing overruns:

- **RP2040**: No FPU, software floating-point is slower
- **RP2350**: Hardware FPU provides 3-5x speedup for matrix operations
- **CPU Budget**: EKF should use < 50% of period for processing headroom
- **Other Tasks**: Navigation, control, telemetry also need CPU time

Targeting 10ms on RP2040 leaves 0ms margin (may need 50Hz fallback). Targeting 5ms on RP2350 provides 50% margin.

## User Story (if applicable)

The system shall complete EKF attitude updates within timing constraints to maintain 100Hz update rate without CPU overload.

## Acceptance Criteria

- [ ] EKF prediction step (400Hz) completes in < 0.5ms on RP2040
- [ ] EKF prediction step (400Hz) completes in < 0.2ms on RP2350
- [ ] EKF accel correction (100Hz) completes in < 5ms on RP2040
- [ ] EKF accel correction (100Hz) completes in < 2ms on RP2350
- [ ] EKF mag correction (10Hz) completes in < 2ms on RP2040
- [ ] EKF mag correction (10Hz) completes in < 1ms on RP2350
- [ ] Full EKF cycle (prediction + correction) < 10ms on RP2040
- [ ] Full EKF cycle (prediction + correction) < 5ms on RP2350
- [ ] CPU usage for AHRS task < 15% on RP2040 @ 133MHz

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Timing Budget:**

| Operation                | RP2040 Target | RP2350 Target | Notes                       |
| ------------------------ | ------------- | ------------- | --------------------------- |
| Prediction (400Hz)       | < 0.5ms       | < 0.2ms       | Quaternion propagation      |
| Accel correction (100Hz) | < 5ms         | < 2ms         | 3x3 matrix inverse          |
| Mag correction (10Hz)    | < 2ms         | < 1ms         | Heading-only update         |
| Full cycle (worst case)  | < 10ms        | < 5ms         | All operations              |
| Average cycle            | < 3ms         | < 1ms         | Prediction only most cycles |

**Computational Breakdown:**

1. **Prediction (every sample)**:
   - Quaternion multiplication: \~10 ops
   - Bias subtraction: 3 ops
   - Normalization: \~10 ops
   - Covariance propagation: 7x7 matrix multiply (\~350 ops)

2. **Accel Correction (every 4th sample)**:
   - Innovation calculation: \~20 ops
   - Jacobian (3x7): \~50 ops
   - S = &#x48;_&#x50;_&#x48;' + R: \~150 ops
   - Matrix inverse (3x3): \~100 ops
   - Kalman gain K: \~150 ops
   - State update: \~50 ops
   - Covariance update: \~200 ops

3. **Mag Correction (every 40th sample)**:
   - Similar to accel but simpler heading-only

**Measurement Method:**

```rust
// Profile EKF timing
let start = timer.now_micros();
ekf.predict(gyro, dt);
let predict_us = timer.now_micros() - start;

let start = timer.now_micros();
ekf.update_accel(accel);
let accel_us = timer.now_micros() - start;

log_debug!("EKF timing: predict={}us, accel={}us", predict_us, accel_us);
```

**Optimization Strategies:**

1. **Inline Functions**: Inline small quaternion/vector operations
2. **Avoid Allocation**: All data on stack, no heap
3. **Precomputed Constants**: Precompute fixed matrices
4. **Fast Math**: Use `micromath` for trig on RP2040
5. **Reduced Precision**: Consider f16 for covariance if needed

## Platform Considerations

### Pico W (RP2040)

- **No FPU**: Software floating-point \~10x slower
- **Clock**: 133MHz Cortex-M0+
- **Fallback**: May need 50Hz update if 100Hz exceeds budget
- **Optimization**: Critical paths may need hand-optimization

### Pico 2 W (RP2350)

- **Hardware FPU**: Single-precision hardware support
- **Clock**: 150MHz Cortex-M33
- **Performance**: 100Hz easily achievable
- **Headroom**: Can add features without concern

### Cross-Platform

- Conditional compilation for platform-specific optimizations
- Common algorithm, different performance characteristics
- Use `cargo build --release` for optimization

## Risks & Mitigation

| Risk                            | Impact | Likelihood | Mitigation                            | Validation           |
| ------------------------------- | ------ | ---------- | ------------------------------------- | -------------------- |
| RP2040 cannot meet 100Hz        | High   | Medium     | Reduce to 50Hz on RP2040, or optimize | Profile on hardware  |
| Matrix inverse unstable         | High   | Low        | Use pseudo-inverse, regularization    | Test with edge cases |
| Interrupt jitter affects timing | Medium | Medium     | Use high-priority task                | Measure jitter       |
| Optimization breaks correctness | High   | Low        | Test accuracy after optimization      | Compare to reference |

## Implementation Notes

**Profiling Code:**

```rust
#[cfg(feature = "pico2_w")]
mod profiling {
    use defmt::*;

    pub struct EkfTiming {
        pub predict_us: u32,
        pub accel_us: u32,
        pub mag_us: u32,
        pub total_us: u32,
    }

    impl EkfTiming {
        pub fn log(&self) {
            info!("EKF timing: pred={}us accel={}us mag={}us total={}us",
                self.predict_us, self.accel_us, self.mag_us, self.total_us);
        }
    }
}
```

**Fast Math for RP2040:**

```rust
#[cfg(not(feature = "fpu"))]
use micromath::F32Ext;

// micromath provides fast approximations:
// sin, cos: ~10x faster than libm
// atan2: ~5x faster
// sqrt: ~3x faster
```

**Adaptive Update Rate:**

```rust
impl AhrsEkf {
    /// Reduce update rate if CPU overloaded
    pub fn check_timing(&mut self, cycle_time_us: u32) {
        if cycle_time_us > 10000 { // > 10ms
            self.update_decimation = 2; // Reduce to 50Hz
            log_warn!("EKF reduced to 50Hz due to CPU load");
        }
    }
}
```

## External References

- [Efficient EKF Implementation](https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/ecl/EKF)
- [micromath Crate](https://crates.io/crates/micromath)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
