# NFR-00053 Pre-Disarm Validation Performance

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-00017-pre-disarm-validation](../analysis/AN-00017-pre-disarm-validation.md)
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

Pre-disarm validation shall complete within 1ms of disarm command to ensure responsive disarm experience while verifying safety conditions before allowing vehicle shutdown.

## Rationale

Pre-disarm validation performs critical safety checks (armed state verification, throttle level check, velocity check, method-specific rules) before allowing the vehicle to disarm. Validation time must be imperceptible to operators while ensuring all safety checks complete:

- **Operator Responsiveness**: Validation delays > 10ms feel sluggish, reduce operator confidence in disarm mechanism
- **Safety Verification**: Throttle check (< 10us), velocity check (< 100us), mode checks (< 5us) must all complete
- **Method Distinction**: Different disarm methods (GCS vs RC vs failsafe) have different validation requirements
- **Industry Standard**: ArduPilot completes pre-disarm validation in < 150us typical, acceptable delay before safety shutdown

The 1ms target provides margin for all validation checks (throttle, velocity, mode restrictions) while maintaining instantaneous response feel. Unlike post-disarm cleanup (100ms target), pre-disarm validation must be imperceptible to operators.

## User Story (if applicable)

The system shall complete pre-disarm validation within 1ms to ensure the operator experiences instantaneous disarm response while all safety verifications are completed before state transition.

## Acceptance Criteria

- [ ] Pre-disarm validation completes within 1ms measured from validation entry to result return (99th percentile)
- [ ] Typical case (all checks enabled) completes within 200us (50th percentile)
- [ ] Validation time measured includes: armed state check, throttle check, velocity check, method-specific validation
- [ ] Performance verified under 75% CPU load (concurrent tasks running)
- [ ] Timing measured on both Pico W and Pico 2 W platforms
- [ ] No individual validation check exceeds 200us
- [ ] Forced disarm bypass adds no measurable overhead (< 1us to skip validation)

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance:**

- **Target Latency**: < 1ms (validation entry to result return)
  - Armed state check: < 1 us (boolean comparison)
  - Throttle value read: < 10 us (RC channel read)
  - Throttle threshold check: < 1 us (float comparison)
  - Velocity read: < 100 us (calculate from sensors)
  - Velocity threshold check: < 1 us (float comparison)
  - Mode capability query: < 5 us (trait method call)
  - Method-specific logic: < 10 us (match statement, branching)
  - **Total typical**: \~150 us
  - **Total worst case**: \~1000 us (1ms)

**Measurement Method:**

```rust
/// Validate disarm conditions with performance profiling
fn validate_disarm(&self, method: DisarmMethod) -> DisarmValidationResult {
    let start_time = timer.now_micros();

    // 1. Check if already disarmed (< 1us)
    if !self.is_armed() {
        return DisarmValidationResult::DeniedAlreadyDisarmed;
    }

    // 2. Apply method-specific validation rules
    let result = match method {
        DisarmMethod::GcsCommand => self.validate_disarm_gcs(),
        DisarmMethod::RcSwitch => self.validate_disarm_rc(),
        DisarmMethod::Failsafe => DisarmValidationResult::Allowed,
        DisarmMethod::ForceDisarm => DisarmValidationResult::Allowed,
        _ => DisarmValidationResult::DeniedUnsafeMode,
    };

    let duration = timer.now_micros() - start_time;

    // Log validation timing for performance analysis
    defmt::trace!("Pre-disarm validation: {}us, result: {:?}", duration, result);

    // Assert performance requirement in test builds
    #[cfg(test)]
    assert!(duration < 1_000, "Validation exceeded 1ms: {}us", duration);

    result
}

/// Validate GCS command disarm
fn validate_disarm_gcs(&self) -> DisarmValidationResult {
    let config = &self.disarm_validation_config;

    // Check throttle (< 20us total)
    let throttle = self.get_throttle_normalized();
    if throttle > config.max_throttle_gcs {
        return DisarmValidationResult::DeniedThrottleActive;
    }

    // Velocity check optional for GCS (< 110us if enabled)
    if config.check_velocity_gcs {
        let velocity = self.get_velocity_mps();
        if velocity > config.max_velocity_mps {
            return DisarmValidationResult::DeniedVelocityTooHigh;
        }
    }

    DisarmValidationResult::Allowed
}
```

**Performance Budget:**

| Validation Step          | Typical   | Worst Case | Notes                                |
| ------------------------ | --------- | ---------- | ------------------------------------ |
| Armed state check        | < 1 us    | < 1 us     | Boolean comparison                   |
| Throttle value read      | 5 us      | 10 us      | RC channel read                      |
| Throttle threshold check | < 1 us    | < 1 us     | Float comparison                     |
| Velocity read            | 50 us     | 100 us     | Calculate from sensors               |
| Velocity threshold check | < 1 us    | < 1 us     | Float comparison                     |
| Mode capability query    | < 5 us    | 5 us       | Trait method call                    |
| Method-specific logic    | < 10 us   | 10 us      | Match statement, branching           |
| **Total**                | **150us** | **1000us** | Target < 1ms for imperceptible delay |

**Implementation Strategies:**

**Option A: Inline Validation (Recommended for Phase 1)**

```rust
fn validate_disarm(&self, method: DisarmMethod) -> DisarmValidationResult {
    // Check armed state
    if !self.is_armed() {
        return DisarmValidationResult::DeniedAlreadyDisarmed;
    }

    // Method-specific validation (inline for performance)
    match method {
        DisarmMethod::GcsCommand => {
            if self.get_throttle_normalized() > self.config.max_throttle_gcs {
                return DisarmValidationResult::DeniedThrottleActive;
            }
            if self.config.check_velocity_gcs &&
               self.get_velocity_mps() > self.config.max_velocity_mps {
                return DisarmValidationResult::DeniedVelocityTooHigh;
            }
        }
        DisarmMethod::RcSwitch => {
            if self.get_throttle_normalized() > self.config.max_throttle_manual {
                return DisarmValidationResult::DeniedThrottleActive;
            }
            if self.get_velocity_mps() > self.config.max_velocity_mps {
                return DisarmValidationResult::DeniedVelocityTooHigh;
            }
            if !self.mode.allows_rc_disarm() {
                return DisarmValidationResult::DeniedUnsafeMode;
            }
        }
        DisarmMethod::Failsafe | DisarmMethod::ForceDisarm => {
            // No validation for emergency methods
        }
        _ => return DisarmValidationResult::DeniedUnsafeMode,
    }

    DisarmValidationResult::Allowed
}
```

**Pros**: Simple, no function call overhead, meets < 1ms target easily
**Cons**: Less modular than separate functions per method

**Optimization Techniques:**

- **Early exit**: Return immediately on first validation failure
- **Inline sensor reads**: Avoid function call overhead for throttle/velocity reads
- **Cache mode capabilities**: Query mode.allows_rc_disarm() once, store result
- **Avoid allocations**: All validation logic uses stack-only data
- **Forced disarm bypass**: Check forced flag before validation entry (< 1us)

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz:

- Sensor reads may be slower (10-15us) due to lower clock speed
- Total validation time likely 200-300us typical
- Synchronous validation easily meets 1ms target on Pico W

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz with FPU:

- Faster sensor reads (5-10us)
- More headroom for concurrent tasks
- Synchronous validation easily meets 150us typical target

### Cross-Platform

Pre-disarm validation must meet 1ms requirement on both platforms. All validation checks complete within budget on both Pico W and Pico 2 W.

## Risks & Mitigation

| Risk                                            | Impact | Likelihood | Mitigation                                                        | Validation                                          |
| ----------------------------------------------- | ------ | ---------- | ----------------------------------------------------------------- | --------------------------------------------------- |
| Velocity calculation exceeds 100us budget       | Medium | Low        | Use cached velocity estimate, update at 10Hz in background        | Measure velocity calculation time on both platforms |
| Throttle read exceeds 10us budget               | Low    | Low        | Use DMA-based RC input, cache latest value                        | Profile RC channel read time on both platforms      |
| Mode capability query causes blocking           | Medium | Low        | Ensure trait method is simple boolean lookup, no I/O              | Verify query completes within 5us                   |
| Total validation time exceeds 1ms on Pico W     | High   | Low        | Profile early, inline sensor reads if needed                      | Measure on real hardware under load                 |
| Performance regression in future updates        | Medium | Medium     | Add performance test to CI, fail if validation time exceeds 1ms   | Automated timing test in test suite                 |
| Complex validation compromises performance goal | Medium | Low        | Keep validation logic simple, defer complex checks to post-disarm | Review validation sequence, minimize branching      |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Pre-disarm validation with timing profiling
pub fn disarm(&mut self, method: DisarmMethod, forced: bool)
              -> Result<(), &'static str> {
    // Fast path: forced disarm bypasses validation (< 1us)
    if !forced {
        let validation_start = timer.now_micros();
        let validation_result = self.validate_disarm(method);
        let validation_duration = timer.now_micros() - validation_start;

        defmt::trace!("Pre-disarm validation: {}us, result: {:?}",
                      validation_duration, validation_result);

        if validation_result != DisarmValidationResult::Allowed {
            // Log denied disarm attempt
            self.log_disarm_denied(method, validation_result)?;
            return Err(validation_result.to_error_message());
        }

        // Verify performance requirement in test builds
        #[cfg(test)]
        assert!(validation_duration < 1_000,
                "Validation exceeded 1ms: {}us", validation_duration);
    }

    // Validation passed (or forced), proceed with disarm
    self.armed = ArmedState::Disarmed;

    // Execute post-disarm cleanup (AN-00016)
    self.post_disarm_cleanup(method, forced)?;

    Ok(())
}
```

**Performance Testing:**

```rust
#[test]
fn test_pre_disarm_validation_performance() {
    let mut state = SystemState::new();

    // Arm vehicle
    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Measure validation over 1000 iterations
    let mut durations = [0u64; 1000];
    for i in 0..1000 {
        let start = timer.now_micros();
        let result = state.validate_disarm(DisarmMethod::GcsCommand);
        durations[i] = timer.now_micros() - start;

        // Verify validation allows disarm
        assert_eq!(result, DisarmValidationResult::Allowed);
    }

    // Calculate statistics
    let mean = durations.iter().sum::<u64>() / 1000;
    let mut sorted = durations;
    sorted.sort();
    let p50 = sorted[500];
    let p95 = sorted[950];
    let p99 = sorted[990];

    println!("Validation latency: mean={}us, p50={}us, p95={}us, p99={}us",
             mean, p50, p95, p99);

    // Assert performance requirements
    assert!(p99 < 1_000, "99th percentile exceeds 1ms: {}us", p99);
    assert!(p50 < 200, "Median exceeds 200us: {}us", p50);
}

#[test]
fn test_forced_disarm_bypass_overhead() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Measure forced disarm (validation bypass)
    let start = timer.now_micros();
    state.disarm(DisarmMethod::ForceDisarm, true).unwrap();
    let duration = timer.now_micros() - start;

    // Forced disarm should have minimal overhead (< 1us to skip validation)
    // Total disarm time includes post-disarm cleanup (~100ms)
    // Validation bypass overhead should be < 1us
    println!("Forced disarm validation bypass overhead: {}us", duration);
}
```

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState::disarm() and validate_disarm()
- `src/vehicle/arming/validation.rs` - Validation logic implementation
- `src/vehicle/arming/types.rs` - DisarmValidationResult, DisarmValidationConfig
- `src/platform/*/timer.rs` - High-resolution timing for profiling

## External References

- Analysis: [AN-00017-pre-disarm-validation](../analysis/AN-00017-pre-disarm-validation.md)
  N/A - No external references
