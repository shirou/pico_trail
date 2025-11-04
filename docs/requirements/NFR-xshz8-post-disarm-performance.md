# NFR-xshz8 Post-Disarm Cleanup Performance

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-081u0-post-disarm-cleanup](../analysis/AN-081u0-post-disarm-cleanup.md)
  - [AN-3ntt5-post-disarm-cleanup](../analysis/AN-3ntt5-post-disarm-cleanup.md)
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

Post-disarm cleanup shall complete within 100ms of disarm command to ensure responsive operator experience while prioritizing actuator safety verification over speed.

## Rationale

Post-disarm cleanup performs critical safety shutdown (logging, actuator safety verification, subsystem notification, state reset) after the vehicle is disarmed. Completion time must be fast enough that operators perceive disarming as instantaneous, while allowing sufficient time for actuator safety verification:

- **Operator Responsiveness**: Delays > 200ms feel sluggish, affect operator confidence
- **Safety Priority**: Actuator safety verification (50ms worst case) is non-negotiable, takes priority over speed
- **Cleanup Completeness**: Logging (10ms), subsystem notification (1ms), state reset (1ms) must complete
- **Industry Standard**: ArduPilot completes post-disarm cleanup in 20-60ms typical, acceptable delay for safety

The 100ms target provides margin for actuator safety verification (50ms worst case) while maintaining responsive feel. Unlike post-arm initialization (50ms target), post-disarm cleanup prioritizes safety over speed.

## User Story (if applicable)

The system shall complete post-disarm cleanup within 100ms to ensure the operator experiences immediate vehicle shutdown response while all safety verifications are completed before cleanup finishes.

## Acceptance Criteria

- [ ] Post-disarm cleanup completes within 100ms measured from disarm command to cleanup complete (95th percentile)
- [ ] Typical case (no flash contention, normal CPU load) completes within 60ms (50th percentile)
- [ ] Cleanup time measured includes: disarm event logging, actuator safety verification, subsystem notification, state reset
- [ ] Performance verified under 75% CPU load (concurrent tasks running)
- [ ] Timing measured on both Pico W and Pico 2 W platforms
- [ ] Actuator safety verification allowed up to 50ms timeout (non-negotiable safety step)
- [ ] No individual cleanup step except actuator verification exceeds 15ms

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance:**

- **Target Latency**: < 100ms (disarm command to cleanup complete)
  - Timestamp recording: < 1 us (read system clock)
  - Disarm event logging: < 10ms (flash write, potentially blocking)
  - Actuator safety verification: < 50ms (PWM readback, hardware timeout)
  - Subsystem notification: < 1ms (update data structures, publish events)
  - GPIO updates: < 1 us (clear pin state)
  - State reset: < 1 us (clear memory structures)
  - Logging persistence check: < 1 ms (determine if continue logging)
  - **Total typical**: \~60ms
  - **Total worst case**: \~100ms (actuator verify slow, flash write slow)

**Measurement Method:**

```rust
// Profile post-disarm cleanup timing
pub fn disarm(&mut self, method: DisarmMethod, reason: DisarmReason)
               -> Result<(), &'static str> {
    let start_time = timer.now_micros();

    // Pre-disarm checks
    if !self.is_armed() {
        return Err("Already disarmed");
    }

    // Calculate armed duration
    let armed_duration_ms = self.time_since_arm_ms()
        .ok_or("Arm time not recorded")?;

    // Set disarmed state
    self.armed = ArmedState::Disarmed;

    // Execute post-disarm cleanup
    let cleanup_start = timer.now_micros();
    self.post_disarm_cleanup(method, reason, armed_duration_ms)?;
    let cleanup_duration = timer.now_micros() - cleanup_start;

    let total_duration = timer.now_micros() - start_time;

    // Log timing for performance analysis
    defmt::info!("Disarm complete: {}us (cleanup: {}us)",
                 total_duration, cleanup_duration);

    // Assert performance requirement in test builds
    #[cfg(test)]
    assert!(cleanup_duration < 100_000); // < 100ms

    Ok(())
}
```

**Performance Budget:**

| Cleanup Step              | Typical  | Worst Case | Notes                              |
| ------------------------- | -------- | ---------- | ---------------------------------- |
| Timestamp recording       | < 1 us   | < 1 us     | Read system clock                  |
| Disarm event logging      | 5 ms     | 10 ms      | Flash write (potentially blocking) |
| Actuator safety verify    | 10 ms    | 50 ms      | PWM readback, hardware timeout     |
| Subsystem notification    | < 1 ms   | 1 ms       | Update structs, publish events     |
| GPIO updates              | < 1 us   | < 1 us     | Clear pin state                    |
| State reset               | < 1 us   | < 1 us     | Clear memory structures            |
| Logging persistence check | < 1 ms   | 1 ms       | Determine if continue logging      |
| **Total**                 | **60ms** | **100ms**  | Target < 100ms for safety priority |

**Implementation Strategies:**

**Option A: Synchronous Cleanup (Recommended for Phase 1)**

```rust
fn post_disarm_cleanup(&mut self, method: DisarmMethod, reason: DisarmReason,
                       armed_duration_ms: u32) -> Result<(), &'static str> {
    // Execute all steps synchronously
    self.record_timestamp()?;
    self.log_disarm_event(method, reason, armed_duration_ms)?; // May block on flash
    self.verify_actuators_safe()?; // Critical: up to 50ms timeout
    self.notify_subsystems_disarmed()?;
    self.update_gpio(false)?;
    self.reset_armed_state()?;

    if self.should_persist_logging(reason) {
        self.extend_logging_duration()?;
    }

    Ok(())
}
```

**Pros**: Simple, deterministic, meets 100ms target, prioritizes actuator safety
**Cons**: Actuator verify may block (50ms worst case), acceptable for safety

**Option B: Async Actuator Verification (Phase 2 if needed)**

```rust
fn post_disarm_cleanup(&mut self, method: DisarmMethod, reason: DisarmReason,
                       armed_duration_ms: u32) -> Result<(), &'static str> {
    self.record_timestamp()?;
    self.log_disarm_event(method, reason, armed_duration_ms)?;

    // Start actuator verification (non-blocking)
    self.start_actuator_verify_async()?;

    // Continue with other cleanup steps
    self.notify_subsystems_disarmed()?;
    self.update_gpio(false)?;
    self.reset_armed_state()?;

    // Wait for actuator verification to complete (timeout 50ms)
    self.wait_actuator_verify_complete(50)?;

    if self.should_persist_logging(reason) {
        self.extend_logging_duration()?;
    }

    Ok(())
}
```

**Pros**: Other cleanup doesn't block on actuator verify (reduces typical case to 20ms)
**Cons**: More complex, actuator safety still enforced before completion

**Optimization Techniques:**

- Use **DMA** for PWM readback (reduce CPU overhead)
- **Batch operations**: Combine multiple GPIO updates in single transaction
- **Early timeout**: Abort actuator verify after 50ms, log warning but complete disarm
- **Profile early**: Measure actual timing on hardware, optimize bottlenecks
- **Prioritize safety**: Actuator verification is non-negotiable, allocate full 50ms budget

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz:

- Flash write may be slower (10-15ms) due to lower clock speed
- Actuator PWM readback may take longer (30-50ms worst case)
- Total cleanup time likely near 100ms worst case
- Synchronous cleanup acceptable, meets target on Pico W

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz with FPU:

- Faster flash write (5-10ms)
- Faster PWM readback (20-40ms)
- More headroom for concurrent tasks
- Synchronous cleanup easily meets 60ms typical target

### Cross-Platform

Post-disarm cleanup must meet 100ms requirement on both platforms. Actuator safety verification is non-negotiable and allocated up to 50ms on both platforms.

## Risks & Mitigation

| Risk                                            | Impact | Likelihood | Mitigation                                                         | Validation                                       |
| ----------------------------------------------- | ------ | ---------- | ------------------------------------------------------------------ | ------------------------------------------------ |
| Actuator verify exceeds 50ms budget             | Medium | Medium     | Use timeout, log warning but complete disarm, safety first         | Measure actuator readback time on both platforms |
| Flash write exceeds 10ms budget                 | Medium | Low        | Use async logging or optimize flash driver for faster writes       | Profile flash write time on both platforms       |
| Subsystem notification causes blocking          | Medium | Low        | Ensure notifications are non-blocking (update structs only)        | Verify notification completes within 1ms         |
| Total cleanup time exceeds 100ms on Pico W      | High   | Low        | Profile early, async actuator verify if needed                     | Measure on real hardware under load              |
| Performance regression in future updates        | Medium | Medium     | Add performance test to CI, fail if cleanup time exceeds 100ms     | Automated timing test in test suite              |
| Prioritizing speed over safety compromises goal | High   | Low        | Document that actuator safety takes priority, enforce 50ms timeout | Review cleanup sequence, verify safety first     |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Post-disarm cleanup with timing profiling
fn post_disarm_cleanup(&mut self, method: DisarmMethod, reason: DisarmReason,
                       armed_duration_ms: u32) -> Result<(), &'static str> {
    let cleanup_start = timer.now_micros();

    // 1. Record timestamp (< 1us)
    let step_start = timer.now_micros();
    let disarm_time_ms = get_time_ms();
    let timestamp_duration = timer.now_micros() - step_start;

    // 2. Log disarm event (< 10ms)
    let step_start = timer.now_micros();
    self.log_disarm_event(disarm_time_ms, method, reason, armed_duration_ms)?;
    let log_duration = timer.now_micros() - step_start;

    // 3. Verify actuators safe (< 50ms, CRITICAL)
    let step_start = timer.now_micros();
    self.verify_actuators_safe()?;
    let actuator_duration = timer.now_micros() - step_start;

    // 4. Notify subsystems (< 1ms)
    let step_start = timer.now_micros();
    self.notify_subsystems_disarmed()?;
    let notify_duration = timer.now_micros() - step_start;

    // 5. Update GPIO (< 1us)
    let step_start = timer.now_micros();
    self.update_gpio(false)?;
    let gpio_duration = timer.now_micros() - step_start;

    // 6. Reset armed state (< 1us)
    let step_start = timer.now_micros();
    self.reset_armed_state()?;
    let reset_duration = timer.now_micros() - step_start;

    // 7. Check logging persistence (< 1ms)
    let logging_duration = if self.should_persist_logging(reason) {
        let step_start = timer.now_micros();
        self.extend_logging_duration()?;
        timer.now_micros() - step_start
    } else {
        0
    };

    // Store cleanup state
    self.post_disarm_state = PostDisarmState {
        disarm_time_ms,
        disarm_method: method,
        disarm_reason: reason,
        armed_duration_ms,
        actuators_safe: true,
        subsystems_notified: true,
    };

    let total_duration = timer.now_micros() - cleanup_start;

    // Log performance data for analysis
    defmt::debug!(
        "Post-disarm cleanup: {}us (ts:{}, log:{}, act:{}, notify:{}, gpio:{}, reset:{}, logging:{})",
        total_duration,
        timestamp_duration,
        log_duration,
        actuator_duration,
        notify_duration,
        gpio_duration,
        reset_duration,
        logging_duration
    );

    // Verify performance requirement
    if total_duration > 100_000 {
        defmt::warn!("Post-disarm cleanup exceeded 100ms target: {}us", total_duration);
    }

    // Warn if actuator verify was slow (but acceptable)
    if actuator_duration > 30_000 {
        defmt::warn!("Actuator safety verify slow: {}us (acceptable, safety priority)", actuator_duration);
    }

    Ok(())
}
```

**Performance Testing:**

```rust
#[test]
fn test_post_disarm_cleanup_performance() {
    let mut state = SystemState::new();

    // Warm up (ensure caches populated, drivers initialized)
    for _ in 0..10 {
        state.arm(ArmMethod::GcsCommand, true).unwrap();
        state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal).unwrap();
    }

    // Measure over 100 iterations
    let mut durations = [0u64; 100];
    for i in 0..100 {
        state.arm(ArmMethod::GcsCommand, true).unwrap();

        let start = timer.now_micros();
        state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal).unwrap();
        durations[i] = timer.now_micros() - start;
    }

    // Calculate statistics
    let mean = durations.iter().sum::<u64>() / 100;
    let mut sorted = durations;
    sorted.sort();
    let p50 = sorted[50];
    let p95 = sorted[95];
    let p99 = sorted[99];

    println!("Disarm latency: mean={}us, p50={}us, p95={}us, p99={}us",
             mean, p50, p95, p99);

    // Assert performance requirements
    assert!(p95 < 100_000, "95th percentile exceeds 100ms: {}us", p95);
    assert!(p50 < 60_000, "Median exceeds 60ms: {}us", p50);
}

#[test]
fn test_actuator_verify_timeout() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate slow actuator hardware
    state.actuators.set_slow_mode(true);

    let start = timer.now_micros();
    state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal).unwrap();
    let duration = timer.now_micros() - start;

    // Should still complete within 100ms despite slow actuators
    assert!(duration < 100_000, "Disarm with slow actuators exceeded 100ms: {}us", duration);
}
```

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState::disarm() and post_disarm_cleanup()
- `src/core/logging/` - Disarm event logging implementation
- `src/devices/actuators/` - Actuator safety verification
- `src/platform/*/timer.rs` - High-resolution timing for profiling

## External References

- Analysis: [AN-081u0-post-disarm-cleanup](../analysis/AN-081u0-post-disarm-cleanup.md)
  N/A - No external references

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
