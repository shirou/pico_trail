# NFR-00047 Post-Arm Initialization Performance

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

Post-arm initialization shall complete within 50ms of arm command to ensure responsive operator experience and timely vehicle readiness.

## Rationale

Post-arm initialization establishes operational baseline (logging, timestamps, actuator state, subsystem synchronization) before the vehicle is ready for control input. Completion time must be fast enough that operators perceive arming as instantaneous:

- **Operator Responsiveness**: Delays > 100ms feel sluggish, affect operator confidence
- **Safety Window**: Faster initialization reduces time between arm and ready state
- **Control Loop Readiness**: Actuators and sensors must be prepared before first control cycle
- **Industry Standard**: ArduPilot completes post-arm init in 10-30ms typical, 50ms worst case

The 50ms target provides margin for slower operations (flash logging, I2C communication) while maintaining responsive feel.

## User Story (if applicable)

The system shall complete post-arm initialization within 50ms to ensure the operator experiences immediate vehicle readiness and control systems are prepared before the first control cycle begins.

## Acceptance Criteria

- [ ] Post-arm initialization completes within 50ms measured from arm command to ready state (95th percentile)
- [ ] Typical case (no flash contention, normal CPU load) completes within 20ms (50th percentile)
- [ ] Initialization time measured includes: timestamp recording, event logging, actuator init, subsystem notification
- [ ] Performance verified under 75% CPU load (concurrent tasks running)
- [ ] Timing measured on both Pico W and Pico 2 W platforms
- [ ] No individual init step exceeds 15ms (prevents single bottleneck)

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance:**

- **Target Latency**: < 50ms (arm command to ready state)
  - Timestamp recording: < 1 us (read system clock)
  - Arm event logging: < 15ms (flash write, potentially blocking)
  - Actuator initialization: < 10ms (PWM commands, I2C/SPI communication)
  - Subsystem notification: < 5ms (update data structures, publish events)
  - GPIO updates: < 1 us (set pin state)
  - Warning generation: < 2ms (format and send MAVLink STATUSTEXT)
  - **Total typical**: \~20ms
  - **Total worst case**: \~50ms (flash write slow, I2C contention)

**Measurement Method:**

```rust
// Profile post-arm initialization timing
pub fn arm(&mut self, method: ArmMethod, checks_performed: bool)
           -> Result<(), &'static str> {
    let start_time = timer.now_micros();

    // Pre-arm checks
    if self.is_armed() {
        return Err("Already armed");
    }

    // Set armed state
    self.armed = ArmedState::Armed;

    // Execute post-arm initialization
    let init_start = timer.now_micros();
    self.post_arm_init(method, checks_performed)?;
    let init_duration = timer.now_micros() - init_start;

    let total_duration = timer.now_micros() - start_time;

    // Log timing for performance analysis
    defmt::info!("Arm complete: {}us (init: {}us)",
                 total_duration, init_duration);

    // Assert performance requirement in test builds
    #[cfg(test)]
    assert!(init_duration < 50_000); // < 50ms

    Ok(())
}
```

**Performance Budget:**

| Init Step               | Typical  | Worst Case | Notes                            |
| ----------------------- | -------- | ---------- | -------------------------------- |
| Timestamp recording     | < 1 us   | < 1 us     | Read system clock                |
| Arm event logging       | 5 ms     | 15 ms      | Flash write (potentially slow)   |
| Actuator initialization | 5 ms     | 10 ms      | PWM + I2C/SPI commands           |
| Subsystem notification  | 2 ms     | 5 ms       | Update structs, publish events   |
| GPIO updates            | < 1 us   | < 1 us     | Set pin state                    |
| Warning generation      | 1 ms     | 2 ms       | Format + send MAVLink message    |
| **Total**               | **20ms** | **50ms**   | Target < 50ms for responsiveness |

**Implementation Strategies:**

**Option A: Synchronous Initialization (Recommended for Phase 1)**

```rust
fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                 -> Result<(), &'static str> {
    // Execute all steps synchronously
    self.record_timestamp()?;
    self.log_arm_event(method, checks_performed)?; // May block on flash
    self.initialize_actuators()?;
    self.notify_subsystems()?;
    self.update_gpio()?;

    if !checks_performed {
        self.send_warning("Arming checks disabled")?;
    }

    Ok(())
}
```

**Pros**: Simple, deterministic, meets 50ms target
**Cons**: Flash write may block (15ms worst case)

**Option B: Asynchronous Logging (Phase 2 if needed)**

```rust
fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                 -> Result<(), &'static str> {
    self.record_timestamp()?;

    // Queue log write for background execution
    self.log_queue.push(ArmEvent { method, checks_performed });

    // Continue with other init steps (non-blocking)
    self.initialize_actuators()?;
    self.notify_subsystems()?;
    self.update_gpio()?;

    if !checks_performed {
        self.send_warning("Arming checks disabled")?;
    }

    Ok(())
}
```

**Pros**: Logging doesn't block arm operation (< 10ms total)
**Cons**: More complex, log may fail silently

**Optimization Techniques:**

- Use **DMA** for I2C/SPI actuator commands (reduce CPU overhead)
- **Batch operations**: Combine multiple actuator commands in single transaction
- **Defer non-critical tasks**: Move warning generation to background task if needed
- **Cache alignment**: Ensure hot path code is cache-aligned for predictable timing
- **Profile early**: Measure actual timing on hardware, optimize bottlenecks

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz:

- Flash write may be slower (15-20ms) due to lower clock speed
- I2C/SPI actuator commands may take longer (7-10ms)
- Total init time likely near 50ms worst case
- May need async logging to meet target on Pico W

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz with FPU:

- Faster flash write (10-15ms)
- Faster I2C/SPI communication (5-8ms)
- More headroom for concurrent tasks
- Synchronous init likely meets 20ms typical target

### Cross-Platform

Post-arm initialization must meet 50ms requirement on both platforms. Use platform-optimized drivers and consider async logging for Pico W if synchronous approach exceeds target.

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                                   | Validation                                 |
| ----------------------------------------- | ------ | ---------- | ------------------------------------------------------------ | ------------------------------------------ |
| Flash write exceeds 15ms budget           | Medium | Medium     | Use async logging or optimize flash driver for faster writes | Measure flash write time on both platforms |
| I2C/SPI actuator init exceeds 10ms budget | Medium | Low        | Use DMA, batch commands, switch to SPI if I2C too slow       | Profile actuator init with oscilloscope    |
| Subsystem notification causes blocking    | Medium | Low        | Ensure notifications are non-blocking (update structs only)  | Verify notification completes within 5ms   |
| Total init time exceeds 50ms on Pico W    | High   | Medium     | Profile early, implement async logging if needed             | Measure on real hardware under load        |
| Performance regression in future updates  | Medium | Medium     | Add performance test to CI, fail if init time exceeds 50ms   | Automated timing test in test suite        |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Post-arm initialization with timing profiling
fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                 -> Result<(), &'static str> {
    let init_start = timer.now_micros();

    // 1. Record timestamp (< 1us)
    let step_start = timer.now_micros();
    self.record_timestamp()?;
    let timestamp_duration = timer.now_micros() - step_start;

    // 2. Log arm event (< 15ms)
    let step_start = timer.now_micros();
    self.log_arm_event(method, checks_performed)?;
    let log_duration = timer.now_micros() - step_start;

    // 3. Initialize actuators (< 10ms)
    let step_start = timer.now_micros();
    self.initialize_actuators()?;
    let actuator_duration = timer.now_micros() - step_start;

    // 4. Notify subsystems (< 5ms)
    let step_start = timer.now_micros();
    self.notify_subsystems()?;
    let notify_duration = timer.now_micros() - step_start;

    // 5. Update GPIO (< 1us)
    let step_start = timer.now_micros();
    self.update_gpio()?;
    let gpio_duration = timer.now_micros() - step_start;

    // 6. Send warnings if needed (< 2ms)
    let warning_duration = if !checks_performed {
        let step_start = timer.now_micros();
        self.send_warning("Arming checks disabled")?;
        timer.now_micros() - step_start
    } else {
        0
    };

    let total_duration = timer.now_micros() - init_start;

    // Log performance data for analysis
    defmt::debug!("Post-arm init: {}us (ts:{}, log:{}, act:{}, notify:{}, gpio:{}, warn:{})",
                  total_duration,
                  timestamp_duration,
                  log_duration,
                  actuator_duration,
                  notify_duration,
                  gpio_duration,
                  warning_duration);

    // Verify performance requirement
    if total_duration > 50_000 {
        defmt::warn!("Post-arm init exceeded 50ms target: {}us", total_duration);
    }

    Ok(())
}
```

**Performance Testing:**

```rust
#[test]
fn test_post_arm_init_performance() {
    let mut state = SystemState::new();

    // Warm up (ensure caches populated, drivers initialized)
    for _ in 0..10 {
        state.arm(ArmMethod::GcsCommand, true).unwrap();
        state.disarm().unwrap();
    }

    // Measure over 100 iterations
    let mut durations = [0u64; 100];
    for i in 0..100 {
        let start = timer.now_micros();
        state.arm(ArmMethod::GcsCommand, true).unwrap();
        durations[i] = timer.now_micros() - start;
        state.disarm().unwrap();
    }

    // Calculate statistics
    let mean = durations.iter().sum::<u64>() / 100;
    let mut sorted = durations;
    sorted.sort();
    let p50 = sorted[50];
    let p95 = sorted[95];
    let p99 = sorted[99];

    println!("Arm latency: mean={}us, p50={}us, p95={}us, p99={}us",
             mean, p50, p95, p99);

    // Assert performance requirements
    assert!(p95 < 50_000, "95th percentile exceeds 50ms: {}us", p95);
    assert!(p50 < 20_000, "Median exceeds 20ms: {}us", p50);
}
```

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState::arm() and post_arm_init()
- `src/core/logging/` - Arm event logging implementation
- `src/devices/actuators/` - Actuator initialization
- `src/platform/*/timer.rs` - High-resolution timing for profiling

## External References

- Analysis: [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
  N/A - No external references
