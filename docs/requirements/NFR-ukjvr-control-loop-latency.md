# NFR-ukjvr Control Loop Latency

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Control loop latency shall not exceed 20ms (50Hz minimum control rate) from sensor read to actuator output, ensuring stable vehicle control at typical rover and boat speeds.

## Rationale

Control loop latency directly affects system stability and responsiveness:

- **Stability**: Excessive latency introduces phase lag, destabilizing PID controllers
- **Performance**: Low latency enables tighter control, faster response to disturbances
- **Safety**: Quick reaction to sensor inputs prevents dangerous situations

ArduPilot targets 50Hz control loops for rovers/boats (20ms period). This provides adequate response time for vehicles traveling at typical speeds (0.5-5 m/s) while being achievable on embedded hardware.

## User Story (if applicable)

The system shall execute control loops at 50Hz minimum (20ms maximum latency) to ensure that steering and throttle commands respond quickly to sensor inputs, maintaining stable and predictable vehicle behavior at speeds up to 5 m/s.

## Acceptance Criteria

- [ ] Control loop executes at 50Hz minimum (measured over 10-second window)
- [ ] End-to-end latency (sensor read → processing → actuator output) ≤ 20ms
- [ ] Jitter (variation in loop timing) < 2ms (10% of period)
- [ ] Control loop timing maintained under 75% CPU load
- [ ] No missed control loop deadlines during normal operation
- [ ] Latency measured and verified on both Pico W and Pico 2 W platforms

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance:**

- **Control Loop Rate**: 50Hz (20ms period) minimum
  - Pico 2 W target: 100Hz (10ms period) with FPU
  - Pico W target: 50Hz (20ms period) without FPU
- **Latency Budget**:
  - Sensor read (IMU, GPS): 1-2ms
  - AHRS update: 3-5ms
  - Control calculation (PID, mixing): 2-4ms
  - Actuator write (PWM): 1ms
  - **Total**: 7-12ms (leaves 8-13ms margin in 20ms period)

**Measurement Method:**

- Use hardware timer or GPIO toggle to measure loop timing
- Calculate statistics: mean, std dev, min, max, 99th percentile
- Log timing violations (loops exceeding 20ms)

**Latency Sources:**

- Sensor I2C/SPI transaction time
- Floating-point math (AHRS, PID)
- Interrupt service routines
- Memory access latency
- Task scheduling overhead

**Optimization Strategies:**

- Use FPU when available (Pico 2 W)
- Minimize critical section length (interrupts disabled)
- Pre-allocate all buffers (no heap allocation in control loop)
- Cache sensor data (read once per loop iteration)
- Optimize hot paths (inline critical functions)

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz, no FPU:

- Floating-point math is slow (software emulation)
- May require optimization or reduced control rate (25Hz fallback)
- Profile carefully to ensure 20ms deadline met

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz with FPU:

- Hardware FPU accelerates control calculations
- Can target 100Hz control rate (10ms period)
- More headroom for additional features

### Cross-Platform

Control loop implementation must meet latency requirements on both platforms. Use platform-specific optimizations where necessary.

## Risks & Mitigation

| Risk                                     | Impact | Likelihood | Mitigation                                                   | Validation                                |
| ---------------------------------------- | ------ | ---------- | ------------------------------------------------------------ | ----------------------------------------- |
| Control loop exceeds 20ms on Pico W      | High   | Medium     | Profile and optimize critical paths, consider 25Hz fallback  | Measure actual latency on Pico W hardware |
| Interrupt latency causes jitter          | Medium | Medium     | Minimize ISR duration, use RTIC for deterministic scheduling | Measure jitter over 1000 loop iterations  |
| FPU usage makes Pico W code incompatible | Low    | Low        | Use conditional compilation for FPU vs soft-float            | Build and test on both platforms          |
| Sensor read blocks control loop          | High   | Low        | Use async sensor reads, timeout if sensor unresponsive       | Test with slow/unresponsive sensors       |

## Implementation Notes

**Measurement Tools:**

```rust
// GPIO toggle method (measure with oscilloscope)
fn control_loop() {
    loop {
        debug_gpio.set_high(); // Loop start

        read_sensors();
        update_ahrs();
        calculate_control();
        write_actuators();

        debug_gpio.set_low(); // Loop end
        // Measure high pulse width = loop execution time
    }
}

// Timer method (measure with profiler)
fn control_loop() {
    let start = timer.now();
    // ... control loop code ...
    let duration = timer.now() - start;
    if duration > 20_000 { // 20ms in microseconds
        log_warning!("Control loop overrun: {}us", duration);
    }
}
```

**Optimization Techniques:**

- Use `#[inline(always)]` for small, frequently-called functions
- Replace division with multiplication by reciprocal where possible
- Use lookup tables for sin/cos if called frequently
- Batch sensor reads (read all I2C sensors in one transaction)

**Profiling:**

- Use `defmt` timestamps to measure function execution times
- Use `probe-rs` debugger to profile CPU usage
- Use ITM (Instrumentation Trace Macrocell) for detailed timing analysis

Related code areas:

- `src/subsystems/control/` - Control loop implementation
- `src/core/scheduler/` - Task timing and scheduling
- `src/platform/*/timer.rs` - Hardware timer abstraction

## External References

- ArduPilot Rover Control Loop: <https://ardupilot.org/dev/docs/learning-ardupilot-threading.html>
- RTIC Real-Time Guarantees: <https://rtic.rs/2/book/en/>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
