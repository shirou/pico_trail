# NFR-kqvyf Manual Control Latency

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-uk0us-manual-mode](FR-uk0us-manual-mode.md)
  - [FR-993xy-rc-channels-processing](FR-993xy-rc-channels-processing.md)
  - [NFR-ukjvr-control-loop-latency](NFR-ukjvr-control-loop-latency.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-3irm5-manual-control-implementation](../tasks/T-3irm5-manual-control-implementation/README.md)

## Requirement Statement

Manual mode control latency from RC input (RC_CHANNELS message reception) to actuator response (PWM output change) shall not exceed 100ms to ensure immediate and responsive operator control.

## Rationale

Operator expectations for manual control:

- **Immediate response**: Joystick/RC input should feel instant, like driving RC car
- **Safety**: Delayed response reduces operator ability to react to obstacles
- **User experience**: High latency makes vehicle difficult to control accurately

100ms target accommodates:

- MAVLink message latency: 10-50ms (UART), 20-100ms (WiFi)
- Processing latency: Mode update + actuator mixing < 10ms
- PWM update: 0-20ms (next PWM cycle at 50 Hz)
- **Total**: Worst case \~130ms, typical \~70ms

This aligns with human perception: < 100ms feels instant, > 200ms feels laggy.

## User Story (if applicable)

The system shall process manual control inputs within 100ms total latency to ensure operators experience immediate vehicle response when driving manually via Mission Planner joystick or RC transmitter.

## Acceptance Criteria

- [ ] Total latency (RC_CHANNELS reception → PWM output) < 100ms (typical case)
- [ ] Total latency < 130ms (worst case, including WiFi overhead)
- [ ] Mode update execution time < 1ms (measured via profiling)
- [ ] Actuator conversion time < 1ms (normalized → PWM)
- [ ] Control loop jitter < 5ms (consistent 50 Hz execution)
- [ ] Latency measured via automated test: timestamp RC_CHANNELS → log PWM change
- [ ] Performance regression test: verify latency doesn't increase over time

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Latency Breakdown:**

| Component                | Typical  | Worst Case | Notes                                |
| ------------------------ | -------- | ---------- | ------------------------------------ |
| MAVLink transport (UART) | 10ms     | 50ms       | Serial transmission time             |
| MAVLink transport (WiFi) | 20ms     | 100ms      | Network latency                      |
| RC_CHANNELS parsing      | < 1ms    | < 1ms      | Lightweight message deserialization  |
| Mode update (Manual)     | < 1ms    | < 1ms      | Simple pass-through logic            |
| Actuator conversion      | < 1ms    | < 1ms      | Normalized → PWM calculation         |
| PWM hardware update      | 0-20ms   | 0-20ms     | Next PWM cycle (50 Hz = 20ms period) |
| **Total (UART)**         | **12ms** | **72ms**   | Typical operation                    |
| **Total (WiFi)**         | **22ms** | **122ms**  | Acceptable for manual control        |

**Performance Targets:**

- Mode `update()` execution: < 1ms (measured via Embassy timer)
- Actuator `set_steering()` / `set_throttle()`: < 1ms
- Control loop period: 20ms ± 1ms (50 Hz with low jitter)

**Measurement Strategy:**

1. **Latency test**: Inject RC_CHANNELS with timestamp, measure time to PWM change
2. **Profiling**: Use Embassy `Instant::now()` to measure function execution times
3. **Jitter analysis**: Log control loop period over 1000 iterations, calculate std dev

**Optimization Guidelines:**

- Avoid heap allocation in control loop (use stack or static allocation)
- Minimize floating point operations (prefer integer math where possible)
- Profile hot paths: mode update, actuator conversion
- Use `#[inline]` for small, frequently called functions

## Platform Considerations

### Pico W (RP2040)

- CPU: 133 MHz Cortex-M0+ (no FPU)
- Floating point operations slower (software emulation)
- Target: < 100ms total latency achievable with careful optimization

### Pico 2 W (RP2350)

- CPU: 150 MHz Cortex-M33 (with FPU)
- Floating point operations fast (hardware accelerated)
- Target: < 100ms total latency easily achievable

### Cross-Platform

Latency target applies to both platforms. RP2040 may require additional optimization for floating point operations.

## Risks & Mitigation

| Risk                                       | Impact | Likelihood | Mitigation                                         | Validation                                      |
| ------------------------------------------ | ------ | ---------- | -------------------------------------------------- | ----------------------------------------------- |
| WiFi latency exceeds 100ms                 | Medium | Medium     | Measure actual WiFi latency, optimize if needed    | Test: ping latency, RC_CHANNELS round-trip time |
| Mode update takes > 1ms (complexity creep) | Medium | Low        | Profile regularly, optimize hot paths              | Automated performance regression test           |
| Control loop jitter (missed deadlines)     | High   | Low        | Use Embassy ticker, priority-based task scheduling | Measure: control loop period std dev < 1ms      |
| Floating point overhead on RP2040          | Medium | Medium     | Profile FP operations, use fixed-point if needed   | Test: mode update time on RP2040 vs RP2350      |

## Implementation Notes

Preferred approaches:

- **Measure early**: Profile latency during implementation, not after
- **Optimize incrementally**: Start simple, optimize if measurements show issues
- **Embassy timer**: Use `Instant::now()` for microsecond-precision timing
- **Regression tests**: Automated performance tests to catch slowdowns

Known pitfalls:

- **Premature optimization**: Don't optimize until measurements show need
- **Hidden allocations**: Heap allocation can add significant latency (avoid in control loop)
- **Floating point on RP2040**: Software FP is slow, consider fixed-point for critical paths
- **WiFi variability**: Network latency varies, measure worst-case scenarios

Related code areas:

- `src/vehicle/modes/manual.rs` - Manual mode update logic
- `src/vehicle/actuators.rs` - Actuator conversion (normalized → PWM)
- `src/communication/mavlink/handlers/rc_input.rs` - RC_CHANNELS parsing
- `src/core/scheduler/tasks/vehicle.rs` - Control loop execution

## External References

- Human Perception of Latency: <https://www.nngroup.com/articles/response-times-3-important-limits/>
- Embedded Systems Performance: <https://interrupt.memfault.com/blog/profiling-embedded-systems>
- ArduPilot Control Loop Timing: <https://ardupilot.org/dev/docs/learning-ardupilot-timing.html>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
