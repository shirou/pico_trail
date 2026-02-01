# FR-00007 Task Scheduler with Configurable Rates

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements: N/A

- Dependent Requirements:
  - [FR-00004-gps-waypoint-navigation](FR-00004-gps-waypoint-navigation.md)
  - [FR-00002-data-logging](FR-00002-data-logging.md)
  - [FR-00001-ahrs-attitude-estimation](FR-00001-ahrs-attitude-estimation.md)
  - [FR-00053-rc-signal-loss-failsafe](FR-00053-rc-signal-loss-failsafe.md)
  - [FR-00105-icm20948-i2c-driver](FR-00105-icm20948-i2c-driver.md)
  - [FR-00103-mpu9250-i2c-driver](FR-00103-mpu9250-i2c-driver.md)
  - [FR-00061-control-mode-framework](FR-00061-control-mode-framework.md)
  - [FR-00062-control-modes](FR-00062-control-modes.md)
  - [FR-00003-failsafe-mechanisms](FR-00003-failsafe-mechanisms.md)
  - [FR-00013-manual-mode](FR-00013-manual-mode.md)
  - [NFR-00002-imu-sampling-rate](NFR-00002-imu-sampling-rate.md)
  - [NFR-00001-control-loop-latency](NFR-00001-control-loop-latency.md)

- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall maintain a task scheduler that executes periodic tasks at configurable rates ranging from 1Hz to 400Hz with deterministic timing.

## Rationale

Critical for real-time control loop execution. ArduPilot uses a scheduler with different task rates (400Hz IMU sampling, 50Hz control loops, 10Hz telemetry). This architecture ensures that high-priority tasks (sensor sampling) execute frequently while lower-priority tasks (telemetry) run less often, maximizing CPU efficiency while meeting real-time constraints.

## User Story (if applicable)

As an autopilot system, I want to execute tasks at their required rates (400Hz for IMU, 50Hz for control, 10Hz for telemetry), so that sensor data is sampled with low jitter, control loops run deterministically, and CPU resources are used efficiently.

## Acceptance Criteria

- [ ] Scheduler supports task registration with rates from 1Hz to 400Hz
- [ ] Tasks execute within 5% of target period over a 10-second measurement window
- [ ] No task deadlines are missed under normal operational load (75% CPU utilization)
- [ ] Scheduler provides task runtime monitoring (execution time per task)
- [ ] Scheduler calculates and reports overall CPU load percentage
- [ ] Higher-priority tasks preempt lower-priority tasks when necessary

## Technical Details (if applicable)

### Functional Requirement Details

**Task Types:**

- High-frequency tasks: IMU sampling (400Hz), AHRS update (100Hz)
- Medium-frequency tasks: Control loops (50Hz), Navigation (10Hz)
- Low-frequency tasks: Telemetry (10Hz), Logging (5Hz), Parameter save (on-demand)

**Scheduler Interface:**

Tasks should be registered with:

- Execution rate (Hz)
- Priority level (0-255, higher = more important)
- Execution function (callback)
- Stack size requirement

**Timing Requirements:**

- 400Hz task period: 2.5ms ± 0.125ms (5% tolerance)
- 50Hz task period: 20ms ± 1ms
- 10Hz task period: 100ms ± 5ms

**Error Handling:**

- Scheduler detects task overruns (execution time exceeds period)
- Logs warning when task misses deadline
- Provides mechanism to disable misbehaving tasks

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz - May require careful optimization to meet 400Hz timing with limited CPU performance.

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz with FPU - More headroom for complex tasks, FPU accelerates AHRS and control calculations.

### Cross-Platform

Scheduler abstraction must work across both platforms with zero-cost abstractions (compile-time dispatch only).

## Risks & Mitigation

| Risk                                         | Impact | Likelihood | Mitigation                                                      | Validation                                      |
| -------------------------------------------- | ------ | ---------- | --------------------------------------------------------------- | ----------------------------------------------- |
| Scheduler jitter affects control performance | High   | Low        | Use hardware timers for critical tasks, profile execution times | Measure jitter over 1000 cycles on both Picos   |
| Task overruns cause deadline misses          | High   | Medium     | Set execution time budgets, monitor task runtime, optimize code | Stress test with maximum sensor/telemetry rates |
| Async overhead too high for Pico W           | Medium | Medium     | Benchmark Embassy vs RTIC vs custom scheduler, choose best      | Profile CPU usage on Pico W at 400Hz IMU rate   |
| Priority inversion blocks high-priority task | Medium | Low        | Use priority ceiling protocol, minimize critical section length | Test with concurrent high/low priority tasks    |

## Implementation Notes

Preferred approaches:

- **Embassy async executor**: Modern async/await, good for I/O-heavy tasks, proven on Cortex-M0+ and M33
- **RTIC**: Zero-cost abstractions, compile-time scheduling guarantees, Cortex-M only
- **Custom timer-based**: Full control but high development effort

The choice between Embassy, RTIC, or custom scheduler will be documented in ADR-002.

Known pitfalls:

- Avoid heap allocation in high-frequency tasks (use static or stack allocation)
- Minimize interrupt disable time to reduce jitter
- Be aware of task stack size requirements (Embassy tasks need dedicated stacks)

Related code areas:

- `src/core/scheduler/` - Scheduler implementation
- `src/platform/*/timer.rs` - Hardware timer abstractions

## External References

- ArduPilot Scheduler Documentation: <https://ardupilot.org/dev/docs/learning-ardupilot-threading.html>
- Embassy Framework: <https://embassy.dev/>
- RTIC Framework: <https://rtic.rs/>
