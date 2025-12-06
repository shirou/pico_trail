# NFR-iuk5h Navigation Update Rate

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-zd6uw-mission-execution](../analysis/AN-zd6uw-mission-execution.md)
- Related ADRs:
  - [ADR-2hs12-unified-waypoint-navigation](../adr/ADR-2hs12-unified-waypoint-navigation.md)
- Prerequisite Requirements:
  - [FR-2vbe8-navigation-controller](FR-2vbe8-navigation-controller.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-c26bh-unified-waypoint-navigation](../tasks/T-c26bh-unified-waypoint-navigation/README.md)

## Requirement Statement

The navigation system shall update navigation calculations (steering/throttle) at a rate of 50Hz (20ms interval) to ensure smooth vehicle control.

## Rationale

Navigation update rate directly affects control quality:

- **Too slow**: Jerky steering, delayed response to heading errors
- **Too fast**: Unnecessary CPU load, no additional benefit

50Hz is standard for autopilot systems:

- ArduPilot main loop runs at 50Hz (Rover) or 400Hz (Copter)
- Matches typical servo/ESC PWM update rate
- Allows responsive steering corrections
- Leaves CPU headroom for other tasks

## User Story (if applicable)

The system shall maintain 50Hz navigation updates to ensure smooth and responsive steering control during waypoint navigation.

## Acceptance Criteria

- [ ] navigation_task executes every 20ms
- [ ] No navigation update exceeds 25ms (jitter tolerance)
- [ ] CPU utilization for navigation < 10%
- [ ] Steering output changes smoothly (no >0.2 step changes)
- [ ] Timing verified via performance counters or logging

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance Targets:**

| Metric              | Target | Tolerance | Measurement Method        |
| ------------------- | ------ | --------- | ------------------------- |
| Update rate         | 50Hz   | ±2Hz      | Timer interrupt frequency |
| Update latency      | 20ms   | ±5ms      | Task execution time       |
| CPU utilization     | <10%   | -         | Performance monitoring    |
| Steering smoothness | -      | ±0.2/step | Output delta measurement  |

**Task Implementation:**

```rust
#[embassy_executor::task]
async fn navigation_task() {
    let mut ticker = Ticker::every(Duration::from_millis(20));

    loop {
        ticker.next().await;

        // Navigation calculations here
        // Must complete within 20ms
    }
}
```

**Timing Budget (20ms total):**

| Operation          | Budget | Notes                    |
| ------------------ | ------ | ------------------------ |
| GPS state read     | 0.1ms  | Mutex lock               |
| Mission state read | 0.1ms  | Mutex lock               |
| Navigation calc    | 1-2ms  | Trig functions (FPU)     |
| Output write       | 0.1ms  | Mutex lock               |
| Margin             | 17ms   | Available for other work |

**Relationship to GPS Update Rate:**

GPS typically updates at 1-10Hz. Navigation controller must interpolate between GPS updates:

- 50Hz nav / 10Hz GPS = 5 nav updates per GPS update
- Use last known GPS position between updates
- Consider dead reckoning with IMU (future enhancement)

## Platform Considerations

### Pico W (RP2040)

- No hardware FPU - trig functions use software emulation
- May need to reduce rate to 25Hz or use lookup tables
- Test actual timing on hardware

### Pico 2 W (RP2350)

- Hardware FPU supports 50Hz without optimization
- Single-precision (f32) sufficient for navigation

### Cross-Platform

Navigation algorithm identical; only timing may vary.

## Risks & Mitigation

| Risk                                | Impact | Likelihood   | Mitigation                           | Validation                 |
| ----------------------------------- | ------ | ------------ | ------------------------------------ | -------------------------- |
| Trig functions exceed timing budget | Medium | Low (RP2350) | Use FPU, optimize if needed          | Profile on target hardware |
| GPS mutex contention                | Medium | Low          | Use try_lock with fallback           | Test under load            |
| Timer drift                         | Low    | Low          | Use Embassy Ticker (interrupt-based) | Long-duration timing test  |

## Implementation Notes

Preferred approaches:

- Use Embassy `Ticker` for precise timing
- Measure execution time in debug builds
- Add performance counters for monitoring

Known pitfalls:

- Don't use `Timer::after()` - causes drift over time
- Avoid blocking operations in navigation task
- GPS reads may block briefly - use timeout

Related code areas:

- `examples/pico_trail_rover.rs` - navigation_task
- `src/subsystems/navigation/` - Navigation controller

## External References

- [Embassy Executor](https://embassy.dev/book/dev/runtime.html)
- [ArduPilot Scheduler](https://ardupilot.org/dev/docs/apmcopter-programming-attitude-control-2.html)
