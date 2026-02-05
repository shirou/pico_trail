# NFR-00095 SITL Sensor-to-Actuator Latency

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Category: Performance

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Related Requirements:
  - [FR-00153-sitl-lockstep-synchronization](FR-00153-sitl-lockstep-synchronization.md)
- Related Tasks:
  - [T-00160-sitl-multi-vehicle-lockstep-ci](../tasks/T-00160-sitl-multi-vehicle-lockstep-ci/README.md)

## Requirement Statement

SITL sensor-to-actuator latency shall be less than 20ms at the 99th percentile in lockstep mode. This is the time from receiving sensor data to sending actuator commands.

## Rationale

Control loop stability depends on bounded latency. The real hardware targets 50Hz (20ms) control loop. SITL should match or exceed this to ensure simulated behavior reflects real-world timing.

## Category-Specific Details

### Performance Requirements

- Latency measured from `receive_sensors()` completion to `send_actuators()` call
- Includes sensor injection, control loop execution, and actuator collection
- Excludes network I/O time (simulator side)

## Acceptance Criteria

- [ ] Latency <20ms at 99th percentile (p99)
- [ ] Latency <10ms at 50th percentile (p50)
- [ ] No latency spikes >50ms (p99.9)
- [ ] Performance consistent across 1-10 vehicles
- [ ] Benchmark tests measure and report latency distribution
- [ ] CI fails if latency exceeds threshold

## Technical Details (if applicable)

### Measurement Points

```text
Time T0: receive_sensors() returns SensorData
         │
         ├── inject_sensors() to platform
         │
         ├── run_control_loop() (50Hz iteration)
         │
         └── collect_actuator_commands()
         │
Time T1: send_actuators() called

Latency = T1 - T0
```

### Benchmark Implementation

```rust
#[bench]
fn bench_sitl_latency(b: &mut Bencher) {
    let mut bridge = setup_sitl_bridge();
    let sensor_data = generate_test_sensors();

    b.iter(|| {
        let start = Instant::now();

        bridge.inject_sensors(&sensor_data);
        bridge.run_control_loop();
        let _commands = bridge.collect_actuator_commands();

        start.elapsed()
    });
}

#[test]
fn test_latency_under_threshold() {
    let mut latencies = Vec::new();

    for _ in 0..1000 {
        let latency = measure_single_iteration();
        latencies.push(latency);
    }

    latencies.sort();
    let p99 = latencies[990];

    assert!(p99 < Duration::from_millis(20), "p99 latency {} exceeds 20ms", p99.as_millis());
}
```

### Performance Budget

| Component                       | Budget    |
| ------------------------------- | --------- |
| Sensor injection                | <1ms      |
| Control loop (navigation, AHRS) | <15ms     |
| Actuator collection             | <1ms      |
| Overhead (routing, dispatch)    | <3ms      |
| **Total**                       | **<20ms** |

## Measurement / Validation

| Metric        | Target | Measurement Method         |
| ------------- | ------ | -------------------------- |
| p50 latency   | <10ms  | Benchmark 1000 iterations  |
| p99 latency   | <20ms  | Benchmark 1000 iterations  |
| p99.9 latency | <50ms  | Benchmark 10000 iterations |

## Risks & Mitigation

| Risk                             | Impact | Likelihood | Mitigation                                        |
| -------------------------------- | ------ | ---------- | ------------------------------------------------- |
| GC pauses (if using allocations) | Medium | Low        | Minimize allocations in hot path                  |
| Lock contention                  | Medium | Medium     | Use lock-free structures where possible           |
| Debug build slowdown             | Low    | High       | Document release build requirement for benchmarks |

## Implementation Notes

- Measure in release mode only — debug builds are significantly slower
- Consider adding latency histograms to SITL logging
- Alert if latency degrades over time (regression detection)

## External References

- [Control Loop Timing](https://ardupilot.org/dev/docs/apmcopter-code-overview.html#the-main-loop)
