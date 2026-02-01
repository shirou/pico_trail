# NFR-00041 Mode Update Completion Within 5 Milliseconds

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P0
- Category: Performance

## Links

- Parent Analysis: [AN-00014-mode-lifecycle-management](../analysis/AN-00014-mode-lifecycle-management.md)
- Related Requirements: [FR-00048-mode-lifecycle-management](FR-00048-mode-lifecycle-management.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Mode update shall complete within 5ms per call to ensure 50 Hz control loop period of 20 ms is maintained, allowing 15ms for other tasks (sensor reads, actuator output, navigation).

## Rationale

50 Hz control loop requires 20 ms period. Mode update is subset of loop, must leave time for sensors, navigation, actuators. Slow update causes loop overrun, degrading control quality.

## Measurement Criteria

- Update timing: Measure from update() call start to return
- Target: < 5ms mean, < 10ms 99th percentile
- Measurement method: Timestamp before/after update() call
- Monitor per mode across extended operation

## Acceptance Criteria

1. Update timing measurement:
   - Instrument update() with timestamps
   - Calculate duration = end_time - start_time
   - Log slow updates (> 5ms) with warning
2. Performance targets:
   - Mean update time: < 5ms
   - 99th percentile: < 10ms
   - Maximum: < 15ms (avoid loop overrun)
3. Performance monitoring:
   - Real-time update timing tracking
   - Alert if update exceeds 5ms threshold
   - Log timing statistics (mean, max, p99)
4. Optimization strategies:
   - Optimize hot paths in mode logic
   - Minimize navigation computation per cycle
   - Cache expensive calculations

## Success Metrics

- Mean update time < 5ms per mode
- 50 Hz control loop maintained (no overruns)
- Consistent control loop timing
- No control quality degradation

## Verification Methods

- Micro-benchmarks: Measure update() per mode
- HITL testing: Profile in control loop on hardware
- Long-duration testing: 1+ hour operation, monitor timing
- Stress testing: Update timing under high sensor noise

## ArduPilot Comparison

ArduPilot mode update typically completes in:

- Manual/Hold: < 1ms (simple)
- Auto/RTL: 2-4ms (navigation)
- 50 Hz control loop maintained consistently

## Notes

- Target hardware: RP2350 @ 150 MHz
- Budget: < 750K CPU cycles @ 150 MHz = 5ms
- Navigation computation is typically slowest part
- Phase 1: Basic modes (Manual, Hold); Phase 2: Navigation modes
