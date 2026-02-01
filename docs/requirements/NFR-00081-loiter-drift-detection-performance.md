# NFR-00081 Loiter Mode Drift Detection Performance

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00118-rover-loiter-mode](FR-00118-rover-loiter-mode.md)
  - [FR-00117-position-drift-detection](FR-00117-position-drift-detection.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-00029-rover-loiter-mode](../tasks/T-00029-rover-loiter-mode/README.md)

## Requirement Statement

The Loiter mode drift detection calculation shall complete within 1ms per update cycle, ensuring no impact on the 50 Hz control loop timing.

## Rationale

The drift detection runs within the main control loop at up to 50 Hz. If the calculation takes too long, it will delay other control tasks and potentially cause timing violations. The 1ms budget leaves 19ms for other processing in the 20ms (50 Hz) cycle.

## User Story

The system shall maintain low-latency drift detection to ensure control loop timing is not impacted during Loiter mode operation.

## Acceptance Criteria

- [ ] Haversine distance calculation completes in < 500us on RP2350
- [ ] Hysteresis state check completes in < 100us
- [ ] Total drift detection (distance + state) completes in < 1ms
- [ ] No dynamic memory allocation during drift detection
- [ ] Performance verified via benchmark tests on target hardware

## Technical Details

### Non-Functional Requirement Details

**Performance Breakdown:**

| Operation                | Target Latency | Notes               |
| ------------------------ | -------------- | ------------------- |
| Haversine calculation    | < 500us        | Uses sin/cos/sqrt   |
| Hysteresis state machine | < 100us        | Simple comparisons  |
| Parameter read           | < 50us         | Read LOIT_RADIUS    |
| Total drift detection    | < 1ms          | Combined operations |

**Measurement Method:**

```rust
#[cfg(test)]
fn benchmark_drift_detection() {
    let start = embassy_time::Instant::now();

    for _ in 0..1000 {
        let distance = distance_to_loiter(&current, &loiter);
        let _ = check_correction_needed(&mut state, &current);
    }

    let elapsed = start.elapsed().as_micros();
    let per_iteration = elapsed / 1000;

    assert!(per_iteration < 1000, "Drift detection too slow: {}us", per_iteration);
}
```

**Optimization Strategies:**

1. Pre-compute trigonometric values where possible
2. Use `f32` instead of `f64` (sufficient precision, faster on ARM)
3. Inline hot path functions
4. Avoid allocations in calculation loop

## Platform Considerations

- **RP2350**: Primary target, ARM Cortex-M33 @ 150MHz
- **Performance**: FPU available for floating-point operations
- **Measurement**: Use `embassy_time::Instant` for timing

## Risks & Mitigation

| Risk                      | Impact | Likelihood | Mitigation                   | Validation              |
| ------------------------- | ------ | ---------- | ---------------------------- | ----------------------- |
| FPU disabled              | High   | Low        | Ensure FPU enabled in build  | Check build config      |
| Algorithm inefficiency    | Medium | Low        | Use optimized math functions | Profile on target       |
| Compiler optimization off | Medium | Low        | Use release builds           | Benchmark release build |

## Implementation Notes

- Benchmark on actual RP2350 hardware, not host simulation
- Use `#[inline(always)]` for hot path functions
- Consider using CORDIC or lookup tables if FPU performance insufficient
- Profile with `defmt` timing annotations during development

## External References

- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - ARM Cortex-M33 FPU specifications
- [Embedded Rust Performance](https://docs.rust-embedded.org/book/unsorted/speed-vs-size.html) - Optimization guidance
