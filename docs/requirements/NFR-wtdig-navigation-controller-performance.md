# NFR-wtdig Navigation Controller Performance

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-27568-position-target-navigation](../analysis/AN-27568-position-target-navigation.md)
- Prerequisite Requirements:
  - [FR-2vbe8-navigation-controller](../requirements/FR-2vbe8-navigation-controller.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-tto4f-navigation-controller](../tasks/T-tto4f-navigation-controller/README.md)

## Requirement Statement

The navigation controller shall execute within 2ms per update cycle and consume no more than 256 bytes of stack memory, ensuring it does not impact the 50Hz control loop.

## Rationale

Navigation calculations run within the main control loop at 50Hz (20ms period). If navigation takes too long, it will:

- Delay actuator commands, causing jerky movement
- Accumulate latency, degrading navigation accuracy
- Potentially cause watchdog timeouts

The 2ms limit provides headroom for other control loop tasks while maintaining responsive navigation.

## User Story (if applicable)

The system shall execute navigation calculations within 2ms to ensure consistent 50Hz control loop timing and responsive vehicle behavior.

## Acceptance Criteria

- [ ] Navigation controller update completes within 2ms on RP2350
- [ ] Navigation controller update completes within 5ms on RP2040 (relaxed due to no FPU)
- [ ] Stack usage does not exceed 256 bytes
- [ ] No heap allocations during navigation update
- [ ] Performance measured via profiling in release build

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance Targets:**

| Platform | Max Execution Time | Rationale                   |
| -------- | ------------------ | --------------------------- |
| RP2350   | 2ms                | 10% of 20ms control period  |
| RP2040   | 5ms                | 25% budget for software FPU |

**Memory Targets:**

| Resource | Limit | Rationale                |
| -------- | ----- | ------------------------ |
| Stack    | 256B  | Fits within control task |
| Heap     | 0B    | No dynamic allocation    |
| Static   | 128B  | Controller state storage |

**Measurement Methodology:**

```rust
// Timing measurement
let start = timer.now();
let output = controller.update(&current, &target, dt);
let elapsed = timer.now() - start;
assert!(elapsed < Duration::from_micros(2000));
```

**Optimization Strategies (if needed):**

1. Use lookup tables for sin/cos if CPU-bound
2. Reduce calculation frequency (10Hz instead of 50Hz)
3. Use fixed-point math on RP2040
4. Cache expensive calculations between updates

## Platform Considerations

### Pico W (RP2040)

- No hardware FPU - software floating-point is slower
- 5ms execution time budget (relaxed)
- May need lookup tables or fixed-point optimization
- Consider 10Hz navigation update with interpolation

### Pico 2 W (RP2350)

- Hardware FPU supports fast floating-point
- 2ms execution time budget
- Full 50Hz navigation updates feasible

### Cross-Platform

- Same algorithm, different performance budgets
- Profile on target hardware, not host
- Release build optimization essential

## Risks & Mitigation

| Risk                           | Impact | Likelihood | Mitigation                        | Validation              |
| ------------------------------ | ------ | ---------- | --------------------------------- | ----------------------- |
| Exceeds time budget on RP2040  | Medium | Medium     | Reduce update rate to 10Hz        | Profile on hardware     |
| Stack overflow in control task | High   | Low        | Static analysis, limit local vars | Stack usage measurement |
| Heap allocation in hot path    | Medium | Low        | Code review, no_alloc attribute   | Memory profiling        |

## Implementation Notes

Preferred approaches:

- Avoid allocations in update() method
- Pre-compute constants at initialization
- Use `#[inline]` for small helper functions
- Profile with release build, not debug

Known pitfalls:

- Debug builds are significantly slower
- `libm` transcendental functions are already optimized
- Avoid excessive logging in hot path

Related code areas:

- `src/subsystems/navigation/controller.rs` - Navigation controller
- `src/rover/mode/` - Modes that call navigation

## External References

- [Embassy Executor Documentation](https://embassy.dev/)
- [RP2350 Datasheet - FPU Section](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
