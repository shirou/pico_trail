# NFR-00080 Circle Path Calculation Performance

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00033-circle-mode](../analysis/AN-00033-circle-mode.md)
- Prerequisite Requirements:
  - [FR-00112-circle-mode-implementation](FR-00112-circle-mode-implementation.md)
- Dependent Requirements: N/A
- Related ADRs:
  - [ADR-00029-circle-mode-path-generation](../adr/ADR-00029-circle-mode-path-generation.md)
- Related Tasks:
  - [T-00028-circle-mode-implementation](../tasks/T-00028-circle-mode-implementation/README.md)

## Requirement Statement

The circle path target point calculation shall complete within 1ms per update cycle and consume no more than 64 bytes of stack memory, ensuring it does not impact the navigation controller update rate.

## Rationale

Circle mode calculations run within the navigation update loop. The hybrid approach generates continuous target points fed to the L1 controller, requiring efficient computation:

- Target point calculation includes trigonometric functions (sin, cos, atan2)
- Geodetic offset calculation involves multiple floating-point operations
- Must complete before navigation controller starts processing

The 1ms budget is half of the navigation controller's 2ms budget (NFR-00069), ensuring Circle mode overhead is minimal.

## User Story (if applicable)

The system shall calculate circle target points within 1ms to ensure Circle mode does not degrade navigation controller performance.

## Acceptance Criteria

- [ ] Circle target calculation completes within 1ms on RP2350
- [ ] Circle target calculation completes within 2ms on RP2040
- [ ] Stack usage does not exceed 64 bytes
- [ ] No heap allocations during target calculation
- [ ] Memory overhead for circle state does not exceed 48 bytes
- [ ] Performance measured via profiling in release build

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance Targets:**

| Platform | Max Execution Time | Rationale                 |
| -------- | ------------------ | ------------------------- |
| RP2350   | 1ms                | Half of navigation budget |
| RP2040   | 2ms                | Relaxed for software FPU  |

**Memory Targets:**

| Resource     | Limit | Rationale                         |
| ------------ | ----- | --------------------------------- |
| Stack        | 64B   | Fits within navigation call stack |
| Circle State | 48B   | Center (16B) + params (32B)       |
| Heap         | 0B    | No dynamic allocation             |

**Circle State Structure:**

```rust
struct CircleState {
    center: GpsPosition,     // 16 bytes (2x f64)
    radius: f32,             // 4 bytes
    speed: f32,              // 4 bytes
    direction: CircleDir,    // 1 byte
    // padding: 7 bytes
}
// Total: ~32 bytes
```

**Optimization Strategies (if needed):**

1. Cache sin/cos of common angles
2. Use fast inverse sqrt for normalization
3. Reduce calculation frequency to 10Hz (interpolate between)
4. Pre-compute angular velocity on mode entry

**Measurement Methodology:**

```rust
let start = timer.now();
let target = circle.calculate_target(&current_pos);
let elapsed = timer.now() - start;
defmt::info!("Circle calc: {} us", elapsed.as_micros());
```

## Platform Considerations

### Pico W (RP2040)

- No hardware FPU - software floating-point adds latency
- 2ms budget instead of 1ms
- Consider lookup tables for sin/cos if needed
- Trigonometric functions are the primary bottleneck

### Pico 2 W (RP2350)

- Hardware FPU handles all floating-point efficiently
- 1ms budget achievable with standard calculations
- No special optimizations expected

### Cross-Platform

- Same algorithm, different performance budgets
- Profile on actual hardware, not host
- Release build optimization essential

## Risks & Mitigation

| Risk                          | Impact | Likelihood | Mitigation                   | Validation              |
| ----------------------------- | ------ | ---------- | ---------------------------- | ----------------------- |
| Exceeds time budget on RP2040 | Medium | Medium     | Use lookup tables for trig   | Profile on hardware     |
| Stack overflow in calculation | High   | Low        | Minimize local variables     | Stack usage measurement |
| Repeated calculation overhead | Low    | Low        | Cache results where possible | Profile hot path        |

## Implementation Notes

Preferred approaches:

- Inline simple helper functions
- Pre-compute angular velocity on mode entry
- Avoid temporary allocations
- Use `#[inline(always)]` for hot path functions

Known pitfalls:

- Debug builds are significantly slower
- `libm` functions are already reasonably optimized
- Avoid logging in calculation loop

Related code areas:

- `src/rover/mode/circle.rs` - Circle target calculation
- `src/core/math/geodetic.rs` - Offset calculation
- `src/subsystems/navigation/controller.rs` - Consumer of targets

## External References

- [Embassy Executor Documentation](https://embassy.dev/)
- [RP2350 Datasheet - FPU Section](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
