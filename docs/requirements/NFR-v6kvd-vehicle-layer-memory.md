# NFR-v6kvd Vehicle Layer Memory Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [NFR-z2iuk-memory-limits](NFR-z2iuk-memory-limits.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-3irm5-manual-control-implementation](../tasks/T-3irm5-manual-control-implementation/README.md)

## Requirement Statement

The vehicle control layer (modes, actuators, RC input, mode manager) shall consume no more than 5 KB RAM and 20 KB Flash to maintain sufficient memory budget for other autopilot subsystems (AHRS, navigation, logging) on resource-constrained platforms.

## Rationale

Memory budget allocation for Pico W (RP2040):

- **Total RAM**: 264 KB
- **Reserved**: 64 KB (Embassy runtime, network stack, buffers)
- **Available**: 200 KB for autopilot subsystems
- **Vehicle layer target**: 5 KB (2.5% of available)
- **Remaining**: 195 KB for AHRS, navigation, logging, etc.

Vehicle layer is foundational but must remain lean to leave headroom for:

- AHRS/DCM (8-10 KB)
- Navigation/waypoint following (10-15 KB)
- MAVLink buffers (8-10 KB)
- Logger (4-16 KB)
- Future features

## User Story (if applicable)

The system shall implement vehicle control layer with minimal memory footprint (< 5 KB RAM) to ensure sufficient memory remains for autonomous navigation, sensor fusion, and data logging subsystems.

## Acceptance Criteria

- [ ] Vehicle layer RAM usage ≤ 5 KB (measured at runtime)
- [ ] Vehicle layer Flash usage ≤ 20 KB (measured via `cargo size`)
- [ ] RC input state: ≤ 500 bytes (18 channels + metadata)
- [ ] Mode manager: ≤ 1 KB (mode state + manager)
- [ ] Actuator layer: ≤ 500 bytes (actuator config + state)
- [ ] Manual mode implementation: ≤ 200 bytes (simple pass-through)
- [ ] No dynamic heap allocation in control loop (use stack or static allocation)
- [ ] Memory profiling performed on both Pico W (worst case) and Pico 2 W
- [ ] Memory usage regression test: fail build if > 5 KB threshold

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Memory Breakdown (Target):**

| Component             | RAM Usage    | Notes                                  |
| --------------------- | ------------ | -------------------------------------- |
| RC input state        | \~500 B      | 18 channels × 4 bytes (f32) + metadata |
| Mode manager          | \~1 KB       | Mode trait object + state              |
| Actuator layer        | \~500 B      | Actuator config + calibration          |
| Manual mode           | \~200 B      | Simple pass-through logic              |
| Vehicle control task  | \~2 KB       | Task stack                             |
| Mode trait vtable     | \~200 B      | Dynamic dispatch overhead              |
| **Total (estimated)** | **\~4.4 KB** | Comfortable margin to 5 KB target      |

**Flash Breakdown (Target):**

| Component                  | Flash Usage | Notes                          |
| -------------------------- | ----------- | ------------------------------ |
| RC input processing        | \~2 KB      | Parsing + normalization        |
| Mode manager + trait       | \~3 KB      | Mode switching logic           |
| Manual mode implementation | \~2 KB      | Simple control logic           |
| Actuator layer             | \~3 KB      | PWM conversion + safety checks |
| Vehicle control task       | \~2 KB      | Task initialization + loop     |
| Mode trait implementations | \~5 KB      | Trait dispatch + vtables       |
| Safety checks              | \~3 KB      | Armed check + fail-safe logic  |
| **Total (estimated)**      | **\~20 KB** | At target limit                |

**Memory Profiling Strategy:**

1. **Compile-time analysis**: `cargo size --release` for Flash usage
2. **Runtime measurement**: Track stack high-water mark via Embassy
3. **Static allocation**: Count all `static` and `static mut` variables
4. **Heap allocation**: Use `#![no_std]` to prevent accidental heap usage

**Memory Optimization Techniques:**

- **Stack allocation**: Prefer stack for temporary data (< 1 KB per function)
- **Static allocation**: Use `static` for shared state (RC input, system state)
- **Zero-copy**: Pass references, avoid copying large structs
- **Trait dispatch**: Use static dispatch where possible (monomorphization)
- **Compact representations**: Use `u16` for PWM values, `f32` for normalized values

**Memory Budget Allocation:**

```
Pico W (RP2040) - 264 KB RAM:
├─ Embassy runtime:         32 KB  (12%)
├─ Network stack (WiFi):    32 KB  (12%)
├─ Vehicle layer:            5 KB  ( 2%) ← This requirement
├─ AHRS/DCM:                10 KB  ( 4%)
├─ Navigation:              15 KB  ( 6%)
├─ MAVLink buffers:         10 KB  ( 4%)
├─ Logger:                  10 KB  ( 4%)
├─ Parameters:               2 KB  ( 1%)
├─ Task stacks:             20 KB  ( 8%)
├─ Reserved headroom:       50 KB  (19%)
└─ Remaining:               78 KB  (30%) for future features
```

**Regression Testing:**

```rust
#[test]
fn test_vehicle_layer_memory_budget() {
    let vehicle_layer_size = measure_ram_usage(&[
        size_of::<RcInput>(),
        size_of::<ModeManager>(),
        size_of::<Actuators>(),
        size_of::<ManualMode>(),
    ]);

    const MAX_RAM: usize = 5 * 1024; // 5 KB
    assert!(
        vehicle_layer_size <= MAX_RAM,
        "Vehicle layer RAM usage ({} bytes) exceeds limit ({} bytes)",
        vehicle_layer_size,
        MAX_RAM
    );
}
```

**Flash Size Monitoring:**

```bash
# Measure Flash usage after each build
cargo size --release -- -A | grep ".text"

# Example output:
# .text  120000   # Total code size
# Vehicle layer contribution: ~20 KB
```

## Platform Considerations

### Pico W (RP2040)

- Total RAM: 264 KB (strict constraint)
- Vehicle layer: 5 KB target (2.5% of total)
- Flash: 2 MB (less constrained, 20 KB target acceptable)

### Pico 2 W (RP2350)

- Total RAM: 520 KB (more headroom)
- Vehicle layer: 5 KB target (< 1% of total, very comfortable)
- Flash: 4 MB (plenty of space)

### Cross-Platform

Memory target applies to both platforms. Pico W is worst case (tightest constraint).

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                               | Validation                                   |
| ----------------------------------------- | ------ | ---------- | -------------------------------------------------------- | -------------------------------------------- |
| Memory usage exceeds 5 KB (feature creep) | High   | Medium     | Regular profiling, reject features that exceed budget    | Automated regression test on every build     |
| Dynamic heap allocation in control loop   | High   | Low        | Use `#![no_std]`, avoid `Box`, `Vec`, `String`           | Code review: verify no heap allocation       |
| Stack overflow (large local variables)    | High   | Low        | Limit function stack usage to < 1 KB, use static storage | Stack high-water mark monitoring             |
| Mode trait overhead (vtable bloat)        | Medium | Low        | Use static dispatch where possible, measure vtable size  | Compare dynamic dispatch vs monomorphization |

## Implementation Notes

Preferred approaches:

- **Static allocation**: Use `static` for shared state (RC input, actuator config)
- **Stack allocation**: Local variables for temporary computation
- **Zero heap allocation**: Never use `Box`, `Vec`, `String` in control loop
- **Profile early**: Measure memory usage during implementation, not after

Known pitfalls:

- **Hidden allocations**: Trait objects (`Box<dyn Trait>`) allocate on heap
- **Large stack frames**: Copying large structs on stack (use references)
- **Debug vs release**: Debug builds use more memory (always measure release builds)
- **Embassy overhead**: Task stacks contribute to RAM usage

Related code areas:

- `src/vehicle/` - All vehicle layer components
- `Cargo.toml` - Profile settings (optimize for size with `opt-level = "z"`)
- `memory.x` - Linker script (stack/heap size configuration)

## External References

- Embedded Rust Memory Optimization: <https://docs.rust-embedded.org/book/static-guarantees/zero-cost-abstractions.html>
- Embassy Memory Usage: <https://embassy.dev/book/dev/basic_application.html>
- Cortex-M Memory Layout: <https://interrupt.memfault.com/blog/how-to-write-linker-scripts-for-firmware>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
