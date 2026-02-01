# NFR-00003 Memory Usage Limits

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements: N/A
- Dependent Requirements:
  - [NFR-00007-network-memory-overhead](NFR-00007-network-memory-overhead.md)
  - [NFR-00011-vehicle-layer-memory](NFR-00011-vehicle-layer-memory.md)
- Related Tasks:
  - [T-00005-ahrs-dcm-implementation](../tasks/T-00005-ahrs-dcm-implementation/README.md)

## Requirement Statement

The system shall operate within 200 KB RAM on Pico W and 400 KB RAM on Pico 2 W, leaving adequate headroom for future features and stack growth while supporting all core autopilot functionality.

## Rationale

Memory constraints are critical for embedded systems:

- **Pico W**: 264 KB RAM total - Leave 64 KB headroom for stack growth, debugging
- **Pico 2 W**: 520 KB RAM total - Leave 120 KB headroom for future features

Exceeding these limits leads to:

- Stack overflow (difficult to debug, causes crashes)
- Heap exhaustion (in async executors)
- Feature cuts (cannot add new capabilities)

Setting explicit limits ensures the system remains maintainable and extensible.

## User Story (if applicable)

The system shall use no more than 200 KB RAM on Pico W and 400 KB RAM on Pico 2 W to ensure stable operation with adequate margin for stack usage, temporary buffers, and future feature additions.

## Acceptance Criteria

- [ ] Total RAM usage ≤ 200 KB on Pico W (measured via runtime memory profiling)
- [ ] Total RAM usage ≤ 400 KB on Pico 2 W (measured via runtime memory profiling)
- [ ] Stack usage ≤ 32 KB per task (measured via stack painting)
- [ ] Heap usage ≤ 16 KB (if heap used at all, prefer static allocation)
- [ ] No heap allocation in control loops (enforced via code review)
- [ ] Memory usage documented per subsystem in design documents
- [ ] Build fails if `.bss` + `.data` sections exceed limits (linker script check)

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Memory Budget (Pico W, 264 KB RAM):**

| Component       | RAM Usage  | Notes                                 |
| --------------- | ---------- | ------------------------------------- |
| AHRS (DCM)      | 2 KB       | Lightweight attitude estimation       |
| Navigation      | 4 KB       | S-curve planner, waypoint storage     |
| Control (PID)   | 1 KB       | Multiple PID controllers              |
| MAVLink         | 8 KB       | Message buffers + state               |
| Parameters      | 2 KB       | In-memory parameter cache             |
| Logger          | 8 KB       | Circular buffer (small on Pico W)     |
| Task Stacks     | 32 KB      | Multiple task stacks (4 tasks × 8 KB) |
| Device Drivers  | 4 KB       | GPS, IMU, RC buffers                  |
| Mission Storage | 4 KB       | 100 waypoints @ 40 bytes each         |
| **Subtotal**    | **65 KB**  | Core systems                          |
| **Reserve**     | **135 KB** | Additional features, stack headroom   |
| **Total**       | **200 KB** | Target limit                          |

**Memory Budget (Pico 2 W, 520 KB RAM):**

| Component       | RAM Usage  | Notes                                     |
| --------------- | ---------- | ----------------------------------------- |
| AHRS (EKF)      | 8 KB       | 9-state EKF (more capable)                |
| Navigation      | 6 KB       | Larger S-curve buffer                     |
| Control (PID)   | 2 KB       | More controllers (heading, speed, etc.)   |
| MAVLink         | 12 KB      | Larger buffers for higher telemetry rates |
| Parameters      | 4 KB       | More parameters cached                    |
| Logger          | 16 KB      | Larger circular buffer                    |
| Task Stacks     | 64 KB      | More tasks or larger stacks               |
| Device Drivers  | 8 KB       | Additional sensor support                 |
| Mission Storage | 8 KB       | 200 waypoints                             |
| **Subtotal**    | **128 KB** | Core systems                              |
| **Reserve**     | **272 KB** | Future features, stack headroom           |
| **Total**       | **400 KB** | Target limit                              |

**Measurement Methods:**

**Static Analysis (Linker Map):**

```bash
# Check .bss and .data sections in linker map
arm-none-eabi-size -A target/thumbv8m.main-none-eabihf/release/pico_trail
```

**Runtime Profiling:**

```rust
// Stack painting (write known pattern, check how much overwritten)
extern "C" {
    static mut __stack_start: u32;
    static mut __stack_end: u32;
}

fn measure_stack_usage() {
    let stack_size = unsafe {
        (&__stack_end as *const u32 as usize) -
        (&__stack_start as *const u32 as usize)
    };
    // Check how much stack is still 0xDEADBEEF (unused)
}
```

**Heap Profiling (if using allocator):**

```rust
use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 16 * 1024; // 16 KB
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    unsafe { HEAP.init(&mut HEAP_MEM as *const u8 as usize, HEAP_SIZE) }
}
```

## Platform Considerations

### Pico W (RP2040)

264 KB RAM - Tight constraints, requires careful optimization:

- Use DCM instead of EKF (saves 6 KB)
- Smaller MAVLink buffers (8 KB instead of 12 KB)
- Smaller log buffer (8 KB instead of 16 KB)
- Fewer mission waypoints (100 instead of 200)

### Pico 2 W (RP2350)

520 KB RAM - More comfortable, allows advanced features:

- Use EKF for better accuracy
- Larger buffers for higher telemetry rates
- More mission waypoints
- Additional sensors and peripherals

### Cross-Platform

Use conditional compilation to adjust buffer sizes:

```rust
#[cfg(feature = "pico_w")]
const LOG_BUFFER_SIZE: usize = 8 * 1024; // 8 KB

#[cfg(feature = "pico2_w")]
const LOG_BUFFER_SIZE: usize = 16 * 1024; // 16 KB
```

## Risks & Mitigation

| Risk                                   | Impact | Likelihood | Mitigation                                                      | Validation                                                    |
| -------------------------------------- | ------ | ---------- | --------------------------------------------------------------- | ------------------------------------------------------------- |
| Stack overflow due to deep call chains | High   | Medium     | Measure stack usage via painting, increase stack size if needed | Test with maximum nesting (e.g., failsafe during mode change) |
| Memory fragmentation in heap           | Medium | Low        | Avoid heap allocation, use static allocation where possible     | Avoid using heap in critical paths                            |
| Feature additions exceed memory budget | Medium | Medium     | Enforce memory limits in CI, reject PRs that exceed limits      | Monitor memory usage in automated builds                      |
| Inaccurate static analysis             | Low    | Medium     | Complement with runtime profiling, test on actual hardware      | Run stress tests to trigger worst-case usage                  |

## Implementation Notes

**Memory Optimization Techniques:**

- **Static Allocation**: Avoid heap, use `static` or stack allocation
- **Shared Buffers**: Reuse buffers across subsystems when not concurrent
- **Compile-Time Size**: Use `const` for buffer sizes, adjust per platform
- **Feature Flags**: Disable optional features on Pico W to save RAM

**Enforcement:**

Linker script check:

```ld
MEMORY
{
  #ifdef PICO_W
    RAM : ORIGIN = 0x20000000, LENGTH = 200K  /* Fail build if > 200 KB */
  #else
    RAM : ORIGIN = 0x20000000, LENGTH = 400K  /* Fail build if > 400 KB */
  #endif
}
```

Related code areas:

- `memory.x` - Linker script with RAM limits
- `src/platform/*/config.rs` - Platform-specific buffer size configuration
- `.cargo/config.toml` - Build configuration and memory limits

## External References

- Embedded Rust Memory Management: <https://docs.rust-embedded.org/book/c-tips/index.html>
- Stack Painting Technique: <https://interrupt.memfault.com/blog/cortex-m-rtos-context-switching>
