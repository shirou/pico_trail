# NFR-ukx3a Network Transport Memory Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [NFR-z2iuk-memory-limits](NFR-z2iuk-memory-limits.md)
  - [FR-ydttj-udp-network-transport](FR-ydttj-udp-network-transport.md)
  - [FR-6jkia-transport-abstraction](FR-6jkia-transport-abstraction.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-oq110-mavlink-network-transport](../tasks/T-oq110-mavlink-network-transport/README.md)

## Requirement Statement

The network transport implementation (WiFi driver + UDP stack + transport abstraction) shall consume no more than 50 KB of RAM and 100 KB of Flash memory to ensure sufficient resources remain for core autopilot functions.

## Rationale

Memory constraints are critical on embedded platforms:

- **Pico W (RP2040)**: 264 KB RAM total → 50 KB = 19% overhead (acceptable)
- **Pico 2 W (RP2350)**: 520 KB RAM total → 50 KB = 9.6% overhead (comfortable)
- **Core Functions**: AHRS, control loops, sensor fusion need majority of RAM
- **Flash Space**: RP2040 has 2 MB Flash, RP2350 has 4 MB (100 KB is 5-2.5%)

Exceeding 50 KB RAM would compromise core autopilot functionality or force use of Pico 2 W exclusively.

## User Story (if applicable)

The system shall limit network transport memory usage to 50 KB RAM and 100 KB Flash to ensure adequate resources for attitude estimation, control loops, sensor processing, and other core autopilot functions on both Pico W and Pico 2 W platforms.

## Acceptance Criteria

- [ ] Total static RAM allocation ≤ 50 KB (WiFi driver + network stack + buffers)
- [ ] Total Flash usage ≤ 100 KB (compiled code + firmware blobs)
- [ ] Heap allocation: 0 bytes (no dynamic allocation, use static/stack only)
- [ ] Stack usage per task ≤ 4 KB (WiFi task + network task)
- [ ] Memory usage measured and documented
- [ ] Memory profiling performed on Pico W (worst case) and Pico 2 W
- [ ] No memory leaks under sustained operation (24-hour test)
- [ ] Core autopilot functions maintain required memory budget after network transport added

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Memory Budget Breakdown** (target):

| Component             | RAM (KB) | Flash (KB) |
| --------------------- | -------- | ---------- |
| CYW43439 WiFi driver  | 20       | 30         |
| embassy-net stack     | 15       | 40         |
| UDP socket buffers    | 4        | 5          |
| Transport abstraction | 2        | 10         |
| GCS endpoint tracking | 1        | 2          |
| Message deduplication | 1        | 2          |
| WiFi firmware blobs   | 0        | 10         |
| **Total**             | **43**   | **99**     |
| **Margin**            | 7 KB     | 1 KB       |

**Measurement Strategy**:

```rust
// Static RAM measurement
#[link_section = ".bss"]
static mut WIFI_BUFFERS: [u8; 20480] = [0; 20480];  // 20 KB

#[link_section = ".bss"]
static mut NET_STACK_RESOURCES: embassy_net::StackResources<2> = /* ... */;  // ~15 KB

// Compile-time size reporting
fn report_sizes() {
    defmt::info!("WiFi buffers: {} bytes", size_of_val(&WIFI_BUFFERS));
    defmt::info!("Stack resources: {} bytes", size_of_val(&NET_STACK_RESOURCES));
}
```

**Profiling Tools**:

```bash
# Flash usage
cargo size --release -- -A | grep .text

# RAM usage (static + stack)
cargo size --release -- -A | grep -E '.bss|.data'

# Stack usage per task
probe-rs run --chip RP2350 target/release/example
# Monitor defmt logs for stack high-water marks
```

**Optimization Strategies**:

If exceeding budget:

1. **Reduce WiFi buffers**: CYW43439 default 20 KB → try 16 KB
2. **Limit network stack resources**: Reduce concurrent sockets (4 → 2)
3. **Conditional compilation**: Disable TCP if UDP-only
4. **Optimize GCS tracking**: Reduce max GCS from 4 to 2

**Continuous Monitoring**:

```rust
#[embassy_executor::task]
async fn memory_monitor_task() {
    loop {
        let free_ram = get_free_ram();  // Platform-specific
        defmt::info!("Free RAM: {} KB", free_ram / 1024);

        if free_ram < 50 * 1024 {
            defmt::warn!("Low memory: {} KB free", free_ram / 1024);
        }

        Timer::after(Duration::from_secs(60)).await;
    }
}
```

## Platform Considerations

### Unix

N/A - Platform agnostic (embedded system)

### Windows

N/A - Platform agnostic (embedded system)

### Cross-Platform

**Pico W (RP2040)**: 264 KB RAM - Critical constraint, requires profiling

**Pico 2 W (RP2350)**: 520 KB RAM - More headroom, but same limits for cross-platform compatibility

## Risks & Mitigation

| Risk                              | Impact | Likelihood | Mitigation                                       | Validation                             |
| --------------------------------- | ------ | ---------- | ------------------------------------------------ | -------------------------------------- |
| Exceeds 50 KB RAM budget          | High   | Medium     | Profile early, optimize buffers, reduce features | Measure with cargo size + runtime logs |
| Memory fragmentation (heap)       | Medium | Low        | No heap allocation (static only)                 | Verify zero heap usage                 |
| Stack overflow (deep call chains) | High   | Low        | 4 KB per task, monitor high-water marks          | Stress test with max message rates     |
| Flash overflow (firmware blobs)   | Low    | Low        | CYW43439 firmware \~10 KB (known)                | Check final binary size                |
| Memory leak in WiFi driver        | High   | Low        | Use proven cyw43 crate, long-duration testing    | 24-hour soak test, monitor RAM usage   |

## Implementation Notes

Preferred approaches:

- Use `cargo size` for compile-time measurement
- Add runtime memory monitoring task (defmt logs)
- Profile on Pico W (worst case)
- Use `heapless` collections (no heap)
- Minimize static buffers

Known pitfalls:

- CYW43439 requires large buffers (\~20 KB) - non-negotiable
- embassy-net stack allocates resources statically
- Rust `size_of_val` doesn't include padding/alignment
- Stack usage hard to measure (need high-water mark tracking)

Related code areas:

- `src/platform/*/network.rs` - WiFi driver buffers
- Memory analysis: `cargo size`, `probe-rs` runtime logs

Suggested libraries:

- `heapless` - Zero-allocation collections
- `defmt` - Lightweight logging for size tracking

## External References

- RP2040 Datasheet (memory specs): <https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf>
- RP2350 Datasheet (memory specs): <https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf>
- Embassy Resource Requirements: <https://embassy.dev/book/dev/faq.html#how-much-ram-flash-does-embassy-use>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
