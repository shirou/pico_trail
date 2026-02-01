# NFR-00066 STATUSTEXT Performance and Memory Constraints

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00020-statustext-notifications](../analysis/AN-00020-statustext-notifications.md)
- Prerequisite Requirements:
  - [FR-00073-statustext-public-api](../requirements/FR-00073-statustext-public-api.md)
  - [FR-00072-statustext-message-queue](../requirements/FR-00072-statustext-message-queue.md)
- Dependent Requirements:
  - [NFR-00064-statustext-length-limits](../requirements/NFR-00064-statustext-length-limits.md)
  - [NFR-00065-statustext-nostd](../requirements/NFR-00065-statustext-nostd.md)
- Related Tasks:
  - [T-00014-statustext-implementation](../tasks/T-00014-statustext-implementation/README.md)

## Requirement Statement

The STATUSTEXT message queueing system shall not allocate heap memory and shall complete message generation and queueing operations within 100 microseconds (µs) to ensure predictable memory usage, avoid fragmentation, and prevent blocking real-time scheduler operations running at 400 Hz on embedded systems with limited RAM (264 KB RP2350).

## Rationale

Embedded systems require both predictable memory usage and deterministic real-time performance. Heap allocation introduces fragmentation, unpredictable allocation failures, and increased code size for allocator overhead. The RP2350 has only 264 KB of RAM, making heap management risky. Additionally, the rover's real-time scheduler runs at 400 Hz (every 2.5 ms), and status messages may be generated during time-critical operations. Slow notification calls could introduce jitter or latency in control loops, affecting vehicle stability. Fixed-capacity heapless structures provide compile-time memory guarantees while enabling fast O(1) operations that complete within a 100 µs budget (4% of 2.5 ms cycle).

## User Story

The system shall **use only stack and static storage for STATUSTEXT message queueing and complete all operations in under 100 microseconds** to ensure **predictable memory usage, avoid heap fragmentation, and prevent blocking real-time scheduler and interrupt handlers on resource-constrained embedded targets**.

## Acceptance Criteria

**Memory Constraints:**

- [ ] Message queue uses `heapless::Deque` (no heap allocation)
- [ ] Message text storage uses `heapless::String<200>` (stack-based)
- [ ] No `Vec`, `String`, `Box`, or other heap-allocating types in notification path
- [ ] Queue capacity fixed at compile time (16 messages)
- [ ] Total static memory usage for notifier ≤ 4 KB
- [ ] Zero dynamic allocation detected via `cargo-call-stack` or similar tools
- [ ] Binary built with `--release` has no allocator symbols in `.map` file
- [ ] `cargo clippy` reports no heap allocation warnings

**Performance Constraints:**

- [ ] `send_error()` completes in <100 µs (average case, measured on RP2350 @ 150 MHz)
- [ ] `send_warning()` completes in <100 µs (average case, measured on RP2350 @ 150 MHz)
- [ ] `send_info()` completes in <100 µs (average case, measured on RP2350 @ 150 MHz)
- [ ] All severity-specific functions complete in <100 µs (average case)
- [ ] Worst-case time <150 µs (including lock contention)
- [ ] Queue push operation is O(1) constant time
- [ ] String copy operation completes in <50 µs for 200-char message
- [ ] Performance verified via embedded profiling (Embassy timer or cycle counter)

## Technical Details

### Non-Functional Requirement Details

**Memory Architecture:**

- Memory footprint: \~3.2 KB static (16 messages × 200 bytes)
- Zero allocation overhead (no malloc/free calls)
- Deterministic memory usage (fixed at compile time)
- No allocation failures possible (queue full = explicit drop policy)
- No heap fragmentation over long runtime
- Memory usage bounded and predictable

**Performance Targets:**

| Operation                      | Average Time | Worst-Case Time | Complexity |
| ------------------------------ | ------------ | --------------- | ---------- |
| `send_error("message")`        | <100 µs      | <150 µs         | O(1)       |
| String copy (200 chars)        | <50 µs       | <80 µs          | O(n)       |
| Queue push                     | <20 µs       | <40 µs          | O(1)       |
| Lock acquisition (uncontended) | <10 µs       | <30 µs          | O(1)       |

**Scheduler Impact:**

- 400 Hz scheduler = 2.5 ms period
- 100 µs notification = 4% of period
- Acceptable overhead for occasional notifications
- Multiple notifications in single period still safe (2-3 messages = 200-300 µs = 12%)

**Compatibility:**

- Compatible with `no_std` environments (see NFR-00065)
- Works with or without global allocator
- Safe for use in interrupt handlers (no allocator locks)
- Compatible with embassy async runtime

**Heapless Data Structures:**

```rust
use heapless::{Deque, String};

pub struct StatusNotifier {
    queue: Deque<QueuedMessage, 16>,  // Fixed capacity, no heap
    drops: u32,
}

struct QueuedMessage {
    severity: MavSeverity,
    text: String<200>,  // Stack-based string, no heap
}
```

**Memory Layout:**

- `Deque<QueuedMessage, 16>`: 16 × (1 byte + 200 bytes) = \~3.2 KB
- `drops` counter: 4 bytes
- Total: \~3.2 KB static RAM

**Profiling Approach:**

```rust
use embassy_time::Instant;

let start = Instant::now();
send_error("PreArm: Battery voltage low");
let elapsed = start.elapsed().as_micros();

if elapsed > 100 {
    log_warn!("Slow STATUSTEXT: {} µs", elapsed);
}
```

**Optimization Strategies:**

- Use `heapless::String` to avoid allocation
- Keep queue operations O(1) with `heapless::Deque`
- Minimize lock hold time (copy message, release lock, process later)
- Avoid complex formatting in notification path
- Simple string copy, no formatting in enqueue path
- Defer chunking to dequeue phase (see FR-00071)

## Platform Considerations

### Embedded (RP2350)

- Critical for RP2350 with limited 264 KB RAM and 150 MHz processor
- Heapless structures stored in static RAM
- No runtime allocator required
- Profile using Embassy `Instant::now()` for microsecond timing
- Verify performance under real-time scheduler load
- Test with lock contention scenarios
- Compatible with embassy async runtime

### Host Tests

- Heapless structures work identically on host
- Can verify no allocation via custom allocator hooks
- Memory profiling tools can confirm zero allocations
- Host performance not critical (much faster CPU)
- Can still verify O(1) complexity and no allocation
- Use `std::time::Instant` for profiling in tests

### Cross-Platform

- Heapless crate works on all platforms
- No platform-specific allocation behavior
- Performance target specific to RP2350 embedded target
- Host tests validate algorithm efficiency, not absolute time

## Risks & Mitigation

| Risk                                                          | Impact | Likelihood | Mitigation                                            | Validation                                    |
| ------------------------------------------------------------- | ------ | ---------- | ----------------------------------------------------- | --------------------------------------------- |
| Developer accidentally introduces heap allocation             | High   | Medium     | Use `clippy::disallowed_types` lint for Vec/String    | CI lint checks, code review                   |
| Lock contention causes >150 µs delay                          | High   | Low        | Use try_lock variant for interrupt context            | Stress test with concurrent calls             |
| Heapless types insufficient for requirements                  | Medium | Low        | 200-char String and 16-message queue are ample        | Analysis shows typical messages <100 chars    |
| String copy slower than expected (>50 µs for 200 chars)       | Medium | Low        | Benchmark actual hardware, optimize if needed         | Measure with profiler on RP2350               |
| Scheduler jitter from notification calls                      | Medium | Medium     | Monitor scheduler timing, reduce notification rate    | Measure control loop jitter with oscilloscope |
| Static memory usage too large (>4 KB)                         | Medium | Low        | Profile actual usage, tune queue size if needed       | Measure with `size` tool, linker map          |
| Complex message formatting slows down enqueue                 | Medium | Medium     | Move formatting outside critical path, use simple API | Profile with realistic message strings        |
| Heapless crate compatibility issues with rust-mavlink         | Low    | Low        | Both crates designed for embedded, widely used        | Integration testing                           |
| Performance degrades over time (cache effects, fragmentation) | Low    | Low        | No heap allocation eliminates fragmentation           | Long-running soak test (hours)                |
| Message truncation due to 200-char limit                      | Low    | Medium     | Enforce limit at API level, chunk long messages       | See NFR-00064 for length limits               |

## Implementation Notes

**Preferred Patterns:**

- Use `heapless::Deque` for queue (not `Vec` or `VecDeque`)
- Use `heapless::String` for message text (not `String` or `&str` with allocation)
- Store notifier in static `Mutex<RefCell<StatusNotifier>>` (no Box)
- Simple string copy, no formatting in enqueue path
- Defer chunking to dequeue phase (see FR-00071)
- Use `try_lock()` with fallback for interrupt-safe calls

**Known Pitfalls:**

- Do not use `.to_string()` (allocates) - use `.copy_from_slice()` or `String::from()`
- Do not collect into `Vec` - iterate directly or use fixed-size array
- Do not use `format!()` macro (allocates) - use `write!()` to heapless::String
- Do not format strings during enqueue (move to caller if needed)
- Do not perform chunking during enqueue (defer to transmission)
- Do not hold lock longer than necessary

**Validation Commands:**

```bash
# Check for allocator symbols (should be empty if no heap used)
cargo build --release --target thumbv8m.main-none-eabihf
arm-none-eabi-nm target/thumbv8m.main-none-eabihf/release/pico_trail | grep -i alloc

# Check binary size and memory layout
cargo size --release --target thumbv8m.main-none-eabihf
```

**Profiling Code Example:**

```rust
#[cfg(feature = "pico2_w")]
fn profile_notification() {
    use embassy_time::Instant;

    let start = Instant::now();
    send_error("PreArm: Battery voltage 9.8V below minimum 10.5V");
    let elapsed = start.elapsed().as_micros();

    assert!(elapsed < 100, "Notification too slow: {} µs", elapsed);
}
```

**Related Code:**

- heapless crate: `heapless::Deque`, `heapless::String`
- Project logging: `src/core/logging.rs` (already uses heapless patterns)
- Embassy time: `embassy_time::Instant`
- Scheduler: `src/core/scheduler.rs` (400 Hz loop)
- Arming checks: `src/system/arming/` (call notification from checks)

## External References

- [heapless crate documentation](https://docs.rs/heapless/)
- [Embedded Rust Book - Collections](https://docs.rust-embedded.org/book/collections/)
- [RP2350 Datasheet - Memory](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [RP2350 Datasheet - Performance](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [Embassy Time Documentation](https://docs.embassy.dev/embassy-time/)
- [Real-Time Systems Performance](https://en.wikipedia.org/wiki/Real-time_computing)
