# NFR-00072 Log Push Performance Constraint

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-00026-logging-storage-and-routing](../analysis/AN-00026-logging-storage-and-routing.md)
- Prerequisite Requirements:
  - [FR-00096-log-ring-buffer](../requirements/FR-00096-log-ring-buffer.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00021-log-storage-routing](../tasks/T-00021-log-storage-routing/design.md)

## Requirement Statement

Log push operations shall complete within 50 microseconds worst-case, ensuring logging does not introduce jitter or latency in the 400 Hz real-time scheduler.

## Rationale

The rover scheduler runs at 400 Hz (2.5 ms period). Log calls may occur within scheduler tasks, sensor handlers, or control loops. If logging blocks or takes too long, it could cause task overruns, missed deadlines, or control loop jitter. The 50 µs target provides 2% budget of a single scheduler tick.

## User Story

The system shall **complete log push operations within 50 microseconds** to ensure **real-time tasks are not delayed by logging overhead**.

## Acceptance Criteria

- [ ] Log push (log_info!, log_warn!, etc.) completes in ≤50 µs worst-case
- [ ] Non-blocking channel send (try_send, not send)
- [ ] No heap allocation in log push path
- [ ] No I/O operations in log push path
- [ ] Mutex lock held for minimal duration
- [ ] O(1) time complexity for push operation

## Technical Details

### Non-Functional Requirement Details

**Performance Budget:**

| Operation      | Target     | Notes                       |
| -------------- | ---------- | --------------------------- |
| Format message | ≤20 µs     | heapless::String formatting |
| Acquire mutex  | ≤5 µs      | CriticalSection lock        |
| Push to buffer | ≤10 µs     | HistoryBuffer::write        |
| Route to sinks | ≤10 µs     | Channel try_send            |
| Release mutex  | ≤5 µs      | Automatic on drop           |
| **Total**      | **≤50 µs** |                             |

**Critical Path:**

```rust
// Must complete within 50 µs
log_info!("GPS fix: lat={}, lon={}", lat, lon);

// Expands to approximately:
{
    let msg = format_to_heapless_string(...);  // ≤20 µs
    LOG_ROUTER.lock(|router| {                 // ≤5 µs acquire
        router.route(msg);                      // ≤20 µs
    });                                         // ≤5 µs release
}
```

**Prohibited Operations in Log Path:**

- Heap allocation (malloc, Vec::push, String::push)
- I/O operations (UART write, USB write)
- Blocking waits (channel.send(), await)
- Recursive locking

**Benchmark Strategy:**

```rust
#[cfg(test)]
fn benchmark_log_push() {
    let start = Instant::now();
    for _ in 0..1000 {
        log_info!("test message with value: {}", 42);
    }
    let elapsed = start.elapsed();
    let per_call = elapsed / 1000;
    assert!(per_call < Duration::from_micros(50));
}
```

## Platform Considerations

### Embedded (RP2350)

- 150 MHz Cortex-M33 core
- Critical section via `cortex_m::interrupt::free()`
- heapless data structures only
- No allocator in hot path

### Host Tests

- Performance may differ significantly
- Focus on functional correctness, not timing
- Can still verify no heap allocation with custom allocator

### Cross-Platform

- Algorithm complexity identical (O(1))
- Absolute timing varies by platform

## Risks & Mitigation

| Risk                                 | Impact | Likelihood | Mitigation                                    | Validation                   |
| ------------------------------------ | ------ | ---------- | --------------------------------------------- | ---------------------------- |
| Format string causes heap allocation | High   | Medium     | Use heapless::String with sufficient capacity | Memory profiling             |
| Mutex contention causes delay        | Medium | Low        | Short critical sections, try_lock fallback    | Stress testing               |
| Long format strings exceed budget    | Low    | Medium     | Document string length limits                 | Benchmark with long messages |
| Interrupt during critical section    | Low    | Low        | CriticalSection disables interrupts briefly   | Real-time analysis           |

## Implementation Notes

**Preferred Patterns:**

- Use `try_send()` not `send()` for channel operations
- Pre-allocate heapless::String with sufficient capacity
- Keep critical sections minimal (just the push)

**Known Pitfalls:**

- `format!()` macro uses heap - use custom formatting to heapless::String
- Long format strings may exceed capacity - truncate gracefully

**Measurement Approach:**

- Use `embassy_time::Instant` for timing on target
- Profile with probe-rs or logic analyzer
- Track P99 latency, not just average

**Related Code:**

- Scheduler: `src/core/scheduler/`
- Current logging: `src/core/logging.rs`

## External References

- [Embassy Timer documentation](https://docs.embassy.dev/embassy-time/)
- [Cortex-M Critical Section](https://docs.rs/cortex-m/latest/cortex_m/interrupt/fn.free.html)
