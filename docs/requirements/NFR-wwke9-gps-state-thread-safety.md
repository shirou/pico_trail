# NFR-wwke9 GPS State Thread Safety

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-cs42u-gps-navigation-state-access](../requirements/FR-cs42u-gps-navigation-state-access.md)
- Related Tasks:
  - [T-gvsxb-gps-state-telemetry](../tasks/T-gvsxb-gps-state-telemetry/README.md)

## Requirement Statement

GPS state sharing shall be thread-safe without blocking, allowing concurrent access from multiple consumers.

## Rationale

GPS state is updated by the GPS driver task and read by multiple consumers: telemetry handler, navigation subsystem, and control loop. These run as separate Embassy async tasks. Thread-safe access prevents data races while non-blocking behavior ensures real-time control loop performance.

## User Story (if applicable)

The system shall provide thread-safe GPS state access to ensure data integrity without introducing blocking delays in critical paths.

## Acceptance Criteria

- [ ] GPS state can be read by multiple tasks concurrently without data corruption
- [ ] GPS state write (from GPS driver) does not block readers
- [ ] GPS state read does not block writers
- [ ] No deadlock possible in GPS state access pattern
- [ ] GPS state updates are atomic (readers see consistent state, not partial updates)
- [ ] Pattern supports single writer (GPS driver) and multiple readers

## Technical Details (if applicable)

### Non-Functional Requirement Details

- Reliability:
  - Single writer pattern: Only GPS driver task updates state
  - Multiple readers: Telemetry, navigation, control loop
  - Atomic update: All fields updated together
- Performance:
  - Read latency: < 100us even under contention
  - No priority inversion in Embassy async context

**Recommended Pattern:**

```rust
// Option A: Embassy Mutex (simple, sufficient for most cases)
static GPS_STATE: Mutex<CriticalSectionRawMutex, GpsState> = Mutex::new(GpsState::default());

// Option B: Single-writer atomic update (if Mutex proves problematic)
// Use AtomicU64 for timestamp, atomic swap for position struct
```

## Platform Considerations

### Embedded (RP2350)

- Embassy `Mutex<CriticalSectionRawMutex, T>` for single-core safety
- Critical sections are short (disable interrupts during access)
- No RTOS-style priority inversion concerns in Embassy cooperative scheduler

### Host Tests

- Use std::sync::Mutex for equivalent behavior
- Test concurrent access with multiple threads

## Risks & Mitigation

| Risk                              | Impact | Likelihood | Mitigation                           | Validation                   |
| --------------------------------- | ------ | ---------- | ------------------------------------ | ---------------------------- |
| Mutex held too long               | Medium | Low        | Keep critical section minimal        | Code review, timing analysis |
| Forgotten lock release            | High   | Low        | Use RAII pattern (MutexGuard)        | Compiler enforces            |
| Interrupt during critical section | Low    | Medium     | Critical section disables interrupts | Architecture review          |

## Implementation Notes

- Embassy Mutex is the recommended starting point
- If performance issues arise, consider lock-free alternatives:
  - Atomic timestamp + seqlock pattern
  - Triple buffering for lock-free reads
- Document the access pattern in code comments

## External References

- [Embassy Mutex Documentation](https://docs.embassy.dev/embassy-sync/git/default/mutex/struct.Mutex.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
