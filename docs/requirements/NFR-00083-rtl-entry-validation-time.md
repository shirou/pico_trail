# NFR-00083 RTL Entry Validation Time

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00120-rtl-entry-validation](FR-00120-rtl-entry-validation.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00036-rtl-mode](../analysis/AN-00036-rtl-mode.md)
- Related Tasks:
  - [T-00030-rtl-smartrtl-implementation](../tasks/T-00030-rtl-smartrtl-implementation/README.md)

## Requirement Statement

RTL mode entry validation (can_enter() check) shall complete within 1 millisecond, ensuring fast mode switching response to operator commands or failsafe triggers.

## Rationale

Fast mode switching is critical for:

- Responsive operator control (mode change feels instant)
- Failsafe actions (quick response to battery/RC/GCS loss)
- System stability (no long-running blocking operations)

1ms is consistent with other mode validation timing requirements.

## User Story

The system shall validate RTL entry conditions within 1ms to ensure responsive mode switching.

## Acceptance Criteria

- [ ] `RtlMode::can_enter()` completes in < 1ms
- [ ] Validation involves only memory reads (no I/O)
- [ ] SYSTEM_STATE lock acquisition is brief
- [ ] Timing verified in unit tests or benchmarks

## Technical Details

### Non-Functional Requirement Details

**Performance Target:**

- Maximum latency: 1ms
- Typical latency: < 100us (microseconds)

**Validation Operations:**

1. Acquire SYSTEM_STATE mutex lock
2. Check `gps.has_fix()` (bool check)
3. Check `home_position.is_some()` (Option check)
4. Release mutex lock
5. Return Result

All operations are O(1) with no allocations.

**Measurement Method:**

```rust
#[test]
fn test_can_enter_timing() {
    let start = Instant::now();
    let _ = RtlMode::can_enter();
    let elapsed = start.elapsed();
    assert!(elapsed < Duration::from_millis(1));
}
```

## Platform Considerations

N/A - Platform agnostic timing requirements

## Risks & Mitigation

| Risk                        | Impact | Likelihood | Mitigation                      | Validation     |
| --------------------------- | ------ | ---------- | ------------------------------- | -------------- |
| Mutex contention delays     | Low    | Low        | Short critical section          | Stress testing |
| GPS check becomes expensive | Low    | Very Low   | GPS state is simple bool/struct | Code review    |

## Implementation Notes

- can_enter() is a static method - no RtlMode instance needed
- Validation is pre-construction check
- Keep critical section as short as possible
- Consider: Using try_lock with timeout for extra safety

## External References

N/A - No external references
