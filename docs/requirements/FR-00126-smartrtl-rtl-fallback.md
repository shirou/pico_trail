# FR-00126 SmartRTL to Direct RTL Fallback

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00125-smartrtl-return-navigation](FR-00125-smartrtl-return-navigation.md)
  - [FR-00122-rtl-navigate-home](FR-00122-rtl-navigate-home.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00036-rtl-mode](../analysis/AN-00036-rtl-mode.md)
- Related Tasks:
  - [T-00030-rtl-smartrtl-implementation](../tasks/T-00030-rtl-smartrtl-implementation/README.md)

## Requirement Statement

The system shall automatically fall back to direct RTL navigation when SmartRTL path is unavailable, empty, or corrupted, sending a STATUSTEXT notification to inform the operator.

## Rationale

SmartRTL depends on a recorded path. Fallback to direct RTL ensures return capability in all scenarios:

- First arm after boot (no path recorded yet)
- Path buffer overflow discarded all points
- Path corruption detected
- GPS was unavailable during outbound travel

Direct RTL provides a safe fallback even when SmartRTL cannot function.

## User Story

As an operator, I want RTL to always work even when SmartRTL is unavailable, so that the vehicle can return home in any situation.

## Acceptance Criteria

- [ ] Fallback triggered when path buffer empty on SmartRTL entry
- [ ] Fallback triggered when path_recorder.has_valid_path() returns false
- [ ] STATUSTEXT sent: "SmartRTL: No path, using direct RTL"
- [ ] Direct RTL mode activated with same entry validation (FR-00120)
- [ ] Fallback is transparent - vehicle still returns home
- [ ] Log message records fallback reason

## Technical Details

### Functional Requirement Details

**Fallback Detection:**

```rust
pub fn enter_rtl() -> Result<(), &'static str> {
    // Validate common prerequisites
    RtlMode::can_enter()?;

    // Check SmartRTL availability
    if path_recorder.has_valid_path() {
        crate::log_info!("Entering SmartRTL mode");
        enter_smartrtl_mode()
    } else {
        crate::log_warn!("SmartRTL: No path, using direct RTL");
        send_statustext("SmartRTL: No path, using direct RTL");
        enter_direct_rtl_mode()
    }
}
```

**Path Validity Check:**

- Buffer contains at least 2 points (start + current)
- Points are chronologically ordered
- No corruption detected (checksum or bounds check)

**Fallback Scenarios:**

| Scenario           | Detection                | Action     |
| ------------------ | ------------------------ | ---------- |
| No path recorded   | buffer.is_empty()        | Direct RTL |
| Path too short     | buffer.len() < 2         | Direct RTL |
| Path corrupted     | Checksum/bounds failure  | Direct RTL |
| GPS never acquired | No valid points recorded | Direct RTL |

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                           | Impact | Likelihood | Mitigation                     | Validation              |
| ------------------------------ | ------ | ---------- | ------------------------------ | ----------------------- |
| Fallback unnoticed by operator | Low    | Low        | STATUSTEXT notification        | Verify GCS receives msg |
| Direct RTL fails too           | High   | Very Low   | Separate validation (FR-00120) | Test both modes         |
| Frequent fallbacks frustrating | Low    | Low        | Document path requirements     | User documentation      |

## Implementation Notes

- Entry logic in mode switching or RTL mode factory
- path_recorder.has_valid_path() as validation gate
- Both SmartRTL and direct RTL share common validation
- Consider: Telemetry showing which RTL mode is active

## External References

- [ArduPilot SmartRTL Fallback](https://ardupilot.org/rover/docs/smartrtl-mode.html) - Fallback behavior reference
