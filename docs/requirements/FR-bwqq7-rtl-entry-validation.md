# FR-bwqq7 RTL Entry Validation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: None
- Dependent Requirements:
  - [FR-lia7r-rtl-navigate-home](FR-lia7r-rtl-navigate-home.md)
  - [FR-8dug4-smartrtl-return-navigation](FR-8dug4-smartrtl-return-navigation.md)
  - [NFR-nm9hf-rtl-entry-validation-time](NFR-nm9hf-rtl-entry-validation-time.md)
- Related Analyses:
  - [AN-75408-rtl-mode](../analysis/AN-75408-rtl-mode.md)
- Related Tasks:
  - [T-bo6xc-rtl-smartrtl-implementation](../tasks/T-bo6xc-rtl-smartrtl-implementation/README.md)

## Requirement Statement

The system shall validate that a valid GPS fix (3D or better) and a stored home position exist before allowing entry into RTL mode, rejecting entry with a clear error message if prerequisites are not met.

## Rationale

RTL mode requires both GPS position and a destination (home) to function. Without either:

- GPS: Cannot determine current position for navigation
- Home: No destination to navigate toward

Entry validation prevents entering a non-functional mode and provides clear feedback to operators.

## User Story

As an operator, I want the system to prevent entering RTL mode when prerequisites are not met, so that I understand why RTL cannot be activated and can take corrective action.

## Acceptance Criteria

- [ ] RTL entry is rejected if GPS fix is not 3D or better
- [ ] RTL entry is rejected if home_position is None
- [ ] Rejection returns error message: "RTL requires GPS fix" or "RTL requires home position"
- [ ] Validation check completes within 1ms (see NFR-nm9hf)
- [ ] Validation uses `RtlMode::can_enter()` static method
- [ ] Failed entry attempt is logged for diagnostics

## Technical Details

### Functional Requirement Details

**Validation Logic:**

```rust
pub fn can_enter() -> Result<(), &'static str> {
    let state = SYSTEM_STATE.lock();
    if !state.gps.has_fix() {
        return Err("RTL requires GPS fix");
    }
    if state.home_position.is_none() {
        return Err("RTL requires home position");
    }
    Ok(())
}
```

**GPS Fix Requirements:**

- Minimum: 3D fix (lat, lon, alt)
- Check via `state.gps.has_fix()` method

**Home Position Requirements:**

- Set via MAV_CMD_DO_SET_HOME from GCS
- Or auto-set on first valid GPS fix
- Check via `state.home_position.is_some()`

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                       | Impact | Likelihood | Mitigation                         | Validation                   |
| -------------------------- | ------ | ---------- | ---------------------------------- | ---------------------------- |
| User confused by rejection | Low    | Medium     | Clear error messages               | Review message clarity       |
| GPS temporarily lost       | Medium | Medium     | User can retry when GPS returns    | Test GPS recovery scenarios  |
| Home never set             | Medium | Low        | Document home-setting requirements | Verify home auto-set on boot |

## Implementation Notes

- Validation implemented as static method `RtlMode::can_enter()`
- Called by mode switching logic before constructing RtlMode
- Error message returned to GCS via STATUSTEXT
- Consider: MAV_CMD_DO_SET_HOME handler in mavlink handlers

## External References

- [ArduPilot Home Position](https://ardupilot.org/rover/docs/common-configuring-a-telemetry-radio-using-mission-planner.html) - Home position concepts
