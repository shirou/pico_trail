# FR-00175 Home Position Pre-Arm Check

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00171-home-auto-set-gps-fix](FR-00171-home-auto-set-gps-fix.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00047-home-position-management](../analysis/AN-00047-home-position-management.md)
- Related Tasks:
  - [T-00181-home-position-management](../tasks/T-00181-home-position-management/README.md)

## Requirement Statement

The system shall require the home position to be set as a pre-arm check in the GPS check category. Arming shall be blocked with the message "waiting for home" if `home_position` is `None`. The check shall be skippable via the `ARMING_CHECK` bitmask (GPS bit).

## Rationale

ArduPilot blocks arming with "AHRS: waiting for home" if home is not set (`AP_Arming::gps_checks()` at `AP_Arming.cpp:694-738`). This ensures that RTL, battery failsafe RTL, and GCS failsafe RTL always have a valid home destination. Without this check, the vehicle could arm without home, and any failsafe requiring RTL would fail -- potentially leading to the vehicle stopping in an unsafe location instead of returning home.

## User Story

As an operator, I want the vehicle to refuse arming until a home position is established, so that I know RTL and failsafe actions will have a valid return destination during flight.

## Acceptance Criteria

- [ ] Add home position check in GPS arming check category
- [ ] Arming fails with message "waiting for home" if `home_position` is `None`
- [ ] Check is skippable via `ARMING_CHECK` bitmask (when GPS check is disabled)
- [ ] Check passes immediately once home is auto-set on GPS fix (FR-00171)
- [ ] Check passes if home was set via `MAV_CMD_DO_SET_HOME` by GCS
- [ ] Failure message is sent to GCS via STATUSTEXT

## Technical Details

### Functional Requirement Details

**Pre-arm check implementation:**

```rust
// In GPS arming check category:
fn check_gps(&self, state: &SystemState) -> Result<(), &'static str> {
    // ... existing GPS fix checks ...

    if state.home_position.is_none() {
        return Err("waiting for home");
    }

    Ok(())
}
```

**ArduPilot reference:**

- `AP_Arming::gps_checks()` at `repo/ardupilot/libraries/AP_Arming/AP_Arming.cpp:694-738`
- Home check is within the `Check::GPS` category
- Error message: "AHRS: waiting for home"
- pico_trail uses "waiting for home" (without "AHRS:" prefix since pico_trail has no AHRS subsystem naming)

**Integration with existing arming flow:**

- Existing arming checks in `crates/core/src/arming/` have check categories
- Adding a home check fits the established pattern
- The check is naturally satisfied once FR-00171 (auto-set on GPS fix) is implemented

## Platform Considerations

N/A - Platform agnostic (core crate logic)

## Risks & Mitigation

| Risk                                     | Impact | Likelihood | Mitigation                                                            | Validation                  |
| ---------------------------------------- | ------ | ---------- | --------------------------------------------------------------------- | --------------------------- |
| GPS fix delayed, blocking arm            | Medium | Medium     | Clear "waiting for home" message tells operator the cause             | Test with cold GPS start    |
| Operator bypasses check via ARMING_CHECK | Low    | Low        | Documented behavior; operator assumes risk                            | Document ARMING_CHECK usage |
| Home check races with auto-set           | Low    | Low        | Auto-set runs at 50 Hz; home is set well before operator attempts arm | Test timing                 |

## Implementation Notes

- Add to existing GPS check category in `crates/core/src/arming/`
- Home is typically auto-set within seconds of GPS 3D fix (FR-00171), so this check rarely blocks in practice
- The check ensures safety for edge cases where GPS is unreliable or slow to acquire fix
- ARMING_CHECK bitmask behavior follows existing pattern for other pre-arm checks

## External References

- ArduPilot `gps_checks()` pre-arm: `repo/ardupilot/libraries/AP_Arming/AP_Arming.cpp:694-738`
- ArduPilot ARMING_CHECK parameter: <https://ardupilot.org/rover/docs/parameters.html#arming-check>
