# FR-00178 MAV_CMD_GET_HOME_POSITION Support

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00171-home-auto-set-gps-fix](FR-00171-home-auto-set-gps-fix.md)
  - [FR-00005-mavlink-protocol](FR-00005-mavlink-protocol.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00047-home-position-management](../analysis/AN-00047-home-position-management.md)
- Related Tasks:
  - [T-00181-home-position-management](../tasks/T-00181-home-position-management/README.md)

## Requirement Statement

The system shall handle `MAV_CMD_GET_HOME_POSITION` (command 410, deprecated) by responding with the HOME_POSITION message. The system shall return `MAV_RESULT_ACCEPTED` if home is set, or `MAV_RESULT_FAILED` if home is not set.

## Rationale

`MAV_CMD_GET_HOME_POSITION` is deprecated in favor of `MAV_CMD_REQUEST_MESSAGE(242)`, but Mission Planner may still send this command on connect to retrieve the home position. ArduPilot supports it for backward compatibility (`GCS_Common.cpp:5092-5107`). Supporting this command ensures compatibility with older GCS software and Mission Planner versions that have not migrated to the newer protocol.

## User Story

As an operator using Mission Planner, I want the vehicle to respond to legacy home position queries, so that my GCS can display the home position regardless of which protocol version it uses.

## Acceptance Criteria

- [ ] Handle `MAV_CMD_GET_HOME_POSITION` (command 410) in COMMAND_LONG handler
- [ ] Return HOME_POSITION message in response when home is set
- [ ] Return `MAV_RESULT_ACCEPTED` if home is set
- [ ] Return `MAV_RESULT_FAILED` if home is not set
- [ ] Use existing `build_home_position_message()` to construct the response

## Technical Details

### Functional Requirement Details

**Handler implementation:**

```rust
// In handle_command_long:
MavCmd::MAV_CMD_GET_HOME_POSITION => {
    if let Some(home) = &state.home_position {
        let home_msg = build_home_position_message(home);
        extra_messages.push(home_msg);
        MavResult::MAV_RESULT_ACCEPTED
    } else {
        MavResult::MAV_RESULT_FAILED
    }
}
```

**ArduPilot reference:**

- `handle_command_get_home_position()` at `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:5092-5107`
- ArduPilot also sends GPS_GLOBAL_ORIGIN alongside HOME_POSITION; pico_trail omits GPS_GLOBAL_ORIGIN (no EKF origin concept)

**Note on GPS_GLOBAL_ORIGIN:**

ArduPilot sends both HOME_POSITION and GPS_GLOBAL_ORIGIN in response. pico_trail does not have an EKF origin concept, so only HOME_POSITION is sent. This is documented as an intentional deviation in the analysis (Out of Scope section).

## Platform Considerations

N/A - Platform agnostic (core crate logic)

## Risks & Mitigation

| Risk                                  | Impact | Likelihood | Mitigation                                                                     | Validation                |
| ------------------------------------- | ------ | ---------- | ------------------------------------------------------------------------------ | ------------------------- |
| Deprecated command removed from GCS   | Low    | Low        | FR-00174 (REQUEST_MESSAGE) is the standard path; this is a fallback            | Both paths tested         |
| Missing GPS_GLOBAL_ORIGIN in response | Low    | Low        | Mission Planner handles HOME_POSITION alone; GPS_GLOBAL_ORIGIN is EKF-specific | Test with Mission Planner |

## Implementation Notes

- Simple handler addition in `handle_command_long` (same pattern as FR-00174)
- Shares `build_home_position_message()` with FR-00174 and FR-00173
- Low implementation effort -- single match arm in command handler

## External References

- ArduPilot `handle_command_get_home_position()`: `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:5092-5107`
- MAVLink MAV_CMD_GET_HOME_POSITION: <https://mavlink.io/en/messages/common.html#MAV_CMD_GET_HOME_POSITION>
