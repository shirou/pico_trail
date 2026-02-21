# FR-00174 MAV_CMD_REQUEST_MESSAGE for HOME_POSITION

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

The system shall respond to `MAV_CMD_REQUEST_MESSAGE` (command 512) with param1=242 by sending the current HOME_POSITION message. The system shall return `MAV_RESULT_ACCEPTED` if home is set, or `MAV_RESULT_FAILED` if home is not set.

## Rationale

`MAV_CMD_REQUEST_MESSAGE` is the standard MAVLink protocol for GCS to request specific messages on demand. ArduPilot maps message ID 242 to `MSG_HOME` in `mavlink_id_to_ap_message_id()` (`GCS_Common.cpp:1046`). GCS uses this to request home after reconnection or when the on-change broadcast was missed. Without this support, a GCS that reconnects mid-flight has no way to learn the vehicle's home position.

## User Story

As a GCS operator, I want to be able to request the vehicle's home position on demand, so that my ground station can display home correctly even after reconnecting to the vehicle.

## Acceptance Criteria

- [ ] Handle message ID 242 in `MAV_CMD_REQUEST_MESSAGE` (command 512) handler
- [ ] Return HOME_POSITION message in command response when home is set
- [ ] Return `MAV_RESULT_ACCEPTED` when home is set and HOME_POSITION is sent
- [ ] Return `MAV_RESULT_FAILED` when home is not set (`home_position` is `None`)
- [ ] Use existing `build_home_position_message()` to construct the response

## Technical Details

### Functional Requirement Details

**Handler integration:**

The existing `handle_request_message()` returns only `MavResult`. Since HOME_POSITION needs an extra message in the response, handle message ID 242 at the `handle_command_long` level where the return type already supports extra messages via `Vec<MavMessage, 4>`:

```rust
// In handle_command_long, before delegating to handle_request_message:
MavCmd::MAV_CMD_REQUEST_MESSAGE => {
    let msg_id = command.param1 as u32;
    match msg_id {
        242 => {  // HOME_POSITION
            if let Some(home) = &state.home_position {
                let home_msg = build_home_position_message(home);
                extra_messages.push(home_msg);
                MavResult::MAV_RESULT_ACCEPTED
            } else {
                MavResult::MAV_RESULT_FAILED
            }
        }
        _ => handle_request_message(msg_id),
    }
}
```

**ArduPilot reference:**

- `handle_command_request_message()` at `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3272-3303`
- Message ID 242 mapped to `MSG_HOME` at `GCS_Common.cpp:1046`
- Calls `send_home_position()` which builds and sends HOME_POSITION

**Current state:**

- `handle_request_message()` handles IDs 148 (AUTOPILOT_VERSION) and 300 (PROTOCOL_VERSION) only
- Adding ID 242 extends existing pattern

## Platform Considerations

N/A - Platform agnostic (core crate logic)

## Risks & Mitigation

| Risk                                                        | Impact | Likelihood | Mitigation                                                  | Validation                           |
| ----------------------------------------------------------- | ------ | ---------- | ----------------------------------------------------------- | ------------------------------------ |
| Refactoring handle_request_message breaks existing behavior | Medium | Low        | Handle at handle_command_long level (no refactoring needed) | Test existing message IDs still work |
| GCS sends request before home is set                        | Low    | Low        | Return MAV_RESULT_FAILED; GCS can retry                     | Test with home unset                 |

## Implementation Notes

- Handle at `handle_command_long` level to access extra messages Vec
- No refactoring of existing `handle_request_message` needed
- `build_home_position_message()` may need to be accessible from both command handler and control loop

## External References

- ArduPilot `handle_command_request_message()`: `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3272-3303`
- MAVLink MAV_CMD_REQUEST_MESSAGE: <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE>
- MAVLink HOME_POSITION message: <https://mavlink.io/en/messages/common.html#HOME_POSITION>
