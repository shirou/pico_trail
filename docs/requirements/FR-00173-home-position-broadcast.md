# FR-00173 HOME_POSITION Broadcast on Change

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00171-home-auto-set-gps-fix](FR-00171-home-auto-set-gps-fix.md)
  - [FR-00005-mavlink-protocol](FR-00005-mavlink-protocol.md)
- Dependent Requirements:
  - [NFR-00180-home-broadcast-bandwidth](NFR-00180-home-broadcast-bandwidth.md)
- Related Analyses:
  - [AN-00047-home-position-management](../analysis/AN-00047-home-position-management.md)
- Related Tasks:
  - [T-00181-home-position-management](../tasks/T-00181-home-position-management/README.md)

## Requirement Statement

The system shall broadcast a HOME_POSITION MAVLink message on all active MAVLink channels whenever the home position is set or updated. HOME_POSITION shall be sent on-change only, not as a periodic stream.

## Rationale

ArduPilot broadcasts HOME_POSITION inside `AP_AHRS::set_home()` via `GCS_SEND_MESSAGE(MSG_HOME)` (`AP_AHRS.cpp:3035-3085`). HOME_POSITION is not in any ArduPilot stream group -- it is purely event-driven. This on-change approach ensures GCS (Mission Planner) immediately knows the home position without consuming bandwidth on periodic broadcasts. Without this broadcast, Mission Planner reports "failed to set home" when entering Guided mode because it has no knowledge of the vehicle's home position.

## User Story

As a GCS operator, I want to be automatically notified whenever the vehicle's home position changes, so that my ground station displays the correct home marker and distance-to-home without manual polling.

## Acceptance Criteria

- [ ] Send HOME_POSITION message when `set_home()` is called (auto-set on GPS fix)
- [ ] Send HOME_POSITION message when `set_home_to_current()` updates home (disarmed refinement)
- [ ] Send HOME_POSITION message when GCS overrides home via `MAV_CMD_DO_SET_HOME`
- [ ] HOME_POSITION includes lat/lon in degE7 format and altitude in mm
- [ ] HOME_POSITION is not added to any periodic telemetry stream
- [ ] Use existing `build_home_position_message()` function to construct the message

## Technical Details

### Functional Requirement Details

**Broadcast pattern:**

The caller (vehicle control loop or command handler) is responsible for building and sending HOME_POSITION after modifying home:

```rust
// After set_home_to_current() or set_home() in the control loop:
critical_section::with(|cs| {
    let mut state = SYSTEM_STATE.borrow_ref_mut(cs);
    state.set_home_to_current();
});
// After critical section, build and send:
let msg = build_home_position_message(&state);
mavlink_sender.send(msg);
```

**ArduPilot reference:**

- `AP_AHRS::set_home()` at `repo/ardupilot/libraries/AP_AHRS/AP_AHRS.cpp:3035-3085`
- `GCS_SEND_MESSAGE(MSG_HOME)` broadcasts to all active channels
- Not in `STREAM_POSITION_msgs[]` array (`GCS_MAVLink_Parameters.cpp:272`)

**HOME_POSITION message fields (ID 242):**

| Field     | Format        | Source                                                |
| --------- | ------------- | ----------------------------------------------------- |
| latitude  | degE7 (int32) | home.latitude \* 1e7                                  |
| longitude | degE7 (int32) | home.longitude \* 1e7                                 |
| altitude  | mm (int32)    | home.altitude \* 1000                                 |
| q         | \[f32; 4]     | \[NaN; 4] (unknown orientation, ArduPilot convention) |

## Platform Considerations

N/A - Platform agnostic (core crate logic)

## Risks & Mitigation

| Risk                                 | Impact | Likelihood | Mitigation                                                       | Validation                 |
| ------------------------------------ | ------ | ---------- | ---------------------------------------------------------------- | -------------------------- |
| GCS misses on-change broadcast       | Low    | Low        | GCS can request via MAV_CMD_REQUEST_MESSAGE(242) (FR-00174)      | Test reconnection scenario |
| Broadcast from critical section      | Medium | Low        | Build and send after critical section exits                      | Code review                |
| Multiple rapid updates flood channel | Low    | Low        | On-change only; updates are infrequent (1 Hz max while disarmed) | Monitor bandwidth          |

## Implementation Notes

- `build_home_position_message()` already exists in command handler; move to shared location or re-use
- Broadcast must happen outside `critical_section::with()` blocks (MAVLink send is non-trivial)
- MavlinkLoopRunner already has access to both SYSTEM_STATE and the MAVLink sender
- For `MAV_CMD_DO_SET_HOME`, the existing handler already sends HOME_POSITION in its response

## External References

- ArduPilot `AP_AHRS::set_home()` with broadcast: `repo/ardupilot/libraries/AP_AHRS/AP_AHRS.cpp:3035-3085`
- ArduPilot `send_home_position()`: `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3095-3125`
- MAVLink HOME_POSITION message: <https://mavlink.io/en/messages/common.html#HOME_POSITION>
