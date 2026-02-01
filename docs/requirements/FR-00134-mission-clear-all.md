# FR-00134 Mission Clear All

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00042-mission-execution-gaps](../analysis/AN-00042-mission-execution-gaps.md)
- Related ADRs:
  - [ADR-00023-unified-waypoint-navigation](../adr/ADR-00023-unified-waypoint-navigation.md)
  - [ADR-00034-mission-execution-telemetry-architecture](../adr/ADR-00034-mission-execution-telemetry-architecture.md)
- Prerequisite Requirements:
  - [FR-00088-mission-execution-state](FR-00088-mission-execution-state.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-00037-mission-execution-sequencer](../tasks/T-00037-mission-execution-sequencer/README.md)

## Requirement Statement

The system shall handle MISSION_CLEAR_ALL by clearing MissionStorage and resetting MissionState to Idle, rejecting the request if the vehicle is armed and a mission is running.

## Rationale

Operators must be able to clear missions from the GCS without rebooting the vehicle. Currently there is no handler for MISSION_CLEAR_ALL, making it impossible to remove uploaded missions remotely. Following ArduPilot's safety pattern, clearing is rejected while armed and running to prevent unexpected behavior.

## User Story (if applicable)

As a GCS operator, I want to clear the uploaded mission from Mission Planner, so that I can upload a new mission or reset the vehicle's mission state.

## Acceptance Criteria

- [ ] MISSION_CLEAR_ALL handler implemented in mission handler
- [ ] Handler clears all waypoints from MissionStorage
- [ ] Handler resets MissionState to Idle
- [ ] Handler responds with MISSION_ACK (type=ACCEPTED) on success
- [ ] Handler rejects with MISSION_ACK (type=DENIED) if armed and mission is Running
- [ ] STATUSTEXT sent on clear: "Mission cleared"

## Technical Details (if applicable)

### Functional Requirement Details

**Safety Logic:**

```
if armed AND mission_state == Running:
    return MISSION_ACK(MAV_MISSION_DENIED)
else:
    clear MissionStorage
    reset MissionState to Idle
    return MISSION_ACK(MAV_MISSION_ACCEPTED)
```

**Existing Support:**

`clear_mission()` already exists in `crates/firmware/src/core/mission/state.rs:100-103` but is not called by any message handler. The handler needs to be wired into the dispatcher.

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                            | Impact | Likelihood | Mitigation                      | Validation                     |
| ------------------------------- | ------ | ---------- | ------------------------------- | ------------------------------ |
| Clear during active navigation  | High   | Medium     | Reject if armed and running     | Test clear while armed/running |
| Race condition on storage clear | Medium | Low        | Use critical_section for access | Test concurrent access         |

## Implementation Notes

Preferred approaches:

- Add MISSION_CLEAR_ALL case to `MissionHandler` dispatch in `crates/firmware/src/communication/mavlink/handlers/mission.rs`
- Reuse existing `clear_mission()` function from `state.rs`
- Check arm state and mission state before clearing

Related code areas:

- `crates/firmware/src/communication/mavlink/handlers/mission.rs:494-504` - MISSION_ACK handling area
- `crates/firmware/src/core/mission/state.rs:100-103` - `clear_mission()` function
- `crates/firmware/src/communication/mavlink/dispatcher.rs:464-467` - Unhandled message fallthrough

## External References

- [MAVLink MISSION_CLEAR_ALL](https://mavlink.io/en/messages/common.html#MISSION_CLEAR_ALL)
- [MAVLink MISSION_ACK](https://mavlink.io/en/messages/common.html#MISSION_ACK)
