# FR-00138 Mission Set Current

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
  - [FR-00090-mission-waypoint-navigation](FR-00090-mission-waypoint-navigation.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-00037-mission-execution-sequencer](../tasks/T-00037-mission-execution-sequencer/README.md)

## Requirement Statement

The system shall handle MISSION_SET_CURRENT by changing the current waypoint index to the specified sequence number and responding with a MISSION_CURRENT message as acknowledgment.

## Rationale

Operators need the ability to jump to a specific waypoint during mission execution (e.g., to skip a waypoint or restart from an earlier point). Mission Planner sends MISSION_SET_CURRENT when the user right-clicks a waypoint and selects "Fly to here" or "Set as current."

## User Story (if applicable)

As a GCS operator, I want to set the current waypoint to a specific index, so that I can restart from a specific point or skip waypoints during mission execution.

## Acceptance Criteria

- [ ] MISSION_SET_CURRENT handler implemented in mission handler
- [ ] Handler validates sequence number is within mission bounds
- [ ] Handler updates current waypoint index in MissionStorage
- [ ] If mission running: navigation immediately targets new waypoint
- [ ] If mission not running: sets start point for next mission start
- [ ] Handler responds with MISSION_CURRENT message as acknowledgment
- [ ] Invalid sequence number rejected (no change, no MISSION_CURRENT sent)

## Technical Details (if applicable)

### Functional Requirement Details

**MISSION_SET_CURRENT Message Fields:**

| Field              | Type   | Description                             |
| ------------------ | ------ | --------------------------------------- |
| `target_system`    | uint8  | Target system ID                        |
| `target_component` | uint8  | Target component ID                     |
| `seq`              | uint16 | Sequence number of desired current item |

**Handler Logic:**

```
if seq >= mission_count:
    ignore (no response)
else:
    set_current_index(seq)
    if mission_state == Running:
        navigation targets new waypoint immediately
    send MISSION_CURRENT(seq, total, mission_state)
```

**Existing Support:**

`set_current_index()` exists in MissionStorage but is not called by any external message handler.

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                          | Impact | Likelihood | Mitigation                         | Validation                         |
| ----------------------------- | ------ | ---------- | ---------------------------------- | ---------------------------------- |
| Jump to non-NAV command index | Medium | Medium     | Find next NAV command at/after seq | Test with DO command at seq        |
| Index out of bounds           | Low    | Low        | Validate against mission count     | Test with invalid sequence numbers |

## Implementation Notes

Preferred approaches:

- Add MISSION_SET_CURRENT case to `MissionHandler` dispatch
- Reuse `set_current_index()` from MissionStorage
- Emit MISSION_CURRENT via telemetry send path

Related code areas:

- `crates/firmware/src/communication/mavlink/handlers/mission.rs` - Mission handler
- `crates/core/src/mission/mod.rs` - `set_current_index()`
- `crates/firmware/src/communication/mavlink/dispatcher.rs` - Message dispatch

## External References

- [MAVLink MISSION_SET_CURRENT](https://mavlink.io/en/messages/common.html#MISSION_SET_CURRENT)
