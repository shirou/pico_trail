# FR-00089 Mission Start Command

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00024-mission-execution](../analysis/AN-00024-mission-execution.md)
- Related ADRs:
  - [ADR-00023-unified-waypoint-navigation](../adr/ADR-00023-unified-waypoint-navigation.md)
- Prerequisite Requirements:
  - [FR-00088-mission-execution-state](FR-00088-mission-execution-state.md)
  - [FR-00082-auto-mode-mission-execution](FR-00082-auto-mode-mission-execution.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-00019-unified-waypoint-navigation](../tasks/T-00019-unified-waypoint-navigation/README.md)

## Requirement Statement

The system shall implement a MAV_CMD_MISSION_START command handler that initiates mission execution in AUTO mode.

## Rationale

ArduPilot requires MAV_CMD_MISSION_START to begin mission execution in AUTO mode. This command:

1. Validates mission is loaded
2. Sets mission state to Running
3. Triggers navigation to first waypoint

Without this command handler, the AUTO mode cannot be started properly from GCS applications like Mission Planner.

## User Story (if applicable)

As a GCS operator, I want to send a MAV_CMD_MISSION_START command to begin autonomous mission execution, so that the rover navigates through the uploaded waypoints in AUTO mode.

## Acceptance Criteria

- [ ] MAV_CMD_MISSION_START handler implemented in command.rs
- [ ] Command accepted only when in AUTO mode (or triggers mode change)
- [ ] Command validates mission is not empty
- [ ] Command sets mission state to Running
- [ ] Command sets current waypoint index to first waypoint (or specified index)
- [ ] Command returns MAV_RESULT_ACCEPTED on success
- [ ] Command returns MAV_RESULT_FAILED if no mission loaded
- [ ] STATUSTEXT sent: "Mission started"

## Technical Details (if applicable)

### Functional Requirement Details

**MAV_CMD_MISSION_START (ID: 300):**

| Parameter | Name       | Description                         |
| --------- | ---------- | ----------------------------------- |
| param1    | first_item | First mission item to run (0-based) |
| param2    | last_item  | Last mission item to run (0 = all)  |
| param3-7  | -          | Reserved                            |

**Command Handler Logic:**

```rust
fn handle_mission_start(
    first_item: u16,
    last_item: u16,
) -> MavResult {
    // 1. Validate mission exists
    let mission_count = MISSION_STORAGE.lock().await.count();
    if mission_count == 0 {
        return MavResult::MAV_RESULT_FAILED;
    }

    // 2. Validate indices
    let first = first_item as usize;
    let last = if last_item == 0 { mission_count - 1 } else { last_item as usize };

    if first >= mission_count {
        return MavResult::MAV_RESULT_FAILED;
    }

    // 3. Set mission execution state
    MISSION_STATE.lock().await.start(first, last);

    // 4. Set current waypoint
    MISSION_STORAGE.lock().await.set_current(first);

    MavResult::MAV_RESULT_ACCEPTED
}
```

**Integration with AUTO Mode:**

- AUTO mode checks mission state before navigation
- If mission state is Idle, AUTO mode holds position
- If mission state is Running, AUTO mode navigates through waypoints
- If mission state is Completed, AUTO mode transitions to Hold

**Mode Transition Behavior:**

Two valid approaches (choose during implementation):

1. **Strict**: Command only valid in AUTO mode
2. **Permissive**: Command automatically switches to AUTO mode

ArduPilot uses the permissive approach.

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                             | Impact | Likelihood | Mitigation                     | Validation                            |
| -------------------------------- | ------ | ---------- | ------------------------------ | ------------------------------------- |
| Mission start while DISARMED     | Medium | Medium     | Reject if not armed            | Test command when disarmed            |
| Invalid first_item index         | Low    | Low        | Validate against mission count | Test with out-of-range indices        |
| Command during active navigation | Medium | Low        | Reset to new start index       | Test restart during mission execution |

## Implementation Notes

Preferred approaches:

- Add handler to `src/communication/mavlink/handlers/command.rs`
- Follow existing command handler patterns (e.g., ARM_DISARM)
- Use existing MISSION_STORAGE global
- Create MISSION_STATE global if not exists

Known pitfalls:

- first_item is 0-based in MAVLink protocol
- last_item = 0 means "run to end of mission"
- ArduPilot may send this command with mode change

Related code areas:

- `src/communication/mavlink/handlers/command.rs` - Command handlers
- `src/core/mission/mod.rs` - MissionStorage
- `src/rover/mode/` - AUTO mode implementation

## External References

- [MAV_CMD_MISSION_START](https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START)
- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html)
