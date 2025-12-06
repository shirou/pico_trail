# FR-evl5k Position Target Mission Integration

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-zd6uw-mission-execution](../analysis/AN-zd6uw-mission-execution.md)
- Related ADRs:
  - [ADR-2hs12-unified-waypoint-navigation](../adr/ADR-2hs12-unified-waypoint-navigation.md)
- Prerequisite Requirements:
  - [FR-m2c9z-mission-waypoint-navigation](FR-m2c9z-mission-waypoint-navigation.md)
  - [FR-obwjs-position-target-command-handler](FR-obwjs-position-target-command-handler.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-c26bh-unified-waypoint-navigation](../tasks/T-c26bh-unified-waypoint-navigation/README.md)

## Requirement Statement

When SET_POSITION_TARGET_GLOBAL_INT is received, the system shall add the position as a waypoint to MissionStorage and set it as the current navigation target.

## Rationale

This requirement unifies the two navigation target sources:

1. **MISSION_ITEM protocol**: Waypoints uploaded via Mission Planner
2. **SET_POSITION_TARGET**: Real-time targets from GCS "click-to-go" feature

By integrating SET_POSITION_TARGET with MissionStorage, the navigation system only needs to read from one source (MissionStorage), simplifying the architecture and ensuring consistent behavior.

## User Story (if applicable)

As a GCS operator, I want to use the "click on map" feature to set a navigation target, so that the rover moves to the clicked position without uploading a full mission.

## Acceptance Criteria

- [ ] SET_POSITION_TARGET_GLOBAL_INT adds waypoint to MissionStorage
- [ ] Added waypoint set as current navigation target (current_index updated)
- [ ] Previous single-target waypoint replaced (not accumulated)
- [ ] Works in both GUIDED and AUTO modes
- [ ] Does not interfere with uploaded mission waypoints
- [ ] MAV_RESULT_ACCEPTED returned on success
- [ ] Navigation begins immediately (if armed and in appropriate mode)

## Technical Details (if applicable)

### Functional Requirement Details

**Handler Behavior:**

```rust
fn handle_set_position_target_global_int(
    msg: &SET_POSITION_TARGET_GLOBAL_INT_DATA,
) -> MavResult {
    // Create waypoint from position target
    let waypoint = MissionItem {
        command: MavCmd::MAV_CMD_NAV_WAYPOINT,
        x: msg.lat_int,  // Already in 1e7 format
        y: msg.lon_int,  // Already in 1e7 format
        z: msg.alt,
        // ... other fields
    };

    // Add to MissionStorage as waypoint 0 (or replace existing)
    let mut storage = MISSION_STORAGE.lock().await;
    storage.set_position_target_waypoint(waypoint);

    MavResult::MAV_RESULT_ACCEPTED
}
```

**MissionStorage Extension:**

Add method to handle position target separately from uploaded missions:

```rust
impl MissionStorage {
    /// Set position target as single waypoint
    /// Used by SET_POSITION_TARGET_GLOBAL_INT
    pub fn set_position_target_waypoint(&mut self, waypoint: MissionItem) {
        // Option 1: Replace entire mission with single waypoint
        self.clear();
        self.add(waypoint);
        self.current_index = 0;

        // Option 2: Insert at index 0, shift others
        // self.insert_at(0, waypoint);
        // self.current_index = 0;
    }
}
```

**Coordinate Format:**

SET_POSITION_TARGET_GLOBAL_INT uses:

- `lat_int`: Latitude in 1e7 degrees
- `lon_int`: Longitude in 1e7 degrees
- `alt`: Altitude in meters

This matches MISSION_ITEM_INT format directly.

**Mode Interaction:**

| Mode   | Behavior                               |
| ------ | -------------------------------------- |
| GUIDED | Navigate to target immediately         |
| AUTO   | Insert as waypoint, continue execution |
| MANUAL | Store but do not navigate              |

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                                | Impact | Likelihood | Mitigation                          | Validation                             |
| ----------------------------------- | ------ | ---------- | ----------------------------------- | -------------------------------------- |
| Overwrites uploaded mission         | Medium | Medium     | Document behavior, option to append | Test interaction with uploaded mission |
| Unexpected navigation in wrong mode | Low    | Low        | Only navigate in GUIDED/AUTO        | Test SET_POSITION in MANUAL mode       |
| Coordinate format mismatch          | Medium | Low        | Validate lat/lon ranges             | Test with edge-case coordinates        |

## Implementation Notes

Preferred approaches:

- Modify existing SET_POSITION_TARGET handler
- Remove or deprecate NAV_TARGET global (replaced by MissionStorage)
- Keep backwards compatibility during transition

Known pitfalls:

- SET_POSITION_TARGET has mask field - respect it for partial updates
- Velocity and acceleration fields in message are optional
- Some GCS may send position with velocity targets

Related code areas:

- `src/communication/mavlink/handlers/` - Message handlers
- `src/core/mission/mod.rs` - MissionStorage
- `src/subsystems/navigation/mod.rs` - NAV_TARGET (to be replaced)

## External References

- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)
- [MAVLink Guided Mode Commands](https://ardupilot.org/dev/docs/mavlink-rover-commands.html)
