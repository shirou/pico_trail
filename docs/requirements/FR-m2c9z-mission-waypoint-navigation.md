# FR-m2c9z Mission Waypoint Navigation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-zd6uw-mission-execution](../analysis/AN-zd6uw-mission-execution.md)
- Related ADRs:
  - [ADR-2hs12-unified-waypoint-navigation](../adr/ADR-2hs12-unified-waypoint-navigation.md)
- Prerequisite Requirements:
  - [FR-2vbe8-navigation-controller](FR-2vbe8-navigation-controller.md)
  - [FR-cs42u-gps-navigation-state-access](FR-cs42u-gps-navigation-state-access.md)
- Dependent Requirements:
  - [FR-evl5k-position-target-mission-integration](FR-evl5k-position-target-mission-integration.md)
  - [FR-pbuh7-guided-mode-arm-start](FR-pbuh7-guided-mode-arm-start.md)
  - [FR-jm7mj-auto-mode-mission-execution](FR-jm7mj-auto-mode-mission-execution.md)
  - [FR-zcnqw-mission-item-reached](FR-zcnqw-mission-item-reached.md)
  - [FR-aulp3-mission-set-current](FR-aulp3-mission-set-current.md)
- Related Tasks:
  - [T-c26bh-unified-waypoint-navigation](../tasks/T-c26bh-unified-waypoint-navigation/README.md)

## Requirement Statement

The navigation_task shall read the current waypoint from MissionStorage and use it as the navigation target, providing a unified waypoint source for both GUIDED and AUTO modes.

## Rationale

Mission Planner and other GCS applications upload waypoints via the MISSION_ITEM protocol, not SET_POSITION_TARGET. The current implementation has a disconnect: waypoints are stored in MissionStorage but navigation_task reads from NAV_TARGET (only set by SET_POSITION_TARGET). This requirement bridges that gap by making MissionStorage the unified source for navigation waypoints.

Key benefits:

- Enables mission execution from waypoints uploaded via Mission Planner
- Provides consistent behavior between GUIDED and AUTO modes
- Removes the need for separate position target handling in navigation

## User Story (if applicable)

As a GCS operator, I want waypoints uploaded via Mission Planner to be used for navigation, so that the rover moves to the uploaded positions when armed in GUIDED or AUTO mode.

## Acceptance Criteria

- [ ] navigation_task reads current waypoint from MissionStorage
- [ ] Waypoint position (lat/lon) converted to navigation target format
- [ ] Navigation output (steering/throttle) computed from GPS to waypoint
- [ ] Navigation continues until waypoint acceptance radius reached
- [ ] No navigation occurs if MissionStorage is empty
- [ ] NAV_OUTPUT updated with computed steering and throttle values

## Technical Details (if applicable)

### Functional Requirement Details

**Navigation Flow:**

1. Check if MissionStorage has waypoints
2. Read current waypoint (at `current_index`)
3. Extract lat/lon from MAV_CMD_NAV_WAYPOINT item
4. Pass to navigation controller with current GPS position
5. Write navigation output to NAV_OUTPUT

**MissionStorage Integration:**

```rust
// Read current waypoint from MissionStorage
if let Some(waypoint) = mission_storage.get_current_waypoint() {
    let target = PositionTarget {
        lat: waypoint.x as f64 / 1e7,
        lon: waypoint.y as f64 / 1e7,
    };
    // Use navigation controller to compute output
}
```

**Supported Mission Items:**

| Command              | ID  | Fields Used | Description          |
| -------------------- | --- | ----------- | -------------------- |
| MAV_CMD_NAV_WAYPOINT | 16  | x, y        | Navigate to position |

**Parameters:**

- `WP_RADIUS`: Waypoint acceptance radius (default 2.0 m, ArduPilot standard)

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                            | Validation                      |
| ---------------------------------- | ------ | ---------- | ------------------------------------- | ------------------------------- |
| MissionStorage empty on navigation | Medium | Medium     | Return zero output, no navigation     | Test with empty MissionStorage  |
| Invalid waypoint coordinates       | Medium | Low        | Validate lat/lon before navigation    | Test with invalid coordinates   |
| Race condition on MissionStorage   | Medium | Low        | Use critical_section for state access | Test concurrent access patterns |

## Implementation Notes

Preferred approaches:

- Modify existing navigation_task in `examples/pico_trail_rover.rs`
- Add helper method to MissionStorage: `get_current_waypoint() -> Option<MissionItem>`
- Keep NAV_TARGET for backwards compatibility with SET_POSITION_TARGET

Known pitfalls:

- MISSION_ITEM uses x/y as lat/lon in 1e7 format
- MISSION_ITEM_INT uses x/y as lat/lon in native int32 format
- Ensure consistent coordinate format conversion

Related code areas:

- `src/core/mission/mod.rs` - MissionStorage
- `examples/pico_trail_rover.rs` - navigation_task (lines 829-908)
- `src/subsystems/navigation/mod.rs` - NAV_TARGET, NAV_OUTPUT

## External References

- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html)
- [MAV_CMD_NAV_WAYPOINT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT)
