# ADR-2hs12 Unified Waypoint Navigation Architecture

## Metadata

- Type: ADR
- Status: Approved

## Links

- Related Analyses:
  - [AN-zd6uw-mission-execution](../analysis/AN-zd6uw-mission-execution.md)
- Impacted Requirements:
  - [FR-m2c9z-mission-waypoint-navigation](../requirements/FR-m2c9z-mission-waypoint-navigation.md)
  - [FR-pbuh7-guided-mode-arm-start](../requirements/FR-pbuh7-guided-mode-arm-start.md)
  - [FR-w893v-mission-start-command](../requirements/FR-w893v-mission-start-command.md)
  - [FR-v6571-mission-execution-state](../requirements/FR-v6571-mission-execution-state.md)
  - [FR-evl5k-position-target-mission-integration](../requirements/FR-evl5k-position-target-mission-integration.md)
  - [NFR-iuk5h-navigation-update-rate](../requirements/NFR-iuk5h-navigation-update-rate.md)
- Related Tasks:
  - [T-c26bh-unified-waypoint-navigation](../tasks/T-c26bh-unified-waypoint-navigation/README.md)

## Context

The current implementation has a disconnect between mission waypoints and navigation:

1. **Mission waypoints** are stored in `MissionStorage` via MISSION_ITEM protocol
2. **Navigation target** is stored in `NAV_TARGET` and only set by SET_POSITION_TARGET_GLOBAL_INT
3. **navigation_task** reads from `NAV_TARGET`, not `MissionStorage`

This causes the rover to not move when armed in GUIDED mode with a mission set, because:

- Mission Planner uploads waypoints via MISSION_ITEM, not SET_POSITION_TARGET
- The navigation system never reads the uploaded waypoints

**Forces in tension:**

- Mission Planner workflow (MISSION_ITEM upload) vs SET_POSITION_TARGET workflow
- GUIDED mode (single target) vs AUTO mode (sequential waypoints)
- Simplicity vs ArduPilot behavioral compatibility
- Unified source vs multiple navigation target sources

## Decision

We will use **MissionStorage as the unified waypoint source** for both GUIDED and AUTO modes.

### Decision Drivers

- Mission Planner compatibility (primary GCS for this project)
- Single source of truth for navigation targets
- Clear mode-specific behavior (GUIDED: single waypoint, AUTO: sequential)
- Minimal architectural changes to existing codebase

### Considered Options

- **Option A**: Minimal changes - MissionStorage as unified source
- **Option B**: Separate mission executor module
- **Option C**: Full ArduPilot-style mission manager

### Option Analysis

- **Option A** — Pros: Minimal code changes, reuses existing MissionStorage, clear behavior | Cons: Mission state embedded in navigation logic
- **Option B** — Pros: Clean separation, testable executor | Cons: More code, additional abstraction layer
- **Option C** — Pros: Complete ArduPilot compatibility | Cons: Significant complexity, overkill for current needs

## Rationale

Option A is selected because:

1. **Immediate need**: The rover needs to work with Mission Planner now
2. **Minimal risk**: Reuses proven MissionStorage implementation
3. **Upgrade path**: Clean interfaces allow future migration to Option B if needed
4. **Consistency**: Both modes use the same waypoint source, reducing confusion

The key insight is that Mission Planner's workflow is MISSION_ITEM-based, not SET_POSITION_TARGET-based. Aligning with this workflow provides the best user experience.

## Consequences

### Positive

- Rover moves when armed in GUIDED mode with uploaded waypoint
- Single source of truth for navigation targets (MissionStorage)
- Consistent behavior between GUIDED and AUTO modes
- Compatible with Mission Planner's standard workflow

### Negative

- NAV_TARGET global becomes redundant (or needs integration)
- SET_POSITION_TARGET now modifies MissionStorage (behavior change)
- Mission state management added to navigation logic

### Neutral

- Existing GUIDED mode via SET_POSITION_TARGET still works (via MissionStorage integration)
- No change to MissionStorage API for mission upload

## Implementation Notes

### Architecture

```
                     +------------------+
                     |  MissionStorage  |
                     |  (unified source)|
                     +--------+---------+
                              |
              +---------------+---------------+
              |                               |
    +---------v---------+           +---------v---------+
    |   MISSION_ITEM    |           | SET_POSITION_     |
    |   Protocol        |           | TARGET_GLOBAL_INT |
    +-------------------+           +-------------------+
              |                               |
              +---------------+---------------+
                              |
                     +--------v---------+
                     |  navigation_task |
                     +--------+---------+
                              |
              +---------------+---------------+
              |                               |
    +---------v---------+           +---------v---------+
    |   GUIDED Mode     |           |    AUTO Mode      |
    |   (single WP)     |           |   (sequential)    |
    +-------------------+           +-------------------+
```

### Key Changes

1. **navigation_task**: Read from MissionStorage instead of NAV_TARGET
2. **MissionState**: Add global state for Idle/Running/Completed
3. **GUIDED mode**: Start navigation on ARM if waypoint exists
4. **AUTO mode**: Add MAV_CMD_MISSION_START handler
5. **SET_POSITION_TARGET**: Add waypoint to MissionStorage

### Mode Behavior Summary

| Mode   | Trigger       | Behavior                          | Waypoint Advance |
| ------ | ------------- | --------------------------------- | ---------------- |
| GUIDED | ARM           | Navigate to current waypoint      | No               |
| GUIDED | SET_POSITION  | Update current waypoint, navigate | No               |
| AUTO   | MISSION_START | Navigate through all waypoints    | Yes              |

### Parameters

Use existing ArduPilot-compatible parameters:

- `WP_RADIUS`: Waypoint acceptance radius (default 2.0 m)
- `WP_SPEED`: Maximum navigation speed (default 2.0 m/s)

## Open Questions

- [x] Should GUIDED mode use mission waypoints? → Yes, unified source
- [x] What triggers mission start in AUTO mode? → MAV_CMD_MISSION_START
- [x] What happens when mission completes? → Transition to Hold mode
- [ ] Should mission support pause/resume? → Defer to future enhancement
- [ ] What is the exact waypoint acceptance radius? → Use ArduPilot default (WP_RADIUS = 2.0m)

## External References

- [ArduPilot Rover AUTO mode](https://ardupilot.org/rover/docs/auto-mode.html)
- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html)
- [MAV_CMD_MISSION_START](https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START)
- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)
