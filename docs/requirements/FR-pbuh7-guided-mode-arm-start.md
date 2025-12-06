# FR-pbuh7 Guided Mode ARM Start

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
  - [FR-erpze-guided-mode-navigation](FR-erpze-guided-mode-navigation.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-c26bh-unified-waypoint-navigation](../tasks/T-c26bh-unified-waypoint-navigation/README.md)

## Requirement Statement

In GUIDED mode, the system shall begin navigation to the current waypoint immediately upon ARM when a waypoint is set in MissionStorage.

## Rationale

Mission Planner uploads waypoints via MISSION_ITEM protocol, not SET_POSITION_TARGET. When an operator:

1. Uploads a waypoint via Mission Planner
2. Sets mode to GUIDED
3. Arms the vehicle

The vehicle should immediately start navigating to the uploaded waypoint. This matches the expected workflow for GCS operators who use Mission Planner's point-and-click interface.

Key difference from existing GUIDED mode:

- Existing: Waits for SET_POSITION_TARGET command after ARM
- New: Starts navigation immediately on ARM if waypoint is set

## User Story (if applicable)

As a GCS operator, I want the rover to start moving toward the uploaded waypoint when I arm it in GUIDED mode, so that I can control the rover using Mission Planner's map interface without additional commands.

## Acceptance Criteria

- [ ] ARM in GUIDED mode with waypoint set triggers immediate navigation
- [ ] Navigation targets current waypoint from MissionStorage (index 0)
- [ ] Vehicle stops when waypoint reached (no automatic advancement)
- [ ] ARM in GUIDED mode without waypoint holds position
- [ ] SET_POSITION_TARGET still works to update target after ARM
- [ ] DISARM stops navigation and clears target
- [ ] STATUSTEXT sent: "Guided: navigating to waypoint"

## Technical Details (if applicable)

### Functional Requirement Details

**ARM Behavior in GUIDED Mode:**

```
[DISARMED + GUIDED]
       |
       | ARM command received
       |
       v
[Check MissionStorage]
       |
       |-- Has waypoint --> [Start navigation to waypoint]
       |
       |-- No waypoint --> [Hold position]
```

**Navigation Behavior:**

- Navigate to single waypoint only (current_index in MissionStorage)
- Do not advance to next waypoint when reached
- Stop within WP_RADIUS of target
- Remain in GUIDED mode after reaching target

**Interaction with SET_POSITION_TARGET:**

When SET_POSITION_TARGET received:

1. Update current waypoint in MissionStorage (or add as new waypoint)
2. Navigation controller switches to new target
3. Maintains GUIDED mode behavior (no auto-advance)

**State Diagram:**

```
                    ARM
                     |
                     v
              +-----------+
              | Check WP  |
              +-----------+
             /             \
    Has WP  /               \ No WP
           v                 v
    +------------+    +-----------+
    | Navigating |    | Holding   |
    +------------+    +-----------+
          |                 ^
          | Reached         | SET_POSITION_TARGET
          v                 |
    +-----------+    +------+------+
    | Holding   |<---|    Update   |
    +-----------+    |   Waypoint  |
                     +-------------+
```

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                         | Validation                        |
| ---------------------------------- | ------ | ---------- | ---------------------------------- | --------------------------------- |
| Unexpected movement on ARM         | High   | Low        | Clear STATUSTEXT before navigation | Test ARM behavior with/without WP |
| Operator expects SET_POSITION flow | Medium | Medium     | Document behavior, send STATUSTEXT | User testing with Mission Planner |
| Waypoint set for different mode    | Low    | Low        | Check mode before starting nav     | Test mode-waypoint interactions   |

## Implementation Notes

Preferred approaches:

- Extend existing GUIDED mode state machine
- Check MissionStorage on ARM transition
- Reuse navigation_task for actual navigation
- Send STATUSTEXT to inform operator of behavior

Known pitfalls:

- Must distinguish between "no waypoint" and "waypoint at current position"
- ARM command may come from multiple sources (MAV_CMD_COMPONENT_ARM_DISARM, stick gesture)
- Ensure consistent behavior regardless of ARM source

Related code areas:

- `src/rover/mode/` - Mode implementations
- `examples/pico_trail_rover.rs` - navigation_task
- `src/core/mission/mod.rs` - MissionStorage
- `src/communication/mavlink/handlers/command.rs` - ARM command handler

## External References

- [ArduPilot Guided Mode](https://ardupilot.org/rover/docs/guided-mode.html)
