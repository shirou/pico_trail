# FR-jm7mj Auto Mode Mission Execution

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-27568-position-target-navigation](../analysis/AN-27568-position-target-navigation.md)
  - [AN-7ix56-navigation-approach](../analysis/AN-7ix56-navigation-approach.md)
- Prerequisite Requirements:
  - [FR-2vbe8-navigation-controller](FR-2vbe8-navigation-controller.md)
  - [FR-tmibt-position-target-state](FR-tmibt-position-target-state.md)
  - [FR-333ym-gps-waypoint-navigation](FR-333ym-gps-waypoint-navigation.md)
  - [FR-q2sjt-control-mode-framework](FR-q2sjt-control-mode-framework.md)
  - [FR-cs42u-gps-navigation-state-access](FR-cs42u-gps-navigation-state-access.md)
  - [FR-m2c9z-mission-waypoint-navigation](FR-m2c9z-mission-waypoint-navigation.md)
  - [FR-v6571-mission-execution-state](FR-v6571-mission-execution-state.md)
- Dependent Requirements:
  - [FR-w893v-mission-start-command](FR-w893v-mission-start-command.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall implement Auto mode that executes uploaded waypoint missions by navigating sequentially through mission items using the navigation controller.

## Rationale

Auto mode is the core autonomous operation capability:

- Executes pre-planned missions uploaded via MAVLink
- Enables autonomous surveys, patrols, and delivery routes
- Provides predictable, repeatable navigation behavior
- Standard mode in ArduPilot expected by GCS applications

## User Story (if applicable)

As an operator, I want to upload a mission and have the vehicle automatically navigate through all waypoints, so that I can perform autonomous operations without manual control.

## Acceptance Criteria

- [ ] Auto mode entry requires valid GPS fix (3D fix minimum)
- [ ] Auto mode entry requires at least one waypoint in mission
- [ ] Vehicle navigates to first waypoint on mode entry
- [ ] Vehicle advances to next waypoint when within WP_RADIUS
- [ ] Navigation uses configured navigation controller (L1 or S-curve)
- [ ] Mission progress reported via MISSION_CURRENT message
- [ ] Mode transitions to Hold when mission completes
- [ ] GPS fix loss triggers failsafe (transition to Hold or RTL)
- [ ] HEARTBEAT custom_mode reports Auto mode (mode 3)
- [ ] STATUSTEXT sent: "Auto mode - starting mission"

## Technical Details (if applicable)

### Functional Requirement Details

**Mode Entry Conditions:**

- GPS fix type >= 3D_FIX
- Mission uploaded with at least one NAV_WAYPOINT item
- Vehicle not in emergency stop state
- Pre-arm checks passed (if armed)

**Mode Behavior:**

1. On entry: Load first waypoint as navigation target
2. Each update cycle:
   - Get current GPS position
   - Calculate distance to current waypoint
   - If distance < WP_RADIUS: Advance to next waypoint
   - Calculate steering/throttle via navigation controller
   - Output commands to actuators
3. On mission complete: Transition to Hold mode
4. On exit: Clear navigation state

**Mission Integration:**

- Read waypoints from MissionStorage (existing mission handler)
- Support MAV_CMD_NAV_WAYPOINT items
- Track current waypoint index in mission sequence
- Report progress via MISSION_CURRENT MAVLink message

**Waypoint Acceptance:**

- Distance to waypoint < WP_RADIUS triggers advance
- WP_RADIUS configurable (default 2.0 m)

**State Diagram:**

```
         [Entry with mission]
                 |
                 v
        +----------------+
        |   Navigate to  |
        |   Waypoint N   |
        +----------------+
              |    |
    Reached   |    | GPS lost
              v    v
     +-------------+  +----------+
     | Advance to  |  | Failsafe |
     | Waypoint    |  +----------+
     | N+1         |
     +-------------+
              |
              | Last waypoint
              v
        +-----------+
        | Mission   |
        | Complete  |
        | -> Hold   |
        +-----------+
```

**Mission Commands Supported (Phase 1):**

| Command              | ID  | Description          |
| -------------------- | --- | -------------------- |
| MAV_CMD_NAV_WAYPOINT | 16  | Navigate to position |

**Navigation Parameters (ArduPilot-compatible):**

- `WP_RADIUS`: Waypoint acceptance radius (default 2.0 m)
- `WP_SPEED`: Maximum navigation speed (default 2.0 m/s)

## Platform Considerations

### Pico W (RP2040)

May need to limit mission size due to memory constraints. Navigation calculations may require optimization.

### Pico 2 W (RP2350)

Full mission support with hardware FPU for navigation calculations.

### Cross-Platform

Mode logic identical; mission storage capacity may vary.

## Risks & Mitigation

| Risk                            | Impact | Likelihood | Mitigation                               | Validation                            |
| ------------------------------- | ------ | ---------- | ---------------------------------------- | ------------------------------------- |
| Empty mission causes crash      | High   | Low        | Validate mission before mode entry       | Test mode entry with empty mission    |
| Waypoint skipping at high speed | Medium | Medium     | Reduce speed near waypoints, tune radius | Test at various speeds                |
| Mission corruption              | Medium | Low        | Validate mission checksum on load        | Test with corrupted mission data      |
| GPS loss mid-mission            | High   | Low        | Trigger failsafe, log mission progress   | Test GPS disconnect during navigation |

## Implementation Notes

Preferred approaches:

- Implement as `AutoMode` struct implementing `Mode` trait
- Reuse navigation controller from Guided mode
- Read mission items from existing MissionStorage
- Track waypoint index as mode state

Known pitfalls:

- Mission storage uses 0-based indexing
- MISSION_CURRENT uses 0-based sequence number
- Ensure mission changes don't corrupt in-progress navigation
- Handle mission clear/update while in Auto mode

Related code areas:

- `src/rover/mode/` - Mode implementations
- `src/core/mission/` - Mission storage
- `src/subsystems/navigation/` - Navigation controller
- `src/communication/mavlink/handlers/mission.rs` - Mission protocol

## External References

- [ArduPilot Auto Mode](https://ardupilot.org/rover/docs/auto-mode.html)
- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
