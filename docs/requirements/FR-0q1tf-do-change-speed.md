# FR-0q1tf DO Change Speed

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-l8emi-mission-execution-gaps](../analysis/AN-l8emi-mission-execution-gaps.md)
- Related ADRs:
  - [ADR-2hs12-unified-waypoint-navigation](../adr/ADR-2hs12-unified-waypoint-navigation.md)
  - [ADR-0yapl-mission-execution-telemetry-architecture](../adr/ADR-0yapl-mission-execution-telemetry-architecture.md)
- Prerequisite Requirements:
  - [FR-5zqns-nav-do-command-separation](FR-5zqns-nav-do-command-separation.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-r7k2p-mission-execution-sequencer](../tasks/T-r7k2p-mission-execution-sequencer/README.md)

## Requirement Statement

The system shall execute MAV_CMD_DO_CHANGE_SPEED (command ID 178) during mission execution to change the navigation speed, reverting to the default speed on mission completion.

## Rationale

Speed changes during missions are common for:

- Reducing speed near obstacles or in confined areas
- Increasing speed on open stretches
- Area-specific speed limits

This is one of the most commonly used DO commands in ArduPilot Rover missions. Mission Planner includes DO_CHANGE_SPEED in its mission planning interface.

## User Story (if applicable)

As a GCS operator, I want to include speed change commands in my mission, so that the rover adjusts its speed at specific points during autonomous navigation.

## Acceptance Criteria

- [ ] `MAV_CMD_DO_CHANGE_SPEED` (178) recognized as a DO command
- [ ] Speed parameter (param2) applied to navigation controller
- [ ] Speed persists until next DO_CHANGE_SPEED or mission end
- [ ] Speed reverts to default (`WP_SPEED`) on mission completion
- [ ] Speed value validated (> 0, <= maximum allowed speed)
- [ ] Invalid speed values logged and ignored

## Technical Details (if applicable)

### Functional Requirement Details

**MAV_CMD_DO_CHANGE_SPEED (ID: 178):**

| Parameter | Name       | Description                             |
| --------- | ---------- | --------------------------------------- |
| param1    | speed_type | 0=Airspeed, 1=Ground speed, 2=Climb     |
| param2    | speed      | Speed value (m/s), -1 = no change       |
| param3    | throttle   | Throttle percentage (%), -1 = no change |

For Rover, `speed_type` is always ground speed. `param2` is the target speed in m/s. `param3` (throttle) is typically -1 (no change) for Rover.

**Execution:**

```rust
fn execute_do_change_speed(waypoint: &Waypoint) {
    let speed = waypoint.param2;
    if speed > 0.0 {
        set_mission_speed(speed);
    }
}
```

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                  | Impact | Likelihood | Mitigation                     | Validation               |
| --------------------- | ------ | ---------- | ------------------------------ | ------------------------ |
| Excessive speed value | High   | Low        | Clamp to maximum allowed speed | Test with extreme values |
| Speed not reverting   | Medium | Low        | Reset speed on mission end     | Test mission completion  |
| Zero speed deadlock   | Medium | Low        | Reject speed <= 0              | Test with zero/negative  |

## Implementation Notes

Preferred approaches:

- Add speed override to navigation controller or Auto mode state
- Reset on mission completion in Auto mode `exit()` or when MissionState transitions to Completed

Related code areas:

- `crates/firmware/src/rover/mode/auto.rs` - Auto mode update loop
- `crates/firmware/src/subsystems/navigation/controller.rs` - Navigation controller (speed handling)

## External References

- [MAV_CMD_DO_CHANGE_SPEED](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_CHANGE_SPEED)
- [ArduPilot DO_CHANGE_SPEED](https://ardupilot.org/rover/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-do-change-speed)
