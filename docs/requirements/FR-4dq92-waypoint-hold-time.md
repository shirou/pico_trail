# FR-4dq92 Waypoint Hold Time

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
  - [FR-jm7mj-auto-mode-mission-execution](FR-jm7mj-auto-mode-mission-execution.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-r7k2p-mission-execution-sequencer](../tasks/T-r7k2p-mission-execution-sequencer/README.md)

## Requirement Statement

The system shall support NAV_WAYPOINT param1 as hold time in seconds, causing the rover to stop at the waypoint for the specified duration before advancing to the next waypoint.

## Rationale

ArduPilot's NAV_WAYPOINT command uses param1 for hold time (delay in seconds). Mission Planner allows operators to set hold time per waypoint. Currently, the rover advances immediately on waypoint arrival, ignoring this parameter. This causes missions with planned stops to behave incorrectly.

## User Story (if applicable)

As a GCS operator, I want to set a hold time at specific waypoints, so that the rover pauses at observation points or waiting areas before continuing the mission.

## Acceptance Criteria

- [ ] NAV_WAYPOINT param1 interpreted as hold time in seconds
- [ ] Rover stops motors on waypoint arrival (within `WP_RADIUS`)
- [ ] Timer starts on first arrival detection
- [ ] Waypoint not advanced until hold time elapses
- [ ] MISSION_ITEM_REACHED sent after hold time expires (not on initial arrival)
- [ ] param1 == 0 means no hold (advance immediately, current behavior)
- [ ] Hold is passive (motors stop, no active position correction)

## Technical Details (if applicable)

### Functional Requirement Details

**NAV_WAYPOINT (ID: 16) Parameters:**

| Parameter | Name       | Description                                       |
| --------- | ---------- | ------------------------------------------------- |
| param1    | hold_time  | Hold time at waypoint (seconds, 0 = no hold)      |
| param2    | accept_rad | Acceptance radius (meters, 0 = default WP_RADIUS) |
| param3    | pass_rad   | Pass radius (0 = through waypoint)                |
| param4    | yaw        | Desired yaw angle (degrees)                       |

**Hold Time Logic in Auto Mode:**

```
State: Navigating
  -> at_target detected
  -> Read param1 from current waypoint
  -> If param1 == 0: send MISSION_ITEM_REACHED, advance
  -> If param1 > 0: record arrival_time, enter Holding state

State: Holding
  -> Set steering = 0, throttle = 0
  -> Check elapsed time since arrival_time
  -> If elapsed >= param1: send MISSION_ITEM_REACHED, advance
```

**State Extension:**

Auto mode needs an internal holding state with `arrival_time: Option<Instant>` to track when holding began.

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                  | Impact | Likelihood | Mitigation                            | Validation                    |
| --------------------- | ------ | ---------- | ------------------------------------- | ----------------------------- |
| Timer accuracy        | Low    | Low        | Embassy timer is millisecond-accurate | Test hold time precision      |
| GPS drift during hold | Low    | Medium     | Passive hold (no correction)          | Test GPS position during hold |
| Very long hold times  | Low    | Low        | Accept any positive value             | Test with large param1        |

## Implementation Notes

Preferred approaches:

- Add `holding_since: Option<Instant>` field to Auto mode struct
- Check hold state in `update()` before calling `advance_waypoint()`
- Use `embassy_time::Instant` for timing

Known pitfalls:

- param1 is `f32` in Waypoint struct; convert to `Duration` for comparison
- Must reset `holding_since` when advancing to next waypoint

Related code areas:

- `crates/firmware/src/rover/mode/auto.rs:223-241` - Waypoint arrival handling
- `crates/core/src/mission/mod.rs:37-62` - Waypoint struct (param1 field)

## External References

- [MAV_CMD_NAV_WAYPOINT](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT)
- [ArduPilot Rover mode_auto.cpp](https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_auto.cpp)
