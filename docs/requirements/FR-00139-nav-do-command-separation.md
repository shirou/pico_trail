# FR-00139 NAV DO Command Separation

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
  - [FR-00082-auto-mode-mission-execution](FR-00082-auto-mode-mission-execution.md)
- Dependent Requirements:
  - [FR-00133-do-change-speed](FR-00133-do-change-speed.md)
  - [NFR-00090-mission-command-execution-overhead](NFR-00090-mission-command-execution-overhead.md)
- Related Tasks:
  - [T-00037-mission-execution-sequencer](../tasks/T-00037-mission-execution-sequencer/README.md)

## Requirement Statement

The Auto mode mission executor shall classify mission items as NAV commands (command ID <= 95) or DO commands, executing NAV commands as navigation targets and DO commands as immediate actions without affecting navigation.

## Rationale

ArduPilot missions contain two types of commands:

- **NAV commands** (ID <= `MAV_CMD_NAV_LAST` = 95): Control vehicle position (e.g., NAV_WAYPOINT, NAV_LOITER)
- **DO commands** (ID > 95): Perform immediate actions (e.g., DO_CHANGE_SPEED, DO_SET_SERVO)

Currently all mission items are treated as NAV_WAYPOINT regardless of command type. DO commands stored in missions are ignored during execution, and attempting to navigate to a DO command (which has no valid lat/lon) causes undefined behavior.

## User Story (if applicable)

As a GCS operator, I want missions containing both waypoints and DO commands to execute correctly, so that I can create complex missions with speed changes and servo actions at specific points.

## Acceptance Criteria

- [ ] Mission items classified as NAV (command ID <= 95) or DO (command ID > 95)
- [ ] NAV commands drive vehicle navigation (pass to navigation controller)
- [ ] DO commands execute immediately without affecting navigation target
- [ ] DO commands between two NAV commands execute when the preceding NAV starts
- [ ] Unknown DO commands are skipped with a log warning
- [ ] Mission advancement driven by NAV command completion only

## Technical Details (if applicable)

### Functional Requirement Details

**Command Classification:**

```rust
fn is_nav_command(command_id: u16) -> bool {
    command_id <= 95  // MAV_CMD_NAV_LAST
}
```

**Execution Model (ArduPilot-style):**

The Auto mode update loop processes two slots:

1. **NAV slot**: Currently executing navigation command
2. **DO slot**: DO commands associated with the current NAV command

When advancing to the next NAV command, scan forward and execute any DO commands encountered before the next NAV command.

**Example Mission:**

```
seq 0: NAV_WAYPOINT (lat1, lon1)     <- NAV: navigate here
seq 1: DO_CHANGE_SPEED (5 m/s)       <- DO: execute when seq 0 starts
seq 2: NAV_WAYPOINT (lat2, lon2)     <- NAV: navigate here
seq 3: DO_CHANGE_SPEED (2 m/s)       <- DO: execute when seq 2 starts
seq 4: NAV_WAYPOINT (lat3, lon3)     <- NAV: navigate here
```

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                          | Impact | Likelihood | Mitigation                            | Validation                        |
| ----------------------------- | ------ | ---------- | ------------------------------------- | --------------------------------- |
| DO command affects nav timing | Medium | Medium     | Execute synchronously in update cycle | Test timing with DO commands      |
| Mission with only DO commands | Low    | Low        | Complete immediately                  | Test edge case                    |
| Unknown command ID            | Medium | Medium     | Skip with warning, advance            | Test with unsupported command IDs |

## Implementation Notes

Preferred approaches:

- Modify Auto mode `update()` in `crates/firmware/src/rover/mode/auto.rs`
- Add `is_nav_command()` helper to `crates/core/src/mission/mod.rs`
- Process DO commands in a loop before starting NAV navigation
- Use a simple forward scan rather than full dual-slot model for initial implementation

Related code areas:

- `crates/firmware/src/rover/mode/auto.rs:195-208` - Current update loop (treats all as NAV)
- `crates/core/src/mission/mod.rs:37-62` - Waypoint struct with `command: u16`

## External References

- [ArduPilot Mission Commands](https://ardupilot.org/rover/docs/common-mavlink-mission-command-messages-mav_cmd.html)
- [MAVLink Commands (MAV_CMD)](https://mavlink.io/en/messages/common.html#mav_commands)
