# FR-ahzhr Mission Command Preservation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-l8emi-mission-execution-gaps](../analysis/AN-l8emi-mission-execution-gaps.md)
- Related ADRs:
  - [ADR-0yapl-mission-execution-telemetry-architecture](../adr/ADR-0yapl-mission-execution-telemetry-architecture.md)
- Prerequisite Requirements: N/A
- Dependent Requirements: N/A
- Related Tasks:
  - [T-r7k2p-mission-execution-sequencer](../tasks/T-r7k2p-mission-execution-sequencer/README.md)

## Requirement Statement

The `waypoint_to_mission_item()` conversion function shall preserve the original command ID and frame type when converting stored waypoints to MAVLink mission items for download.

## Rationale

The current implementation has a bug: `waypoint_to_mission_item()` in `mission.rs:518-521` falls back all command IDs to `MAV_CMD_NAV_WAYPOINT`, causing data corruption during mission download. When a GCS uploads a mission with DO_CHANGE_SPEED, DO_SET_SERVO, or other non-waypoint commands, downloading the mission shows all items as NAV_WAYPOINT. This is a data integrity issue that affects:

- Mission verification (GCS cannot verify uploaded mission)
- Mission editing (re-downloaded mission loses command types)
- Protocol compliance (mission round-trip must be lossless)

## User Story (if applicable)

As a GCS operator, I want to download a mission and see the same command types I uploaded, so that I can verify and edit the mission correctly.

## Acceptance Criteria

- [ ] `waypoint_to_mission_item()` preserves original command ID from Waypoint struct
- [ ] All MAV_CMD values supported (not just NAV_WAYPOINT)
- [ ] `frame` field preserves original `MAV_FRAME` value (not just `MAV_FRAME_GLOBAL_RELATIVE_ALT`)
- [ ] Mission round-trip is lossless: upload → download produces identical command IDs and frames for all commands representable by the `MavCmd` enum
- [ ] Behavior for command IDs that cannot be mapped to a known `MavCmd` enum value is explicitly defined, documented, and covered by tests (e.g., a consistent fallback strategy)

## Technical Details (if applicable)

### Functional Requirement Details

**Current Bug (mission.rs:514-521):**

```rust
// CURRENT - DATA LOSS
let command = match wp.command {
    16 => MavCmd::MAV_CMD_NAV_WAYPOINT,
    _ => MavCmd::MAV_CMD_NAV_WAYPOINT, // ALL COMMANDS BECOME NAV_WAYPOINT
};
```

**Fix:**

```rust
// FIXED - Preserve original command ID
let command = MavCmd::from(wp.command);
// Or use a lookup that handles the u16 -> MavCmd conversion
```

The `MavCmd` enum in the MAVLink library should support `From<u16>` or similar conversion. If not, use the raw integer in the MAVLink message fields.

**Frame Conversion (same bug pattern):**

```rust
// CURRENT - DATA LOSS
let frame = match wp.frame {
    3 => MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT,
    _ => MavFrame::MAV_FRAME_GLOBAL_RELATIVE_ALT, // ALL FRAMES BECOME RELATIVE_ALT
};
```

Must also be fixed to preserve the original frame value.

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                        | Impact | Likelihood | Mitigation                          | Validation                           |
| --------------------------- | ------ | ---------- | ----------------------------------- | ------------------------------------ |
| Unknown MavCmd enum variant | Low    | Medium     | Use raw u16 if enum lacks variant   | Test with non-standard command IDs   |
| Protocol incompatibility    | Low    | Low        | Follow MAVLink spec for field types | Test round-trip with Mission Planner |

## Implementation Notes

Preferred approaches:

- Fix the match statement to use proper enum conversion
- Test with a mission containing mixed command types (NAV_WAYPOINT + DO_CHANGE_SPEED)

Known pitfalls:

- The MAVLink Rust library may not have all MavCmd variants; use raw integer conversion as fallback
- Frame values also need the same fix

Related code areas:

- `crates/firmware/src/communication/mavlink/handlers/mission.rs:514-521` - Command fallback bug
- `crates/firmware/src/communication/mavlink/handlers/mission.rs:507-540` - `waypoint_to_mission_item()`

## External References

- [MAVLink Commands (MAV_CMD)](https://mavlink.io/en/messages/common.html#mav_commands)
- [MAVLink Frame (MAV_FRAME)](https://mavlink.io/en/messages/common.html#MAV_FRAME)
