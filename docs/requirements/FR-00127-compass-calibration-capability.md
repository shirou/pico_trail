# FR-00127 Compass Calibration Capability Advertisement

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00038-compass-calibration-via-mission-planner](../analysis/AN-00038-compass-calibration-via-mission-planner.md)
- Prerequisite Requirements:
  - [FR-00005-mavlink-protocol](FR-00005-mavlink-protocol.md)
- Dependent Requirements:
  - [FR-00128-fixed-mag-cal-yaw-handler](FR-00128-fixed-mag-cal-yaw-handler.md)
- Related Tasks:
  - [T-00034-compass-calibration](../tasks/T-00034-compass-calibration/README.md)

## Requirement Statement

The system shall advertise the `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` capability in the `AUTOPILOT_VERSION` message to enable calibration UI in ground control stations.

## Rationale

Mission Planner and other ground control stations check the `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` flag in the `AUTOPILOT_VERSION` message to determine whether to enable calibration UI. Without this capability advertised:

- Calibration buttons in Mission Planner are disabled
- Users cannot access "Large Vehicle MagCal" feature
- No visual indication that compass calibration is supported

This is a prerequisite for any compass calibration workflow.

## User Story (if applicable)

As an operator using Mission Planner, I want the compass calibration UI to be enabled, so that I can access and use the Large Vehicle MagCal feature for field calibration.

## Acceptance Criteria

- [ ] `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` flag is included in AUTOPILOT_VERSION capabilities
- [ ] Mission Planner displays enabled "Large Vehicle MagCal" button when connected
- [ ] Capability value 4096 (0x1000) is correctly set in the capabilities bitmask
- [ ] Existing capabilities (MISSION_INT, COMMAND_INT, MAVLINK2, etc.) remain unchanged

## Technical Details (if applicable)

### Functional Requirement Details

**Current Capabilities** (`src/communication/mavlink/handlers/command.rs`):

```rust
let capabilities = MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_INT
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT;
```

**Required Change:**

```rust
let capabilities = MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_INT
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;  // ADD
```

**MAVLink Definition (common.xml):**

```xml
<entry value="4096" name="MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION">
  <description>Autopilot supports onboard compass calibration.</description>
</entry>
```

**Mission Planner Behavior:**

| Capability Flag | Mission Planner Behavior         |
| --------------- | -------------------------------- |
| Present         | Shows Onboard Calibration option |
| Absent          | Calibration buttons disabled     |

## Platform Considerations

### Cross-Platform

N/A - Platform agnostic (MAVLink capability advertisement)

## Risks & Mitigation

| Risk                                    | Impact | Likelihood | Mitigation                               | Validation                         |
| --------------------------------------- | ------ | ---------- | ---------------------------------------- | ---------------------------------- |
| Capability alone enables incomplete UI  | Low    | Low        | Implement command handler simultaneously | Test full calibration workflow     |
| Other capabilities accidentally removed | Medium | Low        | Use bitwise OR, preserve existing flags  | Verify all capabilities in testing |

## Implementation Notes

- Modify capability bitmask in AUTOPILOT_VERSION handler
- Location: `src/communication/mavlink/handlers/command.rs`
- Single line addition using bitwise OR
- Test by connecting Mission Planner and verifying calibration UI is enabled

## External References

- [MAV_PROTOCOL_CAPABILITY enum](https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY)
- [Mission Planner PR #1550 - Compass calibration capability check](https://github.com/ArduPilot/MissionPlanner/pull/1550)
