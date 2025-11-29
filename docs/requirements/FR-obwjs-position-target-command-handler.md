# FR-obwjs Position Target Command Handler

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-27568-position-target-navigation](../analysis/AN-27568-position-target-navigation.md)
- Prerequisite Requirements:
  - [FR-gpzpz-mavlink-protocol](FR-gpzpz-mavlink-protocol.md)
  - [FR-cs42u-gps-navigation-state-access](FR-cs42u-gps-navigation-state-access.md)
  - [FR-tmibt-position-target-state](FR-tmibt-position-target-state.md)
- Dependent Requirements:
  - [FR-erpze-guided-mode-navigation](FR-erpze-guided-mode-navigation.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall receive and process SET_POSITION_TARGET_GLOBAL_INT MAVLink messages to accept position target commands from GCS or companion computers.

## Rationale

SET_POSITION_TARGET_GLOBAL_INT is the standard MAVLink message for commanding vehicle position in Guided mode. This enables:

- GCS "Fly To Here" functionality (click on map to command movement)
- Companion computer control for automated missions
- Real-time position target updates during operation

## User Story (if applicable)

As a GCS operator, I want to click on the map to command the vehicle to a specific GPS position, so that I can dynamically control the vehicle's destination without uploading a full mission.

## Acceptance Criteria

- [ ] System parses SET_POSITION_TARGET_GLOBAL_INT message fields correctly
- [ ] lat_int and lon_int fields converted from degE7 to internal representation
- [ ] type_mask validated to determine control mode (position, velocity, yaw)
- [ ] Position-only mode (type_mask 0x0DFC) supported
- [ ] Invalid coordinate_frame values rejected with COMMAND_NAK
- [ ] Target position stored in NavigationState for navigation subsystem
- [ ] Message ignored if not in Guided mode (or triggers mode switch if allowed)
- [ ] Unit tests cover all supported type_mask combinations

## Technical Details (if applicable)

### Functional Requirement Details

**Message Structure (SET_POSITION_TARGET_GLOBAL_INT, ID: 86):**

| Field            | Type   | Unit  | Description                     |
| ---------------- | ------ | ----- | ------------------------------- |
| time_boot_ms     | uint32 | ms    | Timestamp since boot            |
| target_system    | uint8  | -     | Target system ID                |
| target_component | uint8  | -     | Target component ID             |
| coordinate_frame | uint8  | -     | MAV_FRAME enum                  |
| type_mask        | uint16 | -     | Bitmask for ignored fields      |
| lat_int          | int32  | degE7 | Latitude (degrees × 10^7)       |
| lon_int          | int32  | degE7 | Longitude (degrees × 10^7)      |
| alt              | float  | m     | Altitude                        |
| vx, vy, vz       | float  | m/s   | Velocity components (NED frame) |
| yaw              | float  | rad   | Yaw setpoint                    |
| yaw_rate         | float  | rad/s | Yaw rate setpoint               |

**Supported type_mask Values (Phase 1):**

| Mode     | Hex    | Decimal | Description      |
| -------- | ------ | ------- | ---------------- |
| Position | 0x0DFC | 3580    | Use lat/lon only |

**Coordinate Frame Support:**

- MAV_FRAME_GLOBAL (0): Altitude relative to MSL
- MAV_FRAME_GLOBAL_RELATIVE_ALT (3): Altitude relative to home

**Unit Conversions:**

```
latitude_deg = lat_int / 1e7
longitude_deg = lon_int / 1e7
```

**Error Handling:**

- Unsupported type_mask: Log warning, ignore message
- Invalid coordinate_frame: Send COMMAND_NAK
- Position outside geofence: Reject (future enhancement)

## Platform Considerations

### Pico W (RP2040)

N/A - Message parsing is platform-independent.

### Pico 2 W (RP2350)

N/A - Message parsing is platform-independent.

### Cross-Platform

Message handler implementation must work identically on all platforms.

## Risks & Mitigation

| Risk                            | Impact | Likelihood | Mitigation                            | Validation                              |
| ------------------------------- | ------ | ---------- | ------------------------------------- | --------------------------------------- |
| Precision loss in degE7 to f32  | Low    | Medium     | Use f64 for intermediate calculations | Test with known coordinates             |
| Invalid type_mask causes crash  | High   | Low        | Validate type_mask before processing  | Unit tests with all type_mask values    |
| Message flood overwhelms system | Medium | Low        | Rate-limit position target updates    | Stress test with rapid message sequence |

## Implementation Notes

Preferred approaches:

- Create `PositionTargetHandler` in `src/communication/mavlink/handlers/`
- Register handler in MessageDispatcher for SET_POSITION_TARGET_GLOBAL_INT
- Store target in `NavigationState.target_position: Option<PositionTarget>`
- Use `PositionTarget` struct with lat, lon, alt, timestamp, validity

Known pitfalls:

- degE7 is signed int32 - handle negative latitudes/longitudes correctly
- type_mask bits indicate fields to IGNORE, not fields to use
- Altitude interpretation depends on coordinate_frame

Related code areas:

- `src/communication/mavlink/handlers/` - Handler implementations
- `src/communication/mavlink/state.rs` - NavigationState
- `src/communication/mavlink/router.rs` - Message routing

## External References

- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)
- [Rover Commands in Guided Mode](https://ardupilot.org/dev/docs/mavlink-rover-commands.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
