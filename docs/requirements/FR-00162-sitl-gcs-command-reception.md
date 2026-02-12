# FR-00162 SITL GCS Command Reception

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Prerequisite Requirements:
  - [FR-00154-sitl-platform-trait](FR-00154-sitl-platform-trait.md)
  - [FR-00155-sitl-vehicle-mavlink-ports](FR-00155-sitl-vehicle-mavlink-ports.md)
- Dependent Requirements:
  - [FR-00163-sitl-autopilot-loop-integration](FR-00163-sitl-autopilot-loop-integration.md)
- Related Tasks:
  - [T-00164-sitl-autopilot-core-integration](../tasks/T-00164-sitl-autopilot-core-integration/README.md)

## Requirement Statement

The SITL GCS link shall receive, parse, and process MAVLink commands from ground control stations, translating them into vehicle actions including arming/disarming, mode changes, and manual motor control via RC channel overrides.

## Rationale

Currently the SITL GCS link only sends telemetry to Mission Planner. Without command reception, operators cannot control the simulated vehicle. GCS command processing is the essential bridge between the operator's intent and the simulated vehicle's behavior.

## User Story (if applicable)

As a developer using SITL with Mission Planner, I want to arm the rover, switch to manual mode, and steer it with the joystick so that I can verify the vehicle responds correctly in Gazebo.

## Acceptance Criteria

- [ ] GcsLink processes `COMMAND_LONG` with `MAV_CMD_COMPONENT_ARM_DISARM` to arm/disarm the vehicle
- [ ] GcsLink processes `SET_MODE` to change the vehicle's reported flight mode
- [ ] GcsLink processes `RC_CHANNELS_OVERRIDE` to map RC channels to motor outputs
- [ ] GcsLink processes `MANUAL_CONTROL` for joystick input
- [ ] Armed state is reflected in HEARTBEAT `base_mode` field
- [ ] Current mode is reflected in HEARTBEAT `custom_mode` field
- [ ] Disarmed vehicles send zero motor output regardless of RC input
- [ ] Command ACK (`COMMAND_ACK`) is sent for all `COMMAND_LONG` messages
- [ ] Per-vehicle command routing works correctly (system_id matching)
- [ ] Unit tests for each command type

## Technical Details (if applicable)

### MAVLink Messages to Handle

| Message                                       | Action                                           |
| --------------------------------------------- | ------------------------------------------------ |
| `COMMAND_LONG (MAV_CMD_COMPONENT_ARM_DISARM)` | Set vehicle armed/disarmed state                 |
| `SET_MODE`                                    | Update flight mode for heartbeat reporting       |
| `RC_CHANNELS_OVERRIDE`                        | Map channels to steering/throttle, mix to motors |
| `MANUAL_CONTROL`                              | Map x/y/z/r to steering/throttle                 |

### RC Channel Mapping (ArduPilot Rover Convention)

- Channel 1: Steering (-500 to +500 around 1500 center)
- Channel 3: Throttle (-500 to +500 around 1500 center)

### Motor Mixing

Uses `pico_trail_core::kinematics::DifferentialDrive` for steering+throttle to left/right motor conversion.

## Platform Considerations

### Host Only

- Runs on host (Linux/macOS/Windows) with tokio async runtime
- MAVLink over TCP (current GcsLink implementation)

## Risks & Mitigation

| Risk                                              | Impact | Likelihood | Mitigation                                    | Validation                        |
| ------------------------------------------------- | ------ | ---------- | --------------------------------------------- | --------------------------------- |
| Mission Planner sends unexpected message variants | Low    | Medium     | Log unhandled messages, handle gracefully     | Integration testing               |
| RC channel mapping differs across GCS apps        | Medium | Low        | Follow ArduPilot convention, document mapping | Test with Mission Planner and QGC |

## Implementation Notes

- ADR-00161 migrates the autopilot integration layer (MessageDispatcher, handlers) from firmware to core
- SITL GcsLink delegates to core's `MessageDispatcher` for command processing rather than reimplementing handlers
- Motor mixing uses core's `DifferentialDrive` via the mode implementations migrated to core

## External References

- [MAVLink Common Messages](https://mavlink.io/en/messages/common.html)
- [ArduPilot Rover RC Input](https://ardupilot.org/rover/docs/rover-first-drive.html)
