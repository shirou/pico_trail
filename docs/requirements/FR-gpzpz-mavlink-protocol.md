# FR-gpzpz MAVLink Protocol Communication

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements: N/A
- Dependent Requirements:
  - [FR-4e922-data-logging](FR-4e922-data-logging.md)
  - [FR-a1cuu-runtime-parameters](FR-a1cuu-runtime-parameters.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall implement MAVLink protocol for communication with ground control stations (GCS), supporting essential message types for telemetry streaming, parameter management, mission upload/download, and command execution.

## Rationale

MAVLink is the de facto standard protocol for autopilot-to-GCS communication. Supporting MAVLink enables:

- **GCS Compatibility**: Works with QGroundControl, Mission Planner, and other standard tools
- **Real-time Telemetry**: Stream position, attitude, battery status to GCS
- **Parameter Configuration**: Tune vehicle parameters without reflashing firmware
- **Mission Management**: Upload waypoint missions, monitor progress
- **Command Execution**: Arm/disarm, mode changes, calibration procedures

ArduPilot and PX4 both use MAVLink, making it essential for compatibility and community adoption.

## User Story (if applicable)

As a GCS operator, I want to communicate with the autopilot using standard MAVLink protocol, so that I can plan missions, monitor telemetry, adjust parameters, and command the vehicle using familiar ground control software.

## Acceptance Criteria

- [ ] System implements MAVLink 2.0 protocol (backwards compatible with MAVLink 1.0)
- [ ] Compatible with QGroundControl (tested with latest stable release)
- [ ] Compatible with Mission Planner (tested on Windows)
- [ ] Supports common messages: HEARTBEAT, ATTITUDE, GPS_RAW_INT, RC_CHANNELS, SYS_STATUS
- [ ] Supports parameter protocol: PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET, PARAM_VALUE
- [ ] Supports mission protocol: MISSION_REQUEST_LIST, MISSION_COUNT, MISSION_ITEM, MISSION_ACK
- [ ] Supports command protocol: COMMAND_LONG, COMMAND_ACK for common commands (arm, disarm, mode change)
- [ ] Telemetry streams at configurable rates (default 10Hz for attitude, 5Hz for GPS)
- [ ] Message signing optional (for secured links)

## Technical Details (if applicable)

### Functional Requirement Details

**Essential Message Types:**

**Heartbeat & Status:**

- `HEARTBEAT` (1Hz) - Autopilot type, mode, armed status
- `SYS_STATUS` (1Hz) - Battery voltage, current, CPU load

**Sensor Data:**

- `ATTITUDE` (10Hz) - Roll, pitch, yaw
- `GPS_RAW_INT` (5Hz) - Latitude, longitude, altitude, fix type
- `RC_CHANNELS` (5Hz) - RC input values

**Parameter Protocol:**

- `PARAM_REQUEST_LIST` - GCS requests all parameters
- `PARAM_REQUEST_READ` - GCS requests specific parameter
- `PARAM_SET` - GCS sets parameter value
- `PARAM_VALUE` - Autopilot sends parameter value

**Mission Protocol:**

- `MISSION_REQUEST_LIST` - GCS requests mission waypoint count
- `MISSION_COUNT` - Autopilot sends waypoint count
- `MISSION_ITEM` - Waypoint data (lat, lon, alt, command)
- `MISSION_ACK` - Acknowledge mission upload/download

**Command Protocol:**

- `COMMAND_LONG` - Generic command (arm, disarm, mode change, calibration)
- `COMMAND_ACK` - Acknowledge command execution

**Telemetry Streams:**

Configurable via `SR_*` parameters (ArduPilot convention):

- `SR_EXTRA1`: ATTITUDE, 10Hz default
- `SR_POSITION`: GPS_RAW_INT, 5Hz default
- `SR_RC_CHAN`: RC_CHANNELS, 5Hz default
- `SR_RAW_SENS`: IMU_SCALED, 5Hz default

**Transport:**

- Primary: UART (115200 baud default, configurable up to 921600)
- Optional: USB CDC (if platform supports)

## Platform Considerations

### Unix

N/A - Platform agnostic (embedded system)

### Windows

N/A - Platform agnostic (embedded system)

### Cross-Platform

MAVLink implementation must work across Pico W and Pico 2 W. Use platform UART abstraction.

## Risks & Mitigation

| Risk                                  | Impact | Likelihood | Mitigation                                                  | Validation                                 |
| ------------------------------------- | ------ | ---------- | ----------------------------------------------------------- | ------------------------------------------ |
| MAVLink buffer overflow at high rates | Medium | Low        | Implement flow control, limit telemetry rates, size buffers | Stress test with maximum telemetry rates   |
| GCS incompatibility (QGC/MP versions) | High   | Low        | Use standard mavlink crate, test with latest GCS versions   | Test with QGC 4.x and MP 1.3.x             |
| Message parsing CPU overhead          | Low    | Low        | Use zero-copy parsing where possible, profile CPU usage     | Measure CPU usage during telemetry streams |
| Parameter sync issues (stale values)  | Medium | Medium     | Send PARAM_VALUE after every PARAM_SET, cache in RAM        | Test parameter changes via GCS             |

## Implementation Notes

Preferred approaches:

- Use **`mavlink` Rust crate** (<https://crates.io/crates/mavlink>) for message parsing/generation
- Define message set via `common.xml` and `ardupilotmega.xml` (standard ArduPilot messages)
- Implement MAVLink router pattern (single UART, multiple message handlers)

Known pitfalls:

- MAVLink buffers can be large (\~8 KB) - size appropriately for platform RAM
- Message signing adds overhead - make optional
- Telemetry rates too high can saturate UART - enforce rate limits
- Parameter names limited to 16 characters - use abbreviations

Related code areas:

- `src/communication/mavlink/` - MAVLink protocol implementation
- `src/communication/telemetry/` - Telemetry streaming
- `src/core/parameters/` - Parameter system
- `src/platform/*/uart.rs` - UART abstraction

Suggested libraries:

- `mavlink` crate for protocol parsing/generation
- `heapless` for fixed-size buffers (no_std compatible)

## External References

- MAVLink Protocol: <https://mavlink.io/en/>
- MAVLink Common Messages: <https://mavlink.io/en/messages/common.html>
- MAVLink Rust Crate: <https://crates.io/crates/mavlink>
- QGroundControl: <https://qgroundcontrol.com/>
- Mission Planner: <https://ardupilot.org/planner/>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
