# T-fuytd MAVLink Protocol Communication

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-fuytd-mavlink-communication-plan](plan.md)
- Related ADRs:
  - [ADR-ggou4-mavlink-implementation](../../adr/ADR-ggou4-mavlink-implementation.md)
- Related Requirements:
  - [FR-gpzpz-mavlink-protocol](../../requirements/FR-gpzpz-mavlink-protocol.md)
  - [FR-a1cuu-runtime-parameters](../../requirements/FR-a1cuu-runtime-parameters.md)
  - [NFR-z2iuk-memory-limits](../../requirements/NFR-z2iuk-memory-limits.md)

## Overview

Implement MAVLink 2.0 protocol communication for ground control station (GCS) integration using the rust-mavlink crate. The system will support telemetry streaming, parameter management, mission protocol, and command execution compatible with QGroundControl and Mission Planner.

## Success Metrics

- [ ] Compatible with QGroundControl 4.x (tested connection, telemetry display)
- [ ] Compatible with Mission Planner 1.3.x (tested connection, telemetry display)
- [ ] MAVLink state uses < 10 KB RAM (measured via defmt memory logging)
- [ ] 10Hz telemetry streams without dropped messages (verified via GCS logs)
- [ ] < 0.1% message corruption rate (CRC validation tracking)

## Background and Current State

- Context: The autopilot needs standardized communication with GCS for monitoring, configuration, and mission management. MAVLink is the de facto standard used by ArduPilot and PX4.
- Current behavior: No GCS communication exists. Platform abstraction (T-egg4f) and task scheduler (T-g729p) are complete and provide UART and periodic task infrastructure.
- Pain points: Without MAVLink, users cannot monitor telemetry, configure parameters, or upload missions using standard GCS software.
- Constraints:
  - no_std embedded environment
  - < 10 KB RAM budget for MAVLink buffers and state
  - UART transport at 115200 baud (11.5 KB/s theoretical max)
  - Must work on both RP2040 and RP2350 platforms
- Related ADRs: ADR-ggou4 selected rust-mavlink crate with custom message handlers

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────┐
│         Application Layer                   │
│  (Scheduler, AHRS, GPS, Parameters)         │
└───────────────┬─────────────────────────────┘
                │ (Data queries)
┌───────────────▼─────────────────────────────┐
│      MAVLink Message Handlers               │
│  - ParamHandler (PARAM_SET/REQUEST)         │
│  - MissionHandler (MISSION upload/download) │
│  - CommandHandler (COMMAND_LONG)            │
│  - TelemetryStreamer (HEARTBEAT, ATTITUDE)  │
└───────────────┬─────────────────────────────┘
                │ (MAVLink messages)
┌───────────────▼─────────────────────────────┐
│       rust-mavlink Crate                    │
│  - Message parsing (read_v2_msg)            │
│  - Message encoding (serialize)             │
│  - CRC validation (mavlink::calculate_crc)  │
└───────────────┬─────────────────────────────┘
                │ (Byte stream)
┌───────────────▼─────────────────────────────┐
│       Embassy UART Driver                   │
│  (Platform abstraction UART trait)          │
└─────────────────────────────────────────────┘
```

### Components

**Core MAVLink Module** (`src/communication/mavlink/`)

- `router.rs`: MavlinkRouter struct coordinating all handlers
- `parser.rs`: Async message reading from UART using rust-mavlink
- `writer.rs`: Async message writing to UART with buffering

**Message Handlers** (`src/communication/mavlink/handlers/`)

- `param.rs`: ParamHandler - PARAM_REQUEST_LIST, PARAM_SET, PARAM_VALUE
- `mission.rs`: MissionHandler - MISSION_COUNT, MISSION_ITEM, MISSION_ACK
- `command.rs`: CommandHandler - COMMAND_LONG, COMMAND_ACK
- `telemetry.rs`: TelemetryStreamer - HEARTBEAT, ATTITUDE, GPS_RAW_INT, SYS_STATUS

**Parameter System** (`src/core/parameters/`)

- `registry.rs`: Static parameter registry (name, type, value, min/max)
- `storage.rs`: Parameter persistence (deferred to FR-a1cuu task)

**State Tracking** (`src/communication/mavlink/state.rs`)

- `SystemState`: Current vehicle state (armed, mode, battery)
- `ConnectionState`: GCS connection status (last heartbeat time)

### Data Flow

**Telemetry Streaming (Outbound):**

1. Scheduler executes TelemetryStreamer task (10Hz for attitude, 5Hz for GPS)
2. TelemetryStreamer queries application state (AHRS for attitude, GPS for position)
3. Construct MAVLink messages (ATTITUDE, GPS_RAW_INT)
4. Serialize via rust-mavlink and write to UART TX buffer
5. Embassy UART driver transmits bytes asynchronously

**Command Reception (Inbound):**

1. Embassy UART driver receives bytes into RX buffer
2. MAVLink parser reads and validates messages (CRC check)
3. MavlinkRouter dispatches to appropriate handler based on message type
4. Handler executes command (e.g., CommandHandler arms vehicle, ParamHandler sets parameter)
5. Handler sends ACK message back to GCS

### Data Models and Types

**MAVLink Message Types** (from rust-mavlink `common` dialect):

- `HEARTBEAT` (ID 0): type, autopilot, base_mode, custom_mode, system_status
- `SYS_STATUS` (ID 1): onboard_control_sensors_present/enabled/health, load, voltage_battery, current_battery
- `ATTITUDE` (ID 30): roll, pitch, yaw, rollspeed, pitchspeed, yawspeed
- `GPS_RAW_INT` (ID 24): lat, lon, alt, eph, epv, vel, cog, fix_type, satellites_visible
- `PARAM_VALUE` (ID 22): param_id, param_value, param_type, param_count, param_index
- `COMMAND_LONG` (ID 76): command, confirmation, param1-7
- `MISSION_ITEM_INT` (ID 73): seq, frame, command, current, autocontinue, param1-4, x, y, z

**Parameter Registry Entry:**

```rust
pub struct ParamMetadata {
    pub name: &'static str,        // Max 16 chars (MAVLink limit)
    pub param_type: ParamType,     // Int32, Float, etc.
    pub value: ParamValue,         // Current value
    pub default: ParamValue,       // Default value
    pub min: Option<f32>,          // Min bound
    pub max: Option<f32>,          // Max bound
}
```

\_\_Stream Rate Configuration (ArduPilot SR\_\_ parameters):\_\*

```rust
pub struct StreamRates {
    pub sr_extra1: u8,      // ATTITUDE rate (Hz), default 10
    pub sr_position: u8,    // GPS_RAW_INT rate (Hz), default 5
    pub sr_rc_chan: u8,     // RC_CHANNELS rate (Hz), default 5
    pub sr_raw_sens: u8,    // IMU_SCALED rate (Hz), default 5
}
```

### Error Handling

- **Parse Errors**: Log CRC failures and malformed messages via defmt, increment error counter, continue processing
- **Buffer Overflow**: If TX buffer full, drop telemetry message (log warning), prioritize HEARTBEAT and ACK messages
- **Invalid Commands**: Send COMMAND_ACK with MAV_RESULT_DENIED or MAV_RESULT_UNSUPPORTED
- **Parameter Errors**: Send PARAM_VALUE with unchanged value if validation fails, log error

All error messages in English. Use defmt for embedded logging.

### Security Considerations

- **Message Signing**: Deferred to Phase 4 (optional security feature per ADR-ggou4)
- **Command Validation**: Validate COMMAND_LONG parameters before execution (e.g., reject invalid mode numbers)
- **Parameter Bounds**: Enforce min/max bounds on PARAM_SET to prevent unsafe configurations
- **Rate Limiting**: Cap telemetry stream rates to prevent UART saturation (enforce max 50Hz per stream)

### Performance Considerations

- **Zero-Copy Parsing**: rust-mavlink uses `&[u8]` slices where possible to avoid allocations
- **Buffer Sizing**:
  - RX buffer: 512 bytes (2x max MAVLink message size of 280 bytes)
  - TX buffer: 1024 bytes (allow burst of telemetry messages)
- **Async I/O**: Use Embassy async UART to avoid blocking scheduler tasks
- **Hot Paths**: TelemetryStreamer update() is called at 10Hz, optimize message construction
- **Statistics Tracking**: Count sent/received messages per type, log every 10 seconds via monitor task

### Platform Considerations

#### RP2040 (Pico W)

- UART0 or UART1 at 115200 baud (pins configurable)
- 264 KB RAM total, < 10 KB for MAVLink (< 4% of RAM)
- Cortex-M0+ (no FPU), avoid heavy floating-point in hot paths

#### RP2350 (Pico 2 W)

- UART0 or UART1 at 115200 baud (pins configurable)
- 520 KB RAM total, < 10 KB for MAVLink (< 2% of RAM)
- Cortex-M33 with FPU, floating-point conversions efficient

#### Cross-Platform

- Use platform abstraction UART trait from T-egg4f
- UART configuration (baud rate, pins) passed at initialization
- No platform-specific code outside `src/platform/`

## Alternatives Considered

1. **Custom MAVLink Implementation**
   - Pros: Minimal code size (\~20 KB Flash vs \~50 KB), full control
   - Cons: High development effort, maintenance burden, potential bugs
   - Decision: Rejected per ADR-ggou4, rust-mavlink provides faster development and proven correctness

2. **C mavlink Library via FFI**
   - Pros: Reference implementation (ArduPilot/PX4 use it), maximum compatibility
   - Cons: FFI overhead, unsafe code, requires C compiler, not idiomatic Rust
   - Decision: Rejected per ADR-ggou4, prefer pure Rust solution

3. **Synchronous Blocking UART**
   - Pros: Simpler implementation (no async)
   - Cons: Blocks scheduler tasks, unacceptable latency for control loops
   - Decision: Rejected, Embassy async required for non-blocking I/O

## Decision Rationale

- **rust-mavlink**: Chosen for development velocity, correctness, and no_std support (per ADR-ggou4)
- **Custom Handlers**: rust-mavlink provides parsing, but application logic (parameter storage, mission handling) requires custom implementation
- **Embassy Async**: Required to avoid blocking scheduler tasks during UART I/O
- **Common Dialect**: Use `common` message set for maximum GCS compatibility and minimal code size

Trade-offs accepted:

- **Code Size**: \~50 KB Flash for rust-mavlink vs \~20 KB custom (acceptable, Pico has 2 MB Flash)
- **RAM Usage**: \~8 KB for buffers vs \~4 KB minimal (acceptable, < 4% of RAM on RP2040)

## Migration and Compatibility

- Backward/forward compatibility: New feature, no migration required
- Rollout plan: Single-phase enablement, MAVLink task spawned at startup
- Deprecation plan: N/A

## Testing Strategy

### Unit Tests

- **Parameter Registry**: Test parameter lookup by name, bounds validation, type conversion
- **Message Construction**: Test HEARTBEAT, ATTITUDE, GPS_RAW_INT message serialization with known values
- **Command Parsing**: Test COMMAND_LONG parsing and validation logic
- **Stream Rate Calculation**: Test `should_send()` logic for 1Hz, 5Hz, 10Hz rates

Place tests in `#[cfg(test)]` modules within each file.

### Integration Tests

- **GCS Simulation**: Create mock GCS that sends PARAM_REQUEST_LIST, COMMAND_LONG, verify responses
- **Telemetry Loop**: Run TelemetryStreamer task, capture UART output, verify message rates and content
- **Round-Trip**: Send PARAM_SET, verify PARAM_VALUE response and parameter actually changed

Note: Hardware integration tests require RP2350 target, use `#[cfg(feature = "pico2_w")]`.

### External GCS Testing

- **QGroundControl**: Connect Pico 2 W via UART, verify telemetry display, parameter editing
- **Mission Planner**: Connect via UART, verify telemetry, attempt mission upload
- Test plan documented in Phase 4 acceptance criteria

### Performance Testing

- **Message Rate**: Run all telemetry streams at max rates (10Hz attitude, 5Hz GPS, 1Hz heartbeat), verify no dropped messages via GCS logs
- **CPU Load**: Monitor scheduler CPU load during full telemetry streaming, verify < 10% overhead
- **Latency**: Measure COMMAND_LONG to COMMAND_ACK round-trip time, target < 50ms

## Documentation Impact

- Add `docs/mavlink.md` with usage guide (connecting GCS, configuring stream rates, supported messages)
- Update `docs/architecture.md` with MAVLink component diagram
- Add code examples in message handler files
- Update `README.md` with GCS compatibility section

## External References

- MAVLink Protocol: <https://mavlink.io/en/>
- rust-mavlink Crate: <https://crates.io/crates/mavlink>
- MAVLink Common Messages: <https://mavlink.io/en/messages/common.html>
- ArduPilot MAVLink Basics: <https://ardupilot.org/dev/docs/mavlink-basics.html>
- QGroundControl: <https://qgroundcontrol.com/>
- Mission Planner: <https://ardupilot.org/planner/>

## Open Questions

- [ ] Should we implement all stream rate parameters or start with a subset? → Next step: Start with essential streams (HEARTBEAT, ATTITUDE, GPS), add RC channels and raw sensors if needed
- [ ] How to handle parameter storage persistence (flash writes)? → Decision: Defer to FR-a1cuu task, keep parameters in RAM only for this task
- [ ] Should we support USB CDC as alternative to UART? → Method: USB support as future enhancement, UART-only for initial implementation
- [ ] What system ID and component ID to use? → Decision: System ID 1 (configurable parameter), Component ID MAV_COMP_ID_AUTOPILOT1 (1)

## Appendix

### Message Priority

High priority (never drop):

- HEARTBEAT
- COMMAND_ACK
- PARAM_VALUE (in response to request)

Normal priority (drop if buffer full):

- ATTITUDE
- GPS_RAW_INT
- SYS_STATUS
- RC_CHANNELS

### Telemetry Message Sizes

- HEARTBEAT: 9 bytes payload + 12 bytes header = 21 bytes
- ATTITUDE: 28 bytes payload + 12 bytes header = 40 bytes
- GPS_RAW_INT: 30 bytes payload + 12 bytes header = 42 bytes
- SYS_STATUS: 31 bytes payload + 12 bytes header = 43 bytes

At 115200 baud (11.5 KB/s):

- 10Hz ATTITUDE: 400 bytes/s (3.5% bandwidth)
- 5Hz GPS_RAW_INT: 210 bytes/s (1.8% bandwidth)
- 1Hz HEARTBEAT: 21 bytes/s (0.2% bandwidth)
- Total: \~631 bytes/s (\~5.5% bandwidth)

### Example Message Flow

**Parameter Request:**

```text
GCS → Autopilot: PARAM_REQUEST_LIST()
Autopilot → GCS: PARAM_VALUE("SR_EXTRA1", 10.0, MAV_PARAM_TYPE_UINT8, 150, 0)
Autopilot → GCS: PARAM_VALUE("SR_POSITION", 5.0, MAV_PARAM_TYPE_UINT8, 150, 1)
... (repeat for all 150 parameters)
```

**Arm Command:**

```text
GCS → Autopilot: COMMAND_LONG(MAV_CMD_COMPONENT_ARM_DISARM, param1=1.0)
Autopilot: [Validates safety checks, arms motors]
Autopilot → GCS: COMMAND_ACK(MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED)
```

### Glossary

- **GCS**: Ground Control Station (QGroundControl, Mission Planner)
- **MAVLink**: Micro Air Vehicle Link, lightweight messaging protocol
- **SR\_**\*: Stream Rate parameters (ArduPilot convention for telemetry rates)
- **CRC**: Cyclic Redundancy Check (message integrity validation)
- **System ID**: Unique identifier for vehicle (1-255)
- **Component ID**: Identifier for component within vehicle (autopilot=1, gimbal=154, etc.)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
