# ADR-ggou4 MAVLink Implementation: rust-mavlink Crate with Custom Message Handlers

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-gpzpz-mavlink-protocol](../requirements/FR-gpzpz-mavlink-protocol.md)
  - [FR-a1cuu-runtime-parameters](../requirements/FR-a1cuu-runtime-parameters.md)
  - [NFR-z2iuk-memory-limits](../requirements/NFR-z2iuk-memory-limits.md)
- Supersedes ADRs: N/A
- Related Tasks: Will be created after approval

## Context

The autopilot must communicate with ground control stations (QGroundControl, Mission Planner) using the MAVLink protocol. MAVLink provides:

- **Telemetry streaming**: Position, attitude, battery status
- **Parameter management**: Read/write parameters via GCS
- **Mission protocol**: Upload/download waypoint missions
- **Command execution**: Arm/disarm, mode changes, calibration

### Problem

We need a MAVLink implementation that:

- Supports MAVLink 2.0 (backwards compatible with 1.0)
- Works on resource-constrained microcontrollers (< 8 KB RAM for buffers)
- Integrates with async Embassy executor (non-blocking I/O)
- Compatible with ArduPilot message set (common + ardupilotmega)

### Constraints

- **Memory Budget**: < 10 KB RAM for MAVLink buffers and state
- **no_std**: Must work without standard library (embedded Rust)
- **UART Transport**: Primary communication over UART (115200 baud default)
- **Message Size**: MAVLink messages limited to 280 bytes (MAVLink 2.0 max)

### Prior Art

- **rust-mavlink crate**: Official Rust MAVLink implementation, supports message generation from XML
- **pymavlink**: Python implementation used by ArduPilot (reference implementation)
- **Custom Implementation**: Write MAVLink parser/encoder from scratch

## Success Metrics

- **Compatibility**: Works with QGroundControl 4.x and Mission Planner 1.3.x
- **Memory**: MAVLink state < 10 KB RAM
- **Throughput**: Support 10Hz telemetry streams without dropped messages
- **Reliability**: < 0.1% message corruption rate (CRC validation)

## Decision

**We will use the `mavlink` Rust crate with custom message handlers for parameter, mission, and command protocols.**

### Architecture

```
┌─────────────────────────────────────────────┐
│         Application Layer                   │
│  (Parameter System, Mission, Command)       │
└───────────────┬─────────────────────────────┘
                │
┌───────────────▼─────────────────────────────┐
│      MAVLink Message Handlers               │
│  (ParamHandler, MissionHandler, etc.)       │
└───────────────┬─────────────────────────────┘
                │
┌───────────────▼─────────────────────────────┐
│       rust-mavlink Crate                    │
│  (Message Parsing, Encoding, CRC)           │
└───────────────┬─────────────────────────────┘
                │
┌───────────────▼─────────────────────────────┐
│       Embassy UART Driver                   │
│  (Async Serial Communication)               │
└─────────────────────────────────────────────┘
```

### Decision Drivers

1. **Ecosystem**: rust-mavlink is the official Rust implementation, well-maintained
2. **Code Generation**: Automatically generates message types from XML definitions
3. **no_std Support**: rust-mavlink works in embedded (no_std) environments
4. **Compatibility**: Supports ArduPilot message set (common.xml + ardupilotmega.xml)
5. **Development Velocity**: Avoid reimplementing message parsing/encoding

### Considered Options

- **Option A: rust-mavlink Crate** ⭐ Selected
- **Option B: Custom MAVLink Implementation**
- **Option C: C mavlink Library via FFI**

### Option Analysis

**Option A: rust-mavlink Crate**

- **Pros**:
  - Official Rust implementation
  - Code generation from XML (supports all MAVLink messages)
  - no_std compatible
  - Active maintenance and community support
  - Zero-copy parsing where possible
- **Cons**:
  - Large dependency (generates code for all messages, even unused ones)
  - Some RAM overhead (\~8 KB for buffers)
- **Estimated Overhead**: \~8 KB RAM, \~50 KB Flash

**Option B: Custom MAVLink Implementation**

- **Pros**:
  - Minimal code size (only implement needed messages)
  - Full control over parsing and encoding
  - Can optimize for specific use cases
- **Cons**:
  - High development effort (reimplement parser, CRC, encoding)
  - Maintenance burden (update when MAVLink spec changes)
  - Potential bugs in parser (MAVLink is complex)
  - Reinventing the wheel
- **Estimated Overhead**: \~4 KB RAM, \~20 KB Flash (after implementation)

**Option C: C mavlink Library via FFI**

- **Pros**:
  - Reference implementation (used by ArduPilot/PX4)
  - Maximum compatibility
- **Cons**:
  - FFI overhead and complexity
  - C code in Rust project (unsafe, harder to maintain)
  - Requires C compiler in build chain
  - Not idiomatic Rust
- **Estimated Overhead**: \~6 KB RAM, \~40 KB Flash

## Rationale

rust-mavlink was chosen over custom implementation and C library for:

1. **Development Velocity**: Avoid reimplementing MAVLink protocol from scratch
2. **Correctness**: rust-mavlink is well-tested and maintained
3. **Ecosystem**: Official Rust implementation with active community
4. **Code Generation**: Automatically generates message types from XML

### Trade-offs Accepted

- **Code Size**: rust-mavlink generates code for all messages, even unused ones (\~50 KB Flash vs \~20 KB custom)
- **RAM Usage**: \~8 KB for buffers vs \~4 KB custom implementation

**Decision**: We accept the overhead for faster development and better maintainability.

## Consequences

### Positive

- **Faster Development**: No need to implement MAVLink parser/encoder
- **Correctness**: Proven implementation, less risk of protocol bugs
- **Maintainability**: Updates to MAVLink spec automatically via XML regeneration
- **Compatibility**: Full ArduPilot message set support out-of-the-box

### Negative

- **Code Size**: \~50 KB Flash for rust-mavlink (larger than minimal custom implementation)
- **RAM Usage**: \~8 KB for buffers (more than strictly necessary)
- **Dependency**: External crate (adds supply chain risk)

### Neutral

- **Message Set**: Using ArduPilot message set (common + ardupilotmega) for GCS compatibility

## Implementation Notes

### Message Handler Architecture

```rust
use mavlink::{self, Message, MavlinkVersion};

pub struct MavlinkRouter {
    param_handler: ParamHandler,
    mission_handler: MissionHandler,
    command_handler: CommandHandler,
    telemetry: TelemetryStreamer,
}

impl MavlinkRouter {
    pub async fn handle_message(&mut self, msg: mavlink::ardupilotmega::MavMessage) {
        use mavlink::ardupilotmega::MavMessage;

        match msg {
            // Parameter protocol
            MavMessage::PARAM_REQUEST_LIST(_) => {
                self.param_handler.send_all_params().await;
            }
            MavMessage::PARAM_SET(param) => {
                self.param_handler.set_param(&param).await;
            }

            // Mission protocol
            MavMessage::MISSION_REQUEST_LIST(_) => {
                self.mission_handler.send_mission_count().await;
            }
            MavMessage::MISSION_ITEM(item) => {
                self.mission_handler.store_waypoint(&item).await;
            }

            // Command protocol
            MavMessage::COMMAND_LONG(cmd) => {
                self.command_handler.execute_command(&cmd).await;
            }

            _ => {
                // Ignore unknown messages
            }
        }
    }
}
```

### Telemetry Streaming

```rust
pub struct TelemetryStreamer {
    rates: StreamRates,      // SR_* parameters
    last_send: Instant,
}

impl TelemetryStreamer {
    pub async fn update(&mut self) {
        // HEARTBEAT (1Hz)
        if self.should_send(StreamId::Heartbeat) {
            self.send_heartbeat().await;
        }

        // ATTITUDE (10Hz by default)
        if self.should_send(StreamId::Attitude) {
            self.send_attitude().await;
        }

        // GPS_RAW_INT (5Hz by default)
        if self.should_send(StreamId::Position) {
            self.send_gps().await;
        }
    }
}
```

### UART Integration

```rust
#[embassy_executor::task]
async fn mavlink_task(mut uart: UartTx, mut rx_uart: UartRx) {
    let mut router = MavlinkRouter::new();
    let mut buffer = [0u8; 280]; // MAVLink 2.0 max message size

    loop {
        // Receive MAVLink message
        if let Ok(msg) = read_mavlink_message(&mut rx_uart, &mut buffer).await {
            router.handle_message(msg).await;
        }

        // Send telemetry
        router.telemetry.update().await;
    }
}
```

### Message Rate Configuration

```rust
// ArduPilot SR_* parameters for stream rates
pub struct StreamRates {
    sr_extra1: u8,    // ATTITUDE, 10Hz default
    sr_position: u8,  // GPS_RAW_INT, 5Hz default
    sr_rc_chan: u8,   // RC_CHANNELS, 5Hz default
    sr_raw_sens: u8,  // IMU_SCALED, 5Hz default
}
```

## Platform Considerations

- **Pico W**: UART at 115200 baud (11.5 KB/s theoretical max)
- **Pico 2 W**: Same UART constraints
- **Cross-Platform**: rust-mavlink is platform-independent (uses no_std)

## Monitoring & Logging

- **Message Statistics**: Count sent/received messages per type
- **Buffer Usage**: Monitor MAVLink TX/RX buffer fill levels
- **Parse Errors**: Log CRC failures and malformed messages
- **Dropped Messages**: Warn if TX buffer full (telemetry rate too high)

## Open Questions

- [ ] Should we implement message signing for secure links? → Decision: Defer to Phase 2 (optional security feature)
- [ ] Can we reduce code size by excluding unused messages? → Method: Investigate conditional compilation in rust-mavlink
- [ ] Should we support USB CDC as alternative transport? → Next step: USB support as future enhancement

## External References

- rust-mavlink Crate: <https://crates.io/crates/mavlink>
- MAVLink Protocol: <https://mavlink.io/en/>
- MAVLink Message Definitions: <https://github.com/mavlink/mavlink>
- ArduPilot MAVLink: <https://ardupilot.org/dev/docs/mavlink-basics.html>

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
