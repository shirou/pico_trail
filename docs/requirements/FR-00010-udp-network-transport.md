# FR-00010 UDP Network Transport

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-00009-transport-abstraction](FR-00009-transport-abstraction.md)
  - [FR-00011-wifi-configuration](FR-00011-wifi-configuration.md)
  - [FR-00005-mavlink-protocol](FR-00005-mavlink-protocol.md)

- Dependent Requirements:
  - [FR-00008-concurrent-transports](FR-00008-concurrent-transports.md)
  - [NFR-00006-network-latency-limits](NFR-00006-network-latency-limits.md)
  - [NFR-00007-network-memory-overhead](NFR-00007-network-memory-overhead.md)

- Related Tasks:
  - [T-00006-mavlink-network-transport](../tasks/T-00006-mavlink-network-transport/README.md)

## Requirement Statement

The system shall support MAVLink communication over UDP/IP network transport, enabling wireless operation between the autopilot and ground control stations.

## Rationale

UDP transport provides production-grade wireless MAVLink connectivity with several advantages:

- **Wireless Operation**: Eliminates physical cable requirements for GCS communication
- **Low Latency**: UDP's connectionless nature provides \~20-50ms round-trip time vs TCP's \~50-100ms
- **Simplicity**: No connection management overhead (handshake, keepalive)
- **Multiple GCS**: Easy broadcast/multicast support for multiple ground stations
- **Proven Protocol**: MAVLink protocol layer handles reliability (retransmission at application level)

ArduPilot and PX4 both use UDP as primary network transport (default port 14550), making this the industry standard approach.

## User Story (if applicable)

As a GCS operator, I want to connect to the autopilot over WiFi using UDP, so that I can monitor telemetry, send commands, and manage missions without physical cable connections during both development and production operations.

## Acceptance Criteria

- [ ] UDP socket binds to configurable port (default 14550, MAVLink standard)
- [ ] Bidirectional communication: autopilot sends telemetry, receives commands
- [ ] QGroundControl can connect via UDP and perform all operations (telemetry, parameters, commands, mission)
- [ ] Mission Planner can connect via UDP (Windows compatibility)
- [ ] Support for multiple concurrent UDP clients (broadcast to all connected GCS)
- [ ] Telemetry streams at configured rates (10Hz attitude, 5Hz GPS by default)
- [ ] All MAVLink message types work over UDP (HEARTBEAT, ATTITUDE, GPS_RAW_INT, PARAM\_\_, MISSION\_\_, COMMAND\_\*)
- [ ] UDP transport coexists with UART transport (messages sent to both)
- [ ] Packet loss < 1% on local WiFi (100 Mbps network)
- [ ] No memory leaks or buffer overflows under sustained load

## Technical Details (if applicable)

### Functional Requirement Details

**UDP Socket Configuration**:

- Port: 14550 (default, configurable via parameter)
- Bind address: 0.0.0.0 (listen on all interfaces)
- Buffer size: 2048 bytes (accommodate MAVLink 2.0 max message size of 280 bytes + overhead)
- Non-blocking operation (async Embassy integration)

**Message Flow**:

```
Autopilot → UDP → GCS
  - Telemetry (HEARTBEAT, ATTITUDE, GPS_RAW_INT, SYS_STATUS)
  - Parameter values (PARAM_VALUE)
  - Mission items (MISSION_ITEM_INT)
  - Command acknowledgments (COMMAND_ACK)

GCS → UDP → Autopilot
  - Parameter requests (PARAM_REQUEST_LIST, PARAM_SET)
  - Mission upload (MISSION_COUNT, MISSION_ITEM_INT)
  - Commands (COMMAND_LONG: arm, disarm, mode change)
```

**Multiple GCS Handling**:

- Track list of active GCS endpoints (IP address + port)
- Add new endpoint when message received from unknown source
- Broadcast outbound messages to all known endpoints
- Remove inactive endpoints after timeout (no messages for 10 seconds)

**Integration with MAVLink Router**:

UDP transport implements `MavlinkTransport` trait:

```rust
impl MavlinkTransport for UdpTransport {
    async fn available(&self) -> usize { /* bytes available */ }
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize> { /* recv from socket */ }
    async fn write(&mut self, buf: &[u8]) -> Result<usize> { /* send to all GCS */ }
    async fn flush(&mut self) -> Result<()> { /* no-op for UDP */ }
}
```

**Error Handling**:

- Socket bind failure: Log error, fall back to UART only
- Send failure (buffer full): Drop message, increment dropped counter
- Receive error: Log and continue (don't crash)
- Malformed packet: Ignore, rely on MAVLink CRC validation

## Platform Considerations

### Unix

N/A - Platform agnostic (embedded system)

### Windows

N/A - Platform agnostic (embedded system)

### Cross-Platform

UDP transport must work on both Pico W (RP2040) and Pico 2 W (RP2350) using platform WiFi abstraction. Implementation uses Embassy `embassy-net` crate for cross-platform UDP socket support.

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                              | Validation                                      |
| ----------------------------------------- | ------ | ---------- | ------------------------------------------------------- | ----------------------------------------------- |
| Packet loss on congested WiFi             | Medium | Medium     | MAVLink protocol handles retransmission                 | Test with network packet loss simulation        |
| Memory exhaustion (GCS endpoint tracking) | Medium | Low        | Limit max concurrent GCS to 4, timeout inactive clients | Stress test with > 4 GCS connections            |
| UDP socket creation failure               | High   | Low        | Fall back to UART, log error clearly                    | Test with WiFi disabled                         |
| Broadcast storm (too many GCS)            | Low    | Low        | Rate limit outbound messages, max 4 GCS                 | Connect 10 GCS simultaneously, measure CPU load |

## Implementation Notes

Preferred approaches:

- Use `embassy-net::UdpSocket` for async UDP operations
- Store GCS endpoints in `heapless::Vec<SocketAddr, 4>` (no heap allocation)
- Implement timeout-based GCS cleanup (remove after 10s inactivity)
- Reuse single socket for all GCS communication (no per-client sockets)

Known pitfalls:

- Don't create new socket per message (expensive)
- UDP is connectionless: track endpoints manually
- embassy-net UDP recv is async: use `select!` for concurrent recv/send
- MAVLink messages may arrive fragmented: buffer reassembly required

Related code areas:

- `src/communication/mavlink/transport/udp.rs` - UDP transport implementation
- `src/communication/mavlink/router.rs` - Transport-agnostic message routing
- `src/platform/*/network.rs` - Platform WiFi driver integration

Suggested libraries:

- `embassy-net` - Async TCP/UDP stack
- `heapless::Vec` - Fixed-size collections (no_std)
- `embedded-nal-async` - Network abstraction layer (optional)

## External References

- MAVLink UDP Transport: <https://mavlink.io/en/guide/routing.html>
- QGroundControl UDP Settings: <https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/settings_comm.html>
- ArduPilot Network Setup: <https://ardupilot.org/copter/docs/common-network.html>
- Embassy Network: <https://embassy.dev/book/dev/layer_by_layer.html#the-network-layer>
