# ADR-aul2v UDP as Primary Network Transport for MAVLink

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-ydttj-udp-network-transport](../requirements/FR-ydttj-udp-network-transport.md)
  - [NFR-eddfs-network-latency-limits](../requirements/NFR-eddfs-network-latency-limits.md)
  - [NFR-ukx3a-network-memory-overhead](../requirements/NFR-ukx3a-network-memory-overhead.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-oq110-mavlink-network-transport](../tasks/T-oq110-mavlink-network-transport/README.md)

## Context

Adding network transport to MAVLink requires choosing between TCP and UDP protocols. The choice significantly impacts latency, reliability model, implementation complexity, and resource usage.

### Problem

MAVLink currently operates over UART (serial) which provides:

- **Reliable delivery**: UART hardware handles framing
- **Low latency**: Direct hardware connection
- **Simple model**: Byte stream, no packet boundaries

Network transport introduces new considerations:

- **Latency**: TCP handshake/ACKs add \~50-100ms, UDP \~20-50ms
- **Reliability**: TCP guarantees delivery, UDP doesn't
- **Multiple GCS**: Need to support >1 ground station
- **Resource usage**: TCP requires per-connection state

### Prior Art

**ArduPilot**:

- Primary network transport: UDP port 14550
- TCP optional (port 5760 for specific use cases)
- UDP broadcast for GCS discovery
- MAVLink protocol layer handles reliability

**PX4**:

- UDP as standard for telemetry
- TCP for file transfers (logs)
- UDP multicast for multiple GCS

**Industry Standard**:

- QGroundControl default: UDP 14550
- Mission Planner default: UDP 14550
- MAVProxy default: UDP 14550

### Constraints

- **Memory**: Limited RAM (< 50 KB budget for network stack)
- **Latency**: Commands must respond < 110ms total
- **Multiple GCS**: Support 4 concurrent ground stations
- **MAVLink Protocol**: Already has application-layer reliability

## Success Metrics

- Command round-trip latency < 110ms (COMMAND_LONG → COMMAND_ACK)
- Support 4 concurrent GCS without performance degradation
- Memory usage < 40 KB RAM (UDP stack + buffers)
- Packet loss < 1% on local WiFi (100 Mbps)
- Works with QGroundControl and Mission Planner out-of-box

## Decision

**We will use UDP as the primary network transport for MAVLink communication, with TCP deferred as optional future enhancement.**

### Architecture

```rust
pub struct UdpTransport {
    socket: UdpSocket<'static>,
    bind_port: u16,  // Default 14550
    gcs_endpoints: heapless::Vec<SocketAddr, 4>,  // Track active GCS
    last_activity: heapless::FnvIndexMap<SocketAddr, u32, 4>,  // Timeout tracking
}

impl UdpTransport {
    pub async fn new(stack: &'static Stack<NetDriver<'static>>, port: u16) -> Self {
        let mut socket = UdpSocket::new(stack, &mut RX_META, &mut RX_BUFFER, &mut TX_META, &mut TX_BUFFER);
        socket.bind(port).await.unwrap();

        Self {
            socket,
            bind_port: port,
            gcs_endpoints: heapless::Vec::new(),
            last_activity: heapless::FnvIndexMap::new(),
        }
    }

    /// Track new GCS or update activity timestamp
    fn track_endpoint(&mut self, endpoint: SocketAddr) {
        let now_ms = embassy_time::Instant::now().as_millis() as u32;

        if !self.gcs_endpoints.contains(&endpoint) {
            if self.gcs_endpoints.len() < 4 {
                self.gcs_endpoints.push(endpoint).ok();
                defmt::info!("New GCS connected: {:?}", endpoint);
            }
        }

        self.last_activity.insert(endpoint, now_ms).ok();
    }

    /// Remove inactive GCS (no activity for 10 seconds)
    fn cleanup_inactive(&mut self) {
        let now_ms = embassy_time::Instant::now().as_millis() as u32;

        self.gcs_endpoints.retain(|endpoint| {
            if let Some(last) = self.last_activity.get(endpoint) {
                now_ms - *last < 10_000
            } else {
                false
            }
        });
    }
}

impl MavlinkTransport for UdpTransport {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        let (n, endpoint) = self.socket.recv_from(buf).await
            .map_err(|_| TransportError::IoError)?;

        self.track_endpoint(endpoint);
        Ok(n)
    }

    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        self.cleanup_inactive();

        // Broadcast to all known GCS
        for endpoint in &self.gcs_endpoints {
            if let Err(e) = self.socket.send_to(buf, *endpoint).await {
                defmt::warn!("Send to {:?} failed: {:?}", endpoint, e);
            }
        }

        Ok(buf.len())
    }
}
```

### Decision Drivers

1. **Industry Standard**: UDP 14550 is de facto standard for MAVLink
2. **Low Latency**: UDP \~20-50ms vs TCP \~50-100ms round-trip
3. **Multiple GCS**: UDP broadcast simpler than TCP multi-connection
4. **Memory Efficiency**: UDP \~35 KB vs TCP \~40 KB
5. **MAVLink Reliability**: Protocol already handles retransmission

### Considered Options

- **Option A: UDP Primary** ⭐ Selected
- **Option B: TCP Primary**
- **Option C: Both TCP and UDP**

### Option Analysis

**Option A: UDP Primary**

- **Pros**:
  - Industry standard (QGC/MP default)
  - Lower latency (\~20-50ms)
  - Less memory (\~35 KB)
  - Simpler multiple GCS (broadcast)
  - Connectionless (no handshake overhead)
- **Cons**:
  - Unreliable (packet loss possible)
  - No connection state (manual endpoint tracking)
- **Effort**: Low-Medium

**Option B: TCP Primary**

- **Pros**:
  - Guaranteed delivery
  - Connection state (know when GCS connects/disconnects)
  - Ordered packets
- **Cons**:
  - Higher latency (\~50-100ms handshake)
  - More memory (\~40 KB)
  - Complex multiple GCS (need multiple sockets or connection multiplexing)
  - Connection overhead
- **Effort**: Medium

**Option C: Both TCP and UDP**

- **Pros**:
  - Maximum flexibility
  - Use TCP for reliable streams (logs), UDP for telemetry
- **Cons**:
  - Most memory (\~45 KB)
  - Increased complexity
  - Configuration burden (which port for what?)
- **Effort**: High

## Rationale

**Option A (UDP primary) was chosen** because:

1. **Industry Standard**: All major GCS (QGroundControl, Mission Planner, MAVProxy) default to UDP 14550
2. **Low Latency**: UDP's connectionless nature provides best latency for real-time telemetry
3. **MAVLink Design**: MAVLink protocol already includes sequence numbers and can handle packet loss at application layer
4. **Memory Budget**: UDP requires \~5 KB less than TCP (important on Pico W)
5. **Proven**: ArduPilot and PX4 use UDP as primary successfully

### Trade-offs Accepted

- **Packet Loss**: Acceptable on local WiFi (\~0.1% typical)
- **No Connection State**: Must manually track GCS endpoints
- **No Ordering**: MAVLink sequence numbers handle out-of-order packets

**Reliability is handled by MAVLink protocol layer**, not transport:

- Sequence numbers detect lost packets
- Critical commands (ARM, mode change) acknowledged via COMMAND_ACK
- GCS retransmits if no ACK received
- Telemetry is fire-and-forget (old data not critical)

### Alternatives Rejected

- **TCP (Option B)**: Latency too high for real-time telemetry, memory overhead
- **Both (Option C)**: Unnecessary complexity, no clear use case for TCP

## Consequences

### Positive

- **Standard Compatibility**: Works with all major GCS out-of-box
- **Low Latency**: Real-time telemetry with minimal delay
- **Simple Implementation**: No connection management, handshake, or keepalive
- **Multiple GCS**: Easy broadcast to all active ground stations
- **Memory Efficient**: Minimal RAM footprint (\~35 KB)

### Negative

- **Packet Loss**: Possible on congested WiFi (mitigated by MAVLink protocol)
- **Manual Endpoint Tracking**: Must track GCS addresses and timeouts
- **No Connection Events**: Can't detect GCS disconnect immediately (must timeout)

### Neutral

- **Best Effort**: Fire-and-forget model acceptable for telemetry
- **Application Reliability**: MAVLink protocol handles reliability, not transport

## Implementation Notes

### Phase 1: Basic UDP Transport

1. Implement `UdpTransport` struct with `embassy-net::UdpSocket`
2. Bind to port 14550
3. Track single GCS endpoint
4. Implement `MavlinkTransport` trait

### Phase 2: Multiple GCS Support

1. Track up to 4 GCS endpoints in `heapless::Vec`
2. Implement timeout-based cleanup (10 seconds)
3. Broadcast outbound messages to all active GCS

### Phase 3: Optimization

1. Profile latency and packet loss
2. Add statistics (packets sent/received per GCS)
3. Optimize buffer sizes

### GCS Discovery

```rust
// GCS auto-discovery via heartbeat messages
impl UdpTransport {
    async fn handle_receive(&mut self) {
        let (n, endpoint) = self.socket.recv_from(&mut buf).await?;

        // Any message from new endpoint = add to GCS list
        self.track_endpoint(endpoint);

        // Parse MAVLink message...
    }
}
```

### Configuration

```rust
// Port configuration via parameter (optional)
const DEFAULT_UDP_PORT: u16 = 14550;  // MAVLink standard

// Future: configurable via parameter
// NET_UDP_PORT parameter (14550 default)
```

## Examples

### Basic Usage

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize WiFi and network stack
    let stack = initialize_network().await;

    // Create UDP transport
    let udp = UdpTransport::new(stack, 14550).await;

    // Register with MAVLink router
    let mut router = MavlinkRouter::new();
    router.add_transport(Box::new(udp));

    // Router handles message routing automatically
    loop {
        let (msg, transport_id) = router.receive_message().await?;
        handle_mavlink_message(msg).await;
    }
}
```

### QGroundControl Connection

1. **Pico WiFi**: Connect to network (DHCP or static IP)
2. **QGroundControl**: Add UDP connection
   - Type: UDP
   - Port: 14550
   - Listening: Yes (GCS waits for heartbeat)
3. **Pico sends HEARTBEAT**: QGC receives, learns Pico's IP
4. **QGC responds**: Pico learns QGC's IP/port
5. **Bidirectional communication**: Established

## Platform Considerations

- **RP2040 (Pico W)**: 264 KB RAM - UDP \~35 KB acceptable
- **RP2350 (Pico 2 W)**: 520 KB RAM - UDP \~35 KB negligible
- **CYW43439 WiFi**: 802.11n, \~1 Mbps typical (sufficient for MAVLink)
- **Embassy Network**: `embassy-net` provides async UDP socket

## Monitoring & Logging

```rust
pub struct UdpStats {
    packets_sent: u32,
    packets_received: u32,
    send_errors: u32,
    active_gcs_count: u8,
}

impl UdpTransport {
    pub fn get_stats(&self) -> UdpStats {
        UdpStats {
            packets_sent: self.packets_sent,
            packets_received: self.packets_received,
            send_errors: self.send_errors,
            active_gcs_count: self.gcs_endpoints.len() as u8,
        }
    }
}

// Log stats periodically
#[embassy_executor::task]
async fn stats_task(transport: &UdpTransport) {
    loop {
        Timer::after(Duration::from_secs(60)).await;

        let stats = transport.get_stats();
        defmt::info!("UDP stats: sent={}, recv={}, errors={}, gcs={}",
            stats.packets_sent,
            stats.packets_received,
            stats.send_errors,
            stats.active_gcs_count);
    }
}
```

## Open Questions

- [ ] Should we implement UDP broadcast for GCS auto-discovery? → Method: Start with unicast, add broadcast if needed (complexity vs benefit)
- [ ] How to handle WiFi disconnection (no route to GCS)? → Next step: Detect WiFi state, clear GCS endpoints on disconnect
- [ ] Should we add TCP support later for specific use cases (log downloads)? → Decision: Defer to future, evaluate based on user feedback

## External References

- MAVLink UDP Routing: <https://mavlink.io/en/guide/routing.html>
- ArduPilot Network Setup: <https://ardupilot.org/copter/docs/common-network.html>
- Embassy UDP Socket: <https://docs.embassy.dev/embassy-net/git/default/udp/index.html>
- QGroundControl UDP Setup: <https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/settings_comm.html>

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
