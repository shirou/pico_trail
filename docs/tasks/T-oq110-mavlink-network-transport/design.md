# T-oq110 MAVLink Network Transport Implementation

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [plan.md](./plan.md)

## Overview

This task implements UDP network transport for MAVLink communication alongside the existing UART transport. The implementation uses trait-based abstraction following ArduPilot's proven `AP_HAL::UARTDriver` pattern, enabling concurrent operation of UART and UDP transports with independent failure isolation. WiFi configuration uses compile-time environment variables for security and simplicity. The expected outcome is production-grade wireless MAVLink communication compatible with industry-standard ground control stations (QGroundControl, Mission Planner).

## Success Metrics

- [ ] Command round-trip latency < 110ms (COMMAND_LONG → COMMAND_ACK via UDP)
- [ ] Memory usage ≤ 50 KB RAM for network stack and transport layer
- [ ] Packet loss < 1% on local WiFi (100 Mbps, 2.4 GHz)
- [ ] Zero UART regressions (existing UART functionality preserved)
- [ ] Works with QGroundControl/Mission Planner without GCS configuration changes

## Background and Current State

- Context: MAVLink communication currently operates exclusively over UART (serial). Adding UDP network transport enables wireless debugging, telemetry monitoring, and command execution over WiFi, improving development workflow and operational flexibility.
- Current behavior:
  - MAVLink parser: `src/communication/mavlink/parser.rs`
  - MAVLink writer: `src/communication/mavlink/writer.rs`
  - MAVLink task: `src/communication/mavlink/task.rs`
  - UART-specific code tightly coupled to MAVLink logic
- Pain points:
  - Cannot debug or monitor telemetry without physical UART connection
  - Adding new transport requires duplicating MAVLink protocol logic
  - No wireless communication capability for field operations
- Constraints:
  - Memory budget: ≤50 KB RAM, ≤100 KB Flash for network transport
  - Latency requirement: ≤100ms additional latency over UART baseline
  - Platform: RP2040/RP2350 with CYW43439 WiFi (no secure enclave)
- Related ADRs:
  - [ADR-ckv8z-transport-abstraction](../../adr/ADR-ckv8z-transport-abstraction.md)
  - [ADR-aul2v-udp-primary-transport](../../adr/ADR-aul2v-udp-primary-transport.md)
  - [ADR-dxdj0-wifi-config-strategy](../../adr/ADR-dxdj0-wifi-config-strategy.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                     MAVLink Application                      │
│                  (Protocol Logic, Handlers)                  │
└──────────────────────────┬──────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    MAVLink Router                            │
│  - Message routing to all transports                         │
│  - Message deduplication (sequence numbers)                  │
│  - Transport health monitoring                               │
└──────┬───────────────────────────────────────────────┬──────┘
       │                                               │
       │ MavlinkTransport trait                        │ MavlinkTransport trait
       │                                               │
       ▼                                               ▼
┌─────────────────┐                         ┌─────────────────┐
│  UartTransport  │                         │  UdpTransport   │
│  - UART RX/TX   │                         │  - UDP socket   │
│  - Buffering    │                         │  - GCS tracking │
└────────┬────────┘                         └────────┬────────┘
         │                                           │
         ▼                                           ▼
┌─────────────────┐                         ┌─────────────────┐
│  UART Hardware  │                         │  WiFi Stack     │
│  (embassy-rp)   │                         │  (embassy-net)  │
└─────────────────┘                         └────────┬────────┘
                                                     │
                                                     ▼
                                            ┌─────────────────┐
                                            │  CYW43439 WiFi  │
                                            │  (2.4 GHz)      │
                                            └─────────────────┘
```

### Components

#### 1. Transport Trait (`src/communication/mavlink/transport/mod.rs`)

```rust
/// Transport abstraction for MAVLink communication
pub trait MavlinkTransport {
    /// Returns number of bytes available to read
    async fn available(&self) -> usize;

    /// Read bytes from transport into buffer
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;

    /// Write bytes from buffer to transport
    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError>;

    /// Flush any buffered writes
    async fn flush(&mut self) -> Result<(), TransportError>;
}

#[derive(Debug, Clone, Copy)]
pub enum TransportError {
    IoError,
    Timeout,
    Disconnected,
}
```

**Responsibilities:**

- Define common interface for all MAVLink transports
- Provide async methods compatible with Embassy executor
- Enable zero-cost abstraction via trait monomorphization

#### 2. UART Transport (`src/communication/mavlink/transport/uart.rs`)

```rust
pub struct UartTransport {
    tx: UartTx<'static, UART0, Async>,
    rx: UartRx<'static, UART0, Async>,
}

impl MavlinkTransport for UartTransport {
    // Implementation wraps existing UART operations
}
```

**Responsibilities:**

- Wrap existing UART RX/TX operations
- Maintain backward compatibility with current UART functionality
- Provide buffering if needed

#### 3. UDP Transport (`src/communication/mavlink/transport/udp.rs`)

```rust
pub struct UdpTransport {
    socket: UdpSocket<'static>,
    bind_port: u16,  // Default 14550
    gcs_endpoints: heapless::Vec<SocketAddr, 4>,
    last_activity: heapless::FnvIndexMap<SocketAddr, u32, 4>,
}

impl MavlinkTransport for UdpTransport {
    // Implementation handles UDP socket operations and GCS tracking
}
```

**Responsibilities:**

- Bind UDP socket to port 14550 (MAVLink standard)
- Track active GCS endpoints via heartbeat messages
- Broadcast outbound messages to all known GCS
- Remove inactive GCS after 10-second timeout
- Handle packet loss gracefully (MAVLink protocol handles reliability)

#### 4. MAVLink Router (`src/communication/mavlink/router.rs`)

```rust
pub struct MavlinkRouter {
    transports: heapless::Vec<Box<dyn MavlinkTransport>, 4>,
    message_dedup: MessageDeduplicator,
    stats: [TransportStats; 4],
}

impl MavlinkRouter {
    pub async fn receive_message(&mut self) -> Result<(MavMessage, usize), RouterError>;
    pub async fn send_message(&mut self, msg: &MavMessage) -> Result<(), RouterError>;
    pub fn add_transport(&mut self, transport: Box<dyn MavlinkTransport>);
}
```

**Responsibilities:**

- Manage multiple transports (UART, UDP, future TCP)
- Route incoming messages from any transport to application layer
- Broadcast outgoing messages to all active transports
- Deduplicate messages based on sequence numbers
- Track per-transport statistics (messages sent/received, errors)
- Isolate transport failures (one transport error doesn't affect others)

#### 5. WiFi Configuration (`src/platform/rp2350/network.rs`)

```rust
pub struct WifiConfig {
    pub ssid: String<32>,
    pub password: String<63>,
    pub use_dhcp: bool,
    pub static_ip: Option<Ipv4Address>,
    pub netmask: Option<Ipv4Address>,
    pub gateway: Option<Ipv4Address>,
}

impl WifiConfig {
    pub fn from_params(params: &WifiParams) -> Self {
        Self {
            ssid: params.ssid.get().clone(),
            password: params.password.get().clone(),
            use_dhcp: params.use_dhcp.get(),
            static_ip: if params.use_dhcp.get() {
                None
            } else {
                Some(params.static_ip.get())
            },
            netmask: if params.use_dhcp.get() {
                None
            } else {
                Some(params.netmask.get())
            },
            gateway: if params.use_dhcp.get() {
                None
            } else {
                Some(params.gateway.get())
            },
        }
    }
}
```

**Responsibilities:**

- Load WiFi credentials from runtime parameters (Flash storage)
- Provide IP configuration (DHCP or static)
- Handle empty SSID (skip WiFi initialization, UART-only mode)
- Convert parameter types to WiFi driver types

#### 6. Parameter Storage (`src/parameters/storage.rs`)

```rust
pub struct ParameterStore {
    parameters: heapless::FnvIndexMap<&'static str, ParamValue, 64>,
    dirty: bool,  // Needs Flash write
}

pub struct WifiParams {
    pub ssid: ParamString<32>,      // NET_SSID
    pub password: ParamString<63>,  // NET_PASS (hidden)
    pub use_dhcp: ParamBool,        // NET_DHCP
    pub static_ip: ParamIpv4,       // NET_IP
    pub netmask: ParamIpv4,         // NET_NETMASK
    pub gateway: ParamIpv4,         // NET_GATEWAY
}

impl ParameterStore {
    pub fn load_from_flash() -> Self {
        // Read parameter block from Flash
        // Deserialize into parameter map
    }

    pub fn save_to_flash(&mut self) -> Result<(), FlashError> {
        // Serialize parameter map
        // Write to Flash parameter block
        self.dirty = false;
        Ok(())
    }

    pub fn get(&self, name: &str) -> Option<&ParamValue> {
        self.parameters.get(name)
    }

    pub fn set(&mut self, name: &str, value: ParamValue) -> Result<(), ParamError> {
        if let Some(param) = self.parameters.get_mut(name) {
            *param = value;
            self.dirty = true;
            Ok(())
        } else {
            Err(ParamError::NotFound)
        }
    }

    pub fn is_hidden(&self, name: &str) -> bool {
        matches!(name, "NET_PASS")  // Password hidden from readout
    }
}
```

**Responsibilities:**

- Store parameters in Flash-backed storage
- Load parameters from Flash on boot
- Save parameters to Flash on modification (PARAM_SET)
- Provide parameter access to application code
- Hide sensitive parameters (NET_PASS) from MAVLink PARAM_REQUEST_READ
- Support parameter types: String, Bool, Int, Float, Ipv4

#### 7. TCP Transport (Future Enhancement)

**Note**: TCP transport is deferred to future implementation but designed into the abstraction layer.

```rust
pub struct TcpTransport {
    listener: TcpSocket<'static>,
    connections: heapless::Vec<TcpSocket<'static>, 4>,
    bind_port: u16,  // Default 5760 (ArduPilot convention)
    last_activity: heapless::FnvIndexMap<SocketAddr, u32, 4>,
}

impl MavlinkTransport for TcpTransport {
    // Implementation handles TCP socket operations and connection management
}
```

**Responsibilities:**

- Bind TCP listener socket to port 5760 (ArduPilot convention for TCP)
- Accept incoming connections from GCS (up to 4 concurrent)
- Maintain per-connection state (connected/disconnected)
- Send messages to all connected GCS
- Close inactive connections after timeout
- Handle TCP-specific flow control and buffering

**Key Differences from UDP:**

- **Connection-Oriented**: Requires accept/connect handshake (\~50-100ms overhead)
- **Guaranteed Delivery**: TCP handles retransmission at transport layer
- **Ordered Packets**: TCP ensures in-order delivery
- **Higher Memory**: \~5 KB additional RAM per connection (40 KB total for 4 GCS)
- **Higher Latency**: \~50-100ms vs UDP's \~20-50ms due to ACKs and flow control
- **Connection State**: Can detect GCS disconnect immediately (TCP FIN/RST)

**When to Use TCP (Future Criteria):**

- Log file downloads (large sequential transfers)
- Firmware updates (reliability critical)
- High-latency networks (TCP retransmission beneficial)
- Environments where UDP blocked by firewall

**Implementation Considerations:**

- Use `embassy-net::TcpSocket` for async TCP operations
- Track connections in `heapless::Vec<TcpSocket, 4>` (max 4 concurrent GCS)
- Implement accept loop in separate Embassy task
- Broadcast messages to all active connections (similar to UDP)
- Close connections on write timeout or error
- Memory budget: \~40 KB RAM for 4 TCP connections vs UDP's \~35 KB

**Integration with Router:**

TCP transport would implement the same `MavlinkTransport` trait, enabling seamless integration:

```rust
// Future usage
let tcp_transport = TcpTransport::new(stack, 5760).await;
router.add_transport(Box::new(tcp_transport));

// Router automatically broadcasts to UART, UDP, and TCP
router.send_message(&heartbeat).await?;
```

### Data Flow

#### Outbound Message Flow (Application → GCS)

1. Application creates MAVLink message (e.g., `HEARTBEAT`)
2. Application calls `router.send_message(&msg)`
3. Router encodes message to MAVLink wire format
4. Router iterates all registered transports
5. For each transport:
   - Call `transport.write(&encoded_bytes)`
   - If UART: Write to UART TX buffer
   - If UDP: Broadcast to all tracked GCS endpoints
6. Router logs transport-specific errors but continues to other transports
7. Router updates per-transport statistics

#### Inbound Message Flow (GCS → Application)

1. Router polls all transports concurrently using `embassy_futures::select::select_array`
2. First transport with available data returns bytes
3. Router calls `transport.read(&mut buffer)` to retrieve bytes
4. If UDP: Track sender endpoint for future broadcasts
5. Router parses MAVLink message from bytes
6. Router checks message sequence number for duplicates
7. If not duplicate: Forward message to application layer
8. Router updates per-transport statistics

#### WiFi Connection Flow

1. Application loads parameters from Flash at startup
2. Read WiFi configuration from parameters (`WifiConfig::from_params()`)
3. If SSID empty: Skip WiFi initialization, UART-only mode
4. Initialize CYW43439 WiFi driver
5. Call `control.join_wpa2(ssid, password)` with 30-second timeout
6. If connection fails: Retry with exponential backoff (1s, 2s, 4s, 8s, 16s)
7. After 5 failures: Disable WiFi, continue UART-only
8. If DHCP: Start DHCP client and wait for IP assignment (30s timeout)
9. If static IP: Configure static IP address, netmask, gateway
10. Create UDP socket and bind to port 14550
11. Register UDP transport with MAVLink router
12. Send initial `HEARTBEAT` message to announce presence

### Data Models and Types

#### Transport Error

```rust
#[derive(Debug, Clone, Copy)]
pub enum TransportError {
    IoError,        // Generic I/O error (UART error, socket error)
    Timeout,        // Operation timed out
    Disconnected,   // Transport disconnected (WiFi dropped, UART unplugged)
}
```

#### Router Error

```rust
#[derive(Debug, Clone, Copy)]
pub enum RouterError {
    TransportError(TransportError),
    ParseError,              // MAVLink parse failure
    NoTransportsAvailable,   // No transports registered
    BufferTooSmall,          // Provided buffer insufficient
}
```

#### Transport Statistics

```rust
pub struct TransportStats {
    pub transport_id: usize,
    pub messages_sent: u32,
    pub messages_received: u32,
    pub errors: u32,
    pub last_activity_ms: u32,
}
```

#### GCS Endpoint Tracking (UDP-specific)

```rust
// In UdpTransport
gcs_endpoints: heapless::Vec<SocketAddr, 4>  // Max 4 concurrent GCS
last_activity: heapless::FnvIndexMap<SocketAddr, u32, 4>  // Endpoint → timestamp
```

### Error Handling

- **Transport Errors**: Isolated per transport. If UDP fails, UART continues operation.
- **Router Error Logging**: Use `defmt::warn!` for transport errors, `defmt::error!` for critical router failures.
- **WiFi Connection Failure**: Retry WiFi connection with exponential backoff (1s, 2s, 4s, 8s, max 30s).
- **UDP Send Failure**: Log error but continue to next GCS endpoint (best-effort delivery).
- **UART Error**: Log error and attempt to recover UART peripheral.
- **Message Parse Error**: Log error and discard malformed message (don't crash).
- **No English error messages** for embedded defmt logging (compact format preferred).

### Security Considerations

- **WiFi Password Storage**: Stored in firmware binary (Flash). Extracting firmware reveals password.
  - **Mitigation**: Document risk, recommend strong network password, physical device security.
  - **Future Enhancement**: RP2350 Flash encryption (not in Phase 1).
- **MAVLink Authentication**: Not implemented (not part of MAVLink v2.0 base spec).
  - **Risk**: Unauthorized GCS can send commands if on same network.
  - **Mitigation**: Use WPA2-secured WiFi network, trusted network only.
- **Credential Logging**: Never log WiFi password via `defmt`. Only log SSID and connection status.
- **GCS Endpoint Trust**: Any GCS sending valid MAVLink messages is trusted.
  - **Risk**: Rogue GCS can issue commands.
  - **Mitigation**: Operate on trusted network only.

### Performance Considerations

- **Hot Path**: Message routing (`send_message`, `receive_message`) is called frequently.
  - **Optimization**: Use `heapless` collections (no heap allocation).
  - **Optimization**: Minimize copies (pass by reference where possible).
- **Async Concurrency**: Use Embassy's `select_array` to poll all transports concurrently.
- **Memory Budget**:
  - CYW43439 WiFi driver: \~20 KB RAM
  - embassy-net stack: \~15 KB RAM
  - UDP socket buffers (4 KB RX, 4 KB TX): \~8 KB RAM
  - Transport overhead (vtables, router state): \~2 KB RAM
  - **Total**: \~45 KB RAM (within 50 KB budget)
- **Latency Budget**:
  - WiFi PHY: \~10-20ms
  - UDP stack processing: \~5-10ms
  - MAVLink encode/decode: \~1-2ms
  - Router overhead: <1ms
  - **Total additional latency**: \~20-33ms (within 100ms budget)

### Platform Considerations

#### RP2040 (Pico W)

- 264 KB RAM total
- Network stack overhead: \~45 KB RAM (\~17% of total RAM)
- CYW43439 WiFi: 802.11n, 2.4 GHz only
- No hardware crypto acceleration
- No Flash encryption

#### RP2350 (Pico 2 W)

- 520 KB RAM total
- Network stack overhead: \~45 KB RAM (\~8.7% of total RAM)
- CYW43439 WiFi: Same as RP2040
- Optional Flash encryption (not implemented in Phase 1)
- More headroom for future enhancements

#### Cross-Platform

- Embassy framework provides unified async runtime for both platforms
- WiFi driver (cyw43) same for both platforms
- Network stack (embassy-net) same for both platforms
- No platform-specific code needed for transport abstraction

## Alternatives Considered

### 1. Enum-Based Transport Selection

```rust
pub enum Transport {
    Uart(UartTransport),
    Udp(UdpTransport),
}

impl Transport {
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        match self {
            Transport::Uart(t) => t.read(buf).await,
            Transport::Udp(t) => t.read(buf).await,
        }
    }
}
```

- Pros: No trait objects (stack-only), simple, fast dispatch
- Cons: Not extensible (can't add transports from external crates), code duplication

### 2. Function Pointers (C-style)

```rust
pub struct Transport {
    read_fn: fn(&mut [u8]) -> Result<usize>,
    write_fn: fn(&[u8]) -> Result<usize>,
    context: *mut c_void,
}
```

- Pros: Zero overhead, simple memory layout
- Cons: Not async-compatible, unsafe (raw pointers), no type safety

### 3. TCP as Primary Transport

- Pros: Guaranteed delivery, connection state, ordered packets
- Cons: Higher latency (\~50-100ms), more memory (\~40 KB), complex multiple GCS

### Decision Rationale

**Trait-based abstraction (Option 1 from alternatives)** was chosen because:

1. **Idiomatic Rust**: Traits are the standard abstraction mechanism
2. **Type Safety**: Compile-time verification of transport implementations
3. **Async Native**: Trait methods can be async (required for Embassy)
4. **Proven Pattern**: ArduPilot's `AP_HAL::UARTDriver` demonstrates viability
5. **Extensibility**: External crates can implement transport trait

**UDP as primary transport** was chosen because:

1. **Industry Standard**: QGroundControl/Mission Planner default to UDP 14550
2. **Lower Latency**: \~20-50ms vs TCP's \~50-100ms
3. **Less Memory**: \~35 KB vs TCP's \~40 KB
4. **MAVLink Reliability**: Protocol already handles retransmission at application layer

## Migration and Compatibility

- Backward Compatibility: Existing UART functionality fully preserved. No breaking changes to MAVLink message handling.
- Forward Compatibility: Transport abstraction designed for future extensions (TCP, USB CDC, Bluetooth).
- Rollout Plan:
  - Phase 1: Transport abstraction + UDP (UART functionality unchanged)
  - Phase 2: Testing and optimization
  - Phase 3: Future enhancements (TCP, runtime WiFi config) if needed
- Deprecation Plan: No deprecations. UART transport remains primary for wired debugging.

## Testing Strategy

### Unit Tests

- **Transport Trait**: Mock transport implementation for testing router logic
  - Test message routing to multiple transports
  - Test error isolation (one transport fails, others continue)
  - Test message deduplication
- **WiFi Config**: Test `WifiConfig::from_env()` with various environment variable combinations
  - Test DHCP mode
  - Test static IP mode
  - Test missing credentials (compile error expected)
- **UDP Endpoint Tracking**: Test GCS endpoint discovery and timeout
  - Test adding new GCS
  - Test inactive GCS removal after 10 seconds
  - Test max 4 GCS limit

### Integration Tests

Not applicable for embedded target (no std environment). Integration testing performed via:

- **Hardware-in-the-Loop**: Test with actual Pico 2 W hardware, QGroundControl, WiFi network
- **Manual Verification**: Connect QGroundControl via UDP, verify bidirectional communication

### External API Parsing (if applicable)

Not applicable (no external API calls).

### Performance & Benchmarks (if applicable)

- **Latency Measurement**: Measure command round-trip time (COMMAND_LONG → COMMAND_ACK)
  - Target: < 110ms
  - Method: Timestamp at send, timestamp at ACK receive, calculate difference
- **Memory Profiling**: Measure RAM usage with network stack active
  - Target: ≤ 50 KB RAM
  - Method: `defmt::info!` of heap/stack usage, embassy-net statistics
- **Packet Loss**: Monitor UDP packet loss over 1-hour test
  - Target: < 1%
  - Method: MAVLink sequence number gap detection

## Documentation Impact

- Update `docs/mavlink.md`: Add section on UDP network transport usage
- Add example configuration in `docs/mavlink.md`: WiFi environment variable setup
- Update `README.md`: Mention network transport capability
- Update build script documentation: Document WIFI_SSID and WIFI_PASSWORD variables

## External References (optional)

- ArduPilot HAL UARTDriver: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/UARTDriver.h>
- MAVLink Protocol Specification: <https://mavlink.io/en/guide/serialization.html>
- Embassy Async Patterns: <https://embassy.dev/book/dev/runtime.html>
- embassy-net UDP Socket: <https://docs.embassy.dev/embassy-net/git/default/udp/index.html>

## Open Questions

- [ ] Should we implement UDP broadcast for GCS auto-discovery? → Next step: Start with unicast (current plan), evaluate user feedback, add broadcast if needed
- [ ] How to handle WiFi disconnection/reconnection during flight? → Method: Monitor WiFi state, clear GCS endpoints on disconnect, re-discover on reconnect
- [ ] Should we add transport prioritization (e.g., UART has priority over UDP)? → Next step: Start with equal priority, add prioritization if needed based on testing

## Appendix

### Diagrams

#### Message Flow Diagram

```text
Application Layer
      │
      ▼
┌────────────────────────────────────┐
│      MAVLink Router                │
│  ┌──────────────────────────────┐  │
│  │  send_message()              │  │
│  │  - Encode message            │  │
│  │  - Broadcast to transports   │  │
│  └──────────┬───────────────────┘  │
│             │                       │
│   ┌─────────┴─────────┐            │
│   ▼                   ▼            │
│ write()             write()        │
└───┬───────────────────┬────────────┘
    │                   │
    ▼                   ▼
UartTransport      UdpTransport
    │                   │
    │                   │ (broadcast to all GCS)
    ▼                   ▼
  UART0            GCS 1, GCS 2, GCS 3, GCS 4
```

### Examples

#### Basic Usage in Main Task

```rust
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize peripherals
    let p = embassy_rp::init(Default::default());

    // Initialize UART transport
    let uart_config = UartConfig::default();
    let uart = Uart::new(p.UART0, p.PIN_0, p.PIN_1, Irqs, p.DMA_CH0, p.DMA_CH1, uart_config);
    let (uart_tx, uart_rx) = uart.split();
    let uart_transport = UartTransport::new(uart_tx, uart_rx);

    // Initialize WiFi and network stack
    let wifi_config = WifiConfig::from_env();
    let (stack, control) = initialize_wifi(spawner, wifi_config).await;

    // Initialize UDP transport
    let udp_transport = UdpTransport::new(stack, 14550).await;

    // Create router and register transports
    let mut router = MavlinkRouter::new();
    router.add_transport(Box::new(uart_transport));
    router.add_transport(Box::new(udp_transport));

    // Main message loop
    loop {
        // Receive message from any transport
        let (msg, transport_id) = router.receive_message().await?;

        // Handle message
        handle_mavlink_message(&msg, transport_id).await;

        // Send response if needed
        let response = create_response(&msg);
        router.send_message(&response).await?;
    }
}
```

#### WiFi Configuration Example

```bash
# Set environment variables
export WIFI_SSID="MyNetwork"
export WIFI_PASSWORD="SecurePassword123"
export WIFI_DHCP=true

# Build for RP2350
./scripts/build-rp2350.sh --release

# Flash to Pico 2 W
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo
```

### Glossary

- **MAVLink**: Micro Air Vehicle Link, lightweight messaging protocol for drones/autopilots
- **GCS**: Ground Control Station (e.g., QGroundControl, Mission Planner)
- **UART**: Universal Asynchronous Receiver-Transmitter (serial communication)
- **UDP**: User Datagram Protocol (connectionless network protocol)
- **Embassy**: Async Rust executor for embedded systems
- **CYW43439**: WiFi chip on Raspberry Pi Pico W/2W
- **Trait Object**: Dynamic dispatch via `dyn Trait` (small runtime overhead)
- **DHCP**: Dynamic Host Configuration Protocol (automatic IP assignment)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
