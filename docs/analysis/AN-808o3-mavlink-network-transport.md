# AN-808o3 MAVLink Network Transport

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-cp76d-ardupilot-analysis](../analysis/AN-cp76d-ardupilot-analysis.md)
- Related Requirements:
  - [FR-gpzpz-mavlink-protocol](../requirements/FR-gpzpz-mavlink-protocol.md)
  - [FR-ydttj-udp-network-transport](../requirements/FR-ydttj-udp-network-transport.md)
  - [FR-6jkia-transport-abstraction](../requirements/FR-6jkia-transport-abstraction.md)
  - [FR-dxvrs-wifi-configuration](../requirements/FR-dxvrs-wifi-configuration.md)
  - [FR-eutkf-concurrent-transports](../requirements/FR-eutkf-concurrent-transports.md)
  - [NFR-ukx3a-network-memory-overhead](../requirements/NFR-ukx3a-network-memory-overhead.md)
  - [NFR-eddfs-network-latency-limits](../requirements/NFR-eddfs-network-latency-limits.md)
  - [NFR-ypgpm-transport-independence](../requirements/NFR-ypgpm-transport-independence.md)
- Related ADRs:
  - [ADR-ggou4-mavlink-implementation](../adr/ADR-ggou4-mavlink-implementation.md)
  - [ADR-ckv8z-transport-abstraction](../adr/ADR-ckv8z-transport-abstraction.md)
  - [ADR-aul2v-udp-primary-transport](../adr/ADR-aul2v-udp-primary-transport.md)
  - [ADR-dxdj0-wifi-config-strategy](../adr/ADR-dxdj0-wifi-config-strategy.md)
- Related Tasks: N/A

## Executive Summary

This analysis explores adding TCP/UDP network transport to the existing UART-based MAVLink communication system. Network transport would provide production-grade wireless communication with ground control stations, enabling flexible deployment options alongside UART connectivity. The Raspberry Pi Pico W and Pico 2 W both include WiFi hardware (CYW43439), making network transport technically feasible. This analysis evaluates TCP vs UDP approaches, memory/performance implications, and integration strategies with the existing MAVLink implementation.

## Problem Space

### Current State

The MAVLink implementation (FR-gpzpz, ADR-ggou4) currently supports only UART transport:

- **Physical connection required**: UART requires USB-serial adapter or direct wiring
- **Single transport**: No runtime switching between transports
- **Debug workflow**: Requires physical connection during testing (cable management, device access)
- **GCS limitation**: One GCS per UART port
- **Flexibility**: Changing transport requires hardware rewiring

**Implementation details** (`docs/mavlink.md`):

- UART0 (GPIO 0/1) at 115200 baud
- Works with QGroundControl and Mission Planner
- \~5% bandwidth utilization at default telemetry rates

### Desired State

Enable wireless MAVLink communication for both production and development:

- **Wireless operation**: Connect GCS over WiFi without physical cables
- **Multi-transport**: Support both UART and network transports simultaneously or selectively
- **Flexible deployment**: Switch between transports without firmware changes or hardware rewiring
- **Multiple GCS**: Support multiple ground stations over network (QGC + Mission Planner)
- **Remote access**: Operate and monitor from any location without physical device access

### Gap Analysis

**Missing capabilities**:

1. **Network stack**: No TCP/UDP socket abstraction in current codebase
2. **WiFi configuration**: No WiFi setup, credential management, or connection handling
3. **Transport abstraction**: MAVLink tightly coupled to UART implementation
4. **Memory overhead**: Network stack requires additional RAM (\~20-40 KB)
5. **Concurrent transports**: No mechanism to handle multiple transports simultaneously

**Technical deltas**:

- Add Embassy network stack (embassy-net)
- Integrate CYW43439 WiFi driver
- Abstract MAVLink transport layer
- Add WiFi configuration (SSID, password, IP settings)
- Handle transport selection (UART, TCP, UDP)

## Stakeholder Analysis

| Stakeholder       | Interest/Need                                                       | Impact | Priority |
| ----------------- | ------------------------------------------------------------------- | ------ | -------- |
| Developers        | Wireless testing, flexible debugging                                | High   | P0       |
| GCS Operators     | Multiple GCS connections, remote monitoring                         | High   | P0       |
| Production Users  | Both UART and WiFi as production-grade transports, flexible options | High   | P0       |
| Embedded Platform | Memory/CPU overhead acceptable                                      | Medium | P1       |

## Research & Discovery

### User Feedback

From operational experience:

- UART requires physical access to device
- Cable management during operation is cumbersome
- Multiple GCS connections require mavproxy router (additional setup)
- WiFi-based operation is common in ArduPilot/PX4 deployments for both development and production

### Competitive Analysis

**ArduPilot**:

- Supports multiple transports: UART, TCP, UDP (MAVProxy)
- Common setup: `mavproxy.py --master=/dev/ttyUSB0 --out=udp:192.168.1.100:14550`
- TCP on port 5760, UDP on port 14550 (default)
- Uses lwIP network stack on embedded platforms

**PX4**:

- Supports UART, TCP, UDP transports
- UDP broadcast discovery for automatic GCS connection
- TCP for reliable streams (logs, parameters)
- Network transport configurable via parameters

**ESP32-based autopilots**:

- WiFi native (ESP32 WiFi built-in)
- TCP server on port 5760 for QGroundControl
- UDP broadcast for telemetry

### Technical Investigation

**Hardware capabilities**:

- **Pico W**: RP2040 + CYW43439 WiFi (802.11n, 2.4GHz only)
- **Pico 2 W**: RP2350 + CYW43439 WiFi (802.11n, 2.4GHz only)
- WiFi driver: `cyw43` crate (maintained by Embassy project)
- Network stack: `embassy-net` (async TCP/UDP stack)

**Memory analysis**:

| Component         | RAM Overhead | Flash Overhead |
| ----------------- | ------------ | -------------- |
| cyw43 WiFi driver | \~20 KB      | \~30 KB        |
| embassy-net stack | \~10-20 KB   | \~40 KB        |
| TCP sockets (2)   | \~8 KB       | \~5 KB         |
| UDP sockets (2)   | \~4 KB       | \~5 KB         |
| **Total (TCP)**   | **\~40 KB**  | **\~80 KB**    |
| **Total (UDP)**   | **\~35 KB**  | **\~75 KB**    |

**Platform availability**:

- **Pico W (RP2040)**: 264 KB RAM → \~15% overhead (TCP), \~13% overhead (UDP)
- **Pico 2 W (RP2350)**: 520 KB RAM → \~7.7% overhead (TCP), \~6.7% overhead (UDP)

**Performance considerations**:

- WiFi throughput: \~1 Mbps typical (MAVLink needs \~40 KB/min at default rates)
- TCP connection setup: \~100ms (adds latency vs UART)
- UDP broadcast: No connection setup (lower latency)
- CPU overhead: \~5-10% for network stack processing

**Embassy integration**:

Embassy provides async network stack:

```rust
use embassy_net::{Stack, StackResources, TcpSocket, UdpSocket};
use cyw43::{Control, NetDriver};

// Network task (runs alongside MAVLink task)
#[embassy_executor::task]
async fn net_task(stack: &'static Stack<NetDriver<'static>>) -> ! {
    stack.run().await
}

// MAVLink over TCP
#[embassy_executor::task]
async fn mavlink_tcp_task(stack: &'static Stack<NetDriver<'static>>) {
    let mut socket = TcpSocket::new(stack, &mut RX_BUF, &mut TX_BUF);
    socket.accept(5760).await.unwrap(); // Listen on port 5760

    loop {
        // Read MAVLink message from socket
        // Parse and route via existing MavlinkRouter
    }
}
```

### Data Analysis

**Bandwidth requirements** (from `docs/mavlink.md`):

- Default telemetry rates: \~40 KB/min (\~670 bytes/sec)
- WiFi capacity: \~1 Mbps (\~125 KB/sec)
- **Headroom**: 185x (WiFi bandwidth >> MAVLink needs)

**Latency requirements**:

- Command acknowledgment: < 10ms
- TCP round-trip: \~50-100ms (WiFi latency)
- UDP round-trip: \~20-50ms (WiFi latency)
- **Acceptable**: UDP meets latency, TCP marginal for commands

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The system shall support MAVLink communication over TCP/IP network transport → Will become FR-<id>
  - Rationale: Enable wireless operation and remote GCS connections for production and development
  - Acceptance Criteria:
    - TCP server listens on configurable port (default 5760)
    - QGroundControl can connect via TCP
    - All existing MAVLink messages (telemetry, parameters, commands, mission) work over TCP
    - Connection handling: accept, disconnect, reconnect

- [ ] **FR-DRAFT-2**: The system shall support MAVLink communication over UDP network transport → Will become FR-<id>
  - Rationale: Lower latency and connectionless operation for telemetry streaming
  - Acceptance Criteria:
    - UDP socket bound to configurable port (default 14550)
    - Bidirectional communication (GCS sends commands, autopilot sends telemetry)
    - QGroundControl can connect via UDP
    - Broadcast support for GCS discovery (optional)

- [ ] **FR-DRAFT-3**: The system shall allow runtime configuration of WiFi credentials and network settings → Will become FR-<id>
  - Rationale: Different networks during development (home, office, field)
  - Acceptance Criteria:
    - WiFi SSID and password configurable via parameters or build-time constants
    - IP address configuration: DHCP or static IP
    - WiFi connection status visible in telemetry (SYS_STATUS or custom message)

- [ ] **FR-DRAFT-4**: The system shall support concurrent MAVLink transport on UART and network → Will become FR-<id>
  - Rationale: Use UART for production flight logs while debugging over WiFi
  - Acceptance Criteria:
    - MAVLink messages broadcast to all active transports (UART + TCP/UDP)
    - Commands received on any transport processed identically
    - Independent transport failures (WiFi down ≠ UART failure)

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Network transport overhead shall not exceed 50 KB RAM and 100 KB Flash → Will become NFR-<id>
  - Category: Performance
  - Rationale: Maintain sufficient RAM for other autopilot functions
  - Target: ≤ 50 KB RAM (< 20% of Pico W, < 10% of Pico 2 W)

- [ ] **NFR-DRAFT-2**: Network transport shall not increase MAVLink message latency by more than 100ms → Will become NFR-<id>
  - Category: Performance
  - Rationale: Commands must remain responsive for safety
  - Target: Command ACK within 110ms (10ms baseline + 100ms network)

- [ ] **NFR-DRAFT-3**: WiFi connection failure shall not prevent UART transport operation → Will become NFR-<id>
  - Category: Reliability
  - Rationale: UART is primary transport for production flights
  - Target: UART continues operating if WiFi unavailable

## Design Considerations

### Technical Constraints

- **Memory budget**: Pico W has limited RAM (264 KB), network stack consumes \~40 KB
- **WiFi driver**: CYW43439 requires \~20 KB RAM for buffers
- **Single-core access**: CYW43439 SPI driver requires exclusive access (async Embassy handles coordination)
- **Platform-specific**: WiFi only available on Pico W/2W (not standard RP2040)
- **Embassy dependency**: Network stack requires Embassy executor (already used)

### Potential Approaches

1. **Option A: TCP Transport Only**
   - Pros:
     - Reliable, connection-oriented (guaranteed delivery)
     - Simpler connection management (accept, disconnect events)
     - Compatible with QGroundControl TCP mode
     - Easier debugging (connection state visible)
   - Cons:
     - Higher latency (\~50-100ms round-trip)
     - Connection overhead (SYN/ACK handshake)
     - Single GCS per connection (requires multiple sockets for multiple GCS)
     - More RAM (\~40 KB) for TCP buffers
   - Effort: Medium

2. **Option B: UDP Transport Only**
   - Pros:
     - Lower latency (\~20-50ms round-trip)
     - Connectionless (simpler state machine)
     - Less RAM (\~35 KB)
     - Broadcast support (GCS auto-discovery)
     - Multiple GCS trivial (send to multiple addresses)
   - Cons:
     - Unreliable (packet loss possible, though unlikely on local WiFi)
     - No connection state (harder to detect GCS disconnect)
     - Requires GCS to handle retransmission (MAVLink protocol layer)
   - Effort: Low-Medium

3. **Option C: Both TCP and UDP**
   - Pros:
     - Flexibility: Use TCP for reliable streams (parameters, mission), UDP for telemetry
     - Maximum GCS compatibility (some prefer TCP, others UDP)
     - Allows performance comparison and optimization
   - Cons:
     - Higher memory (\~45 KB RAM)
     - More complex configuration (which transport for which messages?)
     - Increased maintenance (two transport implementations)
   - Effort: High

4. **Option D: Transport Abstraction Layer + UDP Primary** ⭐ Recommended
   - Pros:
     - Clean abstraction: MAVLink agnostic to transport
     - Future transports easy to add (Bluetooth, USB CDC)
     - UDP as primary (simpler, lower latency)
     - UART and UDP coexist cleanly
     - Proven approach (ArduPilot uses similar architecture)
   - Cons:
     - Abstraction overhead (small)
     - Initial effort higher (design trait/interface)
   - Effort: Medium-High

### ArduPilot Implementation Insights

ArduPilot provides a proven reference architecture for transport abstraction:

**Transport Abstraction Layer**:

ArduPilot uses `AP_HAL::UARTDriver` as a unified interface for all transports (UART, TCP, UDP):

```cpp
// GCS_MAVLINK uses generic UART driver
AP_HAL::UARTDriver *_port;  // Works with any transport

// Transport-agnostic operations
void update_receive();  // Read from any transport
void update_send();     // Write to any transport
```

**Network Manager Integration**:

The `AP_Networking::Port` class implements transport selection:

```cpp
enum class NetworkPortType {
    NONE,
    UDP_CLIENT,
    UDP_SERVER,
    TCP_CLIENT,
    TCP_SERVER
};
```

Each network port registers with `AP_SerialManager`, making TCP/UDP ports appear as serial devices to MAVLink:

```cpp
class Port : public AP_SerialManager::RegisteredPort {
    // MAVLink treats this like UART
};
```

**Multi-Channel Routing**:

ArduPilot's `MAVLink_routing` class implements learning-based routing:

- **Route Learning**: Automatically discovers paths by observing incoming messages
- **System/Component ID**: Routes messages based on target system and component IDs
- **Broadcast Support**: Forwards messages with target=0 to all channels
- **Duplicate Prevention**: Each message sent once per channel
- **Private Channels**: Special handling for point-to-point protocols

**Key Design Principles**:

1. **Transport Transparency**: MAVLink code remains unaware of underlying transport
2. **Unified Interface**: All transports implement common driver methods (`txspace()`, `read()`, `write()`)
3. **Channel Independence**: Each transport channel maintains separate state
4. **Dynamic Routing**: Routes learned automatically, no static configuration
5. **Flow Control**: Check buffer space before sending (`txspace()`)

**Rust Translation Strategy**:

For pico_trail, we can adapt this architecture using Rust traits:

```rust
// Transport trait (analogous to AP_HAL::UARTDriver)
pub trait MavlinkTransport {
    async fn available(&self) -> usize;
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize>;
    async fn write(&mut self, buf: &[u8]) -> Result<usize>;
    async fn flush(&mut self) -> Result<()>;
}

// Implementations for different transports
impl MavlinkTransport for UartTransport { /* ... */ }
impl MavlinkTransport for UdpTransport { /* ... */ }
impl MavlinkTransport for TcpTransport { /* ... */ }

// Router manages multiple transports
pub struct MavlinkRouter {
    transports: Vec<Box<dyn MavlinkTransport>>,
    routes: RouteTable,  // System/Component ID → Transport mapping
}
```

This approach provides:

- Type-safe transport abstraction via Rust traits
- Zero-cost abstraction (trait dispatch optimized at compile time)
- Async-first design (Embassy executor integration)
- Multiple concurrent transports
- ArduPilot-compatible routing logic

### Architecture Impact

**New ADR required**:

- **ADR-<id> MAVLink Transport Abstraction**: Define transport trait and routing strategy
- Decision: TCP vs UDP vs both
- Decision: Concurrent transports or runtime-selectable
- Decision: WiFi configuration mechanism (compile-time vs runtime parameters)

**Affected modules**:

- `src/communication/mavlink/`: Add transport abstraction
- `src/platform/*/network.rs`: Platform-specific WiFi driver integration
- `src/core/parameters/`: Add WiFi configuration parameters

## Risk Assessment

| Risk                                     | Probability | Impact | Mitigation Strategy                                                                 |
| ---------------------------------------- | ----------- | ------ | ----------------------------------------------------------------------------------- |
| WiFi driver instability (CYW43439 bugs)  | Medium      | High   | Use proven embassy-net examples, thorough testing                                   |
| Memory exhaustion (network stack)        | Low         | High   | Profile memory usage, test on Pico W (worst case)                                   |
| Increased latency breaks safety commands | Low         | High   | Measure latency, fallback to UART if WiFi slow                                      |
| WiFi connection drops during operation   | Medium      | Medium | Implement transport redundancy, both UART and WiFi remain operational independently |
| Configuration complexity (SSID/password) | Medium      | Low    | Start with compile-time config, add runtime params later                            |

## Open Questions

- [ ] Should network transport be compile-time or runtime selectable? → Next step: Draft ADR with compile-time first (simpler), runtime later if needed
- [ ] TCP, UDP, or both? → Method: Prototype UDP first (simpler, lower latency), add TCP if reliability issues arise
- [ ] How to configure WiFi credentials? → Next step: Create FR for WiFi configuration (compile-time constants initially, parameter-based later)
- [ ] Should we support WiFi AP mode (Pico as access point) or STA mode (Pico connects to existing network)? → Decision: STA mode first (simpler infrastructure), AP mode future enhancement
- [ ] Do we need WiFi connection status telemetry? → Method: Add WiFi status to SYS_STATUS extended message or create custom MAVLink message

## Recommendations

### Immediate Actions

1. **Adopt Option D architecture**: Follow ArduPilot's proven transport abstraction pattern
2. **Design Rust trait interface**: Define `MavlinkTransport` trait analogous to `AP_HAL::UARTDriver`
3. **Prototype UDP transport**: Start with UDP (simpler, lower latency) as primary network transport
4. **Memory profiling**: Measure actual RAM usage with embassy-net + cyw43 on Pico W

### Next Steps

1. [ ] Create formal requirements: FR-<id> (UDP transport primary), FR-<id> (transport abstraction), FR-<id> (WiFi config), NFR-<id> (memory limits)
2. [ ] Draft ADR for: Transport abstraction architecture (trait-based, following ArduPilot pattern)
3. [ ] Draft ADR for: UDP as primary network transport (TCP optional future enhancement)
4. [ ] Draft ADR for: Routing strategy (simple broadcast vs ArduPilot-style learning)
5. [ ] Create task for: Implementation (transport trait + UDP + WiFi integration)

### Out of Scope

- **WiFi AP mode**: Pico as access point (STA mode sufficient for development)
- **Bluetooth transport**: Not needed for debugging (WiFi sufficient)
- **WiFi security**: WPA2 provided by CYW43439, no additional encryption needed
- **Multi-protocol transport selection**: Compile-time or parameter-based selection sufficient (no runtime auto-negotiation)

## Appendix

### References

- Embassy Network Documentation: <https://embassy.dev/book/dev/layer_by_layer.html#the-network-layer>
- cyw43 Driver: <https://github.com/embassy-rs/embassy/tree/main/cyw43>
- QGroundControl TCP Connection: <https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/settings_comm.html>
- MAVLink UDP Transport: <https://mavlink.io/en/guide/routing.html>
- ArduPilot MAVLink Routing: <https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html>

### Raw Data

**Embassy Pico W WiFi Example** (proof of concept):

```rust
use embassy_executor::Spawner;
use embassy_net::{Config, Stack, StackResources};
use cyw43_pio::PioSpi;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize WiFi driver
    let fw = include_bytes!("../firmware/43439A0.bin");
    let clm = include_bytes!("../firmware/43439A0_clm.bin");
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(/* ... */);

    let state = make_static!(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    // Spawn WiFi task
    spawner.spawn(wifi_task(runner)).unwrap();

    // Configure network stack
    let config = Config::dhcpv4(Default::default());
    let stack = &*make_static!(Stack::new(
        net_device,
        config,
        make_static!(StackResources::<3>::new()),
        seed
    ));

    // Spawn network task
    spawner.spawn(net_task(stack)).unwrap();

    // Connect to WiFi
    control.join_wpa2("SSID", "password").await;

    // Now stack is ready for TCP/UDP sockets
}
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
