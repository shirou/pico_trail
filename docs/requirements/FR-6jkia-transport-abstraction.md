# FR-6jkia MAVLink Transport Abstraction

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-gpzpz-mavlink-protocol](FR-gpzpz-mavlink-protocol.md)
- Dependent Requirements:
  - [FR-ydttj-udp-network-transport](FR-ydttj-udp-network-transport.md)
  - [FR-eutkf-concurrent-transports](FR-eutkf-concurrent-transports.md)
  - [NFR-ukx3a-network-memory-overhead](NFR-ukx3a-network-memory-overhead.md)
  - [NFR-ypgpm-transport-independence](NFR-ypgpm-transport-independence.md)
- Related Tasks:
  - [T-oq110-mavlink-network-transport](../tasks/T-oq110-mavlink-network-transport/README.md)

## Requirement Statement

The system shall provide a transport-agnostic abstraction layer for MAVLink communication, enabling the MAVLink protocol implementation to operate uniformly across different physical transports (UART, UDP, TCP) without modification.

## Rationale

Transport abstraction provides critical architectural benefits:

- **Protocol Independence**: MAVLink code remains unaware of underlying transport
- **Future Extensibility**: Easy to add new transports (Bluetooth, USB CDC) without changing MAVLink logic
- **Code Reuse**: Single MAVLink implementation works with all transports
- **Testing**: Mock transports for unit testing
- **Concurrent Transports**: Multiple transports coexist cleanly through unified interface

ArduPilot's `AP_HAL::UARTDriver` demonstrates this pattern successfully, allowing MAVLink to work seamlessly across UART, TCP, UDP, and USB transports.

## User Story (if applicable)

As a developer, I want MAVLink protocol code to be independent of transport implementation, so that I can add new communication channels without modifying message handling logic and ensure consistent behavior across all transports.

## Acceptance Criteria

- [ ] MAVLink transport trait defined with standard interface (read, write, available, flush)
- [ ] UART transport implements the trait
- [ ] UDP transport implements the trait
- [ ] TCP transport implements the trait (optional, future)
- [ ] MAVLink router uses trait objects (`Box<dyn MavlinkTransport>`) to handle any transport uniformly
- [ ] All MAVLink operations (read message, send message, check buffer space) work identically across transports
- [ ] No transport-specific conditionals in MAVLink message handling code
- [ ] Mock transport can be created for unit testing
- [ ] Zero runtime overhead (trait dispatch optimized away at compile time for monomorphic cases)
- [ ] All transports support async operations (Embassy executor compatible)

## Technical Details (if applicable)

### Functional Requirement Details

**Transport Trait Definition**:

```rust
/// Abstraction for MAVLink transport layers
pub trait MavlinkTransport {
    /// Returns number of bytes available to read
    async fn available(&self) -> usize;

    /// Read bytes from transport into buffer
    /// Returns number of bytes read
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;

    /// Write bytes from buffer to transport
    /// Returns number of bytes written
    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError>;

    /// Flush any buffered writes (no-op for some transports)
    async fn flush(&mut self) -> Result<(), TransportError>;
}

#[derive(Debug)]
pub enum TransportError {
    IoError,
    Timeout,
    Disconnected,
}
```

**Transport Implementations**:

```rust
// UART transport
pub struct UartTransport {
    uart: UartTx<'static, /* ... */>,
    // ...
}

impl MavlinkTransport for UartTransport {
    async fn available(&self) -> usize { /* check UART RX buffer */ }
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        self.uart.read(buf).await.map_err(|_| TransportError::IoError)
    }
    // ...
}

// UDP transport
pub struct UdpTransport {
    socket: UdpSocket<'static>,
    gcs_endpoints: heapless::Vec<SocketAddr, 4>,
    // ...
}

impl MavlinkTransport for UdpTransport {
    async fn available(&self) -> usize { /* check UDP socket */ }
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        let (n, endpoint) = self.socket.recv_from(buf).await
            .map_err(|_| TransportError::IoError)?;
        self.track_endpoint(endpoint);
        Ok(n)
    }
    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        // Broadcast to all known GCS
        for endpoint in &self.gcs_endpoints {
            self.socket.send_to(buf, *endpoint).await
                .map_err(|_| TransportError::IoError)?;
        }
        Ok(buf.len())
    }
    // ...
}

// TCP transport (future enhancement)
pub struct TcpTransport {
    listener: TcpSocket<'static>,
    connections: heapless::Vec<TcpSocket<'static>, 4>,
    bind_port: u16,  // Default 5760
    // ...
}

impl MavlinkTransport for TcpTransport {
    async fn available(&self) -> usize { /* check TCP connections */ }
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        // Read from first available connection
        for conn in &mut self.connections {
            if let Ok(n) = conn.read(buf).await {
                return Ok(n);
            }
        }
        Err(TransportError::Disconnected)
    }
    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        // Broadcast to all active connections
        for conn in &mut self.connections {
            conn.write_all(buf).await.ok();
        }
        Ok(buf.len())
    }
    // ...
}
```

**Router Integration**:

```rust
pub struct MavlinkRouter {
    transports: heapless::Vec<Box<dyn MavlinkTransport>, 4>,
    // ...
}

impl MavlinkRouter {
    /// Receive message from any transport
    pub async fn receive_message(&mut self) -> Result<MavMessage, RouterError> {
        // Poll all transports, return first message
        // Uses Embassy select! for concurrent async operations
    }

    /// Send message to all transports
    pub async fn send_message(&mut self, msg: &MavMessage) -> Result<(), RouterError> {
        for transport in &mut self.transports {
            transport.write(&encode(msg)).await?;
        }
        Ok(())
    }
}
```

**Testing Support**:

```rust
// Mock transport for unit tests
pub struct MockTransport {
    rx_queue: Vec<u8>,
    tx_queue: Vec<u8>,
}

impl MavlinkTransport for MockTransport {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        // Return queued test data
    }
    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        self.tx_queue.extend_from_slice(buf);
        Ok(buf.len())
    }
    // ...
}
```

## Platform Considerations

### Unix

N/A - Platform agnostic (embedded system)

### Windows

N/A - Platform agnostic (embedded system)

### Cross-Platform

Transport abstraction must work on both Pico W (RP2040) and Pico 2 W (RP2350). Platform-specific transports (UART, WiFi) implement the same trait, ensuring MAVLink router code is platform-independent.

## Risks & Mitigation

| Risk                                     | Impact | Likelihood | Mitigation                                                | Validation                          |
| ---------------------------------------- | ------ | ---------- | --------------------------------------------------------- | ----------------------------------- |
| Trait object overhead (dynamic dispatch) | Low    | Medium     | Use monomorphization where possible, measure performance  | Benchmark trait vs concrete calls   |
| Lifetime complexity in trait definition  | Medium | Medium     | Start with simple trait, avoid self-referential lifetimes | Prototype early, adjust as needed   |
| Async trait issues (unstable feature)    | Medium | Low        | Use `async-trait` crate or manual Future implementation   | Test with Embassy executor          |
| Error handling inconsistency             | Medium | Low        | Define comprehensive TransportError enum                  | Test error conditions for each impl |

## Implementation Notes

Preferred approaches:

- Follow ArduPilot's `AP_HAL::UARTDriver` pattern
- Define simple trait with minimal methods (read, write, available, flush)
- Use `async fn` in trait (requires `async-trait` crate or nightly Rust)
- Store transports in `heapless::Vec` (avoid heap allocation)
- Use Embassy `select!` for concurrent transport polling

Known pitfalls:

- Avoid complex lifetimes in trait definition (keep `'static` or simple)
- Don't add transport-specific methods to trait (keep generic)
- Trait objects require `Box<dyn>` or static dispatch for `no_std`
- `async fn` in traits requires `async-trait` crate or unstable feature

Related code areas:

- `src/communication/mavlink/transport/mod.rs` - Trait definition
- `src/communication/mavlink/transport/uart.rs` - UART implementation
- `src/communication/mavlink/transport/udp.rs` - UDP implementation
- `src/communication/mavlink/transport/tcp.rs` - TCP implementation (future)
- `src/communication/mavlink/router.rs` - Multi-transport router

TCP Transport Considerations (Future):

- **Port**: 5760 (ArduPilot convention for TCP MAVLink)
- **Connection Management**: Accept up to 4 concurrent GCS connections
- **Memory**: \~5 KB per connection (40 KB total for 4 GCS)
- **Latency**: \~50-100ms (higher than UDP's \~20-50ms)
- **Use Cases**: Log downloads, firmware updates, high-latency networks, UDP-blocked environments
- **Implementation**: `embassy-net::TcpSocket` with accept loop in separate Embassy task

Suggested libraries:

- `async-trait` - Async methods in traits (if stable async trait not available)
- `heapless` - Fixed-size collections for no_std
- `embassy-executor` - Async runtime for concurrent transport operations

## External References

- ArduPilot HAL UART: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/UARTDriver.h>
- Rust Async Trait: <https://rust-lang.github.io/async-book/07_workarounds/05_async_in_traits.html>
- Embassy Async: <https://embassy.dev/book/dev/runtime.html>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
