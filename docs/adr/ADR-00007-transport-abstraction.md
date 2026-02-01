# ADR-00007 MAVLink Transport Abstraction via Rust Traits

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-00009-transport-abstraction](../requirements/FR-00009-transport-abstraction.md)
  - [FR-00008-concurrent-transports](../requirements/FR-00008-concurrent-transports.md)
  - [NFR-00008-transport-independence](../requirements/NFR-00008-transport-independence.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-00006-mavlink-network-transport](../tasks/T-00006-mavlink-network-transport/README.md)

## Context

The MAVLink implementation currently couples protocol logic directly to UART hardware. Adding network transports (UDP, TCP) requires abstracting the transport layer to enable:

- **Multiple Transports**: UART, UDP, TCP operating concurrently
- **Protocol Independence**: MAVLink code agnostic to underlying transport
- **Future Extensibility**: Easy addition of new transports (Bluetooth, USB CDC)
- **Testability**: Mock transports for unit testing
- **Concurrent Operation**: Multiple transports without code duplication

### Problem

Current implementation (`src/communication/mavlink/`) tightly couples MAVLink message handling to UART:

```rust
// Current: UART-specific
pub async fn mavlink_task(uart: UartTx<'static>) {
    // MAVLink logic mixed with UART operations
}
```

Adding UDP requires duplicating MAVLink logic or refactoring to abstract transport.

### Prior Art

**ArduPilot's `AP_HAL::UARTDriver`**:

- Unified interface for UART, TCP, UDP, USB
- `GCS_MAVLINK` class uses generic driver, agnostic to transport
- Proven in production across 1000+ aircraft types
- Key methods: `txspace()`, `read()`, `write()`

### Constraints

- **no_std Environment**: Embedded Rust, no heap allocation preferred
- **Async-First**: Embassy executor, all I/O async
- **Zero-Cost**: Trait dispatch should optimize to direct calls where possible
- **Memory Budget**: < 2 KB overhead for abstraction layer

## Success Metrics

- MAVLink message handling code has zero transport-specific conditionals
- Adding new transport requires < 100 lines of code (trait implementation only)
- Trait dispatch overhead < 1% CPU (measured via profiling)
- Memory overhead for abstraction < 2 KB RAM
- All existing UART functionality preserved

## Decision

**We will implement MAVLink transport abstraction using Rust traits with async methods, following ArduPilot's proven `UARTDriver` pattern.**

### Core Architecture

```rust
/// Transport abstraction for MAVLink communication
pub trait MavlinkTransport {
    /// Returns number of bytes available to read
    async fn available(&self) -> usize;

    /// Read bytes from transport into buffer
    /// Returns number of bytes read or error
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;

    /// Write bytes from buffer to transport
    /// Returns number of bytes written or error
    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError>;

    /// Flush any buffered writes (no-op for some transports)
    async fn flush(&mut self) -> Result<(), TransportError>;
}

#[derive(Debug, Clone, Copy)]
pub enum TransportError {
    IoError,
    Timeout,
    Disconnected,
}
```

### Transport Implementations

Each transport implements the trait:

```rust
// UART transport
pub struct UartTransport {
    tx: UartTx<'static, /* ... */>,
    rx: UartRx<'static, /* ... */>,
}

impl MavlinkTransport for UartTransport {
    async fn available(&self) -> usize {
        // Check UART RX buffer
    }

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        self.rx.read(buf).await.map_err(|_| TransportError::IoError)
    }

    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        self.tx.write_all(buf).await.map_err(|_| TransportError::IoError)?;
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), TransportError> {
        Ok(()) // UART flushes automatically
    }
}

// UDP transport
pub struct UdpTransport {
    socket: UdpSocket<'static>,
    gcs_endpoints: heapless::Vec<SocketAddr, 4>,
}

impl MavlinkTransport for UdpTransport {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        let (n, endpoint) = self.socket.recv_from(buf).await
            .map_err(|_| TransportError::IoError)?;
        self.track_endpoint(endpoint); // Learn GCS addresses
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
}
```

### Router Integration

MAVLink router manages multiple transports:

```rust
pub struct MavlinkRouter {
    transports: heapless::Vec<Box<dyn MavlinkTransport>, 4>,
}

impl MavlinkRouter {
    /// Receive message from any transport
    pub async fn receive_message(&mut self) -> Result<(MavMessage, usize), RouterError> {
        use embassy_futures::select::select_array;

        // Poll all transports concurrently
        let futures: [_; 4] = self.transports.iter_mut()
            .map(|t| async { t.read(&mut buffer).await })
            .collect();

        let (result, transport_idx) = select_array(futures).await;
        // Parse MAVLink message...
    }

    /// Send message to all transports
    pub async fn send_message(&mut self, msg: &MavMessage) -> Result<(), RouterError> {
        let encoded = self.encode_message(msg)?;

        for transport in &mut self.transports {
            if let Err(e) = transport.write(&encoded).await {
                defmt::warn!("Transport send failed: {:?}", e);
                // Continue to other transports
            }
        }
        Ok(())
    }
}
```

### TCP Transport (Future Enhancement)

TCP transport is deferred to Phase 2+ but designed into the abstraction:

```rust
// TCP transport (future implementation)
pub struct TcpTransport {
    listener: TcpSocket<'static>,
    connections: heapless::Vec<TcpSocket<'static>, 4>,
    bind_port: u16,  // Default 5760 (ArduPilot convention)
    last_activity: heapless::FnvIndexMap<SocketAddr, u32, 4>,
}

impl MavlinkTransport for TcpTransport {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        // Accept new connections in background task
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
            if let Err(e) = conn.write_all(buf).await {
                defmt::warn!("TCP connection write failed: {:?}", e);
                // Continue to other connections
            }
        }
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), TransportError> {
        for conn in &mut self.connections {
            conn.flush().await.ok();
        }
        Ok(())
    }
}

// Usage: same trait interface as UART and UDP
router.add_transport(Box::new(TcpTransport::new(stack, 5760).await));
```

**Key Differences from UDP:**

- **Connection-Oriented**: Requires accept/connect handshake (\~50-100ms)
- **Guaranteed Delivery**: TCP retransmission at transport layer
- **Higher Memory**: \~5 KB per connection (40 KB for 4 GCS vs UDP's 35 KB)
- **Higher Latency**: \~50-100ms vs UDP's \~20-50ms
- **Connection State**: Immediate disconnect detection (TCP FIN/RST)

**When to Use TCP (Future):**

- Log file downloads (large sequential transfers)
- Firmware updates (reliability critical)
- High-latency networks (TCP retransmission beneficial)
- Environments where UDP blocked by firewall

### Decision Drivers

1. **Proven Pattern**: ArduPilot uses similar abstraction successfully
2. **Type Safety**: Rust traits provide compile-time guarantees
3. **Async-First**: Embassy executor requires async trait methods
4. **Extensibility**: New transports require only trait implementation
5. **Zero-Cost**: Trait dispatch optimized away in monomorphic contexts

### Considered Options

- **Option A: Rust Trait with Async Methods** ⭐ Selected
- **Option B: Enum-Based Transport Selection**
- **Option C: Function Pointers (C-style)**

### Option Analysis

**Option A: Rust Trait with Async Methods**

- **Pros**:
  - Type-safe, compile-time verification
  - Async-first (Embassy compatible)
  - Extensible (external crates can implement)
  - ArduPilot-compatible pattern
  - Mock implementations for testing
- **Cons**:
  - Requires `dyn Trait` or monomorphization
  - Slightly complex lifetime management
- **Effort**: Medium

**Option B: Enum-Based Transport Selection**

```rust
pub enum Transport {
    Uart(UartTransport),
    Udp(UdpTransport),
    Tcp(TcpTransport),
}

impl Transport {
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize> {
        match self {
            Transport::Uart(t) => t.read(buf).await,
            Transport::Udp(t) => t.read(buf).await,
            // ...
        }
    }
}
```

- **Pros**:
  - No trait objects (stack-only)
  - Simple implementation
  - Fast dispatch (enum match)
- **Cons**:
  - Not extensible (can't add transports from external crates)
  - Code duplication in match arms
  - Less idiomatic Rust
- **Effort**: Low

**Option C: Function Pointers (C-style)**

```rust
pub struct Transport {
    read_fn: fn(&mut [u8]) -> Result<usize>,
    write_fn: fn(&[u8]) -> Result<usize>,
    context: *mut c_void,
}
```

- **Pros**:
  - Zero overhead (direct function calls)
  - Simple memory layout
- **Cons**:
  - Not async-compatible
  - Unsafe (raw pointers)
  - No type safety
  - Not idiomatic Rust
- **Effort**: Low

## Rationale

**Option A (Trait-based abstraction) was chosen** because:

1. **Idiomatic Rust**: Traits are the standard abstraction mechanism
2. **Type Safety**: Compile-time verification of transport implementations
3. **Async Native**: Trait methods can be async (required for Embassy)
4. **Proven Pattern**: ArduPilot's success demonstrates viability
5. **Extensibility**: External crates can implement transport trait

### Trade-offs Accepted

- **Trait objects** (`Box<dyn MavlinkTransport>`): Small overhead vs enum, but enables extensibility
- **Lifetime complexity**: Manageable with `'static` lifetimes for embedded contexts
- **Dynamic dispatch**: Acceptable for I/O operations (not hot path)

### Alternatives Rejected

- **Enum (Option B)**: Not extensible, code duplication
- **Function pointers (Option C)**: Unsafe, not async-compatible

## Consequences

### Positive

- **Clean Separation**: MAVLink logic completely independent of transport
- **Concurrent Transports**: UART and UDP coexist naturally
- **Testability**: Mock transports for unit tests
- **Future-Proof**: Easy to add Bluetooth, USB CDC, TCP
- **Type Safety**: Compiler enforces transport interface

### Negative

- **Trait Object Overhead**: `Box<dyn>` adds small memory/CPU cost
- **Learning Curve**: Developers must understand trait-based abstraction
- **Lifetime Complexity**: Async traits with lifetimes can be tricky

### Neutral

- **Memory**: \~2 KB overhead (vtable + trait objects) - within budget
- **Performance**: Trait dispatch optimized in most cases, negligible overhead

## Implementation Notes

### Phase 1: Define Trait and Refactor UART

1. Create `src/communication/mavlink/transport/mod.rs` with trait definition
2. Implement `UartTransport` struct wrapping existing UART code
3. Refactor MAVLink router to use `Box<dyn MavlinkTransport>`
4. Verify no functional changes (UART-only operation)

### Phase 2: Add UDP Transport

1. Implement `UdpTransport` in `src/communication/mavlink/transport/udp.rs`
2. Register UDP transport with router
3. Test concurrent UART + UDP operation

### Phase 3: Testing and Optimization

1. Add mock transport for unit tests
2. Profile trait dispatch overhead
3. Optimize hot paths if needed (monomorphization)

### Interface Conventions

```rust
// Transport registration
let mut router = MavlinkRouter::new();
router.add_transport(Box::new(UartTransport::new(uart_tx, uart_rx)));
router.add_transport(Box::new(UdpTransport::new(socket)));

// Error handling
match transport.read(&mut buf).await {
    Ok(n) => { /* process n bytes */ }
    Err(TransportError::IoError) => { /* log and continue */ }
    Err(TransportError::Timeout) => { /* retry */ }
    Err(TransportError::Disconnected) => { /* disable transport */ }
}
```

### Storage and State

- Transport instances stored in `heapless::Vec<Box<dyn MavlinkTransport>, 4>`
- Maximum 4 concurrent transports (UART, UDP, TCP, mock)
- Each transport maintains independent state (buffers, endpoints)

### Error Handling

- Transport errors isolated (don't propagate to other transports)
- Router logs errors via `defmt::warn!` but continues operation
- Failed transports can be removed and re-added (recovery)

## Examples

### Adding a New Transport

```rust
// Custom USB CDC transport
pub struct UsbCdcTransport {
    usb: embassy_usb::class::cdc_acm::CdcAcmClass<'static, Driver<'static, USB>>,
}

impl MavlinkTransport for UsbCdcTransport {
    async fn available(&self) -> usize {
        // Check USB CDC buffer
    }

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        self.usb.read_packet(buf).await
            .map_err(|_| TransportError::IoError)
    }

    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        self.usb.write_packet(buf).await
            .map_err(|_| TransportError::IoError)
    }

    async fn flush(&mut self) -> Result<(), TransportError> {
        self.usb.wait_connection().await;
        Ok(())
    }
}

// Register with router
router.add_transport(Box::new(UsbCdcTransport::new(usb)));
```

### Mock Transport for Testing

```rust
#[cfg(test)]
mod tests {
    use super::*;

    struct MockTransport {
        rx_data: Vec<u8>,
        tx_data: Vec<u8>,
    }

    impl MavlinkTransport for MockTransport {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
            let n = usize::min(buf.len(), self.rx_data.len());
            buf[..n].copy_from_slice(&self.rx_data[..n]);
            self.rx_data.drain(..n);
            Ok(n)
        }

        async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
            self.tx_data.extend_from_slice(buf);
            Ok(buf.len())
        }
    }

    #[test]
    fn test_mavlink_router() {
        let mock = MockTransport {
            rx_data: vec![/* MAVLink message bytes */],
            tx_data: vec![],
        };

        let mut router = MavlinkRouter::new();
        router.add_transport(Box::new(mock));

        // Test message routing...
    }
}
```

## Platform Considerations

- **RP2040 (Pico W)**: 264 KB RAM - trait objects acceptable overhead
- **RP2350 (Pico 2 W)**: 520 KB RAM - trait objects negligible
- **Cross-Platform**: Trait abstraction works identically on both platforms
- **Embassy Executor**: Async trait methods compatible with Embassy runtime

## Monitoring & Logging

```rust
// Transport health monitoring
pub struct TransportStats {
    transport_id: usize,
    messages_sent: u32,
    messages_received: u32,
    errors: u32,
    last_activity_ms: u32,
}

impl MavlinkRouter {
    pub fn get_stats(&self, transport_id: usize) -> &TransportStats {
        &self.stats[transport_id]
    }

    pub async fn send_message(&mut self, msg: &MavMessage) {
        for (id, transport) in self.transports.iter_mut().enumerate() {
            match transport.write(&encoded).await {
                Ok(_) => {
                    self.stats[id].messages_sent += 1;
                    self.stats[id].last_activity_ms = now_ms();
                }
                Err(e) => {
                    self.stats[id].errors += 1;
                    defmt::warn!("Transport {} error: {:?}", id, e);
                }
            }
        }
    }
}
```

## Open Questions

- [ ] Should we use `async-trait` crate or wait for stable async trait support? → Decision: Use `async-trait` for now, migrate when stable
- [ ] How to handle transport prioritization (send commands to UART first, then UDP)? → Method: Add priority field to transport, sort before sending
- [ ] Should transports be hot-pluggable (add/remove at runtime)? → Next step: Start with static registration, add hot-plug if needed

## External References

- ArduPilot HAL UARTDriver: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL/UARTDriver.h>
- Rust Async Trait: <https://rust-lang.github.io/async-book/07_workarounds/05_async_in_traits.html>
- async-trait crate: <https://crates.io/crates/async-trait>
- Embassy Async Patterns: <https://embassy.dev/book/dev/runtime.html>
