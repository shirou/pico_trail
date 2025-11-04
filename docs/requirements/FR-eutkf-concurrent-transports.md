# FR-eutkf Concurrent MAVLink Transports

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-6jkia-transport-abstraction](FR-6jkia-transport-abstraction.md)
  - [FR-gpzpz-mavlink-protocol](FR-gpzpz-mavlink-protocol.md)
  - [FR-ydttj-udp-network-transport](FR-ydttj-udp-network-transport.md)

- Dependent Requirements:
  - [NFR-ypgpm-transport-independence](NFR-ypgpm-transport-independence.md)

- Related Tasks:
  - [T-oq110-mavlink-network-transport](../tasks/T-oq110-mavlink-network-transport/README.md)

## Requirement Statement

The system shall support simultaneous operation of multiple MAVLink transports (UART and UDP), enabling messages to be sent and received concurrently across all active transports.

## Rationale

Concurrent transports provide critical operational flexibility:

- **Redundancy**: UART backup if WiFi fails, WiFi backup for physical access issues
- **Flexibility**: Use UART for production logging while debugging over WiFi
- **Multiple Stakeholders**: Different operators can connect via different transports simultaneously
- **Production Grade**: Both UART and WiFi are first-class transports, not debug vs production
- **ArduPilot Compatibility**: Follows industry standard multi-transport architecture

ArduPilot supports up to 6 concurrent MAVLink channels, demonstrating this is a proven production pattern.

## User Story (if applicable)

As a system operator, I want UART and WiFi transports to work simultaneously, so that I can maintain a wired backup connection while using wireless for convenience, and multiple operators can connect via different methods without conflicts.

## Acceptance Criteria

- [ ] UART and UDP transports operate concurrently without interference
- [ ] Messages received on any transport are processed identically
- [ ] Outbound messages broadcast to all active transports
- [ ] Each transport maintains independent state (buffers, counters)
- [ ] Transport failure doesn't affect other transports (WiFi down ≠ UART failure)
- [ ] GCS can connect via UART while another GCS connects via UDP
- [ ] Telemetry streams to all transports at configured rates
- [ ] Parameter changes via one transport visible on all transports
- [ ] Mission upload via one transport visible on all transports
- [ ] No message duplication (same message not processed twice from different transports)
- [ ] Performance: 10Hz attitude + 5Hz GPS on both transports simultaneously without dropped messages

## Technical Details (if applicable)

### Functional Requirement Details

**Router Architecture**:

```rust
pub struct MavlinkRouter {
    transports: heapless::Vec<Box<dyn MavlinkTransport>, 4>,
    message_dedup: MessageDeduplicator,  // Prevent duplicate processing
}

impl MavlinkRouter {
    /// Poll all transports for incoming messages
    pub async fn receive_message(&mut self) -> Result<(MavMessage, TransportId), RouterError> {
        use embassy_futures::select::select_array;

        // Poll all transports concurrently
        let futures = self.transports.iter_mut()
            .map(|t| t.read_message())
            .collect();

        let (msg, transport_id) = select_array(futures).await;

        // Check for duplicate (same message from multiple transports)
        if self.message_dedup.is_duplicate(&msg) {
            return self.receive_message().await;  // Skip duplicate
        }

        Ok((msg, transport_id))
    }

    /// Send message to all active transports
    pub async fn send_message(&mut self, msg: &MavMessage) -> Result<(), RouterError> {
        let encoded = encode_mavlink_message(msg)?;

        for (id, transport) in self.transports.iter_mut().enumerate() {
            if let Err(e) = transport.write(&encoded).await {
                defmt::warn!("Transport {} send failed: {:?}", id, e);
                // Continue to other transports (don't fail all)
            }
        }

        Ok(())
    }
}
```

**Message Deduplication**:

Prevent processing same message received on multiple transports (e.g., GCS sends to both UART and WiFi):

```rust
pub struct MessageDeduplicator {
    recent_messages: heapless::Deque<MessageFingerprint, 16>,
}

struct MessageFingerprint {
    msg_id: u32,
    sequence: u8,
    timestamp_ms: u32,
}

impl MessageDeduplicator {
    pub fn is_duplicate(&mut self, msg: &MavMessage) -> bool {
        let fingerprint = MessageFingerprint::from(msg);

        if self.recent_messages.contains(&fingerprint) {
            return true;
        }

        self.recent_messages.push_back(fingerprint);
        if self.recent_messages.len() > 16 {
            self.recent_messages.pop_front();
        }

        false
    }
}
```

**Independent Transport State**:

Each transport maintains separate:

- RX/TX buffers
- Message counters (sent, received, dropped)
- Error counters
- Last activity timestamp

```rust
pub struct TransportStats {
    messages_sent: u32,
    messages_received: u32,
    messages_dropped: u32,
    errors: u32,
    last_activity_ms: u32,
}
```

**Concurrent Task Architecture**:

```rust
#[embassy_executor::task]
async fn mavlink_uart_task(transport: UartTransport) {
    // UART-specific message handling
}

#[embassy_executor::task]
async fn mavlink_udp_task(transport: UdpTransport) {
    // UDP-specific message handling
}

#[embassy_executor::task]
async fn mavlink_router_task(
    uart_rx: Receiver<MavMessage>,
    udp_rx: Receiver<MavMessage>,
    broadcast_tx: Sender<MavMessage>,
) {
    loop {
        select! {
            msg = uart_rx.recv() => {
                // Process message from UART
                broadcast_tx.send(msg).await;
            }
            msg = udp_rx.recv() => {
                // Process message from UDP
                broadcast_tx.send(msg).await;
            }
        }
    }
}
```

## Platform Considerations

### Unix

N/A - Platform agnostic (embedded system)

### Windows

N/A - Platform agnostic (embedded system)

### Cross-Platform

Concurrent transports must work on both Pico W (RP2040) and Pico 2 W (RP2350). Embassy executor handles concurrent async tasks efficiently on both platforms.

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                          | Validation                             |
| ----------------------------------------- | ------ | ---------- | --------------------------------------------------- | -------------------------------------- |
| Message duplication (same msg both ports) | Medium | High       | Implement message deduplication based on seq number | Test sending same msg via UART and UDP |
| CPU overhead (concurrent tasks)           | Medium | Low        | Profile CPU usage, optimize if >20%                 | Measure CPU load with both active      |
| Buffer contention (shared state)          | Low    | Low        | Use separate buffers per transport                  | Stress test with max message rates     |
| Memory exhaustion (multiple buffers)      | Medium | Low        | Fixed-size buffers, profile memory usage            | Measure RAM with all transports active |
| Transport failure cascade                 | High   | Low        | Isolate transports, don't propagate errors          | Kill WiFi, verify UART continues       |

## Implementation Notes

Preferred approaches:

- Use Embassy `select!` for concurrent transport polling
- Implement message deduplication (sequence number + timestamp)
- Each transport in separate async task or single router with `select_array`
- Shared message queue using Embassy channels
- Follow ArduPilot's multi-channel pattern

Known pitfalls:

- Don't process duplicate messages (GCS may send to multiple transports)
- Avoid shared mutable state between transports (use message passing)
- Embassy `select!` requires `&mut` refs (borrow checker challenges)
- Memory overhead: each transport needs separate buffers

Related code areas:

- `src/communication/mavlink/router.rs` - Multi-transport router
- `src/communication/mavlink/transport/` - Individual transport implementations
- `src/communication/mavlink/dedup.rs` - Message deduplication

Suggested libraries:

- `embassy-executor` - Concurrent async tasks
- `embassy-sync::channel` - Inter-task message passing
- `heapless::Deque` - Fixed-size message history for dedup

## External References

- ArduPilot Multi-Channel: <https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html>
- Embassy Async: <https://embassy.dev/book/dev/runtime.html>
- MAVLink Sequence Numbers: <https://mavlink.io/en/guide/sequence_numbers.html>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
