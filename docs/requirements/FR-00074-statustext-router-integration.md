# FR-00074 STATUSTEXT Integration with MAVLink Router

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00020-statustext-notifications](../analysis/AN-00020-statustext-notifications.md)
- Prerequisite Requirements:
  - [FR-00072-statustext-message-queue](../requirements/FR-00072-statustext-message-queue.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00014-statustext-implementation](../tasks/T-00014-statustext-implementation/README.md)

## Requirement Statement

The system shall integrate STATUSTEXT message transmission with the existing MAVLink router, draining the notification queue during telemetry generation and routing messages to all connected transports (UART, USB) alongside heartbeat and sensor telemetry.

## Rationale

STATUSTEXT messages must reach ground control stations via the same MAVLink transport as heartbeats and telemetry. The MAVLink router already handles message routing to multiple transports. Integration ensures STATUSTEXT messages are delivered reliably using existing infrastructure without duplicating transport logic.

## User Story

As a **MAVLink router component**, I want **to drain the STATUSTEXT notification queue during telemetry cycles**, so that **status messages are transmitted to all connected GCS transports alongside regular telemetry data**.

## Acceptance Criteria

- [ ] MAVLink router drains STATUSTEXT queue during each telemetry generation cycle
- [ ] STATUSTEXT messages sent via MavlinkRouter to all transports
- [ ] Messages routed to UART transport (primary GCS connection)
- [ ] Messages routed to USB transport (secondary/debug connection)
- [ ] STATUSTEXT transmission does not block heartbeat or sensor telemetry
- [ ] Compatible with existing 1 Hz heartbeat streaming
- [ ] Compatible with existing sensor telemetry streaming
- [ ] Multiple STATUSTEXT messages sent in single cycle if queued (batch drain)
- [ ] Empty queue results in no STATUSTEXT transmission (no overhead)

## Technical Details

### Functional Requirement Details

**Integration Point:**

The MAVLink router's telemetry generation loop already sends heartbeats and sensor data. STATUSTEXT messages should be drained and sent in the same loop.

**Proposed Router Modification:**

```rust
// In src/communication/mavlink/router.rs or similar
async fn telemetry_loop() {
    loop {
        // Existing heartbeat
        send_heartbeat().await;

        // Existing telemetry
        send_battery_status().await;
        send_sys_status().await;

        // NEW: Drain STATUSTEXT queue
        while let Some(msg) = status_notifier.dequeue() {
            let chunks = chunk_statustext(msg.severity, &msg.text);
            for chunk in chunks {
                send_message(chunk).await;
            }
        }

        Timer::after(Duration::from_millis(1000)).await;
    }
}
```

**Message Routing:**

- STATUSTEXT messages use same `send_message()` or `write_versioned()` as other MAVLink messages
- Router handles transport multiplexing (same message to all transports)
- No special handling needed for STATUSTEXT vs other message types

**Batch Draining:**

- Drain all queued messages in single cycle (while queue not empty)
- Prevents multi-second delay for queued messages
- Rate limiting at enqueue side prevents flooding (see FR-00072)

**Error Handling:**

- If transport write fails, log error but continue draining queue
- Do not re-queue failed messages (fire-and-forget semantic)

## Platform Considerations

### Embedded (RP2350)

- Integration must not increase telemetry loop time significantly
- Dequeue and send operations must be efficient (<1ms total)
- Compatible with embassy async runtime

### Host Tests

- Mock router can verify STATUSTEXT messages sent
- Test suite can verify message routing to multiple transports
- Can verify batch draining behavior

### Cross-Platform

- Router integration pattern same on all platforms
- Transport abstraction hides platform-specific UART/USB details

## Risks & Mitigation

| Risk                                                      | Impact | Likelihood | Mitigation                                      | Validation                                    |
| --------------------------------------------------------- | ------ | ---------- | ----------------------------------------------- | --------------------------------------------- |
| STATUSTEXT transmission blocks heartbeat (GCS timeout)    | High   | Low        | Limit batch size to 5 messages per cycle        | Monitor heartbeat timing with oscilloscope    |
| Queue drain increases telemetry loop time beyond 1 second | Medium | Low        | Benchmark drain time, optimize if needed        | Measure loop time with 16 queued messages     |
| Transport write failure loses messages                    | Medium | Low        | Accept fire-and-forget semantic, log failures   | Test with intentional transport disconnection |
| Message ordering not preserved across transports          | Low    | Low        | Use same router logic as other MAVLink messages | Verify identical message stream on UART/USB   |
| Integration breaks existing telemetry streaming           | High   | Low        | Thorough testing with existing telemetry tests  | Run full telemetry test suite after changes   |

## Implementation Notes

**Preferred Patterns:**

- Add STATUSTEXT drain to existing telemetry loop
- Reuse existing message sending infrastructure
- No special transport handling required

**Known Pitfalls:**

- Do not drain queue before heartbeat (heartbeat must be timely)
- Do not block on queue drain (async or limited batch size)
- Do not assume queue is always empty (test with burst scenarios)

**Related Code:**

- MAVLink router: `src/communication/mavlink/router.rs`
- Message dispatcher: `src/communication/mavlink/dispatcher.rs`
- Transport abstraction: `src/communication/mavlink/` (UART/USB handlers)

**Suggested Approach:**

1. Add queue drain after existing telemetry messages
2. Limit batch size to 5 messages per cycle initially
3. Monitor actual queue depth and batch size in production
4. Adjust limits based on observed behavior

## External References

- [MAVLink Router Pattern](https://mavlink.io/en/guide/routing.html)
- [MAVLink Streaming Rates](https://mavlink.io/en/services/stream_rates.html)
