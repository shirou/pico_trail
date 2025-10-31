# NFR-eddfs Network Transport Latency Limits

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-ydttj-udp-network-transport](FR-ydttj-udp-network-transport.md)
- Dependent Requirements:
  - [NFR-ukjvr-control-loop-latency](NFR-ukjvr-control-loop-latency.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Network transport shall not add more than 100ms additional latency to MAVLink command acknowledgment compared to UART transport, ensuring safety-critical commands remain responsive.

## Rationale

Latency impacts safety and user experience:

- **Safety Commands**: ARM/DISARM, emergency stop must respond quickly
- **Control Responsiveness**: Mode changes, parameter updates need timely feedback
- **User Experience**: GCS operators expect <200ms response for commands
- **UART Baseline**: Current UART latency is <10ms for COMMAND_ACK

Adding 100ms keeps total latency <110ms, well within acceptable range for manual operations.

## User Story (if applicable)

The system shall limit network transport latency to 100ms additional overhead to ensure commands like ARM/DISARM, mode changes, and emergency stop respond quickly enough for safe manual operation and acceptable user experience.

## Acceptance Criteria

- [ ] Command round-trip latency ≤ 110ms (COMMAND_LONG → COMMAND_ACK via UDP)
- [ ] Parameter read latency ≤ 150ms (PARAM_REQUEST_READ → PARAM_VALUE via UDP)
- [ ] Telemetry latency ≤ 50ms (sensor data → UDP transmission)
- [ ] WiFi connection establishment ≤ 30 seconds
- [ ] UDP socket bind ≤ 100ms
- [ ] No added latency during normal operation (queuing delay = 0)
- [ ] Latency measured and documented under normal and high-load conditions
- [ ] UART transport latency remains unchanged when network transport active

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Latency Budget Breakdown**:

| Operation              | UART Baseline | UDP Target | Additional Latency | Total |
| ---------------------- | ------------- | ---------- | ------------------ | ----- |
| Command ACK            | <10ms         | 100ms      | +90ms              | 110ms |
| Parameter read         | <50ms         | 150ms      | +100ms             | 150ms |
| Telemetry transmission | <5ms          | 50ms       | +45ms              | 50ms  |

**Latency Sources**:

1. **WiFi PHY latency**: \~10-20ms (802.11n overhead)
2. **UDP stack processing**: \~5-10ms (embassy-net)
3. **Transport abstraction**: <1ms (trait dispatch)
4. **Queuing delay**: 0ms (direct send, no queue)
5. **Network congestion**: Variable (mitigated by local WiFi)

**Measurement Strategy**:

```rust
// Latency measurement
pub struct LatencyTracker {
    command_start_us: u64,
}

impl LatencyTracker {
    pub fn mark_command_sent(&mut self) {
        self.command_start_us = embassy_time::Instant::now().as_micros();
    }

    pub fn mark_command_ack(&mut self) {
        let now = embassy_time::Instant::now().as_micros();
        let latency_us = now - self.command_start_us;

        defmt::info!("Command latency: {} ms", latency_us / 1000);

        if latency_us > 110_000 {
            defmt::warn!("High latency: {} ms", latency_us / 1000);
        }
    }
}
```

**Performance Targets**:

- **Normal load** (10Hz attitude + 5Hz GPS): Latency <50ms
- **High load** (50Hz attitude + 10Hz GPS): Latency <100ms
- **Maximum load** (saturated WiFi): Latency <200ms (acceptable degradation)

**Optimization Strategies**:

If latency exceeds budget:

1. **Prioritize commands**: Send commands immediately, queue telemetry
2. **Reduce telemetry rates**: Lower default SR_EXTRA1 from 10Hz to 5Hz
3. **UDP vs TCP**: UDP chosen specifically for lower latency (no connection setup)
4. **Direct send**: Avoid queuing, send messages immediately

**Latency Monitoring**:

```rust
#[embassy_executor::task]
async fn latency_monitor_task(rx: Receiver<LatencyEvent>) {
    let mut histogram = LatencyHistogram::new();

    loop {
        let event = rx.recv().await;
        histogram.record(event.latency_ms);

        // Log statistics every 60 seconds
        if histogram.count() % 60 == 0 {
            defmt::info!("Latency stats: p50={} ms, p95={} ms, p99={} ms",
                histogram.percentile(50),
                histogram.percentile(95),
                histogram.percentile(99));
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

Network latency requirements apply equally to Pico W (RP2040) and Pico 2 W (RP2350). WiFi hardware (CYW43439) and network stack (embassy-net) are identical on both platforms.

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                 | Validation                               |
| ----------------------------------------- | ------ | ---------- | ------------------------------------------ | ---------------------------------------- |
| WiFi congestion increases latency         | Medium | Medium     | Limit telemetry rates, use local WiFi      | Test on congested 2.4GHz network         |
| Embassy-net processing overhead           | Low    | Low        | Profile async task scheduling              | Measure with embedded profiler           |
| Queuing delay under high load             | Medium | Low        | Priority queue for commands                | Stress test with max message rates       |
| Network packet loss causes retransmission | Medium | Medium     | UDP (no retransmission at transport level) | Test with simulated packet loss          |
| CYW43439 driver latency                   | Low    | Low        | Use proven cyw43 crate                     | Measure WiFi driver latency in isolation |

## Implementation Notes

Preferred approaches:

- Use `embassy_time::Instant` for high-resolution timestamps
- Add latency tracking to command handler
- Log latency percentiles (p50, p95, p99)
- No queuing: send messages immediately
- UDP for lower latency vs TCP

Known pitfalls:

- WiFi latency variable (10-50ms range)
- Network congestion unpredictable (use local WiFi)
- Async task scheduling can add jitter
- Don't rely on microsecond precision (use milliseconds)

Related code areas:

- `src/communication/mavlink/router.rs` - Latency measurement
- `src/communication/mavlink/handlers/command.rs` - Command ACK timing

Suggested libraries:

- `embassy-time` - High-resolution timestamps
- `heapless::histogram` - Latency statistics (if available)

## External References

- WiFi Latency Analysis: <https://www.intel.com/content/www/us/en/wireless-network/80211n-latency.html>
- Embassy Time: <https://docs.embassy.dev/embassy-time/>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
