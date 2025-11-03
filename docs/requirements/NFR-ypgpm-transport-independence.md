# NFR-ypgpm Transport Independence and Reliability

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-6jkia-transport-abstraction](FR-6jkia-transport-abstraction.md)
  - [FR-eutkf-concurrent-transports](FR-eutkf-concurrent-transports.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-oq110-*](../tasks/T-oq110-*/README.md)

## Requirement Statement

Failure or unavailability of one MAVLink transport shall not prevent operation of other transports or core autopilot functions, ensuring system reliability and graceful degradation.

## Rationale

Transport independence is critical for production reliability:

- **Redundancy**: UART continues if WiFi fails, WiFi continues if UART disconnected
- **Graceful Degradation**: Partial connectivity better than total failure
- **Production Robustness**: WiFi instability shouldn't crash entire system
- **Development Flexibility**: Can disable transports individually for testing

Single transport failure causing system failure is unacceptable for production systems.

## User Story (if applicable)

The system shall isolate transport failures to ensure WiFi connection drops don't affect UART operation, UART disconnection doesn't affect WiFi, and transport failures don't interfere with core autopilot functions like attitude estimation and control loops.

## Acceptance Criteria

- [ ] WiFi connection failure doesn't prevent UART operation
- [ ] UART disconnection doesn't prevent WiFi operation
- [ ] Transport failure doesn't crash main autopilot task
- [ ] Core functions (AHRS, control loops) unaffected by transport failures
- [ ] Telemetry continues on working transports when one fails
- [ ] Failed transport can recover without system reboot
- [ ] Transport errors logged clearly (defmt) without flooding
- [ ] No memory leaks from failed transport cleanup
- [ ] System boots normally when WiFi unavailable
- [ ] System operates normally when UART not connected

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Isolation Architecture**:

```rust
// Each transport in separate task
#[embassy_executor::task]
async fn mavlink_uart_task(mut transport: UartTransport, tx: Sender<MavMessage>) {
    loop {
        match transport.read_message().await {
            Ok(msg) => {
                tx.send(msg).await.ok();  // Best effort, don't block
            }
            Err(e) => {
                defmt::warn!("UART error: {:?}", e);
                // Don't crash, continue after error
            }
        }
    }
}

#[embassy_executor::task]
async fn mavlink_udp_task(mut transport: UdpTransport, tx: Sender<MavMessage>) {
    loop {
        match transport.read_message().await {
            Ok(msg) => {
                tx.send(msg).await.ok();
            }
            Err(e) => {
                defmt::warn!("UDP error: {:?}", e);
                // Isolated from UART task
            }
        }
    }
}
```

**Error Handling Strategy**:

- **Non-Fatal Errors**: Log and continue (packet loss, timeout)
- **Fatal Errors**: Disable transport, continue with others (socket bind failure)
- **Recovery**: Attempt transport reinitialization after delay

**Failure Scenarios**:

1. **WiFi unavailable at boot**:
   - Attempt connection for 30s
   - If failure: Log error, disable WiFi transport
   - Continue with UART only

2. **WiFi disconnects during operation**:
   - Log disconnection event
   - Attempt reconnection (exponential backoff)
   - UART continues unaffected

3. **UART disconnected**:
   - Detect via send failure
   - Log event
   - WiFi continues unaffected

4. **Network stack panic** (critical):
   - Catch panic boundary (if possible)
   - Disable WiFi transport
   - System continues with UART

**Health Monitoring**:

```rust
pub struct TransportHealth {
    uart_healthy: bool,
    udp_healthy: bool,
    last_uart_msg_ms: u32,
    last_udp_msg_ms: u32,
}

impl TransportHealth {
    pub fn check(&mut self) {
        let now_ms = embassy_time::Instant::now().as_millis();

        // Mark unhealthy if no activity for 10 seconds
        if now_ms - self.last_uart_msg_ms > 10_000 {
            if self.uart_healthy {
                defmt::warn!("UART transport unhealthy");
                self.uart_healthy = false;
            }
        }

        if now_ms - self.last_udp_msg_ms > 10_000 {
            if self.udp_healthy {
                defmt::warn!("UDP transport unhealthy");
                self.udp_healthy = false;
            }
        }

        // Send health status in SYS_STATUS
        self.send_sys_status();
    }
}
```

**Telemetry Integration**:

Add transport health to SYS_STATUS message:

```rust
sys_status.onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_UART;
sys_status.onboard_control_sensors_present |= MAV_SYS_STATUS_SENSOR_NETWORK;

if uart_healthy {
    sys_status.onboard_control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_UART;
    sys_status.onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_UART;
}

if udp_healthy {
    sys_status.onboard_control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_NETWORK;
    sys_status.onboard_control_sensors_health |= MAV_SYS_STATUS_SENSOR_NETWORK;
}
```

## Platform Considerations

### Unix

N/A - Platform agnostic (embedded system)

### Windows

N/A - Platform agnostic (embedded system)

### Cross-Platform

Transport independence requirements apply equally to Pico W (RP2040) and Pico 2 W (RP2350). Both platforms must handle WiFi and UART failures gracefully.

## Risks & Mitigation

| Risk                                       | Impact | Likelihood | Mitigation                                  | Validation                             |
| ------------------------------------------ | ------ | ---------- | ------------------------------------------- | -------------------------------------- |
| WiFi driver crash affects UART             | High   | Low        | Isolate in separate task, catch panics      | Kill WiFi driver, verify UART works    |
| Transport failure cascades to core systems | High   | Low        | Separate transport tasks from main loop     | Test with both transports failed       |
| Memory leak from failed transport cleanup  | Medium | Low        | Static allocation only, no heap             | Monitor memory after multiple failures |
| Error log flooding (rapid failures)        | Low    | Medium     | Rate-limit error messages (max 1/second)    | Disconnect WiFi repeatedly             |
| Failed transport never recovers            | Medium | Medium     | Implement recovery with exponential backoff | Test recovery after WiFi restoration   |

## Implementation Notes

Preferred approaches:

- Separate async task per transport (isolation)
- Use Result types, never panic on transport errors
- Rate-limit error logging (max 1 per second per transport)
- Health monitoring task (check activity every 10s)
- Report transport health in SYS_STATUS

Known pitfalls:

- Don't use `unwrap()` or `expect()` on transport operations
- Avoid shared mutable state between transports
- Embassy task panics may crash executor (use `catch_unwind` carefully)
- Network stack errors may not be recoverable (disable transport)

Related code areas:

- `src/communication/mavlink/transport/` - Individual transport error handling
- `src/communication/mavlink/health.rs` - Transport health monitoring

Suggested libraries:

- `embassy-executor` - Task isolation
- `defmt` - Lightweight error logging

## External References

- Fault Isolation Patterns: <https://en.wikipedia.org/wiki/Fault_tolerance>
- ArduPilot Serial Manager: <https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
