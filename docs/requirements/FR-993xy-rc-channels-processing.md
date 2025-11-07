# FR-993xy RC_CHANNELS Message Processing

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-gpzpz-mavlink-protocol](FR-gpzpz-mavlink-protocol.md)

- Dependent Requirements:
  - [FR-knp1u-rc-input-health-monitoring](FR-knp1u-rc-input-health-monitoring.md)
  - [FR-p3lx0-emergency-stop-triggers](FR-p3lx0-emergency-stop-triggers.md)
  - [FR-uk0us-manual-mode](FR-uk0us-manual-mode.md)
  - [FR-meyja-manual-mode-implementation](FR-meyja-manual-mode-implementation.md)
  - [NFR-kqvyf-manual-control-latency](NFR-kqvyf-manual-control-latency.md)

- Related Tasks:
  - [T-3irm5-manual-control-implementation](../tasks/T-3irm5-manual-control-implementation/README.md)

## Requirement Statement

The system shall receive and process RC_CHANNELS messages (MAVLink message ID 65) from ground control stations, extracting and normalizing channel values to provide RC input state for control modes.

## Rationale

Mission Planner and other ground control stations can send RC_CHANNELS messages to simulate physical RC transmitter input, enabling manual control without dedicated RC hardware. This is essential for:

- Development and testing without physical RC transmitter
- Remote operation via GCS joystick/gamepad
- Integration testing of manual control modes
- RC input failover (physical RC or MAVLink RC)

Processing RC_CHANNELS provides the foundation for all manual control modes (Manual, Stabilize, etc.).

## User Story (if applicable)

As an operator using Mission Planner, I want to control the rover with a USB joystick connected to my computer, so that I can manually drive the vehicle without purchasing a dedicated RC transmitter.

## Acceptance Criteria

- [ ] Parse RC_CHANNELS messages (MAVLink message ID 65) from any active transport (UART, UDP)
- [ ] Extract all 18 channel values (chan1_raw through chan18_raw)
- [ ] Normalize channel values from raw (0-65535) to standard range (-1.0 to +1.0)
- [ ] Provide RC input state accessible to control modes
- [ ] Detect RC timeout if no RC_CHANNELS received for 1 second
- [ ] Set RC state to "lost" on timeout, triggering fail-safe behavior
- [ ] Update RC state at received message rate (typically 5-10 Hz)
- [ ] Store most recent channel values for access by control loop (50 Hz)
- [ ] Log RC_CHANNELS reception timestamp for debugging

## Technical Details (if applicable)

### Functional Requirement Details

**MAVLink RC_CHANNELS Message Format:**

```rust
pub struct RC_CHANNELS_DATA {
    pub time_boot_ms: u32,    // Timestamp (ms since boot)
    pub chan1_raw: u16,       // Channel 1, 0-65535
    pub chan2_raw: u16,       // Channel 2, 0-65535
    pub chan3_raw: u16,       // Channel 3, 0-65535
    // ... channels 4-18
    pub chancount: u8,        // Number of active channels (1-18)
    pub rssi: u8,             // Signal strength (255 = no physical RC)
}
```

**Channel Normalization:**

Standard RC pulse width: 1000-2000 μs, mapped to 0-65535 in MAVLink:

- **Minimum**: 1000 μs → 0 → -1.0 (normalized)
- **Neutral**: 1500 μs → 32768 → 0.0 (normalized)
- **Maximum**: 2000 μs → 65535 → +1.0 (normalized)

Normalization formula:

```rust
fn normalize_channel(raw: u16) -> f32 {
    // Map 0-65535 to -1.0 to +1.0
    // Center point: 32768 (neutral)
    let centered = raw as i32 - 32768;
    (centered as f32) / 32768.0
}
```

**RC Input State:**

```rust
pub struct RcInput {
    /// Channel values (normalized -1.0 to +1.0)
    pub channels: [f32; 18],
    /// Number of active channels
    pub channel_count: u8,
    /// Last update timestamp (microseconds)
    pub last_update_us: u64,
    /// RC connection status
    pub status: RcStatus,
}

pub enum RcStatus {
    /// RC input active, recent message received
    Active,
    /// RC input lost, timeout exceeded
    Lost,
    /// RC never connected
    NeverConnected,
}
```

**RC Timeout Detection:**

- Timeout threshold: 1000 ms (1 second)
- Check on every control loop iteration (50 Hz)
- Transition to `RcStatus::Lost` if `current_time - last_update_us > 1_000_000`
- Fail-safe action: Set all channels to neutral (0.0)

**Mission Planner Configuration:**

Mission Planner sends RC_CHANNELS when:

- Joystick connected and configured (Setup → Joystick)
- Channel mapping set (axis X → steering, axis Y → throttle)
- Message rate configurable via SR_RC_CHAN parameter (default 5 Hz)

## Platform Considerations

### Pico W (RP2040)

N/A - Platform agnostic (MAVLink message processing independent of hardware)

### Pico 2 W (RP2350)

N/A - Platform agnostic

### Cross-Platform

RC_CHANNELS processing is fully cross-platform. All platforms receive messages via MAVLink transport abstraction.

## Risks & Mitigation

| Risk                                             | Impact | Likelihood | Mitigation                                                             | Validation                                     |
| ------------------------------------------------ | ------ | ---------- | ---------------------------------------------------------------------- | ---------------------------------------------- |
| RC_CHANNELS format mismatch with Mission Planner | High   | Low        | Follow MAVLink spec exactly, test with real Mission Planner            | Connect Mission Planner, verify channel values |
| RC timeout too short (false positives)           | Medium | Medium     | Use 1 second timeout (ArduPilot standard), make parameter-configurable | Test with intermittent network                 |
| Normalization error (incorrect range)            | High   | Low        | Unit test normalization function with boundary values                  | Verify neutral = 0.0, min = -1.0, max = +1.0   |
| Concurrent access to RC state (race condition)   | High   | Low        | Use Mutex or critical section for RC state updates                     | Test with multiple readers (mode + telemetry)  |

## Implementation Notes

Preferred approaches:

- **RC Input Handler**: Create `src/communication/mavlink/handlers/rc_input.rs`
- **State Storage**: Store RC state in shared structure (e.g., `RcInput` in static or Mutex)
- **Access Pattern**: Control modes read RC state, MAVLink handler writes RC state
- **Thread Safety**: Use `embassy_sync::mutex::Mutex` for concurrent access
- **Timeout Check**: Vehicle control task checks timeout on every iteration

Known pitfalls:

- **Stale data**: Ensure timestamp is updated on every message reception
- **Race conditions**: RC state must be thread-safe (multiple readers, one writer)
- **Overflow**: Channel count can be 0-18, validate before array access
- **Normalization edge cases**: Raw value 32768 exactly → 0.0, handle rounding

Related code areas:

- `src/communication/mavlink/handlers/` - MAVLink message handlers
- `src/communication/mavlink/state.rs` - System state (add RC state here)
- `src/vehicle/` - Control modes (will read RC state)

## External References

- MAVLink RC_CHANNELS: <https://mavlink.io/en/messages/common.html#RC_CHANNELS>
- ArduPilot RC Input: <https://ardupilot.org/copter/docs/common-rc-systems.html>
- Mission Planner Joystick Setup: <https://ardupilot.org/planner/docs/joystick.html>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
