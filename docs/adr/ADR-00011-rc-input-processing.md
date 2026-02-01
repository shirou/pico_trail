# ADR-00011 RC Input Processing: MAVLink RC_CHANNELS with Normalization

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-00014-rc-channels-processing](../requirements/FR-00014-rc-channels-processing.md)
  - [FR-00013-manual-mode](../requirements/FR-00013-manual-mode.md)
  - [NFR-00010-manual-control-latency](../requirements/NFR-00010-manual-control-latency.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-00007-manual-control-implementation](../tasks/T-00007-manual-control-implementation/README.md)

## Context

Vehicle control modes (Manual, Stabilize) require RC input to translate operator commands into actuator movements. Traditionally, RC input comes from physical RC receivers (SBUS, PPM, PWM), but for development and testing, ground control stations like Mission Planner can send RC_CHANNELS messages over MAVLink to simulate RC input.

### Problem

We need an RC input processing system that:

- Receives RC_CHANNELS messages (MAVLink message ID 65) from any active transport
- Extracts 18 channel values and normalizes to standard range (-1.0 to +1.0)
- Detects RC timeout (no message received within threshold)
- Provides thread-safe access to RC state (MAVLink task writes, vehicle task reads)
- Works without physical RC hardware (MAVLink-only RC input)

### Constraints

- **Latency**: RC input to actuator response < 100ms (NFR-00010)
- **Update Rate**: RC_CHANNELS sent at 5-10 Hz from Mission Planner
- **Timeout**: Detect RC loss within 1 second (standard failsafe threshold)
- **Memory**: RC state < 200 bytes RAM
- **Thread Safety**: Concurrent access (MAVLink handler writes, control mode reads)

### Prior Art

**ArduPilot RC Input**:

```cpp
class RC_Channel {
    uint16_t radio_in;  // Raw RC value (1000-2000 μs)
    float norm_input(); // Normalized -1.0 to +1.0
};

class RC {
    RC_Channel channels[18];
    bool has_new_input();
    bool failsafe_active();
};
```

**PX4 RC Input**:

- `rc_input` uORB topic publishes RC channel values
- `ManualControl` module subscribes and normalizes values
- Timeout detection via timestamp comparison

## Success Metrics

- **Compatibility**: Works with Mission Planner RC simulation (joystick/gamepad)
- **Accuracy**: Normalization accurate to ±0.01 for 16-bit input
- **Latency**: RC message to mode read < 10ms
- **Reliability**: 100% of timeouts detected within 1 second

## Decision

**We will process RC_CHANNELS messages in a dedicated MAVLink handler, normalize channel values to -1.0 to +1.0, store in a shared RcInput structure protected by Mutex, and detect timeout by comparing timestamps.**

### Architecture

```
┌─────────────────────────────────────────────┐
│    Mission Planner / Ground Control         │
│  (Joystick → RC_CHANNELS MAVLink message)   │
└───────────────┬─────────────────────────────┘
                │ RC_CHANNELS (5-10 Hz)
┌───────────────▼─────────────────────────────┐
│     MAVLink Message Handler                 │
│  (Receive, parse, validate CRC)             │
└───────────────┬─────────────────────────────┘
                │
┌───────────────▼─────────────────────────────┐
│    RC_CHANNELS Handler                      │
│  - Extract 18 channel values                │
│  - Normalize to -1.0 to +1.0                │
│  - Update timestamp                         │
│  - Write to shared RcInput (Mutex)          │
└───────────────┬─────────────────────────────┘
                │
        ┌───────▼────────┐
        │   RcInput      │
        │  (Shared State)│
        │  - Channels[18]│
        │  - Timestamp   │
        │  - Status      │
        └───────┬────────┘
                │
┌───────────────▼─────────────────────────────┐
│      Vehicle Control Task (50 Hz)           │
│  - Read RcInput (Mutex)                     │
│  - Check timeout (current time - last update)│
│  - Mode update with RC values               │
└─────────────────────────────────────────────┘
```

### Decision Drivers

1. **Mission Planner Compatibility**: RC_CHANNELS is standard MAVLink message for RC input
2. **No Hardware Dependency**: Eliminates need for physical RC receiver during development
3. **Thread Safety**: Mutex protects shared state (MAVLink task writes, vehicle task reads)
4. **Standard Normalization**: -1.0 to +1.0 matches ArduPilot convention
5. **Timeout Detection**: Timestamp-based approach is reliable and simple

### Considered Options

- **Option A: RC_CHANNELS + Mutex + Timeout Check** ⭐ Selected
- **Option B: Physical RC Receiver (SBUS/PPM)**
- **Option C: Embassy Channel for RC Updates**

### Option Analysis

**Option A: RC_CHANNELS + Mutex + Timeout Check**

- **Pros**:
  - Works with Mission Planner (no physical RC needed)
  - Standard MAVLink protocol (widely supported)
  - Simple normalization formula
  - Mutex ensures thread safety
  - Timeout detection via timestamp comparison
- **Cons**:
  - Requires active MAVLink connection (no offline testing)
  - Mutex overhead (\~100 ns lock/unlock)
  - Update rate limited by GCS (5-10 Hz, not 50 Hz)
- **Estimated Overhead**: \~200 bytes RAM, \~3 KB Flash

**Option B: Physical RC Receiver (SBUS/PPM)**

- **Pros**:
  - Low latency (direct hardware decoding)
  - High update rate (50+ Hz)
  - Works offline (no GCS needed)
  - Standard in production systems
- **Cons**:
  - Requires additional hardware (RC receiver, transmitter)
  - Platform-specific implementation (UART/GPIO)
  - More complex (protocol decoding, failsafe logic)
  - Higher cost for development
- **Estimated Overhead**: \~500 bytes RAM, \~8 KB Flash

**Option C: Embassy Channel for RC Updates**

- **Pros**:
  - Lock-free (channel is async-safe)
  - Better performance than Mutex
  - Idiomatic Embassy approach
- **Cons**:
  - More complex (async channel setup)
  - May drop messages if buffer full
  - Still requires RC_CHANNELS handling (same logic)
- **Estimated Overhead**: \~300 bytes RAM, \~4 KB Flash

## Rationale

**RC_CHANNELS + Mutex** was selected for:

1. **Development Velocity**: No physical RC hardware needed for testing
2. **Compatibility**: Works with Mission Planner and other GCS out-of-the-box
3. **Simplicity**: Standard MAVLink message, well-documented normalization
4. **Thread Safety**: Mutex is simple and sufficient for 5-10 Hz update rate
5. **Flexibility**: Can add physical RC support later (dual-source RC input)

### Trade-offs Accepted

- **Update Rate**: 5-10 Hz from GCS (vs 50+ Hz physical RC), acceptable for Manual mode
- **Mutex Overhead**: \~100 ns per access (negligible at 50 Hz control loop)

**Decision**: We accept lower update rate for faster development and no hardware dependency.

## Consequences

### Positive

- **No Hardware Dependency**: Develop and test without physical RC transmitter/receiver
- **GCS Integration**: Mission Planner joystick control works immediately
- **Standard Protocol**: RC_CHANNELS is widely supported MAVLink message
- **Thread Safe**: Mutex protects concurrent access
- **Simple Implementation**: Straightforward normalization and timeout logic

### Negative

- **Update Rate**: 5-10 Hz limited by GCS (vs 50+ Hz physical RC)
- **Requires Connection**: No offline RC input (must have active MAVLink link)
- **Mutex Contention**: Potential lock contention (unlikely at low update rates)

### Neutral

- **Physical RC Support**: Can be added later as alternative RC source
- **Dual RC Sources**: Could support both MAVLink and physical RC with source selection

## Implementation Notes

### RcInput State Structure

```rust
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

/// RC input state (shared between MAVLink handler and vehicle task)
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RcStatus {
    /// RC input active, recent message received
    Active,
    /// RC input lost, timeout exceeded
    Lost,
    /// RC never connected
    NeverConnected,
}

impl RcInput {
    pub fn new() -> Self {
        Self {
            channels: [0.0; 18],
            channel_count: 0,
            last_update_us: 0,
            status: RcStatus::NeverConnected,
        }
    }

    /// Update RC channels from MAVLink RC_CHANNELS message
    pub fn update_from_mavlink(&mut self, msg: &mavlink::common::RC_CHANNELS_DATA, current_time_us: u64) {
        // Normalize all channels
        self.channels[0] = Self::normalize_channel(msg.chan1_raw);
        self.channels[1] = Self::normalize_channel(msg.chan2_raw);
        self.channels[2] = Self::normalize_channel(msg.chan3_raw);
        self.channels[3] = Self::normalize_channel(msg.chan4_raw);
        self.channels[4] = Self::normalize_channel(msg.chan5_raw);
        self.channels[5] = Self::normalize_channel(msg.chan6_raw);
        self.channels[6] = Self::normalize_channel(msg.chan7_raw);
        self.channels[7] = Self::normalize_channel(msg.chan8_raw);
        self.channels[8] = Self::normalize_channel(msg.chan9_raw);
        self.channels[9] = Self::normalize_channel(msg.chan10_raw);
        self.channels[10] = Self::normalize_channel(msg.chan11_raw);
        self.channels[11] = Self::normalize_channel(msg.chan12_raw);
        self.channels[12] = Self::normalize_channel(msg.chan13_raw);
        self.channels[13] = Self::normalize_channel(msg.chan14_raw);
        self.channels[14] = Self::normalize_channel(msg.chan15_raw);
        self.channels[15] = Self::normalize_channel(msg.chan16_raw);
        self.channels[16] = Self::normalize_channel(msg.chan17_raw);
        self.channels[17] = Self::normalize_channel(msg.chan18_raw);

        self.channel_count = msg.chancount;
        self.last_update_us = current_time_us;
        self.status = RcStatus::Active;
    }

    /// Normalize channel value from MAVLink range (0-65535) to -1.0 to +1.0
    fn normalize_channel(raw: u16) -> f32 {
        // MAVLink RC_CHANNELS mapping:
        // 0 = 1000 μs (min)
        // 32768 = 1500 μs (neutral)
        // 65535 = 2000 μs (max)
        //
        // Normalize to -1.0 to +1.0
        let centered = raw as i32 - 32768;
        (centered as f32) / 32768.0
    }

    /// Get channel value (1-indexed, like MAVLink and ArduPilot)
    pub fn get_channel(&self, channel: usize) -> f32 {
        if channel == 0 || channel > 18 {
            0.0 // Invalid channel
        } else {
            self.channels[channel - 1]
        }
    }

    /// Check if RC input is lost (timeout exceeded)
    pub fn check_timeout(&mut self, current_time_us: u64) {
        const RC_TIMEOUT_US: u64 = 1_000_000; // 1 second

        if self.status == RcStatus::Active {
            let elapsed = current_time_us.saturating_sub(self.last_update_us);
            if elapsed > RC_TIMEOUT_US {
                self.status = RcStatus::Lost;
                defmt::warn!("RC input lost (timeout)");
                // Set all channels to neutral
                self.channels.fill(0.0);
            }
        }
    }

    /// Check if RC is currently active
    pub fn is_active(&self) -> bool {
        self.status == RcStatus::Active
    }

    /// Check if RC is lost
    pub fn is_lost(&self) -> bool {
        self.status == RcStatus::Lost
    }
}
```

### MAVLink RC_CHANNELS Handler

```rust
// In src/communication/mavlink/handlers/rc_input.rs

use mavlink::common::RC_CHANNELS_DATA;

pub async fn handle_rc_channels(
    msg: &RC_CHANNELS_DATA,
    rc_input: &Mutex<CriticalSectionRawMutex, RcInput>,
) {
    let current_time_us = embassy_time::Instant::now().as_micros();

    // Update RC input state (thread-safe)
    rc_input.lock().await.update_from_mavlink(msg, current_time_us);

    defmt::trace!(
        "RC_CHANNELS: ch1={}, ch3={}, count={}",
        msg.chan1_raw,
        msg.chan3_raw,
        msg.chancount
    );
}
```

### Vehicle Control Task Integration

```rust
#[embassy_executor::task]
async fn vehicle_control_task(
    mode_manager: &'static mut ModeManager,
    rc_input: &'static Mutex<CriticalSectionRawMutex, RcInput>,
) {
    let mut ticker = Ticker::every(Duration::from_millis(20)); // 50 Hz

    loop {
        let current_time_us = embassy_time::Instant::now().as_micros();

        // Check RC timeout
        rc_input.lock().await.check_timeout(current_time_us);

        // Execute active mode
        if let Err(e) = mode_manager.execute(current_time_us) {
            defmt::error!("Mode execution error: {}", e);
        }

        ticker.next().await;
    }
}
```

### Manual Mode RC Input Access

```rust
impl ManualMode {
    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        // Access RC input (lock held briefly)
        let rc = self.rc_input.lock().await;

        // Check RC status
        if rc.is_lost() {
            // Failsafe: neutral outputs
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        }

        // Read RC channels (1-indexed, like MAVLink)
        let steering = rc.get_channel(1); // Channel 1: steering
        let throttle = rc.get_channel(3); // Channel 3: throttle

        // Release lock
        drop(rc);

        // Command actuators
        self.actuators.set_steering(steering)?;
        self.actuators.set_throttle(throttle)?;

        Ok(())
    }
}
```

### RC Channel Mapping

```rust
/// RC channel mapping (ArduPilot-compatible)
pub mod rc_channels {
    pub const STEERING: usize = 1;  // Roll (channel 1)
    pub const THROTTLE: usize = 3;  // Throttle (channel 3)
    pub const MODE_SWITCH: usize = 5; // Aux1 (channel 5) - Optional
}
```

### Unit Test Example

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_channel() {
        // Test neutral (32768 → 0.0)
        let normalized = RcInput::normalize_channel(32768);
        assert!((normalized - 0.0).abs() < 0.01, "Neutral should be 0.0");

        // Test minimum (0 → -1.0)
        let normalized = RcInput::normalize_channel(0);
        assert!((normalized - (-1.0)).abs() < 0.01, "Min should be -1.0");

        // Test maximum (65535 → +1.0)
        let normalized = RcInput::normalize_channel(65535);
        assert!((normalized - 1.0).abs() < 0.01, "Max should be +1.0");
    }

    #[test]
    fn test_rc_timeout_detection() {
        let mut rc_input = RcInput::new();

        // Simulate RC message
        let msg = create_test_rc_channels();
        rc_input.update_from_mavlink(&msg, 1_000_000);
        assert_eq!(rc_input.status, RcStatus::Active);

        // Check timeout after 500ms (should still be active)
        rc_input.check_timeout(1_500_000);
        assert_eq!(rc_input.status, RcStatus::Active);

        // Check timeout after 1.5 seconds (should be lost)
        rc_input.check_timeout(2_600_000);
        assert_eq!(rc_input.status, RcStatus::Lost);

        // Channels should be zeroed
        assert_eq!(rc_input.get_channel(1), 0.0);
    }
}
```

## Platform Considerations

- **Platform Agnostic**: RC_CHANNELS processing is hardware-independent
- **RP2040/RP2350**: No platform-specific considerations
- **Cross-Platform**: All platforms receive RC_CHANNELS via MAVLink transport

## Security & Privacy

- No sensitive data in RC channels
- RC_CHANNELS accepted from any authenticated MAVLink source

## Monitoring & Logging

- **RC Status**: Log RC connection status changes (Active → Lost)
- **Channel Values**: Log RC channels at 1 Hz for debugging (trace level)
- **Timeout Events**: Log RC timeout events immediately (warn level)
- **Normalization**: Verify channel normalization in unit tests

## Open Questions

- [ ] Should RC timeout be configurable via parameter? → Decision: Yes, add FS_RC_TIMEOUT parameter (default 1000ms)
- [ ] Do we support RC override (switch from Auto to Manual via RC)? → Next step: Defer to Phase 2, implement mode switch via channel 5
- [ ] Should we filter RC input (smoothing, deadband)? → Method: Start without filtering, add if needed for noisy inputs

## External References

- MAVLink RC_CHANNELS: <https://mavlink.io/en/messages/common.html#RC_CHANNELS>
- ArduPilot RC Input: <https://ardupilot.org/copter/docs/common-rc-systems.html>
- Mission Planner Joystick Setup: <https://ardupilot.org/planner/docs/joystick.html>
- RC PWM Standard: <https://www.rcgroups.com/forums/showthread.php?1323503-PWM-to-PPM-encoder-(Tutorial)>
