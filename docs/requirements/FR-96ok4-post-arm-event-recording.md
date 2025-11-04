# FR-96ok4 Post-Arm Event Recording

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Parent Analysis: [AN-m4dpl-post-arm-initialization](../analysis/AN-m4dpl-post-arm-initialization.md)
- Prerequisite Requirements: N/A - Foundational post-arm initialization requirement
- Dependent Requirements:
  - [FR-c6ej0-arm-subsystem-notification](FR-c6ej0-arm-subsystem-notification.md)
  - [FR-fusl9-arming-checks-warning](FR-fusl9-arming-checks-warning.md)
  - [FR-xeo4y-actuator-armed-initialization](FR-xeo4y-actuator-armed-initialization.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall record arm events with complete context (timestamp, method, checks status), storing in PostArmState and logging to persistent storage with format "ARM,timestamp_ms,method,checks_enabled", enabling time-based vehicle logic, post-flight analysis, and audit trail compliance.

## Rationale

Complete arm event recording is essential for:

- **Time-based logic**: Monitoring system needs arm timestamp for RC/GCS loss detection, failsafe timeout calculations, and duration tracking
- **Post-flight analysis**: Investigators need to know when, how, and under what conditions vehicle armed
- **Safety investigations**: Regulatory bodies require complete operational history including arm method and checks status
- **Operational debugging**: Developers troubleshoot by correlating arm events with subsequent failures
- **Compliance**: Aerospace certification standards mandate logging of all state transitions

ArduPilot records `last_arm_time_us` and `_last_arm_method` immediately after arming, logs all arm events via `Log_Write_Arm()`, and uses this data throughout the codebase for timeout calculations and audit trails.

## User Story (if applicable)

As a vehicle subsystem (monitoring, failsafe, logging) or safety investigator, I want complete arm event context (when, how, checks status), so that I can calculate timeouts, track operational duration, correlate events temporally, and reconstruct the vehicle's operational timeline for incident analysis.

## Acceptance Criteria

### Timestamp Recording

- [ ] Record arm timestamp in milliseconds (u32) when vehicle arms
- [ ] Timestamp recorded immediately after armed state set (first post-arm action)
- [ ] `time_since_arm_ms()` API returns Option<u32>: Some(duration) if armed, None if disarmed
- [ ] Timestamp persists until disarm (reset to None on disarm)
- [ ] Timestamp used by monitoring for RC/GCS loss detection
- [ ] Timestamp wraps safely after 49 days (2^32 ms), delta calculations handle wrap-around

### Arm Method Recording

- [ ] Support arm methods: GcsCommand, RcRudder, RcSwitch, ForceArm
- [ ] Arm method stored in PostArmState when arm() succeeds
- [ ] Arm method accessible via `arm_method()` API returning Option<ArmMethod>
- [ ] Arm method reset to None on disarm
- [ ] Distinguish forced arm (checks bypassed) from normal arm

### Event Logging

- [ ] Log entry format: "ARM,{timestamp_ms},{method},{checks}" where checks is 1 (enabled) or 0 (disabled)
- [ ] Log written immediately after timestamp/method recording, before actuator initialization
- [ ] Log write failure warns operator (STATUSTEXT) but does not prevent arming
- [ ] Log persists across power cycles (flash or persistent storage)
- [ ] Log readable post-flight via USB serial, MAVLink, or SD card

### Integration

- [ ] Test coverage: verify timestamp/method/logging on arm, cleared on disarm
- [ ] Verify monitoring system uses timestamp for timeout detection
- [ ] Verify log format correct and parseable

## Technical Details (if applicable)

### Functional Requirement Details

**PostArmState Structure:**

```rust
/// Post-arm initialization data
pub struct PostArmState {
    /// Timestamp when vehicle armed (milliseconds since boot)
    pub arm_time_ms: u32,

    /// Method used to arm
    pub arm_method: ArmMethod,

    /// Were arming checks performed?
    pub checks_performed: bool,
}

/// Arm method enumeration (for logging and audit)
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ArmMethod {
    /// Unknown arm method (should not occur in normal operation)
    Unknown,

    /// Armed via RC stick pattern (throttle down + rudder right for 1 second)
    RcRudder,

    /// Armed via MAV_CMD_COMPONENT_ARM_DISARM from GCS
    GcsCommand,

    /// Armed via RC aux switch (RCx_OPTION = arm/disarm)
    RcSwitch,

    /// Forced arm (bypassing all arming checks)
    ForceArm,
}
```

**Post-Arm Initialization Sequence:**

```rust
impl SystemState {
    /// Post-arm initialization sequence
    fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                     -> Result<(), &'static str> {
        // 1. Record timestamp (first action)
        let arm_time_ms = get_time_ms();
        self.post_arm_state = PostArmState {
            arm_time_ms,
            arm_method: method,
            checks_performed,
        };

        // 2. Log arming event (immediately after timestamp)
        if let Err(e) = self.log_arm_event(arm_time_ms, method, checks_performed) {
            // Log failure: warn operator but continue arming
            self.send_warning("Arm event logging failed")?;
            warn!("Arm logging error: {}", e);
            // Do NOT return error - logging failure is not critical
        }

        // 3. Continue with other post-arm steps...
        Ok(())
    }

    /// Log arming event to persistent storage
    fn log_arm_event(&mut self, timestamp: u32, method: ArmMethod,
                     checks_performed: bool) -> Result<(), &'static str> {
        let log_entry = format!(
            "ARM,{},{:?},{}",
            timestamp,
            method,
            if checks_performed { "1" } else { "0" }
        );

        self.logger.write(&log_entry)?;
        info!("Arm event logged: {}", log_entry);
        Ok(())
    }
}
```

**API Design:**

```rust
impl SystemState {
    /// Get time since arm in milliseconds
    /// Returns None if vehicle is disarmed
    pub fn time_since_arm_ms(&self) -> Option<u32> {
        if !self.is_armed() {
            return None;
        }

        let current_time = get_time_ms();
        // Handle wrapping: u32 subtraction wraps correctly
        Some(current_time.wrapping_sub(self.post_arm_state.arm_time_ms))
    }

    /// Get current arm method
    pub fn arm_method(&self) -> Option<ArmMethod> {
        if !self.is_armed() {
            return None;
        }

        Some(self.post_arm_state.arm_method)
    }

    /// Get armed duration in seconds (convenience method)
    pub fn armed_duration_s(&self) -> Option<f32> {
        self.time_since_arm_ms().map(|ms| ms as f32 / 1000.0)
    }
}
```

**Log Entry Format:**

```
ARM,12345678,GcsCommand,1
ARM,12346789,RcRudder,1
ARM,12347890,ForceArm,0
```

Fields:

1. **Event Type**: "ARM" (literal string)
2. **Timestamp**: arm_time_ms (u32, milliseconds since boot)
3. **Arm Method**: ArmMethod enum as debug string
4. **Checks Enabled**: "1" if performed, "0" if bypassed

**Timestamp Source:**

- Embedded (RP2040/RP2350): `embassy_time::Instant::now().as_millis()` or custom monotonic timer
- Host tests: Mock time source for deterministic testing
- Resolution: 1 millisecond (sufficient for timeout detection)
- Overflow: 2^32 ms = 49.7 days, acceptable for typical flight durations

**Storage Targets:**

- **Flash memory**: Most common for embedded, wear-leveling required
- **SD card**: If hardware present, easier post-flight retrieval
- **RAM buffer**: Temporary (not ideal for audit trail)
- **MAVLink streaming**: Real-time (supplementary, not primary)

**Retrieval Methods:**

- **USB serial**: Read logs via serial console post-flight
- **MAVLink**: Request logs via LOG_REQUEST_LIST/LOG_REQUEST_DATA
- **Direct SD card access**: Remove SD card, read files on PC

**Arm Method Determination:**

**1. GCS Command (MAVLink):**

```rust
fn handle_arm_disarm(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
    let should_arm = cmd.param1 > 0.5;
    let force_arm = cmd.param2 > 0.5;  // param2=21196 for force arm

    if should_arm {
        let method = if force_arm {
            ArmMethod::ForceArm
        } else {
            ArmMethod::GcsCommand
        };

        match self.state.arm(method, !force_arm) {
            Ok(()) => MavResult::MAV_RESULT_ACCEPTED,
            Err(_) => MavResult::MAV_RESULT_DENIED,
        }
    }
}
```

**2. RC Rudder (Stick Pattern):**

```rust
fn check_arm_disarm_stick_pattern(&mut self) {
    let throttle = self.rc.channel(2);
    let rudder = self.rc.channel(3);

    if throttle < -0.8 && rudder > 0.8 {
        if self.stick_pattern_duration > 1000 {
            let _ = self.state.arm(ArmMethod::RcRudder, true);
        }
    }
}
```

**3. RC Switch (Aux Channel):**

```rust
fn handle_rc_arm_switch(&mut self, switch_state: RcSwitchState) {
    match switch_state {
        RcSwitchState::High => {
            let _ = self.state.arm(ArmMethod::RcSwitch, true);
        }
        _ => {}
    }
}
```

**Usage Example (Monitoring System):**

```rust
// Check if RC signal lost for > 3 seconds while armed
if let Some(time_since_arm) = system_state.time_since_arm_ms() {
    let time_since_last_rc = current_time - last_rc_timestamp;

    if time_since_last_rc > 3000 && time_since_arm > 1000 {
        // RC lost for 3s, vehicle armed for 1s -> trigger failsafe
        trigger_rc_lost_failsafe();
    }
}
```

## Platform Considerations

### Pico W (RP2040)

- **Flash storage**: 2 MB flash, use LittleFS or custom wear-leveling
- **No SD card**: Logs stored in flash, retrieved via USB serial
- **Wear leveling**: Flash endurance \~10k-100k cycles, limit log frequency

### Pico 2 W (RP2350)

- **Flash storage**: 4 MB flash, same considerations as RP2040
- **Optional SD card**: If hardware added, prefer SD for easier retrieval

### Cross-Platform

- Timestamp source abstracted via platform-specific `get_time_ms()` function
- Log format and API platform-agnostic
- Storage backend abstracted via logging subsystem trait

## Risks & Mitigation

| Risk                                                  | Impact | Likelihood | Mitigation                                                                             | Validation                                                       |
| ----------------------------------------------------- | ------ | ---------- | -------------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| Timestamp wraps after 49 days, calculations incorrect | Medium | Very Low   | Use wrapping_sub() for delta calculations, document overflow behavior                  | Test: simulate 49-day wrap, verify duration calculations work    |
| Clock source unavailable/unreliable on embedded       | High   | Low        | Use hardware timer with known reliability, test on actual hardware                     | Integration test: verify timestamp monotonic and accurate        |
| Flash wear-out from excessive logging                 | Medium | Medium     | Limit log frequency (only arm/disarm events), use wear-leveling, flash has 10k+ cycles | Monitor flash health in testing, estimate cycles per lifetime    |
| Log write blocks arm operation (latency)              | Medium | Low        | Non-blocking write (async or timeout), warn on failure but continue                    | Test: simulate slow flash, verify arm completes within 50ms      |
| Storage full, log write fails silently                | Medium | Medium     | Check storage space periodically, warn operator if low, implement log rotation         | Test: fill storage, verify warning sent and arm continues        |
| Log corruption from power loss during write           | Medium | Low        | Use atomic writes (write to temp, rename), or log entries with CRC/checksums           | Test: power cycle during write, verify log readable              |
| Arm method Unknown in logs (indicates logic gap)      | Medium | Low        | Default to Unknown if method not determined, investigate and fix root cause            | Review logs: verify no Unknown methods in normal operation       |
| ForceArm overused by operators (safety risk)          | High   | Medium     | Log ForceArm usage prominently, send operator warning, track frequency in telemetry    | Operational review: monitor ForceArm frequency, provide training |
| RC rudder pattern false positive (accidental arm)     | High   | Low        | Require 1-second stick hold, center sticks on disarm to prevent re-arm                 | Test: verify 1s hold required, verify no false positives         |
| time_since_arm_ms() called before arm, returns stale  | Low    | Low        | Always check armed state first (returns None if disarmed), clear timestamp on disarm   | Unit test: call API when disarmed, verify None returned          |

## Implementation Notes

**Preferred approaches:**

- **Record timestamp first**: First action in post_arm_init(), before any steps that might fail
- **Required parameter**: Arm method required parameter to `arm()`, cannot arm without providing
- **Non-blocking write**: Logging failure must NOT prevent arming
- **Simple format**: CSV-like format, easy to parse with scripts or spreadsheets
- **Enum for type safety**: Use enum (not string/int) for compile-time checking
- **Wrapping arithmetic**: Use `wrapping_sub()` for all delta calculations to handle overflow
- **Monotonic time**: Use monotonic clock (time since boot), not wall clock

**Known pitfalls:**

- **Unknown method**: Always provide explicit method, never default to Unknown (indicates logic bug)
- **ForceArm casual usage**: ForceArm should be rare (test/emergency only), log prominently
- **Blocking writes**: Do NOT wait indefinitely for flash write, use timeout or async write
- **Silent failures**: Always warn operator if logging fails (don't fail silently)
- **Format drift**: Document log format in ADR, version it, maintain backward compatibility
- **Off-by-one**: Check armed state BEFORE accessing timestamp (defense against stale data)
- **Clock discontinuities**: Avoid wall clock (RTC) that can jump backward, use monotonic timer

**Related code areas:**

- `src/communication/mavlink/state.rs` - SystemState arm() and post_arm_init()
- `src/communication/mavlink/handlers/command.rs` - MAVLink arm/disarm command
- `src/vehicle/rc/` - RC input processing (RcRudder, RcSwitch detection)
- `src/core/logging/` - Logging subsystem
- `src/platform/rp2040/storage.rs` - Platform-specific flash storage
- `src/core/time.rs` - Platform-specific time source abstraction

**Suggested libraries:**

- **LittleFS**: Embedded filesystem with wear leveling (good for flash storage)
- **embassy-embedded-hal**: Flash driver abstraction for RP2040/RP2350
- **heapless**: Fixed-size string formatting for log entries (no allocation)

## External References

- Analysis: [AN-m4dpl-post-arm-initialization](../analysis/AN-m4dpl-post-arm-initialization.md)
- ArduPilot Arming Methods: <https://ardupilot.org/copter/docs/arming_the_motors.html>
- ArduPilot Logging: <https://ardupilot.org/copter/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html>
- MAVLink MAV_CMD_COMPONENT_ARM_DISARM: <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>
- MAVLink LOG_REQUEST: <https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST>
- LittleFS: <https://github.com/littlefs-project/littlefs>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
