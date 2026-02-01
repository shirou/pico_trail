# FR-00050 Post-Disarm Event Recording

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Parent Analysis: [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
- Prerequisite Requirements: N/A - Foundational post-disarm cleanup requirement
- Dependent Requirements:
  - [FR-00027-disarm-subsystem-notification](FR-00027-disarm-subsystem-notification.md)
  - [FR-00016-actuator-safety-verification](FR-00016-actuator-safety-verification.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall record disarm events with complete context (timestamp, method, reason, armed duration), storing in PostDisarmState and logging to persistent storage with format "DISARM,timestamp_ms,method,reason,duration_ms", enabling post-flight analysis, audit trail compliance, and operational pattern analysis.

## Rationale

Complete disarm event recording is essential for:

- **Post-flight analysis**: Investigators need to know when, how, and why vehicle disarmed
- **Audit trails**: Distinguish between operator commands (GCS/RC) and automatic actions (failsafe)
- **Security analysis**: Forced disarms indicate override conditions requiring investigation
- **User behavior analysis**: Understand whether operators prefer GCS or RC for disarm
- **System validation**: Verify failsafe system correctly triggers automatic disarms
- **Safety investigations**: Forced disarms may indicate emergency conditions
- **Operational patterns**: Analyze disarm reasons to identify training or process issues

Understanding both disarm method (HOW) and reason (WHY) provides complete context:

- **Disarm method** = HOW disarm was initiated (GcsCommand, RcCommand, Failsafe, ForceDisarm)
- **Disarm reason** = WHY disarm occurred (Manual, BatteryFailsafe, Crash, etc.)

ArduPilot logs all disarm events via `Log_Write_Disarm()`, recording timestamp, method, reason, and armed duration. This data is essential for post-flight analysis and regulatory compliance.

## User Story (if applicable)

As an operator or safety investigator, I want complete disarm event context (when, how, why, duration), so that I can understand disarm circumstances, analyze system behavior and operator actions, and reconstruct the vehicle's operational timeline for incident analysis.

## Acceptance Criteria

### Disarm Method Recording

- [ ] Support disarm methods: GcsCommand, RcCommand, Failsafe, ForceDisarm
- [ ] Disarm method stored in PostDisarmState structure
- [ ] Disarm method accessible via system API for runtime queries
- [ ] Invalid or unknown methods logged as "Unknown" but do not prevent disarm
- [ ] Distinguish forced disarm (checks bypassed) from normal disarm

### Disarm Reason Recording

- [ ] Support disarm reasons: Manual, Auto, BatteryFailsafe, RcLoss, GcsLoss, Crash, EmergencyStop
- [ ] Disarm reason stored in PostDisarmState structure
- [ ] Disarm reason accessible via system API for runtime queries
- [ ] Invalid or unknown reasons logged as "Unknown" but do not prevent disarm
- [ ] Reason complements method in providing full context

### Event Logging

- [ ] Log entry format: "DISARM,{timestamp_ms},{method},{reason},{duration_ms}"
- [ ] Log written immediately after subsystem notification, before actuator safety verification
- [ ] Log write failure warns operator (STATUSTEXT) but does not prevent disarm
- [ ] Log persists across power cycles (flash or persistent storage)
- [ ] Log readable post-flight via USB serial, MAVLink, or SD card

### Integration

- [ ] Disarm method persists until next arm cycle (available for post-disarm queries)
- [ ] Disarm reason persists until next arm cycle
- [ ] Armed duration calculated from arm timestamp to disarm timestamp
- [ ] Test coverage: verify method/reason/logging on disarm, accessible post-disarm

## Technical Details (if applicable)

### Functional Requirement Details

**PostDisarmState Structure:**

```rust
/// Post-disarm cleanup data
pub struct PostDisarmState {
    /// Timestamp when vehicle disarmed (milliseconds since boot)
    pub disarm_time_ms: u32,

    /// Method used to disarm
    pub disarm_method: DisarmMethod,

    /// Reason for disarm
    pub disarm_reason: DisarmReason,

    /// Duration vehicle was armed (milliseconds)
    pub armed_duration_ms: u32,
}

/// Disarm method enumeration
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum DisarmMethod {
    Unknown,       // Unknown or unspecified method
    GcsCommand,    // MAV_CMD_COMPONENT_ARM_DISARM from GCS
    RcCommand,     // RC aux switch or stick pattern
    Failsafe,      // Failsafe system triggered automatic disarm
    ForceDisarm,   // Forced disarm bypassing checks (emergency)
}

/// Disarm reason enumeration
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum DisarmReason {
    Unknown,          // Unknown or unspecified reason
    Manual,           // Operator-commanded disarm (normal operation)
    Auto,             // Automatic disarm (mission complete, land complete)
    BatteryFailsafe,  // Battery voltage/capacity failsafe triggered
    RcLoss,           // RC signal loss failsafe
    GcsLoss,          // GCS heartbeat loss failsafe
    Crash,            // Crash detection triggered
    EmergencyStop,    // Emergency stop activated
}
```

**Post-Disarm Cleanup Sequence:**

```rust
impl SystemState {
    /// Post-disarm cleanup sequence
    fn post_disarm_cleanup(&mut self, method: DisarmMethod, reason: DisarmReason)
                           -> Result<(), &'static str> {
        // 1. Calculate armed duration
        let disarm_time_ms = get_time_ms();
        let armed_duration_ms = if let Some(arm_time) = self.post_arm_state.as_ref() {
            disarm_time_ms.wrapping_sub(arm_time.arm_time_ms)
        } else {
            0
        };

        // 2. Store disarm context
        self.post_disarm_state = Some(PostDisarmState {
            disarm_time_ms,
            disarm_method: method,
            disarm_reason: reason,
            armed_duration_ms,
        });

        // 3. Notify subsystems of disarm
        self.notify_subsystems_disarmed(method, reason)?;

        // 4. Log disarm event (immediately after notification)
        if let Err(e) = self.log_disarm_event(disarm_time_ms, method, reason, armed_duration_ms) {
            // Log failure: warn operator but continue disarm
            self.send_warning("Disarm event logging failed")?;
            warn!("Disarm logging error: {}", e);
            // Do NOT return error - logging failure is not critical
        }

        // 5. Continue with other post-disarm steps...
        Ok(())
    }

    /// Log disarm event to persistent storage
    fn log_disarm_event(&mut self, timestamp: u32, method: DisarmMethod,
                        reason: DisarmReason, duration: u32) -> Result<(), &'static str> {
        let log_entry = format!(
            "DISARM,{},{:?},{:?},{}",
            timestamp,
            method,
            reason,
            duration
        );

        self.logger.write(&log_entry)?;
        info!("Disarm event logged: {}", log_entry);
        Ok(())
    }
}
```

**API Design:**

```rust
impl SystemState {
    /// Get last disarm method
    /// Returns None if never disarmed or currently armed
    pub fn last_disarm_method(&self) -> Option<DisarmMethod> {
        self.post_disarm_state.as_ref().map(|s| s.disarm_method)
    }

    /// Get last disarm reason
    /// Returns None if never disarmed or currently armed
    pub fn last_disarm_reason(&self) -> Option<DisarmReason> {
        self.post_disarm_state.as_ref().map(|s| s.disarm_reason)
    }

    /// Get last armed duration in milliseconds
    /// Returns None if never disarmed
    pub fn last_armed_duration_ms(&self) -> Option<u32> {
        self.post_disarm_state.as_ref().map(|s| s.armed_duration_ms)
    }

    /// Get last armed duration in seconds (convenience method)
    pub fn last_armed_duration_s(&self) -> Option<f32> {
        self.last_armed_duration_ms().map(|ms| ms as f32 / 1000.0)
    }
}
```

**Log Entry Format:**

```
DISARM,timestamp_ms,method,reason,duration_ms
```

Example log entries:

```
DISARM,123456,GcsCommand,Manual,30500
DISARM,156789,Failsafe,BatteryFailsafe,42300
DISARM,189012,RcCommand,EmergencyStop,15200
DISARM,221345,ForceDisarm,Crash,5400
```

Fields:

1. **Event Type**: "DISARM" (literal string)
2. **Timestamp**: disarm_time_ms (u32, milliseconds since boot)
3. **Disarm Method**: DisarmMethod enum as debug string
4. **Disarm Reason**: DisarmReason enum as debug string
5. **Armed Duration**: armed_duration_ms (u32, milliseconds)

**Method + Reason Combinations:**

| Method      | Typical Reasons                             | Example Use Case                          |
| ----------- | ------------------------------------------- | ----------------------------------------- |
| GcsCommand  | Manual, Auto                                | Operator disarms via Mission Planner      |
| RcCommand   | Manual, EmergencyStop                       | Pilot disarms via RC switch               |
| Failsafe    | BatteryFailsafe, RcLoss, GcsLoss, Crash     | Automatic disarm triggered by failsafe    |
| ForceDisarm | EmergencyStop, any reason (bypasses checks) | Emergency forced disarm via MAVLink param |

**Disarm Method Determination:**

**1. GCS Command (MAVLink):**

```rust
fn handle_arm_disarm(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
    let should_arm = cmd.param1 > 0.5;
    let force = cmd.param2 > 0.5;  // param2=21196 for force

    if !should_arm {
        let method = if force {
            DisarmMethod::ForceDisarm
        } else {
            DisarmMethod::GcsCommand
        };

        match self.state.disarm(method, DisarmReason::Manual) {
            Ok(()) => MavResult::MAV_RESULT_ACCEPTED,
            Err(_) => MavResult::MAV_RESULT_DENIED,
        }
    }
}
```

**2. RC Command (Aux Switch):**

```rust
fn handle_rc_arm_switch(&mut self, switch_state: RcSwitchState) {
    match switch_state {
        RcSwitchState::Low => {
            let _ = self.state.disarm(DisarmMethod::RcCommand, DisarmReason::Manual);
        }
        _ => {}
    }
}
```

**3. Failsafe (Automatic):**

```rust
fn handle_battery_failsafe(&mut self) {
    // Battery failsafe triggered
    let _ = self.state.disarm(DisarmMethod::Failsafe, DisarmReason::BatteryFailsafe);
}

fn handle_rc_loss_failsafe(&mut self) {
    // RC loss failsafe triggered
    let _ = self.state.disarm(DisarmMethod::Failsafe, DisarmReason::RcLoss);
}
```

**4. Emergency Stop:**

```rust
fn handle_emergency_stop(&mut self) {
    // Emergency stop activated
    let _ = self.state.disarm(DisarmMethod::RcCommand, DisarmReason::EmergencyStop);
}
```

**Storage and Retrieval:**

Same as arm event logging:

- **Flash memory**: LittleFS with wear leveling
- **SD card**: If available
- **Retrieval**: USB serial, MAVLink LOG_REQUEST, direct SD card access

## Platform Considerations

### Pico W (RP2040)

- Same flash storage considerations as arm event logging
- 2 MB flash with LittleFS
- Logs retrieved via USB serial

### Pico 2 W (RP2350)

- 4 MB flash, same considerations
- Optional SD card support

### Cross-Platform

- Timestamp source abstracted via `get_time_ms()`
- Log format and API platform-agnostic
- Storage backend abstracted via logging subsystem trait

## Risks & Mitigation

| Risk                                                    | Impact | Likelihood | Mitigation                                                      | Validation                                          |
| ------------------------------------------------------- | ------ | ---------- | --------------------------------------------------------------- | --------------------------------------------------- |
| Incorrect method provided by caller                     | Medium | Medium     | Document method taxonomy, code review, validation in disarm()   | Test each disarm path with expected method          |
| Method + reason inconsistency (e.g., Manual + Failsafe) | Medium | Medium     | Document valid combinations, validation logic in disarm()       | Test various method/reason combinations             |
| Method taxonomy incomplete (missing trigger types)      | Low    | Low        | Review ArduPilot taxonomy, extend as needed                     | Compare with ArduPilot disarm methods               |
| Reason taxonomy incomplete (missing failure modes)      | Low    | Low        | Review ArduPilot taxonomy, extend as needed                     | Compare with ArduPilot disarm reasons               |
| Armed duration calculation overflow                     | Low    | Very Low   | Use wrapping_sub() for timestamp delta, document 49-day limit   | Test: verify duration correct after timestamp wrap  |
| Log write blocks disarm operation (latency)             | Medium | Low        | Non-blocking write (async or timeout), warn on failure          | Test: simulate slow flash, verify disarm completes  |
| Storage full, log write fails silently                  | Medium | Medium     | Check storage space periodically, warn operator, log rotation   | Test: fill storage, verify warning sent             |
| Method/reason not provided (caller forgets)             | Medium | Medium     | Make both required parameters, no default value (compile error) | Code review, ensure all disarm() calls specify both |

## Implementation Notes

**Preferred approaches:**

- **Required parameters**: Both method and reason required, cannot disarm without providing
- **Enum for type safety**: Use enums (not strings/ints) for compile-time checking
- **Non-blocking write**: Logging failure must NOT prevent disarm
- **Follow ArduPilot**: Use ArduPilot's method/reason taxonomy (proven in production)
- **Simple format**: CSV-like format, easy to parse
- **Document combinations**: Document valid method + reason combinations in ADR
- **Validate consistency**: Check method + reason combinations for logical consistency

**Known pitfalls:**

- **Unknown method/reason**: Always provide explicit values, never default to Unknown
- **Confusing method/reason**: Method is HOW, reason is WHY - they are complementary
- **ForceDisarm casual usage**: ForceDisarm should be rare (emergency only), log prominently
- **Blocking writes**: Do NOT wait indefinitely for flash write, use timeout or async
- **Silent failures**: Always warn operator if logging fails
- **Format drift**: Document log format in ADR, version it, maintain backward compatibility

**Related code areas:**

- `src/communication/mavlink/state.rs` - SystemState::disarm() and post_disarm_cleanup()
- `src/communication/mavlink/handlers/command.rs` - GCS-triggered disarms
- `src/vehicle/arming/types.rs` - DisarmMethod and DisarmReason enum definitions
- `src/vehicle/failsafe/actions.rs` - Failsafe-triggered disarms
- `src/platform/rc/` - RC-triggered disarms
- `src/core/logging/` - Logging subsystem
- `src/platform/rp2040/storage.rs` - Platform-specific flash storage

**Suggested patterns:**

- Enum for type safety and exhaustive matching
- Builder pattern for disarm context (method, reason, timestamp)
- Validation function to check method + reason consistency

## External References

- Analysis: [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
- ArduPilot AP_Arming Disarm Methods: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Arming/AP_Arming.cpp>
- MAVLink MAV_CMD_COMPONENT_ARM_DISARM: <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>
- ArduPilot Logging: <https://ardupilot.org/copter/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html>
