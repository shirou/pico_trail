# NFR-qaev2 Disarm Attempt Logging Completeness

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-dqzc6-pre-disarm-validation](../analysis/AN-dqzc6-pre-disarm-validation.md)
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

All disarm attempts shall be logged regardless of outcome (success, denied, forced) with timestamp, method, validation result, and current state to support post-incident analysis and debugging.

## Rationale

Disarm attempt logging provides critical audit trail for safety analysis, debugging, and compliance verification. Both successful and denied disarm attempts reveal important operational insights:

- **Post-Incident Analysis**: Understand sequence of events leading to incident (why disarm denied, when forced disarm used)
- **Safety Compliance**: Demonstrate validation system working correctly (denied unsafe disarms, allowed safe disarms)
- **Debugging Support**: Troubleshoot validation failures during testing and field operations
- **Operator Training**: Identify common operator errors (disarming with throttle active, disarming while moving)
- **System Health**: Detect anomalies (repeated failed disarm attempts, unexpected forced disarms)
- **Industry Standard**: ArduPilot logs all arm/disarm attempts with context for safety audit trail

Complete logging of all disarm attempts (not just successful disarms) enables comprehensive safety analysis and operational improvement.

## User Story (if applicable)

The system shall log all disarm attempts (successful, denied, forced) with timestamp, method, validation result, and current vehicle state to enable post-incident analysis, debugging, and safety compliance verification.

## Acceptance Criteria

- [ ] Every disarm attempt (successful or denied) generates a log entry before returning to caller
- [ ] Log entry includes: timestamp (milliseconds since boot), disarm method, validation result, current mode
- [ ] Log entry includes: throttle level (percent), velocity (m/s), armed duration (seconds)
- [ ] Successful disarm logged with validation result "Allowed"
- [ ] Denied disarm logged with specific denial reason (throttle active, velocity high, unsafe mode, already disarmed)
- [ ] Forced disarm logged with forced flag and original validation result (what validation would have denied)
- [ ] Logging is non-blocking (does not delay disarm response to operator)
- [ ] Log storage failure does not prevent disarm operation (log best-effort, disarm always completes)
- [ ] Logs retrievable via MAVLink LOG_REQUEST_DATA or serial download for analysis

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Reliability / Auditability:**

- **Completeness**: Every disarm attempt logged (100% coverage)
- **Timeliness**: Log written before returning result to caller
- **Persistence**: Logs stored in non-volatile storage (flash) for post-flight analysis
- **Non-Blocking**: Log write does not block disarm response (async or fire-and-forget)
- **Best-Effort**: Log storage failure does not prevent disarm (log warning, continue)

**Log Entry Format:**

```rust
/// Disarm attempt log entry
#[derive(Clone, Copy, Debug)]
pub struct DisarmAttemptLog {
    /// Timestamp (milliseconds since boot)
    pub timestamp_ms: u32,

    /// Disarm method (GCS, RC, failsafe, forced)
    pub method: DisarmMethod,

    /// Validation result (allowed or denial reason)
    pub validation_result: DisarmValidationResult,

    /// Whether disarm was forced (bypassed validation)
    pub forced: bool,

    /// Outcome (success or failure)
    pub outcome: DisarmOutcome,

    /// Current vehicle mode at time of attempt
    pub mode: VehicleMode,

    /// Throttle level (0-100 percent)
    pub throttle_percent: u8,

    /// Velocity (m/s, 0.1 resolution)
    pub velocity_cmps: u16, // cm/s for compact storage

    /// Armed duration (seconds since arm, 0 if not armed)
    pub armed_duration_sec: u16,
}

/// Disarm attempt outcome
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum DisarmOutcome {
    Success,       // Disarm succeeded
    Denied,        // Disarm denied by validation
}
```

**Logging Implementation:**

```rust
/// Log disarm attempt (both successful and denied)
fn log_disarm_attempt(
    &mut self,
    method: DisarmMethod,
    validation_result: DisarmValidationResult,
    forced: bool,
    outcome: DisarmOutcome
) -> Result<(), &'static str> {
    let timestamp_ms = get_time_ms();
    let armed_duration_sec = if self.is_armed() {
        (self.time_since_arm_ms().unwrap_or(0) / 1000) as u16
    } else {
        0
    };

    let log_entry = DisarmAttemptLog {
        timestamp_ms,
        method,
        validation_result,
        forced,
        outcome,
        mode: self.mode,
        throttle_percent: (self.get_throttle_normalized() * 100.0) as u8,
        velocity_cmps: (self.get_velocity_mps() * 100.0) as u16, // Convert to cm/s
        armed_duration_sec,
    };

    // Write to log storage (non-blocking, best-effort)
    match self.logger.log_disarm_attempt(&log_entry) {
        Ok(()) => {
            defmt::debug!("Disarm attempt logged: {:?}", log_entry);
        }
        Err(e) => {
            // Log storage failure - warn but don't fail disarm
            defmt::warn!("Failed to log disarm attempt: {}", e);
        }
    }

    Ok(())
}

/// Disarm with comprehensive logging
pub fn disarm(&mut self, method: DisarmMethod, forced: bool)
              -> Result<(), &'static str> {
    // Validate disarm conditions (unless forced)
    let validation_result = if forced {
        DisarmValidationResult::Allowed
    } else {
        self.validate_disarm(method)
    };

    // Determine outcome
    let outcome = if validation_result == DisarmValidationResult::Allowed || forced {
        DisarmOutcome::Success
    } else {
        DisarmOutcome::Denied
    };

    // Log attempt BEFORE returning (ensures both success and denial logged)
    self.log_disarm_attempt(method, validation_result, forced, outcome)?;

    // Handle result
    if outcome == DisarmOutcome::Denied {
        return Err(validation_result.to_error_message());
    }

    // Proceed with disarm
    self.armed = ArmedState::Disarmed;
    self.post_disarm_cleanup(method, forced)?;

    Ok(())
}
```

**Log Entry Examples:**

**Successful Disarm:**

```
DisarmAttemptLog {
    timestamp_ms: 120456,
    method: GcsCommand,
    validation_result: Allowed,
    forced: false,
    outcome: Success,
    mode: Hold,
    throttle_percent: 0,
    velocity_cmps: 0,
    armed_duration_sec: 45,
}
```

**Denied Disarm (Throttle Active):**

```
DisarmAttemptLog {
    timestamp_ms: 135890,
    method: RcSwitch,
    validation_result: DeniedThrottleActive,
    forced: false,
    outcome: Denied,
    mode: Manual,
    throttle_percent: 25,
    velocity_cmps: 150, // 1.5 m/s
    armed_duration_sec: 60,
}
```

**Forced Disarm (Emergency):**

```
DisarmAttemptLog {
    timestamp_ms: 156234,
    method: ForceDisarm,
    validation_result: Allowed, // Validation bypassed
    forced: true,
    outcome: Success,
    mode: Manual,
    throttle_percent: 50, // Still had throttle active!
    velocity_cmps: 300, // 3.0 m/s - moving fast!
    armed_duration_sec: 75,
}
```

**Log Analysis Examples:**

**Detect repeated denied disarms (operator error):**

```rust
fn analyze_disarm_logs(logs: &[DisarmAttemptLog]) {
    let denied_count = logs.iter()
        .filter(|log| log.outcome == DisarmOutcome::Denied)
        .count();

    if denied_count > 5 {
        println!("WARNING: {} denied disarm attempts detected", denied_count);
        println!("Common causes:");

        let throttle_denials = logs.iter()
            .filter(|log| log.validation_result == DisarmValidationResult::DeniedThrottleActive)
            .count();
        if throttle_denials > 0 {
            println!("  - Throttle active: {} times", throttle_denials);
        }

        let velocity_denials = logs.iter()
            .filter(|log| log.validation_result == DisarmValidationResult::DeniedVelocityTooHigh)
            .count();
        if velocity_denials > 0 {
            println!("  - Vehicle moving: {} times", velocity_denials);
        }
    }
}
```

**Detect unexpected forced disarms:**

```rust
fn detect_forced_disarms(logs: &[DisarmAttemptLog]) {
    let forced_disarms: Vec<&DisarmAttemptLog> = logs.iter()
        .filter(|log| log.forced)
        .collect();

    if !forced_disarms.is_empty() {
        println!("ALERT: {} forced disarm(s) detected:", forced_disarms.len());
        for log in forced_disarms {
            println!("  - Time: {}ms, Throttle: {}%, Velocity: {}m/s",
                     log.timestamp_ms,
                     log.throttle_percent,
                     log.velocity_cmps / 100);
        }
    }
}
```

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                                          | Impact | Likelihood | Mitigation                                                      | Validation                                         |
| --------------------------------------------- | ------ | ---------- | --------------------------------------------------------------- | -------------------------------------------------- |
| Log storage full (cannot write log entry)     | Medium | Medium     | Log best-effort, warn but don't fail disarm, rotate old logs    | Test log storage limits, implement rotation        |
| Logging delays disarm response                | High   | Low        | Use async logging or fire-and-forget, non-blocking write        | Measure logging latency, verify < 1ms overhead     |
| Log corruption or loss                        | Medium | Low        | Use checksummed log format, redundant storage if available      | Test log integrity after power loss                |
| Logs not retrievable (no download mechanism)  | High   | Medium     | Implement MAVLink LOG_REQUEST_DATA or serial log download       | Test log retrieval via GCS                         |
| Missing context in logs (insufficient detail) | Medium | Medium     | Include all relevant state (throttle, velocity, mode, duration) | Review log entries, verify sufficient for analysis |
| Excessive logging fills storage               | Medium | Medium     | Limit log retention (e.g., last 1000 entries), rotate old logs  | Monitor log storage usage, set retention policy    |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Disarm with comprehensive logging (both success and failure)
pub fn disarm(&mut self, method: DisarmMethod, forced: bool)
              -> Result<(), &'static str> {
    let start_time = timer.now_micros();

    // Validate disarm conditions (unless forced)
    let validation_result = if forced {
        DisarmValidationResult::Allowed
    } else {
        self.validate_disarm(method)
    };

    // Determine outcome before logging
    let outcome = if validation_result == DisarmValidationResult::Allowed || forced {
        DisarmOutcome::Success
    } else {
        DisarmOutcome::Denied
    };

    // Log attempt BEFORE returning (ensures all attempts logged)
    // Note: Log write is best-effort, does not fail disarm on error
    let _ = self.log_disarm_attempt(method, validation_result, forced, outcome);

    // Handle denied disarm
    if outcome == DisarmOutcome::Denied {
        // Send error to GCS
        self.send_status_text(
            MAV_SEVERITY::MAV_SEVERITY_WARNING,
            validation_result.to_error_message()
        )?;

        return Err(validation_result.to_error_message());
    }

    // Disarm allowed, proceed
    self.armed = ArmedState::Disarmed;

    // Execute post-disarm cleanup
    self.post_disarm_cleanup(method, forced)?;

    let total_duration = timer.now_micros() - start_time;
    defmt::info!("Disarm complete: {}us (outcome: {:?})", total_duration, outcome);

    Ok(())
}

/// Log disarm attempt with full context
fn log_disarm_attempt(
    &mut self,
    method: DisarmMethod,
    validation_result: DisarmValidationResult,
    forced: bool,
    outcome: DisarmOutcome
) -> Result<(), &'static str> {
    let timestamp_ms = get_time_ms();
    let armed_duration_sec = if self.is_armed() {
        (self.time_since_arm_ms().unwrap_or(0) / 1000) as u16
    } else {
        0
    };

    let log_entry = DisarmAttemptLog {
        timestamp_ms,
        method,
        validation_result,
        forced,
        outcome,
        mode: self.mode,
        throttle_percent: (self.get_throttle_normalized() * 100.0) as u8,
        velocity_cmps: (self.get_velocity_mps() * 100.0) as u16,
        armed_duration_sec,
    };

    // Write to persistent log storage (non-blocking)
    // If logging fails, warn but don't fail disarm
    match self.logger.log_disarm_attempt(&log_entry) {
        Ok(()) => {
            defmt::debug!(
                "Disarm attempt logged: outcome={:?}, method={:?}, validation={:?}",
                outcome, method, validation_result
            );
        }
        Err(e) => {
            defmt::warn!("Failed to log disarm attempt: {} (continuing anyway)", e);
        }
    }

    Ok(())
}
```

**Testing:**

```rust
#[test]
fn test_all_disarm_attempts_logged() {
    let mut state = SystemState::new();
    let mut logger = MockLogger::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Test 1: Successful disarm logs
    state.disarm(DisarmMethod::GcsCommand, false).unwrap();
    assert_eq!(logger.disarm_logs.len(), 1);
    assert_eq!(logger.disarm_logs[0].outcome, DisarmOutcome::Success);

    // Test 2: Denied disarm logs
    state.arm(ArmMethod::GcsCommand, true).unwrap();
    state.set_throttle_normalized(0.5); // 50% throttle
    let result = state.disarm(DisarmMethod::GcsCommand, false);
    assert!(result.is_err());
    assert_eq!(logger.disarm_logs.len(), 2);
    assert_eq!(logger.disarm_logs[1].outcome, DisarmOutcome::Denied);
    assert_eq!(logger.disarm_logs[1].validation_result,
               DisarmValidationResult::DeniedThrottleActive);

    // Test 3: Forced disarm logs (even with invalid conditions)
    let result = state.disarm(DisarmMethod::ForceDisarm, true);
    assert!(result.is_ok());
    assert_eq!(logger.disarm_logs.len(), 3);
    assert_eq!(logger.disarm_logs[2].outcome, DisarmOutcome::Success);
    assert_eq!(logger.disarm_logs[2].forced, true);
}

#[test]
fn test_log_storage_failure_does_not_prevent_disarm() {
    let mut state = SystemState::new();
    let mut logger = FailingLogger::new(); // Always fails

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Disarm should succeed even if logging fails
    let result = state.disarm(DisarmMethod::GcsCommand, false);
    assert!(result.is_ok(), "Disarm should succeed even if logging fails");
}
```

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState::disarm() and log_disarm_attempt()
- `src/core/logging/logger.rs` - Logger with log_disarm_attempt()
- `src/core/logging/types.rs` - DisarmAttemptLog structure
- `src/communication/mavlink/log_download.rs` - MAVLink log retrieval

## External References

- Analysis: [AN-m4dpl-post-arm-initialization](../analysis/AN-m4dpl-post-arm-initialization.md)
- MAVLink LOG_REQUEST_DATA: <https://mavlink.io/en/messages/common.html#LOG_REQUEST_DATA>
- MAVLink LOG_ENTRY: <https://mavlink.io/en/messages/common.html#LOG_ENTRY>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
