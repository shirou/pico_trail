# NFR-9s8i3 Post-Arm Initialization Safety

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-m4dpl-post-arm-initialization](../analysis/AN-m4dpl-post-arm-initialization.md)
- Related Tasks:
  - [T-zmv9u-arming-system-implementation](../tasks/T-zmv9u-arming-system-implementation/README.md)

## Requirement Statement

Post-arm initialization errors shall not leave vehicle in unsafe state; if any initialization step fails, the system shall disarm the vehicle immediately and report the error to the operator to prevent partial initialization from creating hazardous conditions.

## Rationale

Partial post-arm initialization is more dangerous than no initialization:

- **Unsafe State**: Vehicle appears armed but actuators not initialized (unpredictable behavior)
- **Monitoring Failure**: Subsystems not notified (failsafe may not activate)
- **Audit Gap**: Logging failure means no record of arm event (investigation impossible)
- **Operator Confusion**: Vehicle shows armed but some systems not ready (unexpected responses)

Safety principle: **All-or-nothing initialization**. Either complete initialization succeeds (vehicle safe to operate) or initialization fails (vehicle disarms, error reported).

ArduPilot follows this pattern: if post-arm initialization fails (e.g., GPS initialization timeout, compass calibration failure), the system disarms and reports error to operator.

## User Story (if applicable)

The system shall ensure that post-arm initialization either completes fully (vehicle safe to operate) or fails completely (vehicle disarms, operator alerted), preventing partial initialization from creating unsafe operating conditions where the vehicle appears armed but critical systems are not ready.

## Acceptance Criteria

- [ ] If any post-arm init step fails, vehicle shall disarm immediately
- [ ] Operator shall receive error message (MAVLink STATUSTEXT) identifying failed step
- [ ] Error message severity shall be MAV_SEVERITY_ERROR (red text in GCS)
- [ ] Vehicle shall not remain armed with partial initialization (verified via state check)
- [ ] All initialization steps shall have error detection (no silent failures)
- [ ] Disarm-on-failure shall be tested for each init step (unit tests)
- [ ] Recovery: operator can re-arm after fixing issue (e.g., wait for actuator ready)

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Safety:**

- **Fail-Safe Behavior**: Disarm on any initialization error
- **Operator Awareness**: Clear error messages with specific failure details
- **State Consistency**: Never leave vehicle in "armed but not ready" state
- **Recovery Path**: Operator can diagnose and retry after fixing root cause

**Error Handling Strategy:**

```rust
/// Post-arm initialization with fail-safe error handling
fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                 -> Result<(), &'static str> {
    // 1. Record timestamp (should never fail)
    let timestamp_ms = get_time_ms();
    self.post_arm_state.arm_time_ms = timestamp_ms;
    self.post_arm_state.arm_method = method;
    self.post_arm_state.checks_performed = checks_performed;

    // 2. Log arm event (may fail if flash full)
    if let Err(e) = self.log_arm_event(method, checks_performed) {
        defmt::error!("Arm logging failed: {}", e);

        // Disarm and report error
        self.armed = ArmedState::Disarmed;
        self.send_error("Arm failed: logging error")?;
        return Err("Logging failed");
    }

    // 3. Initialize actuators (may fail if hardware not responding)
    if let Err(e) = self.initialize_actuators() {
        defmt::error!("Actuator init failed: {}", e);

        // Disarm and report error
        self.armed = ArmedState::Disarmed;
        self.send_error("Arm failed: actuator init error")?;
        return Err("Actuator initialization failed");
    }
    self.post_arm_state.actuators_initialized = true;

    // 4. Notify subsystems (should not fail, but handle gracefully)
    if let Err(e) = self.notify_subsystems() {
        defmt::error!("Subsystem notification failed: {}", e);

        // Disarm and report error
        self.armed = ArmedState::Disarmed;
        self.send_error("Arm failed: subsystem notification error")?;
        return Err("Subsystem notification failed");
    }
    self.post_arm_state.subsystems_notified = true;

    // 5. Update GPIO (should not fail)
    if let Err(e) = self.update_gpio(true) {
        defmt::warn!("GPIO update failed: {}", e);
        // Non-critical - continue arming
    }

    // 6. Send warnings if checks disabled
    if !checks_performed {
        self.send_warning("Arming checks disabled")?;
    }

    defmt::info!("Post-arm initialization complete");
    Ok(())
}
```

**Disarm Helper:**

```rust
/// Disarm vehicle with error reporting
fn disarm_with_error(&mut self, error_msg: &str) -> Result<(), &'static str> {
    // Set disarmed state
    self.armed = ArmedState::Disarmed;

    // Clear post-arm state
    self.post_arm_state = PostArmState::new();

    // Send error to operator
    self.send_error(error_msg)?;

    defmt::error!("Disarmed due to error: {}", error_msg);
    Ok(())
}

/// Send error message to operator (MAV_SEVERITY_ERROR)
fn send_error(&mut self, message: &str) -> Result<(), &'static str> {
    // Send MAVLink STATUSTEXT with ERROR severity
    self.mavlink_router.send_statustext(
        MavSeverity::MAV_SEVERITY_ERROR,
        message
    )?;

    defmt::error!("Error sent to GCS: {}", message);
    Ok(())
}
```

**Error Categories:**

| Init Step               | Failure Mode                    | Severity | Disarm Required | Recovery                        |
| ----------------------- | ------------------------------- | -------- | --------------- | ------------------------------- |
| Timestamp recording     | System clock not available      | CRITICAL | Yes             | Fix clock, reboot               |
| Arm event logging       | Flash full, write failed        | HIGH     | Yes             | Clear logs, retry               |
| Actuator initialization | I2C/SPI timeout, no response    | CRITICAL | Yes             | Check hardware, power cycle     |
| Subsystem notification  | Monitoring/failsafe unavailable | HIGH     | Yes             | Debug subsystem, restart        |
| GPIO update             | Pin unavailable                 | LOW      | No              | Ignore (non-critical indicator) |
| Warning generation      | MAVLink router unavailable      | LOW      | No              | Ignore (warning not critical)   |

**Rollback Logic:**

```rust
/// Post-arm initialization with rollback on failure
fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                 -> Result<(), &'static str> {
    // Save original state for rollback
    let original_state = self.armed;

    // Try each init step
    let result = (|| -> Result<(), &'static str> {
        self.record_timestamp()?;
        self.log_arm_event(method, checks_performed)?;
        self.initialize_actuators()?;
        self.notify_subsystems()?;
        self.update_gpio(true)?;

        if !checks_performed {
            self.send_warning("Arming checks disabled")?;
        }

        Ok(())
    })();

    // If any step failed, rollback
    if let Err(e) = result {
        defmt::error!("Post-arm init failed: {}, rolling back", e);

        // Restore original state (disarmed)
        self.armed = original_state;
        self.post_arm_state = PostArmState::new();

        // Notify operator
        self.send_error(&format!("Arm failed: {}", e))?;

        return Err(e);
    }

    Ok(())
}
```

**State Verification:**

```rust
/// Verify post-arm state is consistent
fn verify_post_arm_state(&self) -> Result<(), &'static str> {
    if self.is_armed() {
        // If armed, all init must be complete
        if !self.post_arm_state.actuators_initialized {
            return Err("Armed but actuators not initialized");
        }

        if !self.post_arm_state.subsystems_notified {
            return Err("Armed but subsystems not notified");
        }

        if self.post_arm_state.arm_time_ms == 0 {
            return Err("Armed but no arm timestamp");
        }
    }

    Ok(())
}
```

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz:

- I2C/SPI actuator initialization more likely to timeout (slower)
- Flash write may be slower and more prone to failure
- Error handling must be robust on this platform

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz:

- Faster communication reduces failure likelihood
- More headroom for error handling logic
- Same safety requirements apply

### Cross-Platform

Post-arm safety requirements must be met on both platforms. Error handling logic must be identical to ensure consistent operator experience.

## Risks & Mitigation

| Risk                                        | Impact   | Likelihood | Mitigation                                                    | Validation                              |
| ------------------------------------------- | -------- | ---------- | ------------------------------------------------------------- | --------------------------------------- |
| Partial init leaves vehicle in unsafe state | CRITICAL | Medium     | Disarm on any init failure, verify state consistency          | Test each failure mode, verify disarm   |
| Operator not notified of init failure       | HIGH     | Low        | Send MAVLink STATUSTEXT error, log to persistent storage      | Verify error message in GCS, check logs |
| Disarm fails after init error               | CRITICAL | Low        | Ensure disarm has no dependencies, cannot fail                | Test disarm under all error conditions  |
| Error message too generic (no diagnostic)   | Medium   | Medium     | Include specific error detail (e.g., "Actuator init timeout") | Review error messages for clarity       |
| Recovery path unclear to operator           | Medium   | Medium     | Document recovery procedures, include hints in error messages | Test recovery, update documentation     |
| Rollback leaves stale state                 | HIGH     | Low        | Reset all post-arm state fields on rollback, verify cleanup   | Test state after failed arm             |

## Implementation Notes

**Recommended Architecture:**

```rust
/// High-level arm function with error handling
pub fn arm(&mut self, method: ArmMethod, checks_performed: bool)
           -> Result<(), &'static str> {
    // Pre-arm validation
    if self.is_armed() {
        return Err("Already armed");
    }

    if self.battery.is_critical() {
        return Err("Battery voltage too low");
    }

    // Set armed state (optimistic)
    self.armed = ArmedState::Armed;

    // Execute post-arm initialization (may fail)
    if let Err(e) = self.post_arm_init(method, checks_performed) {
        defmt::error!("Post-arm init failed: {}", e);

        // Disarm and report error
        self.armed = ArmedState::Disarmed;
        self.post_arm_state = PostArmState::new();

        // Send detailed error to operator
        let error_msg = format!("Arm failed: {}", e);
        self.send_error(&error_msg)?;

        return Err(e);
    }

    // Verify state consistency
    if let Err(e) = self.verify_post_arm_state() {
        defmt::error!("Post-arm state verification failed: {}", e);

        // Disarm and report error
        self.armed = ArmedState::Disarmed;
        self.post_arm_state = PostArmState::new();

        self.send_error("Arm failed: state inconsistent")?;

        return Err(e);
    }

    defmt::info!("Vehicle armed successfully (method: {:?})", method);
    Ok(())
}
```

**Testing Strategy:**

```rust
#[test]
fn test_arm_disarms_on_logging_failure() {
    let mut state = SystemState::new();

    // Simulate logging failure
    state.logger.set_mode(LogMode::AlwaysFail);

    // Attempt arm
    let result = state.arm(ArmMethod::GcsCommand, true);

    // Verify arm failed
    assert!(result.is_err());
    assert_eq!(result.unwrap_err(), "Logging failed");

    // Verify vehicle is disarmed
    assert!(!state.is_armed());

    // Verify error message sent
    assert!(state.last_error_message().contains("Arm failed: logging error"));
}

#[test]
fn test_arm_disarms_on_actuator_failure() {
    let mut state = SystemState::new();

    // Simulate actuator initialization failure
    state.actuators.set_mode(ActuatorMode::InitFails);

    // Attempt arm
    let result = state.arm(ArmMethod::GcsCommand, true);

    // Verify arm failed
    assert!(result.is_err());
    assert_eq!(result.unwrap_err(), "Actuator initialization failed");

    // Verify vehicle is disarmed
    assert!(!state.is_armed());

    // Verify no partial initialization
    assert!(!state.post_arm_state.actuators_initialized);
    assert_eq!(state.post_arm_state.arm_time_ms, 0);
}

#[test]
fn test_arm_continues_on_non_critical_failure() {
    let mut state = SystemState::new();

    // Simulate GPIO failure (non-critical)
    state.gpio.set_mode(GpioMode::AlwaysFail);

    // Attempt arm
    let result = state.arm(ArmMethod::GcsCommand, true);

    // Verify arm succeeded despite GPIO failure
    assert!(result.is_ok());

    // Verify vehicle is armed
    assert!(state.is_armed());

    // Verify critical init steps completed
    assert!(state.post_arm_state.actuators_initialized);
    assert!(state.post_arm_state.subsystems_notified);
}

#[test]
fn test_recovery_after_init_failure() {
    let mut state = SystemState::new();

    // Simulate actuator failure
    state.actuators.set_mode(ActuatorMode::InitFails);

    // First arm attempt fails
    let result = state.arm(ArmMethod::GcsCommand, true);
    assert!(result.is_err());
    assert!(!state.is_armed());

    // Fix actuator issue
    state.actuators.set_mode(ActuatorMode::Normal);

    // Second arm attempt succeeds
    let result = state.arm(ArmMethod::GcsCommand, true);
    assert!(result.is_ok());
    assert!(state.is_armed());
}

#[test]
fn test_state_consistency_after_failure() {
    let mut state = SystemState::new();

    // Simulate subsystem notification failure
    state.monitoring.set_mode(MonitoringMode::NotificationFails);

    // Attempt arm
    let result = state.arm(ArmMethod::GcsCommand, true);
    assert!(result.is_err());

    // Verify state fully reset
    assert!(!state.is_armed());
    assert_eq!(state.post_arm_state.arm_time_ms, 0);
    assert_eq!(state.post_arm_state.arm_method, ArmMethod::Unknown);
    assert!(!state.post_arm_state.actuators_initialized);
    assert!(!state.post_arm_state.subsystems_notified);
}
```

**Error Message Guidelines:**

- **Be Specific**: Include which step failed (e.g., "Actuator init failed", not "Arm failed")
- **Suggest Action**: Include recovery hint if possible (e.g., "Check actuator power")
- **Use ERROR Severity**: Ensure red text in GCS (MAV_SEVERITY_ERROR)
- **Keep Concise**: < 50 characters (fits in MAVLink STATUSTEXT, 50-char limit)
- **Log Details**: Log full error context to persistent storage for debugging

**Example Error Messages:**

```rust
// Good: Specific with action hint
"Arm failed: Actuator timeout (check power)"

// Good: Specific with context
"Arm failed: Flash full (clear logs)"

// Good: Specific failure point
"Arm failed: Subsystem init error"

// Bad: Too generic
"Arm failed"

// Bad: No action hint
"Initialization error occurred"
```

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState::arm() and post_arm_init()
- `src/communication/mavlink/router.rs` - Error message sending
- `src/devices/actuators/` - Actuator initialization error handling

## External References

- Analysis: [AN-m4dpl-post-arm-initialization](../analysis/AN-m4dpl-post-arm-initialization.md)
- MAVLink STATUSTEXT: <https://mavlink.io/en/messages/common.html#STATUSTEXT>
- ArduPilot Arming Errors: <https://ardupilot.org/copter/docs/arming_the_motors.html#recognizing-failure-to-arm>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
