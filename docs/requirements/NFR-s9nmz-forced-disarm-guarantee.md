# NFR-s9nmz Forced Disarm Guaranteed Success

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

Forced disarm shall always succeed even if validation fails, bypassing all safety checks to ensure guaranteed vehicle shutdown capability in emergency situations.

## Rationale

Emergency scenarios require absolute guarantee of disarm capability regardless of vehicle state. Forced disarm is the ultimate safety override when normal validation prevents necessary shutdown:

- **Emergency Shutdown**: Critical situations require immediate motor stop (fire, collision, runaway throttle)
- **Failsafe Reliability**: Failsafe system must guarantee disarm even if sensors malfunction or validation fails
- **Operator Override**: GCS operator must have ultimate control to stop vehicle in any condition
- **Safety Priority**: Forcing vehicle to remain armed (due to validation failure) is more dangerous than forced disarm
- **Industry Standard**: ArduPilot forced disarm (MAV_CMD_COMPONENT_ARM_DISARM param2=21196) always succeeds

Forced disarm provides critical safety escape hatch when normal validation would dangerously prevent shutdown. Validation denial must never prevent emergency disarm.

## User Story (if applicable)

The system shall guarantee forced disarm always succeeds regardless of validation result or vehicle state to ensure operators can stop the vehicle in emergency situations when normal disarm is denied.

## Acceptance Criteria

- [ ] Forced disarm always returns success, never fails or returns error
- [ ] Forced disarm bypasses all validation checks (throttle, velocity, mode restrictions)
- [ ] Forced disarm transitions vehicle to disarmed state immediately
- [ ] Forced disarm executes post-disarm cleanup (actuator safety, logging) normally
- [ ] Forced disarm logged with forced flag and original validation result (what would have denied)
- [ ] MAVLink MAV_CMD_COMPONENT_ARM_DISARM with param2=21196 triggers forced disarm
- [ ] Forced disarm completes within 150ms (validation bypass + post-disarm cleanup)
- [ ] Forced disarm functional regardless of sensor failures or invalid state

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Safety / Reliability:**

- **Guaranteed Success**: Forced disarm always succeeds (100% success rate)
- **Validation Bypass**: All pre-disarm checks skipped (throttle, velocity, mode, armed state)
- **Emergency Priority**: Safety escape hatch when normal validation would trap vehicle in armed state
- **Fail-Safe**: Works even with sensor failures, invalid state, corrupted data
- **Deterministic**: Always completes successfully, no conditional failures

**Forced Disarm Logic:**

```rust
/// Disarm with optional forced flag
pub fn disarm(&mut self, method: DisarmMethod, forced: bool)
              -> Result<(), &'static str> {
    // Fast path: forced disarm bypasses validation (< 1us)
    if forced {
        defmt::warn!("FORCED DISARM: Bypassing all validation checks");

        // Log what validation would have denied (for audit trail)
        let validation_result = self.validate_disarm(method);
        if validation_result != DisarmValidationResult::Allowed {
            defmt::warn!(
                "FORCED DISARM: Would have denied by validation: {:?}",
                validation_result
            );
        }

        // Log forced disarm attempt
        let _ = self.log_disarm_attempt(
            method,
            validation_result, // What validation would have returned
            true,              // Forced flag
            DisarmOutcome::Success
        );

        // ALWAYS succeed: Set disarmed state
        self.armed = ArmedState::Disarmed;

        // Execute post-disarm cleanup (actuator safety, etc.)
        self.post_disarm_cleanup(method, true)?;

        defmt::info!("FORCED DISARM: Complete (validation bypassed)");
        return Ok(());
    }

    // Normal path: validate disarm conditions
    let validation_result = self.validate_disarm(method);

    if validation_result != DisarmValidationResult::Allowed {
        // Validation denied, log and return error
        let _ = self.log_disarm_attempt(
            method,
            validation_result,
            false,
            DisarmOutcome::Denied
        );
        return Err(validation_result.to_error_message());
    }

    // Validation passed, proceed with disarm
    let _ = self.log_disarm_attempt(
        method,
        validation_result,
        false,
        DisarmOutcome::Success
    );

    self.armed = ArmedState::Disarmed;
    self.post_disarm_cleanup(method, false)?;

    Ok(())
}
```

**MAVLink Integration:**

```rust
/// Handle MAV_CMD_COMPONENT_ARM_DISARM command
fn handle_arm_disarm(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
    let should_arm = cmd.param1 > 0.5;
    let force = cmd.param2 == 21196.0;  // Magic value for force

    if !should_arm {
        // Disarm
        let method = DisarmMethod::GcsCommand;

        match self.state.disarm(method, force) {
            Ok(()) => {
                if force {
                    defmt::warn!("Vehicle FORCE disarmed via GCS");
                    // Send warning to GCS about forced disarm
                    self.send_status_text(
                        MAV_SEVERITY::MAV_SEVERITY_WARNING,
                        "FORCED DISARM: All checks bypassed"
                    )?;
                } else {
                    defmt::info!("Vehicle disarmed via GCS");
                }
                MavResult::MAV_RESULT_ACCEPTED
            }
            Err(error_msg) => {
                // This should NEVER happen for forced disarm
                if force {
                    defmt::error!("CRITICAL: Forced disarm failed: {}", error_msg);
                    // This is a critical bug - forced disarm must always succeed
                }
                self.send_status_text(MAV_SEVERITY::MAV_SEVERITY_WARNING, error_msg)?;
                MavResult::MAV_RESULT_DENIED
            }
        }
    }
}
```

**Forced Disarm Scenarios:**

| Scenario                                 | Normal Disarm         | Forced Disarm  | Notes                                          |
| ---------------------------------------- | --------------------- | -------------- | ---------------------------------------------- |
| Throttle at 0%, vehicle stopped          | Success               | Success        | Normal conditions, both succeed                |
| Throttle at 50%, vehicle moving 5 m/s    | DENIED (throttle)     | Success        | Forced disarm bypasses validation              |
| Mode doesn't allow RC disarm             | DENIED (mode)         | Success        | Forced disarm bypasses mode restrictions       |
| Already disarmed                         | DENIED (redundant)    | Success        | Forced disarm always sets state                |
| Sensor failure (invalid throttle/vel)    | DENIED (invalid)      | Success        | Forced disarm works with corrupt data          |
| Flash logging full (cannot log)          | Success (best-effort) | Success        | Both succeed, logging best-effort              |
| Post-disarm cleanup fails (actuator I/O) | Success (warn)        | Success (warn) | Cleanup failure logged but doesn't fail disarm |

**Guaranteed Success Implementation:**

```rust
/// Forced disarm ALWAYS succeeds (no error return in forced path)
pub fn disarm(&mut self, method: DisarmMethod, forced: bool)
              -> Result<(), &'static str> {
    if forced {
        // Forced disarm: ALWAYS succeed, no validation, no conditional failures

        // Log for audit trail (best-effort, failure doesn't prevent disarm)
        let validation_result = self.validate_disarm(method);
        let _ = self.log_disarm_attempt(method, validation_result, true, DisarmOutcome::Success);

        // Set disarmed state (cannot fail)
        self.armed = ArmedState::Disarmed;

        // Execute cleanup (failure logged but doesn't fail disarm)
        if let Err(e) = self.post_disarm_cleanup(method, true) {
            defmt::warn!("Post-disarm cleanup failed (forced disarm succeeded anyway): {}", e);
        }

        return Ok(()); // ALWAYS return Ok() for forced disarm
    }

    // Normal disarm: validation can deny
    // ...
}
```

**Failsafe Integration:**

```rust
/// Failsafe disarm (forced)
fn execute_failsafe_disarm(&mut self, reason: FailsafeReason) {
    // Failsafe disarm is always forced (must succeed)
    match self.state.disarm(DisarmMethod::Failsafe, true) {
        Ok(()) => {
            defmt::warn!("Failsafe disarm: success (reason: {:?})", reason);
        }
        Err(e) => {
            // This should NEVER happen (forced disarm always succeeds)
            defmt::error!("CRITICAL BUG: Failsafe disarm failed: {}", e);
            // Vehicle still armed - critical safety failure
        }
    }
}
```

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                                                     | Impact   | Likelihood | Mitigation                                                    | Validation                                         |
| -------------------------------------------------------- | -------- | ---------- | ------------------------------------------------------------- | -------------------------------------------------- |
| Forced disarm fails in edge case (critical bug)          | CRITICAL | Low        | Thorough testing, forced path has NO conditional failures     | Test all edge cases, forced disarm always Ok()     |
| Forced disarm abused by operator (bypasses safety)       | Medium   | Low        | Log forced disarms prominently, warn operator in GCS          | Monitor forced disarm usage, alert on repeated use |
| Post-disarm cleanup failure prevents forced disarm       | CRITICAL | Low        | Cleanup errors logged but don't fail forced disarm            | Verify cleanup error doesn't propagate to caller   |
| Logging failure prevents forced disarm                   | CRITICAL | Low        | Logging best-effort, failure doesn't prevent disarm           | Test with log storage full, verify disarm succeeds |
| Validation check accidentally runs in forced path        | High     | Low        | Forced path bypasses validation entirely (early return)       | Code review, verify forced flag checked first      |
| Forced disarm leaves vehicle in unsafe state (motors on) | High     | Low        | Post-disarm cleanup still runs (actuator safety verification) | Verify cleanup always executes for forced disarm   |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Disarm with guaranteed success for forced disarm
pub fn disarm(&mut self, method: DisarmMethod, forced: bool)
              -> Result<(), &'static str> {
    // Fast path: forced disarm ALWAYS succeeds
    if forced {
        defmt::warn!(
            "FORCED DISARM initiated (method: {:?}, bypassing all checks)",
            method
        );

        // Check what validation would have done (for logging only)
        let validation_result = self.validate_disarm(method);
        if validation_result != DisarmValidationResult::Allowed {
            defmt::warn!(
                "FORCED DISARM: Validation would have denied: {:?}",
                validation_result
            );
        }

        // Log forced disarm (best-effort, don't fail on error)
        let _ = self.log_disarm_attempt(
            method,
            validation_result,
            true, // forced
            DisarmOutcome::Success
        );

        // Set disarmed state (cannot fail)
        self.armed = ArmedState::Disarmed;

        // Execute post-disarm cleanup
        // Note: Cleanup errors logged but don't fail forced disarm
        if let Err(e) = self.post_disarm_cleanup(method, true) {
            defmt::warn!(
                "FORCED DISARM: Cleanup failed: {} (disarm still succeeded)",
                e
            );
        }

        defmt::info!("FORCED DISARM: Complete (validation bypassed)");

        // ALWAYS return success for forced disarm
        return Ok(());
    }

    // Normal disarm path: validation can deny
    let validation_result = self.validate_disarm(method);

    if validation_result != DisarmValidationResult::Allowed {
        let _ = self.log_disarm_attempt(method, validation_result, false, DisarmOutcome::Denied);
        return Err(validation_result.to_error_message());
    }

    // Validation passed
    let _ = self.log_disarm_attempt(method, validation_result, false, DisarmOutcome::Success);
    self.armed = ArmedState::Disarmed;
    self.post_disarm_cleanup(method, false)?;

    Ok(())
}
```

**Testing:**

```rust
#[test]
fn test_forced_disarm_always_succeeds() {
    let mut state = SystemState::new();

    // Test 1: Forced disarm with throttle active
    state.arm(ArmMethod::GcsCommand, true).unwrap();
    state.set_throttle_normalized(0.8); // 80% throttle
    let result = state.disarm(DisarmMethod::GcsCommand, true);
    assert!(result.is_ok(), "Forced disarm must succeed with active throttle");
    assert!(!state.is_armed());

    // Test 2: Forced disarm while moving
    state.arm(ArmMethod::GcsCommand, true).unwrap();
    state.set_velocity_mps(5.0); // 5 m/s
    let result = state.disarm(DisarmMethod::GcsCommand, true);
    assert!(result.is_ok(), "Forced disarm must succeed while moving");
    assert!(!state.is_armed());

    // Test 3: Forced disarm in unsafe mode
    state.arm(ArmMethod::GcsCommand, true).unwrap();
    state.set_mode(VehicleMode::Manual); // Mode that doesn't allow RC disarm
    let result = state.disarm(DisarmMethod::RcSwitch, true);
    assert!(result.is_ok(), "Forced disarm must succeed in unsafe mode");
    assert!(!state.is_armed());

    // Test 4: Forced disarm when already disarmed
    assert!(!state.is_armed());
    let result = state.disarm(DisarmMethod::GcsCommand, true);
    assert!(result.is_ok(), "Forced disarm must succeed even if already disarmed");
    assert!(!state.is_armed());
}

#[test]
fn test_forced_disarm_with_sensor_failures() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate sensor failures
    state.set_throttle_invalid(); // Invalid throttle reading
    state.set_velocity_invalid(); // Invalid velocity reading

    // Normal disarm would fail (invalid sensor data)
    let result = state.disarm(DisarmMethod::GcsCommand, false);
    assert!(result.is_err(), "Normal disarm should fail with invalid sensors");
    assert!(state.is_armed(), "Vehicle should remain armed");

    // Forced disarm must succeed despite sensor failures
    let result = state.disarm(DisarmMethod::ForceDisarm, true);
    assert!(result.is_ok(), "Forced disarm must succeed with sensor failures");
    assert!(!state.is_armed());
}

#[test]
fn test_forced_disarm_with_logging_failure() {
    let mut state = SystemState::new();
    let mut logger = FailingLogger::new(); // Always fails to write logs

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Forced disarm must succeed even if logging fails
    let result = state.disarm(DisarmMethod::ForceDisarm, true);
    assert!(result.is_ok(), "Forced disarm must succeed even if logging fails");
    assert!(!state.is_armed());
}

#[test]
fn test_forced_disarm_bypass_overhead() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Measure forced disarm time (validation bypass)
    let start = timer.now_micros();
    state.disarm(DisarmMethod::ForceDisarm, true).unwrap();
    let duration = timer.now_micros() - start;

    println!("Forced disarm duration: {}us", duration);

    // Should complete within 150ms (no validation overhead, only cleanup)
    assert!(duration < 150_000, "Forced disarm exceeded 150ms: {}us", duration);
}

#[test]
fn test_failsafe_disarm_always_forced() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();
    state.set_throttle_normalized(0.9); // 90% throttle

    // Failsafe disarm is always forced
    let result = state.disarm(DisarmMethod::Failsafe, true);
    assert!(result.is_ok(), "Failsafe disarm must always succeed");
    assert!(!state.is_armed());
}
```

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState::disarm() with forced flag
- `src/communication/mavlink/handlers/command.rs` - MAV_CMD_COMPONENT_ARM_DISARM handler
- `src/vehicle/failsafe/` - Failsafe system forced disarm
- `src/core/logging/logger.rs` - Best-effort logging (doesn't fail disarm)

**Design Principles:**

- **Forced disarm path has NO conditional failures** - Always returns Ok()
- **Validation bypassed entirely** - Early return before validation logic
- **Logging is best-effort** - Failure doesn't prevent disarm
- **Cleanup errors logged but don't fail** - Disarm still succeeds
- **Deterministic behavior** - Forced disarm always has same outcome (success)

## External References

- Analysis: [AN-dqzc6-pre-disarm-validation](../analysis/AN-dqzc6-pre-disarm-validation.md)
- MAVLink MAV_CMD_COMPONENT_ARM_DISARM: <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>
  - param2 = 21196 for forced arm/disarm
- ArduPilot AP_Arming forced disarm implementation

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
