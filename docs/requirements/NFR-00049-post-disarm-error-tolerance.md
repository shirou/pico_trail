# NFR-00049 Post-Disarm Cleanup Error Tolerance

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: N/A - No prerequisite requirements
- Dependent Requirements: N/A - No dependent requirements
- Related Analysis:
  - [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
  - [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

Post-disarm cleanup errors shall not prevent disarming from completing, ensuring the vehicle transitions to disarmed state even if cleanup partially fails, with all failures logged and reported to the operator.

## Rationale

Disarm must succeed under all circumstances, even if post-disarm cleanup fails. Preventing disarm due to cleanup failure would leave the vehicle in armed state, which is more dangerous than incomplete cleanup:

- **Safety Priority**: Being disarmed is always safer than remaining armed, even with incomplete cleanup
- **Hardware Failure Tolerance**: Logging failure, actuator readback failure, or subsystem notification failure must not block disarm
- **Operator Control**: Operator must be able to disarm regardless of system state
- **Graceful Degradation**: System logs all failures but completes disarm in safest possible state

ArduPilot's post-disarm cleanup logs errors but never prevents disarm from completing. PX4's Commander state machine ensures disarm state transition always succeeds, with subsystem cleanup failures logged but not blocking.

This requirement ensures fail-safe behavior: disarm always succeeds, cleanup failures are reported to operator for investigation, and vehicle is left in the safest possible state (disarmed, even if cleanup incomplete).

## User Story (if applicable)

The system shall complete disarm operation even if post-disarm cleanup fails to ensure the vehicle transitions to disarmed state under all circumstances, with cleanup errors logged and reported to the operator for investigation and manual verification.

## Acceptance Criteria

- [ ] Disarm operation completes successfully even if logging fails
- [ ] Disarm operation completes successfully even if actuator verification fails or times out
- [ ] Disarm operation completes successfully even if subsystem notification fails
- [ ] Disarm operation completes successfully even if GPIO update fails
- [ ] Disarm operation completes successfully even if state reset fails
- [ ] All cleanup failures are logged to defmt for debugging
- [ ] All cleanup failures send MAVLink STATUSTEXT warning to operator
- [ ] Operator is warned to manually inspect vehicle if cleanup fails
- [ ] PostDisarmState tracks which cleanup steps completed successfully
- [ ] Test cases verify disarm succeeds with simulated cleanup failures

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Safety / Reliability:**

- **Non-Blocking**: Cleanup errors do not block disarm
- **Error Reporting**: All failures logged and reported to operator
- **Graceful Degradation**: Vehicle left in safest possible state
- **Operator Warning**: Clear guidance for manual verification if cleanup fails

**Error Handling Strategy:**

```rust
/// Post-disarm cleanup with error tolerance
fn post_disarm_cleanup(&mut self, method: DisarmMethod, reason: DisarmReason,
                       armed_duration_ms: u32) -> Result<(), &'static str> {
    let disarm_time_ms = get_time_ms();

    // Track cleanup status
    let mut cleanup_status = PostDisarmCleanupStatus {
        logging_ok: false,
        actuators_ok: false,
        subsystems_ok: false,
        gpio_ok: false,
        state_reset_ok: false,
    };

    // 1. Log disarm event (non-blocking)
    match self.log_disarm_event(disarm_time_ms, method, reason, armed_duration_ms) {
        Ok(()) => {
            defmt::info!("Disarm event logged successfully");
            cleanup_status.logging_ok = true;
        }
        Err(e) => {
            // Log error but continue
            defmt::error!("Disarm event logging failed: {}", e);
            self.send_statustext_best_effort("WARN: Disarm log failed");
            // DO NOT return error - continue cleanup
        }
    }

    // 2. Verify actuators safe (non-blocking, critical for safety)
    match self.verify_actuators_safe() {
        Ok(()) => {
            defmt::info!("Actuators verified safe");
            cleanup_status.actuators_ok = true;
        }
        Err(e) => {
            // Log error but continue
            defmt::error!("Actuator verify failed: {}", e);
            self.send_statustext_best_effort("ERROR: Actuator verify failed, inspect vehicle!");
            // DO NOT return error - continue cleanup
        }
    }

    // 3. Notify subsystems (non-blocking)
    match self.notify_subsystems_disarmed() {
        Ok(()) => {
            defmt::info!("Subsystems notified successfully");
            cleanup_status.subsystems_ok = true;
        }
        Err(e) => {
            // Log error but continue
            defmt::error!("Subsystem notification failed: {}", e);
            self.send_statustext_best_effort("WARN: Subsystem notify failed");
            // DO NOT return error - continue cleanup
        }
    }

    // 4. Update GPIO (non-blocking)
    match self.update_gpio(false) {
        Ok(()) => {
            defmt::info!("GPIO updated successfully");
            cleanup_status.gpio_ok = true;
        }
        Err(e) => {
            // Log error but continue
            defmt::error!("GPIO update failed: {}", e);
            // Minor failure, no STATUSTEXT warning
            // DO NOT return error - continue cleanup
        }
    }

    // 5. Reset armed state (non-blocking)
    match self.reset_armed_state() {
        Ok(()) => {
            defmt::info!("Armed state reset successfully");
            cleanup_status.state_reset_ok = true;
        }
        Err(e) => {
            // Log error but continue
            defmt::error!("State reset failed: {}", e);
            self.send_statustext_best_effort("WARN: State reset failed");
            // DO NOT return error - continue cleanup
        }
    }

    // 6. Check logging persistence (non-blocking)
    if self.should_persist_logging(reason) {
        match self.extend_logging_duration() {
            Ok(()) => defmt::info!("Logging extended for failsafe analysis"),
            Err(e) => {
                defmt::error!("Logging extension failed: {}", e);
                // Non-critical, do not warn operator
            }
        }
    }

    // Store cleanup state (success/failure status)
    self.post_disarm_state = PostDisarmState {
        disarm_time_ms,
        disarm_method: method,
        disarm_reason: reason,
        armed_duration_ms,
        actuators_safe: cleanup_status.actuators_ok,
        subsystems_notified: cleanup_status.subsystems_ok,
    };

    // Warn operator if critical cleanup failed
    if !cleanup_status.actuators_ok {
        self.send_statustext_best_effort("CRITICAL: Actuators NOT verified safe - inspect vehicle!");
    }

    // Log overall cleanup status
    if cleanup_status.all_ok() {
        defmt::info!("Post-disarm cleanup complete (all steps OK)");
    } else {
        defmt::warn!(
            "Post-disarm cleanup incomplete: log={}, act={}, sub={}, gpio={}, reset={}",
            cleanup_status.logging_ok,
            cleanup_status.actuators_ok,
            cleanup_status.subsystems_ok,
            cleanup_status.gpio_ok,
            cleanup_status.state_reset_ok
        );
    }

    // ALWAYS return Ok - disarm must succeed even if cleanup failed
    Ok(())
}

/// Cleanup status tracking
struct PostDisarmCleanupStatus {
    logging_ok: bool,
    actuators_ok: bool,
    subsystems_ok: bool,
    gpio_ok: bool,
    state_reset_ok: bool,
}

impl PostDisarmCleanupStatus {
    fn all_ok(&self) -> bool {
        self.logging_ok && self.actuators_ok && self.subsystems_ok &&
        self.gpio_ok && self.state_reset_ok
    }
}

/// Send STATUSTEXT with best-effort (never fails)
fn send_statustext_best_effort(&mut self, msg: &str) {
    match self.send_statustext(msg) {
        Ok(()) => {}
        Err(e) => {
            defmt::error!("STATUSTEXT send failed: {}", e);
            // Ignore error, best-effort only
        }
    }
}
```

**Error Classification:**

| Cleanup Step           | Criticality | Failure Action                             | Operator Warning |
| ---------------------- | ----------- | ------------------------------------------ | ---------------- |
| Disarm event logging   | Medium      | Log error, continue cleanup                | WARN             |
| Actuator safety verify | High        | Log error, continue cleanup, warn operator | ERROR (critical) |
| Subsystem notification | Medium      | Log error, continue cleanup                | WARN             |
| GPIO update            | Low         | Log error, continue cleanup                | No warning       |
| State reset            | Medium      | Log error, continue cleanup                | WARN             |
| Logging persistence    | Low         | Log error, continue cleanup                | No warning       |

**Testing Strategy:**

```rust
#[test]
fn test_disarm_succeeds_with_logging_failure() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate logging failure (full flash)
    state.logger.set_full(true);

    // Disarm should still succeed
    let result = state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal);
    assert!(result.is_ok(), "Disarm should succeed despite logging failure");

    // Verify disarmed state
    assert!(!state.is_armed());
}

#[test]
fn test_disarm_succeeds_with_actuator_verify_failure() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate actuator hardware failure
    state.actuators.set_failure_mode(true);

    // Disarm should still succeed
    let result = state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal);
    assert!(result.is_ok(), "Disarm should succeed despite actuator verify failure");

    // Verify disarmed state
    assert!(!state.is_armed());

    // Verify actuators_safe flag is false
    assert!(!state.post_disarm_state.actuators_safe);

    // Verify STATUSTEXT warning sent
    let statustext = state.mavlink.get_last_statustext().unwrap();
    assert!(statustext.contains("Actuators NOT verified safe"));
}

#[test]
fn test_disarm_succeeds_with_subsystem_notify_failure() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate subsystem failure
    state.subsystems.set_failure_mode(true);

    // Disarm should still succeed
    let result = state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal);
    assert!(result.is_ok(), "Disarm should succeed despite subsystem notify failure");

    // Verify disarmed state
    assert!(!state.is_armed());

    // Verify subsystems_notified flag is false
    assert!(!state.post_disarm_state.subsystems_notified);
}

#[test]
fn test_disarm_succeeds_with_multiple_failures() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate multiple failures
    state.logger.set_full(true);
    state.actuators.set_failure_mode(true);
    state.subsystems.set_failure_mode(true);

    // Disarm should still succeed
    let result = state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal);
    assert!(result.is_ok(), "Disarm should succeed despite multiple cleanup failures");

    // Verify disarmed state
    assert!(!state.is_armed());

    // Verify all flags are false
    assert!(!state.post_disarm_state.actuators_safe);
    assert!(!state.post_disarm_state.subsystems_notified);

    // Verify multiple STATUSTEXT warnings sent
    let statustexts = state.mavlink.get_all_statustexts();
    assert!(statustexts.len() >= 2, "Multiple warnings should be sent");
}

#[test]
fn test_cleanup_status_tracking() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate partial failure (actuators fail, others succeed)
    state.actuators.set_failure_mode(true);

    state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal).unwrap();

    // Verify cleanup status tracked correctly
    assert!(!state.post_disarm_state.actuators_safe, "Actuators should be marked unsafe");
    assert!(state.post_disarm_state.subsystems_notified, "Subsystems should be notified");
}
```

**Operator Guidance:**

When cleanup fails, operator receives clear guidance:

```
STATUSTEXT: "WARN: Disarm log failed"
→ Operator action: Note in flight log, check storage capacity

STATUSTEXT: "ERROR: Actuator verify failed, inspect vehicle!"
→ Operator action: Manually verify actuators are safe before approaching vehicle

STATUSTEXT: "CRITICAL: Actuators NOT verified safe - inspect vehicle!"
→ Operator action: DO NOT approach vehicle until manual verification complete

STATUSTEXT: "WARN: Subsystem notify failed"
→ Operator action: Check system status, may need reboot
```

## Platform Considerations

### Pico W (RP2040)

N/A - Platform agnostic

### Pico 2 W (RP2350)

N/A - Platform agnostic

### Cross-Platform

Error tolerance behavior must be identical on both platforms. All cleanup failures are non-blocking on both Pico W and Pico 2 W.

## Risks & Mitigation

| Risk                                               | Impact       | Likelihood | Mitigation                                                     | Validation                                       |
| -------------------------------------------------- | ------------ | ---------- | -------------------------------------------------------------- | ------------------------------------------------ |
| **Cleanup failure leaves vehicle in unsafe state** | **Critical** | **Low**    | **Disarm always succeeds, operator warned to inspect vehicle** | **Test all failure modes, verify warnings sent** |
| Operator ignores cleanup failure warnings          | High         | Medium     | Use CRITICAL severity for actuator failures, clear guidance    | Review warning messages with operators           |
| Multiple failures overwhelm operator               | Medium       | Low        | Prioritize warnings (critical first), combine non-critical     | Test with multiple simultaneous failures         |
| Silent failure (no warning sent)                   | High         | Low        | Best-effort STATUSTEXT, always log to defmt                    | Verify all failures logged and warned            |
| Cleanup failure causes cascading issues            | Medium       | Low        | Isolate failures, continue cleanup even if one step fails      | Test each failure mode independently             |
| Retry logic causes infinite loop                   | Medium       | Low        | No retry in cleanup (single attempt), timeout enforced         | Verify no retry loops in cleanup code            |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Post-disarm cleanup with full error tolerance
fn post_disarm_cleanup(&mut self, method: DisarmMethod, reason: DisarmReason,
                       armed_duration_ms: u32) -> Result<(), &'static str> {
    let disarm_time_ms = get_time_ms();

    // Track cleanup status (for diagnostics)
    let mut cleanup_status = PostDisarmCleanupStatus::new();

    // Execute all cleanup steps (best-effort)
    cleanup_status.logging_ok = self.log_disarm_event_best_effort(
        disarm_time_ms, method, reason, armed_duration_ms
    );

    cleanup_status.actuators_ok = self.verify_actuators_safe_best_effort();

    cleanup_status.subsystems_ok = self.notify_subsystems_disarmed_best_effort();

    cleanup_status.gpio_ok = self.update_gpio_best_effort(false);

    cleanup_status.state_reset_ok = self.reset_armed_state_best_effort();

    // Check logging persistence (best-effort)
    if self.should_persist_logging(reason) {
        let _ = self.extend_logging_duration(); // Ignore error
    }

    // Store cleanup state
    self.post_disarm_state = PostDisarmState {
        disarm_time_ms,
        disarm_method: method,
        disarm_reason: reason,
        armed_duration_ms,
        actuators_safe: cleanup_status.actuators_ok,
        subsystems_notified: cleanup_status.subsystems_ok,
    };

    // Warn operator if critical cleanup failed
    if !cleanup_status.actuators_ok {
        self.send_statustext_best_effort("CRITICAL: Actuators NOT verified safe - inspect vehicle!");
    } else if !cleanup_status.all_ok() {
        self.send_statustext_best_effort("WARN: Post-disarm cleanup incomplete");
    }

    // Log overall status
    defmt::info!(
        "Post-disarm cleanup: log={}, act={}, sub={}, gpio={}, reset={}",
        cleanup_status.logging_ok,
        cleanup_status.actuators_ok,
        cleanup_status.subsystems_ok,
        cleanup_status.gpio_ok,
        cleanup_status.state_reset_ok
    );

    // ALWAYS return Ok - disarm must succeed
    Ok(())
}

/// Best-effort cleanup step (never fails)
fn log_disarm_event_best_effort(&mut self, timestamp_ms: u32, method: DisarmMethod,
                                 reason: DisarmReason, duration_ms: u32) -> bool {
    match self.log_disarm_event(timestamp_ms, method, reason, duration_ms) {
        Ok(()) => {
            defmt::info!("Disarm event logged");
            true
        }
        Err(e) => {
            defmt::error!("Disarm event logging failed: {}", e);
            self.send_statustext_best_effort("WARN: Disarm log failed");
            false
        }
    }
}

/// Best-effort actuator verify (never fails, but reports failure)
fn verify_actuators_safe_best_effort(&mut self) -> bool {
    match self.verify_actuators_safe() {
        Ok(()) => {
            defmt::info!("Actuators verified safe");
            true
        }
        Err(e) => {
            defmt::error!("Actuator verify failed: {}", e);
            self.send_statustext_best_effort("ERROR: Actuator verify failed!");
            false
        }
    }
}

// Similar best-effort wrappers for other cleanup steps...
```

**Key Principles:**

1. **Never return error from post_disarm_cleanup()** - Always return Ok
2. **Log all failures to defmt** - For debugging and diagnostics
3. **Warn operator of critical failures** - Especially actuator verification
4. **Track cleanup status** - Store in PostDisarmState for visibility
5. **Best-effort STATUSTEXT** - Send warnings but don't fail if STATUSTEXT fails
6. **Single attempt only** - No retry logic that could block or loop
7. **Continue on error** - Execute all cleanup steps even if some fail

Related code areas:

- `src/communication/mavlink/state.rs` - post_disarm_cleanup() and error handling
- `src/core/logging/` - Log error handling
- `src/devices/actuators/` - Actuator error handling
- `src/communication/mavlink/message.rs` - STATUSTEXT best-effort sending

## External References

- Analysis: [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
  N/A - No external references
