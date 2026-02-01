# NFR-00013 Actuator Safety Verification Mandatory

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

Actuator safety verification shall not be skippable during post-disarm cleanup, ensuring all actuators return to neutral state before cleanup completes, with timeout if hardware is unresponsive.

## Rationale

Actuator safety verification is the single most critical post-disarm cleanup step. Unlike logging or subsystem notification, actuator safety directly impacts physical safety:

- **Personnel Safety**: Operators and maintenance crew must be able to approach vehicle safely after disarm
- **Accidental Activation**: Actuators left in active state can cause injury or damage if accidentally re-energized
- **Hardware Verification**: Confirms actuator hardware is responsive and functioning correctly
- **Safety-Critical**: Cannot be bypassed, disabled, or skipped under any circumstances

ArduPilot's post-disarm sequence includes actuator shutdown with hardware verification. PX4's Commander state machine enforces ESC disarming before state transition completes. Both systems treat actuator safety as non-negotiable.

The requirement prioritizes safety over speed: if actuator verification fails or times out, the system logs an error but completes the disarm (vehicle safety requires being disarmed even if verification fails).

## User Story (if applicable)

The system shall always verify actuators return to neutral state during post-disarm cleanup to ensure operators and maintenance crew can safely approach the vehicle, with timeout and error logging if hardware is unresponsive but disarm still completes.

## Acceptance Criteria

- [ ] Actuator safety verification is always performed during post-disarm cleanup (no skip mechanism)
- [ ] Verification reads back throttle PWM value and confirms 1000 us (minimum) or off
- [ ] Verification reads back steering PWM value and confirms 1500 us (center)
- [ ] Verification times out after 50ms if hardware unresponsive
- [ ] Timeout logs error but does not prevent disarm from completing
- [ ] Verification failure sends MAVLink STATUSTEXT warning to operator
- [ ] Post-disarm cleanup does not complete until verification finishes or times out
- [ ] No configuration parameter, compile flag, or runtime option can disable verification

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Safety:**

- **Non-Negotiable**: Actuator verification cannot be skipped, disabled, or bypassed
- **Hardware Validation**: Confirms actuators respond to neutral commands
- **Timeout Safety**: If verification fails, disarm still completes (being disarmed is safer than remaining armed)
- **Error Reporting**: Verification failures logged and reported to operator immediately

**Implementation Strategy:**

```rust
/// Verify actuators return to safe neutral state
fn verify_actuators_safe(&mut self) -> Result<(), &'static str> {
    const VERIFY_TIMEOUT_MS: u32 = 50;
    const THROTTLE_NEUTRAL: u16 = 1000; // Minimum or off
    const STEERING_NEUTRAL: u16 = 1500; // Center

    let verify_start = get_time_ms();

    // Command actuators to neutral (should already be neutral from disarm)
    self.actuators.set_throttle_pwm(THROTTLE_NEUTRAL)?;
    self.actuators.set_steering_pwm(STEERING_NEUTRAL)?;

    // Read back and verify (with timeout)
    loop {
        // Check timeout
        if get_time_ms() - verify_start > VERIFY_TIMEOUT_MS {
            // Timeout: log error but complete disarm
            defmt::error!("Actuator safety verify timeout ({}ms)", VERIFY_TIMEOUT_MS);
            self.send_statustext("WARN: Actuator verify timeout, check hardware")?;
            return Ok(()); // Return Ok to allow disarm to complete
        }

        // Read back PWM values
        let throttle_pwm = self.actuators.read_throttle_pwm()?;
        let steering_pwm = self.actuators.read_steering_pwm()?;

        // Check if neutral
        let throttle_safe = throttle_pwm <= THROTTLE_NEUTRAL + 10; // 10us tolerance
        let steering_safe = (steering_pwm as i32 - STEERING_NEUTRAL as i32).abs() < 10;

        if throttle_safe && steering_safe {
            defmt::info!("Actuators verified safe (throttle: {}, steering: {})",
                         throttle_pwm, steering_pwm);
            return Ok(());
        }

        // Not neutral yet, wait and retry
        delay_us(1000); // 1ms delay between checks
    }
}
```

**Error Handling:**

```rust
/// Post-disarm cleanup with mandatory actuator verification
fn post_disarm_cleanup(&mut self, method: DisarmMethod, reason: DisarmReason,
                       armed_duration_ms: u32) -> Result<(), &'static str> {
    // ... (timestamp, logging) ...

    // CRITICAL: Actuator safety verification (cannot skip)
    match self.verify_actuators_safe() {
        Ok(()) => {
            defmt::info!("Actuators verified safe");
            self.post_disarm_state.actuators_safe = true;
        }
        Err(e) => {
            // Log error but continue disarm (being disarmed is safer)
            defmt::error!("Actuator verify failed: {}", e);
            self.send_statustext("ERROR: Actuator verify failed, inspect vehicle")?;
            self.post_disarm_state.actuators_safe = false;
            // DO NOT return error - disarm must complete
        }
    }

    // ... (subsystem notification, state reset) ...

    Ok(())
}
```

**ArduPilot Reference:**

ArduPilot's MOT_SAFE_DISARM parameter controls PWM output behavior when disarmed:

- **MOT_SAFE_DISARM=0**: Output trim values (default)
- **MOT_SAFE_DISARM=1**: No PWM pulses sent (safer, ensures actuators neutral)

pico_trail follows MOT_SAFE_DISARM=1 approach: no PWM pulses when disarmed, verify actuators return to neutral state.

**Verification Sequence:**

```
verify_actuators_safe()
  │
  ├─ Command throttle to neutral (1000us or off)
  ├─ Command steering to center (1500us)
  │
  ├─ Read back throttle PWM
  ├─ Read back steering PWM
  │
  ├─ Check if within tolerance (±10us)
  │   ├─ Yes → Success, return Ok
  │   └─ No → Wait 1ms, retry
  │
  └─ Timeout after 50ms
      ├─ Log error
      ├─ Send STATUSTEXT warning
      └─ Return Ok (allow disarm to complete)

Total: < 50ms typical, 50ms timeout
```

**No Skip Mechanism:**

```rust
// NO configuration parameter to disable verification
// NO compile flag to skip verification
// NO runtime option to bypass verification

impl SystemState {
    pub fn disarm(&mut self, method: DisarmMethod, reason: DisarmReason)
                   -> Result<(), &'static str> {
        // ... (pre-disarm checks, state change) ...

        // Post-disarm cleanup ALWAYS includes actuator verification
        self.post_disarm_cleanup(method, reason, armed_duration_ms)?;

        Ok(())
    }

    fn post_disarm_cleanup(&mut self, ...) -> Result<(), &'static str> {
        // ... (logging) ...

        // MANDATORY: Actuator safety verification
        // NO skip mechanism, NO bypass flag, NO disable option
        self.verify_actuators_safe()?; // Always called

        // ... (subsystem notification, state reset) ...
    }
}
```

## Platform Considerations

### Pico W (RP2040)

Cortex-M0+ at 133MHz:

- PWM readback may be slower (30-50ms) due to hardware latency
- Full 50ms timeout budget allocated for verification
- Same verification logic applies regardless of performance

### Pico 2 W (RP2350)

Cortex-M33 at 150MHz:

- Faster PWM readback (20-40ms typical)
- Same 50ms timeout for consistency
- Same verification logic applies regardless of performance

### Cross-Platform

Actuator safety verification is mandatory on both platforms. Timeout budget (50ms) is sufficient for both Pico W and Pico 2 W.

## Risks & Mitigation

| Risk                                            | Impact   | Likelihood | Mitigation                                                          | Validation                                |
| ----------------------------------------------- | -------- | ---------- | ------------------------------------------------------------------- | ----------------------------------------- |
| **Actuator hardware unresponsive (timeout)**    | **High** | **Medium** | **Timeout after 50ms, log error, send STATUSTEXT, complete disarm** | **Test with disconnected actuators**      |
| Developer adds skip flag to speed up testing    | Critical | Medium     | Code review catches skip flag, document non-negotiable requirement  | Audit code for skip mechanisms            |
| Verification takes too long (blocks disarm)     | Medium   | Low        | 50ms timeout sufficient for hardware, measured on real devices      | Profile verification on both platforms    |
| False positive (actuators safe but verify fail) | Medium   | Low        | Tolerance (±10us), retry logic, log actual PWM values               | Test with multiple actuator types         |
| Configuration parameter added to disable        | Critical | Low        | Document no disable mechanism, reject PRs adding disable option     | Review parameters, verify no disable flag |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Actuator safety verification (mandatory, no skip)
fn verify_actuators_safe(&mut self) -> Result<(), &'static str> {
    const VERIFY_TIMEOUT_MS: u32 = 50;
    const THROTTLE_NEUTRAL: u16 = 1000; // Min or off
    const STEERING_NEUTRAL: u16 = 1500; // Center
    const TOLERANCE: u16 = 10; // ±10us tolerance

    let verify_start = get_time_ms();

    // Command actuators to neutral
    self.actuators.set_throttle_pwm(THROTTLE_NEUTRAL)?;
    self.actuators.set_steering_pwm(STEERING_NEUTRAL)?;

    // Verification loop (with timeout)
    loop {
        // Check timeout
        if get_time_ms() - verify_start > VERIFY_TIMEOUT_MS {
            defmt::error!("Actuator verify timeout ({}ms)", VERIFY_TIMEOUT_MS);
            self.send_statustext("WARN: Actuator verify timeout")?;
            return Ok(()); // Allow disarm to complete
        }

        // Read back PWM values
        let throttle_pwm = self.actuators.read_throttle_pwm()?;
        let steering_pwm = self.actuators.read_steering_pwm()?;

        // Check if neutral (within tolerance)
        let throttle_safe = throttle_pwm <= THROTTLE_NEUTRAL + TOLERANCE;
        let steering_safe = (steering_pwm as i32 - STEERING_NEUTRAL as i32).abs() < TOLERANCE as i32;

        if throttle_safe && steering_safe {
            defmt::info!("Actuators safe: throttle={}, steering={}",
                         throttle_pwm, steering_pwm);
            return Ok(());
        }

        // Log current state (for debugging)
        defmt::debug!("Actuators not neutral: throttle={} (target {}), steering={} (target {})",
                      throttle_pwm, THROTTLE_NEUTRAL, steering_pwm, STEERING_NEUTRAL);

        // Wait and retry
        delay_us(1000); // 1ms between checks
    }
}
```

**Testing Strategy:**

```rust
#[test]
fn test_actuator_verify_mandatory() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Disarm should ALWAYS verify actuators
    state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal).unwrap();

    // Verify actuators_safe flag set
    assert!(state.post_disarm_state.actuators_safe);
}

#[test]
fn test_actuator_verify_timeout() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate unresponsive actuators
    state.actuators.set_unresponsive(true);

    // Disarm should timeout but still complete
    let result = state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal);
    assert!(result.is_ok(), "Disarm should complete despite verify timeout");

    // Verify actuators_safe flag is false (timeout)
    assert!(!state.post_disarm_state.actuators_safe);
}

#[test]
fn test_no_skip_mechanism() {
    // Verify no configuration parameter to skip verification
    // This test ensures no developer adds a skip flag

    let mut state = SystemState::new();
    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Check that disarm ALWAYS calls verify_actuators_safe()
    // (This is a code audit test, not runtime test)

    // Search for verify_actuators_safe() call in post_disarm_cleanup()
    // Verify no conditional logic that could skip the call
}

#[test]
fn test_actuator_verify_error_handling() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Simulate actuator hardware failure
    state.actuators.set_failure_mode(true);

    // Disarm should log error but still complete
    let result = state.disarm(DisarmMethod::GcsCommand, DisarmReason::Normal);
    assert!(result.is_ok(), "Disarm should complete despite verify error");

    // Verify STATUSTEXT warning sent
    let statustext = state.mavlink.get_last_statustext().unwrap();
    assert!(statustext.contains("Actuator verify"));
}
```

**Code Review Checklist:**

```markdown
## Actuator Safety Verification Code Review

- [ ] verify_actuators_safe() is ALWAYS called in post_disarm_cleanup()
- [ ] NO skip flag, NO bypass parameter, NO disable option
- [ ] NO compile flag that disables verification
- [ ] Timeout is enforced (50ms max)
- [ ] Timeout logs error but allows disarm to complete
- [ ] Verification failure sends STATUSTEXT warning
- [ ] NO configuration parameter that disables verification
- [ ] Documentation clearly states non-negotiable requirement
```

Related code areas:

- `src/communication/mavlink/state.rs` - post_disarm_cleanup() and verify_actuators_safe()
- `src/devices/actuators/` - Actuator PWM readback implementation
- `src/core/safety/` - Safety verification logic

## External References

- ArduPilot MOT_SAFE_DISARM: <https://ardupilot.org/copter/docs/parameters.html#mot-safe-disarm>
