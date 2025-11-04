# NFR-uq1ux Disarm Validation Message Usability

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

Validation failure messages shall be human-readable, actionable, and specific to the failure reason, enabling operators to understand why disarm was denied and what corrective action to take.

## Rationale

Pre-disarm validation denies disarm commands when safety conditions are not met (throttle too high, vehicle moving, unsafe mode). Operators need clear feedback to understand the denial and correct the issue:

- **Operator Understanding**: Generic error messages ("Cannot disarm") frustrate operators, waste time troubleshooting
- **Actionable Guidance**: Error messages must explain what to do ("Reduce throttle to neutral" vs "Throttle not at neutral")
- **Safety Communication**: Operators need to know why safety system prevented disarm (not just that it failed)
- **Operational Efficiency**: Clear messages reduce support burden, training time, operator errors
- **Industry Standard**: ArduPilot provides specific denial reasons with recommended actions in error messages

Usable error messages reduce operator frustration, improve safety compliance, and accelerate troubleshooting during testing and operations.

## User Story (if applicable)

The system shall provide human-readable validation failure messages that explain the specific reason disarm was denied and include recommended corrective action to enable operators to quickly resolve the issue.

## Acceptance Criteria

- [ ] Each validation failure returns a unique error message identifying the specific safety condition that failed
- [ ] Error messages use plain language understandable to non-technical operators (no jargon, acronyms, or codes)
- [ ] Error messages include recommended corrective action (e.g., "Reduce throttle to neutral position")
- [ ] Error messages sent via MAVLink STATUSTEXT to ground station with severity level MAV_SEVERITY_WARNING
- [ ] Error messages logged to system log with additional context (timestamp, method, current state)
- [ ] Validation success returns no message (silent success, no operator distraction)
- [ ] Error message length limited to MAVLink STATUSTEXT constraint (50 characters maximum)
- [ ] All error messages tested with representative operators to verify comprehension

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Usability:**

- **Clarity**: Error messages use simple, direct language
  - Good: "Cannot disarm: Throttle not at neutral"
  - Bad: "ERR_DISARM_VAL_FAIL_0x02"
- **Specificity**: Each failure reason has unique message
  - "Cannot disarm: Throttle not at neutral"
  - "Cannot disarm: Vehicle moving too fast"
  - "Cannot disarm: Current mode does not allow disarm"
  - "Already disarmed"
- **Actionability**: Messages explain how to resolve the issue
  - "Cannot disarm: Throttle not at neutral. Reduce throttle to neutral position."
  - "Cannot disarm: Vehicle moving too fast. Stop vehicle before disarming."
  - "Cannot disarm: Current mode does not allow disarm. Switch to Hold mode first."

**Message Standards:**

| Validation Failure       | Error Message                                                          | Recommended Action                     |
| ------------------------ | ---------------------------------------------------------------------- | -------------------------------------- |
| Already Disarmed         | "Already disarmed"                                                     | None (informational)                   |
| Throttle Active          | "Cannot disarm: Throttle not at neutral. Reduce throttle to neutral."  | Move throttle stick to center position |
| Velocity Too High        | "Cannot disarm: Vehicle moving too fast. Stop vehicle before disarm."  | Brake vehicle to complete stop         |
| Unsafe Mode              | "Cannot disarm: Current mode does not allow disarm. Switch to Hold."   | Change mode to Hold before disarming   |
| Generic Denial (unknown) | "Cannot disarm: Safety check failed. Ensure vehicle is safe to disarm" | Verify all safety conditions manually  |

**Implementation:**

```rust
/// Disarm validation result with error messages
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum DisarmValidationResult {
    Allowed,
    DeniedThrottleActive,
    DeniedVelocityTooHigh,
    DeniedUnsafeMode,
    DeniedAlreadyDisarmed,
}

impl DisarmValidationResult {
    /// Convert validation result to human-readable error message
    pub fn to_error_message(&self) -> &'static str {
        match self {
            DisarmValidationResult::Allowed => "Validation passed",
            DisarmValidationResult::DeniedThrottleActive =>
                "Cannot disarm: Throttle not at neutral. Reduce throttle to neutral.",
            DisarmValidationResult::DeniedVelocityTooHigh =>
                "Cannot disarm: Vehicle moving too fast. Stop vehicle before disarm.",
            DisarmValidationResult::DeniedUnsafeMode =>
                "Cannot disarm: Current mode does not allow disarm. Switch to Hold.",
            DisarmValidationResult::DeniedAlreadyDisarmed =>
                "Already disarmed",
        }
    }

    /// Get MAVLink severity level for the validation result
    pub fn to_mav_severity(&self) -> MAV_SEVERITY {
        match self {
            DisarmValidationResult::Allowed => MAV_SEVERITY::MAV_SEVERITY_INFO,
            DisarmValidationResult::DeniedAlreadyDisarmed =>
                MAV_SEVERITY::MAV_SEVERITY_INFO,
            DisarmValidationResult::DeniedThrottleActive |
            DisarmValidationResult::DeniedVelocityTooHigh |
            DisarmValidationResult::DeniedUnsafeMode =>
                MAV_SEVERITY::MAV_SEVERITY_WARNING,
        }
    }
}

/// Handle disarm validation failure
fn handle_disarm_validation_failure(
    &mut self,
    method: DisarmMethod,
    result: DisarmValidationResult
) -> MavResult {
    // Get human-readable error message
    let error_msg = result.to_error_message();
    let severity = result.to_mav_severity();

    // Log denial with context
    defmt::warn!("Disarm denied: {} (method: {:?})", error_msg, method);

    // Send STATUSTEXT to ground station
    self.send_status_text(severity, error_msg)?;

    // Log detailed context for debugging
    self.log_disarm_denied(method, result)?;

    MavResult::MAV_RESULT_DENIED
}
```

**MAVLink Integration:**

```rust
/// Send STATUSTEXT to ground station with validation failure message
fn send_status_text(&mut self, severity: MAV_SEVERITY, text: &str)
                    -> Result<(), &'static str> {
    // MAVLink STATUSTEXT limited to 50 characters
    let truncated = if text.len() > 50 {
        &text[..50]
    } else {
        text
    };

    let msg = MAVLinkMessage::STATUSTEXT(STATUSTEXT_DATA {
        severity,
        text: truncated.as_bytes().try_into()
            .map_err(|_| "Failed to create STATUSTEXT")?,
    });

    self.mavlink_transport.send_message(&msg)?;
    Ok(())
}
```

**Logging Context:**

```rust
/// Log denied disarm attempt with full context
fn log_disarm_denied(&mut self, method: DisarmMethod,
                     reason: DisarmValidationResult) -> Result<(), &'static str> {
    let timestamp = get_time_ms();
    let throttle = self.get_throttle_normalized();
    let velocity = self.get_velocity_mps();

    // Log detailed context for post-incident analysis
    let log_entry = DisarmDeniedLog {
        timestamp,
        method,
        reason,
        current_mode: self.mode,
        throttle_percent: (throttle * 100.0) as u8,
        velocity_mps: (velocity * 10.0) as u16, // 0.1 m/s resolution
    };

    self.logger.log_disarm_denied(&log_entry)?;

    defmt::warn!(
        "DISARM_DENIED: reason={:?}, method={:?}, throttle={}%, velocity={}m/s",
        reason, method, throttle * 100.0, velocity
    );

    Ok(())
}
```

**Message Length Constraints:**

MAVLink STATUSTEXT field limited to 50 characters. Error messages must be concise:

| Message                                                               | Length | Valid? |
| --------------------------------------------------------------------- | ------ | ------ |
| "Cannot disarm: Throttle not at neutral. Reduce throttle to neutral." | 70     | NO     |
| "Cannot disarm: Throttle active. Reduce to neutral"                   | 50     | YES    |
| "Cannot disarm: Vehicle moving too fast. Stop vehicle before disarm." | 69     | NO     |
| "Cannot disarm: Moving too fast. Stop first"                          | 43     | YES    |
| "Cannot disarm: Current mode does not allow disarm. Switch to Hold."  | 68     | NO     |
| "Cannot disarm: Mode unsafe. Switch to Hold"                          | 43     | YES    |

**Revised Messages (50 char limit):**

```rust
impl DisarmValidationResult {
    pub fn to_error_message(&self) -> &'static str {
        match self {
            DisarmValidationResult::Allowed => "Validation passed",
            DisarmValidationResult::DeniedThrottleActive =>
                "Cannot disarm: Throttle active. Move to neutral", // 49 chars
            DisarmValidationResult::DeniedVelocityTooHigh =>
                "Cannot disarm: Moving too fast. Stop first", // 43 chars
            DisarmValidationResult::DeniedUnsafeMode =>
                "Cannot disarm: Mode unsafe. Switch to Hold", // 43 chars
            DisarmValidationResult::DeniedAlreadyDisarmed =>
                "Already disarmed", // 16 chars
        }
    }
}
```

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                                                   | Impact | Likelihood | Mitigation                                                            | Validation                                       |
| ------------------------------------------------------ | ------ | ---------- | --------------------------------------------------------------------- | ------------------------------------------------ |
| Error messages too technical for operators             | Medium | Medium     | Test messages with representative operators, use plain language       | User testing with non-technical operators        |
| Message length exceeds MAVLink STATUSTEXT limit (50ch) | High   | Medium     | Enforce 50 character limit, truncate if needed, prioritize clarity    | Review all messages, automated length validation |
| Messages not actionable (don't explain how to fix)     | Medium | Medium     | Include recommended action in all error messages                      | Review checklist: each message has action        |
| Too many messages overwhelm operator                   | Low    | Low        | Only send message on validation failure, silent success               | Verify no messages during normal disarm          |
| Message ambiguity causes confusion                     | Medium | Low        | Use specific language, test with operators, iterate based on feedback | User testing, track operator confusion incidents |
| Localization needed for non-English operators          | Low    | Low        | Phase 1: English only, Phase 2: consider localization                 | Track user requests for other languages          |

## Implementation Notes

**Recommended Architecture:**

```rust
/// Disarm command handler with usable error messages
fn handle_arm_disarm(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
    let should_arm = cmd.param1 > 0.5;
    let force = cmd.param2 == 21196.0;  // Magic value for force

    if !should_arm {
        // Disarm
        let method = DisarmMethod::GcsCommand;

        match self.state.disarm(method, force) {
            Ok(()) => {
                // Success: silent (no STATUSTEXT)
                defmt::info!("Vehicle disarmed via GCS");
                MavResult::MAV_RESULT_ACCEPTED
            }
            Err(error_msg) => {
                // Failure: send human-readable message to GCS
                defmt::warn!("Disarm rejected: {}", error_msg);

                // Send STATUSTEXT with error message (50 char limit)
                self.send_status_text(MAV_SEVERITY::MAV_SEVERITY_WARNING, error_msg)?;

                MavResult::MAV_RESULT_DENIED
            }
        }
    }
}
```

**Usability Testing:**

```rust
#[test]
fn test_error_message_clarity() {
    // Verify all error messages are clear, specific, actionable
    let cases = [
        (DisarmValidationResult::DeniedThrottleActive,
         "Cannot disarm: Throttle active. Move to neutral"),
        (DisarmValidationResult::DeniedVelocityTooHigh,
         "Cannot disarm: Moving too fast. Stop first"),
        (DisarmValidationResult::DeniedUnsafeMode,
         "Cannot disarm: Mode unsafe. Switch to Hold"),
        (DisarmValidationResult::DeniedAlreadyDisarmed,
         "Already disarmed"),
    ];

    for (result, expected_msg) in cases {
        let msg = result.to_error_message();

        // Verify message matches expected
        assert_eq!(msg, expected_msg);

        // Verify message length within MAVLink limit
        assert!(msg.len() <= 50, "Message too long: {} chars: '{}'", msg.len(), msg);

        // Verify message is specific (not generic)
        assert!(!msg.contains("error"), "Message too generic: {}", msg);
        assert!(!msg.contains("failed"), "Message too generic: {}", msg);

        // Verify message is actionable (except info messages)
        if result != DisarmValidationResult::Allowed &&
           result != DisarmValidationResult::DeniedAlreadyDisarmed {
            // Should explain what to do
            assert!(msg.contains("Move") || msg.contains("Stop") || msg.contains("Switch"),
                    "Message not actionable: {}", msg);
        }
    }
}

#[test]
fn test_statustext_integration() {
    let mut state = SystemState::new();

    state.arm(ArmMethod::GcsCommand, true).unwrap();

    // Set throttle high to trigger validation failure
    state.set_throttle_normalized(0.5); // 50% throttle

    // Attempt disarm (should fail with clear message)
    let result = state.disarm(DisarmMethod::GcsCommand, false);

    // Verify disarm failed
    assert!(result.is_err());

    // Verify error message is clear
    let error_msg = result.unwrap_err();
    assert_eq!(error_msg, "Cannot disarm: Throttle active. Move to neutral");

    // Verify STATUSTEXT sent to GCS
    let messages = state.mavlink_transport.get_sent_messages();
    assert_eq!(messages.len(), 1);

    match &messages[0] {
        MAVLinkMessage::STATUSTEXT(msg) => {
            assert_eq!(msg.severity, MAV_SEVERITY::MAV_SEVERITY_WARNING);
            assert!(String::from_utf8_lossy(&msg.text).contains("Throttle active"));
        }
        _ => panic!("Expected STATUSTEXT message"),
    }
}
```

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState::disarm() and error handling
- `src/communication/mavlink/handlers/command.rs` - MAV_CMD_COMPONENT_ARM_DISARM handler
- `src/vehicle/arming/types.rs` - DisarmValidationResult with to_error_message()
- `src/communication/mavlink/transport.rs` - send_status_text() implementation

**Message Design Guidelines:**

- **Be specific**: Identify exact failure condition ("Throttle active" not "Safety check failed")
- **Be actionable**: Tell operator what to do ("Move to neutral" not "Throttle not at neutral")
- **Be concise**: Fit within 50 character MAVLink limit
- **Be consistent**: Use same terminology across all messages
- **Be respectful**: Avoid blame language ("Move to neutral" not "You must move to neutral")
- **Be contextual**: Include just enough context to understand failure ("Mode unsafe. Switch to Hold")

## External References

- Analysis: [AN-dqzc6-pre-disarm-validation](../analysis/AN-dqzc6-pre-disarm-validation.md)
- MAVLink STATUSTEXT Message: <https://mavlink.io/en/messages/common.html#STATUSTEXT>
- MAVLink Severity Levels: <https://mavlink.io/en/messages/common.html#MAV_SEVERITY>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
