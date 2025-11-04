# FR-ejx8b Disarm Validation Failure Feedback

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-dqzc6-pre-disarm-validation](../analysis/AN-dqzc6-pre-disarm-validation.md)
- Prerequisite Requirements:
  - [FR-jvydv-disarm-armed-state-check](FR-jvydv-disarm-armed-state-check.md)
  - [FR-jmtr1-disarm-throttle-validation](FR-jmtr1-disarm-throttle-validation.md)
  - [FR-waw1p-disarm-method-validation](FR-waw1p-disarm-method-validation.md)
- Dependent Requirements: N/A – No dependent requirements

## Requirement Statement

The system shall provide clear, specific, and actionable feedback when disarm validation fails, including error messages describing the failure reason, current values vs. required thresholds, recommended operator actions, STATUSTEXT messages to GCS, and log entries for audit trail.

## Rationale

Validation failure feedback is essential for operator understanding and safety:

- **Operator awareness:** Operators need to know why disarm was denied to take corrective action
- **Safety education:** Clear messages help operators understand safety requirements and avoid unsafe practices
- **Debugging support:** Specific error messages with current values help diagnose issues (sensor failures, configuration errors)
- **Audit trail:** Log entries document all disarm attempts for post-flight analysis and safety investigations
- **Reduced frustration:** Actionable recommendations help operators resolve issues quickly rather than repeatedly attempting disarm

Without clear feedback, operators are left guessing why disarm failed, leading to frustration, unsafe workarounds (like forced disarm when not appropriate), and inability to diagnose underlying issues.

## User Story

As an operator, I want clear feedback when disarm is denied, including the specific reason, current values, and what I need to do to resolve the issue, so that I can safely disarm the vehicle once conditions are appropriate.

## Acceptance Criteria

- [ ] Return specific error message for each validation failure type
- [ ] Error message includes failure reason (throttle high, moving too fast, etc.)
- [ ] Error message includes current value and required threshold
- [ ] Send STATUSTEXT message to GCS with failure details
- [ ] Log validation failure with timestamp, method, reason, and relevant values
- [ ] Include recommended action in error message (e.g., "reduce throttle to < 5%")
- [ ] Different message formats per disarm method (RC vs GCS)
- [ ] Support multiple validation failures (report first failure encountered)
- [ ] Feedback generation completes in less than 50 microseconds

## Technical Details

### Functional Requirement Details

**Input:**

- Validation failure result (DisarmValidationResult enum)
- Disarm method (for context-specific messaging)
- Current values (throttle percentage, mode, etc.)

**Output:**

- Error message string (returned to caller)
- STATUSTEXT message to GCS (MAVLink)
- Log entry with failure details
- Optional recommended action

**Validation Failure Types and Messages:**

| Failure Type          | Error Message                        | STATUSTEXT Message                                     | Recommended Action              |
| --------------------- | ------------------------------------ | ------------------------------------------------------ | ------------------------------- |
| DeniedAlreadyDisarmed | "Already disarmed"                   | "Disarm denied: Vehicle already disarmed"              | "Vehicle is already disarmed"   |
| DeniedThrottleActive  | "Throttle not at neutral"            | "Disarm denied: Throttle at X%, reduce to < Y%"        | "Reduce throttle to neutral"    |
| DeniedUnsafeMode      | "Current mode does not allow disarm" | "Disarm denied: Mode \[MODE] does not allow RC disarm" | "Change to Hold mode to disarm" |

**Behavior:**

1. Validation check fails and returns DisarmValidationResult
2. disarm() receives failure result
3. Generate error message based on failure type:
   - Convert result to human-readable message
   - Include current value vs. threshold
   - Add recommended action
4. Send STATUSTEXT to GCS:
   - Severity: MAV_SEVERITY_WARNING
   - Text: Detailed failure reason with values
5. Log validation failure:
   - Timestamp
   - Disarm method
   - Validation result type
   - Current throttle/mode
   - Error message sent to operator
6. Return error to caller with message

**Pseudocode:**

```rust
impl DisarmValidationResult {
    /// Convert validation result to error message
    pub fn to_error_message(&self) -> &'static str {
        match self {
            DisarmValidationResult::Allowed =>
                "Validation passed",
            DisarmValidationResult::DeniedAlreadyDisarmed =>
                "Already disarmed",
            DisarmValidationResult::DeniedThrottleActive =>
                "Throttle not at neutral",
            DisarmValidationResult::DeniedUnsafeMode =>
                "Current mode does not allow disarm",
        }
    }

    /// Get detailed message with current values
    pub fn to_detailed_message(&self, throttle: f32, threshold_throttle: f32)
                                -> String {
        match self {
            DisarmValidationResult::DeniedThrottleActive =>
                format!("Disarm denied: Throttle at {:.1}%, reduce to < {:.1}%",
                        throttle * 100.0, threshold_throttle * 100.0),
            _ => self.to_error_message().to_string(),
        }
    }

    /// Get recommended operator action
    pub fn get_recommended_action(&self) -> &'static str {
        match self {
            DisarmValidationResult::DeniedThrottleActive =>
                "Reduce throttle to neutral position",
            DisarmValidationResult::DeniedUnsafeMode =>
                "Change to Hold mode to disarm",
            DisarmValidationResult::DeniedAlreadyDisarmed =>
                "Vehicle is already disarmed",
            _ => "Unknown issue",
        }
    }
}

impl SystemState {
    fn handle_validation_failure(&mut self, method: DisarmMethod,
                                   result: DisarmValidationResult) -> Result<(), &'static str> {
        // 1. Log validation failure with details
        self.log_disarm_denied(method, result)?;

        // 2. Send detailed STATUSTEXT to GCS
        let throttle = self.get_throttle_normalized();
        let config = &self.disarm_validation_config;

        let detailed_msg = result.to_detailed_message(
            throttle,
            config.max_throttle_manual
        );
        self.send_statustext(MAV_SEVERITY_WARNING, &detailed_msg)?;

        // 3. Optionally send recommended action (if GCS supports multi-line)
        let action = result.get_recommended_action();
        self.send_statustext(MAV_SEVERITY_INFO, &format!("Action: {}", action))?;

        // 4. Return error message to caller
        Err(result.to_error_message())
    }

    fn log_disarm_denied(&self, method: DisarmMethod,
                         result: DisarmValidationResult) -> Result<(), &'static str> {
        let throttle = self.get_throttle_normalized();

        let log_entry = format!(
            "DISARM_DENIED,{},{:?},{:?},throttle={:.1}%",
            get_time_ms(),
            method,
            result,
            throttle * 100.0
        );

        // TODO: Write to log storage
        warn!("Disarm denied: {:?} via {:?} - throttle={:.1}%",
              result, method, throttle * 100.0);
        Ok(())
    }
}
```

**Example Output Sequence:**

```
Operator attempts RC disarm with throttle at 12%:

1. Error returned: Err("Throttle not at neutral")
2. STATUSTEXT: "Disarm denied: Throttle at 12.0%, reduce to < 5.0%"
3. STATUSTEXT: "Action: Reduce throttle to neutral position"
4. Log: "DISARM_DENIED,1234567,RcManual,DeniedThrottleActive,throttle=12.0%"
```

**Error Conditions:**

- STATUSTEXT send fails: Log error but continue with disarm denial
- Log write fails: Log to console, continue with disarm denial
- String formatting error: Use fallback static message

## Platform Considerations

### Unix

N/A – Platform agnostic

### Windows

N/A – Platform agnostic

### Cross-Platform

N/A – Platform agnostic

## Risks & Mitigation

| Risk                                            | Impact | Likelihood | Mitigation                                                             | Validation                                 |
| ----------------------------------------------- | ------ | ---------- | ---------------------------------------------------------------------- | ------------------------------------------ |
| Error messages too technical for operators      | Medium | Medium     | Use clear language, include current values and thresholds              | Test messages with non-technical operators |
| STATUSTEXT messages not displayed by GCS        | Medium | Low        | Log all messages, document GCS display requirements                    | Test with Mission Planner and QGC          |
| Multiple failures only report first failure     | Low    | Medium     | Document behavior, consider reporting all failures in future           | Test multiple validation failures          |
| String formatting overhead delays disarm denial | Low    | Low        | Use efficient formatting, measure timing, optimize if needed           | Profile feedback generation timing         |
| Log entries too verbose (fills storage)         | Low    | Medium     | Compact format, only log denied attempts (not successful disarms here) | Monitor log storage usage                  |
| Recommended actions not helpful                 | Medium | Medium     | Review actions with operators, iterate based on feedback               | Collect operator feedback on messages      |

## Implementation Notes

**Preferred approaches:**

- Enum-based error types for type safety and exhaustive matching
- Detailed messages with current values and thresholds
- Separate STATUSTEXT messages for error and recommended action
- Compact log format for storage efficiency
- Use warn!() for denied disarms (indicates unusual but non-critical event)

**Known pitfalls:**

- Do not use generic error messages like "Cannot disarm" without details
- Do not assume GCS displays all STATUSTEXT messages (log everything)
- Avoid overly technical jargon in operator-facing messages
- Do not make error messages too long (STATUSTEXT limited to 50 chars)
- Ensure recommended actions are actionable (not vague suggestions)

**Related code areas:**

- `src/communication/mavlink/state.rs` - SystemState::disarm() error handling
- `src/vehicle/arming/validation.rs` - DisarmValidationResult enum and methods
- `src/vehicle/arming/types.rs` - Error message generation
- `src/communication/mavlink/messages/statustext.rs` - STATUSTEXT sending
- `src/logging/disarm_events.rs` - Disarm denial logging

**Suggested patterns:**

- Enum with methods for error message generation
- Builder pattern for detailed messages with context
- Separate logging and operator messaging (different audiences)
- Fallback static messages if formatting fails

## External References

- Analysis: [AN-dqzc6-pre-disarm-validation](../analysis/AN-dqzc6-pre-disarm-validation.md)
- [MAVLink STATUSTEXT Message](https://mavlink.io/en/messages/common.html#STATUSTEXT) - STATUSTEXT message format
- [Human Interface Guidelines](https://en.wikipedia.org/wiki/Human_interface_guidelines) - Error message best practices

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
