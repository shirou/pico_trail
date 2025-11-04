# FR-jynu9 Forced Disarm Emergency Override

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-dqzc6-pre-disarm-validation](../analysis/AN-dqzc6-pre-disarm-validation.md)
  - [AN-4ez27-emergency-stop](../analysis/AN-4ez27-emergency-stop.md)
- Prerequisite Requirements:
  - [FR-jvydv-disarm-armed-state-check](FR-jvydv-disarm-armed-state-check.md)
- Dependent Requirements: N/A – No dependent requirements

## Requirement Statement

The system shall support forced disarm override that bypasses all validation checks (except armed state verification) for emergency situations, triggered via MAVLink command parameter or explicit API call, ensuring immediate disarm capability when safety requires it.

## Rationale

Forced disarm override is critical for emergency scenarios where validation checks must be bypassed:

- **Emergency shutdown:** Critical situations (fire, mechanical failure, crash) require immediate power cutoff regardless of throttle, velocity, or mode
- **Validation failure recovery:** If validation logic has bugs or sensor failures prevent normal disarm, operator needs emergency override
- **Failsafe escalation:** When vehicle is in unsafe state and normal disarm is blocked, forced disarm provides last-resort shutdown
- **Regulatory requirement:** Safety standards mandate ability to cut power immediately in emergency situations
- **Operator control:** Final authority to disable vehicle must rest with operator, not automated checks

Without forced disarm, operators could be trapped in situations where vehicle is armed but cannot be disarmed due to validation failures, sensor errors, or unexpected conditions, creating dangerous scenarios with no safe resolution.

## User Story

As an operator in an emergency situation, I want to force disarm the vehicle immediately bypassing all safety checks, so that I can cut power to prevent or mitigate a dangerous situation regardless of validation rules.

## Acceptance Criteria

- [ ] Accept `forced` parameter (boolean) in disarm() function
- [ ] Forced disarm bypasses throttle validation check
- [ ] Forced disarm bypasses velocity validation check
- [ ] Forced disarm bypasses mode-specific restrictions
- [ ] Forced disarm still requires armed state check (only bypass validation checks)
- [ ] MAV_CMD_COMPONENT_ARM_DISARM with param2=21196 triggers forced disarm
- [ ] Log forced disarm with reason code indicating emergency override
- [ ] Send STATUSTEXT to GCS: "FORCED DISARM executed"
- [ ] Forced disarm always succeeds (returns Ok) if vehicle is armed
- [ ] Forced disarm completes in less than 1 millisecond

## Technical Details

### Functional Requirement Details

**Input:**

- Forced disarm flag (boolean parameter to disarm())
- Disarm method (for logging)
- MAVLink MAV_CMD_COMPONENT_ARM_DISARM with param2=21196 (magic value)

**Output:**

- Disarm success (always succeeds if armed)
- Log entry marking forced disarm
- STATUSTEXT warning message to GCS
- Post-disarm cleanup execution

**Behavior:**

1. disarm() called with method and forced=true parameters
2. Check armed state (only validation that is NOT bypassed)
3. If not armed: return error "Already disarmed"
4. If armed and forced=true:
   - Skip throttle validation
   - Skip velocity validation
   - Skip mode restrictions
   - Log forced disarm with method and reason
   - Send STATUSTEXT: "FORCED DISARM executed"
   - Set armed state to Disarmed
   - Execute post-disarm cleanup
   - Return Ok(())
5. Forced disarm never fails due to validation (only fails if not armed)

**MAVLink Forced Disarm Trigger:**

```
MAV_CMD_COMPONENT_ARM_DISARM:
  param1 = 0.0        (disarm)
  param2 = 21196.0    (magic value for force flag)
```

The magic value 21196 is ArduPilot's standard for forced disarm commands.

**Pseudocode:**

```rust
pub fn disarm(&mut self, method: DisarmMethod, forced: bool) -> Result<(), &'static str> {
    // 1. Armed state check (always required, even for forced)
    if !self.is_armed() {
        self.log_disarm_denied(method, DisarmValidationResult::DeniedAlreadyDisarmed)?;
        return Err("Already disarmed");
    }

    // 2. Apply validation checks (unless forced)
    if !forced {
        let validation_result = self.validate_disarm(method);
        if validation_result != DisarmValidationResult::Allowed {
            self.log_disarm_denied(method, validation_result)?;
            self.send_statustext_for_denial(method, validation_result)?;
            return Err(validation_result.to_error_message());
        }
    } else {
        // Log forced disarm (bypassed validation)
        warn!("FORCED DISARM: Bypassing validation checks");
        self.log_forced_disarm(method)?;
        self.send_statustext(MAV_SEVERITY_WARNING, "FORCED DISARM executed")?;
    }

    // 3. Validation passed or forced, proceed with disarm
    self.armed = ArmedState::Disarmed;
    self.post_disarm_cleanup(method, forced)?;
    Ok(())
}

fn log_forced_disarm(&self, method: DisarmMethod) -> Result<(), &'static str> {
    // Log forced disarm with special marker for audit trail
    let log_entry = format!(
        "FORCED_DISARM,{},{:?},validation_bypassed",
        get_time_ms(),
        method
    );
    // TODO: Write to log storage
    warn!("Forced disarm via {:?}", method);
    Ok(())
}
```

**MAVLink Command Handler:**

```rust
fn handle_arm_disarm(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
    let should_arm = cmd.param1 > 0.5;
    let force = cmd.param2 == 21196.0;  // ArduPilot magic value

    if should_arm {
        // Arm logic...
    } else {
        // Disarm with forced flag
        let method = DisarmMethod::GcsCommand;
        match self.state.disarm(method, force) {
            Ok(()) => {
                if force {
                    warn!("Vehicle FORCED disarmed via GCS");
                } else {
                    info!("Vehicle disarmed via GCS");
                }
                MavResult::MAV_RESULT_ACCEPTED
            }
            Err(reason) => {
                // Forced disarm should never fail (except already disarmed)
                warn!("Disarm rejected: {}", reason);
                self.send_status_text(MAV_SEVERITY_WARNING, reason)?;
                MavResult::MAV_RESULT_DENIED
            }
        }
    }
}
```

**Error Conditions:**

- Forced disarm on already disarmed vehicle: Return error "Already disarmed"
- Post-disarm cleanup fails during forced disarm: Log error but complete disarm
- Forced flag misused: Log warning if forced used without emergency context

**Logging:**

```
FORCED_DISARM,<timestamp>,<method>,validation_bypassed,throttle=<value>,velocity=<value>
```

Log includes actual throttle and velocity values to document conditions that would have prevented normal disarm.

## Platform Considerations

### Unix

N/A – Platform agnostic

### Windows

N/A – Platform agnostic

### Cross-Platform

N/A – Platform agnostic

## Risks & Mitigation

| Risk                                               | Impact | Likelihood | Mitigation                                                                   | Validation                                   |
| -------------------------------------------------- | ------ | ---------- | ---------------------------------------------------------------------------- | -------------------------------------------- |
| Forced disarm used inappropriately (not emergency) | Medium | Medium     | Log forced disarm with warning, send STATUSTEXT alert, document proper usage | Review forced disarm logs for misuse         |
| Forced disarm still blocked by some check          | High   | Low        | Ensure forced flag bypasses ALL validation except armed state                | Test forced disarm with active throttle      |
| Operator does not know how to force disarm         | High   | Medium     | Document forced disarm procedure, include in training, GCS UI support        | Operator training and procedure verification |
| Forced disarm causes equipment damage              | High   | Low        | Acceptable trade-off - emergency situations require immediate shutdown       | Document when forced disarm is appropriate   |
| MAVLink magic value conflicts with future changes  | Low    | Low        | Use ArduPilot standard value (21196), document in code                       | Test forced disarm via MAVLink command       |
| Post-disarm cleanup failure prevents forced disarm | High   | Low        | Complete disarm even if cleanup fails, log cleanup errors separately         | Test forced disarm with cleanup failures     |

## Implementation Notes

**Preferred approaches:**

- Use boolean `forced` parameter in disarm() function signature
- Make forced flag explicit (not hidden in configuration)
- Log all forced disarms with warning severity for audit trail
- Follow ArduPilot convention for MAVLink forced disarm (param2=21196)
- Send clear STATUSTEXT to GCS indicating forced disarm executed

**Known pitfalls:**

- Do not allow forced disarm on already-disarmed vehicle (maintains consistency)
- Do not silently force disarm (always log and notify GCS)
- Avoid complex forced disarm conditions (forced means "bypass everything")
- Do not fail forced disarm due to cleanup errors (disarm must complete)
- Ensure forced disarm works even if sensor data is unavailable

**Related code areas:**

- `src/communication/mavlink/state.rs` - SystemState::disarm() with forced parameter
- `src/communication/mavlink/handlers/command.rs` - MAV_CMD_COMPONENT_ARM_DISARM handling
- `src/vehicle/arming/validation.rs` - Validation bypass logic for forced disarm
- `src/vehicle/arming/cleanup.rs` - Post-disarm cleanup (must not block forced disarm)
- `src/vehicle/failsafe/actions.rs` - Failsafe-triggered forced disarm

**Suggested patterns:**

- Boolean flag parameter for forced disarm (simple, explicit)
- Early check for forced flag to bypass validation
- Separate logging function for forced disarm (audit trail)
- Clear warning messages to operator about forced disarm

## External References

- Analysis: [AN-dqzc6-pre-disarm-validation](../analysis/AN-dqzc6-pre-disarm-validation.md)
- [MAVLink MAV_CMD_COMPONENT_ARM_DISARM](https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM) - param2=21196 for forced disarm
- [ArduPilot AP_Arming forced disarm](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Arming/AP_Arming.cpp) - Forced disarm implementation

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
