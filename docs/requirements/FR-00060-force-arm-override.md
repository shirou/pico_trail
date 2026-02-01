# FR-00060 Force Arm Emergency Override

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00008-pre-arm-checks](../analysis/AN-00008-pre-arm-checks.md)
- Prerequisite Requirements:
  - [FR-00051-prearm-capability-enforcement](FR-00051-prearm-capability-enforcement.md)
- Dependent Requirements: N/A – No dependent requirements
- Related ADRs:
  - [ADR-00012-arming-system-architecture](../adr/ADR-00012-arming-system-architecture.md)
- Related Tasks:
  - [T-00008-arming-system-implementation](../tasks/T-00008-arming-system-implementation/README.md)

## Requirement Statement

The system shall support force-arm override that bypasses all pre-arm validation checks for testing and emergency situations, triggered via MAVLink command parameter (param2=21196), ensuring temporary check bypass capability without permanently disabling ARMING_CHECK parameter.

## Rationale

Force-arm override is critical for specific scenarios where pre-arm checks must be temporarily bypassed:

- **Bench testing:** Development and testing environments where sensors may not be fully functional or calibrated
- **SITL simulation:** Software-in-the-loop testing environments where hardware sensors are simulated or unavailable
- **Emergency recovery:** Situations where sensor failures prevent normal arming but vehicle operation is required
- **Troubleshooting:** Diagnostic scenarios to isolate whether arming failures are due to check logic or actual hardware issues
- **One-time bypass:** Safer alternative to ARMING_CHECK=0 (permanent configuration) that requires explicit action each time

Without force-arm, operators would need to disable ARMING_CHECK entirely (ARMING_CHECK=0), which:

- Persists across reboots (unsafe default)
- Masks underlying check configuration issues
- Provides no audit trail for when checks were bypassed
- Requires parameter file modification

Force-arm provides a safer mechanism with explicit operator acknowledgment, audit logging, and ArduPilot GCS compatibility.

## User Story

As a developer testing the arming system on a bench setup without all sensors connected, I want to force-arm the vehicle bypassing pre-arm checks for this specific arming attempt only, so that I can test arming behavior without permanently disabling safety checks or requiring full sensor setup.

## Acceptance Criteria

- [ ] MAV_CMD_COMPONENT_ARM_DISARM with param1=1.0 and param2=21196.0 bypasses all pre-arm checks
- [ ] Force-arm still validates already-armed state (cannot arm if already armed)
- [ ] Force-arm logs attempt with WARNING severity and "FORCE ARM" audit marker
- [ ] Send STATUSTEXT to GCS: "Armed (FORCED)" with MAV_SEVERITY_WARNING
- [ ] Normal arming (param2=0.0) still executes pre-arm checks
- [ ] ARMING_CHECK parameter remains unchanged (not disabled by force-arm)
- [ ] Force-arm executes post-arm initialization sequence normally
- [ ] Force-arm attempt completes in less than 100 milliseconds
- [ ] Force-arm succeeds even when pre-arm checks would fail (RC disconnected, battery low, etc.)
- [ ] Each force-arm creates separate audit log entry

## Technical Details

### Functional Requirement Details

**Input:**

- MAVLink MAV_CMD_COMPONENT_ARM_DISARM command
  - param1 = 1.0 (arm request)
  - param2 = 21196.0 (magic number for force operation)
- Arm method (for logging)

**Output:**

- Arm success (bypasses pre-arm checks)
- Log entry marking force-arm with WARNING severity
- STATUSTEXT message to GCS: "Armed (FORCED)"
- Post-arm initialization execution

**Behavior:**

1. MAV_CMD_COMPONENT_ARM_DISARM received with param1=1.0, param2=21196.0
2. Detect force-arm request via magic number (21196.0)
3. Check already-armed state (only validation NOT bypassed)
4. If already armed: return error "Already armed"
5. If not armed and force=true:
   - Skip all pre-arm checks (RC, battery, sensors, configuration, etc.)
   - Log force-arm with method and timestamp
   - Send STATUSTEXT: "Armed (FORCED)" with WARNING severity
   - Set armed state to Armed
   - Execute post-arm initialization sequence
   - Return Ok(())
6. Force-arm never fails due to pre-arm check failures (only fails if already armed)

**MAVLink Force-Arm Trigger:**

```
MAV_CMD_COMPONENT_ARM_DISARM:
  param1 = 1.0        (arm request)
  param2 = 21196.0    (magic value for force flag)
```

The magic value 21196 is ArduPilot's standard for force operations (both arm and disarm).

**Pseudocode:**

```rust
// src/communication/mavlink/handlers/command.rs
pub fn handle_component_arm_disarm(
    cmd: &COMMAND_LONG_DATA,
    state: &mut SystemState,
    context: &SystemContext,
) -> MavResult {
    const FORCE_MAGIC_NUMBER: f32 = 21196.0;

    let arm_request = cmd.param1 == 1.0;
    let force = cmd.param2 == FORCE_MAGIC_NUMBER;

    if arm_request {
        // Arm request
        let result = if force {
            // Force-arm bypasses all pre-arm checks
            warn!("FORCE ARM requested via GCS - bypassing all pre-arm checks");
            state.arm_forced(context, ArmMethod::GcsCommand)
        } else {
            // Normal arm with pre-arm checks
            state.arm(context, ArmMethod::GcsCommand)
        };

        match result {
            Ok(()) => {
                let msg = if force { "Armed (FORCED)" } else { "Armed" };
                let severity = if force {
                    MavSeverity::MAV_SEVERITY_WARNING
                } else {
                    MavSeverity::MAV_SEVERITY_INFO
                };
                context.mavlink.send_statustext(msg, severity);
                MavResult::MAV_RESULT_ACCEPTED
            }
            Err(e) => {
                context.mavlink.send_statustext(
                    &format!("Arm failed: {}", e),
                    MavSeverity::MAV_SEVERITY_ERROR
                );
                MavResult::MAV_RESULT_FAILED
            }
        }
    } else {
        // Disarm logic...
    }
}

// src/communication/mavlink/state.rs
impl SystemState {
    pub fn arm_forced(
        &mut self,
        context: &SystemContext,
        method: ArmMethod
    ) -> Result<(), ArmingError> {
        // 1. Already-armed check (always required, even for forced)
        if self.is_armed() {
            return Err(ArmingError::AlreadyArmed);
        }

        // 2. Log force-arm (bypassed validation)
        warn!("FORCE ARM via {:?} - bypassing all pre-arm checks", method);
        self.log_force_arm(method)?;

        // 3. Transition to armed state (skip pre-arm checks)
        self.armed = true;

        // 4. Execute post-arm initialization (still required for safety)
        context.post_arm_init.execute(context)?;

        warn!("Vehicle FORCE armed - operator accepted risk");
        Ok(())
    }

    fn log_force_arm(&self, method: ArmMethod) -> Result<(), ArmingError> {
        // Log force-arm with special marker for audit trail
        let log_entry = format!(
            "FORCE_ARM,{},{:?},pre_arm_checks_bypassed",
            get_time_ms(),
            method
        );
        // TODO: Write to log storage
        warn!("Force-arm via {:?}", method);
        Ok(())
    }
}
```

**Error Conditions:**

- Force-arm on already-armed vehicle: Return error "Already armed"
- Post-arm initialization fails during force-arm: Return error and remain disarmed
- Invalid magic number: Treat as normal arm request (execute pre-arm checks)

**Logging:**

```
FORCE_ARM,<timestamp>,<method>,pre_arm_checks_bypassed,ARMING_CHECK=<value>
```

Log includes current ARMING_CHECK value to document which checks would have been active if normal arming was used.

## Platform Considerations

### Unix

N/A – Platform agnostic

### Windows

N/A – Platform agnostic

### Cross-Platform

N/A – Platform agnostic

## Risks & Mitigation

| Risk                                                      | Impact | Likelihood | Mitigation                                                                     | Validation                                   |
| --------------------------------------------------------- | ------ | ---------- | ------------------------------------------------------------------------------ | -------------------------------------------- |
| Force-arm used inappropriately (not testing/emergency)    | Medium | Medium     | Log force-arm with WARNING, send STATUSTEXT alert, document proper usage       | Review force-arm logs for misuse patterns    |
| Force-arm masks underlying sensor/configuration issues    | Medium | Medium     | ARMING_CHECK remains enabled, normal arm still executes checks                 | Compare force-arm vs normal arm success rate |
| Operator does not understand difference vs ARMING_CHECK=0 | Medium | Medium     | Document force-arm vs ARMING_CHECK=0 trade-offs, training materials            | Operator training verification               |
| Force-arm fails due to post-arm initialization error      | Medium | Low        | Acceptable - post-arm init still required for safety baseline                  | Test force-arm with init failures            |
| MAVLink magic value conflicts with future protocol        | Low    | Low        | Use ArduPilot standard value (21196), widely adopted and documented            | Test force-arm via MAVLink command           |
| Force-arm used on production flights                      | High   | Low        | Audit logs show force-arm usage, document that force-arm is test/emergency use | Review flight logs for force-arm usage       |

## Implementation Notes

**Preferred approaches:**

- Implement separate `arm_forced()` method distinct from `arm()`
- Check magic number (21196.0) in MAVLink command handler
- Log all force-arms with WARNING severity for audit trail
- Follow ArduPilot convention for MAVLink force operations
- Send clear STATUSTEXT to GCS indicating forced arm executed
- Execute post-arm initialization even for force-arm (safety baseline)

**Known pitfalls:**

- Do not allow force-arm on already-armed vehicle (maintains consistency)
- Do not silently force-arm (always log and notify GCS)
- Do not skip post-arm initialization (still needed for operational baseline)
- Ensure force-arm works even if pre-arm checks would fail
- Do not modify ARMING_CHECK parameter when force-arm used

**Related code areas:**

- `src/communication/mavlink/state.rs` - SystemState::arm_forced() implementation
- `src/communication/mavlink/handlers/command.rs` - MAV_CMD_COMPONENT_ARM_DISARM handling
- `src/core/arming/checks.rs` - PreArmCheck framework (bypassed by force-arm)
- `src/core/arming/initialization.rs` - Post-arm initialization (still executed)

**Suggested patterns:**

- Separate method for force-arm (clear intent, no boolean flag)
- Magic number constant defined in command handler
- Dedicated logging function for force-arm audit trail
- Warning-level messages to operator about force-arm

## External References

- Analysis: [AN-00008-pre-arm-checks](../analysis/AN-00008-pre-arm-checks.md) - Force-arm mechanism
- ADR: [ADR-00012-arming-system-architecture](../adr/ADR-00012-arming-system-architecture.md) - Force-arm integration
- [MAVLink MAV_CMD_COMPONENT_ARM_DISARM](https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM) - param2=21196 for force operations
- [ArduPilot MAVLink Arming Documentation](https://ardupilot.org/dev/docs/mavlink-arming-and-disarming.html) - Force-arm implementation
