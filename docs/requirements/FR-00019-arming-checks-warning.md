# FR-00019 Arming Checks Disabled Warning

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00049-post-arm-event-recording](FR-00049-post-arm-event-recording.md) (ForceArm method implies checks disabled)
- Dependent Requirements: N/A - Warning is leaf functionality
- Related Analysis:
  - [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall send a STATUSTEXT warning message with severity MAV_SEVERITY_WARNING to the GCS immediately after arming if arming checks were disabled or bypassed (ForceArm method), with message text "Arming checks disabled" to ensure operator awareness of reduced safety verification.

## Rationale

Arming checks disabled warning is critical for operator safety awareness:

- **Operator notification**: Operator must know if safety checks were bypassed (ForceArm, ARMING_CHECK=0)
- **Reduced safety**: Bypassing checks increases risk of vehicle malfunction or crash
- **Audit trail**: Warning logged in GCS, provides evidence operator was notified
- **Training indicator**: Frequent bypass warnings may indicate operator training needed
- **Regulatory compliance**: Some aviation regulations require operator notification of degraded safety states

ArduPilot sends prominent warning if arming checks disabled or bypassed, ensuring operator cannot claim ignorance of reduced safety verification.

## User Story (if applicable)

As an operator, I want to receive a clear warning if arming checks were bypassed, so that I'm aware the vehicle may not have passed all safety verifications and can make an informed decision about flight operations.

## Acceptance Criteria

- [ ] Send STATUSTEXT message if arming checks disabled (checks_performed = false)
- [ ] Warning severity: MAV_SEVERITY_WARNING (severity level 4)
- [ ] Message text: "Arming checks disabled" (exact string for consistency)
- [ ] Warning sent immediately after arm, before returning from post_arm_init()
- [ ] Warning sent to all connected GCS instances (broadcast)
- [ ] Warning logged locally (console/log file) in addition to MAVLink
- [ ] If warning send fails (no GCS connected), log locally but continue arming
- [ ] Test coverage: verify warning sent when ForceArm used, verify not sent when checks performed

## Technical Details (if applicable)

### Functional Requirement Details

**Implementation:**

```rust
impl SystemState {
    /// Post-arm initialization sequence
    fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                     -> Result<(), &'static str> {
        // 1. Record timestamp
        let arm_time_ms = get_time_ms();
        self.post_arm_state = PostArmState {
            arm_time_ms,
            arm_method: method,
            checks_performed,
            // ...
        };

        // 2. Log arming event
        self.log_arm_event(arm_time_ms, method, checks_performed)?;

        // 3. Initialize actuators
        self.initialize_actuators()?;

        // 4. Notify subsystems
        self.notify_subsystems_armed(arm_time_ms, method)?;

        // 5. Update GPIO (if hardware present)
        self.update_arm_gpio(true)?;

        // 6. Send warnings if checks disabled
        if !checks_performed {
            if let Err(e) = self.send_warning("Arming checks disabled") {
                // Warning send failed (no GCS?), log locally but continue
                warn!("Failed to send arming checks warning: {}", e);
            }
        }

        info!("Post-arm initialization complete (method: {:?})", method);
        Ok(())
    }

    /// Send warning message to GCS
    fn send_warning(&self, message: &str) -> Result<(), &'static str> {
        // Create MAVLink STATUSTEXT message
        let statustext = mavlink::common::STATUSTEXT_DATA {
            severity: mavlink::common::MavSeverity::MAV_SEVERITY_WARNING,
            text: message_to_mavlink_text(message),
            id: 0, // Optional message ID
            chunk_seq: 0, // Optional chunk sequence
        };

        // Send to all connected GCS instances
        self.mavlink_router.broadcast(mavlink::common::MavMessage::STATUSTEXT(statustext))?;

        // Also log locally for console output
        warn!("{}", message);

        Ok(())
    }
}

/// Convert Rust string to MAVLink fixed-size text array
fn message_to_mavlink_text(message: &str) -> [u8; 50] {
    let mut text = [0u8; 50];
    let bytes = message.as_bytes();
    let len = bytes.len().min(50);
    text[..len].copy_from_slice(&bytes[..len]);
    text
}
```

**STATUSTEXT Message Format:**

MAVLink STATUSTEXT message (message ID 253):

```rust
pub struct STATUSTEXT_DATA {
    pub severity: MavSeverity,  // MAV_SEVERITY_WARNING (4)
    pub text: [u8; 50],          // "Arming checks disabled\0\0..."
    pub id: u16,                 // Optional ID (0 if not used)
    pub chunk_seq: u8,           // Optional chunk sequence (0 if not used)
}
```

**Severity Levels:**

- MAV_SEVERITY_EMERGENCY (0): System unusable
- MAV_SEVERITY_ALERT (1): Action must be taken immediately
- MAV_SEVERITY_CRITICAL (2): Critical condition
- MAV_SEVERITY_ERROR (3): Error condition
- **MAV_SEVERITY_WARNING (4)**: Warning condition ← Used for arming checks warning
- MAV_SEVERITY_NOTICE (5): Normal but significant condition
- MAV_SEVERITY_INFO (6): Informational message
- MAV_SEVERITY_DEBUG (7): Debug-level message

**Trigger Conditions:**

Warning sent if any of the following true:

1. **ForceArm method**: Explicitly bypassed checks (method == ArmMethod::ForceArm)
2. **checks_performed = false**: Passed to arm() function
3. **ARMING_CHECK = 0**: All checks disabled via parameter (future enhancement)

**GCS Display:**

Mission Planner and QGroundControl display STATUSTEXT messages in HUD:

- Yellow/orange background for WARNING severity
- Message displayed for 5-10 seconds (GCS-dependent)
- Message also logged to GCS log file

**Failure Handling:**

If warning send fails (no GCS connected, MAVLink error):

- Log warning to console: `warn!("Failed to send arming checks warning: {}")`
- Continue with arm operation (don't fail arm due to warning failure)
- Rationale: Warning is informational, not critical to arm operation

**ArduPilot Comparison:**

ArduPilot sends warning if checks disabled:

```cpp
if (!do_arming_checks) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "Warning: Arming Checks Disabled");
}
```

pico_trail uses same approach: STATUSTEXT with MAV_SEVERITY_WARNING.

## Platform Considerations

### Unix

N/A - Platform agnostic (embedded system)

### Windows

N/A - Platform agnostic (embedded system)

### Cross-Platform

STATUSTEXT message and severity levels standard across all MAVLink implementations. GCS display behavior may vary (Mission Planner vs QGroundControl) but all support STATUSTEXT.

## Risks & Mitigation

| Risk                                                  | Impact | Likelihood | Mitigation                                                                                | Validation                                                   |
| ----------------------------------------------------- | ------ | ---------- | ----------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| Operator ignores warning, arms without checks         | High   | Medium     | Make warning highly visible (WARNING severity, prominent GCS display), log to audit trail | User testing: verify operators notice warning in GCS         |
| Warning not sent (no GCS connected)                   | Medium | Medium     | Log warning locally (console/log file), operator should see before connecting GCS         | Test: arm without GCS, verify local log contains warning     |
| Warning send blocks arm operation (latency)           | Medium | Low        | STATUSTEXT send should be fast (< 1 ms), non-blocking, timeout if GCS not responding      | Benchmark: measure warning send time, verify < 5 ms          |
| Warning message truncated (> 50 bytes)                | Low    | Very Low   | "Arming checks disabled" = 22 bytes, well under 50-byte limit                             | Test: verify message fits in STATUSTEXT text field           |
| Warning sent multiple times (duplicate notifications) | Low    | Low        | Send warning ONCE per arm operation, only in post_arm_init()                              | Test: verify single warning per arm event                    |
| Warning severity wrong (ERROR not WARNING)            | Medium | Very Low   | Use MAV_SEVERITY_WARNING (4), not ERROR (3), to avoid operator panic                      | Code review: verify severity level correct                   |
| GCS doesn't display warning (unsupported message)     | Medium | Very Low   | STATUSTEXT universally supported, all GCS implementations display it                      | Test with multiple GCS: Mission Planner, QGC, verify display |
| Warning text unclear to operator                      | Medium | Low        | Use clear, concise message: "Arming checks disabled", avoid jargon                        | User testing: verify operators understand warning meaning    |

## Implementation Notes

Preferred approaches:

- **Non-blocking send**: Warning failure should NOT prevent arming
- **Standard severity**: Use MAV_SEVERITY_WARNING (4) for consistency with ArduPilot
- **Clear message**: "Arming checks disabled" is concise and unambiguous
- **Broadcast to all GCS**: Ensure all connected GCS instances receive warning

Known pitfalls:

- **Blocking send**: Don't wait indefinitely for MAVLink send, use timeout
- **Silent failures**: Always log locally if GCS send fails (don't fail silently)
- **Wrong severity**: ERROR severity may cause operator panic, use WARNING
- **Message truncation**: STATUSTEXT text field is 50 bytes, ensure message fits

Related code areas:

- `src/communication/mavlink/state.rs` - SystemState post_arm_init() sends warning
- `src/communication/mavlink/router.rs` - MAVLink router broadcast() method
- `src/communication/mavlink/messages/` - STATUSTEXT message construction
- `src/core/logging/` - Local console logging (warn! macro)

## External References

- Analysis: [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
- MAVLink STATUSTEXT: <https://mavlink.io/en/messages/common.html#STATUSTEXT>
- MAVLink MAV_SEVERITY: <https://mavlink.io/en/messages/common.html#MAV_SEVERITY>
- ArduPilot Arming Checks: <https://ardupilot.org/copter/docs/common-prearm-safety-checks.html>
