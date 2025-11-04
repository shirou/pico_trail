# FR-lusbw RC Signal Loss Failsafe

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
  - [FR-knp1u-rc-input-health-monitoring](FR-knp1u-rc-input-health-monitoring.md)
  - [FR-sp3at-vehicle-modes](FR-sp3at-vehicle-modes.md)

- Dependent Requirements:
  - [FR-0wy2c-failsafe-action-priority](FR-0wy2c-failsafe-action-priority.md)
  - [FR-mg2bv-failsafe-integration](FR-mg2bv-failsafe-integration.md)
  - [FR-xrlkn-failsafe-recovery](FR-xrlkn-failsafe-recovery.md)
  - [NFR-ao3x5-failsafe-event-logging](NFR-ao3x5-failsafe-event-logging.md)
  - [NFR-df1qu-failsafe-detection-latency](NFR-df1qu-failsafe-detection-latency.md)
  - [NFR-qp6ya-no-false-failsafe-triggers](NFR-qp6ya-no-false-failsafe-triggers.md)
  - [NFR-sm5dx-failsafe-ram-overhead](NFR-sm5dx-failsafe-ram-overhead.md)

- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall detect RC signal loss within 1.5 seconds of the last RC_CHANNELS message and trigger a configured failsafe action (Hold, RTL, or Disarm), sending notification to the GCS and logging the event.

## Rationale

RC loss is the most common failsafe scenario during testing and operation. Loss of RC signal means the operator cannot override the autopilot, requiring automated protective action. ArduPilot uses 1.5 second timeout as proven field-tested default balancing false trigger avoidance with timely response.

## User Story (if applicable)

As an operator, I want the vehicle to automatically enter a safe mode (Hold or RTL) when RC signal is lost for more than 1.5 seconds, so that I can recover control when signal returns without the vehicle continuing unsafe operation.

## Acceptance Criteria

- [ ] Monitor RC_CHANNELS message arrival time continuously
- [ ] Trigger failsafe if no RC_CHANNELS received within `FS_TIMEOUT` seconds (default 1.5s)
- [ ] Execute configured failsafe action from `FS_ACTION` parameter (Hold, RTL, SmartRTL, or Disarm)
- [ ] Send STATUSTEXT message: "Failsafe: RC Lost" with severity WARNING
- [ ] Log failsafe event with timestamp, trigger type, and action taken
- [ ] Clear failsafe automatically when RC signal resumes (RC_CHANNELS messages arriving within timeout)
- [ ] Hysteresis: Require RC signal stable for 1 second before clearing failsafe
- [ ] Update RC timestamp on every RC_CHANNELS message received from any transport

## Technical Details (if applicable)

### Functional Requirement Details

**Detection Logic**:

```rust
fn check_rc_failsafe(
    last_rc_message_ms: u32,
    current_time_ms: u32,
    fs_timeout: f32,
) -> bool {
    let age_ms = current_time_ms - last_rc_message_ms;
    let timeout_ms = (fs_timeout * 1000.0) as u32;
    age_ms > timeout_ms
}
```

**Failsafe Actions** (via `FS_ACTION` parameter):

- 0 = None: No action (warning only)
- 1 = Hold: Stop vehicle and hold position
- 2 = RTL: Return to launch point
- 3 = SmartRTL: Return via intelligent path
- 4 = SmartRTL_Hold: Try SmartRTL, fallback to Hold
- 5 = Disarm: Immediate disarm (emergency only)

**Recovery Conditions**:

- RC_CHANNELS messages arriving within `FS_TIMEOUT`
- Signal stable for 1 second (hysteresis prevents flapping)
- Send STATUSTEXT: "Failsafe: RC Recovered"
- Do not automatically switch back to previous mode (operator must manually switch)

**ArduPilot Parameters**:

- `FS_TIMEOUT`: RC failsafe timeout in seconds (float, default 1.5, range 0.0-10.0)
- `FS_THR_ENABLE`: Enable RC failsafe (bool, default true)
- `FS_ACTION`: RC failsafe action (u8, default 1=Hold)

## Platform Considerations

### Cross-Platform

RC timeout detection is platform-independent. Uses system time from platform abstraction.

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                                     | Validation                                   |
| ----------------------------------------- | ------ | ---------- | -------------------------------------------------------------- | -------------------------------------------- |
| False trigger from brief signal dropout   | Medium | Medium     | 1.5s timeout allows transient dropouts, test with real RC gear | Field testing with RC range test             |
| Late detection (> 1.5s after signal loss) | High   | Low        | 10 Hz failsafe check rate ensures < 200ms detection latency    | Measure actual detection time in tests       |
| RC signal returns during failsafe action  | Low    | Medium     | Complete current action before checking recovery               | Test RC signal recovery during Hold/RTL mode |

## Implementation Notes

- Follow ArduPilot defaults exactly (1.5s timeout, Hold action)
- Check RC timestamp age at 10 Hz (100ms interval)
- Integrate with mode manager for Hold/RTL mode transitions
- Log RC timestamp on every RC_CHANNELS message

Related code areas:

- `src/vehicle/failsafe/checkers/rc_loss.rs` - RC timeout detection
- `src/communication/mavlink/handlers/telemetry.rs` - RC_CHANNELS timestamp update

## External References

- ArduPilot RC Failsafe: <https://ardupilot.org/rover/docs/apms-failsafe-function.html>
- ArduPilot Rover Parameters: <https://ardupilot.org/rover/docs/parameters.html>
- Analysis: [AN-kajh6-failsafe-system](../analysis/AN-kajh6-failsafe-system.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
