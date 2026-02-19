# FR-00038 Failsafe Recovery When Condition Clears

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00053-rc-signal-loss-failsafe](FR-00053-rc-signal-loss-failsafe.md)
  - [FR-00041-gcs-loss-failsafe](FR-00041-gcs-loss-failsafe.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-00167-gcs-failsafe-implementation](../tasks/T-00167-gcs-failsafe-implementation/README.md)

## Requirement Statement

The system shall clear RC and GCS failsafes when signal resumes, send recovery notification, but shall not automatically switch back to previous mode to allow operator manual control.

## Rationale

Automatic failsafe recovery when signal returns allows mission continuation. However, not switching back to previous mode prevents unexpected behavior -- operator must manually choose next mode after recovery. ArduPilot follows this pattern for GCS failsafe recovery.

## User Story (if applicable)

N/A

## Acceptance Criteria

### RC Failsafe Recovery

- [ ] Clear RC failsafe when RC_CHANNELS messages resume (within `FS_TIMEOUT`)
- [ ] Hysteresis: RC signal stable for 1 second before clearing

### GCS Failsafe Recovery

- [ ] Clear GCS failsafe immediately when HEARTBEAT messages resume (no hysteresis, matching ArduPilot behavior)
- [ ] Clear `failsafe_active` and `condition_detected` flags on heartbeat reception

### Common Recovery Behavior

- [ ] Send recovery notification: "Failsafe: RC Recovered" / "GCS Failsafe Cleared"
- [ ] Do not automatically switch back to previous mode
- [ ] Log failsafe recovery event with duration
- [ ] Battery failsafe does not auto-clear (voltage recovery too slow)

## Technical Details (if applicable)

### Recovery Conditions

- **RC**: RC_CHANNELS arriving within timeout for 1+ seconds (hysteresis)
- **GCS**: HEARTBEAT arriving within timeout -- immediate clear, no hysteresis (ArduPilot pattern per [AN-00046](../analysis/AN-00046-communication-lost-action.md))
- **Battery**: No auto-clear (operator must manually change mode after landing/RTL)

### Post-Recovery

- Vehicle remains in current mode (Hold/RTL)
- Operator manually switches to desired mode

## External References

- Analysis: [AN-00011-failsafe-system](../analysis/AN-00011-failsafe-system.md)
- Analysis: [AN-00046-communication-lost-action](../analysis/AN-00046-communication-lost-action.md)
