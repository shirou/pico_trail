# FR-xrlkn Failsafe Recovery When Condition Clears

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-lusbw-rc-signal-loss-failsafe](FR-lusbw-rc-signal-loss-failsafe.md)
  - [FR-zwsr5-gcs-loss-failsafe](FR-zwsr5-gcs-loss-failsafe.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall clear RC and GCS failsafes when signal resumes (within timeout period), send recovery notification, but shall not automatically switch back to previous mode to allow operator manual control.

## Rationale

Automatic failsafe recovery when signal returns allows mission continuation. However, not switching back to previous mode prevents unexpected behavior - operator must manually choose next mode after recovery.

## Acceptance Criteria

- [ ] Clear RC failsafe when RC_CHANNELS messages resume (within `FS_TIMEOUT`)
- [ ] Clear GCS failsafe when HEARTBEAT messages resume (within `FS_GCS_TIMEOUT`)
- [ ] Send recovery notification: "Failsafe: RC Recovered" / "GCS Recovered"
- [ ] Do not automatically switch back to previous mode
- [ ] Battery failsafe does not auto-clear (voltage recovery too slow)
- [ ] Log failsafe recovery event with duration
- [ ] Hysteresis: Signal stable for 1 second before clearing

## Technical Details (if applicable)

**Recovery Conditions**:

- RC: RC_CHANNELS arriving within timeout for 1+ seconds
- GCS: HEARTBEAT arriving within timeout for 1+ seconds
- Battery: No auto-clear (operator must manually change mode after landing/RTL)

**Post-Recovery**:

- Vehicle remains in current mode (Hold/RTL)
- Operator manually switches to desired mode

## External References

- Analysis: [AN-kajh6-failsafe-system](../analysis/AN-kajh6-failsafe-system.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
