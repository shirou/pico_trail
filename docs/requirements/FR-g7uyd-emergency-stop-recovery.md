# FR-g7uyd Emergency Stop Operator Acknowledgment and Recovery

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-vt1vs-stop-state-tracking](FR-vt1vs-stop-state-tracking.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall require explicit operator acknowledgment to clear emergency stop, preventing automatic resume and ensuring operator awareness before operation resumes.

## Rationale

Preventing accidental resume after emergency stop is critical safety requirement. Operator must explicitly clear stop via RC switch or GCS command, confirming situation is safe to resume.

## Acceptance Criteria

- [ ] Emergency stop does not auto-clear when trigger removed
- [ ] Operator must explicitly clear via RC switch (low position) or GCS command
- [ ] Clearing stop logged with timestamp and duration
- [ ] Vehicle resumes normal operation only after clear confirmed
- [ ] Cannot clear until Stopped state reached (vehicle fully stopped)
- [ ] Failsafe priority overrides clear (cannot clear during active failsafe)

## Technical Details (if applicable)

**Clear Command**:

- RC switch: Return to low position (< 1300 μs)
- GCS: MAV_CMD_COMPONENT_ARM_DISARM with param1=0.0 (or custom command)

**Clear Conditions**:

- Emergency stop in Stopped state
- No active failsafe (failsafe prevents clear)
- Explicit clear command received

## External References

- Analysis: [AN-4ez27-emergency-stop](../analysis/AN-4ez27-emergency-stop.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
