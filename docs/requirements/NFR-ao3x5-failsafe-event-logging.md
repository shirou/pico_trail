# NFR-ao3x5 Failsafe Event Logging

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-lusbw-rc-signal-loss-failsafe](FR-lusbw-rc-signal-loss-failsafe.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Failsafe events shall be logged to persistent storage with timestamp, type, action taken, and duration, enabling post-flight analysis and safety investigations.

## Rationale

Failsafe logs critical for debugging communication issues, tuning timeouts, and safety investigations. Persistent logs enable analysis after power cycle.

## Acceptance Criteria

- [ ] Log all failsafe triggers with type (RC/GCS/Battery)
- [ ] Include timestamp, action executed, vehicle context
- [ ] Log failsafe recovery events with duration
- [ ] Logs persist across reboot
- [ ] Logs accessible via MAVLink or storage download

## Technical Details (if applicable)

**Log Format**:

```
FAILSAFE: timestamp=45678ms, type=RcLoss, action=Hold,
          battery=11.4V, duration=8.2s
```

## External References

- Analysis: [AN-kajh6-failsafe-system](../analysis/AN-kajh6-failsafe-system.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
