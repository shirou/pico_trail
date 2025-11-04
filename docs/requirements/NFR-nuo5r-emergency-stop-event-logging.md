# NFR-nuo5r Emergency Stop Event Logging

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-cxts2-controlled-emergency-stop](FR-cxts2-controlled-emergency-stop.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Emergency stop events shall be logged to persistent storage with timestamp, trigger source, tier, duration, and vehicle context (velocity, mode, battery voltage), enabling post-incident analysis.

## Rationale

Emergency stop logs critical for safety investigations, debugging, and performance tuning. Rich context enables root cause analysis and system improvement.

## Acceptance Criteria

- [ ] Log emergency stop activation with timestamp
- [ ] Include trigger source, stop tier selected
- [ ] Include vehicle context: velocity, mode, battery voltage, RC signal status
- [ ] Log stop completion time (duration)
- [ ] Log stop recovery/clear event
- [ ] Logs persist across reboot
- [ ] Logs accessible via MAVLink or storage download

## Technical Details (if applicable)

**Log Format**:

```
ESTOP: timestamp=12345ms, trigger=RcSwitch, tier=Controlled, velocity=1.2m/s,
       mode=Manual, battery=11.8V, duration=2.8s
```

## External References

- Analysis: [AN-4ez27-emergency-stop](../analysis/AN-4ez27-emergency-stop.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
