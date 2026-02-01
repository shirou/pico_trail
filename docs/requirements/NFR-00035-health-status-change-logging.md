# NFR-00035 Health Status Change Logging

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00057-system-health-status-tracking](FR-00057-system-health-status-tracking.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Health status changes shall be logged to persistent storage with timestamp, subsystem, old state, new state, and vehicle context, enabling post-flight health analysis and troubleshooting.

## Rationale

Health transition logs support debugging sensor issues, identifying degradation patterns, and validating monitoring system behavior. Persistent logs enable analysis after power cycle.

## Acceptance Criteria

- [ ] Log all health transitions (Healthy ↔ Warning ↔ Unhealthy)
- [ ] Include timestamp, subsystem name, old state, new state
- [ ] Include vehicle context: mode, battery voltage, RC signal status
- [ ] Logs persist across reboot
- [ ] Logs accessible via MAVLink or storage download

## Technical Details (if applicable)

**Log Format**:

```
HEALTH: timestamp=23456ms, subsystem=RC, transition=Healthy→Warning,
        mode=Manual, battery=11.6V
```

## External References

- Analysis: [AN-00009-armed-state-monitoring](../analysis/AN-00009-armed-state-monitoring.md)
