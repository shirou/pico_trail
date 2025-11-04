# NFR-az0iv Failsafe Action Execution Time

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-0wy2c-failsafe-action-priority](FR-0wy2c-failsafe-action-priority.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Failsafe action execution shall complete within 1 second of trigger, including mode change to Hold/RTL and initial actuator response, ensuring rapid vehicle response to failure conditions.

## Rationale

Rapid action execution minimizes unsafe operation duration. 1 second target includes mode transition, actuator command, and PWM update, providing timely protective response.

## Acceptance Criteria

- [ ] Mode change + initial actuator response < 1 second
- [ ] Includes: failsafe detection → mode switch → actuator command → PWM output
- [ ] Target: < 1s (95th percentile), < 1.5s (max)
- [ ] Measured and logged for every failsafe activation

## Technical Details (if applicable)

**Performance**: Action execution < 1 second

## External References

- Analysis: [AN-kajh6-failsafe-system](../analysis/AN-kajh6-failsafe-system.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
