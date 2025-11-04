# NFR-z9g39 High-Frequency Monitor Failure Detection Time

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-knp1u-rc-input-health-monitoring](FR-knp1u-rc-input-health-monitoring.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

High-frequency monitors shall detect failures within 200ms of occurrence, enabling rapid failsafe response before vehicle enters unsafe state or travels significant distance.

## Rationale

Critical failures (RC loss, IMU failure) require rapid detection. 200ms target with 50 Hz check rate ensures detection within 10 check cycles, minimizing unsafe operation time.

## Acceptance Criteria

- [ ] RC loss detected within 200ms (50 Hz check rate supports this)
- [ ] IMU failure detected within 200ms
- [ ] Target: < 200ms (95th percentile), < 300ms (max)
- [ ] Measured via timestamp difference: failure occurrence → detection

## Technical Details (if applicable)

**Performance**: Detection latency < 200ms

**Check Frequency**: 50 Hz (20ms period) → \~100ms average latency, 200ms worst case (10 cycles)

## External References

- Analysis: [AN-dgpck-armed-state-monitoring](../analysis/AN-dgpck-armed-state-monitoring.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
