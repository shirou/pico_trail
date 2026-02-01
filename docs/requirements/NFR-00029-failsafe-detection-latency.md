# NFR-00029 Failsafe Detection Latency

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00053-rc-signal-loss-failsafe](FR-00053-rc-signal-loss-failsafe.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Failsafe detection latency shall not exceed 200ms after timeout threshold crossed, enabling timely protective action before vehicle travels unsafe distance or encounters hazards.

## Rationale

Rapid failsafe detection critical for safety. 200ms target with 10 Hz check rate ensures detection within 2 check cycles, minimizing unsafe operation time.

## Acceptance Criteria

- [ ] Detect RC loss within 200ms of timeout expiration (1.5s + 0.2s)
- [ ] Detect GCS loss within 200ms of timeout expiration (5.0s + 0.2s)
- [ ] Detect battery threshold crossing within 200ms (10 Hz check rate supports this)
- [ ] Target: < 200ms (95th percentile), < 300ms (max)
- [ ] Measured via timestamp difference: timeout expiration → failsafe trigger

## Technical Details (if applicable)

**Performance**: Detection latency < 200ms

**Check Frequencies**:

- RC/GCS/Battery: 10 Hz (100ms period) → \~100ms average latency, 200ms worst case

## External References

- Analysis: [AN-00011-failsafe-system](../analysis/AN-00011-failsafe-system.md)
