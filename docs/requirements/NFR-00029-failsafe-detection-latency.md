# NFR-00029 Failsafe Detection Latency

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00053-rc-signal-loss-failsafe](FR-00053-rc-signal-loss-failsafe.md)
  - [FR-00041-gcs-loss-failsafe](FR-00041-gcs-loss-failsafe.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-00167-gcs-failsafe-implementation](../tasks/T-00167-gcs-failsafe-implementation/README.md)

## Requirement Statement

Failsafe detection latency shall not exceed 200ms after timeout threshold crossed, enabling timely protective action before vehicle travels unsafe distance or encounters hazards.

## Rationale

Rapid failsafe detection critical for safety. 200ms target with 10 Hz check rate ensures detection within 2 check cycles, minimizing unsafe operation time.

## Acceptance Criteria

- [ ] Detect RC loss within 200ms of timeout expiration (`FS_TIMEOUT` after RC timeout)
- [ ] Detect GCS loss within 200ms of persistence timeout expiration (`FS_GCS_TIMEOUT + FS_TIMEOUT + 200ms` from last heartbeat, default < 6.7s)
- [ ] Detect battery threshold crossing within 200ms (10 Hz check rate supports this)
- [ ] Target: < 200ms (95th percentile), < 300ms (max) after all timeout stages complete
- [ ] Measured via timestamp difference: timeout expiration -> failsafe trigger

## Technical Details (if applicable)

### Performance

Detection latency < 200ms after each timeout stage completes.

### Check Frequencies

- RC/GCS/Battery: 10 Hz (100ms period) -> \~100ms average latency, 200ms worst case

### GCS Two-Stage Timing (per AN-00046)

Total time from last heartbeat to action execution:

- `FS_GCS_TIMEOUT` (default 5.0s) + `FS_TIMEOUT` (default 1.5s) + detection latency (< 200ms)
- Default total: < 6.7s from last heartbeat

## External References

- Analysis: [AN-00011-failsafe-system](../analysis/AN-00011-failsafe-system.md)
- Analysis: [AN-00046-communication-lost-action](../analysis/AN-00046-communication-lost-action.md)
