# NFR-00179 Home Auto-Set Latency

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00171-home-auto-set-gps-fix](FR-00171-home-auto-set-gps-fix.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00047-home-position-management](../analysis/AN-00047-home-position-management.md)
- Related Tasks:
  - [T-00181-home-position-management](../tasks/T-00181-home-position-management/README.md)

## Requirement Statement

Home position auto-set shall complete within 1ms of detecting a valid GPS fix in the control loop iteration. This includes reading the GPS position, setting home in SystemState, and queuing the HOME_POSITION broadcast.

## Rationale

The home auto-set operation consists of a single GPS position read and a memory write to SystemState, which are trivially fast operations. The 1ms budget is generous and ensures the operation does not impact the 50 Hz control loop timing (20ms cycle budget). This is a correctness guarantee rather than a tight performance constraint.

## User Story

The system shall ensure home auto-set does not cause control loop jitter or timing delays, so that vehicle control remains stable during initial GPS acquisition.

## Acceptance Criteria

- [ ] Home auto-set (GPS read + state write + broadcast queue) completes within 1ms
- [ ] No measurable impact on control loop timing at 50 Hz
- [ ] No heap allocations during auto-set operation

## Technical Details

### Non-Functional Requirement Details

- **Operation**: Read GPS position fields, write to HomePosition struct, queue broadcast message
- **Expected duration**: < 100 microseconds (well within 1ms budget)
- **Memory**: No allocations; HomePosition is stack-allocated and copied to SystemState
- **Control loop impact**: Negligible -- single conditional check + struct copy per iteration

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                                        | Impact | Likelihood | Mitigation                                           | Validation      |
| ------------------------------------------- | ------ | ---------- | ---------------------------------------------------- | --------------- |
| Critical section contention delays set_home | Low    | Very Low   | Critical section holds only for struct copy duration | Measure in SITL |

## Implementation Notes

- No special optimization needed; the operation is inherently fast
- Can be verified by timing the control loop with and without home auto-set logic

## External References

N/A
