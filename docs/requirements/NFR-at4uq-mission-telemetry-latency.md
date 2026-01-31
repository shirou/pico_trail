# NFR-at4uq Mission Telemetry Latency

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-l8emi-mission-execution-gaps](../analysis/AN-l8emi-mission-execution-gaps.md)
- Related ADRs:
  - [ADR-0yapl-mission-execution-telemetry-architecture](../adr/ADR-0yapl-mission-execution-telemetry-architecture.md)
- Prerequisite Requirements:
  - [FR-sifsm-mission-current-telemetry](FR-sifsm-mission-current-telemetry.md)
  - [FR-zcnqw-mission-item-reached](FR-zcnqw-mission-item-reached.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-r7k2p-mission-execution-sequencer](../tasks/T-r7k2p-mission-execution-sequencer/README.md)

## Requirement Statement

MISSION_ITEM_REACHED shall be sent within 100ms of waypoint arrival detection, and MISSION_CURRENT shall be sent within 100ms of waypoint index change.

## Rationale

GCS must receive timely mission progress updates for:

- Accurate progress display (no visible lag between rover reaching waypoint and GCS update)
- Logging accuracy (timestamps reflect actual arrival times)
- Operator confidence (delayed updates create confusion about rover state)

100ms is imperceptible to operators and well within the telemetry system's capability.

## User Story (if applicable)

The system shall deliver mission progress messages within 100ms of the triggering event to ensure responsive GCS feedback.

## Acceptance Criteria

- [ ] MISSION_ITEM_REACHED sent within 100ms of waypoint arrival detection
- [ ] MISSION_CURRENT (event-triggered) sent within 100ms of waypoint index change
- [ ] MISSION_CURRENT (periodic) maintains 1Hz ± 0.1Hz rate
- [ ] Latency measured from detection event to message queued for send

## Technical Details (if applicable)

### Non-Functional Requirement Details

- Performance: Message emission latency < 100ms from trigger event
- Reliability: Messages must not be dropped under normal operation
- Compatibility: Timing requirements are well within MAVLink protocol expectations

**Measurement Method:**

Measure time between `at_target` detection in Auto mode and message appearance in the send queue. Can be validated via `defmt` timestamps on embedded target.

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                  | Impact | Likelihood | Mitigation                        | Validation                 |
| --------------------- | ------ | ---------- | --------------------------------- | -------------------------- |
| Send queue delay      | Low    | Low        | Use priority queue or direct send | Profile send queue latency |
| Task scheduling delay | Low    | Low        | Embassy task priority sufficient  | Measure on target hardware |

## Implementation Notes

Related code areas:

- `crates/firmware/src/rover/mode/auto.rs` - Waypoint detection
- `crates/firmware/src/communication/mavlink/handlers/telemetry.rs` - Message streaming

## External References

- [MAVLink MISSION_CURRENT](https://mavlink.io/en/messages/common.html#MISSION_CURRENT)
- [MAVLink MISSION_ITEM_REACHED](https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED)
