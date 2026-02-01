# NFR-00090 Mission Command Execution Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00042-mission-execution-gaps](../analysis/AN-00042-mission-execution-gaps.md)
- Related ADRs:
  - [ADR-00034-mission-execution-telemetry-architecture](../adr/ADR-00034-mission-execution-telemetry-architecture.md)
- Prerequisite Requirements:
  - [FR-00139-nav-do-command-separation](FR-00139-nav-do-command-separation.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-00037-mission-execution-sequencer](../tasks/T-00037-mission-execution-sequencer/README.md)

## Requirement Statement

DO command processing during mission execution shall complete within a single navigation update cycle (< 20ms) to avoid disrupting navigation timing.

## Rationale

DO commands execute synchronously within the Auto mode update loop. If DO command processing takes too long, it delays the navigation update, causing:

- Missed navigation update cycles (jitter in steering)
- Degraded control quality
- Timing budget violation (per NFR-00070 50Hz navigation rate)

Since DO commands are typically parameter changes (speed, servo position), they should be near-instantaneous.

## User Story (if applicable)

The system shall process DO commands within 20ms to ensure navigation update timing is not disrupted.

## Acceptance Criteria

- [ ] DO command execution completes within 20ms (single update cycle)
- [ ] Navigation update rate (50Hz) not degraded by DO command processing
- [ ] Multiple DO commands between NAV commands processed without timing violation
- [ ] Measured via profiling or debug timing on target hardware

## Technical Details (if applicable)

### Non-Functional Requirement Details

- Performance: DO command execution < 20ms per command
- Reliability: DO commands must not block the navigation loop
- Compatibility: Must not violate NFR-00070 (50Hz navigation update rate)

**Timing Budget:**

The navigation update cycle is 20ms total. DO command processing shares this budget:

| Operation      | Budget | Notes                       |
| -------------- | ------ | --------------------------- |
| NAV processing | 2ms    | Existing navigation calc    |
| DO processing  | 1ms    | Speed change, servo command |
| Margin         | 17ms   | Available headroom          |

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                      | Impact | Likelihood | Mitigation                      | Validation                   |
| ------------------------- | ------ | ---------- | ------------------------------- | ---------------------------- |
| Complex DO command blocks | Medium | Low        | All supported DOs are immediate | Profile on target            |
| Many DO commands in batch | Low    | Low        | Limit per-cycle DO processing   | Test with dense DO sequences |

## Implementation Notes

Preferred approaches:

- Execute DO commands synchronously in Auto mode `update()`
- Avoid async operations for DO command execution
- Profile on RP2350 hardware to verify timing

Related code areas:

- `crates/firmware/src/rover/mode/auto.rs` - Auto mode update loop

## External References

- [ArduPilot DO Command Execution](https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_auto.cpp)
