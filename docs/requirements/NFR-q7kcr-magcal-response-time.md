# NFR-q7kcr MagCal Response Time

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-fgiit-compass-calibration-via-mission-planner](../analysis/AN-fgiit-compass-calibration-via-mission-planner.md)
- Prerequisite Requirements:
  - [FR-v8fmy-fixed-mag-cal-yaw-handler](FR-v8fmy-fixed-mag-cal-yaw-handler.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-k9mcl-compass-calibration](../tasks/T-k9mcl-compass-calibration/README.md)

## Requirement Statement

The system shall respond to `MAV_CMD_FIXED_MAG_CAL_YAW` commands within 1 second, including GPS validation and COMMAND_ACK transmission.

## Rationale

- **User Experience**: Immediate feedback confirms the calibration action was received
- **GCS Timeout**: Ground control stations typically timeout waiting for command acknowledgment after several seconds
- **Simple Validation**: Phase 1 implementation performs only GPS fix check - no complex calculations
- **Protocol Compliance**: MAVLink command/response pattern expects timely acknowledgment

## User Story (if applicable)

The system shall respond to calibration commands within 1 second to ensure operators receive immediate feedback on calibration success or failure.

## Acceptance Criteria

- [ ] COMMAND_ACK response sent within 1000ms of receiving MAV_CMD_FIXED_MAG_CAL_YAW
- [ ] STATUSTEXT message sent within 1000ms of command reception
- [ ] GPS state access does not block for more than 100ms
- [ ] Response time consistent regardless of success or failure outcome

## Technical Details (if applicable)

### Non-Functional Requirement Details

- **Performance**: Command-to-ACK latency < 1000ms
- **Category**: Response Time / Latency

**Timing Breakdown (Phase 1):**

| Operation                | Expected Time |
| ------------------------ | ------------- |
| Command parsing          | < 1ms         |
| GPS state mutex acquire  | < 10ms        |
| GPS fix validation       | < 1ms         |
| STATUSTEXT transmission  | < 10ms        |
| COMMAND_ACK transmission | < 10ms        |
| **Total**                | **< 50ms**    |

**Note**: Phase 1 implementation is simple validation only. Phase 2 (WMM-based calibration) may require different timing requirements due to calculation overhead.

**Measurement Method:**

```rust
let start = Instant::now();
let result = handle_fixed_mag_cal_yaw(cmd);
let elapsed = start.elapsed();
assert!(elapsed < Duration::from_secs(1));
```

## Platform Considerations

### Cross-Platform

- Same timing requirements on all platforms
- GPS mutex performance may vary by platform

## Risks & Mitigation

| Risk                          | Impact | Likelihood | Mitigation                        | Validation                 |
| ----------------------------- | ------ | ---------- | --------------------------------- | -------------------------- |
| GPS mutex contention          | Low    | Low        | Use try_lock with timeout         | Test under GPS update load |
| STATUSTEXT queue full         | Low    | Low        | Non-blocking send, log warning    | Test with message flooding |
| Future WMM calculation slower | Medium | Medium     | Document separate NFR for Phase 2 | N/A for Phase 1            |

## Implementation Notes

- Phase 1 implementation should easily meet this requirement (simple validation)
- Add timing instrumentation during development to verify
- Consider adding command processing latency to telemetry for monitoring

## External References

- [MAVLink Command Protocol](https://mavlink.io/en/services/command.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
