# NFR-u885h RTL Navigation Update Rate

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-lia7r-rtl-navigate-home](FR-lia7r-rtl-navigate-home.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-75408-rtl-mode](../analysis/AN-75408-rtl-mode.md)
- Related Tasks:
  - [T-bo6xc-rtl-smartrtl-implementation](../tasks/T-bo6xc-rtl-smartrtl-implementation/README.md)

## Requirement Statement

RTL mode navigation shall update at the control loop rate of 50 Hz (every 20ms), ensuring smooth and responsive steering and throttle adjustments during return-to-home navigation.

## Rationale

Consistent navigation updates are critical for:

- Smooth steering adjustments (no jerky movements)
- Responsive obstacle reaction (if sensors added later)
- Accurate arrival detection
- Consistent behavior regardless of CPU load

50 Hz matches the overall control loop rate used by other modes.

## User Story

The system shall maintain 50 Hz navigation updates to ensure smooth and responsive RTL navigation behavior.

## Acceptance Criteria

- [ ] RTL update() called every 20ms by scheduler
- [ ] update() execution time < 1ms (leaves headroom)
- [ ] Navigation calculations complete within single update cycle
- [ ] No update rate degradation under normal operation
- [ ] Matches control loop rate of Manual and other modes

## Technical Details

### Non-Functional Requirement Details

**Performance Targets:**

- Update Rate: 50 Hz (20ms period)
- Maximum update() latency: 1ms
- Jitter tolerance: +/- 2ms

**Measurement Method:**

- Instrument update() with timing
- Log any updates exceeding 1ms
- Monitor via health telemetry

**Dependencies:**

- Scheduler must call mode update() at 50 Hz
- GPS updates (1-10 Hz) are cached, not blocking
- NavigationController calculations are lightweight

## Platform Considerations

N/A - Platform agnostic timing requirements

## Risks & Mitigation

| Risk                         | Impact | Likelihood | Mitigation                  | Validation                |
| ---------------------------- | ------ | ---------- | --------------------------- | ------------------------- |
| Update rate drops under load | Medium | Low        | Lightweight update() design | Load testing              |
| GPS blocking causes delay    | Low    | Low        | GPS read is async/cached    | Verify async GPS handling |

## Implementation Notes

- update() should avoid blocking operations
- GPS position is read from cached SYSTEM_STATE
- NavigationController update() is computationally simple
- Consider: Timing metrics for update() latency

## External References

- [ArduPilot Scheduling](https://ardupilot.org/dev/docs/code-overview-scheduling-your-new-code-to-run-intermittently.html) - Scheduler design reference
