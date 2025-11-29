# NFR-5utah GPS Telemetry Performance

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-uyie8-gps-mavlink-telemetry](../requirements/FR-uyie8-gps-mavlink-telemetry.md)
- Related Tasks:
  - [T-gvsxb-gps-state-telemetry](../tasks/T-gvsxb-gps-state-telemetry/README.md)

## Requirement Statement

GPS telemetry message generation shall not increase control loop latency by more than 1ms.

## Rationale

The control loop operates at 50Hz (20ms period) and must maintain consistent timing for stable vehicle control. GPS telemetry runs in a separate task but shares data structures with the control loop. Excessive latency in data access or message building could impact control responsiveness.

## User Story (if applicable)

The system shall maintain control loop timing within 1ms overhead to ensure stable vehicle operation during GPS telemetry transmission.

## Acceptance Criteria

- [ ] GPS state access from telemetry handler completes within 100us
- [ ] GPS_RAW_INT message building completes within 500us
- [ ] GLOBAL_POSITION_INT message building completes within 500us
- [ ] Unit conversion functions complete within 10us each
- [ ] No blocking operations in GPS state access path
- [ ] Total GPS telemetry overhead < 1ms per cycle

## Technical Details (if applicable)

### Non-Functional Requirement Details

- Performance:
  - GPS state read: < 100us (single atomic read or short mutex lock)
  - Message building: < 500us per message (unit conversion, struct population)
  - Total per telemetry cycle: < 1ms
- Reliability:
  - Use try_lock if mutex contention possible
  - Graceful handling of lock failure (skip message, retry next cycle)
- Compatibility:
  - Same performance characteristics on RP2040 and RP2350

## Platform Considerations

### Embedded (RP2350)

- Cortex-M33 at 150MHz provides ample performance
- FPU available for floating-point conversions
- Embassy async runtime handles task scheduling

### Host Tests

- Benchmark message building functions
- Profile unit conversion functions

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                              | Validation                    |
| ---------------------------------- | ------ | ---------- | --------------------------------------- | ----------------------------- |
| Mutex contention with control loop | High   | Low        | Use try_lock, separate GPS state        | Stress test concurrent access |
| Slow floating-point operations     | Medium | Low        | Use FPU, consider fixed-point if needed | Benchmark on target           |
| Message building allocation        | Medium | Low        | Pre-allocate message buffers, no heap   | Code review, static analysis  |

## Implementation Notes

- Measure actual latency during development using defmt timestamps
- Consider caching converted values if same GPS data used multiple times
- Profile on actual hardware, not just host tests

## External References

N/A - No external references

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
