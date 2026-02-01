# NFR-00026 Emergency Stop Initiation Latency

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00023-controlled-emergency-stop](FR-00023-controlled-emergency-stop.md)
  - [FR-00033-emergency-stop-triggers](FR-00033-emergency-stop-triggers.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Emergency stop shall initiate within 100ms of trigger detection (RC switch, GCS command, or failsafe condition), ensuring rapid response to emergency situations.

## Rationale

Rapid emergency stop initiation is critical for safety. 100ms target ensures the vehicle begins deceleration quickly after trigger detected, minimizing distance traveled before stop begins.

## User Story (if applicable)

The system shall detect emergency stop triggers and begin stop sequence within 100ms to ensure timely vehicle response and minimize unsafe operation time.

## Acceptance Criteria

- [ ] RC switch trigger → stop initiation < 100ms
- [ ] GCS command trigger → stop initiation < 100ms
- [ ] Failsafe trigger → stop initiation < 100ms
- [ ] Measured via timestamp difference: trigger detection → first stop throttle command
- [ ] Automated test measures and reports actual latency
- [ ] Latency logged for post-flight analysis
- [ ] Target: < 100ms (95th percentile), < 150ms (max)

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance**: Latency < 100ms

**Latency Breakdown**:

- Trigger detection: < 20ms (control loop cycle)
- Stop initiation: < 10ms (state machine transition)
- Actuator command: < 20ms (next control loop)
- PWM update: < 20ms (50 Hz PWM cycle)
- **Total**: \~70ms typical, < 100ms worst case

**Measurement**:

```rust
let trigger_time = get_time_ms();
emergency_stop.start_controlled_stop(trigger);
let initiation_time = get_time_ms();
let latency_ms = initiation_time - trigger_time;
log("Emergency stop latency: {} ms", latency_ms);
```

## Platform Considerations

### Cross-Platform

Latency target applies to all platforms. RP2040/RP2350 at 133 MHz should meet < 100ms easily with 50 Hz control loop.

## Risks & Mitigation

| Risk                              | Impact | Likelihood | Mitigation                                         |
| --------------------------------- | ------ | ---------- | -------------------------------------------------- |
| Latency exceeds 100ms under load  | High   | Low        | Profile control loop, ensure < 20ms execution time |
| Interrupt handling delays trigger | Medium | Low        | Use high-priority task for emergency stop          |

## Implementation Notes

- Measure actual latency in system tests
- Log latency for every emergency stop event
- Alert if latency > 100ms threshold exceeded

Related code areas:

- `src/vehicle/emergency_stop/manager.rs`

## External References

- Analysis: [AN-00010-emergency-stop](../analysis/AN-00010-emergency-stop.md)
