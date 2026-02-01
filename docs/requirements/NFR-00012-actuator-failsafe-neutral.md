# NFR-00012 Actuator Fail-Safe to Neutral Position

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00012-actuator-abstraction](FR-00012-actuator-abstraction.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Actuators shall fail-safe to neutral position (steering centered at 1500 μs, throttle zero at 1500 μs) when disarmed or RC lost, within 1 control loop cycle (20ms), with armed check enforcement at actuator layer providing defense in depth, and automated tests verifying immediate stop during full throttle disarm.

## Rationale

Primary safety mechanism preventing unintended vehicle motion. Fail-safe to neutral is critical safety requirement - vehicle must never move when disarmed. Actuator layer enforcement ensures safety even if mode layer fails.

## User Story (if applicable)

The system shall immediately stop all motors and center steering when disarmed, regardless of mode commands, preventing any unintended vehicle motion.

## Acceptance Criteria

- [ ] Actuators output neutral (1500 μs PWM) when disarmed
- [ ] Actuators output neutral when RC lost (> 1 second no RC_CHANNELS)
- [ ] Neutral output within 1 control loop cycle (20ms at 50 Hz)
- [ ] Armed check enforced at actuator layer (final safety gate)
- [ ] Automated test: Disarm during full throttle, verify immediate stop
- [ ] Neutral position: Steering=0.0, Throttle=0.0 (both → 1500 μs PWM)

## Technical Details (if applicable)

**Safety Requirement**: CRITICAL

**Performance**: Neutral output < 20ms

**Enforcement**: Multi-layer

1. Mode layer checks armed state
2. Actuator layer independently checks armed state (final gate)
3. Disarm event triggers immediate neutral output

**Test Procedure**:

```rust
#[test]
fn test_disarm_stops_motors() {
    let mut actuators = Actuators::new();
    system_state.arm();

    // Command full throttle
    actuators.set_throttle(1.0);
    assert_eq!(actuators.get_pwm_output(), 2000); // Full throttle PWM

    // Disarm
    system_state.disarm();
    actuators.on_disarm_event();

    // Verify immediate neutral
    assert_eq!(actuators.get_pwm_output(), 1500); // Neutral PWM
}
```

## Platform Considerations

### Cross-Platform

Fail-safe behavior applies to all platforms.

## Risks & Mitigation

| Risk                                         | Impact       | Likelihood | Mitigation                                                                |
| -------------------------------------------- | ------------ | ---------- | ------------------------------------------------------------------------- |
| **Disarm event missed, motors keep running** | **CRITICAL** | **Low**    | **Subscribe to disarm events, immediate neutral output, automated tests** |
| Actuator layer bypassed                      | CRITICAL     | Very Low   | Actuator layer is only path to PWM hardware                               |

## Implementation Notes

- Actuator layer is final enforcement point - no bypass possible
- Subscribe to armed state changes
- Output neutral immediately on disarm event
- Automated safety tests run in CI

Related code areas:

- `src/vehicle/actuators.rs`

## External References

- Analysis: [AN-00007-manual-control-implementation](../analysis/AN-00007-manual-control-implementation.md)
