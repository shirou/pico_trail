# FR-00023 Controlled Emergency Stop with Gradual Deceleration

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00062-control-modes](FR-00062-control-modes.md)
  - [FR-00012-actuator-abstraction](FR-00012-actuator-abstraction.md)

- Dependent Requirements:
  - [FR-00056-stop-tier-system](FR-00056-stop-tier-system.md)
  - [FR-00036-failsafe-integration](FR-00036-failsafe-integration.md)
  - [FR-00033-emergency-stop-triggers](FR-00033-emergency-stop-triggers.md)
  - [FR-00055-stop-state-tracking](FR-00055-stop-state-tracking.md)
  - [NFR-00027-emergency-stop-ram-overhead](NFR-00027-emergency-stop-ram-overhead.md)
  - [NFR-00026-emergency-stop-initiation-latency](NFR-00026-emergency-stop-initiation-latency.md)
  - [NFR-00025-emergency-stop-event-logging](NFR-00025-emergency-stop-event-logging.md)
  - [NFR-00021-controlled-stop-completion-time](NFR-00021-controlled-stop-completion-time.md)
  - [NFR-00045-no-rollover-loss-of-control](NFR-00045-no-rollover-loss-of-control.md)

- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall implement controlled emergency stop with gradual deceleration using velocity-based proportional control, preventing vehicle rollover and loss of control while bringing the vehicle to a safe stop within 2-3 seconds from typical operating velocities.

## Rationale

Immediate motor cutoff during emergency stop can cause vehicle rollover on ground vehicles due to abrupt deceleration. A controlled stop using velocity-based deceleration provides:

- Safe deceleration preventing rollover
- Predictable stop behavior for operator planning
- Maintained heading stability during stop
- Integration with ArduPilot's proven `stop_vehicle()` architecture

This is the default emergency stop mechanism and should be used for most failsafe conditions.

## User Story (if applicable)

As a safety officer, I want the vehicle to stop using controlled deceleration when emergency stop is triggered, so that the vehicle does not roll over or lose control during the stop sequence.

## Acceptance Criteria

- [ ] Calculate stop throttle proportional to current velocity (P controller)
- [ ] Deceleration time: 2-3 seconds for controlled stop at typical velocities (1 m/s)
- [ ] Maintain heading stability during deceleration (steering rate = 0 or heading hold)
- [ ] Monitor velocity to detect when vehicle fully stopped (< 0.1 m/s threshold)
- [ ] Return stopped confirmation flag when stop complete
- [ ] P gain configurable via parameter (default 10.0, matching ArduPilot)
- [ ] Stop throttle calculated as: `throttle = -velocity * P_gain`
- [ ] Clamp stop throttle to safe limits (-100 to 0)

## Technical Details (if applicable)

### Functional Requirement Details

**Stop Controller Algorithm**:

```rust
fn calculate_stop_throttle(current_velocity: f32, p_gain: f32) -> f32 {
    // Proportional control: brake force proportional to current speed
    let stop_throttle = -current_velocity * p_gain;

    // Clamp to safe limits (negative = braking)
    stop_throttle.clamp(-100.0, 0.0)
}
```

**Stopped Detection**:

- Velocity threshold: < 0.1 m/s (matching ArduPilot `SPEED_STOP_THRESHOLD`)
- Requires velocity below threshold for 2 consecutive cycles to prevent false stops
- Returns stopped flag when threshold met

**Heading Stability**:

- Steering maintained at neutral (0.0) or heading hold (rate = 0)
- Phase 1: Use simple steering lockout (0.0 steering during stop)
- Phase 2: Integrate with attitude controller for active heading hold

**Parameter Configuration**:

- `ESTOP_P_GAIN`: Proportional gain for controlled stop (default 10.0)
- Higher gain = faster deceleration (more aggressive)
- Lower gain = gentler deceleration (smoother but slower)

## Platform Considerations

### Pico W (RP2040)

N/A - Platform agnostic (stop control logic independent of hardware)

### Pico 2 W (RP2350)

N/A - Platform agnostic

### Cross-Platform

Stop controller implementation is platform-independent. Uses velocity estimation from platform abstraction layer.

## Risks & Mitigation

| Risk                                               | Impact | Likelihood | Mitigation                                                                          | Validation                                  |
| -------------------------------------------------- | ------ | ---------- | ----------------------------------------------------------------------------------- | ------------------------------------------- |
| Velocity estimation error causes poor stop         | Medium | Medium     | Use multiple velocity sources (wheel encoders + GPS), validate                      | Test stop performance at various speeds     |
| P gain too high causes jerky stop                  | Medium | Low        | Start with proven ArduPilot default (10.0), allow tuning                            | Test on actual vehicle, adjust if needed    |
| P gain too low causes slow stop                    | Low    | Medium     | Document tuning procedure, allow operator to increase gain                          | Measure stop time vs. gain, publish results |
| Stop time exceeds 3 seconds at high speed          | Medium | Low        | Increase P gain for high-speed operation, test before missions                      | Test at maximum operating speed             |
| Vehicle slides during stop (insufficient traction) | High   | Medium     | Tune P gain per vehicle mass and terrain, warn operator if stop time exceeds target | Field testing on various surfaces           |

## Implementation Notes

Preferred approaches:

- Use velocity-based P controller (simple, proven, ArduPilot-compatible)
- Start with ArduPilot default gain (10.0), tune via field testing
- Integrate with attitude controller for heading stability (Phase 2)
- Provide parameter for P gain adjustment per vehicle

Known pitfalls:

- Do not use immediate throttle cutoff (causes rollover)
- Do not use time-based deceleration (ignores current velocity)
- Ensure velocity estimate is reliable (filter noisy readings)
- Test stop performance on different surfaces (grass, pavement, gravel)

Related code areas:

- `src/vehicle/emergency_stop/` - Emergency stop system
- `src/vehicle/velocity/` - Velocity estimation
- `src/vehicle/actuators.rs` - Actuator layer

## External References

- ArduPilot stop_vehicle(): <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.cpp>
- ArduPilot Rover Mode Documentation: <https://ardupilot.org/rover/docs/rover-first-drive.html>
- Analysis: [AN-00010-emergency-stop](../analysis/AN-00010-emergency-stop.md)
