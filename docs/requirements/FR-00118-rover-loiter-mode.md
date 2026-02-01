# FR-00118 Rover Loiter Mode Position Holding

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00062-control-modes](FR-00062-control-modes.md)
  - [FR-00081-gps-navigation-state-access](FR-00081-gps-navigation-state-access.md)
  - [FR-00084-navigation-controller](FR-00084-navigation-controller.md)
- Dependent Requirements:
  - [FR-00114-loiter-point-calculation](FR-00114-loiter-point-calculation.md)
  - [FR-00115-loiter-type-parameter](FR-00115-loiter-type-parameter.md)
  - [FR-00117-position-drift-detection](FR-00117-position-drift-detection.md)
  - [FR-00116-position-correction-navigation](FR-00116-position-correction-navigation.md)
  - [NFR-00081-loiter-drift-detection-performance](NFR-00081-loiter-drift-detection-performance.md)
  - [NFR-00082-position-hold-accuracy](NFR-00082-position-hold-accuracy.md)
- Related Tasks:
  - [T-00029-rover-loiter-mode](../tasks/T-00029-rover-loiter-mode/README.md)

## Requirement Statement

The system shall implement Loiter mode for ground rovers that enables position holding at a fixed point, supporting both Type 0 (stop motors) and Type 1 (active position correction) behaviors as configured by LOIT_TYPE parameter.

## Rationale

Loiter mode is a standard ArduPilot Rover mode useful for:

- Temporary stops during operation without disarming
- Position holding while waiting for GCS commands
- Station keeping on slopes or in windy conditions (Type 1)
- Foundation for other position-based modes (RTL, Guided waypoints)

This is distinct from Hold mode which simply stops motors without GPS-based position tracking.

## User Story

As a rover operator, I want the vehicle to hold its position when I switch to Loiter mode, so that I can pause operation without the vehicle drifting away.

## Acceptance Criteria

- [ ] Loiter mode is selectable from GCS via MAVLink DO_SET_MODE command
- [ ] Mode entry records current GPS position as loiter point
- [ ] Mode requires valid GPS fix to enter (rejects entry without fix)
- [ ] LOIT_TYPE=0: Motors stop, no active position correction
- [ ] LOIT_TYPE=1: Active correction when drift exceeds LOIT_RADIUS
- [ ] Mode exits cleanly, setting actuators to neutral
- [ ] Mode reports correct status in MAVLink HEARTBEAT mode field

## Technical Details

### Functional Requirement Details

**Mode Entry:**

1. Validate GPS fix availability (reject if no fix)
2. Calculate loiter point:
   - If stopped or low speed: use current position
   - If moving: project stopping point based on velocity and deceleration
3. Initialize loiter state (point, type, radius from parameters)
4. Log mode entry with loiter point coordinates

**Mode Update (50 Hz):**

1. Read current GPS position
2. If LOIT_TYPE=0: output zero steering/throttle
3. If LOIT_TYPE=1:
   - Calculate distance from loiter point
   - If distance > LOIT_RADIUS: navigate back to loiter point
   - If distance <= LOIT_RADIUS: stop (allow drift within radius)

**Mode Exit:**

1. Set actuators to neutral (steering=0, throttle=0)
2. Log mode exit

**State Machine:**

```
Mode Entry → Record Position → [Type 0: Stop] or [Type 1: Monitor Drift]
                                                        ↓
                                              [Within Radius: Drift]
                                                        ↓
                                              [Outside Radius: Correct]
```

## Platform Considerations

N/A - Platform agnostic (uses navigation controller abstraction)

## Risks & Mitigation

| Risk                            | Impact | Likelihood | Mitigation                      | Validation                    |
| ------------------------------- | ------ | ---------- | ------------------------------- | ----------------------------- |
| GPS fix lost during Loiter      | Medium | Medium     | Degrade to Type 0 (stop motors) | Test GPS disconnect scenarios |
| Oscillation at LOIT_RADIUS edge | Low    | Medium     | Hysteresis in drift detection   | Test with simulated drift     |
| Mode confusion with Hold mode   | Low    | Low        | Clear GCS labels and telemetry  | User acceptance testing       |

## Implementation Notes

- Implement in `src/rover/mode/loiter.rs` with `#[cfg(feature = "rover")]` gate
- Reuse navigation calculations from `src/subsystems/navigation/geo.rs`
- Follow Mode trait pattern from ADR-00013
- Parameters LOIT_TYPE and LOIT_RADIUS are standard ArduPilot parameters

## External References

- [ArduPilot Loiter Mode (Rover)](https://ardupilot.org/rover/docs/loiter-mode.html) - Official documentation
- [ArduPilot LOIT Parameters](https://ardupilot.org/rover/docs/parameters.html#loit-parameters) - Parameter definitions
