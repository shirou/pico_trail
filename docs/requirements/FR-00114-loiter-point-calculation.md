# FR-00114 Loiter Point Calculation on Mode Entry

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00118-rover-loiter-mode](FR-00118-rover-loiter-mode.md)
  - [FR-00081-gps-navigation-state-access](FR-00081-gps-navigation-state-access.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-00029-rover-loiter-mode](../tasks/T-00029-rover-loiter-mode/README.md)

## Requirement Statement

The system shall calculate and record a loiter point on Loiter mode entry, using current position when stopped or projecting a stopping point when moving based on current velocity and maximum deceleration.

## Rationale

When entering Loiter mode at speed, the vehicle cannot instantly stop. Projecting a reasonable stopping point prevents the vehicle from immediately triggering correction behavior and provides a more natural user experience. This matches ArduPilot Rover behavior.

## User Story

As a rover operator, I want the loiter point to be calculated intelligently when switching to Loiter mode, so that the vehicle settles naturally instead of immediately fighting to return to an unreachable position.

## Acceptance Criteria

- [ ] At low speed (< 0.5 m/s): loiter point equals current GPS position
- [ ] At speed: loiter point is projected forward based on stopping distance
- [ ] Stopping distance calculated as v^2 / (2 \* max_decel)
- [ ] Projection direction matches current heading/velocity vector
- [ ] Loiter point stored for duration of mode
- [ ] Loiter point logged on mode entry for debugging

## Technical Details

### Functional Requirement Details

**Stop Point Calculation:**

```rust
fn calculate_stop_point(
    current_pos: &GpsPosition,
    velocity: Vector2D,
    max_decel: f32,  // m/s^2, from vehicle parameters
) -> GpsPosition {
    let speed = velocity.magnitude();

    // Low speed threshold - use current position
    if speed < 0.5 {
        return current_pos.clone();
    }

    // Calculate stopping distance: v^2 / (2 * a)
    let stop_distance = (speed * speed) / (2.0 * max_decel);

    // Project stop point in direction of travel
    let heading = velocity.heading();
    offset_position(current_pos, stop_distance, heading)
}
```

**Offset Position Calculation:**

Uses standard geodetic offset based on distance and bearing:

- North offset: distance \* cos(bearing)
- East offset: distance \* sin(bearing)
- Convert to lat/lon degrees based on local Earth radius

**Parameters Used:**

- `ATC_DECEL_MAX`: Maximum deceleration (ArduPilot standard parameter)
- If not available, use conservative default (1.0 m/s^2)

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                              | Impact | Likelihood | Mitigation                | Validation                    |
| --------------------------------- | ------ | ---------- | ------------------------- | ----------------------------- |
| Velocity estimate inaccurate      | Low    | Medium     | Use GPS ground speed      | Test at various speeds        |
| Decel parameter misconfigured     | Low    | Low        | Clamp to reasonable range | Parameter validation          |
| Projection overshoots destination | Low    | Low        | Cap projection to 50m     | Test high-speed mode switches |

## Implementation Notes

- Velocity can be obtained from GPS ground speed and course over ground
- If velocity vector unavailable, fall back to current position
- Maximum projection distance should be capped (e.g., 50m) as safety limit
- Use existing `offset_position()` from `src/subsystems/navigation/geo.rs`

## External References

- [ArduPilot Loiter Mode](https://ardupilot.org/rover/docs/loiter-mode.html) - Describes stop point projection behavior
