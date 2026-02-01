# FR-00116 Position Correction Navigation to Loiter Point

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00118-rover-loiter-mode](FR-00118-rover-loiter-mode.md)
  - [FR-00117-position-drift-detection](FR-00117-position-drift-detection.md)
  - [FR-00084-navigation-controller](FR-00084-navigation-controller.md)
- Dependent Requirements:
  - [NFR-00082-position-hold-accuracy](NFR-00082-position-hold-accuracy.md)
- Related Tasks:
  - [T-00029-rover-loiter-mode](../tasks/T-00029-rover-loiter-mode/README.md)

## Requirement Statement

The system shall navigate the rover back to the loiter point when position drift exceeds the LOIT_RADIUS threshold (Type 1 mode), using speed proportional to drift distance and choosing the shortest rotation path (forward or reverse).

## Rationale

When drift exceeds the acceptable radius, the rover must return to the loiter point efficiently. Proportional speed control prevents aggressive correction for small drifts while ensuring timely return for larger drifts. Allowing reverse motion reduces unnecessary turning maneuvers.

## User Story

As a rover operator, I want the vehicle to smoothly return to the loiter point when it drifts too far, so that position holding is maintained without aggressive or jerky movements.

## Acceptance Criteria

- [ ] Navigation activates only when drift exceeds LOIT_RADIUS
- [ ] Target speed proportional to distance: 0.5 m/s per meter of drift
- [ ] Speed capped at WP_SPEED parameter (never exceed waypoint speed)
- [ ] Shortest rotation path selected (forward or reverse to target)
- [ ] Navigation stops when within LOIT_RADIUS \* 0.8 (hysteresis)
- [ ] Steering calculated based on bearing to loiter point
- [ ] Reverse motion supported when closer than forward rotation

## Technical Details

### Functional Requirement Details

**Correction Algorithm:**

```rust
fn calculate_correction(
    current_pos: &GpsPosition,
    loiter_point: &GpsPosition,
    current_heading: f32,
    loit_radius: f32,
    wp_speed: f32,
) -> (f32, f32) {  // (steering, throttle)
    let distance = distance_m(current_pos, loiter_point);

    if distance <= loit_radius {
        // Within acceptable radius - stop
        return (0.0, 0.0);
    }

    // Calculate target speed proportional to distance beyond radius
    let distance_to_edge = distance - loit_radius;
    let target_speed = (0.5 * distance_to_edge).min(wp_speed);

    // Calculate bearing to loiter point
    let bearing = bearing_deg(current_pos, loiter_point);

    // Choose shortest rotation path (forward or reverse)
    let heading_error_fwd = wrap_180(bearing - current_heading);
    let heading_error_rev = wrap_180(bearing + 180.0 - current_heading);

    let (steering_input, direction) = if heading_error_fwd.abs() < heading_error_rev.abs() {
        // Forward is shorter rotation
        (calculate_steering(heading_error_fwd), 1.0)
    } else {
        // Reverse is shorter rotation
        (calculate_steering(heading_error_rev), -1.0)
    };

    let throttle = target_speed * direction / wp_speed;  // Normalize to -1.0..1.0

    (steering_input, throttle)
}
```

**Speed Profile:**

| Distance Beyond Radius | Target Speed | Notes                  |
| ---------------------- | ------------ | ---------------------- |
| 0m (at edge)           | 0 m/s        | Just entered zone      |
| 1m                     | 0.5 m/s      | Gentle correction      |
| 2m                     | 1.0 m/s      | Moderate correction    |
| 4m                     | 2.0 m/s      | Faster correction      |
| >WP_SPEED/0.5          | WP_SPEED     | Capped at max wp speed |

**Steering Calculation:**

- Use navigation controller's steering output
- P-controller on heading error is sufficient for this application
- More sophisticated control (PID, L1) available if needed

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                            | Impact | Likelihood | Mitigation                      | Validation                    |
| ------------------------------- | ------ | ---------- | ------------------------------- | ----------------------------- |
| Oscillation around loiter point | Medium | Medium     | Hysteresis + proportional speed | Test station keeping duration |
| Reverse motion unexpected       | Low    | Low        | Document behavior, add param    | User acceptance testing       |
| Speed too aggressive            | Low    | Low        | 0.5 m/s/m is conservative       | Tune based on hardware tests  |

## Implementation Notes

- Integrate with existing navigation controller from `src/subsystems/navigation/`
- `WP_SPEED` is standard ArduPilot parameter for maximum waypoint speed
- Consider adding `LOIT_SPEED` parameter for independent correction speed limit
- Reverse driving can be disabled via future parameter if users prefer forward-only

## External References

- [ArduPilot Loiter Mode](https://ardupilot.org/rover/docs/loiter-mode.html) - Correction behavior description
- [ArduPilot WP_SPEED](https://ardupilot.org/rover/docs/parameters.html#wp-speed) - Speed limit parameter
