# FR-w0spp Position Drift Detection from Loiter Point

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-aw3h3-rover-loiter-mode](FR-aw3h3-rover-loiter-mode.md)
  - [FR-30t03-loiter-type-parameter](FR-30t03-loiter-type-parameter.md)
  - [FR-cs42u-gps-navigation-state-access](FR-cs42u-gps-navigation-state-access.md)
- Dependent Requirements:
  - [FR-9s9th-position-correction-navigation](FR-9s9th-position-correction-navigation.md)
  - [NFR-biag9-loiter-drift-detection-performance](NFR-biag9-loiter-drift-detection-performance.md)
- Related Tasks:
  - [T-n24yy-rover-loiter-mode](../tasks/T-n24yy-rover-loiter-mode/README.md)

## Requirement Statement

The system shall detect position drift from the loiter point by calculating distance from current GPS position to the recorded loiter point, and trigger correction when drift exceeds LOIT_RADIUS threshold (Type 1 mode only).

## Rationale

Position drift detection is the core mechanism enabling active station keeping in Loiter Type 1 mode. Accurate drift detection with appropriate hysteresis prevents oscillation at the radius boundary while ensuring timely correction when drift becomes significant.

## User Story

As a rover operator, I want the system to detect when my vehicle has drifted too far from the loiter point, so that it can automatically correct back to position.

## Acceptance Criteria

- [ ] Distance calculated using Haversine formula for GPS coordinates
- [ ] Drift detection runs at 1-5 Hz (sufficient for typical drift rates)
- [ ] Correction triggered when distance > LOIT_RADIUS
- [ ] Correction stopped when distance < LOIT_RADIUS \* HYSTERESIS_FACTOR (0.8)
- [ ] Hysteresis prevents oscillation at radius boundary
- [ ] Drift detection only active in Type 1 mode
- [ ] is_correcting state tracked for telemetry reporting

## Technical Details

### Functional Requirement Details

**Distance Calculation:**

```rust
fn distance_to_loiter(current: &GpsPosition, loiter: &GpsPosition) -> f32 {
    const EARTH_RADIUS_M: f32 = 6_371_000.0;
    let lat1 = current.latitude.to_radians();
    let lat2 = loiter.latitude.to_radians();
    let dlat = (loiter.latitude - current.latitude).to_radians();
    let dlon = (loiter.longitude - current.longitude).to_radians();

    let a = (dlat / 2.0).sin().powi(2)
        + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();
    EARTH_RADIUS_M * c
}
```

**Hysteresis State Machine:**

```rust
const HYSTERESIS_FACTOR: f32 = 0.8;

fn check_correction_needed(
    state: &mut LoiterState,
    current: &GpsPosition
) -> bool {
    if state.loiter_type == 0 {
        return false;  // Type 0 never corrects
    }

    let distance = distance_to_loiter(current, &state.loiter_point);

    if state.is_correcting {
        // Stop correcting when well within radius
        if distance < state.radius * HYSTERESIS_FACTOR {
            state.is_correcting = false;
        }
    } else {
        // Start correcting when outside radius
        if distance > state.radius {
            state.is_correcting = true;
        }
    }

    state.is_correcting
}
```

**Update Rates:**

| Component         | Rate   | Notes                      |
| ----------------- | ------ | -------------------------- |
| GPS update        | 1-10Hz | Position feedback          |
| Drift check       | 1-5Hz  | Match GPS update rate      |
| Correction output | 50Hz   | Steering/throttle commands |

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                        | Impact | Likelihood | Mitigation                  | Validation                     |
| --------------------------- | ------ | ---------- | --------------------------- | ------------------------------ |
| GPS noise causes jitter     | Low    | Medium     | Hysteresis prevents chatter | Test with noisy GPS simulation |
| High drift rate not caught  | Low    | Low        | Check rate >= GPS update    | Test rapid drift scenarios     |
| Haversine overhead too high | Low    | Low        | Pre-compute constants       | Benchmark on RP2350            |

## Implementation Notes

- Reuse `distance_m()` from `src/subsystems/navigation/geo.rs`
- Hysteresis factor (0.8) can be made configurable if needed
- Consider logging drift distance for post-flight analysis
- State tracking enables telemetry: "LOITER: correcting" vs "LOITER: holding"

## External References

- [Haversine Formula](https://en.wikipedia.org/wiki/Haversine_formula) - Distance calculation method
- [ArduPilot Loiter Mode](https://ardupilot.org/rover/docs/loiter-mode.html) - Drift behavior description
