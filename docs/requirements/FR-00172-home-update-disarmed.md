# FR-00172 Home Position Update While Disarmed

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00171-home-auto-set-gps-fix](FR-00171-home-auto-set-gps-fix.md)
  - [FR-00176-home-lock-mechanism](FR-00176-home-lock-mechanism.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00047-home-position-management](../analysis/AN-00047-home-position-management.md)
- Related Tasks:
  - [T-00181-home-position-management](../tasks/T-00181-home-position-management/README.md)

## Requirement Statement

The system shall update the home position at 1 Hz while the vehicle is disarmed, if the vehicle has moved more than 0.5 meters from the current home position and home is not locked (GCS-set).

## Rationale

ArduPilot's `update_home()` (`Rover/commands.cpp:44-65`) refines home at 1 Hz while disarmed, using `DISTANCE_HOME_MINCHANGE = 0.5f` meters threshold (`Rover/defines.h:81`). This serves two purposes: (1) GPS accuracy improves after initial fix as more satellites are acquired, so refining home provides a more accurate return point; (2) if the operator physically relocates the vehicle between flights (without power cycling), home tracks the new position.

## User Story

As an operator, I want the vehicle to refine its home position while powered on and disarmed, so that home accurately reflects the current launch point even as GPS accuracy improves or I relocate the vehicle.

## Acceptance Criteria

- [ ] Check at 1 Hz in vehicle control loop while vehicle is disarmed
- [ ] Skip update if `home_locked == true` (GCS-set home is preserved)
- [ ] Skip update if home is not set (`home_position.is_none()`)
- [ ] Calculate distance from current GPS position to current home position
- [ ] Skip update if distance is less than 0.5 meters
- [ ] Update home via `set_home_to_current()` if distance >= 0.5 meters
- [ ] Trigger HOME_POSITION broadcast after update (see FR-00173)
- [ ] Do not update home while armed (only while disarmed)

## Technical Details

### Functional Requirement Details

**Disarmed update logic (1 Hz in control loop):**

```rust
// Called at 1 Hz while disarmed:
fn update_home(state: &mut SystemState) -> bool {
    if state.home_locked {
        return false;  // GCS-set home is preserved
    }
    if let (Some(home), Some(gps)) = (&state.home_position, &state.gps_position) {
        let distance = calculate_distance(
            home.latitude, home.longitude,
            gps.lat, gps.lon,
        );
        if distance < DISTANCE_HOME_MINCHANGE {
            return false;  // Hasn't moved enough
        }
        state.set_home_to_current();
        // Broadcast HOME_POSITION
        true
    } else {
        false
    }
}

const DISTANCE_HOME_MINCHANGE: f32 = 0.5;  // meters
```

**ArduPilot reference:**

- `Rover::update_home()` at `repo/ardupilot/Rover/commands.cpp:44-65`
- Called from `one_second_loop()` only while disarmed
- Uses `DISTANCE_HOME_MINCHANGE = 0.5f` (`repo/ardupilot/Rover/defines.h:81`)

**Distance calculation:**

- Use existing navigation distance functions or Haversine formula
- At 0.5m threshold, simplified flat-earth approximation is acceptable for the distance check

## Platform Considerations

N/A - Platform agnostic (core crate logic)

## Risks & Mitigation

| Risk                                     | Impact | Likelihood | Mitigation                                                       | Validation                     |
| ---------------------------------------- | ------ | ---------- | ---------------------------------------------------------------- | ------------------------------ |
| GPS jitter causes unnecessary updates    | Low    | Low        | 0.5m threshold filters GPS noise (typical GPS jitter \~1-3m CEP) | Test with static GPS           |
| Distance calculation performance         | Low    | Low        | Single calculation at 1 Hz is negligible                         | Benchmark                      |
| Home drifts significantly while disarmed | Low    | Low        | Only updates if moved >0.5m; normal GPS jitter is filtered       | Verify with stationary vehicle |

## Implementation Notes

- 1 Hz rate can be achieved by checking a counter or elapsed time in the control loop
- Use existing `critical_section::with()` pattern for state access
- HOME_POSITION broadcast happens after the critical section
- Home lock check (`home_locked`) must be done before update (see FR-00176)

## External References

- ArduPilot `update_home()`: `repo/ardupilot/Rover/commands.cpp:44-65`
- ArduPilot `DISTANCE_HOME_MINCHANGE`: `repo/ardupilot/Rover/defines.h:81`
