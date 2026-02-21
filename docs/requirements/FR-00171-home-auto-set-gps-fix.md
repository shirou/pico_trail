# FR-00171 Home Auto-Set on GPS Fix

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00077-gps-uart-driver](FR-00077-gps-uart-driver.md)
  - [FR-00081-gps-navigation-state-access](FR-00081-gps-navigation-state-access.md)
- Dependent Requirements:
  - [FR-00172-home-update-disarmed](FR-00172-home-update-disarmed.md)
  - [FR-00173-home-position-broadcast](FR-00173-home-position-broadcast.md)
  - [FR-00174-request-message-home-position](FR-00174-request-message-home-position.md)
  - [FR-00175-home-prearm-check](FR-00175-home-prearm-check.md)
  - [FR-00176-home-lock-mechanism](FR-00176-home-lock-mechanism.md)
  - [FR-00177-global-position-int-relative-alt](FR-00177-global-position-int-relative-alt.md)
  - [FR-00178-get-home-position-command](FR-00178-get-home-position-command.md)
  - [NFR-00179-home-auto-set-latency](NFR-00179-home-auto-set-latency.md)
- Related Analyses:
  - [AN-00047-home-position-management](../analysis/AN-00047-home-position-management.md)
- Related Tasks:
  - [T-00181-home-position-management](../tasks/T-00181-home-position-management/README.md)

## Requirement Statement

The system shall automatically set the home position to the current GPS location when a valid 3D GPS fix is first acquired after power-on. The auto-set home shall be unlocked (eligible for refinement by disarmed updates).

## Rationale

ArduPilot auto-sets home on first valid EKF/GPS position via `ahrs_update()` at 400 Hz (`Rover/Rover.cpp:297-314`). Home must be available before arming because it is a pre-arm prerequisite. Without auto-set, operators must manually send `MAV_CMD_DO_SET_HOME` before flight, and Mission Planner reports "failed to set home" when entering Guided mode because no home position exists.

## User Story

As an operator, I want the vehicle to automatically establish a home position as soon as GPS is available, so that RTL, failsafe actions, and GCS features work without manual intervention.

## Acceptance Criteria

- [ ] Check in vehicle control loop: if home is not set and GPS fix is 3D or better, set home to current GPS position
- [ ] Use existing `SystemState::set_home_to_current()` method
- [ ] Set home as unlocked (`home_locked = false`) so disarmed updates can refine it
- [ ] Trigger HOME_POSITION broadcast after setting home (see FR-00173)
- [ ] Log "Home set to {lat}, {lon}" via `log_info!`
- [ ] Home auto-set occurs only once (subsequent GPS fixes do not re-trigger auto-set if home is already set)
- [ ] Home position is never cleared once set (persists through GPS loss)
- [ ] Home auto-set completes within 1ms (see NFR-00179)

## Technical Details

### Functional Requirement Details

**Auto-set logic in vehicle control loop (MavlinkLoopRunner or equivalent):**

```rust
// On each control loop iteration:
if !state.has_home() {
    if let Some(gps) = &state.gps_position {
        if gps.fix_type >= GpsFixType::Fix3D {
            state.set_home_to_current();
            state.home_locked = false;
            // Build and send HOME_POSITION message
            log_info!("Home set to {}, {}", gps.lat, gps.lon);
        }
    }
}
```

**ArduPilot reference:**

- `Rover::ahrs_update()` at `repo/ardupilot/Rover/Rover.cpp:297-314`
- Called at 400 Hz; sets home on first valid EKF position
- pico_trail checks at control loop rate (50 Hz), which is sufficient

**Key behavior:**

- Home is set as soon as GPS provides a valid 3D position, well before arming
- Once set, auto-set does not re-trigger (only disarmed update refines home)
- Home persists after GPS loss -- once set, it is never cleared

## Platform Considerations

N/A - Platform agnostic (core crate logic)

## Risks & Mitigation

| Risk                                   | Impact | Likelihood | Mitigation                                                            | Validation                     |
| -------------------------------------- | ------ | ---------- | --------------------------------------------------------------------- | ------------------------------ |
| GPS provides inaccurate initial fix    | Medium | Medium     | Disarmed home update (FR-00172) refines position as accuracy improves | Test with GPS warm-up sequence |
| Home set before GPS accuracy converges | Low    | Medium     | Acceptable -- disarmed update corrects within seconds                 | Verify update refines home     |
| Control loop overhead from GPS check   | Low    | Low        | Single conditional check per iteration (\~nanoseconds)                | Benchmark control loop         |

## Implementation Notes

- Logic belongs in vehicle control loop (MavlinkLoopRunner), not in SystemState (data struct, not controller)
- `set_home_to_current()` is called inside `critical_section::with()` block; HOME_POSITION broadcast must happen after the critical section
- Existing `HomePosition::from_gps()` handles lat/lon/alt conversion

## External References

- ArduPilot `ahrs_update()` home auto-set: `repo/ardupilot/Rover/Rover.cpp:297-314`
- ArduPilot Setting Home documentation: <https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html>
- MAVLink HOME_POSITION message: <https://mavlink.io/en/messages/common.html#HOME_POSITION>
