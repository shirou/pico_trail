# FR-zlza4 RTL Arrival Stop Behavior

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-lia7r-rtl-navigate-home](FR-lia7r-rtl-navigate-home.md)
  - [FR-8dug4-smartrtl-return-navigation](FR-8dug4-smartrtl-return-navigation.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-75408-rtl-mode](../analysis/AN-75408-rtl-mode.md)
- Related Tasks:
  - [T-bo6xc-rtl-smartrtl-implementation](../tasks/T-bo6xc-rtl-smartrtl-implementation/README.md)

## Requirement Statement

The system shall stop the vehicle when RTL navigation reaches the home position (within WP_RADIUS threshold), setting throttle to zero and maintaining stopped state until mode is changed.

## Rationale

When the vehicle reaches home, it must have defined behavior. Stopping in place is the safest default action:

- Vehicle is safely at known location
- Operator can assess situation before further action
- No risk of continued movement
- Aligned with ArduPilot Hold behavior

## User Story

As an operator, I want the vehicle to stop when it arrives at home, so that I know the RTL operation completed successfully and the vehicle is in a safe state.

## Acceptance Criteria

- [ ] Arrival detected when NavigationOutput.at_target is true
- [ ] at_target is true when distance < WP_RADIUS (default 2m)
- [ ] Throttle set to 0 upon arrival
- [ ] Steering set to neutral (0) upon arrival
- [ ] Log message "RTL: Arrived at home" on arrival
- [ ] Vehicle remains stopped until mode is changed
- [ ] No disarm-on-arrival (vehicle stays armed)

## Technical Details

### Functional Requirement Details

**Arrival Detection:**

```rust
let output = self.nav_controller.update(&gps, &self.target, dt);
if output.at_target {
    self.arrived = true;
    crate::log_info!("RTL: Arrived at home");
}
```

**Post-Arrival Behavior:**

- Set internal `arrived` flag to true
- On subsequent update() calls, apply zero throttle/steering
- Do not transition to different mode automatically
- Vehicle remains in RTL mode but stationary

**WP_RADIUS Configuration:**

- Default: 2 meters
- Configurable via WP_RADIUS parameter
- Checked by SimpleNavigationController

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                             | Impact | Likelihood | Mitigation                       | Validation                  |
| -------------------------------- | ------ | ---------- | -------------------------------- | --------------------------- |
| WP_RADIUS too small (oscillate)  | Medium | Low        | Default 2m is conservative       | Test with various distances |
| WP_RADIUS too large (inaccurate) | Low    | Low        | 2m is reasonable for rovers      | Verify acceptable accuracy  |
| GPS drift causes re-navigation   | Low    | Low        | arrived flag prevents re-trigger | Test with GPS drift         |

## Implementation Notes

- Arrival state tracked in `RtlMode.arrived: bool`
- Once arrived=true, update() returns early with zero outputs
- No automatic mode transition to Hold (stay in RTL)
- Future enhancement: RTL_OPTIONS bitmask for disarm-on-arrival

## External References

- [ArduPilot WP_RADIUS Parameter](https://ardupilot.org/rover/docs/parameters.html#wp-radius) - Waypoint radius configuration
