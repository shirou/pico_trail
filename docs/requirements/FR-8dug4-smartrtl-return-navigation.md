# FR-8dug4 SmartRTL Return Navigation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-nywcm-smartrtl-path-recording](FR-nywcm-smartrtl-path-recording.md)
  - [FR-me7q8-smartrtl-path-simplification](FR-me7q8-smartrtl-path-simplification.md)
  - [FR-bwqq7-rtl-entry-validation](FR-bwqq7-rtl-entry-validation.md)
- Dependent Requirements:
  - [FR-zlza4-rtl-arrival-stop](FR-zlza4-rtl-arrival-stop.md)
  - [FR-hibx1-smartrtl-rtl-fallback](FR-hibx1-smartrtl-rtl-fallback.md)
- Related Analyses:
  - [AN-75408-rtl-mode](../analysis/AN-75408-rtl-mode.md)
- Related Tasks:
  - [T-bo6xc-rtl-smartrtl-implementation](../tasks/T-bo6xc-rtl-smartrtl-implementation/README.md)

## Requirement Statement

The system shall navigate the vehicle along the recorded path in reverse order when SmartRTL mode is activated, using waypoint-to-waypoint navigation until reaching the home position.

## Rationale

SmartRTL provides safer return navigation than direct RTL by retracing the vehicle's previously traveled path. This is critical for:

- Avoiding obstacles the vehicle navigated around during outbound travel
- Following proven safe terrain
- Providing predictable return behavior for operators
- Working in environments where direct-line return is blocked

## User Story

As an operator, I want SmartRTL to retrace the vehicle's path home, so that it avoids obstacles and follows a route known to be safe.

## Acceptance Criteria

- [ ] SmartRTL activates when RTL mode requested (SmartRTL is default)
- [ ] Navigation follows recorded path points in reverse order
- [ ] Each segment uses SimpleNavigationController for steering/throttle
- [ ] Arrival at each waypoint detected when within WP_RADIUS
- [ ] Path consumption: reached waypoints removed from buffer
- [ ] Final arrival at home triggers stop behavior (same as RTL)
- [ ] RTL_SPEED parameter controls return speed

## Technical Details

### Functional Requirement Details

**Mode Activation:**

```rust
// When RTL requested:
if path_recorder.has_valid_path() {
    enter_smartrtl_mode();
} else {
    enter_direct_rtl_mode();  // fallback
}
```

**Navigation Loop:**

1. Get current target from path buffer (last recorded point)
2. Navigate to target using SimpleNavigationController
3. When at_target reached, pop point from buffer
4. Repeat until buffer empty
5. Final segment: navigate to home position
6. Stop when at home

**Path Consumption:**

- Points removed as vehicle reaches them
- Prevents re-navigation of completed segments
- Buffer empties as return progresses

**Speed Control:**

- RTL_SPEED parameter (m/s)
- 0 = use WP_SPEED default
- Applied as throttle limit during navigation

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                    | Impact | Likelihood | Mitigation                    | Validation                 |
| ----------------------- | ------ | ---------- | ----------------------------- | -------------------------- |
| Path points too sparse  | Medium | Low        | WP_RADIUS handles gaps        | Test with simplified paths |
| Vehicle drifts off path | Low    | Low        | Nav controller corrects       | Test cross-track error     |
| GPS loss during return  | High   | Medium     | Transition to Hold (FR-crqdd) | Test GPS disconnection     |

## Implementation Notes

- SmartRTL mode: `src/rover/mode/smartrtl.rs`
- Uses existing SimpleNavigationController
- Path buffer accessed via shared state
- Consider: Progress telemetry (points remaining)

## External References

- [ArduPilot SmartRTL Mode](https://ardupilot.org/rover/docs/smartrtl-mode.html) - Reference implementation
