# FR-2vbe8 Navigation Controller

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-27568-position-target-navigation](../analysis/AN-27568-position-target-navigation.md)
  - [AN-7ix56-navigation-approach](../analysis/AN-7ix56-navigation-approach.md)
- Prerequisite Requirements:
  - [FR-cs42u-gps-navigation-state-access](FR-cs42u-gps-navigation-state-access.md)
  - [FR-41nab-differential-drive-kinematics](FR-41nab-differential-drive-kinematics.md)
- Dependent Requirements:
  - [FR-9s9th-position-correction-navigation](FR-9s9th-position-correction-navigation.md)
  - [FR-aw3h3-rover-loiter-mode](FR-aw3h3-rover-loiter-mode.md)
  - [FR-erpze-guided-mode-navigation](FR-erpze-guided-mode-navigation.md)
  - [FR-jm7mj-auto-mode-mission-execution](FR-jm7mj-auto-mode-mission-execution.md)
  - [FR-khjpl-circle-mode-implementation](FR-khjpl-circle-mode-implementation.md)
  - [FR-m2c9z-mission-waypoint-navigation](FR-m2c9z-mission-waypoint-navigation.md)
  - [NFR-iuk5h-navigation-update-rate](NFR-iuk5h-navigation-update-rate.md)
  - [NFR-wtdig-navigation-controller-performance](NFR-wtdig-navigation-controller-performance.md)
- Related ADRs:
  - [ADR-wrcuk-navigation-controller-architecture](../adr/ADR-wrcuk-navigation-controller-architecture.md)
- Related Tasks:
  - [T-tto4f-navigation-controller](../tasks/T-tto4f-navigation-controller/README.md)

## Requirement Statement

The system shall implement a navigation controller that calculates steering and throttle commands to navigate from the current GPS position to a target position.

## Rationale

A navigation controller is essential for autonomous operation:

- Converts position target into actuator commands
- Handles path following and course correction
- Provides consistent navigation behavior for all autonomous modes
- Foundation for both Guided and Auto mode operation

The initial implementation uses simplified bearing-to-target navigation (L1-lite), with upgrade path to full L1 and S-curve.

## User Story (if applicable)

As the autonomous navigation system, I need to calculate appropriate steering and throttle commands to reach a target position, so that the vehicle can navigate autonomously in Guided and Auto modes.

## Acceptance Criteria

- [ ] Controller accepts current position and target position as input
- [ ] Controller outputs steering command (-1.0 to +1.0)
- [ ] Controller outputs throttle command (0.0 to 1.0)
- [ ] Steering proportional to heading error (bearing to target vs current heading)
- [ ] Throttle reduces as vehicle approaches target
- [ ] Vehicle stops within WP_RADIUS of target
- [ ] Controller handles GPS update rates from 1Hz to 10Hz
- [ ] Unit tests verify navigation calculations with known positions

## Technical Details (if applicable)

### Functional Requirement Details

**Controller Interface:**

```rust
pub trait NavigationController {
    /// Update navigation state and calculate commands
    fn update(
        &mut self,
        current: &GpsPosition,
        target: &PositionTarget,
        dt: f32,
    ) -> NavigationOutput;
}

pub struct NavigationOutput {
    pub steering: f32,    // -1.0 (left) to +1.0 (right)
    pub throttle: f32,    // 0.0 to 1.0
    pub distance: f32,    // meters to target
    pub bearing: f32,     // degrees to target
    pub at_target: bool,  // within WP_RADIUS
}
```

**Phase 1: Simplified Navigation (L1-lite):**

Basic bearing-to-target steering without full path following:

1. Calculate bearing from current position to target
2. Calculate heading error (bearing - current heading)
3. Steering = heading_error / MAX_HEADING_ERROR (clamped to ±1.0)
4. Calculate distance to target
5. Throttle = min(1.0, distance / APPROACH_DISTANCE)
6. If distance < WP_RADIUS: at_target = true, throttle = 0

**Key Calculations:**

```
bearing = atan2(sin(dlon) * cos(lat2),
                cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon))

distance = 2 * R * asin(sqrt(sin²(dlat/2) + cos(lat1) * cos(lat2) * sin²(dlon/2)))

heading_error = wrap_180(bearing - heading)
```

**Configuration Parameters:**

| Parameter       | Default | Unit    | Description                  |
| --------------- | ------- | ------- | ---------------------------- |
| WP_RADIUS       | 2.0     | m       | Target acceptance radius     |
| WP_SPEED        | 2.0     | m/s     | Maximum navigation speed     |
| APPROACH_DIST   | 10.0    | m       | Distance to start slowing    |
| MAX_HEADING_ERR | 90.0    | degrees | Heading error for full steer |

**Future Enhancement (Phase 2): Full L1 Controller:**

- NAVL1_PERIOD: L1 controller period (default 20s for Rover)
- NAVL1_DAMPING: Damping ratio (default 0.75)
- Path following with cross-track error correction

## Platform Considerations

### Pico W (RP2040)

Trigonometric functions (sin, cos, atan2) use software floating-point. May need optimization or lookup tables for 50Hz update rate.

### Pico 2 W (RP2350)

Hardware FPU supports all navigation calculations at 50Hz without optimization.

### Cross-Platform

Use `libm` crate for transcendental functions. Navigation algorithm identical across platforms.

## Risks & Mitigation

| Risk                                  | Impact | Likelihood | Mitigation                             | Validation                      |
| ------------------------------------- | ------ | ---------- | -------------------------------------- | ------------------------------- |
| Heading wrap-around errors            | Medium | Medium     | Use wrap_180() for all angle diffs     | Test at 0°/360° boundary        |
| GPS noise causes steering oscillation | Medium | Medium     | Add heading error deadband             | Test with simulated noisy GPS   |
| Overshoot at high speed               | Medium | Medium     | Reduce approach speed, tune parameters | Test approach at various speeds |
| Float precision at extreme lat/lon    | Low    | Low        | Use f64 for intermediate calculations  | Test at high-latitude positions |

## Implementation Notes

Preferred approaches:

- Create `src/subsystems/navigation/controller.rs`
- Implement `NavigationController` trait for future extensibility
- Start with `SimpleNavigationController` (L1-lite)
- Use `libm` for trigonometric functions (no_std compatible)

Known pitfalls:

- Latitude/longitude must be in radians for trig functions
- Bearing calculation assumes spherical Earth (sufficient for short distances)
- Heading from GPS (COG) is unreliable at low speeds
- GPS latency means current position is \~200ms old

Related code areas:

- `src/subsystems/navigation/` - Navigation subsystem
- `src/devices/gps.rs` - GPS position data
- `src/communication/mavlink/state.rs` - NavigationState
- `src/rover/mode/` - Mode implementations that use controller

Suggested libraries:

- `libm` - Transcendental math functions for no_std

## External References

- [L1 Controller Paper](https://arc.aiaa.org/doi/10.2514/6.2004-4900) - "A New Nonlinear Guidance Logic for Trajectory Tracking"
- [ArduPilot L1 Controller](https://ardupilot.org/plane/docs/navigation-tuning.html)
- [Haversine Formula](https://en.wikipedia.org/wiki/Haversine_formula)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
