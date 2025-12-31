# FR-khjpl Circle Mode Implementation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-8dqwi-circle-mode](../analysis/AN-8dqwi-circle-mode.md)
- Prerequisite Requirements:
  - [FR-2vbe8-navigation-controller](FR-2vbe8-navigation-controller.md)
  - [FR-cs42u-gps-navigation-state-access](FR-cs42u-gps-navigation-state-access.md)
  - [FR-q2sjt-control-mode-framework](FR-q2sjt-control-mode-framework.md)
  - [FR-m2noq-circle-center-point](FR-m2noq-circle-center-point.md)
  - [FR-7d203-circle-mode-parameters](FR-7d203-circle-mode-parameters.md)
- Dependent Requirements:
  - [NFR-p8ylh-circle-path-calculation-performance](NFR-p8ylh-circle-path-calculation-performance.md)
- Related ADRs:
  - [ADR-897ov-circle-mode-path-generation](../adr/ADR-897ov-circle-mode-path-generation.md)
- Related Tasks:
  - [T-u8q60-circle-mode-implementation](../tasks/T-u8q60-circle-mode-implementation/README.md)

## Requirement Statement

The system shall implement Circle mode that orbits around a fixed center point at a configurable radius and speed, integrating with the existing navigation controller using a continuous circular path generator (hybrid approach).

## Rationale

Circle mode provides essential capabilities for rover and boat operations:

- Survey and inspection patterns around a specific point
- Perimeter patrol of a fixed location
- Aerial photography ground patterns (adapted for surface vehicles)
- Standard ArduPilot Rover mode expected by GCS applications

The hybrid approach (continuous circle generator + L1 controller) provides smooth path following while reusing the existing navigation infrastructure.

## User Story (if applicable)

As a rover operator, I want to use Circle mode to orbit around a specific point at a defined radius and speed, so that I can perform area surveys or perimeter patrols without manual control.

## Acceptance Criteria

- [ ] Circle mode entry requires valid GPS fix (3D fix minimum)
- [ ] Mode calculates center point on entry (CIRC_RADIUS meters ahead)
- [ ] Vehicle navigates in circular path around center point
- [ ] Orbit direction follows CIRC_DIR parameter (0=CW, 1=CCW)
- [ ] Orbit speed follows CIRC_SPEED parameter
- [ ] Orbit radius follows CIRC_RADIUS parameter
- [ ] When CIRC_RADIUS=0, vehicle remains stationary
- [ ] Mode transitions to Hold if GPS fix is lost
- [ ] Mode exit clears circle state
- [ ] HEARTBEAT custom_mode reports Circle mode
- [ ] STATUSTEXT sent on mode entry: "Circle mode entered"

## Technical Details (if applicable)

### Functional Requirement Details

**Mode Entry Conditions:**

- GPS fix type >= 3D_FIX
- CIRC_RADIUS > 0 (or stationary mode if 0)
- Vehicle not in emergency stop state

**Hybrid Path Generation Algorithm:**

1. On entry: Calculate center point (CIRC_RADIUS meters ahead in current heading direction)
2. Calculate angular velocity: `ω = CIRC_SPEED / CIRC_RADIUS` (rad/s)
3. Each update cycle:
   - Calculate current angle from center to vehicle
   - Compute look-ahead target point on circle perimeter (1.5s ahead)
   - Feed target point to L1 navigation controller
   - Output steering/throttle commands to actuators

**Look-ahead Target Calculation:**

```rust
fn calculate_circle_target(&self, current: &GpsPosition, dt: f32) -> GpsPosition {
    let current_angle = bearing_deg(&self.center, current);
    let angular_velocity_deg = (self.speed / self.radius).to_degrees();
    let look_ahead_time = 1.5; // seconds
    let look_ahead_angle = angular_velocity_deg * look_ahead_time;

    let target_angle = match self.direction {
        CircleDirection::Clockwise => current_angle + look_ahead_angle,
        CircleDirection::CounterClockwise => current_angle - look_ahead_angle,
    };

    offset_position(&self.center, self.radius, target_angle)
}
```

**State Diagram:**

```
              [Entry with GPS fix]
                      |
                      v
              +---------------+
              | Calc Center   |
              +---------------+
                      |
     CIRC_RADIUS=0?   |
     +----------------+----------------+
     |                                 |
     v                                 v
+-----------+                 +----------------+
| Stationary|                 |   Circling     |
+-----------+                 +----------------+
                                   |       |
                        GPS lost   |       | Mode exit
                                   v       v
                              +--------+ +------+
                              | ->Hold | | Exit |
                              +--------+ +------+
```

**ArduPilot Parameters:**

| Parameter     | Type  | Default | Description             |
| ------------- | ----- | ------- | ----------------------- |
| `CIRC_RADIUS` | Float | 20.0    | Circle radius in meters |
| `CIRC_SPEED`  | Float | 2.0     | Target speed in m/s     |
| `CIRC_DIR`    | Int   | 0       | Direction: 0=CW, 1=CCW  |

## Platform Considerations

### Pico W (RP2040)

Circle calculations require trigonometric functions. May need optimization or reduced update rate on RP2040.

### Pico 2 W (RP2350)

Full floating-point circle calculations supported at navigation update rate.

### Cross-Platform

Mode logic identical across platforms; trigonometric calculations are the primary performance concern.

## Risks & Mitigation

| Risk                                  | Impact | Likelihood | Mitigation                      | Validation                       |
| ------------------------------------- | ------ | ---------- | ------------------------------- | -------------------------------- |
| GPS drift causes erratic path         | Medium | Medium     | Tune look-ahead distance        | Test with real GPS               |
| High speed + small radius instability | Medium | Low        | Limit min radius based on speed | Test boundary conditions         |
| Center point calculation error        | Medium | Low        | Unit tests for offset_position  | Validate with known coordinates  |
| Slow GPS causes lag in path           | Low    | Medium     | Use dead reckoning if available | Test at various GPS update rates |

## Implementation Notes

Preferred approaches:

- Implement as `CircleMode` struct implementing `Mode` trait
- Store center point and parameters in mode state
- Delegate path following to existing L1 navigation controller
- Use `offset_position` utility for geodetic calculations

Known pitfalls:

- Ensure center point is calculated once on mode entry, not every update
- Handle CIRC_RADIUS=0 case (stationary) explicitly
- Parameter changes require mode re-entry to take effect

Related code areas:

- `src/rover/mode/` - Mode implementations
- `src/subsystems/navigation/` - Navigation controller
- `src/core/math/` - Geodetic calculations

## External References

- [ArduPilot Circle Mode (Rover)](https://ardupilot.org/rover/docs/circle-mode.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
