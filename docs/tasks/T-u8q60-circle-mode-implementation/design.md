# T-u8q60 Circle Mode Implementation Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design document specifies the implementation of Circle mode for the rover. Circle mode enables the vehicle to orbit around a fixed center point at a configurable radius and speed. Following ADR-897ov, we use a hybrid approach: a continuous circle generator feeds look-ahead target points to the existing L1 navigation controller.

## Success Metrics

- [ ] Circle mode calculates target points within 1ms on RP2350
- [ ] Path following error < 2m RMS at typical speeds (2 m/s)
- [ ] Circle state memory < 48 bytes (no heap allocations)
- [ ] All acceptance criteria from FR-khjpl satisfied
- [ ] Unit tests cover all circle calculation functions

## Background and Current State

- Context: Circle mode is a standard ArduPilot Rover mode needed for survey patterns and perimeter patrol
- Current behavior: Only Manual mode is implemented; no autonomous navigation modes exist yet
- Pain points: Cannot perform circular survey patterns or perimeter operations
- Constraints: Must work with GPS update rates of 1-10Hz; RP2350 performance budget
- Related ADRs:
  - [ADR-897ov-circle-mode-path-generation](../../adr/ADR-897ov-circle-mode-path-generation.md)
  - [ADR-wrcuk-navigation-controller-architecture](../../adr/ADR-wrcuk-navigation-controller-architecture.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                    Mode Controller (50Hz)                        │
│                   mode.update(dt) called                         │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                       CircleMode                                 │
│                                                                  │
│  ┌─────────────────┐    ┌──────────────────────────────────┐    │
│  │  Circle State   │    │  calculate_target()               │    │
│  │  - center       │───▶│  1. Current angle from center     │    │
│  │  - radius       │    │  2. Angular velocity calculation  │    │
│  │  - speed        │    │  3. Look-ahead target on circle   │    │
│  │  - direction    │    └───────────────┬──────────────────┘    │
│  └─────────────────┘                    │                        │
│                                         │ target point           │
│                                         ▼                        │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              L1 Navigation Controller                     │   │
│  │              navigate_to(current, target)                 │   │
│  └───────────────────────────┬──────────────────────────────┘   │
│                              │                                   │
│                              ▼                                   │
│                    (steering, throttle)                          │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Actuators                                   │
└─────────────────────────────────────────────────────────────────┘
```

### Components

#### Module Structure

```text
src/rover/mode/
├── mod.rs              # Module exports (add circle)
├── manual.rs           # Existing manual mode
└── circle.rs           # NEW: Circle mode implementation
```

#### CircleMode Struct

```rust
/// Circle orbit direction
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum CircleDirection {
    #[default]
    Clockwise,
    CounterClockwise,
}

/// Circle mode state
pub struct CircleMode {
    /// Center point of the circle (set on mode entry)
    center: GpsPosition,
    /// Circle radius in meters
    radius: f32,
    /// Target speed in m/s
    speed: f32,
    /// Orbit direction
    direction: CircleDirection,
    /// Navigation controller reference
    nav_controller: &'static NavigationController,
    /// GPS state reference
    gps_state: &'static GpsNavigationState,
    /// AHRS reference for heading
    ahrs: &'static Ahrs,
}
```

#### Circle Target Calculation

```rust
impl CircleMode {
    /// Look-ahead time in seconds
    const LOOK_AHEAD_TIME: f32 = 1.5;

    /// Calculate the next target point on the circle perimeter
    pub fn calculate_target(&self, current: &GpsPosition) -> GpsPosition {
        // 1. Calculate current angle from center to vehicle
        let current_angle = calculate_bearing(&self.center, current);

        // 2. Calculate angular velocity: ω = v / r (rad/s)
        let angular_velocity = self.speed / self.radius;
        let angular_velocity_deg = angular_velocity.to_degrees();

        // 3. Calculate look-ahead angle
        let look_ahead_angle = angular_velocity_deg * Self::LOOK_AHEAD_TIME;

        // 4. Calculate target angle based on direction
        let target_angle = match self.direction {
            CircleDirection::Clockwise => current_angle + look_ahead_angle,
            CircleDirection::CounterClockwise => current_angle - look_ahead_angle,
        };

        // 5. Return point on circle perimeter at target angle
        offset_position(&self.center, self.radius, target_angle)
    }
}
```

### Data Flow

1. **Mode Entry** (`enter()`):
   - Validate GPS has 3D fix
   - Get current position and heading (AHRS or GPS COG)
   - Load CIRC_RADIUS, CIRC_SPEED, CIRC_DIR parameters
   - Calculate center point: offset CIRC_RADIUS ahead in heading direction
   - Store circle state

2. **Mode Update** (`update(dt)`):
   - If CIRC_RADIUS == 0: set steering=0, throttle=0 (stationary)
   - Get current GPS position
   - Calculate look-ahead target point on circle
   - Call navigation controller with current position and target
   - Apply steering and throttle to actuators

3. **Mode Exit** (`exit()`):
   - Set actuators to neutral
   - Clear circle state

### Data Models and Types

#### GpsPosition (existing)

```rust
pub struct GpsPosition {
    pub latitude_deg: f32,
    pub longitude_deg: f32,
    pub altitude_m: f32,
}
```

#### Circle Parameters (new, in parameter system)

| Parameter     | Type | Default | Range  | Description             |
| ------------- | ---- | ------- | ------ | ----------------------- |
| `CIRC_RADIUS` | f32  | 20.0    | 0-1000 | Circle radius in meters |
| `CIRC_SPEED`  | f32  | 2.0     | 0-10   | Target speed in m/s     |
| `CIRC_DIR`    | i8   | 0       | 0-1    | 0=CW, 1=CCW             |

### Error Handling

- **No GPS fix**: Return `Err("No GPS fix")` from `enter()`, mode not entered
- **No valid heading**: Return `Err("No heading available")` from `enter()`
- **GPS fix lost during operation**: Transition to Hold mode (future integration)
- **Invalid parameters**: Use defaults if parameters missing or invalid

### Security Considerations

- No external input processing beyond validated GPS data
- Parameters validated before use
- No heap allocations that could be exploited

### Performance Considerations

- All calculations use f32 (hardware FPU on RP2350)
- No heap allocations in update loop
- Trigonometric functions from `libm` crate
- Pre-compute angular velocity on mode entry (not every update)

#### Expected Performance

| Operation                | Time (RP2350) | Time (RP2040) |
| ------------------------ | ------------- | ------------- |
| calculate_bearing()      | \~100µs       | \~300µs       |
| offset_position()        | \~150µs       | \~400µs       |
| Total calculate_target() | \~300µs       | \~800µs       |

### Platform Considerations

#### Embedded (RP2350/RP2040)

- Use `libm` for trigonometric functions
- Hardware FPU on RP2350, software FPU on RP2040
- Same code path, different performance characteristics

#### Host Tests

- Same algorithms run on host for unit testing
- Mock GPS state and navigation controller for integration tests

## Alternatives Considered

1. **Discrete Waypoint Chain**
   - Pros: Reuses waypoint infrastructure completely
   - Cons: Jerky motion at waypoint transitions, higher memory
2. **Direct Angular Velocity Control**
   - Pros: Simplest geometry, most direct control
   - Cons: Bypasses navigation controller, no drift correction

### Decision Rationale

Hybrid approach selected per ADR-897ov. Reuses L1 controller for proven path following while generating smooth continuous motion. Minimal additional code (\~100 lines for circle generator).

## Migration and Compatibility

- New mode, no backward compatibility concerns
- Parameters use ArduPilot-standard names for GCS compatibility
- No breaking changes to existing modes

## Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calculate_target_clockwise() {
        // Vehicle at 12 o'clock position, circling clockwise
        let center = GpsPosition { lat: 0.0, lon: 0.0, alt: 0.0 };
        let current = offset_position(&center, 20.0, 0.0); // North of center
        let mode = CircleMode::new(center, 20.0, 2.0, CircleDirection::Clockwise);

        let target = mode.calculate_target(&current);
        // Target should be to the east (clockwise from north)
        let target_bearing = calculate_bearing(&center, &target);
        assert!(target_bearing > 0.0 && target_bearing < 90.0);
    }

    #[test]
    fn test_calculate_target_counterclockwise() {
        let center = GpsPosition { lat: 0.0, lon: 0.0, alt: 0.0 };
        let current = offset_position(&center, 20.0, 0.0);
        let mode = CircleMode::new(center, 20.0, 2.0, CircleDirection::CounterClockwise);

        let target = mode.calculate_target(&current);
        let target_bearing = calculate_bearing(&center, &target);
        // Target should be to the west (counterclockwise from north)
        assert!(target_bearing > 270.0 || target_bearing < 0.0);
    }

    #[test]
    fn test_center_point_calculation() {
        // Vehicle facing north, radius 20m
        let current = GpsPosition { lat: 35.6762, lon: 139.6503, alt: 0.0 };
        let heading = 0.0; // North
        let radius = 20.0;

        let center = calculate_center_on_entry(&current, heading, radius);
        let distance = calculate_distance(&current, &center);

        assert!((distance - radius).abs() < 0.5); // Within 0.5m
    }

    #[test]
    fn test_stationary_mode() {
        // CIRC_RADIUS = 0 should result in no movement
        let mode = CircleMode::with_radius(0.0);
        let output = mode.update(0.02);
        assert_eq!(output.steering, 0.0);
        assert_eq!(output.throttle, 0.0);
    }
}
```

### Integration Tests

- Test mode entry with mocked GPS state
- Test mode transition from Manual to Circle
- Test GPS loss handling (future)

## Documentation Impact

- Add Circle mode to rover mode documentation
- Document CIRC\_\* parameters in parameter reference
- Update architecture.md with Circle mode module

## External References

- [ArduPilot Circle Mode (Rover)](https://ardupilot.org/rover/docs/circle-mode.html)
- [Movable Type Scripts - Geodetic Calculations](https://www.movable-type.co.uk/scripts/latlong.html)
- [L1 Navigation Controller](https://ardupilot.org/plane/docs/navigation-tuning.html)

## Open Questions

- [ ] Optimal look-ahead time for different speeds? → Method: Empirical testing on hardware
- [ ] Should Circle mode support dynamic center update via MAVLink? → Deferred to future enhancement

## Appendix

### Geodetic Formulas

**Bearing from point A to B:**

```text
θ = atan2(sin(Δλ)·cos(φ₂), cos(φ₁)·sin(φ₂) − sin(φ₁)·cos(φ₂)·cos(Δλ))
```

**Destination point given distance and bearing:**

```text
φ₂ = asin(sin(φ₁)·cos(d/R) + cos(φ₁)·sin(d/R)·cos(θ))
λ₂ = λ₁ + atan2(sin(θ)·sin(d/R)·cos(φ₁), cos(d/R) − sin(φ₁)·sin(φ₂))
```

### Glossary

- **L1**: Line-of-sight navigation algorithm using lateral acceleration
- **COG**: Course Over Ground - GPS-derived heading from velocity vector
- **Look-ahead**: Distance/time ahead on path used for target calculation

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
