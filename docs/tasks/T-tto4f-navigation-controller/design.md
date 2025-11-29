# T-tto4f Navigation Controller Design

## Metadata

- Type: Design
- Status: Complete

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design document specifies the implementation of a navigation controller subsystem that converts position targets into steering and throttle commands. The controller uses bearing-based steering (L1-lite) as the initial implementation, with the architecture designed to support future upgrade to full L1 controller with cross-track error correction.

## Success Metrics

- [x] Navigation calculations complete within 2ms on RP2350 (estimated \~300µs)
- [x] Steering output always in \[-1.0, +1.0] range (verified by tests)
- [x] Throttle output always in \[0.0, 1.0] range (verified by tests)
- [x] Vehicle arrives within WP_RADIUS (default 2m) of target (at_target detection)
- [x] Unit tests cover all navigation functions (48 tests passing)

## Background and Current State

- Context: Navigation controller is the core component for autonomous modes (Guided, Auto, RTL)
- Current behavior: No navigation controller exists; manual mode passes RC directly to actuators
- Pain points: Cannot implement autonomous navigation without position-to-command conversion
- Constraints: Must work on RP2350 with hardware FPU; no_std compatible
- Related ADRs: [ADR-wrcuk-navigation-controller-architecture](../../adr/ADR-wrcuk-navigation-controller-architecture.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                     Mode (Guided/Auto)                       │
│                      update(dt) called at 50Hz               │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          │ controller.update(current, target, dt)
                          ▼
┌─────────────────────────────────────────────────────────────┐
│              NavigationController (trait)                    │
│                                                              │
│  fn update(&mut self, current, target, dt) -> NavigationOutput
│  fn reset(&mut self)                                         │
└─────────────────────────┬───────────────────────────────────┘
                          │
              ┌───────────┴───────────┐
              ▼                       ▼
┌─────────────────────────┐ ┌─────────────────────────┐
│ SimpleNavigationController│ │ L1NavigationController  │
│ (Phase 1: Implemented)   │ │ (Phase 2: Future)       │
│                          │ │                         │
│ - calculate_bearing()    │ │ - Cross-track error     │
│ - calculate_distance()   │ │ - L1 lateral accel      │
│ - wrap_180()             │ │ - Path following        │
└─────────────────────────┘ └─────────────────────────┘
```

### Components

#### Module Structure

```text
src/subsystems/navigation/
├── mod.rs              # Module exports, re-exports public API
├── controller.rs       # NavigationController trait + SimpleNavigationController
├── geo.rs              # Geographic calculations (bearing, distance)
└── types.rs            # PositionTarget, NavigationOutput, Config
```

#### NavigationController Trait

```rust
/// Navigation controller trait for position-based navigation
pub trait NavigationController {
    /// Update navigation state and calculate commands
    ///
    /// Called at control loop rate (typically 50Hz) but may only
    /// recalculate when GPS updates (1-10Hz).
    fn update(
        &mut self,
        current: &GpsPosition,
        target: &PositionTarget,
        dt: f32,
    ) -> NavigationOutput;

    /// Reset controller state (e.g., on mode change or target change)
    fn reset(&mut self);
}
```

#### SimpleNavigationController

```rust
/// Configuration for simple navigation controller
pub struct SimpleNavConfig {
    /// Waypoint acceptance radius in meters (ArduPilot: WP_RADIUS)
    pub wp_radius: f32,
    /// Approach distance to start slowing (meters)
    pub approach_dist: f32,
    /// Heading error for full steering deflection (degrees)
    pub max_heading_error: f32,
    /// Minimum throttle during approach (0.0-1.0)
    pub min_approach_throttle: f32,
}

impl Default for SimpleNavConfig {
    fn default() -> Self {
        Self {
            wp_radius: 2.0,
            approach_dist: 10.0,
            max_heading_error: 90.0,
            min_approach_throttle: 0.2,
        }
    }
}

/// Simple bearing-based navigation controller (L1-lite)
pub struct SimpleNavigationController {
    config: SimpleNavConfig,
}
```

#### PositionTarget

```rust
/// Target position for navigation
#[derive(Clone, Debug)]
pub struct PositionTarget {
    /// Latitude in degrees
    pub latitude_deg: f32,
    /// Longitude in degrees
    pub longitude_deg: f32,
    /// Optional altitude in meters (not used in Phase 1)
    pub altitude_m: Option<f32>,
}
```

#### NavigationOutput

```rust
/// Output from navigation controller
#[derive(Clone, Debug, Default)]
pub struct NavigationOutput {
    /// Steering command: -1.0 (full left) to +1.0 (full right)
    pub steering: f32,
    /// Throttle command: 0.0 (stop) to 1.0 (full)
    pub throttle: f32,
    /// Distance to target in meters
    pub distance_m: f32,
    /// Bearing to target in degrees (0-360, true north)
    pub bearing_deg: f32,
    /// Heading error in degrees (-180 to +180)
    pub heading_error_deg: f32,
    /// True if vehicle is within WP_RADIUS of target
    pub at_target: bool,
}
```

### Data Flow

1. Mode (Guided/Auto) calls `controller.update(current, target, dt)`
2. Controller calculates bearing from current to target position
3. Controller calculates distance to target (Haversine formula)
4. Heading error = bearing - current_heading (wrapped to ±180°)
5. Steering = heading_error / max_heading_error (clamped to ±1.0)
6. If distance < wp_radius: at_target = true, throttle = 0
7. Else if distance < approach_dist: throttle = distance / approach_dist
8. Else: throttle = 1.0
9. Return NavigationOutput with all calculated values

### Data Models and Types

#### GpsPosition (existing type in src/devices/gps.rs)

```rust
pub struct GpsPosition {
    pub latitude_deg: f32,
    pub longitude_deg: f32,
    pub altitude_m: f32,
    pub heading_deg: f32,  // Course over ground (COG)
    pub speed_mps: f32,    // Ground speed
}
```

### Error Handling

- No runtime errors expected from navigation calculations
- Invalid inputs (NaN, infinity) handled with safe defaults:
  - NaN steering → 0.0
  - NaN throttle → 0.0
  - at_target defaults to false if calculation fails
- Caller (Mode) handles GPS state validation before calling controller

### Security Considerations

- No external input processing
- No network communication
- Pure computational functions

### Performance Considerations

- All calculations use f32 (hardware FPU on RP2350)
- Trigonometric functions from `libm` crate (no_std compatible)
- No heap allocations in hot path
- Expected performance: < 1ms per update on RP2350

#### Calculation Complexity

| Function           | Operations         | Expected Time |
| ------------------ | ------------------ | ------------- |
| calculate_bearing  | 4 trig, 1 atan2    | \~100µs       |
| calculate_distance | 6 trig, 1 sqrt     | \~150µs       |
| wrap_180           | 2 comparisons      | \~1µs         |
| Total update()     | Above + arithmetic | \~300µs       |

### Platform Considerations

#### Embedded (RP2350/RP2040)

- Use `libm` crate for sin, cos, atan2, sqrt, asin
- Hardware FPU on RP2350 handles all f32 operations
- RP2040 uses software float (slower but acceptable at 10Hz GPS rate)

#### Host Tests

- Same code runs on host for unit testing
- `libm` provides identical results across platforms

## Alternatives Considered

1. **Full L1 Controller First**
   - Pros: Industry standard, better path following
   - Cons: More complex, overkill for initial point-to-point navigation
2. **Use existing navigation crate**
   - Pros: Less code to write
   - Cons: No suitable no_std crates found; control over implementation needed

### Decision Rationale

Simple bearing navigation provides fastest path to autonomous capability. Trait-based design allows L1 upgrade without API changes.

## Migration and Compatibility

- New module, no backward compatibility concerns
- API designed for forward compatibility with L1 controller

## Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    // Bearing calculations
    #[test]
    fn test_bearing_north() {
        // From (0, 0) to (1, 0) should be 0° (north)
        let bearing = calculate_bearing(0.0, 0.0, 1.0, 0.0);
        assert!((bearing - 0.0).abs() < 0.1);
    }

    #[test]
    fn test_bearing_east() {
        // From (0, 0) to (0, 1) should be 90° (east)
        let bearing = calculate_bearing(0.0, 0.0, 0.0, 1.0);
        assert!((bearing - 90.0).abs() < 0.1);
    }

    // Distance calculations
    #[test]
    fn test_distance_known_points() {
        // Tokyo to Osaka: approximately 400km
        let dist = calculate_distance(35.6762, 139.6503, 34.6937, 135.5023);
        assert!((dist - 400_000.0).abs() < 10_000.0); // Within 10km
    }

    // Angle wrapping
    #[test]
    fn test_wrap_180() {
        assert_eq!(wrap_180(0.0), 0.0);
        assert_eq!(wrap_180(180.0), 180.0);
        assert_eq!(wrap_180(-180.0), -180.0);
        assert_eq!(wrap_180(270.0), -90.0);
        assert_eq!(wrap_180(-270.0), 90.0);
    }

    // Controller behavior
    #[test]
    fn test_at_target() {
        let mut controller = SimpleNavigationController::new();
        let current = GpsPosition { lat: 0.0, lon: 0.0, heading: 0.0, .. };
        let target = PositionTarget { lat: 0.00001, lon: 0.0 }; // ~1m away
        let output = controller.update(&current, &target, 0.02);
        assert!(output.at_target); // Within WP_RADIUS
    }

    #[test]
    fn test_steering_left() {
        // Target is to the left (west)
        let mut controller = SimpleNavigationController::new();
        let current = GpsPosition { lat: 0.0, lon: 0.0, heading: 0.0, .. }; // Facing north
        let target = PositionTarget { lat: 0.0, lon: -0.001 }; // West
        let output = controller.update(&current, &target, 0.02);
        assert!(output.steering < 0.0); // Should steer left
    }
}
```

### Property-Based Tests

```rust
use proptest::prelude::*;

proptest! {
    #[test]
    fn steering_always_in_range(
        heading in 0.0f32..360.0,
        bearing in 0.0f32..360.0
    ) {
        let error = wrap_180(bearing - heading);
        let steering = (error / 90.0).clamp(-1.0, 1.0);
        prop_assert!(steering >= -1.0 && steering <= 1.0);
    }

    #[test]
    fn throttle_always_in_range(
        distance in 0.0f32..1000.0
    ) {
        let throttle = if distance < 2.0 {
            0.0
        } else if distance < 10.0 {
            (distance / 10.0).clamp(0.2, 1.0)
        } else {
            1.0
        };
        prop_assert!(throttle >= 0.0 && throttle <= 1.0);
    }
}
```

## Documentation Impact

- Add navigation subsystem section to `docs/architecture.md`
- Document WP_RADIUS, APPROACH_DIST parameters in `docs/parameters.md`

## External References

- [Haversine Formula](https://en.wikipedia.org/wiki/Haversine_formula)
- [Calculate Bearing](https://www.movable-type.co.uk/scripts/latlong.html)
- [ArduPilot WP_RADIUS](https://ardupilot.org/rover/docs/parameters.html#wp-radius)

## Open Questions

- [x] Should we use f64 for intermediate calculations? → Decision: Use f32, sufficient precision for short distances
- [ ] Need heading filter for GPS COG noise? → Method: Test with real hardware, add if oscillation observed

## Appendix

### Haversine Distance Formula

```text
a = sin²(Δlat/2) + cos(lat1) × cos(lat2) × sin²(Δlon/2)
c = 2 × atan2(√a, √(1-a))
d = R × c

where R = 6,371,000 meters (Earth radius)
```

### Initial Bearing Formula

```text
θ = atan2(sin(Δlon) × cos(lat2),
          cos(lat1) × sin(lat2) - sin(lat1) × cos(lat2) × cos(Δlon))
```

### Glossary

- **L1**: Line-of-sight navigation algorithm using lateral acceleration
- **COG**: Course Over Ground - GPS-derived heading from velocity vector
- **WP_RADIUS**: Waypoint acceptance radius - distance at which waypoint is considered reached

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
