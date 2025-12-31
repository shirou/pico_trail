# T-n24yy Rover Loiter Mode Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design document specifies the implementation of Loiter mode for ground rovers. Loiter mode enables position holding at a fixed GPS point, supporting two behavioral types: Type 0 (simple stop) and Type 1 (active position correction). The implementation follows the Mode trait pattern from ADR-w9zpl and the vehicle type separation from ADR-8icsq.

## Success Metrics

- [ ] Loiter mode activatable via MAVLink DO_SET_MODE
- [ ] Mode entry records loiter point correctly
- [ ] Type 0 outputs zero steering/throttle
- [ ] Type 1 detects drift and triggers correction
- [ ] Drift detection completes within 1ms
- [ ] Unit tests cover all loiter state transitions

## Background and Current State

- Context: Mode system exists with Manual, Hold modes implemented
- Current behavior: Hold mode stops motors but has no GPS-based position tracking
- Pain points: No way to maintain position with GPS backup for drift correction
- Constraints: Must work with existing Mode trait, navigation controller, GPS state
- Related ADRs:
  - [ADR-w9zpl-control-mode-architecture](../../adr/ADR-w9zpl-control-mode-architecture.md) - Mode trait pattern
  - [ADR-8icsq-vehicle-type-separation](../../adr/ADR-8icsq-vehicle-type-separation.md) - Rover/Boat separation

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                 Mode Manager (50 Hz)                        │
│                 calls loiter.update(dt)                     │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│                    RoverLoiter                              │
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              LoiterState                             │   │
│  │  - loiter_point: GpsPosition                         │   │
│  │  - loiter_type: u8 (0 or 1)                          │   │
│  │  - radius: f32 (LOIT_RADIUS)                         │   │
│  │  - is_correcting: bool                               │   │
│  └─────────────────────────────────────────────────────┘   │
│                          │                                  │
│            ┌─────────────┴─────────────┐                    │
│            │                           │                    │
│            ▼                           ▼                    │
│     ┌───────────┐             ┌───────────────┐            │
│     │  Type 0   │             │    Type 1     │            │
│     │   Stop    │             │  Active Hold  │            │
│     │           │             │               │            │
│     │ steering=0│             │ check_drift() │            │
│     │ throttle=0│             │ navigate_if   │            │
│     └───────────┘             │ needed        │            │
│                               └───────────────┘            │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│               NavigationController                          │
│  (used only in Type 1 when is_correcting == true)          │
└─────────────────────────────────────────────────────────────┘
```

### Components

#### Module Structure

```text
src/rover/mode/
├── mod.rs          # FlightMode enum, Mode trait exports
├── manual.rs       # Manual mode (existing)
├── hold.rs         # Hold mode (existing)
└── loiter.rs       # Loiter mode (NEW)

src/parameters/
├── mod.rs          # Parameter registry
└── loiter.rs       # LOIT_TYPE, LOIT_RADIUS parameters (NEW)
```

#### RoverLoiter Struct

```rust
/// Rover Loiter mode - position holding with optional active correction
#[cfg(feature = "rover")]
pub struct RoverLoiter {
    /// Loiter state (initialized on mode entry)
    state: Option<LoiterState>,
    /// Navigation controller for position correction (Type 1)
    nav_controller: SimpleNavigationController,
}

/// Loiter mode state
pub struct LoiterState {
    /// Recorded loiter position
    loiter_point: GpsPosition,
    /// Type of loiter behavior (0=stop, 1=active hold)
    loiter_type: u8,
    /// Acceptable drift radius in meters
    radius: f32,
    /// True if currently correcting position (Type 1 only)
    is_correcting: bool,
}
```

#### Mode Trait Implementation

```rust
#[cfg(feature = "rover")]
impl Mode for RoverLoiter {
    fn enter(&mut self) -> Result<(), &'static str> {
        // 1. Validate GPS fix
        let gps_state = get_gps_state();
        if !gps_state.has_valid_fix() {
            return Err("Loiter requires GPS fix");
        }

        // 2. Read parameters
        let loit_type = params::get_loit_type();
        let loit_radius = params::get_loit_radius();

        // 3. Calculate loiter point
        let loiter_point = self.calculate_loiter_point(&gps_state);

        // 4. Initialize state
        self.state = Some(LoiterState {
            loiter_point,
            loiter_type: loit_type,
            radius: loit_radius,
            is_correcting: false,
        });

        log_info!("Loiter: entered at {:?}", loiter_point);
        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        let state = self.state.as_mut().ok_or("Loiter not initialized")?;

        match state.loiter_type {
            0 => self.update_type0(),
            1 => self.update_type1(state, dt),
            _ => self.update_type0(), // Default to stop
        }
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        // Set actuators to neutral
        set_steering(0.0);
        set_throttle(0.0);
        self.state = None;
        self.nav_controller.reset();
        log_info!("Loiter: exited");
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Loiter"
    }
}
```

#### Type 0 Implementation (Stop)

```rust
fn update_type0(&self) -> Result<(), &'static str> {
    set_steering(0.0);
    set_throttle(0.0);
    Ok(())
}
```

#### Type 1 Implementation (Active Hold)

```rust
fn update_type1(&mut self, state: &mut LoiterState, dt: f32) -> Result<(), &'static str> {
    let gps_state = get_gps_state();

    // Check for GPS validity
    if !gps_state.has_valid_fix() {
        // Degrade to Type 0 if GPS lost
        return self.update_type0();
    }

    let current_pos = gps_state.position();

    // Calculate distance to loiter point
    let distance = distance_m(&current_pos, &state.loiter_point);

    // Hysteresis state machine
    const HYSTERESIS_FACTOR: f32 = 0.8;

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

    if state.is_correcting {
        // Navigate back to loiter point
        let target = PositionTarget {
            latitude_deg: state.loiter_point.latitude_deg,
            longitude_deg: state.loiter_point.longitude_deg,
            altitude_m: None,
        };

        let nav_output = self.nav_controller.update(&current_pos, &target, dt);

        set_steering(nav_output.steering);
        set_throttle(nav_output.throttle);
    } else {
        // Within radius - stop
        set_steering(0.0);
        set_throttle(0.0);
    }

    Ok(())
}
```

#### Loiter Point Calculation

```rust
fn calculate_loiter_point(&self, gps_state: &GpsNavigationState) -> GpsPosition {
    let current = gps_state.position();
    let speed = gps_state.ground_speed_mps();

    // Low speed threshold - use current position
    const LOW_SPEED_THRESHOLD: f32 = 0.5;
    if speed < LOW_SPEED_THRESHOLD {
        return current.clone();
    }

    // Project stop point based on deceleration
    let max_decel = params::get_atc_decel_max().unwrap_or(1.0); // m/s^2
    let stop_distance = (speed * speed) / (2.0 * max_decel);

    // Cap projection to reasonable distance
    const MAX_PROJECTION: f32 = 50.0;
    let stop_distance = stop_distance.min(MAX_PROJECTION);

    // Project in direction of travel
    let heading = gps_state.course_over_ground_deg();
    offset_position(&current, stop_distance, heading)
}
```

### Data Flow

1. Mode Manager calls `loiter.update(dt)` at 50 Hz
2. Loiter mode reads current GPS position
3. Type 0: Output zero steering/throttle
4. Type 1:
   - Calculate distance from loiter point
   - Apply hysteresis to determine is_correcting state
   - If correcting: use navigation controller to return to loiter point
   - If not correcting: output zero steering/throttle

### Parameters

| Parameter     | Type | Range    | Default | Description                       |
| ------------- | ---- | -------- | ------- | --------------------------------- |
| `LOIT_TYPE`   | u8   | 0-1      | 0       | 0=stop, 1=active position hold    |
| `LOIT_RADIUS` | f32  | 0.5-100m | 2.0     | Drift threshold before correction |

### Error Handling

- GPS fix lost during Loiter: Degrade to Type 0 behavior (stop motors)
- Invalid state: Return error from update(), mode manager handles fallback
- NaN in calculations: Use safe defaults (steering=0, throttle=0)

### Performance Considerations

- Distance calculation uses Haversine formula (\~150µs on RP2350)
- Hysteresis check is simple comparisons (\~1µs)
- Total update should complete within 1ms (NFR-biag9)
- Navigation controller update adds \~300µs when correcting

| Operation                  | Expected Time |
| -------------------------- | ------------- |
| GPS state read             | \~50µs        |
| Distance calculation       | \~150µs       |
| Hysteresis check           | \~1µs         |
| Navigation update          | \~300µs       |
| **Total (correcting)**     | \~500µs       |
| **Total (not correcting)** | \~200µs       |

### Platform Considerations

- Feature gated with `#[cfg(feature = "rover")]`
- Uses existing navigation calculations from `src/subsystems/navigation/geo.rs`
- Uses existing `SimpleNavigationController` for position correction
- no_std compatible, uses `libm` for trigonometric functions

## Alternatives Considered

1. **Unified Rover/Boat Loiter**
   - Pros: Single implementation, less code
   - Cons: Different behavioral requirements, complex conditionals
   - Decision: Rejected per ADR-8icsq

2. **Custom navigation for Loiter correction**
   - Pros: Specialized behavior for loiter return
   - Cons: Duplicates navigation code, harder to maintain
   - Decision: Rejected, reuse SimpleNavigationController

## Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_loiter_point_at_rest() {
        // When stopped, loiter point should be current position
        let gps = mock_gps_state(0.0, 0.0, 0.0); // No speed
        let loiter = RoverLoiter::new();
        let point = loiter.calculate_loiter_point(&gps);
        assert_eq!(point.latitude_deg, 0.0);
        assert_eq!(point.longitude_deg, 0.0);
    }

    #[test]
    fn test_loiter_point_moving() {
        // When moving, loiter point should be projected forward
        let gps = mock_gps_state(0.0, 0.0, 5.0); // 5 m/s north
        let loiter = RoverLoiter::new();
        let point = loiter.calculate_loiter_point(&gps);
        assert!(point.latitude_deg > 0.0); // Should be north of current
    }

    #[test]
    fn test_type0_stops() {
        let mut loiter = RoverLoiter::new();
        loiter.state = Some(LoiterState {
            loiter_point: GpsPosition::default(),
            loiter_type: 0,
            radius: 2.0,
            is_correcting: false,
        });

        loiter.update_type0().unwrap();
        // Verify steering and throttle are zero
    }

    #[test]
    fn test_type1_detects_drift() {
        let mut loiter = RoverLoiter::new();
        let mut state = LoiterState {
            loiter_point: GpsPosition { latitude_deg: 0.0, longitude_deg: 0.0, .. },
            loiter_type: 1,
            radius: 2.0,
            is_correcting: false,
        };

        // Position 5m away (outside radius)
        let current = GpsPosition { latitude_deg: 0.00005, .. }; // ~5m north
        let distance = distance_m(&current, &state.loiter_point);

        assert!(distance > state.radius);
        // After update, is_correcting should be true
    }

    #[test]
    fn test_hysteresis_prevents_oscillation() {
        let mut state = LoiterState {
            loiter_point: GpsPosition::default(),
            loiter_type: 1,
            radius: 2.0,
            is_correcting: true,
        };

        // Distance at 1.7m (85% of radius) - should still be correcting
        // due to hysteresis factor of 0.8
        let distance = 1.7;
        if distance < state.radius * 0.8 {
            state.is_correcting = false;
        }
        assert!(state.is_correcting); // 1.7 > 1.6 (2.0 * 0.8)

        // Distance at 1.5m - should stop correcting
        let distance = 1.5;
        if distance < state.radius * 0.8 {
            state.is_correcting = false;
        }
        assert!(!state.is_correcting); // 1.5 < 1.6
    }
}
```

### Integration Tests

- Test mode entry with valid GPS fix
- Test mode entry rejection without GPS fix
- Test mode transition from Manual to Loiter
- Test parameter changes during Loiter mode

## Documentation Impact

- Update `docs/architecture.md` with Loiter mode description
- Add LOIT_TYPE, LOIT_RADIUS to `docs/parameters.md`

## Open Questions

- [ ] Should we support reverse navigation for position correction? → Method: Test with hardware, ArduPilot supports it
- [ ] What minimum LOIT_RADIUS is practical? → Method: Test with GPS noise levels
- [ ] Should we warn user if LOIT_RADIUS < 2m? → Decision: Add STATUSTEXT warning

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
