# ADR-h3k9f Heading Source Integration Strategy

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-2vbe8-navigation-controller](../requirements/FR-2vbe8-navigation-controller.md)
  - [FR-eyuh8-ahrs-attitude-estimation](../requirements/FR-eyuh8-ahrs-attitude-estimation.md)
  - [FR-erpze-guided-mode-navigation](../requirements/FR-erpze-guided-mode-navigation.md)
  - [FR-jm7mj-auto-mode-mission-execution](../requirements/FR-jm7mj-auto-mode-mission-execution.md)
- Related ADRs:
  - [ADR-nzvfy-ahrs-abstraction-architecture](ADR-nzvfy-ahrs-abstraction-architecture.md)
  - [ADR-wrcuk-navigation-controller-architecture](ADR-wrcuk-navigation-controller-architecture.md)
- Related Analysis:
  - [AN-vcxr7-ahrs-navigation-control-integration](../analysis/AN-vcxr7-ahrs-navigation-control-integration.md)
- Related Tasks:
  - [T-r9v2k-heading-source-navigation-integration](../tasks/T-r9v2k-heading-source-navigation-integration/README.md)

## Context

The NavigationController currently uses GPS Course-Over-Ground (COG) exclusively for heading:

```rust
// src/subsystems/navigation/controller.rs:130
let current_heading = current.course_over_ground.unwrap_or(0.0);
```

This approach has limitations:

1. **Stationary vehicle problem**: GPS COG is only valid when the vehicle is moving (speed >= 0.5 m/s). When stationary, heading defaults to 0° (north), causing incorrect initial steering.

2. **Low-speed unreliability**: At low speeds, GPS COG can be noisy or unavailable.

3. **AHRS data unused**: The BNO086 External AHRS provides accurate heading via `SharedAhrsState`, but this data is not consumed by the navigation system.

**Existing pattern**: `CircleMode` already implements AHRS-first heading with GPS COG fallback via a `heading_provider: fn() -> Option<f32>` function pointer.

**Constraint**: The solution must depend only on the `SharedAhrsState` interface, not on specific AHRS implementations (BNO086, MPU9250, etc.), to allow future sensor additions.

## Decision

We will introduce a `HeadingSource` trait and `FusedHeadingSource` implementation that combines AHRS yaw and GPS COG based on vehicle speed.

### Decision Drivers

- AHRS provides accurate heading when stationary
- GPS COG provides drift-free heading when moving
- Must support multiple AHRS implementations via trait abstraction
- Consistent with existing `CircleMode` pattern

### Considered Options

- Option A: Function pointer injection (current CircleMode pattern)
- Option B: Direct SharedAhrsState reference in NavigationController
- Option C: HeadingSource trait with FusedHeadingSource implementation

### Option Analysis

- Option A — Pros: Simple, proven pattern | Cons: No encapsulation of fusion logic, each mode duplicates fallback code
- Option B — Pros: Direct access, minimal indirection | Cons: Tight coupling, harder to test, no abstraction for different fusion strategies
- Option C — Pros: Clean abstraction, testable, encapsulates fusion logic, extensible | Cons: Additional trait complexity

**Selected: Option C**

### HeadingSource Trait

```rust
// src/subsystems/navigation/heading.rs

/// Provides heading information for navigation
pub trait HeadingSource {
    /// Returns current heading in degrees (0-360, 0 = North)
    /// Returns None if no valid heading is available
    fn get_heading(&self) -> Option<f32>;

    /// Returns true if heading source is healthy and providing valid data
    fn is_valid(&self) -> bool;

    /// Returns the current heading source type for telemetry
    fn source_type(&self) -> HeadingSourceType;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HeadingSourceType {
    Ahrs,      // AHRS yaw (stationary or low speed)
    GpsCog,    // GPS Course Over Ground (moving)
    None,      // No valid source
}
```

### FusedHeadingSource Implementation

```rust
pub struct FusedHeadingSource {
    ahrs_state: &'static SharedAhrsState,
    gps_provider: fn() -> Option<GpsPosition>,
    speed_threshold: f32,  // m/s, default 1.0
}

impl HeadingSource for FusedHeadingSource {
    fn get_heading(&self) -> Option<f32> {
        let gps = (self.gps_provider)();
        let ahrs_healthy = self.ahrs_state.is_healthy();

        // Use GPS COG when moving fast enough and COG is available
        if let Some(ref gps) = gps {
            if gps.speed >= self.speed_threshold {
                if let Some(cog) = gps.course_over_ground {
                    return Some(cog);
                }
            }
        }

        // Fall back to AHRS yaw when stationary or GPS COG unavailable
        if ahrs_healthy {
            let yaw_rad = self.ahrs_state.get_yaw();
            return Some(yaw_rad.to_degrees().rem_euclid(360.0));
        }

        // Last resort: GPS COG even if below speed threshold
        gps.and_then(|g| g.course_over_ground)
    }

    fn is_valid(&self) -> bool {
        self.ahrs_state.is_healthy() || (self.gps_provider)().is_some()
    }

    fn source_type(&self) -> HeadingSourceType {
        let gps = (self.gps_provider)();
        if let Some(ref gps) = gps {
            if gps.speed >= self.speed_threshold && gps.course_over_ground.is_some() {
                return HeadingSourceType::GpsCog;
            }
        }
        if self.ahrs_state.is_healthy() {
            return HeadingSourceType::Ahrs;
        }
        HeadingSourceType::None
    }
}
```

### NavigationController Integration

The `NavigationController` trait will accept heading via a new parameter:

```rust
pub trait NavigationController {
    fn update(
        &mut self,
        current: &GpsPosition,
        target: &PositionTarget,
        heading: f32,  // NEW: heading in degrees from HeadingSource
        dt: f32,
    ) -> NavigationOutput;

    fn reset(&mut self);
}
```

Mode implementations will obtain heading from `HeadingSource` and pass it to the controller.

## Rationale

Option C (HeadingSource trait) was selected because:

1. **Encapsulation**: Fusion logic is in one place, not duplicated across modes
2. **Testability**: HeadingSource can be mocked for unit tests
3. **Extensibility**: Future heading sources (dual GPS, external compass) can implement the trait
4. **Telemetry support**: `source_type()` enables GCS to display active heading source
5. **Consistency**: Follows existing trait patterns (Ahrs, NavigationController, Mode)

The speed threshold of 1.0 m/s aligns with ArduPilot conventions and provides a reasonable boundary between stationary and moving states.

## Consequences

### Positive

- Accurate heading available immediately after AHRS initialization
- No heading discontinuity when starting from stationary
- Clear separation between heading source and navigation logic
- Enables future heading source implementations without modifying NavigationController

### Negative

- Additional abstraction layer increases complexity
- Modes must be updated to use HeadingSource instead of direct GPS COG access
- Slight runtime overhead from function pointer indirection

### Neutral

- Existing modes (RTL, SmartRTL, Loiter) will need updates to use HeadingSource
- CircleMode already has similar logic and can be refactored to use HeadingSource

## Implementation Notes

**Phase 1: HeadingSource Infrastructure**

1. Create `src/subsystems/navigation/heading.rs`
2. Implement `HeadingSource` trait and `FusedHeadingSource`
3. Add `HeadingSourceType` enum for telemetry

**Phase 2: NavigationController Integration**

1. Update `NavigationController` trait to accept heading parameter
2. Update `SimpleNavigationController` implementation
3. Remove internal COG fallback logic from controller

**Phase 3: Mode Updates**

1. Update RTL, SmartRTL, Loiter modes to use HeadingSource
2. Refactor CircleMode to use HeadingSource (replace inline logic)
3. Implement Guided and Auto modes with HeadingSource

**Phase 4: Main Application Integration**

1. Instantiate `FusedHeadingSource` with `SharedAhrsState` reference
2. Pass to mode constructors
3. Spawn BNO086 AHRS task to populate `SharedAhrsState`

## Examples

```rust
// Main application setup
use crate::subsystems::ahrs::AHRS_STATE;
use crate::subsystems::navigation::heading::{FusedHeadingSource, HeadingSource};

// Create fused heading source
let heading_source = FusedHeadingSource::new(
    &AHRS_STATE,
    || GPS_POSITION.try_lock().ok().and_then(|g| g.clone()),
    1.0,  // speed threshold m/s
);

// Mode usage
fn update(&mut self, dt: f32) -> Result<(), &'static str> {
    let gps = (self.gps_provider)().ok_or("No GPS")?;
    let heading = self.heading_source.get_heading().ok_or("No heading")?;
    let target = self.get_target()?;

    let output = self.nav_controller.update(&gps, &target, heading, dt);
    self.actuators.set_steering(output.steering);
    self.actuators.set_throttle(output.throttle);
    Ok(())
}
```

## Open Questions

- [x] Which heading fusion strategy to use? → Simple speed-based switch (Option 1 from Analysis)
- [ ] Should mode entry require AHRS healthy status? → Next step: Define in Guided/Auto mode requirements
- [ ] Is BNO086 yaw in NED frame (0° = North)? → Method: Verify in hardware testing

## External References

- [ArduPilot EKF Source Selection](https://ardupilot.org/copter/docs/common-ekf-sources.html)
- [ArduPilot Compass-less Operation](https://ardupilot.org/rover/docs/common-compassless.html)
