# ADR-wrcuk Navigation Controller Architecture: Bearing-Based Path Following

## Metadata

- Type: ADR
- Status: Draft

## Links

- Related Analyses:
  - [AN-27568-position-target-navigation](../analysis/AN-27568-position-target-navigation.md)
- Impacted Requirements:
  - [FR-2vbe8-navigation-controller](../requirements/FR-2vbe8-navigation-controller.md)
  - [NFR-wtdig-navigation-controller-performance](../requirements/NFR-wtdig-navigation-controller-performance.md)
  - [FR-erpze-guided-mode-navigation](../requirements/FR-erpze-guided-mode-navigation.md)
  - [FR-jm7mj-auto-mode-mission-execution](../requirements/FR-jm7mj-auto-mode-mission-execution.md)
- Related ADRs:
  - [ADR-w9zpl-control-mode-architecture](ADR-w9zpl-control-mode-architecture.md)
  - [ADR-xqqbl-gps-state-management](ADR-xqqbl-gps-state-management.md)
  - [ADR-2l5fh-differential-drive-kinematics](ADR-2l5fh-differential-drive-kinematics.md)
- Related Tasks:
  - [T-tto4f-navigation-controller](../tasks/T-tto4f-navigation-controller/README.md)
  - [T-u8q60-circle-mode-implementation](../tasks/T-u8q60-circle-mode-implementation/README.md)
  - [T-bo6xc-rtl-smartrtl-implementation](../tasks/T-bo6xc-rtl-smartrtl-implementation/README.md)
  - [T-w8x3p-fix-guided-mode-heading-oscillation](../tasks/T-w8x3p-fix-guided-mode-heading-oscillation/README.md)
  - [T-f4r7a-break-spin-feedback-loop](../tasks/T-f4r7a-break-spin-feedback-loop/README.md)

## Context

The autopilot requires a navigation controller to convert position targets into steering and throttle commands. This controller is the foundation for all autonomous modes (Guided, Auto, RTL).

### Problem

We need an architecture that:

- Calculates steering command from heading error (bearing to target vs current heading)
- Calculates throttle command with approach slowdown near target
- Detects arrival at target (within WP_RADIUS)
- Handles GPS update rates from 1Hz to 10Hz
- Works within the Mode trait architecture (ADR-w9zpl)
- Extensible for future L1 controller upgrade

### Constraints

- **Compute Budget**: Must complete navigation calculation within 2ms (NFR-wtdig)
- **Memory Budget**: < 500 bytes for controller state
- **no_std**: Must work without standard library
- **Platform**: RP2350 with hardware FPU, RP2040 with software float
- **Input Rate**: GPS updates at 1-10Hz, controller may run at 50Hz

### Prior Art

**ArduPilot L1 Controller**:

- Uses lateral acceleration demand for path following
- NAVL1_PERIOD (default 20s for Rover) determines responsiveness
- Cross-track error correction for path following
- Overly complex for initial implementation

**Simple Bearing-Based Navigation**:

- Direct heading error to steering conversion
- No path following, point-to-point navigation only
- Sufficient for initial autonomous modes
- Used in many hobby autopilots

## Success Metrics

- **Accuracy**: Vehicle arrives within WP_RADIUS (2m default) of target
- **Smoothness**: No steering oscillation during approach
- **Performance**: Navigation calculation < 2ms on RP2350
- **Extensibility**: Can upgrade to L1 without changing interface

## Decision

**We will implement a NavigationController trait with an initial SimpleNavigationController (L1-lite) that uses bearing-based steering with distance-based throttle control.**

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Mode (Guided/Auto)                       │
│                      update(dt) called at 50Hz               │
└─────────────────────────┬───────────────────────────────────┘
                          │
                          │ NavigationController::update()
                          ▼
┌─────────────────────────────────────────────────────────────┐
│              NavigationController (trait)                    │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  fn update(&mut self, current: &GpsPosition,         │   │
│  │            target: &PositionTarget, dt: f32)         │   │
│  │            -> NavigationOutput                       │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────┬───────────────────────────────────┘
                          │
              ┌───────────┴───────────┐
              ▼                       ▼
┌─────────────────────────┐ ┌─────────────────────────┐
│ SimpleNavigationController│ │ L1NavigationController  │
│ (Phase 1: L1-lite)       │ │ (Phase 2: Future)       │
│                          │ │                         │
│ - Bearing to target      │ │ - Cross-track error     │
│ - Heading error steering │ │ - L1 lateral accel      │
│ - Distance throttle      │ │ - Path following        │
└─────────────────────────┘ └─────────────────────────┘
```

### Decision Drivers

1. **Simplicity**: L1-lite provides basic navigation without complex path following
2. **Extensibility**: Trait allows L1 upgrade without changing callers
3. **Testability**: Pure function from inputs to outputs, easy to unit test
4. **Mode Integration**: Fits cleanly into Mode::update() flow
5. **ArduPilot Compatibility**: Similar parameter names (WP_RADIUS, WP_SPEED)

### Considered Options

- **Option A: Simple Bearing Navigation (L1-lite)** - Selected
- **Option B: Full L1 Controller**
- **Option C: Pure Pursuit Controller**

### Option Analysis

**Option A: Simple Bearing Navigation (L1-lite)**

- **Pros**:
  - Simple to implement and understand
  - Minimal computational overhead
  - Sufficient for point-to-point navigation
  - Easy to tune (fewer parameters)
  - Can be upgraded to L1 later
- **Cons**:
  - No cross-track error correction
  - May oscillate on approach if not tuned
  - Not optimal for path following
- **Complexity**: Low (\~200 lines)

**Option B: Full L1 Controller**

- **Pros**:
  - Industry standard for path following
  - Handles cross-track error
  - Smooth approaches
  - Proven in ArduPilot
- **Cons**:
  - More complex to implement
  - More parameters to tune
  - Overkill for initial use case
  - Requires path definition (not just point)
- **Complexity**: High (\~500+ lines)

**Option C: Pure Pursuit Controller**

- **Pros**:
  - Simple path following algorithm
  - Well-documented
  - Good for curved paths
- **Cons**:
  - Requires lookahead distance tuning
  - Not commonly used in ArduPilot ecosystem
  - Less documentation for Rover application
- **Complexity**: Medium (\~300 lines)

## Rationale

**Simple Bearing Navigation (L1-lite)** was selected because:

1. **Sufficient for Phase 1**: Point-to-point navigation meets initial Guided/Auto requirements
2. **Low Risk**: Simple algorithm, easy to debug and tune
3. **Upgrade Path**: Trait interface allows L1 upgrade without API changes
4. **Time to Value**: Can be implemented quickly to enable autonomous modes

### Trade-offs Accepted

- **No Path Following**: Initial implementation navigates point-to-point only
- **Potential Oscillation**: May need deadband or filtering for noisy GPS

## Consequences

### Positive

- **Simple Implementation**: \~200 lines of well-tested code
- **Quick Delivery**: Enables Guided mode in days, not weeks
- **Easy Tuning**: Only 4 parameters (WP_RADIUS, WP_SPEED, APPROACH_DIST, MAX_HEADING_ERR)
- **Testable**: Pure navigation logic with deterministic outputs

### Negative

- **No Cross-Track Correction**: Vehicle may deviate from straight-line path
- **Point-to-Point Only**: Cannot follow curved paths (future L1 will address)

### Neutral

- **Future Upgrade**: L1 implementation will replace SimpleNavigationController
- **Parameter Compatibility**: Same WP_RADIUS, WP_SPEED as ArduPilot

## Implementation Notes

### NavigationController Trait

```rust
/// Navigation controller trait
///
/// Calculates steering and throttle commands to navigate to target position.
pub trait NavigationController {
    /// Update navigation and calculate commands
    ///
    /// # Arguments
    /// * `current` - Current GPS position with heading
    /// * `target` - Target position to navigate to
    /// * `dt` - Delta time since last update (seconds)
    ///
    /// # Returns
    /// Navigation output with steering, throttle, and status
    fn update(
        &mut self,
        current: &GpsPosition,
        target: &PositionTarget,
        dt: f32,
    ) -> NavigationOutput;

    /// Reset controller state
    fn reset(&mut self);
}

/// Navigation output from controller
pub struct NavigationOutput {
    /// Steering command: -1.0 (left) to +1.0 (right)
    pub steering: f32,
    /// Throttle command: 0.0 to 1.0
    pub throttle: f32,
    /// Distance to target in meters
    pub distance_m: f32,
    /// Bearing to target in degrees (0-360)
    pub bearing_deg: f32,
    /// Heading error in degrees (-180 to +180)
    pub heading_error_deg: f32,
    /// True if within WP_RADIUS of target
    pub at_target: bool,
}
```

### SimpleNavigationController

```rust
/// Simple bearing-based navigation controller (L1-lite)
pub struct SimpleNavigationController {
    /// Waypoint acceptance radius (meters)
    wp_radius: f32,
    /// Maximum navigation speed (m/s) - used for throttle scaling
    wp_speed: f32,
    /// Distance to start slowing down (meters)
    approach_dist: f32,
    /// Heading error for full steering deflection (degrees)
    max_heading_error: f32,
}

impl SimpleNavigationController {
    pub fn new() -> Self {
        Self {
            wp_radius: 2.0,       // WP_RADIUS default
            wp_speed: 2.0,        // WP_SPEED default
            approach_dist: 10.0,  // Start slowing 10m from target
            max_heading_error: 90.0, // Full steer at 90° error
        }
    }
}

impl NavigationController for SimpleNavigationController {
    fn update(
        &mut self,
        current: &GpsPosition,
        target: &PositionTarget,
        _dt: f32,
    ) -> NavigationOutput {
        // 1. Calculate bearing and distance to target
        let bearing_deg = calculate_bearing(
            current.latitude_deg,
            current.longitude_deg,
            target.latitude_deg,
            target.longitude_deg,
        );

        let distance_m = calculate_distance(
            current.latitude_deg,
            current.longitude_deg,
            target.latitude_deg,
            target.longitude_deg,
        );

        // 2. Calculate heading error
        let heading_error_deg = wrap_180(bearing_deg - current.heading_deg);

        // 3. Check if at target
        let at_target = distance_m < self.wp_radius;

        // 4. Calculate steering (proportional to heading error)
        let steering = if at_target {
            0.0
        } else {
            (heading_error_deg / self.max_heading_error).clamp(-1.0, 1.0)
        };

        // 5. Calculate throttle (reduce near target)
        let throttle = if at_target {
            0.0
        } else if distance_m < self.approach_dist {
            // Linear slowdown in approach zone
            (distance_m / self.approach_dist).clamp(0.2, 1.0)
        } else {
            1.0
        };

        NavigationOutput {
            steering,
            throttle,
            distance_m,
            bearing_deg,
            heading_error_deg,
            at_target,
        }
    }

    fn reset(&mut self) {
        // No state to reset in simple controller
    }
}
```

### Helper Functions

```rust
use libm::{atan2f, cosf, sinf, sqrtf, asinf};

/// Earth radius in meters
const EARTH_RADIUS_M: f32 = 6_371_000.0;

/// Calculate bearing from point 1 to point 2
///
/// Returns bearing in degrees (0-360)
fn calculate_bearing(lat1_deg: f32, lon1_deg: f32, lat2_deg: f32, lon2_deg: f32) -> f32 {
    let lat1 = lat1_deg.to_radians();
    let lat2 = lat2_deg.to_radians();
    let dlon = (lon2_deg - lon1_deg).to_radians();

    let x = sinf(dlon) * cosf(lat2);
    let y = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlon);

    let bearing_rad = atan2f(x, y);
    let bearing_deg = bearing_rad.to_degrees();

    // Normalize to 0-360
    if bearing_deg < 0.0 {
        bearing_deg + 360.0
    } else {
        bearing_deg
    }
}

/// Calculate distance between two points using Haversine formula
///
/// Returns distance in meters
fn calculate_distance(lat1_deg: f32, lon1_deg: f32, lat2_deg: f32, lon2_deg: f32) -> f32 {
    let lat1 = lat1_deg.to_radians();
    let lat2 = lat2_deg.to_radians();
    let dlat = (lat2_deg - lat1_deg).to_radians();
    let dlon = (lon2_deg - lon1_deg).to_radians();

    let a = sinf(dlat / 2.0).powi(2)
        + cosf(lat1) * cosf(lat2) * sinf(dlon / 2.0).powi(2);
    let c = 2.0 * asinf(sqrtf(a));

    EARTH_RADIUS_M * c
}

/// Wrap angle to -180 to +180 degrees
fn wrap_180(angle_deg: f32) -> f32 {
    let mut result = angle_deg;
    while result > 180.0 {
        result -= 360.0;
    }
    while result < -180.0 {
        result += 360.0;
    }
    result
}
```

### Mode Integration

```rust
// In src/rover/mode/guided.rs
pub struct GuidedMode {
    nav_controller: Box<dyn NavigationController>,
    target: Option<PositionTarget>,
    gps_state: &'static GpsNavigationState,
    actuators: &'static mut Actuators,
}

impl Mode for GuidedMode {
    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        let Some(target) = &self.target else {
            // No target set, hold position (zero outputs)
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        };

        // Get current position from GPS state
        let current = self.gps_state.get_position()
            .ok_or("No GPS fix")?;

        // Calculate navigation commands
        let nav_output = self.nav_controller.update(&current, target, dt);

        // Apply commands to actuators
        self.actuators.set_steering(nav_output.steering)?;
        self.actuators.set_throttle(nav_output.throttle)?;

        // Check if arrived at target
        if nav_output.at_target {
            crate::log_info!("Arrived at waypoint");
            self.target = None; // Clear target, await next command
        }

        Ok(())
    }

    // ... enter(), exit(), name() implementations
}
```

### Module Structure

```
src/
└── subsystems/
    └── navigation/
        ├── mod.rs              # Module exports
        ├── controller.rs       # NavigationController trait + SimpleNavigationController
        ├── geo.rs              # Bearing, distance, angle calculations
        └── types.rs            # PositionTarget, NavigationOutput
```

### Configuration Parameters

| Parameter       | Default | Unit    | ArduPilot Equivalent | Description                  |
| --------------- | ------- | ------- | -------------------- | ---------------------------- |
| WP_RADIUS       | 2.0     | m       | WP_RADIUS            | Target acceptance radius     |
| WP_SPEED        | 2.0     | m/s     | WP_SPEED             | Maximum navigation speed     |
| APPROACH_DIST   | 10.0    | m       | (internal)           | Distance to start slowing    |
| MAX_HEADING_ERR | 90.0    | degrees | (internal)           | Heading error for full steer |

## Platform Considerations

### RP2350 (Pico 2 W)

- Hardware FPU handles all float operations
- Navigation calculation expected < 1ms
- Use `libm` for transcendental functions

### RP2040 (Pico W)

- Software floating-point
- Navigation calculation may take 5-10ms
- Consider lookup tables for trig functions if needed
- Still acceptable at 10Hz GPS update rate

### Cross-Platform

- Use `libm` crate for sin, cos, atan2, sqrt, asin
- Pure Rust implementation, no platform-specific code
- Unit tests run on host (x86/ARM)

## Monitoring & Logging

- **Navigation State**: Log bearing, distance, heading_error at 1Hz
- **Target Changes**: Log when new target set or target reached
- **Performance**: Monitor calculation time (should be < 2ms)
- **Telemetry**: Report nav state via MAVLink NAV_CONTROLLER_OUTPUT

## Open Questions

- [x] Should controller handle GPS timeout internally? → Decision: No, mode handles GPS state checking
- [ ] Need heading filter for noisy GPS COG? → Method: Test with real GPS data, add if needed
- [ ] Should minimum throttle be configurable? → Decision: Defer, use 0.2 minimum during approach

## External References

- [Haversine Formula](https://en.wikipedia.org/wiki/Haversine_formula) - Distance calculation
- [Initial Bearing](https://www.movable-type.co.uk/scripts/latlong.html) - Bearing calculation
- [ArduPilot WP_RADIUS](https://ardupilot.org/rover/docs/parameters.html#wp-radius) - Waypoint radius parameter
- [ArduPilot WP_SPEED](https://ardupilot.org/rover/docs/parameters.html#wp-speed) - Waypoint speed parameter

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
