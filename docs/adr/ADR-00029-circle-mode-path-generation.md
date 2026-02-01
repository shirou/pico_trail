# ADR-00029 Circle Mode Path Generation Architecture

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-00112-circle-mode-implementation](../requirements/FR-00112-circle-mode-implementation.md)
  - [FR-00111-circle-center-point](../requirements/FR-00111-circle-center-point.md)
  - [FR-00113-circle-mode-parameters](../requirements/FR-00113-circle-mode-parameters.md)
  - [NFR-00080-circle-path-calculation-performance](../requirements/NFR-00080-circle-path-calculation-performance.md)
- Related Analyses:
  - [AN-00033-circle-mode](../analysis/AN-00033-circle-mode.md)
- Supersedes ADRs: N/A
- Related ADRs:
  - [ADR-00022-navigation-controller-architecture](ADR-00022-navigation-controller-architecture.md)
- Related Tasks:
  - [T-00028-circle-mode-implementation](../tasks/T-00028-circle-mode-implementation/README.md)

## Context

Circle mode requires the rover to orbit around a fixed center point at a configurable radius and speed. The key architectural decision is how to generate the circular path and integrate it with the existing navigation system.

**Problem Statement:**

- How should Circle mode generate steering/throttle commands?
- Should it bypass the navigation controller or integrate with it?
- What level of path smoothness and accuracy is required?

**Constraints:**

- Must integrate with existing L1 navigation controller
- Performance budget: 1ms for circle calculations on RP2350
- Memory budget: <64 bytes additional state
- Must work with GPS update rates of 1-10Hz
- ArduPilot parameter compatibility required

**Forces in Tension:**

- Simplicity vs. smooth path following
- Direct control vs. navigation controller reuse
- Discrete waypoints vs. continuous path generation
- Calculation overhead vs. path accuracy

## Success Metrics (optional)

- Path following error < 2m RMS at normal speeds
- Circle calculation overhead < 1ms on RP2350
- No visible jerky motion during circular orbit
- Seamless integration with existing navigation infrastructure

## Decision

We will implement Circle mode using the **Hybrid Approach**: a continuous circle generator that feeds look-ahead target points to the existing L1 navigation controller.

### Decision Drivers

- Reuse of existing L1 controller for proven path following
- Smooth continuous motion without waypoint transitions
- Minimal additional code (leverages navigation infrastructure)
- Good performance characteristics (simple calculations)
- Natural integration with navigation controller architecture

### Considered Options

- Option A: Discrete Waypoint Chain
- Option B: Direct Angular Velocity Control
- Option C: Continuous Circle Generator with L1 Integration (Hybrid)

### Option Analysis

**Option A: Discrete Waypoint Chain**

Generate a sequence of waypoints around the circle perimeter and use existing waypoint navigation.

- Pros: Completely reuses existing infrastructure, easy to implement
- Cons: Fixed waypoint spacing causes jerky motion, higher memory for waypoint storage, difficult to handle continuous motion

**Option B: Direct Angular Velocity Control**

Control heading rate directly based on desired angular velocity, bypassing navigation controller.

- Pros: Most direct circle control, simplest geometry
- Cons: Bypasses navigation controller (duplicate code), no path correction for drift, harder to integrate

**Option C: Continuous Circle Generator with L1 Integration (Hybrid)** ✓ Selected

Calculate a look-ahead target point on the circle perimeter continuously and feed it to the L1 navigation controller.

- Pros: Smooth continuous motion, minimal memory, reuses L1 controller, natural drift correction
- Cons: Slightly more complex than direct waypoint approach

## Rationale

The hybrid approach (Option C) was selected because:

1. **Leverages Existing Infrastructure**: The L1 navigation controller already handles path following, steering calculation, and actuator output. Circle mode only needs to provide target points.

2. **Smooth Motion**: By continuously calculating a look-ahead point on the circle, the vehicle follows a smooth arc rather than transitioning between discrete waypoints.

3. **Natural Drift Correction**: The L1 controller automatically corrects for GPS drift and external disturbances, keeping the vehicle on the circular path.

4. **Minimal Code Addition**: Circle mode becomes a relatively simple target point generator (\~100 lines), not a complete navigation system.

5. **Performance**: Look-ahead calculation is a single geodetic offset operation, well within the 1ms budget.

Option A was rejected because discrete waypoints cause visible jerks at each transition and require storing multiple waypoints in memory.

Option B was rejected because it duplicates navigation logic and loses the benefits of L1 path correction.

## Consequences

### Positive

- Clean separation of concerns: Circle mode generates targets, L1 controller follows them
- Consistent navigation behavior across modes (Guided, Auto, Circle all use L1)
- Easy to test: Circle mode can be unit tested independently of navigation
- Future-proof: If we upgrade to S-curve navigation, Circle mode still works

### Negative

- Slight indirection: Circle mode doesn't directly control steering
- Look-ahead tuning: May need to adjust look-ahead distance for optimal path following
- Dependency on L1 controller: Circle mode quality depends on L1 tuning

### Neutral

- Same navigation controller parameters (WP_SPEED, L1_PERIOD) affect Circle mode behavior

## Implementation Notes

**Circle Target Point Calculation:**

```rust
impl CircleMode {
    pub fn calculate_target(&self, current: &GpsPosition) -> GpsPosition {
        // Calculate current angle from center to vehicle
        let current_angle = bearing_deg(&self.center, current);

        // Angular velocity = speed / radius (rad/s)
        let angular_velocity = self.speed / self.radius;
        let angular_velocity_deg = angular_velocity.to_degrees();

        // Look-ahead: typically 1.0-2.0 seconds ahead
        let look_ahead_time = 1.5;
        let look_ahead_angle = angular_velocity_deg * look_ahead_time;

        // Target angle on circle
        let target_angle = match self.direction {
            CircleDirection::Clockwise => current_angle + look_ahead_angle,
            CircleDirection::CounterClockwise => current_angle - look_ahead_angle,
        };

        // Return point on circle perimeter
        offset_position(&self.center, self.radius, target_angle)
    }
}
```

**Integration with Navigation Controller:**

```rust
// In Circle mode update loop
async fn update(&mut self, ctx: &mut ModeContext) {
    let current_pos = ctx.navigation_state.position();

    if self.radius == 0.0 {
        // Stationary mode: stop
        ctx.set_output(0.0, 0.0);
        return;
    }

    // Calculate next target point on circle
    let target = self.calculate_target(&current_pos);

    // Delegate to L1 navigation controller
    let (steering, throttle) = ctx.navigation.navigate_to(&current_pos, &target);

    ctx.set_output(steering, throttle);
}
```

**Mode Entry:**

```rust
fn enter(&mut self, ctx: &ModeContext) -> Result<(), ModeError> {
    // Validate GPS fix
    if !ctx.navigation_state.has_3d_fix() {
        return Err(ModeError::NoGpsFix);
    }

    // Get current position and heading
    let current = ctx.navigation_state.position();
    let heading = ctx.ahrs.heading_deg()
        .or_else(|| ctx.navigation_state.course_over_ground())
        .ok_or(ModeError::NoHeading)?;

    // Load parameters
    let radius = ctx.params.get_f32("CIRC_RADIUS").unwrap_or(20.0);
    let speed = ctx.params.get_f32("CIRC_SPEED").unwrap_or(2.0);
    let dir = ctx.params.get_i8("CIRC_DIR").unwrap_or(0);

    // Calculate center point (ArduPilot: CIRC_RADIUS ahead of vehicle)
    self.center = offset_position(&current, radius, heading);
    self.radius = radius;
    self.speed = speed;
    self.direction = if dir == 0 {
        CircleDirection::Clockwise
    } else {
        CircleDirection::CounterClockwise
    };

    ctx.send_statustext("Circle mode entered");
    Ok(())
}
```

## Open Questions

- [ ] Optimal look-ahead time for different speeds? → Method: Empirical testing on hardware
- [ ] Should Circle mode support dynamic center update via MAVLink? → Deferred to future enhancement

## External References

- [ArduPilot Circle Mode (Rover)](https://ardupilot.org/rover/docs/circle-mode.html)
- [L1 Navigation Controller](https://ardupilot.org/plane/docs/navigation-tuning.html)
- [Movable Type Scripts - Geodetic Calculations](https://www.movable-type.co.uk/scripts/latlong.html)
