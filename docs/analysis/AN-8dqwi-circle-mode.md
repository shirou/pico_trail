# AN-8dqwi Circle Mode

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-7ix56-navigation-approach](AN-7ix56-navigation-approach.md)
  - [AN-27568-position-target-navigation](AN-27568-position-target-navigation.md)
- Related Requirements:
  - [FR-khjpl-circle-mode-implementation](../requirements/FR-khjpl-circle-mode-implementation.md)
  - [FR-m2noq-circle-center-point](../requirements/FR-m2noq-circle-center-point.md)
  - [FR-7d203-circle-mode-parameters](../requirements/FR-7d203-circle-mode-parameters.md)
  - [NFR-p8ylh-circle-path-calculation-performance](../requirements/NFR-p8ylh-circle-path-calculation-performance.md)
- Related ADRs:
  - [ADR-897ov-circle-mode-path-generation](../adr/ADR-897ov-circle-mode-path-generation.md)
- Related Tasks:
  - [T-u8q60-circle-mode-implementation](../tasks/T-u8q60-circle-mode-implementation/README.md)

## Executive Summary

This analysis examines the implementation of Circle mode for the rover, which enables orbiting around a fixed center point at a configurable radius and speed. Circle mode is a standard ArduPilot Rover mode useful for survey/inspection operations, perimeter patrol, and area coverage patterns.

The implementation requires GPS position, heading control, and integration with the existing navigation controller. Key decisions include center point determination (current position vs. offset) and integration with the L1/S-curve navigation system. This analysis recommends implementing Circle mode using the existing navigation infrastructure with a dedicated circular path generator.

## Problem Space

### Current State

**Navigation System (In Progress):**

- L1 navigation controller analysis complete (AN-7ix56)
- Position target navigation defined (AN-27568)
- Basic navigation infrastructure being implemented
- GPS position telemetry functional

**Mode System:**

- FlightMode enum exists with basic modes
- Manual, Hold modes implemented
- Auto, Guided modes in development
- Circle mode NOT implemented

**ArduPilot Circle Mode Reference:**

- Standard rover mode for circular orbits
- Uses CIRC_RADIUS, CIRC_SPEED, CIRC_DIR parameters
- Center point determined on mode entry

### Desired State

A Circle mode that:

- Orbits a fixed center point at configurable radius
- Supports clockwise and counterclockwise rotation
- Operates at configurable speed
- Integrates with existing navigation controller
- Compatible with GCS mode selection

### Gap Analysis

| Component              | Current State   | Desired State               | Gap                 |
| ---------------------- | --------------- | --------------------------- | ------------------- |
| Circle mode            | Enum only       | Functional circular orbit   | Mode implementation |
| Circular path planning | Not implemented | Generate circular waypoints | Algorithm           |
| Circle parameters      | Not defined     | CIRC_RADIUS/SPEED/DIR       | Parameter storage   |
| Center point handling  | N/A             | Fixed center management     | State management    |
| Navigation integration | L1/S-curve WIP  | Circle uses nav controller  | Integration layer   |

## Stakeholder Analysis

| Stakeholder      | Interest/Need                        | Impact | Priority |
| ---------------- | ------------------------------------ | ------ | -------- |
| Survey Operators | Consistent circular coverage pattern | High   | P0       |
| Patrol Users     | Perimeter orbit around fixed point   | High   | P0       |
| GCS Users        | Standard ArduPilot mode interface    | Medium | P1       |
| Developers       | Clean integration with navigation    | Medium | P1       |

## Research & Discovery

### User Feedback

N/A - Derived from ArduPilot standard functionality and feature backlog (FB-001).

### Competitive Analysis

**ArduPilot Circle Mode (Rover):**

From official documentation:

- Center point is placed CIRC_RADIUS meters **in front of** the vehicle when entering the mode
- Three parameters control behavior:
  - `CIRC_RADIUS`: Orbital distance in meters
  - `CIRC_SPEED`: Vehicle velocity in m/s
  - `CIRC_DIR`: Direction (0=clockwise, 1=counterclockwise)

**Key ArduPilot Behaviors:**

- Target point does NOT decelerate if vehicle cannot keep pace
- High radius values may force maximum throttle operation
- Parameter changes require mode exit/re-entry to take effect
- Speed changes during missions possible via DO_CHANGE_SPEED command

**Boat Variant:**

- Uses same parameters
- More affected by current/wind drift
- May require active station-keeping corrections

### Technical Investigation

#### Circle Path Generation Algorithm

**Option A: Continuous Arc Following**

Generate steering commands to follow a circular arc directly:

```
angular_velocity = linear_speed / radius
heading_rate = angular_velocity (rad/s)
```

**Option B: Discrete Waypoint Generation**

Generate waypoints along the circle perimeter and follow using existing navigation:

```
for angle in 0..360 step 10:
    wp.lat = center.lat + radius * cos(angle)
    wp.lon = center.lon + radius * sin(angle)
```

**Option C: Hybrid Approach (Recommended)**

Use continuous circular path calculation with existing L1 controller for smooth following:

```rust
fn circle_target(&self, current_pos: GpsPosition) -> GpsPosition {
    // Calculate angle from center to current position
    let current_angle = bearing(center, current_pos);
    // Look ahead on circle perimeter
    let target_angle = current_angle + look_ahead_angle * direction;
    // Calculate target point on circle
    let target = offset_position(center, radius, target_angle);
    target
}
```

#### Parameter Mapping

ArduPilot uses `CIRC_*` prefix for Circle mode parameters:

| Parameter     | Type  | Range       | Default | Description             |
| ------------- | ----- | ----------- | ------- | ----------------------- |
| `CIRC_RADIUS` | Float | 0.0 - 1000m | 20m     | Circle radius in meters |
| `CIRC_SPEED`  | Float | 0.0 - 10m/s | 2m/s    | Target speed            |
| `CIRC_DIR`    | Int   | 0 or 1      | 0       | 0=CW, 1=CCW             |

**Note:** CIRC_RADIUS=0 causes the rover to remain stationary rather than orbit.

#### Center Point Determination

**ArduPilot Behavior:**

When Circle mode is entered, the center point is placed CIRC_RADIUS meters in front of the current vehicle position (in the direction of current heading).

```rust
fn calculate_center_point(current: &GpsPosition, heading: f32, radius: f32) -> GpsPosition {
    // Offset center point in front of vehicle
    offset_position(current, radius, heading)
}
```

**Alternative: Current Position as Center**

Some use cases prefer the vehicle's current position as the center:

```rust
fn enter_circle_mode(current: &GpsPosition) -> CircleState {
    CircleState {
        center: current.clone(),
        // Vehicle must first move to circle perimeter
        phase: CirclePhase::ApproachingPerimeter,
    }
}
```

### Data Analysis

**Circle Navigation Update Rate:**

| Component          | Rate   | Notes                      |
| ------------------ | ------ | -------------------------- |
| GPS update         | 1-10Hz | Position feedback          |
| Circle path update | 10Hz   | Target point calculation   |
| Navigation output  | 50Hz   | Steering/throttle commands |
| Parameter check    | 1Hz    | Detect parameter changes   |

**Memory Requirements:**

| Component       | Size  | Notes                       |
| --------------- | ----- | --------------------------- |
| Circle state    | \~32B | Center, radius, direction   |
| Look-ahead calc | \~16B | Target position calculation |
| Total           | \~48B | Minimal overhead            |

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: System shall implement Circle mode for circular orbit navigation
  - Rationale: Standard ArduPilot mode for survey and patrol operations
  - Acceptance Criteria:
    - Vehicle orbits at configured radius
    - Speed maintained at CIRC_SPEED
    - Direction follows CIRC_DIR (CW/CCW)
    - Mode activatable from GCS and RC

- [ ] **FR-DRAFT-2**: System shall determine circle center point on mode entry
  - Rationale: ArduPilot standard behavior places center in front of vehicle
  - Acceptance Criteria:
    - Center calculated as CIRC_RADIUS meters ahead on mode entry
    - Center position stored and maintained during mode
    - Center position queryable by GCS (telemetry)

- [ ] **FR-DRAFT-3**: System shall support Circle mode parameters (CIRC_RADIUS, CIRC_SPEED, CIRC_DIR)
  - Rationale: ArduPilot parameter compatibility
  - Acceptance Criteria:
    - Parameters configurable via MAVLink PARAM_SET
    - Parameters persist across reboots
    - Changes require mode re-entry to take effect

- [ ] **FR-DRAFT-4**: System shall handle CIRC_RADIUS=0 as stationary mode
  - Rationale: ArduPilot behavior for zero radius
  - Acceptance Criteria:
    - Vehicle stops when CIRC_RADIUS=0
    - No circular motion attempted
    - Center point still recorded

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Circle path calculation shall complete within 1ms per update
  - Category: Performance
  - Rationale: Must not impact navigation update rate
  - Target: Measured on RP2350

- [ ] **NFR-DRAFT-2**: Circle path following error shall be less than 2m RMS
  - Category: Accuracy
  - Rationale: Acceptable error for survey/patrol operations
  - Target: Measured under normal GPS conditions

## Design Considerations

### Technical Constraints

**Existing Architecture:**

- Navigation controller (L1 or S-curve) handles path following
- Circle mode generates target points for navigation controller
- Mode system provides enter/exit/update lifecycle
- GPS provides position updates at 1-10Hz

**ArduPilot Compatibility:**

- Use CIRC\_\* parameter prefix
- Match ArduPilot center point calculation
- Support same GCS mode selection interface

### Potential Approaches

#### Approach A: Discrete Waypoint Chain

**Description:** Generate a sequence of waypoints around the circle and use existing waypoint navigation.

**Pros:**

- Reuses existing waypoint navigation completely
- Simple implementation
- Easy to visualize/debug

**Cons:**

- Fixed waypoint spacing may cause jerky motion
- Higher memory for waypoint storage
- Need to regenerate on parameter change

**Effort:** Low

#### Approach B: Continuous Circle Generator (Recommended)

**Description:** Calculate look-ahead target point on circle perimeter continuously.

**Pros:**

- Smooth continuous motion
- Minimal memory usage
- Dynamic adjustment to position errors
- Natural integration with L1 controller

**Cons:**

- Slightly more complex math
- Requires heading/bearing calculations

**Effort:** Medium

#### Approach C: Angular Velocity Control

**Description:** Control heading rate directly based on desired angular velocity.

**Pros:**

- Most direct circle control
- Simplest geometry

**Cons:**

- Bypasses navigation controller
- Harder to integrate with existing architecture
- No path correction for drift

**Effort:** Medium

### Architecture Impact

**New Components:**

- `src/rover/mode/circle.rs`: Circle mode implementation
- `src/subsystems/navigation/circle.rs`: Circle path generator (optional, may inline)

**Modified Components:**

- `src/rover/mode/mod.rs`: Add Circle mode to mode enum
- `src/core/parameters/mod.rs`: Add CIRC\_\* parameters

**No New ADRs Required:**

- Circle mode follows established navigation patterns from ADR-wrcuk
- Uses existing L1 controller architecture

## Risk Assessment

| Risk                                         | Probability | Impact | Mitigation Strategy                          |
| -------------------------------------------- | ----------- | ------ | -------------------------------------------- |
| GPS drift causes erratic circle path         | Medium      | Medium | Use heading-based corrections, filter inputs |
| High speed + small radius causes instability | Low         | Medium | Limit min radius based on speed              |
| Center point calculation error               | Low         | Low    | Unit tests for offset calculation            |
| Parameter change during mode causes issues   | Low         | Low    | Ignore changes until mode re-entry           |

## Open Questions

- [x] Should center be in front of vehicle or at current position? → ArduPilot standard: in front by CIRC_RADIUS
- [ ] Should we support dynamic center point adjustment via MAVLink? → Defer to future enhancement
- [ ] What is the minimum safe radius for different vehicle speeds? → Method: Empirical testing with actual hardware

## Recommendations

### Immediate Actions

1. **Define Circle mode parameters:**
   - Add CIRC_RADIUS, CIRC_SPEED, CIRC_DIR to parameter system
   - Use ArduPilot defaults

2. **Implement Circle mode skeleton:**
   - Add Circle variant to FlightMode enum
   - Implement Mode trait with enter/exit/update

3. **Integrate with navigation controller:**
   - Circle mode generates continuous target points
   - L1 controller follows target points

### Next Steps

1. [ ] Create formal requirements from FR-DRAFT items
2. [ ] Create task: T-<id>-circle-mode-implementation
3. [ ] Verify navigation controller is ready for integration
4. [ ] Define test plan for circle accuracy verification

### Out of Scope

- **Mission command LOITER_TURNS**: Handled separately in Auto mode
- **Dynamic center adjustment**: Not in initial implementation
- **Circle with altitude changes**: N/A for rover
- **Spiral patterns**: Future enhancement, not initial scope
- **GCS circle drawing**: GCS functionality, not vehicle firmware

## Appendix

### References

**ArduPilot Documentation:**

- [Circle Mode (Rover)](https://ardupilot.org/rover/docs/circle-mode.html)

**Related Project Documents:**

- [AN-7ix56-navigation-approach](AN-7ix56-navigation-approach.md): Navigation algorithm selection
- [AN-27568-position-target-navigation](AN-27568-position-target-navigation.md): Position target handling
- [FB-001 Feature Backlog](../feature-backlog.md): Original Circle mode feature request

### Raw Data

**Circle Target Point Calculation:**

```rust
/// Calculate next target point on circle perimeter
fn calculate_circle_target(
    center: &GpsPosition,
    current: &GpsPosition,
    radius: f32,
    speed: f32,
    direction: CircleDirection,
    dt: f32,
) -> GpsPosition {
    // Current angle from center
    let current_angle = bearing_deg(center, current);

    // Angular velocity = linear speed / radius
    let angular_velocity = speed / radius; // rad/s
    let angular_velocity_deg = angular_velocity.to_degrees(); // deg/s

    // Look-ahead angle (typically 1-2 seconds ahead)
    let look_ahead_time = 1.5; // seconds
    let look_ahead_angle = angular_velocity_deg * look_ahead_time;

    // Target angle on circle
    let target_angle = match direction {
        CircleDirection::Clockwise => current_angle + look_ahead_angle,
        CircleDirection::CounterClockwise => current_angle - look_ahead_angle,
    };

    // Calculate target point on circle perimeter
    offset_position(center, radius, target_angle)
}
```

**Center Point Calculation on Mode Entry:**

```rust
/// Calculate circle center point (ArduPilot standard: in front of vehicle)
fn calculate_center_on_entry(current: &GpsPosition, heading: f32, radius: f32) -> GpsPosition {
    // Center is CIRC_RADIUS meters in front of current position
    offset_position(current, radius, heading)
}

/// Offset a position by distance and bearing
fn offset_position(origin: &GpsPosition, distance: f32, bearing_deg: f32) -> GpsPosition {
    const EARTH_RADIUS: f32 = 6_371_000.0;

    let bearing_rad = bearing_deg.to_radians();
    let lat1 = origin.latitude.to_radians();
    let lon1 = origin.longitude.to_radians();
    let angular_distance = distance / EARTH_RADIUS;

    let lat2 = (lat1.sin() * angular_distance.cos()
        + lat1.cos() * angular_distance.sin() * bearing_rad.cos())
        .asin();

    let lon2 = lon1
        + (bearing_rad.sin() * angular_distance.sin() * lat1.cos())
            .atan2(angular_distance.cos() - lat1.sin() * lat2.sin());

    GpsPosition {
        latitude: lat2.to_degrees(),
        longitude: lon2.to_degrees(),
        ..*origin
    }
}
```

**Speed and Radius Relationship:**

```
Given:
- CIRC_RADIUS = 20m
- CIRC_SPEED = 2m/s

Circle circumference = 2 * PI * radius = 125.7m
Time per orbit = circumference / speed = 62.8 seconds
Angular velocity = 2 * PI / time = 0.1 rad/s = 5.73 deg/s
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
