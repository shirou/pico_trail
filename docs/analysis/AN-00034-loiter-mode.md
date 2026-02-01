# AN-00034 Loiter Mode

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-00032-boat-loiter-mode](AN-00032-boat-loiter-mode.md) (Boat Loiter Mode)
  - [AN-00003-navigation-approach](AN-00003-navigation-approach.md)
  - [AN-00023-position-target-navigation](AN-00023-position-target-navigation.md)
- Related Requirements:
  - [FR-00118-rover-loiter-mode](../requirements/FR-00118-rover-loiter-mode.md)
  - [FR-00114-loiter-point-calculation](../requirements/FR-00114-loiter-point-calculation.md)
  - [FR-00115-loiter-type-parameter](../requirements/FR-00115-loiter-type-parameter.md)
  - [FR-00117-position-drift-detection](../requirements/FR-00117-position-drift-detection.md)
  - [FR-00116-position-correction-navigation](../requirements/FR-00116-position-correction-navigation.md)
  - [NFR-00081-loiter-drift-detection-performance](../requirements/NFR-00081-loiter-drift-detection-performance.md)
  - [NFR-00082-position-hold-accuracy](../requirements/NFR-00082-position-hold-accuracy.md)
- Related ADRs:
  - [ADR-00030-vehicle-type-separation](../adr/ADR-00030-vehicle-type-separation.md)
- Related Tasks:
  - [T-00029-rover-loiter-mode](../tasks/T-00029-rover-loiter-mode/README.md)

## Executive Summary

This analysis examines the implementation of Loiter mode for ground rovers, which enables position holding at a fixed point. Loiter mode is a standard ArduPilot Rover mode useful for temporary stops during operation and position hold while waiting for commands.

**Scope:** This analysis covers **Rover implementation only**. Boat Loiter mode will be implemented separately with its own analysis, as the behavioral requirements differ significantly (continuous station keeping vs. occasional correction).

**Vehicle Type Separation:** Rover and Boat implementations are separated at compile-time using feature flags. This approach provides cleaner code, optimized behavior per vehicle type, and easier maintenance. Other modes (Circle, RTL, etc.) will follow the same separation pattern.

For ground rovers, Loiter mode primarily means stopping and optionally correcting for position drift. Type 0 (simple stop) is often sufficient for rovers on flat terrain. Type 1 (active correction) is useful for slopes or windy conditions but is less critical than for boats.

## Problem Space

### Current State

**Mode System:**

- FlightMode enum exists with basic modes
- Manual, Hold modes implemented
- Hold mode provides simple motor stop without position correction
- Loiter mode NOT implemented

**Navigation System (In Progress):**

- L1 navigation controller analysis complete (AN-00003)
- Position target navigation defined (AN-00023)
- Navigation infrastructure being implemented
- GPS position telemetry functional

**ArduPilot Loiter Mode Reference:**

- Standard rover mode for position holding
- Two types: stop (Type 0) and active hold (Type 1)
- Uses LOIT_RADIUS to define acceptable drift

### Desired State

A Loiter mode that:

- Records current position as loiter point on mode entry
- Supports Type 0 (stop motors, no correction)
- Supports Type 1 (active position correction when drift exceeds threshold)
- Integrates with existing navigation controller
- Compatible with GCS mode selection

### Gap Analysis

| Component                | Current State    | Desired State               | Gap                  |
| ------------------------ | ---------------- | --------------------------- | -------------------- |
| Loiter mode              | Not implemented  | Functional position hold    | Mode implementation  |
| Position drift detection | Not implemented  | Detect drift from reference | Distance calculation |
| Return-to-point nav      | Partial (Guided) | Navigate back to loiter pt  | Integration          |
| Loiter parameters        | Not defined      | LOIT_RADIUS, LOIT_TYPE      | Parameter storage    |

## Stakeholder Analysis

| Stakeholder     | Interest/Need                         | Impact | Priority |
| --------------- | ------------------------------------- | ------ | -------- |
| Field Operators | Temporary stop during operation       | High   | P0       |
| Boat Users      | Station keeping in water currents     | High   | P0       |
| GCS Users       | Standard ArduPilot mode interface     | Medium | P1       |
| Failsafe System | Hold mode alternative with GPS backup | Medium | P1       |

## Research & Discovery

### User Feedback

N/A - Derived from ArduPilot standard functionality and feature backlog (FB-002).

### Competitive Analysis

**ArduPilot Loiter Mode (Rover):**

From official documentation:

- When entering Loiter mode, the system projects a reasonable stopping point based on current velocity and maximum deceleration
- Two behavioral types controlled by LOIT_TYPE:
  - Type 0 (Stop): Simply stop motors, no active position holding
  - Type 1 (Hold): Actively correct position if drifted beyond LOIT_RADIUS

**Key ArduPilot Parameters:**

| Parameter     | Type  | Range      | Default | Description                    |
| ------------- | ----- | ---------- | ------- | ------------------------------ |
| `LOIT_RADIUS` | Float | 0.5 - 100m | 2m      | Acceptable drift before action |
| `LOIT_TYPE`   | Int   | 0 or 1     | 0       | 0=stop, 1=active hold          |

**Boat Behavior (Active Hold):**

For boats, the Loiter mode is more active:

1. While within LOIT_RADIUS: drift without active control
2. When outside LOIT_RADIUS:
   - Rotate to face toward or away from target (whichever requires less rotation)
   - Move at speed proportional to distance: 0.5 m/s × distance to circle edge
   - Never exceed WP_SPEED
3. This approach handles current and wind drift effectively

**Ground Rover vs Boat:**

| Aspect              | Ground Rover            | Boat                  |
| ------------------- | ----------------------- | --------------------- |
| Primary drift cause | Slopes, wind            | Water current, wind   |
| Drift rate          | Low (usually stops)     | High (continuous)     |
| Correction need     | Occasional              | Continuous            |
| Type 0 usefulness   | High (often sufficient) | Low (will drift away) |
| Type 1 usefulness   | Moderate                | Essential             |

### Technical Investigation

#### Loiter Mode State Machine

```
                 ┌─────────────┐
                 │ Mode Entry  │
                 └──────┬──────┘
                        │
                        ▼
              ┌─────────────────┐
              │ Record Position │
              │ Calculate Stop  │
              │ Point           │
              └────────┬────────┘
                       │
         ┌─────────────┴─────────────┐
         │                           │
         ▼                           ▼
   ┌───────────┐             ┌───────────────┐
   │  Type 0   │             │    Type 1     │
   │   Stop    │             │  Active Hold  │
   └───────────┘             └───────┬───────┘
                                     │
                           ┌─────────┴─────────┐
                           │                   │
                           ▼                   ▼
                   ┌───────────────┐   ┌───────────────┐
                   │ Within Radius │   │Outside Radius │
                   │    (Drift)    │   │ (Correct)     │
                   └───────────────┘   └───────────────┘
```

#### Stop Point Calculation

When entering Loiter mode at speed, the system calculates a projected stop point:

```rust
fn calculate_stop_point(
    current_pos: &GpsPosition,
    velocity: Vector2D,
    max_decel: f32,
) -> GpsPosition {
    // Calculate stopping distance: v² / (2 * a)
    let speed = velocity.magnitude();
    let stop_distance = (speed * speed) / (2.0 * max_decel);

    // Project stop point in direction of travel
    let heading = velocity.heading();
    offset_position(current_pos, stop_distance, heading)
}
```

#### Position Correction Algorithm (Type 1)

```rust
fn calculate_correction(
    current_pos: &GpsPosition,
    loiter_point: &GpsPosition,
    loit_radius: f32,
    wp_speed: f32,
) -> (f32, f32) {  // (steering, throttle)
    let distance = distance_m(current_pos, loiter_point);

    if distance <= loit_radius {
        // Within acceptable radius - stop
        return (0.0, 0.0);
    }

    // Outside radius - navigate back
    let distance_to_edge = distance - loit_radius;
    let target_speed = (0.5 * distance_to_edge).min(wp_speed);

    let bearing = bearing_deg(current_pos, loiter_point);
    let current_heading = get_current_heading();

    // Choose shortest rotation path
    let heading_error = wrap_180(bearing - current_heading);
    let heading_error_reverse = wrap_180(bearing + 180.0 - current_heading);

    let (steering, direction) = if heading_error.abs() < heading_error_reverse.abs() {
        (heading_error.signum(), 1.0)  // Forward
    } else {
        (heading_error_reverse.signum(), -1.0)  // Reverse
    };

    (steering, target_speed * direction)
}
```

### Data Analysis

**Loiter Mode Update Rate:**

| Component         | Rate   | Notes                      |
| ----------------- | ------ | -------------------------- |
| GPS update        | 1-10Hz | Position feedback          |
| Drift check       | 1-5Hz  | Distance calculation       |
| Correction output | 50Hz   | Steering/throttle commands |
| Parameter check   | 1Hz    | Detect parameter changes   |

**Memory Requirements:**

| Component       | Size  | Notes                  |
| --------------- | ----- | ---------------------- |
| Loiter state    | \~24B | Position, type, radius |
| Correction calc | \~16B | Temporary calculation  |
| Total           | \~40B | Minimal overhead       |

## Discovered Requirements

### Functional Requirements

- [x] **FR-00118-rover-loiter-mode**: System shall implement Loiter mode for position holding
  - See: [FR-00118-rover-loiter-mode](../requirements/FR-00118-rover-loiter-mode.md)

- [x] **FR-00114-loiter-point-calculation**: System shall calculate loiter point on mode entry
  - See: [FR-00114-loiter-point-calculation](../requirements/FR-00114-loiter-point-calculation.md)

- [x] **FR-00115-loiter-type-parameter**: System shall support LOIT_TYPE parameter
  - See: [FR-00115-loiter-type-parameter](../requirements/FR-00115-loiter-type-parameter.md)

- [x] **FR-00117-position-drift-detection**: System shall detect position drift from loiter point
  - See: [FR-00117-position-drift-detection](../requirements/FR-00117-position-drift-detection.md)

- [x] **FR-00116-position-correction-navigation**: System shall navigate back to loiter point when drift exceeds threshold (Type 1)
  - See: [FR-00116-position-correction-navigation](../requirements/FR-00116-position-correction-navigation.md)

### Non-Functional Requirements

- [x] **NFR-00081-loiter-drift-detection-performance**: Loiter mode drift detection shall complete within 1ms per update
  - See: [NFR-00081-loiter-drift-detection-performance](../requirements/NFR-00081-loiter-drift-detection-performance.md)

- [x] **NFR-00082-position-hold-accuracy**: Position hold accuracy shall be within LOIT_RADIUS + GPS accuracy
  - See: [NFR-00082-position-hold-accuracy](../requirements/NFR-00082-position-hold-accuracy.md)

## Design Considerations

### Technical Constraints

**Existing Architecture:**

- Hold mode already provides simple motor stop
- Navigation controller handles path following
- GPS provides position updates at 1-10Hz
- Mode system provides enter/exit/update lifecycle

**Vehicle Type Separation (Compile-Time):**

This project separates Rover and Boat implementations at compile-time:

```toml
[features]
rover = []  # Ground vehicle implementation
boat = []   # Watercraft implementation
```

**Rationale for Separation:**

| Aspect                    | Single Implementation     | Separate Implementations  |
| ------------------------- | ------------------------- | ------------------------- |
| Code clarity              | Mixed logic, conditionals | Clean, focused code       |
| Default parameters        | Compromises               | Optimized per vehicle     |
| Type 0 relevance          | Must support both         | Rover: useful, Boat: skip |
| Correction aggressiveness | Single tuning             | Vehicle-appropriate       |
| Maintenance               | Complex                   | Independent evolution     |
| Testing                   | Many combinations         | Focused test cases        |

**Shared Components:**

Despite separation, some components remain shared:

- `src/subsystems/navigation/geo.rs`: Distance/bearing calculations
- `src/core/parameters/`: Parameter infrastructure
- `src/communication/mavlink/`: Protocol handling
- Mode trait interface (`enter`/`exit`/`update`)

**Difference from Hold Mode:**

| Aspect              | Hold Mode             | Loiter Mode        |
| ------------------- | --------------------- | ------------------ |
| Position recording  | No                    | Yes (loiter point) |
| Position correction | No                    | Yes (Type 1)       |
| GPS requirement     | No                    | Yes                |
| Use case            | Emergency/Simple stop | Station keeping    |

**ArduPilot Compatibility:**

- Use LOIT\_\* parameter prefix
- Match ArduPilot behavioral modes
- Support same GCS mode selection interface

### Potential Approaches

#### Approach A: Type 0 Only (Minimal)

**Description:** Implement only Type 0 (stop motors), same as Hold mode with position recording.

**Pros:**

- Trivial implementation
- No navigation dependency
- Works without GPS (degrades gracefully)

**Cons:**

- Not useful for boats
- Missing key ArduPilot functionality
- Limited value over Hold mode

**Effort:** Low

#### Approach B: Full Loiter with Both Types (Recommended)

**Description:** Implement both Type 0 and Type 1 with active position correction.

**Pros:**

- Full ArduPilot compatibility
- Useful for boats and rovers
- Foundation for RTL and other position-based modes
- Clear differentiation from Hold mode

**Cons:**

- Requires navigation controller integration
- More complex state management
- Requires GPS

**Effort:** Medium

#### Approach C: Phased Implementation

**Description:** Start with Type 0, add Type 1 when navigation controller is ready.

**Phase 1:** Type 0 only (simple stop with position recording)
**Phase 2:** Type 1 with active correction

**Pros:**

- Delivers basic functionality early
- Validates mode structure
- Type 1 added when navigation ready

**Cons:**

- Two implementation cycles
- Temporary incomplete functionality

**Effort:** Low + Medium

### Architecture Impact

**New Components (Rover):**

- `src/rover/mode/loiter.rs`: Rover Loiter mode implementation

**Future Components (Boat - Separate Analysis):**

- `src/boat/mode/loiter.rs`: Boat Loiter mode implementation (different analysis)

**Modified Components:**

- `src/rover/mode/mod.rs`: Add Loiter mode to rover mode enum
- `src/parameters/mod.rs`: Add LOIT\_\* parameters (shared)

**Feature Gate Structure:**

```rust
// src/rover/mode/loiter.rs
#[cfg(feature = "rover")]
pub struct RoverLoiter { ... }

// src/boat/mode/loiter.rs (future)
#[cfg(feature = "boat")]
pub struct BoatLoiter { ... }
```

**Dependencies:**

- GPS position (required)
- Navigation controller (for Type 1)
- Heading/bearing calculations (existing in navigation)

**ADR Required:**

- ADR for Vehicle Type Separation architecture (applies to all modes)

## Risk Assessment

| Risk                                         | Probability | Impact | Mitigation Strategy                    |
| -------------------------------------------- | ----------- | ------ | -------------------------------------- |
| GPS drift causes oscillation in Type 1       | Medium      | Low    | Hysteresis in LOIT_RADIUS check        |
| Loiter point calculation error at high speed | Low         | Low    | Unit tests for stop point projection   |
| Mode confusion with Hold mode                | Low         | Low    | Clear documentation and GCS labels     |
| Code duplication between Rover/Boat          | Low         | Low    | Extract shared logic to common modules |

## Open Questions

- [x] Should we implement both types initially? → Yes, Approach B recommended for Rover
- [x] Should Rover and Boat have separate implementations? → Yes, compile-time separation via feature flags
- [x] Will other modes follow the same separation pattern? → Yes, all modes will be separated
- [ ] What should happen if GPS fix is lost during Loiter? → Method: Degrade to Type 0 behavior
- [ ] Should reverse driving be supported for position correction? → ArduPilot supports this, recommend following standard
- [ ] What is the minimum LOIT_RADIUS for practical use? → Method: Test with hardware, likely 1-2m minimum

## Recommendations

### Immediate Actions

1. **Define Loiter mode parameters:**
   - Add LOIT_RADIUS, LOIT_TYPE to parameter system
   - Use ArduPilot defaults (2m, Type 0)

2. **Implement Loiter mode skeleton:**
   - Add Loiter variant to FlightMode enum (if not exists)
   - Implement Mode trait with enter/exit/update
   - Record loiter point on mode entry

3. **Implement Type 0 behavior:**
   - Stop motors on mode entry
   - No position correction
   - Simple and quick to validate

### Next Steps

1. [x] Create ADR: Vehicle Type Separation architecture (affects all modes)
   - Created: [ADR-00030-vehicle-type-separation](../adr/ADR-00030-vehicle-type-separation.md)
2. [x] Create formal requirements: FR-00118-rover-loiter-mode and related
   - Created: 5 FRs and 2 NFRs (see Links section)
3. [ ] Verify navigation controller readiness for Type 1
4. [x] Create task: T-00029-rover-loiter-mode-implementation
   - Created: [T-00029-rover-loiter-mode](../tasks/T-00029-rover-loiter-mode/README.md)
5. [ ] Define test plan for position holding accuracy
6. [ ] Create separate analysis: AN-<id>-boat-loiter-mode (when boat support prioritized)

### Out of Scope

- **Boat Loiter mode**: Separate analysis required (different behavioral requirements)
- **Geofence integration**: Handled by geofence system
- **Weather compensation**: Beyond basic current/wind handling
- **Dynamic loiter point adjustment**: Not standard ArduPilot behavior
- **Altitude holding**: N/A for rover/boat (surface vehicles)

## Appendix

### References

**ArduPilot Documentation:**

- [Loiter Mode (Rover)](https://ardupilot.org/rover/docs/loiter-mode.html)

**Related Project Documents:**

- [AN-00003-navigation-approach](AN-00003-navigation-approach.md): Navigation algorithm selection
- [AN-00023-position-target-navigation](AN-00023-position-target-navigation.md): Position target handling
- [FB-002 Feature Backlog](../feature-backlog.md): Original Loiter mode feature request

### Raw Data

**Distance to Loiter Point Calculation:**

```rust
fn distance_to_loiter(current: &GpsPosition, loiter: &GpsPosition) -> f32 {
    const EARTH_RADIUS_M: f32 = 6_371_000.0;
    let lat1 = current.latitude.to_radians();
    let lat2 = loiter.latitude.to_radians();
    let dlat = (loiter.latitude - current.latitude).to_radians();
    let dlon = (loiter.longitude - current.longitude).to_radians();

    let a = (dlat / 2.0).sin().powi(2)
        + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();
    EARTH_RADIUS_M * c
}
```

**Loiter Mode State:**

```rust
pub struct LoiterState {
    /// Recorded loiter position
    loiter_point: GpsPosition,
    /// Type of loiter behavior (0=stop, 1=active hold)
    loiter_type: u8,
    /// Acceptable drift radius in meters
    radius: f32,
    /// True if currently correcting position
    is_correcting: bool,
}

impl LoiterState {
    pub fn new(position: &GpsPosition, loiter_type: u8, radius: f32) -> Self {
        Self {
            loiter_point: position.clone(),
            loiter_type,
            radius,
            is_correcting: false,
        }
    }

    pub fn should_correct(&self, current: &GpsPosition) -> bool {
        if self.loiter_type == 0 {
            return false;  // Type 0 never corrects
        }
        distance_to_loiter(current, &self.loiter_point) > self.radius
    }
}
```

**Hysteresis for Correction Start/Stop:**

```rust
const HYSTERESIS_FACTOR: f32 = 0.8;

fn check_correction_needed(
    state: &mut LoiterState,
    current: &GpsPosition
) -> bool {
    let distance = distance_to_loiter(current, &state.loiter_point);

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

    state.is_correcting
}
```
