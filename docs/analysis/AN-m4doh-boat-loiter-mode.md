# AN-m4doh Boat Loiter Mode

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-lgfjh-loiter-mode](AN-lgfjh-loiter-mode.md) (Rover Loiter Mode)
  - [AN-7ix56-navigation-approach](AN-7ix56-navigation-approach.md)
  - [AN-27568-position-target-navigation](AN-27568-position-target-navigation.md)
- Related Requirements:
  - None yet
- Related ADRs:
  - None yet (pending: Vehicle Type Separation ADR)
- Related Tasks:
  - None yet

## Executive Summary

This analysis examines the implementation of Loiter mode for boats (watercraft), which enables station keeping at a fixed point despite water currents and wind. Unlike ground rovers where stopping often suffices, boats require **continuous active correction** to maintain position.

**Scope:** This analysis covers **Boat implementation only**. Rover Loiter mode is covered separately in [AN-lgfjh-loiter-mode](AN-lgfjh-loiter-mode.md).

**Vehicle Type Separation:** Rover and Boat implementations are separated at compile-time using feature flags (`#[cfg(feature = "boat")]`). This approach provides optimized behavior per vehicle type - boats focus on continuous correction while rovers can rely on simpler stop-based approaches.

**Key Difference from Rover:** Boats cannot "stop and stay" - water currents and wind continuously push the vessel. Therefore, Type 0 (simple stop) is effectively useless for boats, and the implementation focuses entirely on Type 1 (active position hold) with aggressive correction algorithms.

## Problem Space

### Current State

**Mode System:**

- FlightMode enum exists with basic modes
- Manual, Hold modes implemented
- Loiter mode NOT implemented for boats
- Vehicle type separation architecture planned (pending ADR)

**Navigation System (In Progress):**

- L1 navigation controller analysis complete (AN-7ix56)
- Position target navigation defined (AN-27568)
- Navigation infrastructure being implemented
- GPS position telemetry functional

**ArduPilot Boat Loiter Reference:**

- Uses same LOIT\_\* parameters as Rover
- More aggressive correction behavior
- Supports forward/reverse movement for correction
- Station keeping is essential, not optional

### Desired State

A Boat Loiter mode that:

- Records current position (or projected stop point) on mode entry
- Provides **continuous active position correction**
- Handles water current and wind drift effectively
- Supports forward/reverse movement for optimal correction
- Operates within LOIT_RADIUS tolerance
- Integrates with existing navigation controller

### Gap Analysis

| Component               | Current State   | Desired State                  | Gap                 |
| ----------------------- | --------------- | ------------------------------ | ------------------- |
| Boat Loiter mode        | Not implemented | Continuous station keeping     | Mode implementation |
| Active correction       | Not implemented | Aggressive position correction | Algorithm           |
| Forward/reverse support | Not implemented | Bidirectional movement         | Control integration |
| Current/wind handling   | N/A             | Implicit via correction loop   | Tuning              |
| Loiter parameters       | Not defined     | LOIT_RADIUS (boat defaults)    | Parameter storage   |

## Stakeholder Analysis

| Stakeholder      | Interest/Need                        | Impact | Priority |
| ---------------- | ------------------------------------ | ------ | -------- |
| Boat Operators   | Reliable station keeping in currents | High   | P0       |
| Survey Users     | Hold position during data collection | High   | P0       |
| Fishing/Research | Maintain location without anchor     | High   | P0       |
| GCS Users        | Standard ArduPilot mode interface    | Medium | P1       |
| Safety Systems   | Position hold as failsafe fallback   | Medium | P1       |

## Research & Discovery

### User Feedback

N/A - Derived from ArduPilot standard functionality and feature backlog (FB-002).

### Competitive Analysis

**ArduPilot Boat Loiter Mode:**

From official documentation:

- When entering Loiter mode, the system projects a reasonable stopping point based on current velocity and maximum deceleration
- Within LOIT_RADIUS: drift without active control (allows minor movement)
- Outside LOIT_RADIUS: active correction begins
  - Rotate to face toward or away from target (shortest path)
  - Move at speed proportional to distance: `0.5 m/s × distance_to_edge`
  - Never exceed WP_SPEED
- Continuous loop until mode exit

**Key ArduPilot Parameters (Boat Context):**

| Parameter     | Type  | Range      | Rover Default | Boat Recommended | Description                   |
| ------------- | ----- | ---------- | ------------- | ---------------- | ----------------------------- |
| `LOIT_RADIUS` | Float | 0.5 - 100m | 2m            | 3-5m             | Larger for boat (GPS + drift) |
| `LOIT_TYPE`   | Int   | 0 or 1     | 0             | 1 (implicit)     | Type 1 always for boats       |
| `WP_SPEED`    | Float | 0 - 10m/s  | 2m/s          | 1-2m/s           | Max correction speed          |

**Note on LOIT_TYPE for Boats:**

While the parameter exists, Type 0 (stop) is effectively useless for boats since they will drift immediately. The boat implementation may:

1. Ignore LOIT_TYPE and always use Type 1 behavior
2. Or map Type 0 to "minimal correction" rather than "no correction"

### Technical Investigation

#### Boat Loiter State Machine

```
                 ┌─────────────┐
                 │ Mode Entry  │
                 └──────┬──────┘
                        │
                        ▼
              ┌─────────────────┐
              │ Calculate Stop  │
              │ Point (project) │
              └────────┬────────┘
                       │
                       ▼
              ┌─────────────────┐
              │ Active Station  │◄────────┐
              │    Keeping      │         │
              └────────┬────────┘         │
                       │                  │
         ┌─────────────┴─────────────┐    │
         │                           │    │
         ▼                           ▼    │
  ┌───────────────┐          ┌───────────────┐
  │ Within Radius │          │Outside Radius │
  │   (Allow      │          │  (Correct)    │
  │    Drift)     │          └───────┬───────┘
  └───────────────┘                  │
         │                           │
         └───────────────────────────┘
                  (loop)
```

#### Correction Algorithm (Boat-Optimized)

```rust
fn calculate_boat_correction(
    current_pos: &GpsPosition,
    loiter_point: &GpsPosition,
    loit_radius: f32,
    wp_speed: f32,
    current_heading: f32,
) -> (f32, f32) {  // (steering, throttle)
    let distance = distance_m(current_pos, loiter_point);

    if distance <= loit_radius {
        // Within acceptable radius - allow drift, minimal power
        return (0.0, 0.0);
    }

    // Outside radius - active correction
    let distance_to_edge = distance - loit_radius;

    // Speed proportional to distance (ArduPilot formula)
    // 0.5 m/s per meter outside radius, capped at WP_SPEED
    let target_speed = (0.5 * distance_to_edge).min(wp_speed);

    let bearing = bearing_deg(current_pos, loiter_point);

    // Choose shortest rotation path (forward or reverse)
    let heading_error_fwd = wrap_180(bearing - current_heading);
    let heading_error_rev = wrap_180(bearing + 180.0 - current_heading);

    let (steering_input, direction) = if heading_error_fwd.abs() <= heading_error_rev.abs() {
        // Forward is shorter rotation
        let steer = (heading_error_fwd / 45.0).clamp(-1.0, 1.0);
        (steer, 1.0)
    } else {
        // Reverse is shorter rotation
        let steer = (heading_error_rev / 45.0).clamp(-1.0, 1.0);
        (steer, -1.0)
    };

    // Throttle with direction
    let throttle = (target_speed / wp_speed) * direction;

    (steering_input, throttle)
}
```

#### Hysteresis for Stable Operation

```rust
const HYSTERESIS_ENTER: f32 = 1.0;   // Start correcting at radius
const HYSTERESIS_EXIT: f32 = 0.7;    // Stop correcting at 70% of radius

fn should_correct_boat(
    state: &mut BoatLoiterState,
    distance: f32,
) -> bool {
    if state.is_correcting {
        // Currently correcting - stop when well inside radius
        if distance < state.radius * HYSTERESIS_EXIT {
            state.is_correcting = false;
        }
    } else {
        // Not correcting - start when outside radius
        if distance > state.radius * HYSTERESIS_ENTER {
            state.is_correcting = true;
        }
    }
    state.is_correcting
}
```

#### Current/Wind Compensation (Implicit)

Boats handle current and wind **implicitly** through the correction loop:

1. Current pushes boat outside radius
2. Correction algorithm detects drift
3. Motors activate to return to position
4. Process repeats continuously

No explicit current sensing required - the GPS position feedback loop handles it.

### Data Analysis

**Boat Loiter Update Rates:**

| Component         | Rate   | Notes                      |
| ----------------- | ------ | -------------------------- |
| GPS update        | 1-10Hz | Position feedback          |
| Drift check       | 5-10Hz | Higher rate than rover     |
| Correction output | 50Hz   | Steering/throttle commands |
| Parameter check   | 1Hz    | Detect parameter changes   |

**Memory Requirements:**

| Component         | Size  | Notes                    |
| ----------------- | ----- | ------------------------ |
| Boat Loiter state | \~28B | Position, radius, flags  |
| Correction calc   | \~20B | Temporary calculation    |
| Total             | \~48B | Slightly more than rover |

**Power Consumption Considerations:**

Unlike rovers, boats in Loiter mode consume continuous power:

- Motors run frequently for correction
- Battery planning must account for active station keeping
- Consider low-power threshold for exiting Loiter

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: System shall implement Boat Loiter mode for continuous station keeping
  - Rationale: Boats cannot rely on passive stopping due to currents
  - Acceptance Criteria:
    - Mode activatable from GCS and RC
    - Records loiter point on mode entry
    - Provides continuous active position correction
    - Maintains position within LOIT_RADIUS under normal conditions

- [ ] **FR-DRAFT-2**: System shall calculate loiter point on mode entry considering velocity
  - Rationale: Boats have momentum and cannot stop instantly
  - Acceptance Criteria:
    - At low speed: use current position
    - At speed: project stop point based on deceleration
    - Loiter point stored for mode duration

- [ ] **FR-DRAFT-3**: System shall actively correct position when drift exceeds LOIT_RADIUS
  - Rationale: Essential behavior for boat station keeping
  - Acceptance Criteria:
    - Detect when distance to loiter point exceeds radius
    - Calculate optimal correction heading (forward or reverse)
    - Apply proportional speed based on drift distance
    - Stop correction when within hysteresis threshold

- [ ] **FR-DRAFT-4**: System shall support bidirectional movement for position correction
  - Rationale: Shortest path may require reverse movement
  - Acceptance Criteria:
    - Calculate heading error for both forward and reverse
    - Choose direction requiring less rotation
    - Apply correct throttle polarity for direction

- [ ] **FR-DRAFT-5**: System shall limit correction speed to WP_SPEED
  - Rationale: Safety and smooth operation
  - Acceptance Criteria:
    - Correction speed proportional to distance (0.5 m/s per meter)
    - Never exceed WP_SPEED parameter
    - Smooth throttle transitions

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Boat Loiter correction calculation shall complete within 1ms per update
  - Category: Performance
  - Rationale: Must not impact 50Hz control loop
  - Target: Measured on RP2350

- [ ] **NFR-DRAFT-2**: Position hold accuracy shall be within LOIT_RADIUS under moderate conditions
  - Category: Accuracy
  - Rationale: GPS accuracy + current effects limit precision
  - Target: Maintain position in currents up to 0.5 m/s

- [ ] **NFR-DRAFT-3**: Correction response time shall be less than 500ms from drift detection
  - Category: Responsiveness
  - Rationale: Prevent excessive drift before correction begins
  - Target: Detection to motor response

## Design Considerations

### Technical Constraints

**Existing Architecture:**

- Navigation controller handles path following
- GPS provides position updates at 1-10Hz
- Mode system provides enter/exit/update lifecycle
- Motor control supports forward/reverse

**Vehicle Type Separation (Compile-Time):**

```toml
[features]
rover = []  # Ground vehicle implementation
boat = []   # Watercraft implementation
```

**Boat-Specific Considerations:**

| Aspect               | Rover         | Boat                     |
| -------------------- | ------------- | ------------------------ |
| Type 0 behavior      | Useful (stop) | Useless (will drift)     |
| Type 1 necessity     | Optional      | Essential                |
| Correction frequency | Occasional    | Continuous               |
| Power consumption    | Low in Loiter | High in Loiter           |
| LOIT_RADIUS default  | 2m            | 3-5m (more tolerance)    |
| Reverse movement     | Optional      | Important for efficiency |

**Shared Components:**

- `src/subsystems/navigation/geo.rs`: Distance/bearing calculations
- `src/core/parameters/`: Parameter infrastructure
- `src/communication/mavlink/`: Protocol handling
- Mode trait interface (`enter`/`exit`/`update`)

### Potential Approaches

#### Approach A: Minimal Boat Adaptation

**Description:** Reuse most of Rover Loiter logic, just force Type 1 always.

**Pros:**

- Minimal code duplication
- Quick implementation

**Cons:**

- May not be optimized for boat behavior
- Shared code complicates maintenance
- Doesn't leverage separation architecture

**Effort:** Low

#### Approach B: Boat-Optimized Implementation (Recommended)

**Description:** Dedicated boat implementation with aggressive correction and bidirectional support.

**Pros:**

- Optimized for boat behavior
- Cleaner code without rover conditionals
- Can tune independently
- Follows separation architecture

**Cons:**

- Some code duplication with rover
- Two implementations to maintain

**Effort:** Medium

#### Approach C: Configurable Shared Core

**Description:** Shared LoiterCore with vehicle-specific configuration.

**Pros:**

- Reduced duplication
- Shared fixes benefit both

**Cons:**

- More complex architecture
- Harder to tune independently
- May lead to compromise behavior

**Effort:** Medium-High

### Architecture Impact

**New Components (Boat):**

- `src/boat/mode/loiter.rs`: Boat Loiter mode implementation
- `src/boat/mode/mod.rs`: Boat mode module (if not exists)

**Existing Rover Components (Separate):**

- `src/rover/mode/loiter.rs`: Rover Loiter mode (per AN-lgfjh)

**Modified Components:**

- `src/parameters/mod.rs`: Add LOIT\_\* parameters (shared)
- May need boat-specific defaults

**Feature Gate Structure:**

```rust
// src/boat/mode/loiter.rs
#[cfg(feature = "boat")]
pub struct BoatLoiter {
    loiter_point: GpsPosition,
    radius: f32,
    is_correcting: bool,
}

#[cfg(feature = "boat")]
impl Mode for BoatLoiter {
    fn update(&mut self, ctx: &ModeContext) -> ModeOutput {
        // Boat-specific active station keeping
    }
}
```

**Dependencies:**

- GPS position (required)
- Navigation controller (for correction commands)
- Heading/bearing calculations (existing)
- Motor control with reverse support

**ADR Required:**

- ADR for Vehicle Type Separation architecture (shared with Rover)

## Risk Assessment

| Risk                                         | Probability | Impact | Mitigation Strategy                     |
| -------------------------------------------- | ----------- | ------ | --------------------------------------- |
| GPS drift causes oscillation                 | Medium      | Medium | Hysteresis in correction start/stop     |
| Strong current exceeds motor capability      | Medium      | High   | Warn user, consider failsafe            |
| High power consumption drains battery        | High        | Medium | Battery monitoring, warn user           |
| Continuous motor use causes wear             | Low         | Low    | Use quality marine motors               |
| Code duplication leads to divergent behavior | Low         | Low    | Document shared algorithms, review both |

## Open Questions

- [x] Should Boat use same parameters as Rover? → Yes, LOIT\_\* parameters shared
- [x] Should Type 0 be supported for Boat? → No practical use, focus on Type 1
- [ ] What LOIT_RADIUS default is appropriate for boats? → Method: Testing, likely 3-5m
- [ ] Should there be a current strength warning? → Method: Monitor correction frequency
- [ ] What is maximum sustainable current for station keeping? → Method: Testing with actual boat

## Recommendations

### Immediate Actions

1. **Define shared Loiter parameters:**
   - LOIT_RADIUS, WP_SPEED (shared with Rover)
   - Consider boat-specific defaults in build config

2. **Implement Boat Loiter mode:**
   - Create `src/boat/mode/loiter.rs`
   - Focus on Type 1 (active correction) only
   - Support bidirectional movement

3. **Implement correction algorithm:**
   - Distance-based speed proportional control
   - Forward/reverse selection logic
   - Hysteresis for stable operation

### Next Steps

1. [ ] Create ADR: Vehicle Type Separation architecture (shared with Rover AN-lgfjh)
2. [ ] Create formal requirements: FR-<id>-boat-loiter-mode
3. [ ] Create task: T-<id>-boat-loiter-mode-implementation
4. [ ] Define test plan for station keeping accuracy
5. [ ] Determine appropriate LOIT_RADIUS default for boats

### Out of Scope

- **Rover Loiter mode**: Covered in AN-lgfjh-loiter-mode
- **Type 0 behavior**: Not useful for boats
- **Explicit current sensing**: Handled implicitly via correction loop
- **Anchor simulation**: Different use case
- **Geofence integration**: Separate system

## Appendix

### References

**ArduPilot Documentation:**

- [Loiter Mode (Rover/Boat)](https://ardupilot.org/rover/docs/loiter-mode.html)

**Related Project Documents:**

- [AN-lgfjh-loiter-mode](AN-lgfjh-loiter-mode.md): Rover Loiter Mode (companion analysis)
- [AN-7ix56-navigation-approach](AN-7ix56-navigation-approach.md): Navigation algorithm selection
- [AN-27568-position-target-navigation](AN-27568-position-target-navigation.md): Position target handling
- [FB-002 Feature Backlog](../feature-backlog.md): Original Loiter mode feature request

### Raw Data

**Boat Loiter State:**

```rust
#[cfg(feature = "boat")]
pub struct BoatLoiterState {
    /// Recorded loiter position
    loiter_point: GpsPosition,
    /// Acceptable drift radius in meters
    radius: f32,
    /// Maximum correction speed
    wp_speed: f32,
    /// True if currently correcting position
    is_correcting: bool,
    /// Current correction direction (1.0 = forward, -1.0 = reverse)
    correction_direction: f32,
}

#[cfg(feature = "boat")]
impl BoatLoiterState {
    pub fn new(position: &GpsPosition, radius: f32, wp_speed: f32) -> Self {
        Self {
            loiter_point: position.clone(),
            radius,
            wp_speed,
            is_correcting: false,
            correction_direction: 1.0,
        }
    }
}
```

**Speed Calculation:**

```rust
/// Calculate correction speed based on distance from loiter point
/// ArduPilot formula: 0.5 m/s per meter outside radius
fn calculate_correction_speed(distance_to_edge: f32, wp_speed: f32) -> f32 {
    const SPEED_PER_METER: f32 = 0.5;
    (distance_to_edge * SPEED_PER_METER).min(wp_speed)
}

// Example: LOIT_RADIUS = 3m, WP_SPEED = 2m/s
// At 5m from loiter point:
//   distance_to_edge = 5 - 3 = 2m
//   speed = 0.5 * 2 = 1.0 m/s
// At 10m from loiter point:
//   distance_to_edge = 10 - 3 = 7m
//   speed = 0.5 * 7 = 3.5 m/s → capped to 2.0 m/s
```

**Power Consumption Estimation:**

```
Assumptions:
- Average correction duty cycle: 50-80% (current dependent)
- Motor power: 20-50W per motor (depends on size)
- Dual motor: 40-100W during correction

For 1 hour of station keeping in moderate current:
- Energy: 40-80 Wh
- Battery consideration: Factor this into mission planning
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
