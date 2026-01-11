# AN-vcxr7 AHRS-Navigation-Control Integration for Autonomous Navigation

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-27568-position-target-navigation](AN-27568-position-target-navigation.md)
  - [AN-7ix56-navigation-approach](AN-7ix56-navigation-approach.md)
  - [AN-srhcj-bno086-imu-integration](AN-srhcj-bno086-imu-integration.md)
  - [AN-t47be-mpu9250-imu-and-ekf-integration](AN-t47be-mpu9250-imu-and-ekf-integration.md)
- Related Requirements:
  - [FR-jm7mj-auto-mode-mission-execution](../requirements/FR-jm7mj-auto-mode-mission-execution.md)
  - [FR-erpze-guided-mode-navigation](../requirements/FR-erpze-guided-mode-navigation.md)
  - [FR-2vbe8-navigation-controller](../requirements/FR-2vbe8-navigation-controller.md)
  - [FR-eyuh8-ahrs-attitude-estimation](../requirements/FR-eyuh8-ahrs-attitude-estimation.md)
- Related ADRs:
  - [ADR-nzvfy-ahrs-abstraction-architecture](../adr/ADR-nzvfy-ahrs-abstraction-architecture.md)
  - [ADR-wrcuk-navigation-controller-architecture](../adr/ADR-wrcuk-navigation-controller-architecture.md)
- Related Tasks:
  - [T-7khm3-ahrs-abstraction-layer](../tasks/T-7khm3-ahrs-abstraction-layer/README.md) (Complete)
  - [T-tto4f-navigation-controller](../tasks/T-tto4f-navigation-controller/README.md) (Complete)
  - [T-x8mq2-bno086-driver-implementation](../tasks/T-x8mq2-bno086-driver-implementation/README.md) (Complete)

## Executive Summary

This analysis examines the integration between the AHRS (Attitude and Heading Reference System) subsystem and the Navigation/Control subsystems to enable autonomous navigation in Auto and Guided modes. With the BNO086 IMU driver (T-x8mq2) and AHRS abstraction layer (T-7khm3) now complete, the next step is to connect AHRS heading output to the navigation controller and implement the autonomous control modes.

**Key Findings:**

1. **Heading Source Gap**: The NavigationController currently uses GPS Course-Over-Ground (COG) exclusively. At low speeds or when stationary, COG is unreliable or unavailable, requiring AHRS heading as a fallback.

2. **Mode Implementation Gap**: Guided and Auto modes have defined requirements but no implementation tasks or ADRs exist.

3. **Data Flow Gap**: No mechanism exists to pass AHRS data (heading, angular rate) from SharedAhrsState to the navigation controller.

**Recommendations:**

- Implement heading fusion (GPS COG + AHRS yaw) in the navigation controller
- Create ADR for Guided/Auto mode architecture
- Create implementation tasks for the full integration chain

## Problem Space

### Current State

**Completed Components:**

| Component            | Status   | Location                                  |
| -------------------- | -------- | ----------------------------------------- |
| BNO086 Driver        | Complete | `src/devices/imu/bno086/`                 |
| AHRS Abstraction     | Complete | `src/subsystems/ahrs/traits.rs`           |
| SharedAhrsState      | Complete | `src/subsystems/ahrs/traits.rs`           |
| Bno086ExternalAhrs   | Complete | `src/subsystems/ahrs/external/bno086.rs`  |
| NavigationController | Complete | `src/subsystems/navigation/controller.rs` |
| Manual Mode          | Complete | `src/rover/mode/manual.rs`                |
| RTL Mode             | Complete | `src/rover/mode/rtl.rs`                   |
| Mode Trait           | Complete | `src/rover/mode/mod.rs`                   |

**Current Heading Source in NavigationController:**

```rust
// src/subsystems/navigation/controller.rs:128-130
let current_heading = current.course_over_ground.unwrap_or(0.0);
```

The controller uses `GpsPosition.course_over_ground` exclusively. When COG is unavailable (vehicle stationary or slow), it defaults to 0.0 and reduces throttle.

**Current AHRS Data Flow:**

```
BNO086 Driver → Bno086ExternalAhrs → [No Consumer]
                                           ↓
                               SharedAhrsState (unused)
```

### Desired State

**Target Architecture:**

```
                           ┌─────────────────────┐
                           │   Auto/Guided Mode  │
                           └──────────┬──────────┘
                                      │ steering/throttle commands
                                      ▼
┌───────────────┐         ┌─────────────────────┐
│ SharedAhrsState │───────►│ NavigationController │
│  (yaw, rates)   │        │   (heading fusion)   │
└───────────────┘         └──────────┬──────────┘
                                      │
                          ┌───────────┴───────────┐
                          ▼                       ▼
                ┌─────────────────┐     ┌─────────────────┐
                │  GpsPosition    │     │  PositionTarget │
                │ (lat, lon, COG) │     │ (from mission)  │
                └─────────────────┘     └─────────────────┘
```

**Goals:**

1. Navigation controller uses fused heading (GPS COG when moving, AHRS yaw when stationary)
2. Guided mode accepts SET_POSITION_TARGET_GLOBAL_INT and navigates to target
3. Auto mode executes uploaded waypoint mission
4. Angular rate from AHRS available for future attitude stabilization (D-term in PID)

### Gap Analysis

| Component               | Current State    | Desired State                   | Gap                  |
| ----------------------- | ---------------- | ------------------------------- | -------------------- |
| Heading Source          | GPS COG only     | GPS COG + AHRS fusion           | New heading provider |
| AHRS → Navigation       | Not connected    | SharedAhrsState → NavController | Integration layer    |
| Guided Mode             | Requirement only | Functional implementation       | ADR + Task needed    |
| Auto Mode               | Requirement only | Functional implementation       | ADR + Task needed    |
| Position Target Handler | Not implemented  | SET_POSITION_TARGET_GLOBAL_INT  | Handler needed       |
| BNO086 Task Spawn       | Not implemented  | Task spawned in main            | Main app integration |

## Stakeholder Analysis

| Stakeholder       | Interest/Need                     | Impact | Priority |
| ----------------- | --------------------------------- | ------ | -------- |
| Navigation System | Accurate heading at all speeds    | High   | P0       |
| Control Loops     | Stable attitude data for steering | High   | P0       |
| GCS Operators     | Auto/Guided mode functionality    | High   | P0       |
| Mission Planner   | Waypoint mission execution        | High   | P0       |
| Failsafe System   | Reliable heading for RTL/Hold     | Medium | P1       |

## Research & Discovery

### User Feedback

N/A - Derived from project requirements and ArduPilot standards.

### Competitive Analysis

**ArduPilot Heading Source Selection:**

ArduPilot uses a priority-based heading source selection:

1. **GPS COG** - Primary when speed > 1 m/s (configurable via `GPS_NAVFILTER`)
2. **Compass/AHRS Yaw** - Used when stationary or slow
3. **Dead Reckoning** - Extrapolates from last known heading using gyro

```cpp
// ArduPilot simplified logic (Rover)
if (gps.ground_speed() > GPS_NAVFILTER_SPEED) {
    heading = gps.ground_course();
} else {
    heading = ahrs.yaw_sensor;
}
```

**Key Parameters:**

- `GPS_NAVFILTER`: Speed threshold for GPS heading (default 1.0 m/s)
- `COMPASS_USE`: Enable compass for heading (1 = enabled)

### Technical Investigation

**Heading Fusion Strategy Options:**

#### Option 1: Simple Speed-Based Switch

```rust
fn get_heading(gps: &GpsPosition, ahrs: &AhrsState) -> f32 {
    const SPEED_THRESHOLD: f32 = 1.0; // m/s

    if gps.speed >= SPEED_THRESHOLD && gps.course_over_ground.is_some() {
        gps.course_over_ground.unwrap()
    } else {
        ahrs.yaw.to_degrees()
    }
}
```

- Pros: Simple, matches ArduPilot behavior
- Cons: Abrupt transition at threshold

#### Option 2: Blended Heading

```rust
fn get_heading(gps: &GpsPosition, ahrs: &AhrsState) -> f32 {
    const BLEND_MIN_SPEED: f32 = 0.5;
    const BLEND_MAX_SPEED: f32 = 2.0;

    let gps_weight = match gps.speed {
        s if s < BLEND_MIN_SPEED => 0.0,
        s if s > BLEND_MAX_SPEED => 1.0,
        s => (s - BLEND_MIN_SPEED) / (BLEND_MAX_SPEED - BLEND_MIN_SPEED),
    };

    let gps_heading = gps.course_over_ground.unwrap_or(ahrs.yaw.to_degrees());
    let ahrs_heading = ahrs.yaw.to_degrees();

    wrap_360(gps_weight * gps_heading + (1.0 - gps_weight) * ahrs_heading)
}
```

- Pros: Smooth transition between sources
- Cons: More complex, potential oscillation at boundary

#### Option 3: Kalman Filter Fusion

- Pros: Optimal fusion with uncertainty estimation
- Cons: High complexity, overkill for initial implementation
- Recommendation: Defer to future SoftwareAhrs implementation

**Recommendation:** Start with Option 1 (simple switch), migrate to Option 2 if jitter observed.

### Data Analysis

**Update Rate Budget:**

| Source                 | Update Rate | Latency   |
| ---------------------- | ----------- | --------- |
| BNO086 Rotation Vector | 100 Hz      | \~10 ms   |
| GPS Position           | 1-10 Hz     | 50-200 ms |
| Navigation Controller  | 10-50 Hz    | N/A       |
| Mode Update            | 50 Hz       | N/A       |

AHRS updates faster than GPS, so heading fusion can use latest AHRS data with interpolated GPS position if needed.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: Navigation controller shall use AHRS heading when GPS COG is unavailable or unreliable
  - Rationale: GPS COG requires movement; stationary vehicles need AHRS heading
  - Acceptance Criteria: Heading valid within 5 seconds of power-on; smooth transition at speed threshold

- [ ] **FR-DRAFT-2**: System shall provide heading source indicator in telemetry
  - Rationale: Operators need to know which heading source is active for debugging
  - Acceptance Criteria: AHRS_STATUS or STATUSTEXT indicates heading source

- [ ] **FR-DRAFT-3**: Guided mode shall navigate to position targets from SET_POSITION_TARGET_GLOBAL_INT
  - Rationale: Required for GCS "Fly To Here" functionality
  - Acceptance Criteria: Vehicle navigates to target and stops within WP_RADIUS
  - Note: Covered by FR-erpze, needs implementation task

- [ ] **FR-DRAFT-4**: Auto mode shall execute uploaded waypoint missions sequentially
  - Rationale: Core autonomous operation capability
  - Acceptance Criteria: Vehicle visits all waypoints in order and transitions to Hold on completion
  - Note: Covered by FR-jm7mj, needs implementation task

- [ ] **FR-DRAFT-5**: BNO086 AHRS task shall be spawned in main application
  - Rationale: AHRS data must be available to navigation system
  - Acceptance Criteria: SharedAhrsState updated at 100 Hz

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Heading transition between GPS COG and AHRS shall not cause steering jitter > 5 degrees
  - Category: Stability
  - Rationale: Abrupt heading changes cause erratic vehicle behavior
  - Target: Measured during speed transitions

- [ ] **NFR-DRAFT-2**: AHRS heading shall be available within 1 second of BNO086 initialization
  - Category: Responsiveness
  - Rationale: Fast startup for operational readiness
  - Target: First valid heading < 1s after boot

## Design Considerations

### Technical Constraints

**Existing Architecture:**

- Embassy async runtime with cooperative multitasking
- SharedAhrsState uses critical sections for thread-safety
- NavigationController is synchronous, called from mode update
- Mode trait requires `update(dt: f32)` at 50 Hz

**Memory Constraints:**

- Heading fusion logic: \~50 bytes
- Heading provider callback: \~8 bytes (function pointer)
- Total overhead: < 100 bytes

**Coordination:**

- AHRS task runs independently at 100 Hz
- Navigation reads SharedAhrsState synchronously in update loop
- No direct await/async in Mode::update (synchronous interface)

### Potential Approaches

#### Approach A: Heading Provider Function

Inject heading provider function into NavigationController.

```rust
pub struct SimpleNavigationController {
    config: SimpleNavConfig,
    heading_provider: fn() -> f32,  // NEW: returns heading in degrees
}

impl SimpleNavigationController {
    pub fn with_heading_provider(mut self, provider: fn() -> f32) -> Self {
        self.heading_provider = provider;
        self
    }
}
```

**Mode Usage:**

```rust
fn get_fused_heading() -> f32 {
    let ahrs = AHRS_STATE.read();
    let gps = get_gps_position();

    if gps.speed > 1.0 && gps.course_over_ground.is_some() {
        gps.course_over_ground.unwrap()
    } else {
        ahrs.yaw.to_degrees()
    }
}

let nav_controller = SimpleNavigationController::new()
    .with_heading_provider(get_fused_heading);
```

- Pros: Non-breaking change, flexible injection
- Cons: Function pointer requires static lifetime

#### Approach B: Direct SharedAhrsState Reference

Navigation controller directly reads SharedAhrsState.

```rust
pub struct SimpleNavigationController {
    config: SimpleNavConfig,
    ahrs_state: &'static SharedAhrsState,
}

impl SimpleNavigationController {
    fn get_heading(&self, gps: &GpsPosition) -> f32 {
        if gps.speed > 1.0 && gps.course_over_ground.is_some() {
            gps.course_over_ground.unwrap()
        } else {
            self.ahrs_state.get_yaw().to_degrees()
        }
    }
}
```

- Pros: Direct access, no function indirection
- Cons: Tight coupling to SharedAhrsState, harder to test

#### Approach C: Unified HeadingSource Trait (Recommended)

Define a trait for heading providers, allowing multiple implementations.

```rust
pub trait HeadingSource {
    fn get_heading(&self) -> Option<f32>;
    fn is_valid(&self) -> bool;
}

pub struct FusedHeadingSource {
    ahrs: &'static SharedAhrsState,
    gps_provider: fn() -> Option<GpsPosition>,
    speed_threshold: f32,
}

impl HeadingSource for FusedHeadingSource {
    fn get_heading(&self) -> Option<f32> {
        let ahrs = self.ahrs.read();
        if let Some(gps) = (self.gps_provider)() {
            if gps.speed > self.speed_threshold && gps.course_over_ground.is_some() {
                return gps.course_over_ground;
            }
        }
        if ahrs.healthy {
            Some(ahrs.yaw.to_degrees())
        } else {
            None
        }
    }

    fn is_valid(&self) -> bool {
        self.ahrs.is_healthy() || (self.gps_provider)().is_some()
    }
}
```

- Pros: Clean abstraction, testable, extensible
- Cons: Additional trait complexity
- Recommendation: Best balance of flexibility and clarity

### Architecture Impact

**New Components:**

- `src/subsystems/navigation/heading.rs` - HeadingSource trait and FusedHeadingSource
- `src/rover/mode/guided.rs` - Guided mode implementation
- `src/rover/mode/auto.rs` - Auto mode implementation
- `src/communication/mavlink/handlers/position_target.rs` - SET_POSITION_TARGET handler

**Modified Components:**

- `src/subsystems/navigation/controller.rs` - Accept HeadingSource
- `src/rover/mode/mod.rs` - Export Guided/Auto modes
- `examples/pico_trail_rover.rs` (or main) - Spawn BNO086 task, wire up integration

**ADRs Required:**

- ADR for heading source selection strategy
- ADR for Guided/Auto mode architecture (may extend ADR-w9zpl)

## Risk Assessment

| Risk                                        | Probability | Impact | Mitigation Strategy                                      |
| ------------------------------------------- | ----------- | ------ | -------------------------------------------------------- |
| Heading jitter at speed threshold           | Medium      | Medium | Use blended transition (Option 2) if issue observed      |
| AHRS not healthy at mode entry              | Low         | High   | Require AHRS health as mode entry condition              |
| GPS and AHRS heading disagree significantly | Low         | Medium | Log discrepancy, prefer GPS when moving                  |
| BNO086 initialization delay                 | Low         | Medium | Show AHRS status in GCS, block Auto/Guided until healthy |
| Coordinate frame mismatch (NED vs ENU)      | Medium      | High   | Verify BNO086 outputs NED yaw; add conversion if needed  |

## Open Questions

- [x] Which heading fusion strategy to use? → Recommendation: Option 1 (simple switch) initially, upgrade to Option 2 if jitter observed
- [ ] Should mode entry require AHRS healthy status? → Method: Review ArduPilot pre-arm checks
- [ ] Is BNO086 yaw in NED frame (0° = North)? → Method: Verify in T-7khm3 implementation or test on hardware
- [ ] What is the appropriate GPS speed threshold for heading switch? → Next step: Start with 1.0 m/s (ArduPilot default), tune in testing

## Recommendations

### Immediate Actions

1. **Create ADR for heading source selection** - Document decision on fusion strategy
2. **Create ADR for Guided/Auto mode architecture** - Extend control mode framework
3. **Create formal requirements** - Convert FR-DRAFT items to formal requirements

### Next Steps

1. [ ] Create formal requirements from FR-DRAFT items
2. [ ] Draft ADR: Heading source selection and fusion strategy
3. [ ] Draft ADR: Guided and Auto mode architecture
4. [ ] Create task: T-xxxxx-heading-fusion-integration
   - Phase 1: HeadingSource trait and FusedHeadingSource
   - Phase 2: NavigationController integration
   - Phase 3: BNO086 task spawn in main application
5. [ ] Create task: T-xxxxx-guided-mode-implementation
   - SET_POSITION_TARGET_GLOBAL_INT handler
   - Guided mode implementation
6. [ ] Create task: T-xxxxx-auto-mode-implementation
   - Mission item iteration
   - Auto mode implementation

### Out of Scope

- **Attitude stabilization (PID control)**: Angular rate data is available but not used for stabilization in initial implementation
- **Full L1/S-Curve navigation**: Use existing SimpleNavigationController initially
- **Multi-sensor fusion (Kalman Filter)**: Defer to SoftwareAhrs/EKF implementation (T-p8w8f)
- **Compass calibration**: BNO086 has built-in calibration
- **Heading hold mode**: Future enhancement after basic Auto/Guided work

## Appendix

### References

**ArduPilot Documentation:**

- [Rover Navigation Tuning](https://ardupilot.org/rover/docs/rover-tuning-navigation.html)
- [ArduPilot AHRS](https://ardupilot.org/dev/docs/apmcopter-programming-attitude-control-2.html)
- [Guided Mode Commands](https://ardupilot.org/dev/docs/mavlink-rover-commands.html)

**Project Documentation:**

- [ADR-nzvfy AHRS Abstraction Architecture](../adr/ADR-nzvfy-ahrs-abstraction-architecture.md)
- [T-7khm3 AHRS Abstraction Layer Plan](../tasks/T-7khm3-ahrs-abstraction-layer/plan.md)

### Raw Data

**Current Code References:**

```rust
// NavigationController heading usage (src/subsystems/navigation/controller.rs:128-130)
let current_heading = current.course_over_ground.unwrap_or(0.0);
let heading_error = wrap_180(bearing - current_heading);

// SharedAhrsState API (src/subsystems/ahrs/traits.rs)
pub fn get_yaw(&self) -> f32  // Returns yaw in radians
pub fn is_healthy(&self) -> bool
pub fn read(&self) -> AhrsState
```

**Data Flow Diagram:**

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              Embassy Async Runtime                               │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                   │
│  ┌─────────────────┐     ┌──────────────────┐     ┌──────────────────────────┐  │
│  │  BNO086 Task    │────►│ SharedAhrsState  │────►│  HeadingSource           │  │
│  │  (100 Hz)       │     │  (global static) │     │  (FusedHeadingSource)    │  │
│  └─────────────────┘     └──────────────────┘     └────────────┬─────────────┘  │
│                                                                  │                │
│  ┌─────────────────┐                                            │                │
│  │  GPS Task       │─────────────────────────────────────────────┤                │
│  │  (1-10 Hz)      │                                            │                │
│  └─────────────────┘                                            ▼                │
│                                                   ┌──────────────────────────┐  │
│  ┌─────────────────┐                              │  NavigationController    │  │
│  │  Mode Task      │◄─────────────────────────────│  (heading, position)     │  │
│  │  (50 Hz)        │  steering/throttle           └──────────────────────────┘  │
│  │  [Auto/Guided]  │                                                            │
│  └────────┬────────┘                                                            │
│           │                                                                      │
│           ▼                                                                      │
│  ┌─────────────────┐                                                            │
│  │  Actuators      │                                                            │
│  │  (PWM output)   │                                                            │
│  └─────────────────┘                                                            │
│                                                                                   │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
