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
  - [ADR-h3k9f-heading-source-integration](../adr/ADR-h3k9f-heading-source-integration.md)
- Related Tasks:
  - [T-7khm3-ahrs-abstraction-layer](../tasks/T-7khm3-ahrs-abstraction-layer/README.md) (Complete)
  - [T-tto4f-navigation-controller](../tasks/T-tto4f-navigation-controller/README.md) (Complete)
  - [T-x8mq2-bno086-driver-implementation](../tasks/T-x8mq2-bno086-driver-implementation/README.md) (Complete)
  - [T-r9v2k-heading-source-navigation-integration](../tasks/T-r9v2k-heading-source-navigation-integration/README.md)

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

ArduPilot uses EKF3-based sensor fusion for heading estimation. The heading source is controlled by the `EK3_SRC1_YAW` parameter:

| Value | Source                 | Description                                       |
| ----- | ---------------------- | ------------------------------------------------- |
| 1     | Compass                | Default - magnetometer-based heading              |
| 2     | GPS                    | Dual GPS Moving Baseline (requires 2x uBlox F9)   |
| 3     | GPS + Compass Fallback | GPS yaw with compass offset learning for failover |
| 6     | ExternalNav            | External navigation system (e.g., RealSense T265) |
| 8     | GSF                    | Gaussian Sum Filter - GPS velocity + IMU fusion   |

**GSF (Gaussian Sum Filter):**

For compass-less operation, ArduPilot uses GSF which estimates heading from GPS velocity and IMU data:

- Requires 5 consecutive valid GPS velocity samples for yaw alignment (`GPS_VEL_YAW_ALIGN_COUNT_THRESHOLD`)
- Maximum 15° uncertainty tolerance (`GPS_VEL_YAW_ALIGN_MAX_ANG_ERR`)
- Automatic fallback when compass is disabled (`COMPASS_ENABLE = 0`)
- Depends on good GPS velocity reports (uBlox M8 or better recommended)

**Key Parameters:**

- `EK3_SRC1_YAW`: Primary yaw source selection (1=Compass, 8=GSF, etc.)
- `GPS_NAVFILTER`: GPS navigation filter mode for uBlox receivers
  - 0: Portable, 4: Automotive, 5: Sea, 8: Airborne4G (default)
- `EK3_YAW_M_NSE`: Compass/GPS weighting in EKF (default 0.5, lower = trust compass more)
- `COMPASS_ENABLE`: Enable/disable compass (0 = disabled, triggers GSF fallback)

**Source Switching Mechanisms:**

- RC auxiliary switch (`RCx_OPTION = 90`) for manual EKF source set switching
- MAVLink command `MAV_CMD_SET_EKF_SOURCE_SET` for programmatic switching
- Three independent source configuration sets (`EK3_SRC1_*`, `EK3_SRC2_*`, `EK3_SRC3_*`)

**References:**

- [EKF Source Selection and Switching](https://ardupilot.org/copter/docs/common-ekf-sources.html)
- [Compass-less Operation](https://ardupilot.org/rover/docs/common-compassless.html)
- [AP_NavEKF3_core.h](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_core.h)

**ArduPilot Guided Mode:**

Rover Guided mode supports multiple submodes via SET_POSITION_TARGET_GLOBAL_INT:

| Submode             | Description                                    |
| ------------------- | ---------------------------------------------- |
| WP                  | Waypoint navigation with S-curve path planning |
| HeadingAndSpeed     | Direct heading control (±5° tolerance)         |
| TurnRateAndSpeed    | Angular velocity-based steering                |
| SteeringAndThrottle | Raw motor control inputs                       |
| Loiter              | Circular holding pattern                       |
| Stop                | Emergency vehicle halt                         |

**Bitmask for SET_POSITION_TARGET_GLOBAL_INT:**

| Bitmask        | Decimal | Usage         |
| -------------- | ------- | ------------- |
| 0b110111111100 | 3580    | Position only |
| 0b110111100111 | 3559    | Velocity only |
| 0b100111111111 | 2559    | Heading only  |
| 0b010111111111 | 1535    | Yaw rate only |

- Velocity commands require re-send every 1 second (3-second timeout triggers stop)
- Geofence may silently reject targets outside fence boundary

**References:**

- [Rover Commands in Guided Mode](https://ardupilot.org/dev/docs/mavlink-rover-commands.html)
- [Guided Mode Documentation](https://ardupilot.org/rover/docs/guided-mode.html)

**ArduPilot Auto Mode:**

Rover Auto mode executes uploaded waypoint missions with:

- AHRS origin confirmation before mission start
- Mission change detection and waypoint command restart
- Loiter duration management (`cmd.p1` seconds)
- Trigger mechanisms: digital pin, acceleration kickstart (`AUTO_KICKSTART`), or unrestricted

**References:**

- [mode_auto.cpp](https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_auto.cpp)

### Codebase Investigation

**Current AHRS Architecture:**

The AHRS subsystem provides a trait-based abstraction for attitude estimation:

| Component               | Location                                    | Status   |
| ----------------------- | ------------------------------------------- | -------- |
| `Ahrs` trait            | `src/subsystems/ahrs/traits.rs:250`         | Complete |
| `AhrsState` struct      | `src/subsystems/ahrs/traits.rs:85`          | Complete |
| `SharedAhrsState`       | `src/subsystems/ahrs/traits.rs:280`         | Complete |
| `Bno086ExternalAhrs<T>` | `src/subsystems/ahrs/external/bno086.rs:48` | Complete |

**`SharedAhrsState` API** (thread-safe global state):

```rust
// Read operations (no mutex required - uses critical section)
pub fn read(&self) -> AhrsState
pub fn get_yaw(&self) -> f32           // radians
pub fn get_quaternion(&self) -> Quaternion<f32>
pub fn is_healthy(&self) -> bool

// Write operations (called from AHRS task)
pub fn write(&self, state: &AhrsState)
pub fn update_quaternion(&self, q: Quaternion<f32>, angular_rate: Vector3<f32>, ...)
```

**Current Navigation Controller:**

`SimpleNavigationController` (`src/subsystems/navigation/controller.rs:130`) uses GPS COG exclusively:

```rust
let current_heading = current.course_over_ground.unwrap_or(0.0);
```

When COG is unavailable (vehicle stationary), it defaults to 0° and reduces throttle.

**Existing heading_provider Pattern (CircleMode):**

`CircleMode` (`src/rover/mode/circle.rs:152`) demonstrates AHRS integration:

```rust
pub struct CircleMode<'a> {
    // ...
    #[cfg(feature = "embassy")]
    heading_provider: fn() -> Option<f32>,  // External heading source
}

// Usage (line 260-262):
let heading = (self.heading_provider)()
    .or(gps.course_over_ground)
    .ok_or("No heading available")?;
```

This pattern:

- Tries AHRS heading first via function pointer
- Falls back to GPS COG if AHRS unavailable
- Returns error if neither source is available

**Mode Implementations Status:**

| Mode     | Status          | Heading Source          |
| -------- | --------------- | ----------------------- |
| Manual   | Complete        | N/A (direct RC)         |
| RTL      | Complete        | GPS COG only            |
| SmartRTL | Complete        | GPS COG only            |
| Circle   | Complete        | AHRS → GPS COG fallback |
| Loiter   | Complete        | GPS COG only            |
| Guided   | Not implemented | -                       |
| Auto     | Not implemented | -                       |

**Global Navigation State** (`src/subsystems/navigation/mod.rs:68-89`):

```rust
pub static NAV_TARGET: Mutex<...> = ...;      // SET_POSITION_TARGET_GLOBAL_INT destination
pub static NAV_OUTPUT: Mutex<...> = ...;      // Navigation output at 50Hz
pub static REPOSITION_TARGET: Mutex<...> = ...; // MAV_CMD_DO_REPOSITION destination
```

These global states are defined but not yet consumed by any Mode implementation.

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

- Pros: Simple, easy to implement and debug
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

#### Option 3: Kalman Filter Fusion (EKF/GSF)

- Pros: Optimal fusion with uncertainty estimation
- Cons: High complexity, requires careful tuning
- Recommendation: Consider for future SoftwareAhrs implementation

**Recommendation:** Start with Option 1 (simple switch) for initial implementation. Migrate to Option 2 (blended) if heading jitter is observed at speed transitions. Option 3 (EKF/GSF) should be considered when implementing the full SoftwareAhrs with sensor fusion.

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
