# AN-27568 Position Target Navigation via SET_POSITION_TARGET_GLOBAL_INT

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-7ix56-navigation-approach](AN-7ix56-navigation-approach.md)
  - [AN-r86b9-gps-position-telemetry-and-autonomous-navigation-foundation](AN-r86b9-gps-position-telemetry-and-autonomous-navigation-foundation.md)
  - [AN-lsq0s-guided-mode-heading-oscillation](AN-lsq0s-guided-mode-heading-oscillation.md)
- Related Requirements:
  - [FR-333ym-gps-waypoint-navigation](../requirements/FR-333ym-gps-waypoint-navigation.md)
  - [FR-sp3at-control-modes](../requirements/FR-sp3at-control-modes.md)
  - [FR-cs42u-gps-navigation-state-access](../requirements/FR-cs42u-gps-navigation-state-access.md)
  - [FR-obwjs-position-target-command-handler](../requirements/FR-obwjs-position-target-command-handler.md)
  - [FR-erpze-guided-mode-navigation](../requirements/FR-erpze-guided-mode-navigation.md)
  - [FR-jm7mj-auto-mode-mission-execution](../requirements/FR-jm7mj-auto-mode-mission-execution.md)
  - [FR-2vbe8-navigation-controller](../requirements/FR-2vbe8-navigation-controller.md)
  - [FR-tmibt-position-target-state](../requirements/FR-tmibt-position-target-state.md)
  - [NFR-wtdig-navigation-controller-performance](../requirements/NFR-wtdig-navigation-controller-performance.md)
- Related ADRs:
  - [ADR-wrcuk-navigation-controller-architecture](../adr/ADR-wrcuk-navigation-controller-architecture.md)
- Related Tasks:
  - [T-tto4f-navigation-controller](../tasks/T-tto4f-navigation-controller/README.md)
  - [T-f4r7a-break-spin-feedback-loop](../tasks/T-f4r7a-break-spin-feedback-loop/README.md)

## Executive Summary

This analysis examines the implementation of position target navigation using the MAVLink SET_POSITION_TARGET_GLOBAL_INT message, which enables GCS or companion computers to command the vehicle to navigate to specific GPS coordinates. This capability is fundamental for both Guided mode (real-time position commands) and Auto mode (waypoint mission execution).

With GPS position telemetry now functional (lat/lon acquisition via NEO-M8N), the project can implement the remaining components needed for autonomous navigation: receiving position targets, implementing navigation controllers (L1 or S-curve), and executing the Auto/Guided flight modes. This analysis identifies the required components and recommends a phased implementation approach.

## Problem Space

### Current State

**GPS Subsystem (Complete):**

- NEO-M8N GPS on UART0 provides position data (latitude, longitude, altitude)
- GPS_RAW_INT and GLOBAL_POSITION_INT telemetry implemented
- Course over ground (COG) and heading (hdg) fields populated
- GPS state accessible via `NavigationState` for navigation subsystem

**MAVLink Infrastructure (Partial):**

- Message router handles incoming/outgoing MAVLink messages
- Command handler processes MAV_CMD commands
- Mission handler manages waypoint upload/download (MISSION_ITEM_INT)
- SET_POSITION_TARGET_GLOBAL_INT handler NOT implemented

**Mode System (Partial):**

- FlightMode enum includes Auto, Guided modes
- Mode trait defined with enter/exit/update methods
- Manual mode functional
- Hold mode functional
- **Auto mode NOT implemented** (requires navigation)
- **Guided mode NOT implemented** (requires position target handling)

**Navigation System (Not Implemented):**

- No navigation controller (L1 or S-curve)
- No position target storage/management
- No path following logic

### Desired State

1. **SET_POSITION_TARGET_GLOBAL_INT Handler:**
   - Receive position targets from GCS
   - Validate coordinates and type_mask
   - Store target for navigation system
   - Support position-only and velocity commands

2. **Guided Mode:**
   - Accept real-time position targets from GCS
   - Navigate to commanded position using navigation controller
   - Support "Fly To Here" functionality from Mission Planner/QGC

3. **Auto Mode:**
   - Follow uploaded waypoint mission
   - Progress through waypoints automatically
   - Use same navigation controller as Guided mode

4. **Navigation Controller:**
   - Calculate steering/throttle commands to reach target position
   - Initially implement L1 controller (simpler, lower resource)
   - Future: S-curve path planning for smoother navigation

### Gap Analysis

| Component                       | Current State   | Desired State             | Gap                        |
| ------------------------------- | --------------- | ------------------------- | -------------------------- |
| SET_POSITION_TARGET_GLOBAL_INT  | Not implemented | Handles position commands | New handler needed         |
| Position target storage         | None            | Shared navigation target  | State management           |
| L1 Navigation controller        | Not implemented | Basic path following      | Algorithm implementation   |
| Guided mode                     | Enum only       | Functional mode           | Mode implementation        |
| Auto mode                       | Enum only       | Functional mode           | Mode + mission integration |
| Navigation ↔ Control interface | None            | Steering/throttle output  | Integration layer          |

## Stakeholder Analysis

| Stakeholder     | Interest/Need                               | Impact | Priority |
| --------------- | ------------------------------------------- | ------ | -------- |
| GCS User        | Click on map to command vehicle movement    | High   | P0       |
| Mission Planner | Auto mode execution of uploaded missions    | High   | P0       |
| Control System  | Navigation output (steering/throttle)       | High   | P0       |
| Safety System   | Navigation failsafe on target loss/GPS loss | High   | P1       |
| Developers      | Clean architecture for navigation extension | Medium | P1       |

## Research & Discovery

### User Feedback

N/A - Derived from ArduPilot standard functionality and existing requirements.

### Competitive Analysis

**ArduPilot Guided Mode:**

- Supports SET_POSITION_TARGET_GLOBAL_INT for position/velocity control
- type_mask determines which fields are active
- Position commands take precedence over velocity
- Velocity commands timeout after 3 seconds without refresh
- Supported coordinate frames: GLOBAL, GLOBAL_RELATIVE_ALT, GLOBAL_TERRAIN_ALT

**ArduPilot Auto Mode:**

- Uses MISSION_ITEM_INT waypoints uploaded via MAVLink
- Progresses through mission using same navigation controller
- MAV_CMD_NAV_WAYPOINT items define target positions
- Waypoint acceptance radius determines when to advance

### Technical Investigation

#### SET_POSITION_TARGET_GLOBAL_INT Message (ID: 86)

```
time_boot_ms     (uint32)  - Timestamp since boot (ms)
target_system    (uint8)   - Target system ID
target_component (uint8)   - Target component ID
coordinate_frame (uint8)   - Coordinate frame (MAV_FRAME)
type_mask        (uint16)  - Bitmask for ignored fields
lat_int          (int32)   - Latitude (degE7)
lon_int          (int32)   - Longitude (degE7)
alt              (float)   - Altitude (m)
vx               (float)   - X velocity (m/s, North)
vy               (float)   - Y velocity (m/s, East)
vz               (float)   - Z velocity (m/s, Down)
afx              (float)   - X acceleration (ignored by Rover)
afy              (float)   - Y acceleration (ignored by Rover)
afz              (float)   - Z acceleration (ignored by Rover)
yaw              (float)   - Yaw setpoint (rad)
yaw_rate         (float)   - Yaw rate setpoint (rad/s)
```

#### type_mask Values for Rover

| Control Mode   | Hex    | Decimal | Binary         | Description            |
| -------------- | ------ | ------- | -------------- | ---------------------- |
| Position       | 0x0DFC | 3580    | 0b110111111100 | Use lat/lon only       |
| Velocity       | 0x0DE7 | 3559    | 0b110111100111 | Use vx/vy only         |
| Yaw            | 0x09FF | 2559    | 0b100111111111 | Use yaw only           |
| Yaw Rate       | 0x05FF | 1535    | 0b010111111111 | Use yaw_rate only      |
| Vel + Yaw      | 0x09E7 | 2535    | 0b100111100111 | Velocity with yaw      |
| Vel + Yaw Rate | 0x05E7 | 1511    | 0b010111100111 | Velocity with yaw rate |

**type_mask Bit Definitions:**

- Bit 0: Ignore PosX (lat)
- Bit 1: Ignore PosY (lon)
- Bit 2: Ignore PosZ (alt)
- Bit 3: Ignore VelX
- Bit 4: Ignore VelY
- Bit 5: Ignore VelZ
- Bit 6: Ignore AccX
- Bit 7: Ignore AccY
- Bit 8: Ignore AccZ
- Bit 10: Ignore Yaw
- Bit 11: Ignore Yaw Rate

**Constraints:**

- Position OR velocity must be provided (not both)
- If position provided, velocity/yaw/yaw_rate ignored
- Velocity commands timeout after 3 seconds
- Acceleration fields not supported by Rover

#### L1 Navigation Controller

The L1 controller is a proven lateral guidance algorithm:

```
                Current Position
                      ●
                       \
                        \  L1 Distance
                         \
                          ● Reference Point on Path
                           \
                            ────────────────────
                          Start            Target
```

**Algorithm:**

1. Calculate bearing to target from current position
2. Calculate cross-track error (distance from ideal path)
3. Compute lateral acceleration command to converge to path
4. Convert lateral acceleration to steering command

**Parameters (ArduPilot-compatible):**

```
NAVL1_PERIOD: L1 controller period (default 20 seconds for Rover)
NAVL1_DAMPING: Damping ratio (default 0.75)
```

**Simplified L1 for Initial Implementation:**

For direct point-to-point navigation (no path following):

```rust
fn calculate_steering(current: GpsPosition, target: GpsPosition) -> f32 {
    let bearing_to_target = calculate_bearing(current, target);
    let heading_error = wrap_180(bearing_to_target - current.heading);
    let steering = (heading_error / MAX_HEADING_ERROR).clamp(-1.0, 1.0);
    steering
}
```

### Data Analysis

**Navigation Update Rate Budget:**

| Component                 | Rate    | Notes                        |
| ------------------------- | ------- | ---------------------------- |
| GPS update                | 1-10Hz  | NEO-M8N default 1Hz          |
| Navigation controller     | 10-50Hz | Can interpolate GPS position |
| Control loop              | 50Hz    | Receives nav output          |
| Position target timeout   | 3s      | Velocity commands only       |
| Waypoint acceptance check | 10Hz    | Distance calculation         |

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: System shall receive SET_POSITION_TARGET_GLOBAL_INT messages → Will become FR-<id>
  - Rationale: Standard MAVLink message for position commands in Guided mode
  - Acceptance Criteria:
    - Parses lat_int/lon_int/alt fields correctly
    - Validates type_mask to determine control mode
    - Rejects invalid coordinate_frame values
    - Stores target position for navigation system

- [ ] **FR-DRAFT-2**: System shall implement Guided mode with position target navigation → Will become FR-<id>
  - Rationale: Required for GCS "Fly To Here" functionality
  - Acceptance Criteria:
    - Accepts position targets from SET_POSITION_TARGET_GLOBAL_INT
    - Navigates to target position using navigation controller
    - Stops at target within acceptance radius
    - Exits on mode change or new target

- [ ] **FR-DRAFT-3**: System shall implement Auto mode with waypoint mission execution → Will become FR-<id>
  - Rationale: Core autonomous operation capability
  - Acceptance Criteria:
    - Executes uploaded mission waypoints in sequence
    - Uses navigation controller to reach each waypoint
    - Advances to next waypoint within acceptance radius
    - Completes mission at final waypoint

- [ ] **FR-DRAFT-4**: System shall implement L1 navigation controller → Will become FR-<id>
  - Rationale: Proven algorithm for point-to-point navigation
  - Acceptance Criteria:
    - Calculates steering command to reach target
    - Respects vehicle turn rate limits
    - Provides throttle command based on distance to target
    - Works with GPS update rate of 1-10Hz

- [ ] **FR-DRAFT-5**: System shall manage position target state → Will become FR-<id>
  - Rationale: Navigation needs shared access to current target
  - Acceptance Criteria:
    - Stores target position (lat/lon/alt)
    - Indicates target validity (valid/invalid/expired)
    - Provides distance and bearing to target
    - Clears target on mode exit

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Navigation controller shall execute within 2ms per update → Will become NFR-<id>
  - Category: Performance
  - Rationale: Must not impact 50Hz control loop
  - Target: Measured on RP2350

- [ ] **NFR-DRAFT-2**: Position target response latency shall be < 100ms → Will become NFR-<id>
  - Category: Responsiveness
  - Rationale: GCS expects prompt response to commands
  - Target: From message receipt to navigation start

- [ ] **NFR-DRAFT-3**: Navigation shall support GPS update rates from 1Hz to 10Hz → Will become NFR-<id>
  - Category: Flexibility
  - Rationale: Different GPS modules have different update rates
  - Target: Functional at 1Hz, optimal at 5Hz+

## Design Considerations

### Technical Constraints

**Existing Architecture:**

- MAVLink router dispatches messages to handlers
- SystemState/NavigationState holds GPS position
- Control loop runs at 50Hz
- Mode trait defines enter/exit/update interface

**Embassy Async Runtime:**

- Navigation can run as async task
- Position target updates via channel or shared state
- Control loop consumes navigation output

**Memory Constraints:**

- L1 controller requires \~1KB RAM for state
- Position target: \~32 bytes
- Navigation state: \~64 bytes

### Potential Approaches

#### Approach A: Simplified Direct Navigation

**Description:** Implement basic bearing-to-target steering without full L1 controller.

**Pros:**

- Simple implementation (\~100 lines)
- Minimal RAM usage
- Easy to debug and tune
- Sufficient for low-speed rovers

**Cons:**

- No path following (only point-to-point)
- May overshoot targets at higher speeds
- Less smooth turns

**Effort:** Low

#### Approach B: Full L1 Controller

**Description:** Implement ArduPilot-style L1 lateral controller.

**Pros:**

- Smooth path following
- Proven algorithm
- Compatible with ArduPilot parameters
- Foundation for S-curve later

**Cons:**

- More complex implementation
- More parameters to tune
- Higher CPU usage

**Effort:** Medium

#### Approach C: Phased Implementation (Recommended)

**Description:** Start with simplified navigation, upgrade to L1, then S-curve.

**Phase 1: Basic Navigation**

- Implement SET_POSITION_TARGET_GLOBAL_INT handler
- Simple bearing-to-target steering
- Guided mode with position targets
- Validates architecture

**Phase 2: L1 Controller**

- Replace simple navigation with L1
- Add path following for Auto mode
- ArduPilot parameter compatibility

**Phase 3: S-Curve (Future)**

- Add S-curve path planning
- Smooth acceleration profiles
- Full ArduPilot 4.3+ compatibility

**Pros:**

- Incremental complexity
- Validates architecture early
- Delivers value at each phase
- Reduces implementation risk

**Cons:**

- Requires refactoring between phases
- Total effort higher than single implementation

**Effort:** Medium (Phase 1: Low, Phase 2: Medium, Phase 3: High)

### Architecture Impact

**New Components:**

- `src/communication/mavlink/handlers/position_target.rs`: SET_POSITION_TARGET_GLOBAL_INT handler
- `src/subsystems/navigation/mod.rs`: Navigation subsystem
- `src/subsystems/navigation/target.rs`: Position target state management
- `src/subsystems/navigation/l1.rs`: L1 controller (Phase 2)
- `src/rover/mode/guided.rs`: Guided mode implementation
- `src/rover/mode/auto.rs`: Auto mode implementation

**Modified Components:**

- `src/communication/mavlink/router.rs`: Route new message to handler
- `src/rover/mode/mod.rs`: Add Guided/Auto mode implementations
- `src/core/mission/mod.rs`: Integration with Auto mode

**ADRs Required:**

- ADR for navigation controller selection (simple → L1 → S-curve)
- ADR for position target state management pattern

## Risk Assessment

| Risk                                     | Probability | Impact | Mitigation Strategy                              |
| ---------------------------------------- | ----------- | ------ | ------------------------------------------------ |
| GPS position lag causes navigation error | Medium      | Medium | Add dead reckoning prediction from IMU           |
| Position target timeout causes stops     | Medium      | Low    | Implement target hold behavior                   |
| Navigation overshoot at high speed       | Medium      | Medium | Reduce speed near target, tune acceptance radius |
| type_mask parsing errors                 | Low         | Medium | Unit tests for all type_mask combinations        |
| Mode transition during navigation        | Low         | Low    | Clean state on mode exit                         |

## Open Questions

- [ ] Should velocity commands be supported in Phase 1, or position-only?
  - Recommendation: Position-only for Phase 1, add velocity in Phase 2
- [ ] What should happen when GPS fix is lost during navigation?
  - Recommendation: Trigger failsafe (Hold mode or RTL) per existing failsafe system
- [ ] Should navigation support terrain-following altitude?
  - Recommendation: Not required for rover/boat, use GPS altitude only
- [ ] What is the minimum acceptable waypoint acceptance radius?
  - Next step: Test with 2m default, allow configuration via WP_RADIUS

## Recommendations

### Immediate Actions

1. **Create SET_POSITION_TARGET_GLOBAL_INT handler:**
   - Parse message fields
   - Validate type_mask (position-only for Phase 1)
   - Store target in NavigationState

2. **Implement position target state:**
   - Add target position to NavigationState
   - Include validity flag and timestamp
   - Provide distance/bearing calculations

3. **Implement basic Guided mode:**
   - Accept position target on mode entry
   - Calculate steering toward target
   - Stop when within acceptance radius

### Next Steps

1. [ ] Create formal requirements from FR-DRAFT items
2. [ ] Draft ADR: Navigation controller selection strategy
3. [ ] Create task: T-<id>-position-target-handler (Phase 1)
4. [ ] Create task: T-<id>-guided-mode-implementation
5. [ ] Create task: T-<id>-auto-mode-implementation

### Out of Scope

- **S-curve path planning**: Deferred to Phase 3 per AN-7ix56
- **Velocity control mode**: Phase 2 (position-only for Phase 1)
- **Terrain following**: Not applicable for rover
- **Spline waypoints**: Future enhancement after basic waypoints work
- **Obstacle avoidance**: Separate feature outside navigation core

## Field Testing: Post-T-w8x3p Analysis

### Context

T-w8x3p implemented PD steering, heading-error throttle reduction, and heading source hysteresis to address the root causes identified in AN-lsq0s. Field testing shows the spinning problem persists. This section documents additional root causes discovered through code analysis and field observation.

### Observed Behavior (Post-Fix)

- **Spinning persists**: Rover still spins in circles during Guided mode navigation
- **Intermittent success**: Occasionally navigates straight and reaches the target
- **Physical vibration**: Spinning causes mechanical vibration (shaking), suspected to corrupt AHRS yaw readings
- **Confirmed correct**: ENU→NED quaternion conversion verified mathematically (CW positive, 0°=North, 90°=East)

### Additional Root Causes

#### Root Cause 6: Spin-in-Place Positive Feedback Loop (Critical)

When heading error exceeds 90°, throttle drops to 0 (heading-error throttle reduction works). However, steering remains at full output (1.0). `DifferentialDrive::mix(1.0, 0.0)` produces `left=1.0, right=-1.0` — maximum-speed spin in place.

The feedback loop:

```text
Large heading error (>90°)
  → throttle = 0, steering = 1.0
  → full-speed spin in place
  → vibration from spinning
  → AHRS yaw noise / drift
  → heading error stays large or oscillates
  → spinning continues
```

This loop is self-sustaining because the spin itself corrupts the heading data needed to stop spinning.

#### Root Cause 7: Slew Rate Limiting Disabled in Defaults (High)

`SimpleNavConfig::default()` sets `max_steering_rate: 0.0`, which disables slew rate limiting entirely. The T-w8x3p design document specified `max_steering_rate: 2.0` but the implementation default was left at 0.0. Without slew rate limiting, steering can jump from -1.0 to +1.0 instantly.

**Location**: `crates/core/src/navigation/types.rs:83`

#### Root Cause 8: D-Gain Effectively Negligible (Medium)

`steering_d_gain: 0.005` is too small to provide meaningful damping at normal heading change rates.

| Heading Rate (°/s) | error_rate | D-term (gain=0.005) | Effect     |
| ------------------ | ---------- | ------------------- | ---------- |
| 10                 | 10         | 0.05                | Negligible |
| 50                 | 50         | 0.25                | Weak       |
| 200                | 200        | 1.0                 | Meaningful |
| 500                | 500        | 2.5 (clamped)       | Saturated  |

The D-term only becomes significant at extreme heading rates (>200°/s) that already indicate a spinning condition.

#### Root Cause 9: No Heading Smoothing Filter (Medium)

Raw AHRS yaw is passed directly to the navigation controller without any filtering. Vibration-induced noise in the AHRS heading maps directly to heading error fluctuations. Combined with the P-term (which amplifies any heading error), noise passes through to steering output.

#### Root Cause 10: Priority 3 Heading Fallback (Low-Medium)

`FusedHeadingSource::get_heading()` has a Priority 3 fallback that uses GPS COG even below the speed threshold when AHRS is unhealthy. When spinning in place:

- GPS speed is low → `get_gps_heading()` returns None
- If vibration causes BNO086 to report unhealthy → AHRS heading unavailable
- Falls to Priority 3: raw GPS COG, which is essentially random when spinning in place
- Creates large heading discontinuity

**Location**: `crates/firmware/src/subsystems/navigation/heading.rs:176-181`

#### Root Cause 11: Insufficient Diagnostic Logging (Low)

The navigation log output includes `dist, bearing, steering, throttle, at_target` but omits:

- Actual heading value
- Heading error
- Heading source type (GPS COG vs AHRS)
- Compass yaw offset value

Without these fields, field debugging requires guessing which heading source is active and what heading value the controller sees.

**Location**: `crates/firmware/examples/pico_trail_rover.rs:1358-1365`

### Verified Correct Components

| Component                 | Verification Method | Result                                       |
| ------------------------- | ------------------- | -------------------------------------------- |
| ENU→NED quaternion        | Mathematical proof  | CW positive, 0°=North, 90°=East (correct)    |
| `calculate_bearing`       | Formula review      | Standard great-circle, 0°=North, CW positive |
| `DifferentialDrive::mix`  | Sign trace          | Positive steering → right turn (correct)     |
| `wrap_180`                | Edge case review    | Correct wrapping to \[-180, +180]            |
| Heading source hysteresis | Code review         | Correctly uses upper/lower threshold         |
| Throttle heading scaling  | Code review         | Correctly reduces to 0 at 90°+               |

### Proposed Mitigations

Listed in priority order:

#### Mitigation 1: Enable Slew Rate Limiting

Set `max_steering_rate` to a non-zero value (2.0 recommended — full range change in 1 second). At 50 Hz, max steering change per frame = 0.04. This prevents instantaneous steering reversals.

**Effort**: Minimal (change default value)

#### Mitigation 2: Limit Spin-in-Place Speed

When throttle is near zero, cap steering output to a lower maximum (e.g., 0.3–0.5 instead of 1.0). This reduces spin speed, reducing vibration, giving the AHRS a better chance to track yaw correctly.

```text
effective_steering = if throttle < 0.1 {
    steering.clamp(-MAX_SPIN_STEERING, MAX_SPIN_STEERING)
} else {
    steering
}
```

**Effort**: Small (add clamping in controller update)

#### Mitigation 3: Heading Exponential Moving Average Filter

Apply a low-pass filter to heading before using it for navigation. This smooths out vibration-induced noise:

```text
smoothed_heading = alpha * new_heading + (1 - alpha) * prev_heading
```

With proper angle wrapping. Alpha of 0.3–0.5 (at 50 Hz) provides smoothing while maintaining responsiveness.

**Effort**: Medium (add filter with angle-aware smoothing)

#### Mitigation 4: Use Gyroscope Yaw Rate for D-Term

Instead of computing the derivative from successive heading errors (which amplifies measurement noise), use the BNO086 gyroscope yaw rate directly. The gyroscope rate is already available in NED frame (`angular_rate.z`) and is less affected by vibration-induced heading noise.

**Effort**: Medium (pass angular rate through heading source to controller)

#### Mitigation 5: Improve Diagnostic Logging

Add heading, heading error, and heading source type to navigation log output:

```text
"Nav: heading={}°, error={}°, src={:?}, dist={}m, steer={}, thr={}"
```

**Effort**: Minimal

### Updated Architecture Diagram (Post-Mitigations)

```text
navigation_task (50Hz)
  ├── heading_source.get_heading()
  │     ├── GPS speed vs threshold WITH HYSTERESIS
  │     └── Track using_gps_cog state
  ├── heading_filter.apply(raw_heading)              ← NEW: EMA filter
  │     └── smoothed heading (reduces noise)
  ├── controller.update(lat, lon, target, heading, dt)
  │     ├── calculate_bearing(current → target)
  │     ├── heading_error = wrap_180(bearing - heading)
  │     ├── steering = PD(heading_error, dt) + slew rate limit
  │     ├── throttle = f(distance) * f(heading_error)
  │     └── spin_limit: if throttle ≈ 0, cap steering  ← NEW
  └── NAV_OUTPUT = { steering, throttle }
```

## Appendix

### References

**MAVLink Messages:**

- [SET_POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT)
- [POSITION_TARGET_GLOBAL_INT](https://mavlink.io/en/messages/common.html#POSITION_TARGET_GLOBAL_INT)
- [POSITION_TARGET_TYPEMASK](https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK)

**ArduPilot Documentation:**

- [Rover Commands in Guided Mode](https://ardupilot.org/dev/docs/mavlink-rover-commands.html)
- [Guided Mode](https://ardupilot.org/rover/docs/guided-mode.html)
- [L1 Controller](https://ardupilot.org/plane/docs/navigation-tuning.html)

### Raw Data

**type_mask Validation Logic:**

```rust
const IGNORE_POS_X: u16 = 1 << 0;
const IGNORE_POS_Y: u16 = 1 << 1;
const IGNORE_POS_Z: u16 = 1 << 2;
const IGNORE_VEL_X: u16 = 1 << 3;
const IGNORE_VEL_Y: u16 = 1 << 4;
const IGNORE_VEL_Z: u16 = 1 << 5;
const IGNORE_YAW: u16 = 1 << 10;
const IGNORE_YAW_RATE: u16 = 1 << 11;

fn is_position_target(type_mask: u16) -> bool {
    // Position NOT ignored (bits 0,1 are 0)
    (type_mask & (IGNORE_POS_X | IGNORE_POS_Y)) == 0
}

fn is_velocity_target(type_mask: u16) -> bool {
    // Velocity NOT ignored AND Position IS ignored
    let pos_ignored = (type_mask & (IGNORE_POS_X | IGNORE_POS_Y)) != 0;
    let vel_active = (type_mask & (IGNORE_VEL_X | IGNORE_VEL_Y)) == 0;
    pos_ignored && vel_active
}
```

**Distance Calculation (Haversine):**

```rust
fn distance_m(pos1: &GpsPosition, pos2: &GpsPosition) -> f32 {
    const EARTH_RADIUS_M: f32 = 6_371_000.0;
    let lat1 = pos1.latitude.to_radians();
    let lat2 = pos2.latitude.to_radians();
    let dlat = (pos2.latitude - pos1.latitude).to_radians();
    let dlon = (pos2.longitude - pos1.longitude).to_radians();

    let a = (dlat / 2.0).sin().powi(2)
        + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();
    EARTH_RADIUS_M * c
}
```

**Bearing Calculation:**

```rust
fn bearing_deg(from: &GpsPosition, to: &GpsPosition) -> f32 {
    let lat1 = from.latitude.to_radians();
    let lat2 = to.latitude.to_radians();
    let dlon = (to.longitude - from.longitude).to_radians();

    let x = dlon.sin() * lat2.cos();
    let y = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
    let bearing = x.atan2(y).to_degrees();
    (bearing + 360.0) % 360.0
}
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
