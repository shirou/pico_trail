# AN-00036 RTL Mode | Return to Launch Navigation

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-00011-failsafe-system](AN-00011-failsafe-system.md)
  - [AN-00035-battery-rtl](AN-00035-battery-rtl.md) (depends on this)
- Related Requirements:
  - [FR-00122-rtl-navigate-home](../requirements/FR-00122-rtl-navigate-home.md)
  - [FR-00120-rtl-entry-validation](../requirements/FR-00120-rtl-entry-validation.md)
  - [FR-00119-rtl-arrival-stop](../requirements/FR-00119-rtl-arrival-stop.md)
  - [FR-00121-rtl-gps-loss-handling](../requirements/FR-00121-rtl-gps-loss-handling.md)
  - [FR-00123-smartrtl-path-recording](../requirements/FR-00123-smartrtl-path-recording.md)
  - [FR-00124-smartrtl-path-simplification](../requirements/FR-00124-smartrtl-path-simplification.md)
  - [FR-00125-smartrtl-return-navigation](../requirements/FR-00125-smartrtl-return-navigation.md)
  - [FR-00126-smartrtl-rtl-fallback](../requirements/FR-00126-smartrtl-rtl-fallback.md)
  - [NFR-00085-rtl-update-rate](../requirements/NFR-00085-rtl-update-rate.md)
  - [NFR-00084-rtl-memory-overhead](../requirements/NFR-00084-rtl-memory-overhead.md)
  - [NFR-00083-rtl-entry-validation-time](../requirements/NFR-00083-rtl-entry-validation-time.md)
  - [NFR-00086-smartrtl-memory-budget](../requirements/NFR-00086-smartrtl-memory-budget.md)
- Related ADRs:
  - [ADR-00031-rtl-smartrtl-architecture](../adr/ADR-00031-rtl-smartrtl-architecture.md)
- Related Tasks:
  - [T-00030-rtl-smartrtl-implementation](../tasks/T-00030-rtl-smartrtl-implementation/README.md)

## Executive Summary

This analysis explores the implementation of RTL (Return to Launch) mode for the pico_trail rover. RTL is a fundamental autonomous mode that navigates the vehicle back to its launch point (home position). Currently, `FlightMode::Rtl` exists as an enum value but has no implementation - the rover cannot autonomously return home.

Key findings: The navigation infrastructure required for RTL is already in place (`NavigationController`, `PositionTarget`, `HomePosition`). RTL implementation requires creating `src/rover/mode/rtl.rs` that uses the existing `SimpleNavigationController` to navigate toward the stored home position. This is a straightforward implementation leveraging existing components.

## Problem Space

### Current State

The project has:

- **RTL Enum**: `FlightMode::Rtl` defined in `src/communication/mavlink/state.rs:88`
- **Home Position**: `HomePosition` struct in `src/communication/mavlink/state.rs:213` with lat/lon/alt
- **Home Setting**: `MAV_CMD_DO_SET_HOME` handler sets home via GCS or auto-sets on first GPS fix
- **Navigation Controller**: `SimpleNavigationController` in `src/subsystems/navigation/controller.rs`
- **Position Target**: `PositionTarget` type for navigation destinations
- **Mode Trait**: `Mode` trait in `src/rover/mode/mod.rs` defines enter/update/exit lifecycle

**What exists**:

```
src/rover/mode/
├── mod.rs      # Mode trait definition
├── manual.rs   # Manual mode ✅
├── circle.rs   # Circle mode ✅
└── loiter.rs   # Loiter mode ✅
```

**What's missing**:

```
src/rover/mode/
└── rtl.rs      # RTL mode ❌ NOT IMPLEMENTED
```

**Critical gaps**:

- **No RTL mode file**: `src/rover/mode/rtl.rs` does not exist
- **No navigation to home**: Cannot navigate vehicle to home position
- **No RTL completion handling**: No behavior defined when vehicle reaches home
- **No RTL entry validation**: No check for GPS fix or home position before entering RTL

### Desired State

Implement RTL mode with the following behavior:

1. **Mode Entry**: Validate GPS fix and home position are available
2. **Navigation**: Use `SimpleNavigationController` to navigate toward home
3. **Completion**: Stop at home position (within WP_RADIUS)
4. **Telemetry**: Report distance to home, bearing, ETA
5. **Safety**: Handle GPS loss during RTL (enter Hold mode)

Success criteria:

- Vehicle can autonomously return to launch point from any location
- RTL works as a failsafe action (battery, RC loss, GCS loss)
- Mode integrates with existing mode switching infrastructure
- Navigation uses proven `SimpleNavigationController`

### Gap Analysis

**Missing components**:

1. **RTL Mode Implementation**: `src/rover/mode/rtl.rs` with `Mode` trait implementation
2. **Home Position Validation**: Check home is set before RTL entry
3. **GPS Fix Validation**: Require valid GPS fix for RTL
4. **RTL Completion Logic**: Behavior when vehicle reaches home

**Technical deltas**:

- Create `src/rover/mode/rtl.rs`
- Add RTL to mode exports in `src/rover/mode/mod.rs`
- Implement `Mode` trait for RTL
- Add RTL entry validation in mode switching logic
- Add distance-to-home telemetry

## Stakeholder Analysis

| Stakeholder      | Interest/Need                             | Impact | Priority |
| ---------------- | ----------------------------------------- | ------ | -------- |
| Operators        | Reliable return home on command or safety | High   | P0       |
| Failsafe System  | RTL action for battery/RC/GCS failsafe    | High   | P0       |
| Battery RTL      | Prerequisite for battery-triggered RTL    | High   | P0       |
| Mission Planning | RTL as mission command destination        | Medium | P1       |

## Research & Discovery

### User Feedback

RTL is a fundamental mode expected by all ArduPilot users:

- Most common failsafe action after Hold
- Required for battery failsafe to return vehicle safely
- Expected behavior when "return home" button pressed in GCS
- Must work reliably in any situation where GPS is available

### Competitive Analysis

**ArduPilot Rover RTL Mode**:

From [ArduPilot RTL Mode](https://ardupilot.org/rover/docs/rtl-mode.html):

```cpp
// From Rover/mode_rtl.cpp
bool ModeRTL::_enter()
{
    // refuse RTL if home has not been set
    if (!ahrs.home_is_set()) {
        return false;
    }

    // set destination to home
    if (!set_desired_location(ahrs.get_home())) {
        return false;
    }

    return true;
}

void ModeRTL::update()
{
    // run the navigation controller
    navigate_to_waypoint();

    // check if we have reached home
    if (reached_destination()) {
        // optionally disarm or switch to Hold
        if (g2.rtl_options & RTL_OPTIONS_DISARM_ON_ARRIVAL) {
            rover.arming.disarm(AP_Arming::Method::RTL_COMPLETED);
        } else {
            set_mode(mode_hold, ModeReason::RTL_COMPLETE);
        }
    }
}
```

**Key ArduPilot RTL Parameters**:

| Parameter   | Description                         | Default |
| ----------- | ----------------------------------- | ------- |
| RTL_SPEED   | RTL speed in m/s (0 = use WP_SPEED) | 0       |
| RTL_OPTIONS | Bitmask for RTL behavior            | 0       |

**RTL_OPTIONS Bitmask**:

| Bit | Description              |
| --- | ------------------------ |
| 0   | Disarm on arrival        |
| 1   | Continue mission on exit |

### Technical Investigation

**Existing Navigation Infrastructure**:

```rust
// src/subsystems/navigation/controller.rs
pub trait NavigationController {
    fn update(
        &mut self,
        current: &GpsPosition,
        target: &PositionTarget,
        dt: f32,
    ) -> NavigationOutput;
    fn reset(&mut self);
}

// SimpleNavigationController already implements this
pub struct SimpleNavigationController { ... }
```

**NavigationOutput provides**:

```rust
pub struct NavigationOutput {
    pub steering: f32,       // -1.0 to +1.0
    pub throttle: f32,       // 0.0 to 1.0
    pub distance_m: f32,     // Distance to target
    pub bearing_deg: f32,    // Bearing to target
    pub heading_error_deg: f32,
    pub at_target: bool,     // Within WP_RADIUS
}
```

**Proposed RTL Mode Implementation**:

```rust
// src/rover/mode/rtl.rs
use crate::communication::mavlink::state::{HomePosition, SYSTEM_STATE};
use crate::devices::gps::GpsPosition;
use crate::subsystems::navigation::{
    NavigationController, NavigationOutput, PositionTarget, SimpleNavigationController,
};

use super::Mode;

/// RTL (Return to Launch) Mode
///
/// Navigates the vehicle back to the stored home position using GPS.
/// Requires:
/// - Valid GPS fix (3D fix minimum)
/// - Home position set (via MAV_CMD_DO_SET_HOME or auto-set on first fix)
pub struct RtlMode {
    /// Navigation controller for steering/throttle calculation
    nav_controller: SimpleNavigationController,
    /// Target position (home)
    target: PositionTarget,
    /// Whether we've reached home
    arrived: bool,
}

impl RtlMode {
    /// Create a new RTL mode instance
    pub fn new() -> Self {
        Self {
            nav_controller: SimpleNavigationController::new(),
            target: PositionTarget::default(),
            arrived: false,
        }
    }

    /// Check if RTL can be entered
    ///
    /// Returns Ok if GPS is valid and home is set, Err otherwise.
    pub fn can_enter() -> Result<(), &'static str> {
        // Check GPS fix
        let state = SYSTEM_STATE.lock();
        if !state.gps.has_fix() {
            return Err("RTL requires GPS fix");
        }

        // Check home position
        if state.home_position.is_none() {
            return Err("RTL requires home position");
        }

        Ok(())
    }

    /// Get current navigation output for telemetry
    pub fn navigation_status(&self) -> Option<&NavigationOutput> {
        // Could store last output for telemetry
        None
    }
}

impl Mode for RtlMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        // Validate entry conditions
        Self::can_enter()?;

        // Get home position
        let state = SYSTEM_STATE.lock();
        let home = state.home_position.ok_or("Home position not set")?;

        // Set target to home
        self.target = PositionTarget::new(home.latitude, home.longitude);
        self.arrived = false;

        // Reset navigation controller
        self.nav_controller.reset();

        crate::log_info!("RTL: Navigating to home ({:.6}, {:.6})",
            home.latitude, home.longitude);

        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        if self.arrived {
            // Already at home, just maintain position (stop)
            // TODO: Apply zero throttle/steering to actuators
            return Ok(());
        }

        // Get current GPS position
        let state = SYSTEM_STATE.lock();
        let gps = state.gps.position().ok_or("GPS fix lost")?;

        // Run navigation controller
        let output = self.nav_controller.update(&gps, &self.target, dt);

        // Check if arrived at home
        if output.at_target {
            self.arrived = true;
            crate::log_info!("RTL: Arrived at home");
            // TODO: Transition to Hold or Disarm based on RTL_OPTIONS
        }

        // TODO: Apply steering and throttle to actuators
        // actuators.set_steering(output.steering);
        // actuators.set_throttle(output.throttle);

        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        // Stop the vehicle
        // TODO: Set actuators to neutral
        crate::log_info!("RTL: Exiting RTL mode");
        Ok(())
    }

    fn name(&self) -> &'static str {
        "RTL"
    }
}
```

**Integration with Mode Switching**:

Mode entry validation should check RTL prerequisites:

```rust
// In mode switching logic
fn switch_to_mode(new_mode: FlightMode) -> Result<(), &'static str> {
    match new_mode {
        FlightMode::Rtl => {
            RtlMode::can_enter()?;
            // ... enter RTL mode
        }
        // ... other modes
    }
    Ok(())
}
```

**Memory Analysis**:

| Component                  | RAM Usage  | Notes                          |
| -------------------------- | ---------- | ------------------------------ |
| RtlMode struct             | \~32 B     | NavController + target + flags |
| SimpleNavigationController | \~16 B     | Config struct                  |
| PositionTarget             | \~12 B     | lat/lon/alt                    |
| **Total**                  | **\~60 B** | Minimal overhead               |

### Data Analysis

**RTL Performance Requirements**:

- Update rate: 50 Hz (matches control loop)
- GPS update rate: 1-10 Hz (hardware dependent)
- Navigation recalculation: On each GPS update
- Steering response: Within 1 control cycle (20ms)

**Typical RTL Scenarios**:

1. **Manual RTL**: Operator triggers RTL from GCS
2. **Failsafe RTL**: Battery/RC/GCS loss triggers RTL
3. **Mission RTL**: Mission item commands RTL
4. **Emergency RTL**: Critical condition triggers immediate return

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: System shall navigate vehicle to home position when RTL mode is activated
  - Rationale: Core RTL functionality
  - Acceptance Criteria:
    - Use SimpleNavigationController for steering/throttle
    - Navigate toward stored HomePosition
    - Stop when within WP_RADIUS of home

- [ ] **FR-DRAFT-2**: System shall validate GPS fix and home position before entering RTL mode
  - Rationale: RTL cannot function without these prerequisites
  - Acceptance Criteria:
    - Reject RTL entry if GPS fix is not 3D or better
    - Reject RTL entry if home_position is None
    - Return clear error message indicating failure reason

- [ ] **FR-DRAFT-3**: System shall stop vehicle when RTL reaches home position
  - Rationale: Define behavior at RTL completion
  - Acceptance Criteria:
    - Detect arrival when distance < WP_RADIUS (default 2m)
    - Set throttle to 0, steering to neutral
    - Log "RTL: Arrived at home" message
    - Optionally transition to Hold mode

- [ ] **FR-DRAFT-4**: System shall handle GPS loss during RTL navigation
  - Rationale: Safety during RTL execution
  - Acceptance Criteria:
    - Detect when GPS fix is lost (no fix or <3D)
    - Transition to Hold mode (stop in place)
    - Send STATUSTEXT: "RTL: GPS lost, entering Hold"

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: RTL navigation shall update at control loop rate (50 Hz)
  - Category: Performance
  - Rationale: Smooth navigation requires consistent updates
  - Target: update() called every 20ms

- [ ] **NFR-DRAFT-2**: RTL mode shall add no more than 100 bytes RAM overhead
  - Category: Resource Constraints
  - Rationale: Limited RAM on RP2040/RP2350
  - Target: \~60 B estimated

- [ ] **NFR-DRAFT-3**: RTL entry validation shall complete within 1ms
  - Category: Performance
  - Rationale: Fast mode switching required
  - Target: can_enter() check < 1ms

### SmartRTL Requirements (Potential)

- [ ] **FR-DRAFT-5**: System shall continuously record vehicle path during armed operation
  - Rationale: SmartRTL requires recorded path to retrace
  - Acceptance Criteria:
    - Record GPS position at configurable interval
    - Store points in ring buffer (SRTL_POINTS max)
    - Send "SmartRTL low on space" when buffer nearly full
    - Path recording active when armed and GPS valid

- [ ] **FR-DRAFT-6**: System shall simplify recorded path using SRTL_ACCURACY parameter
  - Rationale: Reduce memory usage while maintaining path accuracy
  - Acceptance Criteria:
    - Apply line simplification within SRTL_ACCURACY meters
    - Prune loops from recorded path
    - Maintain path integrity for safe return

- [ ] **FR-DRAFT-7**: System shall navigate along recorded path in reverse when SmartRTL activated
  - Rationale: Core SmartRTL functionality - retrace safe path
  - Acceptance Criteria:
    - Follow recorded waypoints in reverse order
    - Use SimpleNavigationController for each segment
    - Continue until home position reached

- [ ] **FR-DRAFT-8**: System shall fall back to direct RTL when SmartRTL path unavailable
  - Rationale: Ensure return capability even without recorded path
  - Acceptance Criteria:
    - Detect when path buffer empty or corrupted
    - Automatically switch to direct RTL mode
    - Send STATUSTEXT: "SmartRTL: No path, using direct RTL"

- [ ] **NFR-DRAFT-4**: SmartRTL path buffer shall use no more than 10KB RAM
  - Category: Resource Constraints
  - Rationale: Limited RAM on embedded targets
  - Target: 300 points default (\~9KB), 500 points max (\~15KB)

## Design Considerations

### Technical Constraints

- **GPS dependency**: RTL requires valid GPS fix throughout navigation
- **Home position required**: Must be set before RTL can be used
- **Actuator interface**: Need access to steering/throttle outputs
- **Mode trait compliance**: Must implement enter/update/exit lifecycle

### Potential Approaches

1. **Option A: Simple Direct RTL** (Recommended)
   - Pros:
     - Uses existing SimpleNavigationController
     - Minimal new code (\~100 LOC)
     - Proven navigation algorithms
     - Quick to implement
   - Cons:
     - No speed control (uses WP_SPEED)
     - No obstacle avoidance
   - Effort: Low (4-8 hours)

2. **Option B: RTL with Speed Parameter**
   - Pros:
     - Configurable RTL speed via RTL_SPEED parameter
     - More control over return behavior
   - Cons:
     - Additional parameter management
     - Slightly more complex
   - Effort: Low-Medium (8-12 hours)

3. **Option C: Full ArduPilot RTL Feature Parity with SmartRTL** (Selected)
   - Pros:
     - RTL_OPTIONS bitmask (disarm on arrival, etc.)
     - RTL_SPEED parameter
     - SmartRTL as default mode - retraces recorded path
     - Safer return in environments with obstacles
     - Falls back to direct RTL when path unavailable
   - Cons:
     - More complex implementation
     - Requires path recording infrastructure
     - Higher memory usage for path storage (\~3KB per 100 points)
   - Effort: Medium-High (24-32 hours)

**Recommendation**: Option C with SmartRTL as the default RTL behavior. SmartRTL provides safer return navigation by retracing the recorded path rather than attempting direct-line navigation. Direct RTL serves as fallback when SmartRTL path is unavailable.

### Architecture Impact

**New files**:

- `src/rover/mode/rtl.rs` - Direct RTL mode implementation (fallback)
- `src/rover/mode/smartrtl.rs` - SmartRTL mode implementation (default)
- `src/subsystems/navigation/path_recorder.rs` - Path recording and simplification

**Modified files**:

- `src/rover/mode/mod.rs` - Export RtlMode and SmartRtlMode
- `src/communication/mavlink/state.rs` - Add SmartRTL path buffer state

**ADR required**: New ADR for SmartRTL path recording architecture and memory management.

## Risk Assessment

| Risk                                    | Probability | Impact | Mitigation Strategy                        |
| --------------------------------------- | ----------- | ------ | ------------------------------------------ |
| GPS loss during RTL                     | Medium      | High   | Transition to Hold mode on GPS loss        |
| Home position not set                   | Medium      | High   | Validate home exists before RTL entry      |
| Navigation oscillation near home        | Low         | Medium | Use appropriate WP_RADIUS (default 2m)     |
| RTL activated too far from home         | Low         | Medium | No mitigation needed (RTL is valid action) |
| Vehicle cannot reach home (obstruction) | Low         | Medium | Operator must override manually            |

## Open Questions

- [x] What happens when RTL reaches home? → Decision: Stop in place (Hold behavior)
- [x] Should RTL auto-disarm on arrival? → Decision: No, stay armed in Hold state (safer)
- [x] Should RTL use dedicated speed parameter? → Decision: Yes, RTL_SPEED parameter (0 = use WP_SPEED)
- [x] Should RTL integrate with SmartRTL? → Decision: SmartRTL is default, direct RTL is fallback
- [x] How many path points to store? → Decision: SRTL_POINTS parameter (default 300, max 500)

## Recommendations

### Immediate Actions

1. **Create path recording infrastructure** in `src/subsystems/navigation/path_recorder.rs`
2. **Create `src/rover/mode/smartrtl.rs`** with SmartRTL as default mode
3. **Create `src/rover/mode/rtl.rs`** with direct RTL as fallback
4. **Add entry validation** checking GPS fix, home position, and path availability
5. **Implement path simplification** using SRTL_ACCURACY parameter

### Next Steps

1. [x] Create formal requirements for RTL and SmartRTL
2. [ ] Create ADR for SmartRTL path recording architecture
3. [ ] Create task for: Implement SmartRTL mode with path recording
4. [ ] Test SmartRTL with GPS simulator
5. [ ] Implement RTL_SPEED and RTL_OPTIONS parameters
6. [ ] Add SRTL_POINTS and SRTL_ACCURACY parameters

### Out of Scope

- **Obstacle avoidance**: No sensor input for obstacles (SmartRTL provides path-based safety)
- **RTL altitude management**: Not applicable for rovers
- **Rally points**: ArduPilot feature for alternate return locations

## Appendix

### References

- ArduPilot Rover RTL Mode: <https://ardupilot.org/rover/docs/rtl-mode.html>
- ArduPilot SmartRTL Mode: <https://ardupilot.org/rover/docs/smartrtl-mode.html>
- ArduPilot RTL Parameters: <https://ardupilot.org/rover/docs/parameters.html#rtl-parameters>
- ArduPilot SRTL Parameters: <https://ardupilot.org/rover/docs/parameters.html#srtl-parameters>
- Project navigation controller: `src/subsystems/navigation/controller.rs`
- Home position handling: `src/communication/mavlink/state.rs:213`

### Raw Data

**Existing Mode Implementations**:

```
Manual mode: src/rover/mode/manual.rs (~150 LOC)
Circle mode: src/rover/mode/circle.rs (~200 LOC)
Loiter mode: src/rover/mode/loiter.rs (~180 LOC)
```

**RTL Mode Estimated Size**: \~100-150 LOC

**SimpleNavigationController Usage Example** (from circle.rs):

```rust
let output = self.nav_controller.update(&current_pos, &target, dt);
// output.steering, output.throttle, output.at_target
```
