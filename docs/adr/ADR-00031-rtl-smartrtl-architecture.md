# ADR-00031 RTL and SmartRTL Mode Architecture

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
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
- Related Analyses:
  - [AN-00036-rtl-mode](../analysis/AN-00036-rtl-mode.md)
- Related ADRs:
  - [ADR-00013-control-mode-architecture](ADR-00013-control-mode-architecture.md)
  - [ADR-00022-navigation-controller-architecture](ADR-00022-navigation-controller-architecture.md)
- Related Tasks:
  - [T-00030-rtl-smartrtl-implementation](../tasks/T-00030-rtl-smartrtl-implementation/README.md)

## Context

The pico_trail rover needs RTL (Return to Launch) capability - the ability to autonomously navigate back to its launch point. Currently, `FlightMode::Rtl` exists as an enum value but has no implementation.

### Problem

We need an architecture that:

- Provides reliable navigation back to home position
- Supports both direct RTL (straight-line) and SmartRTL (path-retrace) modes
- Records vehicle path during operation for SmartRTL
- Manages path memory efficiently on embedded targets
- Integrates with the existing Mode trait and ModeManager
- Handles GPS loss gracefully during RTL navigation

### Constraints

- **Memory Budget**: SmartRTL path storage ≤ 10 KB RAM (NFR-00086)
- **Direct RTL overhead**: ≤ 100 bytes RAM (NFR-00084)
- **Real-time**: Navigation update at 50 Hz (NFR-00085)
- **Entry validation**: ≤ 1ms (NFR-00083)
- **no_std**: Must work without standard library

### Prior Art

**ArduPilot RTL Mode** (from Rover/mode_rtl.cpp):

```cpp
bool ModeRTL::_enter() {
    if (!ahrs.home_is_set()) {
        return false;
    }
    if (!set_desired_location(ahrs.get_home())) {
        return false;
    }
    return true;
}

void ModeRTL::update() {
    navigate_to_waypoint();
    if (reached_destination()) {
        set_mode(mode_hold, ModeReason::RTL_COMPLETE);
    }
}
```

**ArduPilot SmartRTL** (from Rover/mode_smartrtl.cpp):

- Records breadcrumb path during operation
- Simplifies path using SRTL_ACCURACY parameter
- Follows path in reverse order during SmartRTL
- Falls back to direct RTL when path unavailable

## Success Metrics

- **Reliability**: 100% successful RTL when GPS and home are valid
- **Memory**: SmartRTL path storage < 10 KB (300 points × \~30 bytes)
- **Performance**: Navigation update < 1ms at 50 Hz
- **Safety**: Graceful degradation on GPS loss (transition to Hold)

## Decision

**We will implement a dual-mode RTL architecture with SmartRTL as the default and direct RTL as fallback, using a shared path recording infrastructure.**

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Mode Manager                              │
│                 (selects active mode)                        │
└───────────────┬─────────────────────────────────────────────┘
                │
        ┌───────┴───────┐
        │               │
┌───────▼──────┐ ┌──────▼───────┐
│  SmartRTL    │ │  Direct RTL  │
│  (default)   │ │  (fallback)  │
│  - Follow    │ │  - Straight  │
│    path in   │ │    line to   │
│    reverse   │ │    home      │
└───────┬──────┘ └──────┬───────┘
        │               │
        └───────┬───────┘
                │
┌───────────────▼─────────────────────────────────────────────┐
│              SimpleNavigationController                      │
│         (steering/throttle calculation)                      │
└───────────────┬─────────────────────────────────────────────┘
                │
┌───────────────▼─────────────────────────────────────────────┐
│                   Path Recorder                              │
│  - Records GPS positions during armed operation              │
│  - Ring buffer with SRTL_POINTS capacity                     │
│  - Path simplification using SRTL_ACCURACY                   │
└─────────────────────────────────────────────────────────────┘
```

### Decision Drivers

1. **Safety**: SmartRTL retraces known-safe path
2. **ArduPilot Compatibility**: Same behavior as ArduPilot Rover
3. **Memory Efficiency**: Fixed-size ring buffer, no heap allocation
4. **Graceful Degradation**: Fallback to direct RTL when path unavailable
5. **Reuse**: Leverages existing SimpleNavigationController

### Considered Options

- **Option A: Direct RTL Only**
- **Option B: SmartRTL Only**
- **Option C: SmartRTL with Direct RTL Fallback** ⭐ Selected

### Option Analysis

**Option A: Direct RTL Only**

- **Pros**:
  - Simple implementation (\~100 LOC)
  - Minimal memory overhead (\~60 bytes)
  - No path recording complexity
- **Cons**:
  - May navigate through obstacles
  - Less safe in complex environments
  - Missing expected ArduPilot feature
- **Estimated Overhead**: \~60 B RAM, \~2 KB Flash

**Option B: SmartRTL Only**

- **Pros**:
  - Safe path-retrace navigation
  - Avoids obstacles already navigated
- **Cons**:
  - Cannot return if path buffer empty (first arm)
  - Higher memory usage (\~10 KB)
  - No fallback on path corruption
- **Estimated Overhead**: \~10 KB RAM, \~4 KB Flash

**Option C: SmartRTL with Direct RTL Fallback** ⭐ Selected

- **Pros**:
  - Safe path-retrace as default
  - Always-available fallback
  - Matches ArduPilot behavior
  - Graceful degradation
- **Cons**:
  - More complex implementation
  - Higher memory than Option A
- **Estimated Overhead**: \~10 KB RAM, \~6 KB Flash

## Rationale

**SmartRTL with Direct RTL Fallback** was selected because:

1. **Safety First**: SmartRTL retraces known-safe path, avoiding obstacles
2. **Reliability**: Direct RTL ensures return capability even without recorded path
3. **ArduPilot Parity**: Matches expected behavior for ArduPilot users
4. **Graceful Degradation**: Path corruption or empty buffer triggers fallback

### Trade-offs Accepted

- **Memory Cost**: \~10 KB for path buffer (acceptable within constraints)
- **Complexity**: Two mode implementations (manageable with shared infrastructure)

## Consequences

### Positive

- **Safety**: Safe path-retrace in complex environments
- **Reliability**: Always-available return capability
- **User Expectation**: Matches ArduPilot behavior
- **Testability**: Direct RTL can be tested without path recording

### Negative

- **Memory Usage**: \~10 KB for path recording
- **Complexity**: Two mode implementations to maintain
- **Recording Overhead**: Continuous path recording during armed operation

### Neutral

- **ArduPilot Compatibility**: Same parameters (SRTL_POINTS, SRTL_ACCURACY)
- **Integration**: Uses existing Mode trait and ModeManager

## Implementation Notes

### Module Structure

```
src/
├── rover/
│   └── mode/
│       ├── mod.rs          # Mode trait, exports
│       ├── rtl.rs          # Direct RTL mode (fallback)
│       └── smartrtl.rs     # SmartRTL mode (default)
│
└── subsystems/
    └── navigation/
        ├── mod.rs
        ├── controller.rs   # SimpleNavigationController (existing)
        └── path_recorder.rs # Path recording and simplification (new)
```

### Path Recorder

```rust
/// Path point stored in ring buffer
pub struct PathPoint {
    pub latitude: f64,   // degrees
    pub longitude: f64,  // degrees
    pub timestamp: u32,  // ms since arm
}
// Size: ~20 bytes per point

/// Path recorder for SmartRTL
pub struct PathRecorder {
    /// Ring buffer of recorded points
    buffer: [PathPoint; SRTL_POINTS_MAX],
    /// Number of valid points
    count: usize,
    /// Write index (ring buffer head)
    write_idx: usize,
    /// Last recorded position
    last_point: Option<PathPoint>,
    /// Recording active
    recording: bool,
}

impl PathRecorder {
    /// Start recording (called on arm)
    pub fn start(&mut self) {
        self.buffer = [PathPoint::default(); SRTL_POINTS_MAX];
        self.count = 0;
        self.write_idx = 0;
        self.recording = true;
    }

    /// Stop recording (called on disarm)
    pub fn stop(&mut self) {
        self.recording = false;
    }

    /// Record a point if distance/time threshold met
    pub fn record(&mut self, position: &GpsPosition, timestamp_ms: u32) {
        if !self.recording {
            return;
        }

        // Check if we should record (distance or time threshold)
        if let Some(last) = &self.last_point {
            let distance = calculate_distance(last, position);
            let time_delta = timestamp_ms - last.timestamp;

            if distance < SRTL_MIN_DISTANCE_M && time_delta < SRTL_MIN_TIME_MS {
                return; // Too close, skip
            }
        }

        // Record point
        self.buffer[self.write_idx] = PathPoint {
            latitude: position.latitude,
            longitude: position.longitude,
            timestamp: timestamp_ms,
        };

        self.write_idx = (self.write_idx + 1) % SRTL_POINTS_MAX;
        if self.count < SRTL_POINTS_MAX {
            self.count += 1;
        }

        // Check low space warning (90%)
        if self.count >= SRTL_POINTS_MAX * 9 / 10 {
            // Send STATUSTEXT: "SmartRTL low on space"
        }
    }

    /// Get path for return navigation (reverse order)
    pub fn get_return_path(&self) -> impl Iterator<Item = &PathPoint> {
        // Return points in reverse order for path-retrace
        self.buffer[..self.count].iter().rev()
    }

    /// Check if path is available
    pub fn has_path(&self) -> bool {
        self.count > 0
    }
}
```

### Direct RTL Mode

```rust
/// Direct RTL Mode - Navigate straight to home
pub struct RtlMode {
    nav_controller: SimpleNavigationController,
    target: PositionTarget,
    arrived: bool,
}

impl RtlMode {
    pub fn new() -> Self {
        Self {
            nav_controller: SimpleNavigationController::new(),
            target: PositionTarget::default(),
            arrived: false,
        }
    }

    /// Check if direct RTL can be entered
    pub fn can_enter() -> Result<(), &'static str> {
        let state = SYSTEM_STATE.lock();
        if !state.gps.has_fix() {
            return Err("RTL requires GPS fix");
        }
        if state.home_position.is_none() {
            return Err("RTL requires home position");
        }
        Ok(())
    }
}

impl Mode for RtlMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        Self::can_enter()?;

        let state = SYSTEM_STATE.lock();
        let home = state.home_position.ok_or("Home not set")?;

        self.target = PositionTarget::new(home.latitude, home.longitude);
        self.arrived = false;
        self.nav_controller.reset();

        crate::log_info!("RTL: Navigating to home");
        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        if self.arrived {
            return Ok(()); // Maintain stop
        }

        let state = SYSTEM_STATE.lock();
        let gps = state.gps.position().ok_or("GPS fix lost")?;

        let output = self.nav_controller.update(&gps, &self.target, dt);

        if output.at_target {
            self.arrived = true;
            crate::log_info!("RTL: Arrived at home");
            // TODO: Apply zero throttle/steering
        }

        // TODO: Apply output.steering, output.throttle to actuators
        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("RTL: Exiting");
        Ok(())
    }

    fn name(&self) -> &'static str {
        "RTL"
    }
}
```

### SmartRTL Mode

```rust
/// SmartRTL Mode - Retrace recorded path to home
pub struct SmartRtlMode {
    nav_controller: SimpleNavigationController,
    /// Current waypoint index in return path
    current_waypoint_idx: usize,
    /// Total waypoints in return path
    total_waypoints: usize,
    /// Current target
    current_target: PositionTarget,
    /// Arrived at final destination
    arrived: bool,
}

impl SmartRtlMode {
    pub fn new() -> Self {
        Self {
            nav_controller: SimpleNavigationController::new(),
            current_waypoint_idx: 0,
            total_waypoints: 0,
            current_target: PositionTarget::default(),
            arrived: false,
        }
    }

    /// Check if SmartRTL can be entered
    pub fn can_enter() -> Result<(), &'static str> {
        // First check GPS and home
        RtlMode::can_enter()?;

        // Then check path availability
        let path = PATH_RECORDER.lock();
        if !path.has_path() {
            return Err("SmartRTL requires recorded path");
        }
        Ok(())
    }

    /// Load next waypoint from recorded path
    fn load_next_waypoint(&mut self) -> bool {
        let path = PATH_RECORDER.lock();
        if let Some(point) = path.get_point(self.current_waypoint_idx) {
            self.current_target = PositionTarget::new(point.latitude, point.longitude);
            true
        } else {
            false
        }
    }
}

impl Mode for SmartRtlMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        Self::can_enter()?;

        let path = PATH_RECORDER.lock();
        self.total_waypoints = path.count();
        self.current_waypoint_idx = 0;
        self.arrived = false;
        self.nav_controller.reset();

        drop(path); // Release lock before load_next_waypoint
        self.load_next_waypoint();

        crate::log_info!("SmartRTL: Retracing {} waypoints", self.total_waypoints);
        Ok(())
    }

    fn update(&mut self, dt: f32) -> Result<(), &'static str> {
        if self.arrived {
            return Ok(());
        }

        let state = SYSTEM_STATE.lock();
        let gps = state.gps.position().ok_or("GPS fix lost")?;
        drop(state);

        let output = self.nav_controller.update(&gps, &self.current_target, dt);

        if output.at_target {
            self.current_waypoint_idx += 1;

            if self.current_waypoint_idx >= self.total_waypoints {
                self.arrived = true;
                crate::log_info!("SmartRTL: Arrived at home");
            } else {
                self.load_next_waypoint();
                crate::log_info!("SmartRTL: Waypoint {}/{}",
                    self.current_waypoint_idx, self.total_waypoints);
            }
        }

        // TODO: Apply output.steering, output.throttle to actuators
        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        crate::log_info!("SmartRTL: Exiting");
        Ok(())
    }

    fn name(&self) -> &'static str {
        "SmartRTL"
    }
}
```

### RTL Mode Selection Logic

```rust
/// Select appropriate RTL mode based on path availability
pub fn select_rtl_mode() -> Box<dyn Mode> {
    // Try SmartRTL first (preferred)
    if SmartRtlMode::can_enter().is_ok() {
        crate::log_info!("RTL: Using SmartRTL (path available)");
        return Box::new(SmartRtlMode::new());
    }

    // Fall back to direct RTL
    if RtlMode::can_enter().is_ok() {
        crate::log_info!("RTL: Using direct RTL (no path)");
        return Box::new(RtlMode::new());
    }

    // Neither available - this should be caught by validation
    panic!("RTL mode requested but neither SmartRTL nor direct RTL available");
}
```

### GPS Loss Handling

```rust
// In RTL/SmartRTL update()
fn update(&mut self, dt: f32) -> Result<(), &'static str> {
    let state = SYSTEM_STATE.lock();

    // Check GPS validity
    if !state.gps.has_fix() {
        // GPS lost - mode manager should transition to Hold
        return Err("GPS fix lost");
    }

    // ... continue navigation
}

// In ModeManager - handle GPS loss during RTL
impl ModeManager {
    pub fn handle_mode_error(&mut self, error: &str) {
        if error == "GPS fix lost" {
            crate::log_warn!("RTL: GPS lost, switching to Hold");
            // Send STATUSTEXT
            self.set_mode(Box::new(HoldMode::new()));
        }
    }
}
```

### Memory Layout

| Component              | RAM Usage     | Notes                     |
| ---------------------- | ------------- | ------------------------- |
| PathRecorder buffer    | \~9,000 B     | 300 points × 30 bytes     |
| PathRecorder state     | \~24 B        | Indices, flags            |
| SmartRtlMode           | \~48 B        | Nav controller + state    |
| RtlMode                | \~32 B        | Nav controller + target   |
| **Total (SmartRTL)**   | **\~9,100 B** | Within 10 KB budget       |
| **Total (Direct RTL)** | **\~60 B**    | Minimal fallback overhead |

### ArduPilot Parameter Mapping

| ArduPilot Parameter | Default | pico_trail Equivalent          |
| ------------------- | ------- | ------------------------------ |
| SRTL_POINTS         | 300     | `SRTL_POINTS_MAX` constant     |
| SRTL_ACCURACY       | 2.0     | `SRTL_ACCURACY_M` constant     |
| RTL_SPEED           | 0       | Use WP_SPEED (not implemented) |

**Note**: These are implemented as compile-time constants initially. Parameter system integration is deferred.

## Platform Considerations

- **Platform Agnostic**: RTL modes use abstract interfaces (NavigationController, Mode trait)
- **Memory**: Fixed-size buffers, no heap allocation
- **RP2040/RP2350**: Path buffer (\~10 KB) fits within available RAM

## Monitoring & Logging

- **Mode Transitions**: Log all RTL mode entries/exits
- **Waypoint Progress**: Log SmartRTL waypoint progression
- **GPS Loss**: Log and send STATUSTEXT on GPS loss
- **Path Recording**: Log buffer space warnings

## Open Questions

- [x] SmartRTL or direct RTL as default? → Decision: SmartRTL default, direct RTL fallback
- [x] Path simplification algorithm? → Decision: Distance-based simplification using SRTL_ACCURACY
- [ ] Should RTL_SPEED parameter be implemented? → Next step: Defer to future enhancement
- [ ] Path persistence across disarm? → Decision: Clear on arm (ArduPilot behavior)

## External References

- [ArduPilot Rover RTL Mode](https://ardupilot.org/rover/docs/rtl-mode.html)
- [ArduPilot SmartRTL Mode](https://ardupilot.org/rover/docs/smartrtl-mode.html)
- [ArduPilot SRTL Parameters](https://ardupilot.org/rover/docs/parameters.html#srtl-parameters)
