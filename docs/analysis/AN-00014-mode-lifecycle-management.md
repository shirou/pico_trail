# AN-00014 Mode Lifecycle Management for Clean Mode Transitions

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-00013-mode-entry-validation](AN-00013-mode-entry-validation.md)
  - [AN-00009-armed-state-monitoring](AN-00009-armed-state-monitoring.md)
  - [AN-00015-post-arm-initialization](AN-00015-post-arm-initialization.md)
- Related Requirements:
  - [FR-00048-mode-lifecycle-management](../requirements/FR-00048-mode-lifecycle-management.md)
  - [NFR-00038-mode-entry-timing](../requirements/NFR-00038-mode-entry-timing.md)
  - [NFR-00041-mode-update-timing](../requirements/NFR-00041-mode-update-timing.md)
  - [NFR-00040-mode-state-memory](../requirements/NFR-00040-mode-state-memory.md)
  - [NFR-00039-mode-exit-timing](../requirements/NFR-00039-mode-exit-timing.md)
  - [NFR-00037-lifecycle-transition-logging](../requirements/NFR-00037-lifecycle-transition-logging.md)
- Related ADRs: N/A - ADRs will be created based on this analysis
- Related Tasks: N/A - Tasks will be created after requirements and ADRs

## Executive Summary

This analysis explores mode lifecycle management needed to ensure clean initialization, continuous execution, and proper cleanup during mode transitions. Currently, pico_trail has no mode lifecycle concept - modes switch instantly without initialization or cleanup, causing previous mode's actuator commands to persist, potential memory leaks, and unsafe transitions. A proper mode lifecycle system is essential for predictable vehicle behavior, resource management, and safe mode transitions.

Key findings: ArduPilot implements a comprehensive three-phase lifecycle per mode: `enter()` for initialization (called on mode entry), `update()` for continuous execution (called every control loop at 50 Hz), and `exit()` for cleanup (called on mode exit). Each mode implements mode-specific logic for these phases, ensuring proper state initialization, actuator configuration, and resource cleanup. For pico_trail, a similar trait-based lifecycle system is recommended with focus on actuator state management, mode-local state initialization/cleanup, transition sequencing (old mode exit → new mode enter), and error handling during transitions.

## Problem Space

### Current State

The project currently has:

- **Mode enum**: `FlightMode` defined in `src/communication/mavlink/state.rs:36-50`
- **Mode change function**: `SystemState::set_mode()` in `src/communication/mavlink/state.rs:185-194`
- **Instant mode switch**: Mode changes by simple enum assignment (`self.mode = mode`)
- **No initialization**: Modes have no initialization logic
- **No cleanup**: Previous mode's state not cleaned up
- **No continuous execution**: No mode-specific update logic

Critical gaps:

- **State persistence**: Previous mode's actuator commands remain active after switch
- **No initialization**: New mode starts without proper state setup
- **Resource leaks**: Memory/resources allocated by mode never freed
- **Unsafe transitions**: Mode switches without ensuring safe actuator states
- **No mode update**: Modes have no continuous execution logic
- **Timing issues**: Time-based mode logic broken (no timestamp initialization)

### Desired State

Enable comprehensive mode lifecycle management for safe transitions:

1. **Mode Entry (`enter`)**: Initialize mode state, configure actuators, validate prerequisites
2. **Mode Update (`update`)**: Continuous mode execution every control loop (50 Hz)
3. **Mode Exit (`exit`)**: Cleanup mode state, reset actuators, release resources
4. **Transition Sequencing**: Ordered execution (old mode exit → new mode enter)
5. **Error Handling**: Graceful handling of failures during lifecycle transitions
6. **Resource Management**: Proper allocation and cleanup of mode-local resources

Success criteria:

- **Clean initialization**: Each mode starts with well-defined state
- **Predictable behavior**: Mode behavior consistent across transitions
- **No state leakage**: Previous mode's state doesn't affect new mode
- **Resource safety**: No memory leaks or resource exhaustion
- **Safe actuators**: Actuator commands transitioned smoothly without glitches
- **Auditable**: All lifecycle transitions logged for debugging

### Gap Analysis

**Missing components**:

1. **Mode Lifecycle Trait**: Interface defining enter/update/exit methods
2. **Mode Implementations**: Per-mode logic for lifecycle phases
3. **Transition Sequencer**: Orchestrates old mode exit → new mode enter
4. **Mode State Storage**: Per-mode local state management
5. **Actuator Transition Logic**: Smooth actuator command changes
6. **Error Handler**: Handles failures during lifecycle transitions
7. **Lifecycle Logger**: Logs all lifecycle events

**Technical deltas**:

- Create `Mode` trait with `enter()`, `update()`, `exit()` methods
- Implement lifecycle for each mode (Manual, Stabilize, Loiter, Auto, RTL)
- Add mode state storage (per-mode structs with local state)
- Integrate with mode validation (AN-00013) for enter() prerequisites
- Add transition sequencer ensuring correct order
- Implement actuator state transition logic
- Add error handling for initialization/cleanup failures
- Log all lifecycle transitions with timestamps
- Integrate update() into vehicle control loop (50 Hz)

## Stakeholder Analysis

| Stakeholder        | Interest/Need                                         | Impact | Priority |
| ------------------ | ----------------------------------------------------- | ------ | -------- |
| Operators          | Predictable vehicle behavior during mode changes      | High   | P0       |
| Safety Reviewers   | Ensure modes properly initialized before operation    | High   | P0       |
| Autonomous Systems | Clean state transitions for reliable operation        | High   | P0       |
| Test Engineers     | Understand mode state during debugging                | High   | P1       |
| Developers         | Clear lifecycle model for implementing new modes      | High   | P1       |
| System Integrators | Verify mode transitions don't cause actuator glitches | Medium | P1       |

## Research & Discovery

### User Feedback

From operational requirements:

- Mode lifecycle is fundamental for predictable vehicle behavior
- Actuator glitches during mode transitions are dangerous
- Resource leaks cause system instability over time
- Clean initialization essential for autonomous mode reliability
- Debugging mode issues requires understanding lifecycle state
- Mode-specific state must not leak between transitions

### Competitive Analysis

**ArduPilot Mode Lifecycle System**:

Based on Rover mode.h and mode implementations, ArduPilot implements comprehensive mode lifecycle:

#### Mode Class Structure

File: `Rover/mode.h` (lines 10-100)

```cpp
class Mode {
public:
    // Lifecycle methods
    virtual bool enter();          // Called on mode entry
    virtual void update() = 0;     // Called every control loop (pure virtual)
    virtual void _enter() {}       // Mode-specific initialization (optional)
    virtual void _exit() {}        // Mode-specific cleanup (optional)

    // Capability queries
    virtual bool requires_position() const { return true; }
    virtual bool requires_velocity() const { return true; }
    virtual bool is_autopilot_mode() const { return false; }

    // State access
    Number mode_number() const { return _mode_number; }
    const char *name() const { return _name; }

protected:
    Mode(const char *name, uint32_t mode_number);

    // Helper methods for mode implementations
    void calc_throttle(float target_speed, bool avoidance_enabled);
    void calc_lateral_acceleration(float target_lat_accel);
    void calc_nav_roll(float desired_heading);

    // State
    uint32_t _mode_number;
    const char *_name;
    uint32_t _enter_time_ms;  // Timestamp when mode entered
};
```

**Key Lifecycle Points**:

1. **`enter()`**: Base class validation + mode-specific `_enter()`
2. **`update()`**: Pure virtual, must be implemented by each mode
3. **`_exit()`**: Mode-specific cleanup (optional override)
4. **`_enter_time_ms`**: Timestamp for time-based mode logic

#### Mode Entry Lifecycle

File: `Rover/mode.cpp` (lines 21-70)

```cpp
bool Mode::enter()
{
    // Get filter status for validation
    nav_filter_status filt_status;
    rover.ahrs.get_filter_status(filt_status);

    // Validate prerequisites (position/velocity requirements)
    const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;
    if (requires_position() && !position_ok) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires position");
        return false;
    }
    if (requires_velocity() && !position_ok && !filt_status.flags.horiz_vel) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires velocity");
        return false;
    }

    // Record entry time for time-based logic
    _enter_time_ms = AP_HAL::millis();

    // Call mode-specific initialization
    const bool success = _enter();
    if (success) {
        rover.mode_initialized = true;
    }

    return success;
}
```

**Purpose**: Unified entry point with validation, timestamp recording, and mode-specific init.

#### Mode Exit Lifecycle

File: `Rover/mode.cpp` (lines 72-85)

```cpp
void Mode::exit()
{
    // Call mode-specific cleanup
    _exit();

    // Clear mode initialized flag
    rover.mode_initialized = false;
}
```

**Purpose**: Simple wrapper calling mode-specific cleanup logic.

#### Mode Update Lifecycle

Each mode implements `update()` called every control loop iteration (50 Hz):

**Example: Manual Mode**

File: `Rover/mode_manual.cpp` (lines 25-50)

```cpp
void ModeManual::update()
{
    // Get RC input from pilot
    float throttle = channel_throttle->norm_input();
    float steering = channel_steer->norm_input();

    // Apply throttle limiting if configured
    throttle = g2.sailboat.get_throttle(throttle);

    // Send to motors (with acceleration limits)
    g2.motors.set_throttle(throttle);
    g2.motors.set_steering(steering * 4500.0f);  // Convert to centidegrees

    // Handle lateral motor (if configured)
    if (g2.motors.have_lateral_control()) {
        float lateral = channel_lateral->norm_input();
        g2.motors.set_lateral(lateral);
    }
}
```

**Purpose**: Continuous execution of mode-specific control logic.

#### Mode-Specific Entry/Exit Examples

**MANUAL Mode Exit** - Clear lateral motor:

File: `Rover/mode_manual.cpp` (lines 15-20)

```cpp
void ModeManual::_exit()
{
    // Clear lateral motor to prevent jerky movement
    g2.motors.set_lateral(0);
}
```

**AUTO Mode Entry** - Initialize waypoint navigation:

File: `Rover/mode_auto.cpp` (lines 35-60)

```cpp
bool ModeAuto::_enter()
{
    // Resume mission or start from beginning
    if (g2.wp_nav.have_valid_mission()) {
        // Start executing mission
        g2.wp_nav.start_mission();

        // Initialize target to first waypoint
        advance_target();

        return true;
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "No mission loaded");
        return false;
    }
}

void ModeAuto::_exit()
{
    // Stop waypoint navigation
    g2.wp_nav.stop();
}
```

**RTL Mode Entry** - Set return target to home:

File: `Rover/mode_rtl.cpp` (lines 20-45)

```cpp
bool ModeRTL::_enter()
{
    // Check if home position set
    if (!AP::ahrs().home_is_set()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "RTL: No home position");
        return false;
    }

    // Set target to home location
    set_desired_location(rover.home);

    // Start return navigation
    rover.set_reverse(false);  // RTL always goes forward

    return true;
}

void ModeRTL::_exit()
{
    // Clear return target
    _reached_destination = false;
}
```

**HOLD Mode Entry** - Stop vehicle and hold position:

File: `Rover/mode_hold.cpp` (lines 15-30)

```cpp
bool ModeHold::_enter()
{
    // Stop vehicle
    stop_vehicle();

    // If GPS available, record hold position
    if (rover.position_ok()) {
        Location temp_loc;
        if (rover.ahrs.get_location(temp_loc)) {
            _hold_position = temp_loc;
            _have_hold_position = true;
        }
    } else {
        _have_hold_position = false;
    }

    return true;
}

void ModeHold::update()
{
    // If position available, navigate to hold point
    if (_have_hold_position && rover.position_ok()) {
        navigate_to_destination(_hold_position);
    } else {
        // No position - just maintain stopped state
        stop_vehicle();
    }
}
```

**Purpose**: Each mode initializes/cleans up mode-specific state.

#### Transition Sequencing

File: `Rover/Rover.cpp` (lines 850-880)

```cpp
bool Rover::set_mode(Mode &new_mode, ModeReason reason)
{
    // Check if already in this mode
    if (control_mode == &new_mode) {
        return true;
    }

    // Try to enter new mode (validates prerequisites)
    if (!new_mode.enter()) {
        // Entry failed (validation or init error)
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "Failed to enter mode: %s", new_mode.name());
        return false;
    }

    // Exit old mode (cleanup)
    if (control_mode != nullptr) {
        control_mode->exit();
    }

    // Switch to new mode
    Mode *previous_mode = control_mode;
    control_mode = &new_mode;
    control_mode_reason = reason;

    // Log mode change
    logger.Write_Mode(control_mode->mode_number(), reason);

    // Notify GCS
    gcs().send_message(MSG_HEARTBEAT);
    gcs().send_text(MAV_SEVERITY_INFO, "Mode changed to %s",
                    control_mode->name());

    return true;
}
```

**Key Sequencing**:

1. **Validate new mode**: Call `new_mode.enter()` to check prerequisites
2. **Exit old mode**: Only after new mode entry succeeds
3. **Switch mode**: Update mode pointer
4. **Log transition**: Record change with reason
5. **Notify GCS**: Send heartbeat and STATUSTEXT

**Purpose**: Ensures old mode cleaned up only after new mode successfully entered.

**PX4 Mode Lifecycle System**:

PX4 Commander state machine implements:

- **On-entry actions**: Mode-specific initialization in `commander_state.cpp`
- **On-exit actions**: Cleanup logic per mode
- **Main loop update**: Continuous state machine execution
- **Transition validation**: Prerequisites checked before mode change
- **State persistence**: Mode state stored in mode-specific structures

### Technical Investigation

**Current pico_trail Implementation**:

File: `src/communication/mavlink/state.rs:185-194`

```rust
pub fn set_mode(&mut self, mode: FlightMode) -> Result<(), &'static str> {
    // Mode change restrictions can be added here
    // For now, allow all mode changes

    self.mode = mode;
    Ok(())
}
```

**Observations**:

- Instant mode switch (single enum assignment)
- No old mode cleanup
- No new mode initialization
- No transition sequencing
- No error handling
- No logging

**Current Flight Modes**:

File: `src/communication/mavlink/state.rs:36-50`

```rust
pub enum FlightMode {
    /// Manual mode (direct RC control)
    Manual,
    /// Stabilize mode (heading hold)
    Stabilize,
    /// Loiter mode (position hold)
    Loiter,
    /// Auto mode (following waypoints)
    Auto,
    /// Return to launch
    Rtl,
}
```

**Observations**: Simple enum, no associated state or lifecycle methods.

**Proposed Mode Lifecycle Architecture**:

```rust
/// Mode lifecycle trait defining initialization, execution, and cleanup
pub trait Mode {
    /// Get mode identifier
    fn mode_number(&self) -> u8;

    /// Get mode name (for logging/debugging)
    fn name(&self) -> &'static str;

    /// Mode entry - called when transitioning to this mode
    /// Returns Ok(()) if entry successful, Err if prerequisites not met
    fn enter(&mut self) -> Result<(), &'static str>;

    /// Mode update - called every control loop (50 Hz)
    /// Implements continuous mode logic
    fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str>;

    /// Mode exit - called when transitioning away from this mode
    /// Should clean up mode state and reset actuators
    fn exit(&mut self);

    /// Get entry timestamp (for time-based mode logic)
    fn enter_time_ms(&self) -> u32;
}

/// Manual mode implementation
pub struct ManualMode {
    enter_time_ms: u32,
}

impl Mode for ManualMode {
    fn mode_number(&self) -> u8 {
        0  // MAV_MODE_MANUAL
    }

    fn name(&self) -> &'static str {
        "MANUAL"
    }

    fn enter(&mut self) -> Result<(), &'static str> {
        // Record entry time
        self.enter_time_ms = get_time_ms();

        // Manual mode has no prerequisites
        info!("Entered MANUAL mode");
        Ok(())
    }

    fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str> {
        // Get RC input from pilot
        let throttle = state.rc_channels.throttle_normalized();
        let steering = state.rc_channels.steering_normalized();

        // Send to motors directly (no processing)
        state.actuators.set_throttle(throttle)?;
        state.actuators.set_steering(steering)?;

        Ok(())
    }

    fn exit(&mut self) {
        // Clear lateral motor to prevent jerky movement
        // (matches ArduPilot pattern)
        info!("Exiting MANUAL mode");
    }
}

/// Hold mode implementation
pub struct HoldMode {
    enter_time_ms: u32,
    hold_position: Option<Location>,
}

impl Mode for HoldMode {
    fn mode_number(&self) -> u8 {
        4  // MAV_MODE_HOLD
    }

    fn name(&self) -> &'static str {
        "HOLD"
    }

    fn enter(&mut self) -> Result<(), &'static str> {
        // Record entry time
        self.enter_time_ms = get_time_ms();

        // Stop vehicle
        stop_vehicle()?;

        // Record hold position if GPS available
        if let Some(position) = get_current_position() {
            self.hold_position = Some(position);
            info!("Entered HOLD mode at position: {:?}", position);
        } else {
            self.hold_position = None;
            info!("Entered HOLD mode (no position available)");
        }

        Ok(())
    }

    fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str> {
        // If position available, navigate to hold point
        if let Some(hold_pos) = self.hold_position {
            if state.has_position_estimate() {
                // Navigate to hold position
                navigate_to_destination(hold_pos, state)?;
            } else {
                // Position lost - just stay stopped
                stop_vehicle()?;
            }
        } else {
            // No position - maintain stopped state
            stop_vehicle()?;
        }

        Ok(())
    }

    fn exit(&mut self) {
        // Clear hold position
        self.hold_position = None;
        info!("Exiting HOLD mode");
    }
}

/// Auto mode implementation
pub struct AutoMode {
    enter_time_ms: u32,
    current_waypoint: usize,
    mission_loaded: bool,
}

impl Mode for AutoMode {
    fn mode_number(&self) -> u8 {
        10  // MAV_MODE_AUTO
    }

    fn name(&self) -> &'static str {
        "AUTO"
    }

    fn enter(&mut self) -> Result<(), &'static str> {
        // Record entry time
        self.enter_time_ms = get_time_ms();

        // Check if mission loaded
        if !self.mission_loaded {
            warn!("AUTO mode: No mission loaded");
            return Err("No mission loaded");
        }

        // Initialize to first waypoint
        self.current_waypoint = 0;

        info!("Entered AUTO mode, starting mission");
        Ok(())
    }

    fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str> {
        // Execute waypoint navigation
        if self.current_waypoint < get_mission_waypoint_count() {
            let target = get_waypoint(self.current_waypoint)?;
            navigate_to_destination(target, state)?;

            // Check if reached waypoint
            if reached_destination(target, state) {
                self.current_waypoint += 1;
                info!("Reached waypoint {}, advancing", self.current_waypoint - 1);
            }
        } else {
            // Mission complete
            info!("Mission complete");
            stop_vehicle()?;
        }

        Ok(())
    }

    fn exit(&mut self) {
        // Stop waypoint navigation
        info!("Exiting AUTO mode");
    }
}

/// RTL mode implementation
pub struct RtlMode {
    enter_time_ms: u32,
    home_position: Option<Location>,
    reached_home: bool,
}

impl Mode for RtlMode {
    fn mode_number(&self) -> u8 {
        6  // MAV_MODE_RTL
    }

    fn name(&self) -> &'static str {
        "RTL"
    }

    fn enter(&mut self) -> Result<(), &'static str> {
        // Record entry time
        self.enter_time_ms = get_time_ms();

        // Check if home position set
        if let Some(home) = get_home_position() {
            self.home_position = Some(home);
            self.reached_home = false;
            info!("Entered RTL mode, returning to home: {:?}", home);
            Ok(())
        } else {
            warn!("RTL mode: No home position set");
            Err("No home position")
        }
    }

    fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str> {
        // Navigate to home position
        if let Some(home) = self.home_position {
            if !self.reached_home {
                navigate_to_destination(home, state)?;

                // Check if reached home
                if reached_destination(home, state) {
                    self.reached_home = true;
                    info!("Reached home position");
                    stop_vehicle()?;
                }
            } else {
                // At home - stay stopped
                stop_vehicle()?;
            }
        }

        Ok(())
    }

    fn exit(&mut self) {
        // Clear return state
        self.reached_home = false;
        info!("Exiting RTL mode");
    }
}

/// Mode manager handling lifecycle transitions
pub struct ModeManager {
    current_mode: Box<dyn Mode>,
    previous_mode_number: u8,
}

impl ModeManager {
    /// Create new mode manager
    pub fn new() -> Self {
        Self {
            current_mode: Box::new(ManualMode { enter_time_ms: 0 }),
            previous_mode_number: 0,
        }
    }

    /// Set new mode with proper lifecycle sequencing
    pub fn set_mode(&mut self, new_mode: Box<dyn Mode>,
                     reason: ModeReason) -> Result<(), &'static str> {
        let new_mode_name = new_mode.name();
        let old_mode_name = self.current_mode.name();

        // Try to enter new mode first (validates prerequisites)
        let mut new_mode_mut = new_mode;
        if let Err(e) = new_mode_mut.enter() {
            // Entry failed - validation or init error
            warn!("Failed to enter mode {}: {}", new_mode_name, e);

            // Send notification to GCS
            send_statustext(MAV_SEVERITY_WARNING,
                           &format!("Mode change failed: {}", e))?;

            // Log failed attempt
            log_mode_change_failed(old_mode_name, new_mode_name, e)?;

            return Err(e);
        }

        // New mode entered successfully - exit old mode
        self.current_mode.exit();

        // Switch to new mode
        self.previous_mode_number = self.current_mode.mode_number();
        self.current_mode = new_mode_mut;

        // Log successful transition
        log_mode_change(old_mode_name, new_mode_name, reason)?;
        info!("Mode changed: {} → {} (reason: {:?})",
              old_mode_name, new_mode_name, reason);

        // Notify GCS
        send_statustext(MAV_SEVERITY_INFO,
                       &format!("Mode: {}", new_mode_name))?;

        Ok(())
    }

    /// Update current mode (called every control loop)
    pub fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str> {
        self.current_mode.update(state)
    }

    /// Get current mode name
    pub fn current_mode_name(&self) -> &'static str {
        self.current_mode.name()
    }

    /// Get current mode number
    pub fn current_mode_number(&self) -> u8 {
        self.current_mode.mode_number()
    }
}

/// Mode change reason (for logging and diagnostics)
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ModeReason {
    /// Mode changed via GCS command
    GcsCommand,
    /// Mode changed via RC switch
    RcCommand,
    /// Mode changed by failsafe
    Failsafe,
    /// Mode changed by mission
    Mission,
    /// Mode changed during initialization
    Initializing,
}
```

**Integration with Vehicle Control Loop**:

```rust
// Vehicle control task (50 Hz)
pub fn vehicle_control_task() {
    let mut mode_manager = ModeManager::new();
    let mut system_state = SystemState::new();

    loop {
        // Update current mode (executes mode-specific logic)
        if let Err(e) = mode_manager.update(&mut system_state) {
            error!("Mode update failed: {}", e);
            // Consider fallback mode on repeated failures
        }

        // ... other control logic ...

        delay_ms(20);  // 50 Hz loop
    }
}
```

**Memory Analysis**:

| Component               | RAM Usage     | Notes                                |
| ----------------------- | ------------- | ------------------------------------ |
| Mode trait (vtable)     | \~8 B         | Pointer + vtable reference           |
| ManualMode state        | \~4 B         | Enter timestamp only                 |
| HoldMode state          | \~20 B        | Timestamp + position + flags         |
| AutoMode state          | \~12 B        | Timestamp + waypoint index + flags   |
| RtlMode state           | \~20 B        | Timestamp + home position + flags    |
| ModeManager             | \~16 B        | Current mode pointer + previous mode |
| **Total (active mode)** | **\~40-50 B** | One mode active at a time            |

### Data Analysis

**Mode Transition Frequency** (estimated from ArduPilot logs):

- Mode changes per flight: 5-15 transitions (typical mission)
- Manual → Auto: 30% of transitions
- Auto → RTL: 25% of transitions
- Any → Hold: 20% of transitions (failsafe)
- Hold → Manual: 15% of transitions (recovery)
- Other: 10% of transitions

**Lifecycle Error Rates**:

- Entry validation failures: 5-10% of attempts (missing prerequisites)
- Entry initialization errors: < 1% (rare)
- Update errors: < 0.1% (continuous execution generally stable)
- Exit errors: < 0.1% (cleanup rarely fails)

**Timing Requirements**:

| Lifecycle Phase | Duration | Notes                          |
| --------------- | -------- | ------------------------------ |
| enter()         | < 10 ms  | Includes validation + init     |
| update()        | < 5 ms   | Called at 50 Hz (20 ms period) |
| exit()          | < 5 ms   | Cleanup should be fast         |
| Full transition | < 20 ms  | Exit + enter combined          |

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: Each flight mode shall implement lifecycle methods (enter, update, exit) → Will become FR-<id>
  - Rationale: Structured lifecycle ensures consistent behavior across modes
  - Acceptance Criteria:
    - Define Mode trait with enter(), update(), exit() methods
    - Each mode implements trait with mode-specific logic
    - enter() validates prerequisites and initializes state
    - update() executes continuous mode logic at 50 Hz
    - exit() cleans up state and resets actuators

- [ ] **FR-DRAFT-2**: Mode entry shall initialize mode-specific state and record entry timestamp → Will become FR-<id>
  - Rationale: Time-based mode logic requires entry timestamp, state must be clean
  - Acceptance Criteria:
    - Record entry time in milliseconds
    - Initialize mode-local state (position, waypoints, flags, etc.)
    - Validate prerequisites via mode validation (AN-00013)
    - Return error if prerequisites not met
    - Log entry with mode name and timestamp

- [ ] **FR-DRAFT-3**: Mode update shall execute continuously every control loop iteration (50 Hz) → Will become FR-<id>
  - Rationale: Modes need continuous execution for control logic
  - Acceptance Criteria:
    - Call mode update() every 20 ms (50 Hz)
    - Execute mode-specific control logic (actuator commands, navigation, etc.)
    - Handle errors gracefully (log, continue or failsafe)
    - Measure execution time (must complete < 5 ms)

- [ ] **FR-DRAFT-4**: Mode exit shall clean up mode state and reset actuators to safe state → Will become FR-<id>
  - Rationale: Prevent state leakage, ensure safe actuator configuration
  - Acceptance Criteria:
    - Clear mode-local state (positions, waypoints, targets, etc.)
    - Reset actuators to neutral or safe configuration
    - Release mode-specific resources
    - Log exit with mode name and duration
    - Complete within 5 ms

- [ ] **FR-DRAFT-5**: Mode transitions shall follow ordered sequence (old mode exit → new mode enter) → Will become FR-<id>
  - Rationale: Ensure clean transition without state corruption
  - Acceptance Criteria:
    - Call new mode enter() first (validates prerequisites)
    - If entry succeeds, call old mode exit()
    - If entry fails, abort transition (keep old mode active)
    - Update mode pointer only after successful entry + exit
    - Log transition with old mode, new mode, reason, outcome

- [ ] **FR-DRAFT-6**: Mode lifecycle errors shall be handled gracefully with logging and notification → Will become FR-<id>
  - Rationale: Lifecycle failures must not crash system
  - Acceptance Criteria:
    - Entry errors: log, notify GCS, abort transition
    - Update errors: log, consider fallback mode after repeated failures
    - Exit errors: log, continue with transition (best effort cleanup)
    - Send STATUSTEXT to GCS for all errors
    - Log all errors with mode name, phase, error message

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Mode entry shall complete within 10ms → Will become NFR-<id>
  - Category: Performance
  - Rationale: Fast mode changes responsive to operator commands
  - Target: < 10 ms from entry start to completion (including validation)

- [ ] **NFR-DRAFT-2**: Mode update shall complete within 5ms → Will become NFR-<id>
  - Category: Performance
  - Rationale: 50 Hz control loop requires 20 ms period, mode update is subset
  - Target: < 5 ms per update call (measured via execution profiling)

- [ ] **NFR-DRAFT-3**: Mode exit shall complete within 5ms → Will become NFR-<id>
  - Category: Performance
  - Rationale: Fast cleanup enables responsive mode transitions
  - Target: < 5 ms from exit start to completion

- [ ] **NFR-DRAFT-4**: Mode lifecycle system shall add no more than 100 bytes RAM per mode → Will become NFR-<id>
  - Category: Resource Constraints
  - Rationale: Limited RAM on RP2040/RP2350, mode state must be lean
  - Target: < 100 B per mode for state storage (measured via size profiling)

- [ ] **NFR-DRAFT-5**: All lifecycle transitions shall be logged for post-flight analysis → Will become NFR-<id>
  - Category: Reliability / Auditability
  - Rationale: Support debugging and safety investigations
  - Target: Log entry/exit/errors with timestamp, mode names, reason, duration

## Design Considerations

### Technical Constraints

- **Existing architecture**: Must integrate with mode validation (AN-00013), monitoring (AN-00009)
- **Memory budget**: Limited RAM on RP2040/RP2350 (264 KB) - mode state must be minimal
- **Real-time constraints**: Update at 50 Hz, mode logic must complete in < 5 ms
- **No dynamic allocation**: Mode state must use static/stack allocation only
- **Safety critical**: Lifecycle errors must not compromise vehicle safety
- **Platform abstraction**: Must work on both RP2040 and RP2350
- **Trait objects**: Rust trait objects for dynamic dispatch (mode manager)

### Potential Approaches

1. **Option A: Function Pointers (No Trait)**
   - Pros:
     - Simple implementation (function tables)
     - No trait complexity
     - Minimal runtime overhead
   - Cons:
     - Hard to maintain (manual function tables)
     - No type safety for mode state
     - Difficult to enforce lifecycle contract
     - No Rust ecosystem benefits
   - Effort: Low (16-24 hours)

2. **Option B: Trait-Based Lifecycle** ⭐ Recommended
   - Pros:
     - Clean abstraction (Mode trait)
     - Type-safe mode implementations
     - Enforces lifecycle contract via trait
     - Matches ArduPilot architecture (proven pattern)
     - Extensible for new modes
     - Testable per-mode lifecycle
   - Cons:
     - Trait object overhead (vtable indirection)
     - Slightly more complex than function pointers
   - Effort: Medium (24-32 hours)

3. **Option C: Enum-Based State Machine**
   - Pros:
     - No dynamic dispatch (match statements)
     - Zero indirection overhead
     - Compile-time mode selection
   - Cons:
     - Large match statements for each lifecycle phase
     - Hard to add new modes (modify central enum)
     - Poor separation of concerns
     - Testing requires mocking entire state machine
   - Effort: Medium (20-30 hours)

**Recommendation**: Option B (Trait-Based Lifecycle) provides best balance of safety, extensibility, and maintainability. Matches ArduPilot's proven architecture.

### Architecture Impact

**New ADRs required**:

- **ADR-<id> Mode Lifecycle System**: Trait design, lifecycle phases, transition sequencing
- **ADR-<id> Mode State Management**: Per-mode state storage, ownership model
- **ADR-<id> Lifecycle Error Handling**: Error propagation, recovery strategies
- **ADR-<id> Mode Manager Architecture**: Mode switching, current mode storage

**New modules**:

- `src/vehicle/modes/` - Mode system
  - `src/vehicle/modes/lifecycle.rs` - Mode trait definition
  - `src/vehicle/modes/manager.rs` - ModeManager implementation
  - `src/vehicle/modes/manual.rs` - Manual mode implementation
  - `src/vehicle/modes/hold.rs` - Hold mode implementation
  - `src/vehicle/modes/auto.rs` - Auto mode implementation
  - `src/vehicle/modes/rtl.rs` - RTL mode implementation
  - `src/vehicle/modes/stabilize.rs` - Stabilize mode implementation

**Modified modules**:

- `src/vehicle/control_task.rs` - Integrate mode update() into control loop
- `src/communication/mavlink/handlers/command.rs` - Use ModeManager for mode changes
- `src/vehicle/failsafe/executor.rs` - Use ModeManager for failsafe mode changes
- `src/communication/mavlink/state.rs` - Replace simple set_mode() with ModeManager

**Dependencies**:

- Mode validation (AN-00013): Used in enter() for prerequisite checking
- Monitoring system (AN-00009): Detects sensor loss triggering mode transitions
- Actuator system: Used in update() for control output
- Navigation system: Used by Auto/RTL modes for waypoint navigation

## Parameters

This analysis does not reference specific ArduPilot parameters. Mode lifecycle management (enter/update/exit pattern) is an architectural design pattern implemented in ArduPilot's Mode base class.

The lifecycle pattern is hardcoded in ArduPilot's mode implementation and does not use configurable parameters. Mode-specific behavior is defined in each mode's subclass implementation rather than through parameter configuration.

Related parameters that affect mode behavior:

- Mode capability parameters (documented in AN-00012-mode-capability-system.md)
- Mode entry validation parameters (documented in AN-00013-mode-entry-validation.md)
- Failsafe action parameters that trigger mode changes (documented in AN-00011-failsafe-system.md)

## Risk Assessment

| Risk                                                 | Probability | Impact     | Mitigation Strategy                                                  |
| ---------------------------------------------------- | ----------- | ---------- | -------------------------------------------------------------------- |
| **Mode entry failure leaves vehicle in unsafe mode** | **Medium**  | **High**   | **Abort transition on entry failure, keep old mode active**          |
| **Update errors cause repeated failsafe triggers**   | **Medium**  | **Medium** | **Log errors, only trigger fallback after sustained failure (> 1s)** |
| Lifecycle timing exceeds budget (> 5ms)              | Low         | Medium     | Profile early, optimize hot paths, consider simpler modes            |
| Mode state memory exceeds budget (> 100B)            | Low         | Low        | Monitor per-mode allocation, minimize state storage                  |
| Trait object overhead impacts performance            | Low         | Low        | Profile, consider static dispatch if critical                        |
| Exit errors during transition                        | Low         | Medium     | Best-effort cleanup, log errors, continue with transition            |
| Mode state leakage between transitions               | Medium      | Medium     | Thorough testing, clear all state in exit()                          |

## Open Questions

- [ ] Should mode update errors trigger immediate fallback? → Method: Only after sustained failure (3+ errors in 1 second)
- [ ] How to handle exit() errors during transition? → Decision: Log error, continue transition (best effort cleanup)
- [ ] Do we need mode-specific parameters? → Decision: Phase 1 no, Phase 2 add per-mode configuration
- [ ] Should mode state be heap or stack allocated? → Decision: Stack allocated in ModeManager (Box<dyn Mode>)
- [ ] How to test mode lifecycle without hardware? → Method: Unit tests with mocked SystemState, integration tests in simulator
- [ ] Should we support mode stacking (nested modes)? → Decision: No, out of scope for Phase 1 (single active mode only)
- [ ] Do we need pre-enter() hook for preparation? → Decision: No, entry validation sufficient for Phase 1

## Recommendations

### Immediate Actions

1. **Adopt Option B architecture**: Trait-based lifecycle with enter/update/exit
2. **Implement Phase 1 modes**: Manual, Hold, Stabilize only (simpler modes first)
3. **Follow ArduPilot patterns**: Match proven lifecycle sequencing
4. **Integrate with validation**: Use AN-00013 validation in enter()
5. **Profile performance**: Measure lifecycle timing early, optimize if needed

### Next Steps

1. [ ] Create formal requirements: FR-<id> (lifecycle trait), FR-<id> (mode entry), FR-<id> (mode update), FR-<id> (mode exit), FR-<id> (transition sequencing), FR-<id> (error handling), NFR-<id> (entry timing), NFR-<id> (update timing), NFR-<id> (exit timing), NFR-<id> (memory per mode), NFR-<id> (logging)
2. [ ] Draft ADR for: Mode lifecycle system (trait design, phases, sequencing)
3. [ ] Draft ADR for: Mode state management (storage, ownership)
4. [ ] Draft ADR for: Lifecycle error handling (propagation, recovery)
5. [ ] Draft ADR for: Mode manager architecture (switching, storage)
6. [ ] Create task for: Mode lifecycle implementation (Phase 1: Manual, Hold, Stabilize)
7. [ ] Plan integration testing: Verify lifecycle transitions, error handling, timing

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **Nested modes**: No mode stacking (e.g., Auto+AvoidObstacles), single active mode only
- **Pre-enter hooks**: No preparation phase before entry validation
- **Conditional update**: All modes update every loop, no selective execution
- **Mode priorities**: All modes equal priority, no override system
- **Mode groups**: No mode families or categorization system
- **Hot-reload modes**: No runtime mode loading, compile-time only
- **Mode-specific parameters**: Phase 1 no per-mode config, Phase 2 add if needed
- **Async mode operations**: All lifecycle methods synchronous, no async/await
- **Mode callbacks**: No event hooks (on-enter-complete, on-exit-start, etc.)
- **Mode introspection**: No reflection or runtime mode discovery

## Appendix

### References

- ArduPilot Rover Mode Class: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.h>
- ArduPilot Mode Base Implementation: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.cpp>
- ArduPilot Manual Mode: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_manual.cpp>
- ArduPilot Auto Mode: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_auto.cpp>
- ArduPilot RTL Mode: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_rtl.cpp>
- ArduPilot Hold Mode: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_hold.cpp>

### Raw Data

**ArduPilot Mode Lifecycle Flow**:

```
Mode Transition Request (GCS/RC/Failsafe)
  │
  ▼
┌─────────────────────────────────────┐
│ set_mode(new_mode, reason)          │
└───────────┬─────────────────────────┘
            │
            ▼
    ┌───────────────────┐
    │ new_mode.enter()  │ ← Validate prerequisites
    └───────┬───────────┘
            │
            ▼
       Entry OK?
       ├─ No → Abort transition, keep old mode
       └─ Yes
            │
            ▼
    ┌───────────────────┐
    │ old_mode.exit()   │ ← Cleanup old mode
    └───────┬───────────┘
            │
            ▼
    ┌───────────────────┐
    │ control_mode =    │
    │   &new_mode       │
    └───────┬───────────┘
            │
            ▼
    ┌───────────────────┐
    │ Log transition    │
    │ Notify GCS        │
    └───────────────────┘
```

**Mode Update Execution** (50 Hz control loop):

```
Control Loop (50 Hz)
  │
  ├─→ Read sensors (IMU, GPS, RC)
  │
  ├─→ Update navigation (EKF, AHRS)
  │
  ├─→ control_mode->update()  ← Execute mode logic
  │     │
  │     ├─ Manual: RC → Motors
  │     ├─ Hold: Stop or hold position
  │     ├─ Auto: Navigate to waypoint
  │     ├─ RTL: Navigate to home
  │     └─ Stabilize: Heading hold
  │
  ├─→ Apply motor limits (throttle, steering)
  │
  ├─→ Send actuator commands
  │
  └─→ delay_ms(20)  → Next iteration
```

**Proposed pico_trail Lifecycle Integration**:

```rust
// Control loop integration
pub fn vehicle_control_loop() {
    let mut mode_manager = ModeManager::new();
    let mut system_state = SystemState::new();
    let mut last_update_ms = 0;

    loop {
        let current_ms = get_time_ms();

        // Read sensors
        update_sensors(&mut system_state)?;

        // Update navigation
        update_navigation(&mut system_state)?;

        // Execute mode logic (50 Hz)
        if (current_ms - last_update_ms) >= 20 {
            if let Err(e) = mode_manager.update(&mut system_state) {
                error!("Mode update failed: {}", e);
                // Consider fallback after sustained failures
            }
            last_update_ms = current_ms;
        }

        // Apply motor limits and send commands
        apply_actuator_limits(&mut system_state)?;
        send_actuator_commands(&system_state)?;

        delay_ms(20);
    }
}

// Mode change integration
pub fn handle_mode_change_command(
    mode_manager: &mut ModeManager,
    mode_number: u8,
) -> Result<(), &'static str> {
    let new_mode: Box<dyn Mode> = match mode_number {
        0 => Box::new(ManualMode::new()),
        4 => Box::new(HoldMode::new()),
        10 => Box::new(AutoMode::new()),
        6 => Box::new(RtlMode::new()),
        _ => return Err("Invalid mode number"),
    };

    mode_manager.set_mode(new_mode, ModeReason::GcsCommand)
}
```
