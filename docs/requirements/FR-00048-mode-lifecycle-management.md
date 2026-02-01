# FR-00048 Mode Lifecycle Management

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P0
- Category: Mode Management / Architecture

## Links

- Parent Analysis: [AN-00014-mode-lifecycle-management](../analysis/AN-00014-mode-lifecycle-management.md)
- Related Requirements:
  - [FR-00045-mode-capability-queries](FR-00045-mode-capability-queries.md)
  - [FR-00046-mode-entry-sensor-validation](FR-00046-mode-entry-sensor-validation.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  - [FR-00039-fallback-mode-selection](FR-00039-fallback-mode-selection.md)
  - [NFR-00040-mode-state-memory](NFR-00040-mode-state-memory.md)
  - [NFR-00038-mode-entry-timing](NFR-00038-mode-entry-timing.md)
  - [NFR-00041-mode-update-timing](NFR-00041-mode-update-timing.md)
  - [NFR-00039-mode-exit-timing](NFR-00039-mode-exit-timing.md)
  - [NFR-00037-lifecycle-transition-logging](NFR-00037-lifecycle-transition-logging.md)

## Requirement Statement

The system shall implement structured mode lifecycle management with trait-based interface (enter, update, exit methods), ordered transition sequencing (new mode entry validation before old mode exit), comprehensive error handling (graceful failures with logging and notification), ensuring clean state transitions, consistent behavior across modes, and system stability during mode changes.

## Rationale

Structured lifecycle management is essential for:

- **Consistent behavior**: All modes follow same lifecycle pattern (enter/update/exit)
- **Bug prevention**: Prevents common issues like state leakage, missing initialization, resource leaks
- **Safe transitions**: Validates new mode before exiting old mode (prevents "no mode" state)
- **System stability**: Graceful error handling maintains system integrity during failures
- **Debuggability**: Comprehensive logging enables post-flight analysis
- **Operator awareness**: GCS notifications alert operator to mode issues

ArduPilot's proven Mode lifecycle pattern (enter/update/exit) provides clear contract for mode implementations, ensuring predictable behavior and maintainability.

## User Story (if applicable)

As a mode implementer, I want a structured lifecycle framework with clear entry/update/exit phases, so that my mode integrates consistently with the system and handles transitions safely.

As an operator, I want mode transitions to be safe and predictable, with clear error messages if transitions fail, so that I understand the vehicle's behavior and can respond appropriately.

## Acceptance Criteria

### Mode Trait Definition

- [ ] Define Mode trait with lifecycle methods:
  - `fn enter(&mut self) -> Result<(), &'static str>`: Initialize mode state, validate prerequisites
  - `fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str>`: Execute mode logic every control loop (50 Hz)
  - `fn exit(&mut self)`: Clean up mode state, reset actuators (best-effort, cannot fail)
  - `fn enter_time_ms(&self) -> u32`: Get entry timestamp for time-based logic
  - `fn mode_number(&self) -> u8`: Get mode identifier
  - `fn name(&self) -> &'static str`: Get mode name for logging

- [ ] All modes implement Mode trait:
  - Manual, Stabilize, Hold, Loiter, Auto, RTL
  - Compile-time enforcement of lifecycle contract

### Entry Phase (Initialization)

- [ ] Entry timestamp recording:
  - Record `get_time_ms()` when mode entered
  - Store in mode-local `enter_time_ms` field
  - Used for time-based logic (e.g., "5 seconds after entering Auto")

- [ ] Mode-specific state initialization:
  - Manual: No state needed
  - Hold: Initialize hold_position (if GPS available), set stopped state
  - Auto: Initialize current_waypoint = 0, validate mission loaded
  - RTL: Set target to home position, verify home set
  - Loiter: Record loiter position

- [ ] Prerequisites validation:
  - Call mode validation (FR-00046) during enter()
  - Return Err() if validation fails
  - Validate sensor requirements via capability queries

- [ ] Entry logging:
  - Log successful entry: `"Entered {mode} at {timestamp}"`
  - Log failed entry: `"Failed to enter {mode}: {reason}"`
  - Log format: `"MODE_ENTRY,{ts},{mode},{result}"`

- [ ] Entry timing: Complete within 10ms target

### Update Phase (Continuous Execution)

- [ ] Control loop integration:
  - Call `mode_manager.update(&mut system_state)` every control loop
  - Control loop frequency: 50 Hz (20 ms period)
  - Mode update called after sensor reads, before actuator output

- [ ] Mode-specific update logic:
  - Manual: Read RC inputs, send to actuators directly
  - Stabilize: Read RC inputs, apply heading stabilization, send to actuators
  - Hold: Stop vehicle or navigate to hold position (if GPS)
  - Auto: Navigate to current waypoint, advance on arrival
  - RTL: Navigate to home position, stop on arrival

- [ ] Update timing:
  - Measure update() duration
  - Target: < 5ms (allows 15ms for other tasks in 20ms loop)
  - Log warning if update exceeds 5ms

### Exit Phase (Cleanup)

- [ ] Mode-specific cleanup logic:
  - Manual: Clear lateral motor to prevent jerky movement
  - Hold: Clear hold_position flag
  - Auto: Stop waypoint navigation, clear current_waypoint
  - RTL: Clear reached_home flag
  - Loiter: Clear loiter_position

- [ ] Actuator state reset:
  - Reset actuators to safe/neutral state if needed
  - Clear residual commands

- [ ] Resource cleanup:
  - Release any mode-allocated resources
  - Clear mode-local state variables
  - Reset flags to initial values

- [ ] Exit logging:
  - Log: `"Exiting {mode} (duration: {duration}ms)"`
  - Include mode name, entry duration

- [ ] Exit timing: Complete within 5ms target

- [ ] Best-effort cleanup:
  - Exit cannot fail (void return type)
  - Errors logged but don't block transition

### Transition Sequencing

- [ ] Ordered transition sequence:
  - Step 1: Call `new_mode.enter()` (validates prerequisites)
  - Step 2: If enter() succeeds, call `old_mode.exit()` (cleanup)
  - Step 3: Update mode pointer: `current_mode = new_mode`
  - Step 4: Log transition
  - If enter() fails: Abort transition, keep `current_mode` unchanged

- [ ] Atomic transition:
  - Mode pointer updated only after both enter() and exit() complete
  - No intermediate state where no mode is active
  - Old mode remains functional until new mode successfully entered

- [ ] Transition logging:
  - Successful: `"MODE_TRANSITION,{ts},{old},{new},{reason},SUCCESS"`
  - Failed: `"MODE_TRANSITION,{ts},{attempted},{reason},DENIED"`

- [ ] Transition duration: Typically < 20ms

### Error Handling

- [ ] Entry error handling:
  - enter() returns Err(): Log with mode name, reason
  - Send STATUSTEXT: "Failed to enter {mode}: {reason}"
  - Abort transition, keep old mode active
  - Log: `"MODE_ENTRY_FAILED,{ts},{mode},{reason}"`
  - Severity: MAV_SEVERITY_WARNING

- [ ] Update error handling:
  - update() returns Err(): Log error
  - Single error: Log and continue
  - Repeated errors (> 3 errors in 1 second): Consider fallback mode
  - Log: `"MODE_UPDATE_ERROR,{ts},{mode},{error}"`
  - Severity: MAV_SEVERITY_ERROR for repeated failures

- [ ] Exit error handling:
  - exit() errors: Log but proceed with transition
  - Best-effort cleanup (exit cannot block transition)
  - Log: `"MODE_EXIT_ERROR,{ts},{mode},{error}"`

- [ ] GCS notification:
  - All errors send STATUSTEXT message
  - Notification within 100ms of error

- [ ] Error recovery:
  - Entry failure: Remain in old mode
  - Update failure (sustained): Trigger fallback mode selection (FR-00039)
  - Exit failure: Proceed with transition (cleanup not critical)

## Technical Details (if applicable)

### Functional Requirement Details

**Mode Trait Definition:**

```rust
/// Mode lifecycle trait for structured mode behavior
pub trait Mode {
    /// Initialize mode state, validate prerequisites
    /// Returns Ok() for successful init, Err() for failure
    fn enter(&mut self) -> Result<(), &'static str>;

    /// Execute mode logic every control loop (50 Hz)
    /// Returns Ok() for normal operation, Err() for errors
    fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str>;

    /// Clean up mode state, reset actuators
    /// Best-effort cleanup, cannot fail
    fn exit(&mut self);

    /// Get entry timestamp for time-based logic
    fn enter_time_ms(&self) -> u32;

    /// Get mode identifier
    fn mode_number(&self) -> u8;

    /// Get mode name for logging
    fn name(&self) -> &'static str;
}
```

**Mode Manager Transition Logic:**

```rust
pub struct ModeManager {
    current_mode: Box<dyn Mode>,
    mode_reason: ModeReason,
}

impl ModeManager {
    /// Set new mode with ordered sequencing
    pub fn set_mode(&mut self, mut new_mode: Box<dyn Mode>, reason: ModeReason)
                    -> Result<(), &'static str> {
        // Check if already in this mode
        if self.current_mode.mode_number() == new_mode.mode_number() {
            return Ok(());
        }

        let old_mode_name = self.current_mode.name();
        let new_mode_name = new_mode.name();

        // Step 1: Try to enter new mode (validates prerequisites)
        if let Err(e) = new_mode.enter() {
            // Entry failed - log, notify, abort
            error!("Failed to enter mode {}: {}", new_mode_name, e);
            self.send_statustext(
                MAV_SEVERITY_WARNING,
                &format!("Failed to enter {}: {}", new_mode_name, e)
            )?;
            self.log_transition_failed(new_mode_name, reason, e)?;
            return Err(e);
        }

        // Step 2: Exit old mode (cleanup, best-effort)
        self.current_mode.exit();

        // Step 3: Update mode pointer (atomic switch)
        let old_mode = std::mem::replace(&mut self.current_mode, new_mode);

        // Step 4: Log successful transition
        info!("Mode transition: {} -> {} (reason: {:?})",
              old_mode_name, new_mode_name, reason);
        self.log_transition_success(old_mode_name, new_mode_name, reason)?;

        self.mode_reason = reason;
        Ok(())
    }

    /// Update current mode (called at 50 Hz)
    pub fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str> {
        // Execute mode logic
        if let Err(e) = self.current_mode.update(state) {
            // Update error - log and handle
            error!("Mode update error ({}): {}", self.current_mode.name(), e);

            // Track error rate
            self.update_error_count += 1;
            let now = get_time_ms();
            if now - self.last_error_check_ms > 1000 {
                // Check error rate over last second
                if self.update_error_count > 3 {
                    // Sustained errors - trigger fallback mode
                    warn!("Sustained mode update errors, triggering fallback");
                    self.trigger_fallback_mode()?;
                }
                self.update_error_count = 0;
                self.last_error_check_ms = now;
            }

            return Err(e);
        }

        Ok(())
    }
}
```

**Example Mode Implementation (Manual):**

```rust
pub struct ManualMode {
    enter_time_ms: u32,
}

impl Mode for ManualMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        // Record entry time
        self.enter_time_ms = get_time_ms();

        // Manual mode has no prerequisites
        info!("Entered Manual mode at {}", self.enter_time_ms);
        Ok(())
    }

    fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str> {
        // Read RC inputs
        let steering = state.rc_input.channel(0); // Channel 1
        let throttle = state.rc_input.channel(2); // Channel 3

        // Send directly to actuators (if armed)
        if state.is_armed() {
            state.actuators.set_steering(steering)?;
            state.actuators.set_throttle(throttle)?;
        }

        Ok(())
    }

    fn exit(&mut self) {
        // Clear lateral motor to prevent jerky movement
        // (Best-effort cleanup, errors ignored)
        info!("Exiting Manual mode (duration: {}ms)",
              get_time_ms().wrapping_sub(self.enter_time_ms));
    }

    fn enter_time_ms(&self) -> u32 {
        self.enter_time_ms
    }

    fn mode_number(&self) -> u8 {
        0 // Manual mode number
    }

    fn name(&self) -> &'static str {
        "Manual"
    }
}
```

**Example Mode Implementation (Auto):**

```rust
pub struct AutoMode {
    enter_time_ms: u32,
    current_waypoint: usize,
    mission: Option<Mission>,
}

impl Mode for AutoMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        // Record entry time
        self.enter_time_ms = get_time_ms();

        // Validate prerequisites via mode validation
        if !self.requires_position() && !has_position_estimate() {
            return Err("Auto mode requires position estimate");
        }

        if !self.requires_gps() && !has_gps_fix() {
            return Err("Auto mode requires GPS fix");
        }

        // Validate mission loaded
        if self.mission.is_none() {
            return Err("Auto mode requires mission loaded");
        }

        // Initialize waypoint navigation
        self.current_waypoint = 0;

        info!("Entered Auto mode at {}, starting mission", self.enter_time_ms);
        Ok(())
    }

    fn update(&mut self, state: &mut SystemState) -> Result<(), &'static str> {
        // Navigate to current waypoint
        let mission = self.mission.as_ref()
            .ok_or("Mission not loaded")?;

        let waypoint = mission.get_waypoint(self.current_waypoint)
            .ok_or("Invalid waypoint index")?;

        // Calculate steering/throttle to reach waypoint
        let (steering, throttle) = self.calculate_nav_commands(waypoint, state)?;

        // Send to actuators
        state.actuators.set_steering(steering)?;
        state.actuators.set_throttle(throttle)?;

        // Check if waypoint reached
        if self.waypoint_reached(waypoint, state) {
            self.current_waypoint += 1;
            info!("Waypoint {} reached, advancing", self.current_waypoint - 1);
        }

        Ok(())
    }

    fn exit(&mut self) {
        // Stop waypoint navigation
        self.current_waypoint = 0;

        info!("Exiting Auto mode (duration: {}ms)",
              get_time_ms().wrapping_sub(self.enter_time_ms));
    }

    fn enter_time_ms(&self) -> u32 {
        self.enter_time_ms
    }

    fn mode_number(&self) -> u8 {
        10 // Auto mode number
    }

    fn name(&self) -> &'static str {
        "Auto"
    }
}
```

**Log Entry Formats:**

```
# Successful transition
MODE_TRANSITION,123456,Manual,Auto,GcsCommand,SUCCESS

# Failed transition
MODE_TRANSITION,123456,Manual,Auto,GcsCommand,DENIED
MODE_ENTRY_FAILED,123456,Auto,Position estimate unavailable

# Update errors
MODE_UPDATE_ERROR,123500,Auto,Waypoint navigation failed

# Exit (informational)
Exiting Manual mode (duration: 30500ms)
```

## Platform Considerations

### Pico W (RP2040) / Pico 2 W (RP2350)

- Memory constraints: Mode state should be minimal (< 100 bytes per mode)
- Timing constraints: enter/exit < 10ms, update < 5ms
- Embedded-friendly: No dynamic allocation in critical path

### Cross-Platform

- Trait-based design is platform-agnostic
- Timing requirements apply to all platforms
- Logging format consistent across platforms

## Risks & Mitigation

| Risk                                                   | Impact | Likelihood | Mitigation                                                                        | Validation                                            |
| ------------------------------------------------------ | ------ | ---------- | --------------------------------------------------------------------------------- | ----------------------------------------------------- |
| Mode state leakage between transitions                 | High   | Medium     | Mandatory exit() cleanup, unit tests verify state reset                           | Test: verify state cleared after exit                 |
| Transition failure leaves vehicle with no active mode  | High   | Low        | Validate new mode entry before exiting old mode, atomic transition                | Test: inject entry failure, verify old mode remains   |
| Update errors crash system                             | High   | Low        | Return Result from update(), log errors, trigger fallback after sustained errors  | Test: inject update errors, verify graceful handling  |
| Entry validation too strict (prevents legitimate use)  | Medium | Medium     | Review validation logic, allow override for test/debug                            | Operational review: monitor entry denial rate         |
| Exit cleanup blocks transition (long-running cleanup)  | Medium | Low        | Exit is best-effort (void return), timeout if needed, measure exit duration       | Test: measure exit timing, verify < 5ms target        |
| Lifecycle timing violations (update takes too long)    | Medium | Medium     | Measure execution time, log violations, optimize slow modes                       | Performance profiling: verify timing requirements met |
| Mode trait not implemented correctly (missing methods) | Low    | Low        | Compile-time enforcement via trait, code review                                   | Compilation check: all modes implement trait          |
| Error flooding (rapid update errors)                   | Low    | Low        | Rate-limit error logging (1 error per second per mode), aggregate repeated errors | Test: induce rapid errors, verify rate limiting works |

## Implementation Notes

**Preferred approaches:**

- **Trait-based design**: Compile-time enforcement of lifecycle contract
- **Atomic transitions**: Mode pointer updated only after successful entry + exit
- **Ordered sequencing**: Always validate new mode entry before exiting old mode
- **Graceful error handling**: Log all errors, notify GCS, maintain system stability
- **Best-effort exit**: Exit cannot fail or block transition
- **Comprehensive logging**: Log all lifecycle events for debugging

**Known pitfalls:**

- **Entry validation too early**: Don't validate in mode constructor, validate in enter()
- **Exit side effects**: Exit should be idempotent (safe to call multiple times)
- **Update assumptions**: Don't assume update() called at exact 50 Hz (may vary)
- **Resource leaks**: Exit must release all mode-specific resources
- **Race conditions**: Mode pointer must be updated atomically
- **Blocking operations**: Never block in enter/update/exit (violates timing constraints)

**Related code areas:**

- `src/vehicle/mode/` - Mode trait and implementations
- `src/vehicle/mode_manager.rs` - Mode manager with transition logic
- `src/vehicle/mode/manual.rs` - Manual mode implementation
- `src/vehicle/mode/auto.rs` - Auto mode implementation
- `src/vehicle/mode/rtl.rs` - RTL mode implementation

**Suggested patterns:**

- Builder pattern for mode initialization (construct → configure → activate)
- State machine for complex modes (Auto with multiple phases)
- Command pattern for mode-specific actions

## External References

- Analysis: [AN-00014-mode-lifecycle-management](../analysis/AN-00014-mode-lifecycle-management.md)
- ArduPilot Mode Class: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.h>
- ArduPilot set_mode(): <https://github.com/ArduPilot/ardupilot/blob/master/Rover/Rover.cpp#L850-880>
