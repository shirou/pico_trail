# FR-q2sjt Control Mode Execution Framework

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
- Dependent Requirements:
  - [FR-uk0us-manual-mode](FR-uk0us-manual-mode.md)
  - [FR-sp3at-control-modes](FR-sp3at-control-modes.md)
- Related Tasks:
  - [T-3irm5-manual-control-implementation](../tasks/T-3irm5-manual-control-implementation/README.md)

## Requirement Statement

The system shall provide a control mode execution framework that manages mode lifecycle (enter, update, exit), handles mode transitions with validation, executes the active mode at 50 Hz, and processes mode change requests via MAVLink commands.

## Rationale

A control mode framework provides the infrastructure for all control modes (Manual, Hold, Auto, RTL, Guided), enabling:

- **Consistent interface**: All modes implement common trait (enter, update, exit)
- **Safe transitions**: Mode manager validates transitions and handles cleanup
- **Execution scheduling**: Active mode runs at 50 Hz control loop rate
- **MAVLink integration**: Mode changes via DO_SET_MODE command
- **Extensibility**: Easy to add new modes (Auto, RTL) without changing framework

The framework is the foundation for all autonomous and manual control modes required by FR-sp3at.

## User Story (if applicable)

As a developer implementing a new control mode, I want a common framework with enter/update/exit lifecycle hooks, so that I can focus on mode-specific logic without reimplementing mode management and execution infrastructure.

## Acceptance Criteria

- [ ] Define `Mode` trait with `enter()`, `update(dt)`, `exit()`, and `name()` methods
- [ ] Implement `ModeManager` to handle mode transitions and execute active mode
- [ ] Execute active mode `update()` method at 50 Hz (every 20ms)
- [ ] Call `enter()` on mode entry (once per transition)
- [ ] Call `exit()` on mode exit (once per transition)
- [ ] Pass delta time (`dt`) to `update()` method for time-aware control
- [ ] Process MAVLink COMMAND_LONG (MAV_CMD_DO_SET_MODE) for mode changes
- [ ] Validate mode transitions: reject invalid transitions (e.g., Auto without GPS)
- [ ] Update system state: `SystemState.mode` reflects current mode
- [ ] Broadcast mode change via MAVLink STATUSTEXT message
- [ ] Log mode transitions with timestamp and reason
- [ ] Support at least 5 modes: Manual, Hold, Auto, RTL, Guided (per FR-sp3at)
- [ ] Default mode: Manual (on boot or mode transition failure)

## Technical Details (if applicable)

### Functional Requirement Details

**Control Mode Trait:**

```rust
/// Control mode trait
///
/// All control modes (Manual, Hold, Auto, RTL, Guided) implement this trait.
pub trait Mode {
    /// Initialize mode (called once on mode entry)
    ///
    /// Returns Err if mode cannot be entered (e.g., Auto without GPS).
    fn enter(&mut self) -> Result<(), &'static str>;

    /// Update mode (called at 50 Hz)
    ///
    /// `dt`: Delta time since last update (seconds)
    fn update(&mut self, dt: f32) -> Result<(), &'static str>;

    /// Cleanup mode (called once on mode exit)
    ///
    /// Should set actuators to safe state (neutral).
    fn exit(&mut self) -> Result<(), &'static str>;

    /// Get mode name for logging and telemetry
    fn name(&self) -> &'static str;
}
```

**Mode Manager:**

```rust
pub struct ModeManager {
    /// Current active mode
    current_mode: Box<dyn Mode>,
    /// System state (for mode reporting)
    system_state: &mut SystemState,
    /// Last update timestamp
    last_update_us: u64,
}

impl ModeManager {
    /// Create mode manager with initial mode (Manual)
    pub fn new(initial_mode: Box<dyn Mode>) -> Self {
        // ...
    }

    /// Execute active mode (call at 50 Hz)
    pub fn execute(&mut self, current_time_us: u64) -> Result<(), &'static str> {
        // Calculate delta time
        let dt = (current_time_us - self.last_update_us) as f32 / 1_000_000.0;
        self.last_update_us = current_time_us;

        // Update active mode
        self.current_mode.update(dt)?;
        Ok(())
    }

    /// Request mode change (from MAVLink command or internal logic)
    pub fn set_mode(&mut self, new_mode: Box<dyn Mode>) -> Result<(), &'static str> {
        // Exit current mode
        self.current_mode.exit()?;

        // Enter new mode
        if let Err(e) = new_mode.enter() {
            // Mode entry failed, revert to Manual (safe fallback)
            let manual_mode = ManualMode::new(/* ... */);
            manual_mode.enter()?;
            self.current_mode = Box::new(manual_mode);
            return Err(e);
        }

        // Update system state
        let mode_name = new_mode.name();
        self.current_mode = new_mode;
        self.system_state.set_mode(/* ... */)?;

        // Log mode change
        // Send MAVLink STATUSTEXT: "Mode changed: Manual → Auto"

        Ok(())
    }

    /// Get current mode name
    pub fn current_mode_name(&self) -> &'static str {
        self.current_mode.name()
    }
}
```

**Mode Transition Validation:**

Mode transitions follow rules defined in FR-sp3at:

- **To Manual**: Always allowed (safety override)
- **To Hold**: Always allowed
- **To Auto**: Requires GPS fix + valid mission
- **To RTL**: Requires GPS fix + known launch point
- **To Guided**: Requires GPS fix

Validation logic in `set_mode()`:

```rust
fn validate_mode_transition(new_mode: &dyn Mode) -> Result<(), &'static str> {
    match new_mode.name() {
        "Manual" => Ok(()), // Always allowed
        "Hold" => Ok(()),   // Always allowed
        "Auto" => {
            if !gps_has_fix() {
                return Err("Auto mode requires GPS fix");
            }
            if !mission_valid() {
                return Err("Auto mode requires valid mission");
            }
            Ok(())
        }
        "RTL" => {
            if !gps_has_fix() {
                return Err("RTL mode requires GPS fix");
            }
            Ok(())
        }
        "Guided" => {
            if !gps_has_fix() {
                return Err("Guided mode requires GPS fix");
            }
            Ok(())
        }
        _ => Err("Unknown mode"),
    }
}
```

**MAVLink Integration:**

Mode change command (from Mission Planner):

```
COMMAND_LONG {
    command: MAV_CMD_DO_SET_MODE (176),
    param1: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1),
    param2: mode_number (0=Manual, 1=Stabilize, 2=Hold, 3=Auto, 4=RTL)
}
```

Handler in `src/communication/mavlink/handlers/command.rs`:

```rust
fn handle_do_set_mode(mode_number: u32) -> Result<(), &'static str> {
    let new_mode = match mode_number {
        0 => ManualMode::new(/* ... */),
        1 => StabilizeMode::new(/* ... */),
        2 => HoldMode::new(/* ... */),
        3 => AutoMode::new(/* ... */),
        4 => RtlMode::new(/* ... */),
        _ => return Err("Invalid mode number"),
    };

    mode_manager.set_mode(Box::new(new_mode))?;
    Ok(())
}
```

**Vehicle Control Task:**

Scheduler task for mode execution:

```rust
#[embassy_executor::task]
async fn vehicle_control_task(mode_manager: &'static mut ModeManager) {
    let mut ticker = Ticker::every(Duration::from_millis(20)); // 50 Hz

    loop {
        let current_time_us = embassy_time::Instant::now().as_micros();

        if let Err(e) = mode_manager.execute(current_time_us) {
            // Log error, potentially switch to failsafe mode
            defmt::error!("Mode execution error: {}", e);
        }

        ticker.next().await;
    }
}
```

## Platform Considerations

### Pico W (RP2040)

N/A - Platform agnostic (framework logic independent of hardware)

### Pico 2 W (RP2350)

N/A - Platform agnostic

### Cross-Platform

Control mode framework is fully cross-platform. Platform-specific details abstracted via:

- `PwmInterface` (actuator control)
- `Timer` (delta time calculation)

## Risks & Mitigation

| Risk                                             | Impact | Likelihood | Mitigation                                                   | Validation                                    |
| ------------------------------------------------ | ------ | ---------- | ------------------------------------------------------------ | --------------------------------------------- |
| Mode transition during critical maneuver         | High   | Low        | Defer transition until safe (see FR-sp3at)                   | Test: mode change during high-speed turn      |
| Mode entry failure (e.g., Auto without GPS)      | Medium | Medium     | Validate pre-conditions, revert to Manual on failure         | Test: attempt Auto without GPS, verify Manual |
| Mode trait overhead (dynamic dispatch)           | Low    | Low        | Use static dispatch where possible, profile 50 Hz execution  | Measure: mode update latency < 1ms            |
| Concurrent mode change requests (race condition) | Medium | Low        | Mode manager owned by vehicle control task (single-threaded) | Design review: verify single owner            |

## Implementation Notes

Preferred approaches:

- **Trait-based polymorphism**: Clean abstraction, Rust idiom
- **Mode manager owns modes**: Single ownership, no shared mutable state
- **Fail-safe to Manual**: If mode entry fails, always revert to Manual (safest mode)
- **Atomic transitions**: Mode change is atomic (exit → enter), no partial state

Known pitfalls:

- **Mode lifetime**: Modes must live as long as mode manager (use `'static` or arena allocation)
- **Delta time calculation**: Use microseconds for precision, convert to seconds for control
- **Error handling**: Mode `update()` errors should log but not crash (fail gracefully)
- **Mode state leakage**: Each mode should clean up in `exit()` (e.g., clear navigation targets)

Related code areas:

- `src/vehicle/mode_manager.rs` - Mode manager implementation
- `src/vehicle/modes/` - Mode implementations (manual.rs, hold.rs, auto.rs, rtl.rs, guided.rs)
- `src/core/scheduler/tasks/vehicle.rs` - Vehicle control task
- `src/communication/mavlink/handlers/command.rs` - DO_SET_MODE command handler

## External References

- ArduPilot Mode Class: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Vehicle/AP_Vehicle_Type.h>
- MAVLink DO_SET_MODE: <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE>
- ArduPilot Mode Switching: <https://ardupilot.org/rover/docs/common-flight-modes.html>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
