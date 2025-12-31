# ADR-w9zpl Control Mode Architecture: Trait-Based Polymorphism with Mode Manager

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-q2sjt-control-mode-framework](../requirements/FR-q2sjt-control-mode-framework.md)
  - [FR-uk0us-manual-mode](../requirements/FR-uk0us-manual-mode.md)
  - [FR-sp3at-control-modes](../requirements/FR-sp3at-control-modes.md)
  - [NFR-v6kvd-vehicle-layer-memory](../requirements/NFR-v6kvd-vehicle-layer-memory.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-3irm5-manual-control-implementation](../tasks/T-3irm5-manual-control-implementation/README.md)
  - [T-n24yy-rover-loiter-mode](../tasks/T-n24yy-rover-loiter-mode/README.md)
  - [T-bo6xc-rtl-smartrtl-implementation](../tasks/T-bo6xc-rtl-smartrtl-implementation/README.md)

## Context

The autopilot must support multiple control modes (Manual, Hold, Auto, RTL, Guided) as defined in FR-sp3at. Each mode has distinct behavior:

- **Manual**: Direct pass-through of RC inputs to actuators
- **Hold**: Maintain current position, stop all movement
- **Auto**: Follow pre-programmed waypoint mission
- **RTL**: Return to launch point autonomously
- **Guided**: Accept real-time navigation commands from GCS

### Problem

We need an architecture that:

- Provides common infrastructure for all control modes
- Handles mode lifecycle (initialization, execution, cleanup)
- Manages mode transitions with validation
- Executes active mode at 50 Hz control loop frequency
- Integrates with MAVLink protocol (DO_SET_MODE command)
- Allows adding new modes without modifying framework

### Constraints

- **Memory Budget**: Vehicle layer < 5 KB RAM (NFR-v6kvd)
- **Real-time**: Mode update must complete within 20ms (50 Hz)
- **no_std**: Must work without standard library (embedded Rust)
- **Safety**: Mode transitions must be atomic and validated

### Prior Art

**ArduPilot Mode Class**:

```cpp
class Mode {
public:
    virtual bool enter() = 0;    // Initialize mode
    virtual void update() = 0;   // Execute mode (called at 50 Hz)
    virtual void exit() = 0;     // Cleanup mode
    virtual const char* name() = 0;
};

class ModeManager {
    Mode* current_mode;
public:
    bool set_mode(Mode* new_mode);
    void update();
};
```

**PX4 Flight Mode Architecture**:

- Uses C++ inheritance with virtual functions
- Mode manager handles transitions and validation
- Separate mode classes for each flight mode

## Success Metrics

- **Extensibility**: Add new mode (e.g., Guided) without modifying framework code
- **Memory**: Vehicle layer RAM usage < 5 KB total
- **Performance**: Mode update latency < 1ms (leaves 19ms for actuator/sensor processing)
- **Reliability**: 100% of mode transitions either succeed or fail-safe to Manual

## Decision

**We will implement control modes using Rust trait-based polymorphism with a mode manager that owns the active mode and handles transitions.**

### Architecture

```
┌─────────────────────────────────────────────┐
│       Vehicle Control Task (50 Hz)          │
│  (Scheduler task executing active mode)     │
└───────────────┬─────────────────────────────┘
                │
┌───────────────▼─────────────────────────────┐
│          Mode Manager                       │
│  - Owns current mode (Box<dyn Mode>)        │
│  - Handles mode transitions                 │
│  - Validates pre-conditions                 │
│  - Executes active mode update()            │
└───────────────┬─────────────────────────────┘
                │
        ┌───────┴───────┬───────────┬──────────┐
        │               │           │          │
┌───────▼──────┐ ┌─────▼─────┐ ┌──▼────┐ ┌───▼────┐
│ ManualMode   │ │ HoldMode  │ │ Auto  │ │  RTL   │
│ (impl Trait) │ │ (impl)    │ │ (impl)│ │ (impl) │
└──────────────┘ └───────────┘ └───────┘ └────────┘
```

### Decision Drivers

1. **Rust Idiom**: Trait-based polymorphism is idiomatic Rust (vs C++ inheritance)
2. **Type Safety**: Traits enforce common interface at compile time
3. **Testability**: Each mode can be tested independently
4. **Extensibility**: New modes implement trait without changing framework
5. **Safety**: Single ownership (no shared mutable state, no race conditions)
6. **ArduPilot Compatibility**: Similar architecture pattern, proven design

### Considered Options

- **Option A: Trait + Mode Manager (Dynamic Dispatch)** ⭐ Selected
- **Option B: Enum-based State Machine**
- **Option C: Function Pointers Table**

### Option Analysis

**Option A: Trait + Mode Manager (Dynamic Dispatch)**

- **Pros**:
  - Clean abstraction, idiomatic Rust
  - Easy to add new modes (implement trait)
  - Each mode is separate module (good code organization)
  - Matches ArduPilot architecture (proven pattern)
  - Mode manager encapsulates transition logic
  - Testable: mock modes for unit tests
- **Cons**:
  - Dynamic dispatch overhead (\~1-2 CPU cycles per call)
  - Requires `Box<dyn Trait>` for heap allocation (or static lifetime)
  - Slightly higher memory (\~200 bytes per mode instance)
- **Estimated Overhead**: \~2 KB RAM, \~15 KB Flash

**Option B: Enum-based State Machine**

- **Pros**:
  - No dynamic dispatch (static dispatch, faster)
  - All modes in single module (simpler structure)
  - Minimal memory overhead
  - Easier to reason about (single match statement)
- **Cons**:
  - Large match statements in update loop (hard to read)
  - Adding new mode requires modifying enum and all match arms
  - Mode-specific state stored in enum variants (complex)
  - Harder to test modes in isolation
  - Not extensible (every change touches central code)
- **Estimated Overhead**: \~1 KB RAM, \~10 KB Flash

**Option C: Function Pointers Table**

- **Pros**:
  - Similar to C approach (familiar to embedded developers)
  - No trait object overhead
  - Lightweight
- **Cons**:
  - Less safe (function pointers can be null)
  - No compile-time interface enforcement
  - Awkward in Rust (not idiomatic)
  - Mode state management unclear
  - Hard to test
- **Estimated Overhead**: \~1.5 KB RAM, \~12 KB Flash

## Rationale

**Trait + Mode Manager** was selected for:

1. **Extensibility**: New modes (Guided, Loiter) can be added without modifying framework
2. **Type Safety**: Compiler enforces all modes implement required methods
3. **Code Organization**: Each mode is separate module, easy to maintain
4. **Testability**: Modes can be unit tested in isolation
5. **Rust Best Practice**: Trait-based polymorphism is idiomatic Rust

### Trade-offs Accepted

- **Dynamic Dispatch Cost**: \~1-2 cycles per call (acceptable, update() is not called frequently)
- **Memory**: \~2 KB for framework (within NFR-v6kvd 5 KB budget)

**Decision**: We accept the minimal overhead for better maintainability and extensibility.

## Consequences

### Positive

- **Clean Architecture**: Separation of concerns, mode logic isolated
- **Extensibility**: Easy to add new modes (Auto, RTL, Guided)
- **Maintainability**: Each mode is independent module
- **Type Safety**: Compiler enforces interface compliance
- **Testability**: Modes can be unit tested without hardware

### Negative

- **Dynamic Dispatch**: Minimal overhead (\~1-2 cycles per call)
- **Memory**: \~2 KB RAM for framework (acceptable within budget)
- **Learning Curve**: Developers must understand trait objects

### Neutral

- **ArduPilot Similarity**: Similar pattern, easier for developers familiar with ArduPilot
- **Rust Idiomatic**: Follows Rust best practices for polymorphism

## Implementation Notes

### Mode Trait

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

### Mode Manager

```rust
pub struct ModeManager {
    /// Current active mode
    current_mode: Box<dyn Mode>,
    /// System state (for mode reporting)
    system_state: &'static mut SystemState,
    /// Last update timestamp
    last_update_us: u64,
}

impl ModeManager {
    /// Create mode manager with initial mode (Manual)
    pub fn new(initial_mode: Box<dyn Mode>, system_state: &'static mut SystemState) -> Self {
        Self {
            current_mode: initial_mode,
            system_state,
            last_update_us: 0,
        }
    }

    /// Execute active mode (call at 50 Hz)
    pub fn execute(&mut self, current_time_us: u64) -> Result<(), &'static str> {
        // Calculate delta time
        let dt = if self.last_update_us > 0 {
            (current_time_us - self.last_update_us) as f32 / 1_000_000.0
        } else {
            0.02 // First iteration: assume 50 Hz (20ms)
        };
        self.last_update_us = current_time_us;

        // Update active mode
        self.current_mode.update(dt)?;
        Ok(())
    }

    /// Request mode change (from MAVLink command or internal logic)
    pub fn set_mode(&mut self, new_mode: Box<dyn Mode>) -> Result<(), &'static str> {
        // Exit current mode
        if let Err(e) = self.current_mode.exit() {
            defmt::warn!("Mode exit failed: {}", e);
            // Continue with transition (exit failure should not block mode change)
        }

        // Enter new mode
        if let Err(e) = new_mode.enter() {
            // Mode entry failed, revert to Manual (safe fallback)
            defmt::error!("Mode entry failed: {}, reverting to Manual", e);

            let manual_mode = ManualMode::new(/* dependencies */);
            if let Err(e2) = manual_mode.enter() {
                // Manual mode entry should never fail
                panic!("Manual mode entry failed: {}", e2);
            }

            self.current_mode = Box::new(manual_mode);
            self.system_state.set_mode(FlightMode::Manual);
            return Err(e);
        }

        // Update system state
        let mode_name = new_mode.name();
        self.current_mode = new_mode;
        // Note: Caller must update system_state.mode based on which mode was set

        defmt::info!("Mode changed to: {}", mode_name);
        // TODO: Send MAVLink STATUSTEXT: "Mode changed: Manual → Auto"

        Ok(())
    }

    /// Get current mode name
    pub fn current_mode_name(&self) -> &'static str {
        self.current_mode.name()
    }
}
```

### Manual Mode Example

```rust
pub struct ManualMode {
    rc_input: &'static RcInput,
    actuators: &'static mut Actuators,
}

impl Mode for ManualMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        defmt::info!("Entering Manual mode");
        Ok(())
    }

    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        // Check RC timeout
        if self.rc_input.is_lost() {
            // Failsafe: neutral outputs (handled by actuator layer)
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        }

        // Read RC inputs (normalized -1.0 to +1.0)
        let steering = self.rc_input.get_channel(1);
        let throttle = self.rc_input.get_channel(3);

        // Direct pass-through to actuators
        // Note: Actuator layer enforces armed check
        self.actuators.set_steering(steering)?;
        self.actuators.set_throttle(throttle)?;

        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        defmt::info!("Exiting Manual mode");
        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Manual"
    }
}
```

### Mode Transition Validation

```rust
impl ModeManager {
    /// Validate mode transition before attempting
    fn validate_transition(&self, target_mode: &dyn Mode) -> Result<(), &'static str> {
        match target_mode.name() {
            "Manual" => Ok(()), // Always allowed
            "Hold" => Ok(()),   // Always allowed
            "Auto" => {
                if !self.system_state.gps_has_fix() {
                    return Err("Auto mode requires GPS fix");
                }
                if !self.system_state.mission_valid() {
                    return Err("Auto mode requires valid mission");
                }
                Ok(())
            }
            "RTL" => {
                if !self.system_state.gps_has_fix() {
                    return Err("RTL mode requires GPS fix");
                }
                if !self.system_state.has_launch_point() {
                    return Err("RTL mode requires known launch point");
                }
                Ok(())
            }
            "Guided" => {
                if !self.system_state.gps_has_fix() {
                    return Err("Guided mode requires GPS fix");
                }
                Ok(())
            }
            _ => Err("Unknown mode"),
        }
    }
}
```

### Vehicle Control Task

```rust
#[embassy_executor::task]
async fn vehicle_control_task(
    mode_manager: &'static mut ModeManager,
) {
    let mut ticker = Ticker::every(Duration::from_millis(20)); // 50 Hz

    loop {
        let current_time_us = embassy_time::Instant::now().as_micros();

        if let Err(e) = mode_manager.execute(current_time_us) {
            // Log error, potentially switch to failsafe mode
            defmt::error!("Mode execution error: {}", e);
            // TODO: Trigger failsafe (Hold or Manual mode)
        }

        ticker.next().await;
    }
}
```

### MAVLink Integration

```rust
// In src/communication/mavlink/handlers/command.rs
fn handle_do_set_mode(
    mode_number: u32,
    mode_manager: &'static mut ModeManager,
) -> Result<(), &'static str> {
    let new_mode: Box<dyn Mode> = match mode_number {
        0 => Box::new(ManualMode::new(/* ... */)),
        1 => Box::new(StabilizeMode::new(/* ... */)), // Future
        2 => Box::new(HoldMode::new(/* ... */)),
        3 => Box::new(AutoMode::new(/* ... */)), // Future
        4 => Box::new(RtlMode::new(/* ... */)),   // Future
        _ => return Err("Invalid mode number"),
    };

    mode_manager.set_mode(new_mode)?;

    // Send COMMAND_ACK
    send_command_ack(MavCmd::DO_SET_MODE, MavResult::MAV_RESULT_ACCEPTED)?;

    Ok(())
}
```

### Module Structure

Following ArduPilot's architecture with common libraries and vehicle-specific implementations:

```
src/
├── libraries/          # Vehicle-agnostic libraries (ArduPilot libraries/)
│   ├── rc_channel/    # RC input processing (RC_Channel equivalent)
│   └── srv_channel/   # Servo output processing (SRV_Channel equivalent)
│
└── rover/              # Rover vehicle implementation (ArduPilot Rover/)
    ├── mod.rs          # Rover module root, exports Mode trait
    ├── mode_manager.rs # Mode manager implementation
    └── mode/
        ├── mod.rs      # Mode trait definition
        ├── manual.rs   # Manual mode
        ├── hold.rs     # Hold mode (future)
        ├── auto.rs     # Auto mode (future)
        └── rtl.rs      # RTL mode (future)
```

**Rationale**: Separates vehicle-agnostic libraries (RC, servo) from vehicle-specific control logic (modes), enabling code reuse for future vehicle types (Boat, Copter).

## Platform Considerations

- **Platform Agnostic**: Mode framework is hardware-independent
- **RP2040/RP2350**: No platform-specific considerations
- **Cross-Platform**: Abstraction via PwmInterface, RcInput traits

## Monitoring & Logging

- **Mode Transitions**: Log all mode changes with timestamp and reason
- **Mode Errors**: Log mode update errors (should be rare)
- **Performance**: Monitor mode update latency (should be < 1ms)
- **Telemetry**: Broadcast mode changes via MAVLink STATUSTEXT

## Open Questions

- [ ] Should mode manager support deferred transitions (wait for safe state)? → Decision: Defer to Phase 2, start with immediate transitions
- [ ] How to handle mode-specific parameters? → Method: Pass parameter storage to mode constructor
- [ ] Should modes share common navigation logic? → Next step: Create shared navigation module when implementing Auto/RTL

## External References

- ArduPilot Mode Class: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Vehicle/AP_Vehicle_Type.h>
- MAVLink DO_SET_MODE: <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE>
- Rust Trait Objects: <https://doc.rust-lang.org/book/ch17-02-trait-objects.html>
- ArduPilot Mode Switching: <https://ardupilot.org/rover/docs/common-flight-modes.html>

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
