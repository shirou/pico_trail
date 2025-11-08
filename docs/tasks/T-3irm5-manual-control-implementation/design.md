# T-3irm5 Manual Control Implementation

## Metadata

- Type: Design
- Status: Complete

## Links

- Associated Plan Document:
  - [plan.md](./plan.md)

## Overview

This task implements manual control capability for rover vehicles by creating a complete vehicle control infrastructure: RC input processing from MAVLink RC_CHANNELS messages, trait-based control mode framework with lifecycle management, actuator abstraction with safety enforcement, and Manual mode implementation. The system enables operator control via Mission Planner joystick/gamepad over MAVLink, providing the foundation for all future control modes (Auto, RTL, Guided). The expected outcome is production-grade manual control with multi-layer safety enforcement (armed state checking, RC timeout detection) and compatibility with industry-standard ground control stations.

## Success Metrics

- [ ] RC input to actuator response latency < 100ms (NFR-kqvyf)
- [ ] 100% of actuator commands enforce armed state check (no bypasses)
- [ ] Actuators neutral within 20ms of disarm event (NFR-jng15)
- [ ] Vehicle layer memory usage < 5 KB RAM (NFR-v6kvd)
- [ ] Mission Planner joystick controls steering and throttle correctly
- [ ] RC timeout detected within 1 second, fail-safe to neutral outputs

## Background and Current State

- Context: The autopilot currently lacks control modes. Manual mode is the foundational capability required before implementing autonomous modes (Auto, RTL). Manual control enables hardware validation, actuator testing, and provides safety override capability for all autonomous operations.
- Current behavior:
  - Mode definition exists: `FlightMode::Manual` enum in `src/communication/mavlink/state.rs:41`
  - PWM interface: `PwmInterface` trait defined in `src/platform/traits/pwm.rs`
  - MAVLink communication: Message parsing and routing in `src/communication/mavlink/`
  - No libraries layer: Missing `src/libraries/` for RC/servo processing
  - No rover layer: Missing `src/rover/` module for mode implementations
  - No RC processing: RC_CHANNELS messages defined but not handled
  - No actuator layer: No abstraction between modes and PWM hardware
- Pain points:
  - Cannot manually control vehicle (no RC input processing)
  - Cannot test actuators without writing platform-specific PWM code
  - No safety enforcement for actuator commands (armed state checking)
  - No infrastructure for adding new control modes
- Constraints:
  - Memory budget: Vehicle layer < 5 KB RAM total (NFR-v6kvd)
  - Real-time: Mode update must complete within 20ms (50 Hz control loop)
  - Safety critical: Actuators must be neutral when disarmed (NFR-jng15)
  - Platform: RP2040/RP2350, no_std embedded environment
- Related ADRs:
  - [ADR-w9zpl-control-mode-architecture](../../adr/ADR-w9zpl-control-mode-architecture.md)
  - [ADR-b8snw-actuator-abstraction-rover](../../adr/ADR-b8snw-actuator-abstraction-rover.md)
  - [ADR-ea7fw-rc-input-processing](../../adr/ADR-ea7fw-rc-input-processing.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│              Mission Planner / Ground Control               │
│         (Joystick → RC_CHANNELS MAVLink message)            │
└───────────────────────┬─────────────────────────────────────┘
                        │ RC_CHANNELS (5-10 Hz)
                        ▼
┌─────────────────────────────────────────────────────────────┐
│               MAVLink Message Router                        │
│         (Route incoming messages to handlers)               │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│            RC_CHANNELS Handler                              │
│  - Parse RC_CHANNELS message                                │
│  - Normalize channels (0-65535 → -1.0 to +1.0)              │
│  - Update RcInput (Mutex-protected shared state)            │
│  - Update timestamp for timeout detection                   │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
                ┌───────────────┐
                │   RcInput     │
                │ (Shared State)│
                │ - Channels[18]│
                │ - Timestamp   │
                │ - Status      │
                └───────┬───────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│         Control Loop Task (50 Hz Embassy Task)              │
│  (Vehicle-agnostic: Rover, Boat, Copter)                    │
│  - Check RC timeout (current time - last update > 1s)       │
│  - Execute mode_manager.execute()                           │
│  - Calculate delta time (dt)                                │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                   Mode Manager                              │
│  - Owns current mode (Box<dyn Mode>)                 │
│  - Handles mode transitions (exit → validate → enter)       │
│  - Executes active mode update(dt) at 50 Hz                 │
└───────────────────────┬─────────────────────────────────────┘
                        │
                ┌───────┴───────┬───────────┬──────────┐
                │               │           │          │
                ▼               ▼           ▼          ▼
        ┌───────────┐   ┌─────────┐ ┌──────────┐ ┌────────┐
        │ManualMode │   │HoldMode │ │AutoMode  │ │RTLMode │
        │(impl Trait│   │(future) │ │(future)  │ │(future)│
        └─────┬─────┘   └─────────┘ └──────────┘ └────────┘
              │
              │ VehicleMode::update(dt)
              │
              ▼
    ┌─────────────────────┐
    │  Manual Mode Logic  │
    │  - Read RC inputs   │
    │  - Check RC timeout │
    │  - Pass-through     │
    │    to actuators     │
    └─────────┬───────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────┐
│              Actuator Abstraction Layer                     │
│  - set_steering(normalized: f32)                            │
│  - set_throttle(normalized: f32)                            │
│  - SAFETY: Enforce armed state check                        │
│  - Convert normalized (-1.0 to +1.0) → PWM (1000-2000 μs)   │
│  - Apply calibration (min/max/trim)                         │
└───────────────────────┬─────────────────────────────────────┘
                        │ PWM duty cycle (5-10%)
                        ▼
┌─────────────────────────────────────────────────────────────┐
│              Platform PWM Interface                         │
│  (PwmInterface trait, RP2040/RP2350 hardware)               │
└─────────────────────────────────────────────────────────────┘
```

### Components

#### 1. RC Input State (`src/libraries/rc_channel/mod.rs`)

```rust
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

/// RC input state (shared between MAVLink handler and vehicle task)
pub struct RcInput {
    /// Channel values (normalized -1.0 to +1.0)
    pub channels: [f32; 18],
    /// Number of active channels
    pub channel_count: u8,
    /// Last update timestamp (microseconds)
    pub last_update_us: u64,
    /// RC connection status
    pub status: RcStatus,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RcStatus {
    Active,          // RC input active, recent message received
    Lost,            // RC input lost, timeout exceeded
    NeverConnected,  // RC never connected
}

impl RcInput {
    /// Update from MAVLink RC_CHANNELS message
    pub fn update_from_mavlink(
        &mut self,
        msg: &mavlink::common::RC_CHANNELS_DATA,
        current_time_us: u64,
    );

    /// Normalize channel (0-65535 → -1.0 to +1.0)
    fn normalize_channel(raw: u16) -> f32;

    /// Get channel value (1-indexed, like MAVLink)
    pub fn get_channel(&self, channel: usize) -> f32;

    /// Check timeout (call at 50 Hz from vehicle task)
    pub fn check_timeout(&mut self, current_time_us: u64);

    /// Check if RC is active
    pub fn is_active(&self) -> bool;

    /// Check if RC is lost
    pub fn is_lost(&self) -> bool;
}

// Global RC input (protected by Mutex)
pub static RC_INPUT: Mutex<CriticalSectionRawMutex, RcInput> = Mutex::new(RcInput::new());
```

#### 2. Control Mode Trait (`src/rover/mode/mod.rs`)

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

#### 3. Mode Manager (`src/rover/mode_manager.rs`)

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
    pub fn new(
        initial_mode: Box<dyn Mode>,
        system_state: &'static mut SystemState,
    ) -> Self;

    /// Execute active mode (call at 50 Hz)
    pub fn execute(&mut self, current_time_us: u64) -> Result<(), &'static str>;

    /// Request mode change (from MAVLink command or internal logic)
    pub fn set_mode(&mut self, new_mode: Box<dyn Mode>) -> Result<(), &'static str>;

    /// Get current mode name
    pub fn current_mode_name(&self) -> &'static str;
}
```

#### 4. Actuator Abstraction (`src/libraries/srv_channel/mod.rs`)

```rust
/// Actuator abstraction for rover steering and throttle
pub trait ActuatorInterface {
    /// Set steering command (-1.0 left, 0.0 center, +1.0 right)
    /// SAFETY: Enforces armed check, outputs neutral if disarmed
    fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str>;

    /// Set throttle command (-1.0 reverse, 0.0 stop, +1.0 forward)
    /// SAFETY: Enforces armed check, outputs neutral if disarmed
    fn set_throttle(&mut self, normalized: f32) -> Result<(), &'static str>;

    /// Get current steering value
    fn get_steering(&self) -> f32;

    /// Get current throttle value
    fn get_throttle(&self) -> f32;
}

pub struct Actuators {
    steering_pwm: &'static mut dyn PwmInterface,
    throttle_pwm: &'static mut dyn PwmInterface,
    system_state: &'static SystemState,
    config: ActuatorConfig,
    current_steering: f32,
    current_throttle: f32,
}

pub struct ActuatorConfig {
    // Steering calibration (pulse width in μs)
    pub steering_min: u16,     // Default: 1000
    pub steering_neutral: u16, // Default: 1500
    pub steering_max: u16,     // Default: 2000

    // Throttle calibration (pulse width in μs)
    pub throttle_min: u16,     // Default: 1000
    pub throttle_neutral: u16, // Default: 1500
    pub throttle_max: u16,     // Default: 2000
}
```

#### 5. Manual Mode (`src/rover/mode/manual.rs`)

```rust
pub struct ManualMode {
    rc_input: &'static Mutex<CriticalSectionRawMutex, RcInput>,
    actuators: &'static mut dyn ActuatorInterface,
}

impl Mode for ManualMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        defmt::info!("Entering Manual mode");
        Ok(())
    }

    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        // Lock RC input briefly
        let rc = self.rc_input.lock().await;

        // Check RC timeout
        if rc.is_lost() {
            // Fail-safe: neutral outputs
            drop(rc);
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        }

        // Read RC channels (1-indexed)
        let steering = rc.get_channel(1); // Channel 1: steering
        let throttle = rc.get_channel(3); // Channel 3: throttle
        drop(rc); // Release lock

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

### Data Flow

1. **RC Input Reception**:
   - Mission Planner sends RC_CHANNELS at 5-10 Hz
   - MAVLink transport receives message
   - RC_CHANNELS handler parses message
   - Handler normalizes 18 channels (0-65535 → -1.0 to +1.0)
   - Handler locks RcInput Mutex and updates state
   - Handler updates timestamp for timeout detection

2. **Mode Execution (50 Hz)**:
   - Control loop task wakes up every 20ms (50 Hz)
   - Task locks RcInput and checks timeout (current_time - last_update > 1s)
   - If timeout: set RcStatus::Lost, zero all channels
   - Task calls mode_manager.execute(current_time_us)
   - Mode manager calculates delta time (dt)
   - Mode manager calls current_mode.update(dt)

3. **Manual Mode Processing**:
   - Lock RcInput, check if RC is lost
   - If lost: unlock, set neutral actuator commands, return
   - If active: read channel 1 (steering), channel 3 (throttle)
   - Unlock RcInput
   - Call actuators.set_steering(steering)
   - Call actuators.set_throttle(throttle)

4. **Actuator Command Processing**:
   - Actuator layer checks system_state.is_armed()
   - If disarmed: override normalized value to 0.0 (neutral)
   - If armed: clamp normalized value to \[-1.0, +1.0]
   - Convert normalized to PWM pulse width (1000-2000 μs)
   - Convert pulse width to duty cycle (5-10% for 50 Hz PWM)
   - Call platform PWM interface set_duty_cycle()

### Data Models and Types

```rust
// RC Input
pub struct RcInput {
    pub channels: [f32; 18],
    pub channel_count: u8,
    pub last_update_us: u64,
    pub status: RcStatus,
}

// Mode Manager
pub struct ModeManager {
    current_mode: Box<dyn Mode>,
    system_state: &'static mut SystemState,
    last_update_us: u64,
}

// Actuator Config
pub struct ActuatorConfig {
    pub steering_min: u16,
    pub steering_neutral: u16,
    pub steering_max: u16,
    pub throttle_min: u16,
    pub throttle_neutral: u16,
    pub throttle_max: u16,
}

// Actuators
pub struct Actuators {
    steering_pwm: &'static mut dyn PwmInterface,
    throttle_pwm: &'static mut dyn PwmInterface,
    system_state: &'static SystemState,
    config: ActuatorConfig,
    current_steering: f32,
    current_throttle: f32,
}
```

### Error Handling

- **RC Timeout**: Set RcStatus::Lost, zero all channels, log warning
- **Mode Entry Failure**: Revert to Manual mode (safe fallback), log error, send MAVLink STATUSTEXT
- **Mode Update Error**: Log error, continue execution (fail gracefully)
- **Actuator Command Error**: Log error, continue (PWM errors are typically non-fatal)
- **Mode Transition Validation**: Reject invalid transitions (e.g., Auto without GPS), send COMMAND_ACK with error

### Security Considerations

- No sensitive data in RC channels (public information)
- Armed state check prevents unauthorized actuator control
- RC_CHANNELS accepted from any authenticated MAVLink source (same as other MAVLink messages)

### Performance Considerations

- **RC Input Lock Contention**: RC input lock held briefly (<100 μs) to minimize contention
- **Mode Update Latency**: Target < 1ms per update (leaves 19ms for other tasks)
- **Actuator Conversion**: Simple linear interpolation, \~10 μs per command
- **Memory**: Total vehicle layer \~4 KB RAM (within 5 KB budget)
- **Hot Paths**:
  - Mode update: called at 50 Hz (critical path)
  - RC timeout check: called at 50 Hz (critical path)
  - Actuator commands: called at 50 Hz (critical path)
  - RC_CHANNELS handler: called at 5-10 Hz (non-critical)

### Module Structure

Following ArduPilot's architecture with vehicle-agnostic libraries and vehicle-specific implementations:

```
src/
├── libraries/              # Common libraries (ArduPilot libraries/)
│   ├── mod.rs
│   ├── rc_channel/        # RC input processing (RC_Channel equivalent)
│   │   └── mod.rs         # RcInput, RC_INPUT global, normalization
│   └── srv_channel/       # Servo output processing (SRV_Channel equivalent)
│       └── mod.rs         # ActuatorInterface, Actuators, calibration
│
├── rover/                  # Rover vehicle implementation (ArduPilot Rover/)
│   ├── mod.rs
│   ├── mode/              # Control mode implementations
│   │   ├── mod.rs         # Mode trait definition
│   │   └── manual.rs      # ManualMode (Phase 1)
│   └── mode_manager.rs    # ModeManager (Phase 2)
│
└── communication/mavlink/handlers/
    └── rc_input.rs        # RC_CHANNELS message handler
```

**Design Rationale**:

- `libraries/`: Vehicle-agnostic functionality shared across Rover, Boat, Copter
- `rover/`: Rover-specific control logic and modes
- Follows ArduPilot's separation between common libraries and vehicle types
- Enables future vehicle types (Boat, Copter) to reuse RC/servo libraries

### Platform Considerations

#### RP2040 (Pico W)

- PWM hardware: 8 PWM slices (16 channels total)
- Steering servo: GPIO configured as PWM output (e.g., GPIO 16)
- Throttle ESC: GPIO configured as PWM output (e.g., GPIO 17)
- 50 Hz PWM frequency: Standard for servos and ESCs
- No platform-specific considerations for vehicle layer (abstracted via traits)

#### RP2350 (Pico 2 W)

- PWM hardware: 12 PWM slices (24 channels total)
- Same PWM interface as RP2040
- No platform-specific considerations for vehicle layer

#### Cross-Platform

- Vehicle layer is fully platform-independent
- Platform-specific details handled by:
  - `PwmInterface` trait (platform PWM driver)
  - `SystemState` (platform-agnostic state)

## Alternatives Considered

### Alternative A: Enum-based State Machine

- Pros: No dynamic dispatch, simpler memory layout, faster
- Cons: Large match statements, hard to extend, mode state management complex
- Decision: Rejected in favor of trait-based approach for extensibility (ADR-w9zpl)

### Alternative B: Direct PWM Control from Modes

- Pros: Simplest approach, no actuator abstraction layer
- Cons: Modes coupled to PWM hardware, armed check duplicated, hard to test
- Decision: Rejected in favor of actuator abstraction for safety and testability (ADR-b8snw)

### Alternative C: Physical RC Receiver (SBUS/PPM)

- Pros: Low latency, high update rate (50+ Hz), works offline
- Cons: Requires additional hardware, platform-specific, higher complexity
- Decision: Deferred to future enhancement, MAVLink RC sufficient for initial implementation (ADR-ea7fw)

### Decision Rationale

Chosen approach (Trait + Mode Manager + Actuator Abstraction) provides:

- **Extensibility**: Easy to add new modes (Auto, RTL, Guided)
- **Safety**: Single enforcement point for armed check (actuator layer)
- **Testability**: Modes can be unit tested with mock actuators
- **Maintainability**: Clear separation of concerns (modes, actuators, RC input)

## Migration and Compatibility

- **Backward compatibility**: N/A (new feature, no existing vehicle control code)
- **Forward compatibility**: Designed to support future modes (Auto, RTL, Guided) without breaking changes
- **Parameter compatibility**: Actuator calibration parameters follow ArduPilot naming (SERVO_STEER_MIN, etc.)

## Testing Strategy

### Unit Tests

#### RC Input Tests

- [ ] Test channel normalization (0 → -1.0, 32768 → 0.0, 65535 → +1.0)
- [ ] Test RC timeout detection (active → lost after 1 second)
- [ ] Test channel access (valid/invalid channel numbers)

#### Actuator Tests

- [ ] Test normalized to PWM conversion (boundary values)
- [ ] Test armed state enforcement (disarmed → neutral outputs)
- [ ] Test calibration application (min/max/trim)
- [ ] Test duty cycle conversion (1000 μs = 5%, 1500 μs = 7.5%, 2000 μs = 10%)

#### Mode Manager Tests

- [ ] Test mode execution (delta time calculation)
- [ ] Test mode transitions (exit → enter)
- [ ] Test mode entry failure (revert to Manual)
- [ ] Test mode validation (GPS requirement for Auto)

#### Manual Mode Tests (with Mock Actuators)

- [ ] Test RC pass-through (steering/throttle mapping)
- [ ] Test RC timeout handling (neutral outputs)
- [ ] Test disarmed behavior (neutral outputs regardless of RC input)

### Integration Tests

- [ ] **Critical Safety Test**: Command full throttle while disarmed, verify neutral output
- [ ] **Critical Safety Test**: Disarm during full throttle, verify immediate neutral output (< 20ms)
- [ ] **RC Timeout Test**: Stop sending RC_CHANNELS, verify timeout within 1 second
- [ ] **Mode Transition Test**: Switch Manual → Hold → Manual, verify clean transitions
- [ ] **End-to-End Test**: Send RC_CHANNELS via MAVLink, verify actuator PWM output

### External API Parsing (if applicable)

N/A - No external API parsing (MAVLink message parsing tested separately)

### Performance & Benchmarks (if applicable)

- [ ] Benchmark mode update latency (target < 1ms)
- [ ] Benchmark RC input lock contention (target < 100 μs)
- [ ] Benchmark actuator conversion (target < 10 μs per command)
- [ ] Profile vehicle layer memory usage (target < 5 KB RAM)

## Documentation Impact

- Update `docs/architecture.md`: Add vehicle layer section with module structure
- Update `docs/mavlink.md`: Add RC_CHANNELS handler documentation
- Add code comments: Inline documentation for VehicleMode trait, safety checks
- No user-facing documentation (internal autopilot feature)

## External References

- ArduPilot Rover Manual Mode: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_manual.cpp>
- MAVLink RC_CHANNELS: <https://mavlink.io/en/messages/common.html#RC_CHANNELS>
- MAVLink DO_SET_MODE: <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE>
- Servo PWM Standard: <https://www.servocity.com/how-do-servos-work>
- Rust Trait Objects: <https://doc.rust-lang.org/book/ch17-02-trait-objects.html>

## Open Questions

- [ ] Should RC timeout threshold be configurable via parameter (FS_RC_TIMEOUT)? → Next step: Decide in plan.md, default 1000ms acceptable for initial implementation
- [ ] Should we implement RC input smoothing/filtering for noisy inputs? → Method: Start without filtering, add if needed based on testing
- [ ] Should actuator commands be rate-limited? → Decision: No, modes control update rate (50 Hz), defer to future enhancement

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
