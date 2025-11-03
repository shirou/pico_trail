# AN-yqeju Manual Control Implementation for Rover via Mission Planner

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-cp76d-ardupilot-analysis](AN-cp76d-ardupilot-analysis.md)
  - [AN-808o3-mavlink-network-transport](AN-808o3-mavlink-network-transport.md)
- Related Requirements:
  - [FR-sp3at-vehicle-modes](../requirements/FR-sp3at-vehicle-modes.md)
  - [FR-gpzpz-mavlink-protocol](../requirements/FR-gpzpz-mavlink-protocol.md)
- Related ADRs:
  - [ADR-w9zpl-vehicle-mode-architecture](../adr/ADR-w9zpl-vehicle-mode-architecture.md)
  - [ADR-ea7fw-rc-input-processing](../adr/ADR-ea7fw-rc-input-processing.md)
  - [ADR-b8snw-actuator-abstraction-rover](../adr/ADR-b8snw-actuator-abstraction-rover.md)
- Related Tasks:
  - [T-3irm5-manual-control-implementation](../tasks/T-3irm5-manual-control-implementation/README.md)

## Executive Summary

This analysis explores implementing Manual mode control for rover vehicles using RC_CHANNELS messages from Mission Planner over MAVLink. Manual mode is defined in FR-sp3at but not yet implemented. This capability enables direct operator control via ground control station (GCS) before implementing autonomous navigation, providing a foundation for testing actuators, validating communication, and enabling safe manual override. The implementation requires three new layers: (1) RC input processing from MAVLink RC_CHANNELS messages, (2) Vehicle mode implementation framework, and (3) Actuator abstraction for steering servo and throttle motor control.

Key findings: ArduPilot's Manual mode architecture separates RC decoding, vehicle mode logic, and actuator mixing. For rover vehicles, Manual mode performs simple pass-through mapping (RC channel 1 → steering, RC channel 3 → throttle) without stabilization. Mission Planner can send RC_CHANNELS messages when connected, simulating a physical RC receiver for testing and development.

## Problem Space

### Current State

The project currently has:

- **Mode definition**: `FlightMode::Manual` enum exists in `src/communication/mavlink/state.rs:41`
- **PWM interface**: `PwmInterface` trait defined for motor/servo control (`src/platform/traits/pwm.rs`)
- **MAVLink communication**: Message parsing and routing implemented (`src/communication/mavlink/`)
- **No vehicle layer**: Missing `src/vehicle/` module for mode implementations
- **No RC processing**: RC_CHANNELS messages defined but not processed
- **No actuator layer**: No abstraction for mixing steering/throttle to PWM outputs

Gaps:

- Cannot receive or process RC_CHANNELS from Mission Planner
- Manual mode cannot translate RC inputs to actuator commands
- No vehicle mode execution framework
- No actuator mixing logic (RC values → PWM duty cycles)

### Desired State

Enable manual control workflow:

1. **Operator connects** Mission Planner to autopilot via MAVLink (UDP or UART)
2. **Operator sends RC inputs** via Mission Planner's RC calibration screen or joystick interface
3. **Autopilot receives** RC_CHANNELS messages (MAVLink message ID 65)
4. **Manual mode processes** RC inputs and generates actuator commands
5. **Actuators respond**: Steering servo turns, throttle motor spins

Success criteria:

- Mission Planner joystick controls rover steering and throttle
- Steering response is immediate and proportional to stick input
- Throttle response matches stick position
- Mode transition to Manual mode works (via MAVLink COMMAND_LONG DO_SET_MODE)
- **Safety: ALL actuators disabled when disarmed** (no motor movement, servos centered)
- **Safety: Actuators immediately stop upon disarm** (< 20ms response time)

### Gap Analysis

**Missing components**:

1. **RC Input Handler**: Parse RC_CHANNELS messages, extract channel values
2. **Vehicle Mode Framework**: Infrastructure to run mode-specific logic
3. **Manual Mode Implementation**: Simple pass-through mapping (RC → actuator)
4. **Actuator Abstraction**: Interface for steering/throttle commands independent of PWM details
5. **Actuator Mixer**: Convert normalized commands (-1.0 to +1.0) to PWM duty cycles
6. **Safety Checks**: Prevent actuator output when disarmed

**Technical deltas**:

- Add `src/vehicle/` module with mode trait
- Implement RC_CHANNELS handler in MAVLink layer
- Create actuator abstraction trait
- Implement rover-specific actuator mixer (Ackermann steering)
- Add vehicle control task to scheduler
- Implement safety layer: armed state check before actuator output

## Stakeholder Analysis

| Stakeholder         | Interest/Need                                         | Impact | Priority |
| ------------------- | ----------------------------------------------------- | ------ | -------- |
| Developers          | Test hardware (servos, motors) before autonomy        | High   | P0       |
| System Integrators  | Validate MAVLink communication end-to-end             | High   | P0       |
| Operators           | Manual override for safety, testing autonomous modes  | High   | P0       |
| Hardware Validators | Confirm PWM outputs work correctly                    | Medium | P1       |
| Autonomous Features | Manual mode is prerequisite for mode switching system | High   | P0       |

## Research & Discovery

### User Feedback

From operational requirements:

- Manual mode is fundamental safety feature (FR-sp3at requirement)
- Physical RC receiver adds complexity and cost
- Mission Planner RC simulation sufficient for development and testing
- Manual override is critical safety requirement before autonomy

### Competitive Analysis

**ArduPilot Rover Manual Mode**:

ArduPilot implements Manual mode in `mode_manual.cpp`:

```cpp
void ModeManual::update()
{
    // Direct pass-through from RC to motors
    float steering = channel_steer->norm_input();  // -1.0 to +1.0
    float throttle = channel_throttle->norm_input(); // -1.0 to +1.0

    // Apply to motors (no stabilization)
    g2.motors.set_steering(steering);
    g2.motors.set_throttle(throttle);
}
```

Key characteristics:

- **No stabilization**: RC inputs directly control actuators
- **Simple mapping**: Channel values normalized to -1.0 to +1.0
- **No processing**: No filtering, rate limiting, or heading hold
- **Safety**: Throttle limited when disarmed

**RC Channel Configuration** (ArduPilot):

- **RCMAP_ROLL** (channel 1): Steering control
- **RCMAP_THROTTLE** (channel 3): Throttle control
- RC values: 1000-2000 μs (PWM pulse width)
- Neutral: 1500 μs
- Normalized: -1.0 (1000 μs) to +1.0 (2000 μs)

**PX4 Manual Mode**:

Similar architecture but different naming:

- `ManualControl` module receives RC inputs
- `VehicleAttitudeSetpoint` for stabilized modes
- Manual mode bypasses attitude control, direct actuator output

### Technical Investigation

**MAVLink RC_CHANNELS Message**:

Message ID: 65

```rust
pub struct RC_CHANNELS_DATA {
    pub time_boot_ms: u32,    // Timestamp (ms since boot)
    pub chan1_raw: u16,       // Channel 1 (steering), 0-65535
    pub chan2_raw: u16,       // Channel 2 (unused for rover)
    pub chan3_raw: u16,       // Channel 3 (throttle), 0-65535
    pub chan4_raw: u16,       // Channel 4 (mode switch, optional)
    // ... channels 5-18
    pub chancount: u8,        // Number of active channels
    pub rssi: u8,             // Signal strength (255 = no RC)
}
```

Channel value mapping:

- **Raw value range**: 0 to 65535
- **Standard RC range**: 1000-2000 μs mapped to 0-65535
- **Neutral**: 32768 (≈ 1500 μs)
- **Normalization**: `(raw - 32768) / 32768.0` → -1.0 to +1.0

**Mission Planner RC Simulation**:

Mission Planner can send RC_CHANNELS messages via:

1. **Joystick Input**: USB joystick/gamepad mapped to RC channels
2. **On-Screen Controls**: Slider widgets for each channel
3. **RC Calibration Screen**: Test RC input without physical transmitter

Configuration (Mission Planner):

- Joystick configuration: "Setup → Mandatory Hardware → Joystick"
- Channel mapping configurable (steering = axis X, throttle = axis Y)
- Sends RC_CHANNELS at 5-10 Hz when connected

**Rover Actuator Configuration**:

Typical rover setup:

- **Steering Servo**: PWM output, 50 Hz, 1000-2000 μs pulse width
  - Neutral: 1500 μs (straight ahead)
  - Left: 1000 μs
  - Right: 2000 μs
- **Throttle ESC**: PWM output, 50 Hz, 1000-2000 μs pulse width
  - Stop: 1500 μs
  - Forward: 1500-2000 μs
  - Reverse: 1500-1000 μs (if supported)

PWM duty cycle conversion:

- 50 Hz period = 20 ms
- 1000 μs pulse = 5% duty cycle
- 1500 μs pulse = 7.5% duty cycle
- 2000 μs pulse = 10% duty cycle

**Memory Analysis**:

| Component             | RAM Usage   | Notes                            |
| --------------------- | ----------- | -------------------------------- |
| RC input state        | \~100 B     | 18 channels × 2 bytes + metadata |
| Vehicle mode instance | \~200 B     | Mode state machine               |
| Actuator mixer        | \~100 B     | Steering/throttle calibration    |
| **Total**             | **\~400 B** | Negligible overhead              |

### Data Analysis

**Latency Requirements**:

- **Operator expectation**: < 100ms from stick input to actuator response
- **MAVLink latency**: 10-50ms (UART), 20-100ms (WiFi)
- **Processing latency**: < 10ms (mode update + actuator mixing)
- **PWM update**: 20ms (50 Hz cycle)
- **Total**: 50-180ms (acceptable for manual control)

**Update Rates**:

- **RC_CHANNELS rate**: 5-10 Hz from Mission Planner (configurable via SR_RC_CHAN parameter)
- **Mode update rate**: 50 Hz (control loop frequency, per NFR-ukjvr)
- **PWM output rate**: 50 Hz (servo standard frequency)

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The system shall receive and process RC_CHANNELS messages from MAVLink → Will become FR-<id>
  - Rationale: Enable RC input from Mission Planner without physical RC receiver
  - Acceptance Criteria:
    - Parse RC_CHANNELS messages (ID 65) from any active transport
    - Extract channel values (chan1_raw through chan18_raw)
    - Normalize channel values to -1.0 to +1.0 range
    - Provide RC input state to vehicle modes
    - Timeout detection: RC loss if no message for 1 second

- [ ] **FR-DRAFT-2**: The system shall implement Manual mode with direct RC-to-actuator pass-through → Will become FR-<id>
  - Rationale: Core requirement from FR-sp3at, foundation for all vehicle control
  - Acceptance Criteria:
    - Read RC channel 1 (steering) and channel 3 (throttle)
    - Map steering directly to steering actuator (-1.0 left, +1.0 right)
    - Map throttle directly to throttle actuator (-1.0 reverse, +1.0 forward)
    - No filtering or stabilization applied
    - **CRITICAL SAFETY: Zero outputs (steering centered, throttle zero) when disarmed**
    - **CRITICAL SAFETY: Check armed state before EVERY actuator command**
    - **CRITICAL SAFETY: Disarm immediately disables all actuators (< 20ms)**
    - Update rate: 50 Hz minimum

- [ ] **FR-DRAFT-3**: The system shall provide actuator abstraction for rover steering and throttle control → Will become FR-<id>
  - Rationale: Decouple vehicle logic from PWM hardware details
  - Acceptance Criteria:
    - Trait/interface for actuator commands: `set_steering(f32)`, `set_throttle(f32)`
    - Normalized input range: -1.0 to +1.0
    - Automatic PWM conversion (1000-2000 μs pulse width)
    - **CRITICAL SAFETY: Armed state check enforced at actuator layer (not mode layer)**
    - **CRITICAL SAFETY: Ignore all commands when disarmed, output neutral PWM**
    - **CRITICAL SAFETY: On disarm event, immediately set neutral PWM (1500 μs)**
    - Calibration: adjustable min/max/trim values per actuator

- [ ] **FR-DRAFT-4**: The system shall implement vehicle mode execution framework → Will become FR-<id>
  - Rationale: Infrastructure for all vehicle modes (Manual, Auto, RTL, etc.)
  - Acceptance Criteria:
    - Mode trait with `enter()`, `update()`, `exit()` methods
    - Mode manager handles mode transitions
    - Current mode executes at 50 Hz
    - Mode change requests via MAVLink COMMAND_LONG (DO_SET_MODE)
    - Log mode changes with timestamp

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Manual mode control latency shall not exceed 100ms from RC input to actuator response → Will become NFR-<id>
  - Category: Performance
  - Rationale: Operator expects immediate response for manual control
  - Target: < 100ms total latency (MAVLink + processing + PWM)

- [ ] **NFR-DRAFT-2**: Actuators shall fail-safe to neutral position (steering centered, throttle zero) when disarmed or RC lost → Will become NFR-<id>
  - Category: Safety (CRITICAL)
  - Rationale: Prevent unintended vehicle motion, primary safety mechanism
  - Target: Actuators neutral within 1 control loop cycle (20ms)
  - Enforcement: Armed check at actuator layer (defense in depth)
  - Testing: Automated test that disarms during full throttle, verify immediate stop

- [ ] **NFR-DRAFT-3**: Vehicle layer shall add no more than 5 KB RAM and 20 KB Flash overhead → Will become NFR-<id>
  - Category: Resource Constraints
  - Rationale: Maintain memory budget for other subsystems
  - Target: Measured via runtime profiling

## Design Considerations

### Technical Constraints

- **Existing architecture**: Must integrate with current scheduler, MAVLink, and platform layers
- **Memory budget**: Pico W has limited RAM (264 KB), vehicle layer must be lean
- **Real-time requirements**: 50 Hz control loop (per NFR-ukjvr)
- **Safety critical**: Actuator control must be fail-safe
- **Platform abstraction**: Must work on both RP2040 and RP2350

### Potential Approaches

1. **Option A: Minimal Vehicle Layer + Direct Mode Implementation**
   - Pros:
     - Simplest implementation (no complex abstraction)
     - Minimal code, easy to understand
     - Low memory overhead (\~1 KB)
     - Fast to implement
   - Cons:
     - No infrastructure for future modes (Auto, RTL)
     - Mode switching logic ad-hoc
     - Hard to test modes in isolation
     - Code duplication across modes
   - Effort: Low

2. **Option B: Mode Trait + Manager Pattern** ⭐ Recommended
   - Pros:
     - Clean abstraction: each mode implements common trait
     - Mode transitions handled by manager
     - Easy to add new modes (Auto, RTL, Guided)
     - Testable: modes can be unit tested
     - Follows Rust best practices (trait-based polymorphism)
     - Matches ArduPilot architecture (proven design)
   - Cons:
     - More upfront design effort
     - Slightly higher memory (\~3-5 KB for framework)
     - Trait object overhead (minimal with static dispatch)
   - Effort: Medium

3. **Option C: ArduPilot-Style Mode Hierarchy with Base Class**
   - Pros:
     - Maximum code reuse across modes
     - Shared functionality (e.g., common navigation logic)
     - Closest match to ArduPilot architecture
   - Cons:
     - Rust doesn't have inheritance (would need composition)
     - More complex design
     - Higher memory overhead
     - Overkill for initial implementation
   - Effort: High

**Recommendation**: Option B (Mode Trait + Manager) provides the best balance of simplicity and extensibility.

### Actuator Abstraction Approaches

1. **Option A: Direct PWM Control**
   - Modes directly call `pwm.set_duty_cycle()`
   - Pros: Simple, no abstraction overhead
   - Cons: Modes coupled to PWM details, hard to add actuator calibration

2. **Option B: Actuator Trait with Mixer** ⭐ Recommended
   - Trait: `ActuatorInterface` with `set_steering()`, `set_throttle()`
   - Mixer: Converts -1.0 to +1.0 → PWM duty cycle
   - Pros: Clean separation, calibration in one place, testable
   - Cons: Additional abstraction layer
   - Effort: Medium

**Recommendation**: Option B (Actuator Trait) provides proper separation and supports future enhancements (calibration, rate limiting).

### Architecture Impact

**New ADRs required**:

- **ADR-<id> Vehicle Mode Architecture**: Mode trait design, manager pattern, mode transitions
- **ADR-<id> Actuator Abstraction for Rover**: Steering/throttle interface, mixer design, PWM mapping
- **ADR-<id> RC Input Processing**: RC_CHANNELS parsing, normalization, timeout handling

**New modules**:

- `src/vehicle/` - Vehicle control layer
  - `src/vehicle/modes/` - Mode implementations (manual.rs, hold.rs, auto.rs, rtl.rs)
  - `src/vehicle/mode_manager.rs` - Mode switching logic
  - `src/vehicle/actuators.rs` - Actuator abstraction
  - `src/vehicle/rc_input.rs` - RC input processing

**Modified modules**:

- `src/communication/mavlink/handlers/` - Add RC_CHANNELS handler
- `src/communication/mavlink/handlers/command.rs` - Add DO_SET_MODE command
- `src/core/scheduler/tasks/` - Add vehicle control task

## Risk Assessment

| Risk                                            | Probability | Impact       | Mitigation Strategy                                                                                                   |
| ----------------------------------------------- | ----------- | ------------ | --------------------------------------------------------------------------------------------------------------------- |
| RC input timeout causes vehicle runaway         | Low         | High         | Implement RC loss detection, fail-safe to neutral                                                                     |
| **PWM output to motors while disarmed**         | **Medium**  | **CRITICAL** | **Multi-layer defense: (1) Armed check in mode layer, (2) Armed check in actuator layer, (3) Automated safety tests** |
| **Disarm command ignored, motors keep running** | **Low**     | **CRITICAL** | **Actuator layer subscribes to armed state changes, immediately disables on disarm event**                            |
| Mode trait overhead impacts performance         | Low         | Low          | Use static dispatch where possible, profile control loop timing                                                       |
| Mission Planner RC_CHANNELS format mismatch     | Low         | Medium       | Follow MAVLink spec exactly, test with real Mission Planner                                                           |
| Actuator calibration incorrect (servo limits)   | Medium      | Medium       | Provide safe defaults (1000-2000 μs), allow parameter tuning                                                          |

## Open Questions

- [ ] Should RC input timeout be configurable or fixed at 1 second? → Next step: Create FR with configurable timeout parameter (FS_RC_TIMEOUT)
- [ ] Do we need RC failsafe action (Hold mode vs neutral outputs)? → Method: Follow FR-sxsvw failsafe requirements, default to Hold mode
- [ ] Should actuator calibration be per-actuator or global? → Decision: Per-actuator (servo trim may differ), stored as parameters
- [ ] Do we support RC passthrough mode (bypass mode logic, direct PWM)? → Decision: No, out of scope for initial implementation
- [ ] Should RC input smoothing/filtering be applied? → Method: Start without filtering (simple), add if needed for noisy inputs

## Recommendations

### Immediate Actions

1. **Adopt Option B architecture**: Mode trait + manager pattern with actuator abstraction
2. **Follow ArduPilot patterns**: Proven design, well-documented, community familiar
3. **Start with Manual mode only**: Minimal viable implementation, other modes follow same pattern
4. **Implement safety checks**: Disarm check, RC timeout, neutral fail-safe

### Next Steps

1. [ ] Create formal requirements: FR-<id> (RC input), FR-<id> (Manual mode), FR-<id> (Actuator abstraction), FR-<id> (Mode framework), NFR-<id> (Latency), NFR-<id> (Fail-safe)
2. [ ] Draft ADR for: Vehicle mode architecture (trait design, manager pattern)
3. [ ] Draft ADR for: Actuator abstraction (trait interface, mixer design)
4. [ ] Draft ADR for: RC input processing (normalization, timeout)
5. [ ] Create task for: Implementation (vehicle layer + Manual mode)

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **Physical RC receiver support**: Only MAVLink RC_CHANNELS (no SBUS, PPM, etc.)
- **RC input filtering**: No smoothing or rate limiting (simple pass-through)
- **RC calibration UI**: Use fixed 1000-2000 μs range (no Mission Planner calibration wizard)
- **Advanced mixing**: No differential steering, crab steering, or skid-steer (Ackermann only)
- **RC failsafe actions**: Only neutral outputs (no automatic mode change to RTL)
- **RC override**: No mechanism to override Auto mode with RC (pure mode-based control)
- **Multiple RC sources**: Only one RC source at a time (no UART + MAVLink RC mixing)

## Appendix

### References

- ArduPilot Rover Manual Mode: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_manual.cpp>
- MAVLink RC_CHANNELS: <https://mavlink.io/en/messages/common.html#RC_CHANNELS>
- MAVLink DO_SET_MODE: <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE>
- ArduPilot RC Input: <https://ardupilot.org/copter/docs/common-rc-systems.html>
- Mission Planner Joystick Setup: <https://ardupilot.org/planner/docs/joystick.html>
- Servo PWM Standard: <https://www.servocity.com/how-do-servos-work>

### Raw Data

**RC_CHANNELS Message Example** (from Mission Planner):

```
RC_CHANNELS {
    time_boot_ms: 12345,
    chan1_raw: 32768,  // Steering neutral (1500 μs)
    chan2_raw: 32768,  // Unused
    chan3_raw: 32768,  // Throttle neutral (1500 μs)
    chan4_raw: 32768,  // Mode switch
    chancount: 8,
    rssi: 255          // No physical RC
}
```

**PWM Duty Cycle Calculation**:

```rust
// Convert normalized value (-1.0 to +1.0) to PWM duty cycle
fn normalized_to_duty_cycle(normalized: f32) -> f32 {
    // Map -1.0..+1.0 to 1000..2000 μs
    let pulse_us = 1500.0 + (normalized * 500.0);

    // Convert to duty cycle (50 Hz = 20 ms period)
    let period_us = 20_000.0;
    pulse_us / period_us
}

// Examples:
// -1.0 → 1000 μs → 0.05 (5% duty cycle)
//  0.0 → 1500 μs → 0.075 (7.5% duty cycle)
// +1.0 → 2000 μs → 0.10 (10% duty cycle)
```

**ArduPilot Parameters**:

ArduPilot Rover defines multiple parameters for controlling RC input, servo outputs, and vehicle modes:

1. **RC Input Mapping (RCMAP)**: Map RC channels to vehicle functions

| Parameter      | Default | Description                                    |
| -------------- | ------- | ---------------------------------------------- |
| RCMAP_ROLL     | 1       | Channel for roll/steering input                |
| RCMAP_PITCH    | 2       | Channel for pitch input (unused for rover)     |
| RCMAP_THROTTLE | 3       | Channel for throttle input                     |
| RCMAP_YAW      | 4       | Channel for yaw input (unused for basic rover) |

For Rover, typically:

- `RCMAP_ROLL = 1`: Channel 1 controls steering (left/right)
- `RCMAP_THROTTLE = 3`: Channel 3 controls throttle (forward/reverse)

2. **RC Input Calibration (RCx)**: Define PWM ranges for each RC channel

| Parameter    | Default | Range    | Description                                      |
| ------------ | ------- | -------- | ------------------------------------------------ |
| RCx_MIN      | 1000    | 800-2200 | Minimum PWM pulse width (μs)                     |
| RCx_MAX      | 2000    | 800-2200 | Maximum PWM pulse width (μs)                     |
| RCx_TRIM     | 1500    | 800-2200 | Neutral/center PWM pulse width (μs)              |
| RCx_DZ       | 0       | 0-200    | Deadzone around trim (±μs, no output)            |
| RCx_REVERSED | 0       | 0/1      | Reverse channel direction (0=normal, 1=reversed) |

Examples:

- `RC1_MIN=1000, RC1_MAX=2000, RC1_TRIM=1500`: Standard steering range
- `RC3_MIN=1000, RC3_MAX=2000, RC3_TRIM=1500, RC3_DZ=30`: Throttle with 30μs deadzone

3. **Servo Output Configuration (SERVOx)**: Define PWM outputs for motors/servos

| Parameter       | Default | Range    | Description                                  |
| --------------- | ------- | -------- | -------------------------------------------- |
| SERVOx_FUNCTION | 0       | 0-109+   | Output function assignment (see table below) |
| SERVOx_MIN      | 1100    | 800-2200 | Minimum PWM output (μs)                      |
| SERVOx_MAX      | 1900    | 800-2200 | Maximum PWM output (μs)                      |
| SERVOx_TRIM     | 1500    | 800-2200 | Neutral PWM output (μs)                      |
| SERVOx_REVERSED | 0       | 0/1      | Reverse output direction                     |

Common `SERVOx_FUNCTION` values for Rover:

| Value | Function       | Description                         |
| ----- | -------------- | ----------------------------------- |
| 0     | Disabled       | Output disabled (no PWM signal)     |
| 26    | GroundSteering | Steering servo (left/right)         |
| 70    | Throttle       | Main throttle motor/ESC             |
| 73    | ThrottleLeft   | Left motor for skid-steer vehicles  |
| 74    | ThrottleRight  | Right motor for skid-steer vehicles |

Default Rover configuration (separate steering + throttle):

- `SERVO1_FUNCTION=26` (GroundSteering): Steering servo on output 1
- `SERVO3_FUNCTION=70` (Throttle): Throttle ESC on output 3

4. **Motor/Servo Options**:

| Parameter    | Default | Description                                                 |
| ------------ | ------- | ----------------------------------------------------------- |
| MOT_PWM_TYPE | 0       | PWM output type: 0=Normal, 1-6=DShot variants, 7-8=Brushed  |
| MOT_PWM_FREQ | 50      | PWM frequency (Hz) for normal PWM (default 50Hz for servos) |

5. **Flight Mode Configuration**:

| Parameter | Default | Range | Description                                              |
| --------- | ------- | ----- | -------------------------------------------------------- |
| MODE_CH   | 8       | 1-16  | RC channel for mode selection (Rover default: channel 8) |
| MODE1     | 0       | 0-20+ | Flight mode for PWM < 1230μs                             |
| MODE2     | 0       | 0-20+ | Flight mode for PWM 1231-1360μs                          |
| MODE3     | 0       | 0-20+ | Flight mode for PWM 1361-1490μs                          |
| MODE4     | 0       | 0-20+ | Flight mode for PWM 1491-1620μs                          |
| MODE5     | 0       | 0-20+ | Flight mode for PWM 1621-1749μs                          |
| MODE6     | 0       | 0-20+ | Flight mode for PWM > 1750μs                             |

Common MODE values for Rover:

| Value | Mode     | Description                                         |
| ----- | -------- | --------------------------------------------------- |
| 0     | Manual   | Direct RC control (steering + throttle passthrough) |
| 3     | Steering | Heading hold with manual throttle                   |
| 4     | Hold     | Stop and hold position                              |
| 10    | Auto     | Follow mission waypoints                            |
| 15    | Guided   | Move to commanded position (GCS/companion computer) |
| 11    | RTL      | Return to launch point                              |
| 12    | SmartRTL | Return via intelligent path                         |
| 7     | Circle   | Circle current position                             |
| 5     | Loiter   | Circle current position (alias for Circle)          |
| 16    | Acro     | Rate-controlled steering (for advanced users)       |

Example configuration for 3-position mode switch:

- `MODE_CH=8`: Use channel 8 for mode selection
- `MODE1=0` (Manual): Position 1 (< 1230μs)
- `MODE3=4` (Hold): Position 2 (1361-1490μs)
- `MODE6=10` (Auto): Position 3 (> 1750μs)

**Mode Trait Design Sketch**:

```rust
/// Vehicle mode trait
pub trait VehicleMode {
    /// Initialize mode (called once on mode entry)
    fn enter(&mut self) -> Result<(), &'static str>;

    /// Update mode (called at 50 Hz)
    fn update(&mut self, dt: f32) -> Result<(), &'static str>;

    /// Cleanup mode (called once on mode exit)
    fn exit(&mut self) -> Result<(), &'static str>;

    /// Get mode name
    fn name(&self) -> &'static str;
}

/// Manual mode implementation
pub struct ManualMode {
    rc_input: &RcInput,
    actuators: &Actuators,
}

impl VehicleMode for ManualMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        // No initialization needed
        Ok(())
    }

    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        // Read RC inputs
        let steering = self.rc_input.channel(1); // -1.0 to +1.0
        let throttle = self.rc_input.channel(3);

        // Direct pass-through to actuators
        // Note: Actuator layer enforces armed check (defense in depth)
        self.actuators.set_steering(steering)?;
        self.actuators.set_throttle(throttle)?;

        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        // Set actuators to neutral
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Manual"
    }
}

/// Actuator abstraction with safety enforcement
pub struct Actuators {
    steering_pwm: &mut dyn PwmInterface,
    throttle_pwm: &mut dyn PwmInterface,
    system_state: &SystemState,
}

impl Actuators {
    /// Set steering command (-1.0 left, +1.0 right)
    /// SAFETY: Enforces armed check, outputs neutral if disarmed
    pub fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str> {
        // CRITICAL SAFETY CHECK: Only output if armed
        let output = if self.system_state.is_armed() {
            normalized
        } else {
            0.0 // Neutral steering when disarmed
        };

        // Convert to PWM duty cycle (1000-2000 μs)
        let duty = Self::normalized_to_duty_cycle(output);
        self.steering_pwm.set_duty_cycle(duty)?;
        Ok(())
    }

    /// Set throttle command (-1.0 reverse, +1.0 forward)
    /// SAFETY: Enforces armed check, outputs neutral if disarmed
    pub fn set_throttle(&mut self, normalized: f32) -> Result<(), &'static str> {
        // CRITICAL SAFETY CHECK: Only output if armed
        let output = if self.system_state.is_armed() {
            normalized
        } else {
            0.0 // Zero throttle when disarmed
        };

        // Convert to PWM duty cycle (1000-2000 μs)
        let duty = Self::normalized_to_duty_cycle(output);
        self.throttle_pwm.set_duty_cycle(duty)?;
        Ok(())
    }

    /// Convert normalized value (-1.0 to +1.0) to PWM duty cycle
    fn normalized_to_duty_cycle(normalized: f32) -> f32 {
        // Clamp to valid range
        let clamped = normalized.clamp(-1.0, 1.0);

        // Map -1.0..+1.0 to 1000..2000 μs
        let pulse_us = 1500.0 + (clamped * 500.0);

        // Convert to duty cycle (50 Hz = 20 ms period)
        let period_us = 20_000.0;
        pulse_us / period_us
    }
}
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
