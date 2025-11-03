# T-3irm5 Manual Control Implementation

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](./design.md)

## Overview

Implement manual control capability for rover vehicles by creating complete vehicle control infrastructure: RC input processing from MAVLink RC_CHANNELS messages, trait-based vehicle mode framework with lifecycle management, actuator abstraction with safety enforcement, and Manual mode implementation. This enables operator control via Mission Planner joystick and provides foundation for all future autonomous modes.

## Success Metrics

- [ ] RC input to actuator response latency < 100ms (NFR-kqvyf)
- [ ] 100% of actuator commands enforce armed state check (automated test required)
- [ ] Actuators neutral within 20ms of disarm event (NFR-jng15)
- [ ] Vehicle layer memory usage < 5 KB RAM (NFR-v6kvd)
- [ ] Mission Planner joystick controls steering and throttle correctly
- [ ] RC timeout detected within 1 second, fail-safe to neutral outputs
- [ ] All tests pass: `cargo fmt`, `cargo clippy`, `cargo test --lib --quiet`

## Scope

- Goal: Production-grade manual control with multi-layer safety enforcement
- Non-Goals:
  - Physical RC receiver support (SBUS/PPM) - MAVLink RC only
  - Other vehicle modes (Hold, Auto, RTL, Guided) - future tasks
  - RC input filtering/smoothing - simple pass-through initially
  - Advanced actuator features (rate limiting, deadband) - deferred
  - Differential/skid-steer mixing - Ackermann only
- Assumptions:
  - Mission Planner or QGroundControl available for testing
  - RP2350 Pico 2 W hardware available with servo/ESC connected
  - MAVLink communication already working (UART/UDP)
- Constraints:
  - Memory budget: Vehicle layer < 5 KB RAM total
  - Real-time: Mode update < 20ms (50 Hz control loop)
  - Safety critical: Actuators must be neutral when disarmed

## ADR & Legacy Alignment

- [x] Confirm the latest ADRs are referenced:
  - [ADR-w9zpl-vehicle-mode-architecture](../../adr/ADR-w9zpl-vehicle-mode-architecture.md) - Trait-based mode framework
  - [ADR-b8snw-actuator-abstraction-rover](../../adr/ADR-b8snw-actuator-abstraction-rover.md) - Actuator interface
  - [ADR-ea7fw-rc-input-processing](../../adr/ADR-ea7fw-rc-input-processing.md) - RC_CHANNELS handling
- [ ] Legacy patterns to address:
  - Current `FlightMode::Manual` enum exists but not implemented
  - No vehicle layer module structure yet (need to create `src/vehicle/`)
  - MAVLink handlers need RC_CHANNELS processing added
  - System state needs mode reporting integration

## Plan Summary

- Phase 1 – Foundation: RC Input + Actuator Abstraction
- Phase 2 – Mode Framework: VehicleMode trait, Mode Manager, Vehicle Control Task
- Phase 3 – Manual Mode: Implementation, Integration, MAVLink DO_SET_MODE
- Phase 4 – Testing & Validation: Safety tests, hardware validation, end-to-end testing

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: Foundation (RC Input + Actuator Abstraction)

### Goal

- Create RC input state structure with MAVLink RC_CHANNELS handler
- Implement actuator abstraction with safety enforcement
- Verify RC input normalization and timeout detection
- Verify actuator PWM conversion and armed state checking

### Inputs

- Documentation:
  - `docs/adr/ADR-ea7fw-rc-input-processing.md` – RC input design
  - `docs/adr/ADR-b8snw-actuator-abstraction-rover.md` – Actuator design
  - `docs/requirements/FR-993xy-rc-channels-processing.md` – RC requirements
  - `docs/requirements/FR-uo1p5-actuator-abstraction.md` – Actuator requirements
- Source Code to Modify:
  - `src/communication/mavlink/handlers/` – Add RC_CHANNELS handler
  - `src/communication/mavlink/state.rs` – System state (armed state)
  - `src/platform/traits/pwm.rs` – PWM interface trait
- Dependencies:
  - Internal: `src/communication/mavlink/` – MAVLink infrastructure
  - Internal: `src/platform/` – PWM platform interface
  - External crates: `embassy-sync` (Mutex), `defmt` (logging)

### Tasks

- [ ] **Create vehicle module structure**
  - [ ] Create `src/vehicle/mod.rs` with module documentation
  - [ ] Add `pub mod vehicle;` to `src/lib.rs`
  - [ ] Export public types: `RcInput`, `ActuatorInterface`, `Actuators`
- [ ] **Implement RC input state**
  - [ ] Create `src/vehicle/rc_input.rs`
  - [ ] Define `RcInput` struct (channels\[18], channel_count, last_update_us, status)
  - [ ] Define `RcStatus` enum (Active, Lost, NeverConnected)
  - [ ] Implement `RcInput::new()` constructor
  - [ ] Implement `RcInput::normalize_channel(raw: u16) -> f32` (0-65535 → -1.0 to +1.0)
  - [ ] Implement `RcInput::update_from_mavlink(msg, current_time_us)`
  - [ ] Implement `RcInput::get_channel(channel: usize) -> f32` (1-indexed)
  - [ ] Implement `RcInput::check_timeout(current_time_us)` (1 second threshold)
  - [ ] Implement `RcInput::is_active()` and `RcInput::is_lost()`
  - [ ] Create global `RC_INPUT: Mutex<RcInput>` static
  - [ ] Add unit tests for normalization (boundary values: 0, 32768, 65535)
  - [ ] Add unit tests for timeout detection (active → lost after 1s)
- [ ] **Create RC_CHANNELS MAVLink handler**
  - [ ] Create `src/communication/mavlink/handlers/rc_input.rs`
  - [ ] Implement `handle_rc_channels(msg: &RC_CHANNELS_DATA)` async function
  - [ ] Lock `RC_INPUT` mutex and call `update_from_mavlink()`
  - [ ] Log RC channel values at trace level (channel 1, channel 3, count)
  - [ ] Register handler in MAVLink router (update `handlers/mod.rs`)
  - [ ] Add integration to MAVLink task message dispatch
- [ ] **Implement actuator abstraction**
  - [ ] Create `src/vehicle/actuators.rs`
  - [ ] Define `ActuatorInterface` trait (set_steering, set_throttle, get_steering, get_throttle)
  - [ ] Define `ActuatorConfig` struct (steering_min/neutral/max, throttle_min/neutral/max)
  - [ ] Implement `ActuatorConfig::default()` (1000/1500/2000 for all)
  - [ ] Define `Actuators` struct (steering_pwm, throttle_pwm, system_state, config, current values)
  - [ ] Implement `Actuators::new(steering_pwm, throttle_pwm, system_state, config)`
  - [ ] Implement `ActuatorInterface::set_steering(normalized: f32)`
    - [ ] Check `system_state.is_armed()`, override to 0.0 if disarmed
    - [ ] Clamp normalized to \[-1.0, +1.0]
    - [ ] Convert normalized → PWM pulse width (1000-2000 μs)
    - [ ] Convert pulse width → duty cycle (5-10% for 50 Hz PWM)
    - [ ] Call `steering_pwm.set_duty_cycle(duty)`
    - [ ] Store `current_steering` for telemetry
  - [ ] Implement `ActuatorInterface::set_throttle(normalized: f32)` (same logic as steering)
  - [ ] Implement `normalized_to_pulse(normalized, min, neutral, max) -> u16` helper
  - [ ] Implement `pulse_to_duty_cycle(pulse_us: u16) -> f32` helper (50 Hz = 20ms period)
  - [ ] Add unit tests for PWM conversion (1000 μs = 5%, 1500 μs = 7.5%, 2000 μs = 10%)
  - [ ] Add unit tests for armed state enforcement (disarmed → neutral outputs)

### Deliverables

- `src/vehicle/mod.rs` - Vehicle module root
- `src/vehicle/rc_input.rs` - RC input state and processing
- `src/communication/mavlink/handlers/rc_input.rs` - RC_CHANNELS handler
- `src/vehicle/actuators.rs` - Actuator abstraction implementation
- Unit tests for RC normalization, timeout, actuator conversion, armed state

### Verification

```bash
# Format and lint
cargo fmt
cargo clippy --all-targets -- -D warnings

# Unit tests
cargo test --lib --quiet rc_input
cargo test --lib --quiet actuators

# Build for RP2350
./scripts/build-rp2350.sh
```

### Acceptance Criteria (Phase Gate)

- RC input normalization accurate (0 → -1.0, 32768 → 0.0, 65535 → +1.0)
- RC timeout detection works (active → lost after 1 second)
- Actuator PWM conversion accurate (1000 μs = 5%, 1500 μs = 7.5%, 2000 μs = 10%)
- **CRITICAL**: Actuators output neutral (0.0) when disarmed (automated test passes)
- All unit tests pass
- Code compiles without warnings

### Rollback/Fallback

- If RC input Mutex causes lock contention: Use `embassy_sync::signal::Signal` instead
- If actuator abstraction adds significant overhead: Profile and optimize conversion functions

---

## Phase 2: Mode Framework (VehicleMode Trait + Mode Manager)

### Goal

- Define VehicleMode trait with lifecycle methods
- Implement Mode Manager for mode transitions
- Create Vehicle Control Task (50 Hz mode execution)
- Verify mode framework operates correctly without actual modes

### Inputs

- Documentation:
  - `docs/adr/ADR-w9zpl-vehicle-mode-architecture.md` – Mode framework design
  - `docs/requirements/FR-q2sjt-vehicle-mode-framework.md` – Framework requirements
- Source Code to Modify:
  - `src/vehicle/mod.rs` – Add mode framework exports
  - `src/core/scheduler/tasks/` – Add vehicle control task
- Dependencies:
  - Phase 1: RC input and actuator abstraction must be complete
  - Internal: `src/communication/mavlink/state.rs` – System state
  - External crates: `embassy-time` (Ticker), `embassy-executor` (task)

### Tasks

- [ ] **Define VehicleMode trait**
  - [ ] Create `src/vehicle/mode.rs`
  - [ ] Define `VehicleMode` trait with methods:
    - [ ] `fn enter(&mut self) -> Result<(), &'static str>` - Initialize mode
    - [ ] `fn update(&mut self, dt: f32) -> Result<(), &'static str>` - Execute mode (50 Hz)
    - [ ] `fn exit(&mut self) -> Result<(), &'static str>` - Cleanup mode
    - [ ] `fn name(&self) -> &'static str` - Get mode name
  - [ ] Add comprehensive trait documentation with examples
  - [ ] Add `#[allow(async_fn_in_trait)]` if using async methods
- [ ] **Implement Mode Manager**
  - [ ] Create `src/vehicle/mode_manager.rs`
  - [ ] Define `ModeManager` struct (current_mode: Box<dyn VehicleMode>, system_state, last_update_us)
  - [ ] Implement `ModeManager::new(initial_mode, system_state)`
  - [ ] Implement `ModeManager::execute(current_time_us) -> Result<(), &'static str>`
    - [ ] Calculate delta time (dt = (current_time_us - last_update_us) / 1_000_000.0)
    - [ ] Call `current_mode.update(dt)`
    - [ ] Update `last_update_us`
  - [ ] Implement `ModeManager::set_mode(new_mode: Box<dyn VehicleMode>) -> Result<(), &'static str>`
    - [ ] Call `current_mode.exit()` (log warnings, continue if error)
    - [ ] Call `new_mode.enter()` (if error, revert to Manual fallback)
    - [ ] Update `current_mode`
    - [ ] Update `system_state.mode`
    - [ ] Log mode change (defmt::info)
  - [ ] Implement `ModeManager::current_mode_name() -> &'static str`
  - [ ] Add unit tests for mode transitions (mock modes)
  - [ ] Add unit tests for mode entry failure (verify Manual fallback)
- [ ] **Create Vehicle Control Task**
  - [ ] Create `src/core/scheduler/tasks/vehicle.rs`
  - [ ] Define `vehicle_control_task(mode_manager: &'static mut ModeManager)` Embassy task
  - [ ] Create 50 Hz ticker (`Ticker::every(Duration::from_millis(20))`)
  - [ ] In loop:
    - [ ] Get current timestamp (`embassy_time::Instant::now().as_micros()`)
    - [ ] Lock `RC_INPUT` and call `check_timeout(current_time_us)`
    - [ ] Call `mode_manager.execute(current_time_us)`
    - [ ] Log errors if mode execution fails (defmt::error)
    - [ ] Wait for next tick (`ticker.next().await`)
  - [ ] Add task to scheduler initialization (update `src/core/scheduler/mod.rs`)
  - [ ] Export task from `tasks/mod.rs`

### Deliverables

- `src/vehicle/mode.rs` - VehicleMode trait definition
- `src/vehicle/mode_manager.rs` - Mode manager implementation
- `src/core/scheduler/tasks/vehicle.rs` - Vehicle control task (50 Hz)
- Unit tests for mode manager (transitions, delta time calculation)

### Verification

```bash
# Format and lint
cargo fmt
cargo clippy --all-targets -- -D warnings

# Unit tests
cargo test --lib --quiet mode_manager

# Build for RP2350
./scripts/build-rp2350.sh
```

### Acceptance Criteria (Phase Gate)

- VehicleMode trait compiles and is well-documented
- Mode manager handles transitions correctly (exit → enter)
- Mode manager handles entry failure (reverts to Manual fallback)
- Vehicle control task executes at 50 Hz (verify with logs)
- Delta time calculation accurate (20ms ± 1ms)
- RC timeout checked every control loop iteration
- All unit tests pass

### Rollback/Fallback

- If trait object overhead is significant: Profile and consider enum-based state machine
- If 50 Hz execution timing is unstable: Investigate scheduler priority or Embassy Ticker behavior

---

## Phase 3: Manual Mode Implementation

### Goal

- Implement Manual mode with RC pass-through logic
- Integrate Manual mode with Mode Manager
- Add MAVLink DO_SET_MODE command handler
- Verify end-to-end manual control via Mission Planner

### Inputs

- Documentation:
  - `docs/requirements/FR-uk0us-manual-mode.md` – Manual mode requirements
- Source Code to Modify:
  - `src/vehicle/modes/` – Create modes directory
  - `src/communication/mavlink/handlers/command.rs` – Add DO_SET_MODE handler
- Dependencies:
  - Phase 1: RC input and actuators complete
  - Phase 2: Mode framework complete
  - Internal: `src/vehicle/` – Mode framework and actuators
  - External crates: None (reuse existing)

### Tasks

- [ ] **Create modes module**
  - [ ] Create `src/vehicle/modes/mod.rs`
  - [ ] Export `ManualMode` struct
  - [ ] Add module documentation
- [ ] **Implement Manual Mode**
  - [ ] Create `src/vehicle/modes/manual.rs`
  - [ ] Define `ManualMode` struct (rc_input: &'static Mutex<RcInput>, actuators: &'static mut dyn ActuatorInterface)
  - [ ] Implement `ManualMode::new(rc_input, actuators)`
  - [ ] Implement `VehicleMode::enter()`
    - [ ] Log "Entering Manual mode" (defmt::info)
    - [ ] Return Ok(())
  - [ ] Implement `VehicleMode::update(dt: f32)`
    - [ ] Lock `rc_input` mutex
    - [ ] Check `rc.is_lost()`, if true: unlock, set neutral actuators, return Ok
    - [ ] Read channel 1 (steering): `rc.get_channel(1)`
    - [ ] Read channel 3 (throttle): `rc.get_channel(3)`
    - [ ] Unlock `rc_input`
    - [ ] Call `actuators.set_steering(steering)`
    - [ ] Call `actuators.set_throttle(throttle)`
    - [ ] Return Ok(())
  - [ ] Implement `VehicleMode::exit()`
    - [ ] Log "Exiting Manual mode" (defmt::info)
    - [ ] Set neutral actuators: `set_steering(0.0)`, `set_throttle(0.0)`
    - [ ] Return Ok(())
  - [ ] Implement `VehicleMode::name() -> &'static str` (return "Manual")
  - [ ] Add unit tests with mock RC input and mock actuators
    - [ ] Test RC pass-through (steering/throttle mapping)
    - [ ] Test RC timeout handling (neutral outputs)
    - [ ] Test disarmed behavior (neutral outputs regardless of RC)
- [ ] **Add DO_SET_MODE MAVLink handler**
  - [ ] Update `src/communication/mavlink/handlers/command.rs`
  - [ ] Implement `handle_do_set_mode(mode_number: u32, mode_manager: &mut ModeManager)`
  - [ ] Match mode_number:
    - [ ] 0 → Create ManualMode, call `mode_manager.set_mode()`
    - [ ] 1 → Future: StabilizeMode (return error "Not implemented")
    - [ ] 2 → Future: HoldMode (return error "Not implemented")
    - [ ] Other → Return error "Invalid mode number"
  - [ ] Send COMMAND_ACK with MAV_RESULT_ACCEPTED or MAV_RESULT_FAILED
  - [ ] Register handler in MAVLink command dispatcher
- [ ] **Integration with scheduler**
  - [ ] Update scheduler initialization to create ManualMode as initial mode
  - [ ] Pass ManualMode to ModeManager constructor
  - [ ] Verify vehicle control task starts with Manual mode

### Deliverables

- `src/vehicle/modes/mod.rs` - Modes module root
- `src/vehicle/modes/manual.rs` - Manual mode implementation
- Updated `src/communication/mavlink/handlers/command.rs` - DO_SET_MODE handler
- Unit tests for Manual mode (RC pass-through, timeout, disarmed)

### Verification

```bash
# Format and lint
cargo fmt
cargo clippy --all-targets -- -D warnings

# Unit tests
cargo test --lib --quiet manual_mode

# Build for RP2350
./scripts/build-rp2350.sh

# Flash and test with Mission Planner
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/debug/examples/manual_control_demo

# Manual testing checklist:
# 1. Connect Mission Planner via UDP
# 2. Configure joystick (Setup → Joystick)
# 3. Arm vehicle
# 4. Move joystick X-axis → verify steering servo moves
# 5. Move joystick Y-axis → verify throttle motor responds
# 6. Disarm vehicle → verify all actuators neutral
# 7. Stop sending RC_CHANNELS → verify timeout after 1s, actuators neutral
```

### Acceptance Criteria (Phase Gate)

- Manual mode implements VehicleMode trait correctly
- RC pass-through works (joystick → RC_CHANNELS → actuators)
- RC timeout triggers neutral outputs within 1 second
- **CRITICAL**: Disarmed state prevents all motor output (verified with hardware)
- DO_SET_MODE command switches to Manual mode
- Mission Planner joystick controls steering and throttle
- All unit tests pass
- All integration tests pass

### Rollback/Fallback

- If RC lock contention causes latency: Optimize lock hold time or use lock-free channel
- If actuator response is slow: Profile and optimize conversion functions

---

## Phase 4: Testing & Validation

### Goal

- Create comprehensive automated tests for safety-critical behavior
- Validate end-to-end manual control with hardware
- Measure and verify performance metrics (latency, memory usage)
- Document known limitations and follow-up tasks

### Tasks

- [ ] **Safety-Critical Automated Tests**
  - [ ] **Test: Disarmed prevents motor output**
    - [ ] Create integration test `test_disarmed_safety()`
    - [ ] Set system state to disarmed
    - [ ] Send RC_CHANNELS with full throttle (channel 3 = 65535)
    - [ ] Verify actuator outputs neutral PWM (1500 μs)
    - [ ] Assert: NO motor movement when disarmed
  - [ ] **Test: Disarm during full throttle stops motors**
    - [ ] Create integration test `test_disarm_immediate_stop()`
    - [ ] Arm vehicle, send RC_CHANNELS with full throttle
    - [ ] Verify actuator outputs non-neutral PWM
    - [ ] Disarm vehicle
    - [ ] Verify actuator outputs neutral PWM within 20ms
    - [ ] Assert: Motors stop within one control loop cycle
  - [ ] **Test: RC timeout triggers failsafe**
    - [ ] Create integration test `test_rc_timeout_failsafe()`
    - [ ] Arm vehicle, send RC_CHANNELS with full throttle
    - [ ] Stop sending RC_CHANNELS
    - [ ] Wait 1.1 seconds
    - [ ] Verify RcStatus::Lost
    - [ ] Verify actuator outputs neutral PWM
    - [ ] Assert: Timeout detected within 1 second
- [ ] **Hardware Validation**
  - [ ] Test with physical servo on steering channel (GPIO 16)
  - [ ] Test with physical ESC on throttle channel (GPIO 17)
  - [ ] Verify servo moves proportionally to joystick X-axis
  - [ ] Verify throttle responds proportionally to joystick Y-axis
  - [ ] Verify neutral outputs when disarmed (servo centered, motor stopped)
  - [ ] Verify smooth operation at 50 Hz (no jitter or lag)
- [ ] **Performance Measurements**
  - [ ] Measure RC input to actuator latency (target < 100ms)
    - [ ] Send RC_CHANNELS via MAVLink
    - [ ] Timestamp when message received
    - [ ] Timestamp when PWM duty cycle updated
    - [ ] Calculate delta (should be < 100ms)
  - [ ] Measure mode update latency (target < 1ms)
    - [ ] Instrument mode_manager.execute() with timestamps
    - [ ] Calculate average, max, min execution time
    - [ ] Verify average < 1ms, max < 5ms
  - [ ] Measure vehicle layer memory usage (target < 5 KB RAM)
    - [ ] Use external profiling tool (e.g., probe-rs memory view)
    - [ ] Calculate sizeof(RcInput) + sizeof(Actuators) + sizeof(ModeManager) + sizeof(ManualMode)
    - [ ] Verify total < 5 KB
- [ ] **Documentation**
  - [ ] Update `docs/architecture.md`: Add vehicle layer section
  - [ ] Document module structure: `src/vehicle/` hierarchy
  - [ ] Document known limitations:
    - [ ] MAVLink RC only (no physical RC receiver)
    - [ ] Manual mode only (no autonomous modes yet)
    - [ ] No RC input filtering/smoothing
    - [ ] Ackermann steering only (no differential/skid-steer)
  - [ ] Document follow-up tasks:
    - [ ] Add Hold mode (maintain position, stop movement)
    - [ ] Add physical RC receiver support (SBUS/PPM)
    - [ ] Add RC input filtering (low-pass filter for noise)
    - [ ] Add actuator rate limiting (prevent sudden movements)

### Deliverables

- Safety-critical integration tests (disarmed, disarm during throttle, RC timeout)
- Hardware validation report (servo/ESC testing results)
- Performance measurement results (latency, memory usage)
- Updated `docs/architecture.md` with vehicle layer documentation
- Known limitations and follow-up tasks documented

### Verification

```bash
# Run all tests
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet

# Build for RP2350
./scripts/build-rp2350.sh

# Flash and run safety tests on hardware
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/debug/examples/safety_tests
```

### Acceptance Criteria

- **CRITICAL**: All safety tests pass (disarmed, disarm during throttle, RC timeout)
- Latency < 100ms (RC input to actuator response)
- Mode update latency < 1ms average
- Vehicle layer memory < 5 KB RAM
- Hardware validation successful (servo and ESC respond correctly)
- Documentation updated with module structure and limitations
- All clippy warnings resolved
- All tests pass

---

## Definition of Done

- [ ] `cargo check` passes
- [ ] `cargo fmt` applied
- [ ] `cargo clippy --all-targets -- -D warnings` passes (zero warnings)
- [ ] `cargo test --lib --quiet` passes (all unit tests)
- [ ] Hardware testing complete:
  - [ ] Mission Planner joystick controls steering and throttle
  - [ ] Disarmed prevents all motor output (verified)
  - [ ] Disarm during throttle stops motors within 20ms (verified)
  - [ ] RC timeout triggers failsafe within 1 second (verified)
- [ ] Performance metrics met:
  - [ ] RC input to actuator latency < 100ms
  - [ ] Mode update latency < 1ms
  - [ ] Vehicle layer memory < 5 KB RAM
- [ ] Documentation updated:
  - [ ] `docs/architecture.md` includes vehicle layer section
  - [ ] Known limitations documented
  - [ ] Follow-up tasks documented
- [ ] Safety-critical tests automated and passing
- [ ] No unsafe code in vehicle layer
- [ ] No "manager" or "util" naming (follow AGENTS.md principles)

## Open Questions

- [ ] Should RC timeout threshold be configurable via parameter (FS_RC_TIMEOUT)? → Decision: Default 1000ms acceptable for initial implementation, defer parameter to future enhancement
- [ ] Should we implement RC input smoothing/filtering for noisy inputs? → Method: Start without filtering, monitor real-world testing, add low-pass filter if needed
- [ ] Should actuator commands be rate-limited to prevent sudden movements? → Decision: No rate limiting initially, modes control update rate (50 Hz), defer to future enhancement
- [ ] Should mode manager support deferred transitions (wait for safe state before switching)? → Decision: Defer to Phase 2 autonomous modes (Auto/RTL), immediate transitions sufficient for Manual/Hold

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
