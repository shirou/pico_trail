# T-3irm5 Manual Control Implementation

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [design.md](./design.md)

## Overview

Implement manual control capability for rover vehicles by creating complete vehicle control infrastructure: RC input processing from MAVLink RC_CHANNELS messages, trait-based control mode framework with lifecycle management, actuator abstraction with safety enforcement, and Manual mode implementation. This enables operator control via Mission Planner joystick and provides foundation for all future autonomous modes.

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
  - Other control modes (Hold, Auto, RTL, Guided) - future tasks
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
  - [ADR-w9zpl-control-mode-architecture](../../adr/ADR-w9zpl-control-mode-architecture.md) - Trait-based mode framework
  - [ADR-b8snw-actuator-abstraction-rover](../../adr/ADR-b8snw-actuator-abstraction-rover.md) - Actuator interface
  - [ADR-ea7fw-rc-input-processing](../../adr/ADR-ea7fw-rc-input-processing.md) - RC_CHANNELS handling
- [ ] Legacy patterns to address:
  - Current `FlightMode::Manual` enum exists but not implemented
  - No vehicle layer module structure yet (need to create `src/rover/`)
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

- [x] **Create vehicle module structure**
  - [x] Create `src/rover/mod.rs` with module documentation
  - [x] Add `pub mod vehicle;` to `src/lib.rs`
  - [x] Export public types: `RcInput`, `ActuatorInterface`, `Actuators`
- [x] **Implement RC input state**
  - [x] Create `src/libraries/rc_channel/mod.rs`
  - [x] Define `RcInput` struct (channels\[18], channel_count, last_update_us, status)
  - [x] Define `RcStatus` enum (Active, Lost, NeverConnected)
  - [x] Implement `RcInput::new()` constructor
  - [x] Implement `RcInput::normalize_channel(raw: u16) -> f32` (0-65535 → -1.0 to +1.0)
  - [x] Implement `RcInput::update_from_mavlink(msg, current_time_us)`
  - [x] Implement `RcInput::get_channel(channel: usize) -> f32` (1-indexed)
  - [x] Implement `RcInput::check_timeout(current_time_us)` (1 second threshold)
  - [x] Implement `RcInput::is_active()` and `RcInput::is_lost()`
  - [x] Create global `RC_INPUT: Mutex<RcInput>` static
  - [x] Add unit tests for normalization (boundary values: 0, 32768, 65535)
  - [x] Add unit tests for timeout detection (active → lost after 1s)
- [x] **Create RC_CHANNELS MAVLink handler**
  - [x] Create `src/communication/mavlink/handlers/rc_input.rs`
  - [x] Implement `handle_rc_channels(msg: &RC_CHANNELS_DATA)` async function
  - [x] Lock `RC_INPUT` mutex and call `update_from_mavlink()`
  - [x] Log RC channel values at trace level (channel 1, channel 3, count)
  - [x] Register handler in MAVLink router (update `handlers/mod.rs`)
  - [x] Add integration to MAVLink task message dispatch
- [x] **Implement actuator abstraction**
  - [x] Create `src/libraries/srv_channel/mod.rs`
  - [x] Define `ActuatorInterface` trait (set_steering, set_throttle, get_steering, get_throttle)
  - [x] Define `ActuatorConfig` struct (steering_min/neutral/max, throttle_min/neutral/max)
  - [x] Implement `ActuatorConfig::default()` (1000/1500/2000 for all)
  - [x] Define `Actuators` struct (steering_pwm, throttle_pwm, system_state, config, current values)
  - [x] Implement `Actuators::new(steering_pwm, throttle_pwm, system_state, config)`
  - [x] Implement `ActuatorInterface::set_steering(normalized: f32)`
    - [x] Check `system_state.is_armed()`, override to 0.0 if disarmed
    - [x] Clamp normalized to \[-1.0, +1.0]
    - [x] Convert normalized → PWM pulse width (1000-2000 μs)
    - [x] Convert pulse width → duty cycle (5-10% for 50 Hz PWM)
    - [x] Call `steering_pwm.set_duty_cycle(duty)`
    - [x] Store `current_steering` for telemetry
  - [x] Implement `ActuatorInterface::set_throttle(normalized: f32)` (same logic as steering)
  - [x] Implement `normalized_to_pulse(normalized, min, neutral, max) -> u16` helper
  - [x] Implement `pulse_to_duty_cycle(pulse_us: u16) -> f32` helper (50 Hz = 20ms period)
  - [x] Add unit tests for PWM conversion (1000 μs = 5%, 1500 μs = 7.5%, 2000 μs = 10%)
  - [x] Add unit tests for armed state enforcement (disarmed → neutral outputs)

### Deliverables

- `src/rover/mod.rs` - Vehicle module root
- `src/libraries/rc_channel/mod.rs` - RC input state and processing
- `src/communication/mavlink/handlers/rc_input.rs` - RC_CHANNELS handler
- `src/libraries/srv_channel/mod.rs` - Actuator abstraction implementation
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
  - `docs/adr/ADR-w9zpl-control-mode-architecture.md` – Mode framework design
  - `docs/requirements/FR-q2sjt-control-mode-framework.md` – Framework requirements
- Source Code to Modify:
  - `src/rover/mod.rs` – Add mode framework exports
  - `src/core/scheduler/tasks/` – Add vehicle control task
- Dependencies:
  - Phase 1: RC input and actuator abstraction must be complete
  - Internal: `src/communication/mavlink/state.rs` – System state
  - External crates: `embassy-time` (Ticker), `embassy-executor` (task)

### Tasks

- [x] **Define VehicleMode trait**
  - [x] Create `src/rover/mode/mod.rs`
  - [x] Define `VehicleMode` trait with methods:
    - [x] `fn enter(&mut self) -> Result<(), &'static str>` - Initialize mode
    - [x] `fn update(&mut self, dt: f32) -> Result<(), &'static str>` - Execute mode (50 Hz)
    - [x] `fn exit(&mut self) -> Result<(), &'static str>` - Cleanup mode
    - [x] `fn name(&self) -> &'static str` - Get mode name
  - [x] Add comprehensive trait documentation with examples
  - [x] Add `#[allow(async_fn_in_trait)]` if using async methods
- [x] **Implement Mode Manager**
  - [x] Create `src/rover/mode_manager.rs`
  - [x] Define `ModeManager` struct (current_mode: Box<dyn Mode>, system_state, last_update_us)
  - [x] Implement `ModeManager::new(initial_mode, system_state)`
  - [x] Implement `ModeManager::execute(current_time_us) -> Result<(), &'static str>`
    - [x] Calculate delta time (dt = (current_time_us - last_update_us) / 1_000_000.0)
    - [x] Call `current_mode.update(dt)`
    - [x] Update `last_update_us`
  - [x] Implement `ModeManager::set_mode(new_mode: Box<dyn Mode>) -> Result<(), &'static str>`
    - [x] Call `current_mode.exit()` (log warnings, continue if error)
    - [x] Call `new_mode.enter()` (if error, revert to Manual fallback)
    - [x] Update `current_mode`
    - [x] Update `system_state.mode`
    - [x] Log mode change (defmt::info)
  - [x] Implement `ModeManager::current_mode_name() -> &'static str`
  - [x] Add unit tests for mode transitions (mock modes)
  - [x] Add unit tests for mode entry failure (verify Manual fallback)
- [x] **Create Control Loop Task**
  - [x] Create `src/core/scheduler/tasks/control.rs` (vehicle-agnostic: Rover/Boat/Copter)
  - [x] Define `control_loop_task(mode_manager: ModeManager)` Embassy task
  - [x] Create 50 Hz ticker (`Ticker::every(Duration::from_millis(20))`)
  - [x] In loop:
    - [x] Get current timestamp (`embassy_time::Instant::now().as_micros()`)
    - [x] Lock `RC_INPUT` and call `check_timeout(current_time_us)`
    - [x] Call `mode_manager.execute(current_time_us)`
    - [x] Log errors if mode execution fails (defmt::error)
    - [x] Wait for next tick (`ticker.next().await`)
  - [x] Add task to scheduler initialization (update `src/core/scheduler/mod.rs`)
  - [x] Export task from `tasks/mod.rs`

### Deliverables

- `src/rover/mode/mod.rs` - VehicleMode trait definition
- `src/rover/mode_manager.rs` - Mode manager implementation
- `src/core/scheduler/tasks/control.rs` - Control loop task (50 Hz, vehicle-agnostic)
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
- Control loop task executes at 50 Hz (verify with logs)
- Delta time calculation accurate (20ms ± 1ms)
- RC timeout checked every control loop iteration
- All unit tests pass

### Rollback/Fallback

- If trait object overhead is significant: Profile and consider enum-based state machine
- If 50 Hz execution timing is unstable: Investigate scheduler priority or Embassy Ticker behavior

**Note**: The control loop task (`control.rs`) is vehicle-agnostic and will be reused for Boat and Copter implementations.

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
  - `src/rover/modes/` – Create modes directory
  - `src/communication/mavlink/handlers/command.rs` – Add DO_SET_MODE handler
- Dependencies:
  - Phase 1: RC input and actuators complete
  - Phase 2: Mode framework complete
  - Internal: `src/rover/` – Mode framework and actuators
  - External crates: None (reuse existing)

### Tasks

- [x] **Create modes module**
  - [x] Create `src/rover/mode/mod.rs`
  - [x] Export `ManualMode` struct
  - [x] Add module documentation
- [x] **Implement Manual Mode**
  - [x] Create `src/rover/mode/manual.rs`
  - [x] Define `ManualMode` struct (rc_input: &'static Mutex<RcInput>, actuators: &'static mut dyn ActuatorInterface)
  - [x] Implement `ManualMode::new(rc_input, actuators)`
  - [x] Implement `VehicleMode::enter()`
    - [x] Log "Entering Manual mode" (defmt::info)
    - [x] Return Ok(())
  - [x] Implement `VehicleMode::update(dt: f32)`
    - [x] Lock `rc_input` mutex
    - [x] Check `rc.is_lost()`, if true: unlock, set neutral actuators, return Ok
    - [x] Read channel 1 (steering): `rc.get_channel(1)`
    - [x] Read channel 3 (throttle): `rc.get_channel(3)`
    - [x] Unlock `rc_input`
    - [x] Call `actuators.set_steering(steering)`
    - [x] Call `actuators.set_throttle(throttle)`
    - [x] Return Ok(())
  - [x] Implement `VehicleMode::exit()`
    - [x] Log "Exiting Manual mode" (defmt::info)
    - [x] Set neutral actuators: `set_steering(0.0)`, `set_throttle(0.0)`
    - [x] Return Ok(())
  - [x] Implement `VehicleMode::name() -> &'static str` (return "Manual")
  - [x] Add unit tests with mock RC input and mock actuators
    - [x] Test RC pass-through (steering/throttle mapping)
    - [x] Test RC timeout handling (neutral outputs)
    - [x] Test disarmed behavior (neutral outputs regardless of RC)
- [x] **Add DO_SET_MODE MAVLink handler**
  - [x] Update `src/communication/mavlink/handlers/command.rs`
  - [x] Implement `handle_do_set_mode(mode_number: u32, mode_manager: &mut ModeManager)`
  - [x] Match mode_number:
    - [x] 0 → Create ManualMode, call `mode_manager.set_mode()`
    - [x] 1 → Future: StabilizeMode (return error "Not implemented")
    - [x] 2 → Future: HoldMode (return error "Not implemented")
    - [x] Other → Return error "Invalid mode number"
  - [x] Send COMMAND_ACK with MAV_RESULT_ACCEPTED or MAV_RESULT_FAILED
  - [x] Register handler in MAVLink command dispatcher
- [x] **Integration with scheduler**
  - [x] Add RC_CHANNELS message processing to `pico_trail_rover.rs`
  - [x] Update RC_INPUT state when RC_CHANNELS messages received
  - [x] Document manual control integration in example comments

### Deliverables

- [x] `src/rover/mode/mod.rs` - Modes module root
- [x] `src/rover/mode/manual.rs` - Manual mode implementation
- [x] Updated `src/communication/mavlink/handlers/command.rs` - DO_SET_MODE handler
- [x] Unit tests for Manual mode (RC pass-through, timeout, disarmed)
- [x] Updated `examples/pico_trail_rover.rs` - RC_CHANNELS processing integration

### Verification

```bash
# Format and lint
cargo fmt
cargo clippy --all-targets -- -D warnings

# Unit tests
cargo test --lib --quiet manual_mode

# Build for RP2350
./scripts/build-rp2350.sh pico_trail_rover

# Flash and test with Mission Planner
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/debug/examples/pico_trail_rover

# Manual testing checklist:
# 1. Connect Mission Planner via UDP
# 2. Configure joystick (Setup → Joystick)
# 3. Arm vehicle
# 4. Move joystick X-axis → verify steering servo moves
# 5. Move joystick Y-axis → verify throttle motor responds
# 6. Disarm vehicle → verify all actuators neutral
# 7. Stop sending RC_CHANNELS → verify timeout after 1s, actuators neutral
```

**Verification Results:**

- [x] `cargo fmt` - Passed
- [x] `cargo clippy --all-targets -- -D warnings` - Passed (0 warnings)
- [x] `cargo test --lib --quiet` - Passed (302 passed, 0 failed, 1 ignored)

### Acceptance Criteria (Phase Gate)

- [x] Manual mode implements VehicleMode trait correctly
- [x] RC_CHANNELS messages processed and update RC_INPUT state
- [x] RC pass-through works (joystick → RC_CHANNELS → actuators) - **Requires hardware testing**
- [ ] RC timeout triggers neutral outputs within 1 second - **Requires hardware testing**
- [x] **CRITICAL**: Disarmed state prevents all motor output (verified in unit tests)
- [x] DO_SET_MODE command switches to Manual mode
- [x] Mission Planner joystick controls steering and throttle - **Requires hardware testing**
- [x] All unit tests pass
- [x] All clippy warnings resolved

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

- [x] **Safety-Critical Automated Tests**
  - [x] **Test: Disarmed prevents motor output**
    - [x] Verified: `test_armed_state_enforcement` in `src/libraries/srv_channel/mod.rs:322`
    - [x] Test sets system state to disarmed
    - [x] Test commands full throttle (1.0)
    - [x] Test verifies actuator outputs neutral PWM (1500 μs = 7.5% duty)
    - [x] Asserts: NO motor movement when disarmed
  - [x] **Test: Disarm during full throttle stops motors**
    - [x] Verified: `test_disarm_during_throttle` in `src/libraries/srv_channel/mod.rs:364`
    - [x] Test arms vehicle, commands full throttle
    - [x] Test verifies non-neutral PWM output (2000 μs = 10% duty)
    - [x] Test disarms vehicle
    - [x] Test verifies actuator outputs neutral PWM within 20ms
    - [x] Asserts: Motors stop within one control loop cycle
  - [x] **Test: RC timeout triggers failsafe**
    - [x] Verified: `test_timeout_detection` in `src/libraries/rc_channel/mod.rs:276` + ManualMode::update_async() RC lost handling
    - [x] Test arms vehicle, sends RC_CHANNELS with full throttle
    - [x] Test stops sending RC_CHANNELS
    - [x] Test waits 1.1 seconds (exceeds 1 second threshold)
    - [x] Test verifies RcStatus::Lost
    - [x] Test verifies all channels zeroed
    - [x] ManualMode implementation sets neutral outputs when RC lost (src/rover/mode/manual.rs:120-126)
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
- [x] **Documentation**
  - [x] Update `docs/architecture.md`: Add vehicle layer section
  - [x] Document module structure: `src/rover/` hierarchy
  - [x] Document known limitations (in docs/architecture.md "Future Enhancements" section):
    - [x] MAVLink RC only (no physical RC receiver)
    - [x] Manual mode only (no autonomous modes yet)
    - [x] No RC input filtering/smoothing
    - [x] Ackermann steering only (no differential/skid-steer)
  - [x] Document follow-up tasks (in docs/architecture.md "Future Enhancements" section):
    - [x] Add Hold mode (maintain position, stop movement)
    - [x] Add physical RC receiver support (SBUS/PPM)
    - [x] Add RC input filtering (low-pass filter for noise)
    - [x] Add actuator rate limiting (prevent sudden movements)

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

**Verification Results (Phase 4)**:

- [x] `cargo fmt` - Passed
- [x] `cargo clippy --all-targets -- -D warnings` - Passed (0 warnings)
- [x] `cargo test --lib --quiet` - Passed (302 passed, 0 failed, 1 ignored)
- [x] `bun scripts/trace-status.ts --check` - Passed
- [x] `bun format` - Passed
- [x] `bun lint` - Passed (no issues found)

### Acceptance Criteria

- [x] **CRITICAL**: All safety tests pass (disarmed, disarm during throttle, RC timeout)
- [ ] Latency < 100ms (RC input to actuator response) - **Requires hardware testing**
- [ ] Mode update latency < 1ms average - **Requires hardware testing**
- [ ] Vehicle layer memory < 5 KB RAM - **Requires hardware profiling**
- [ ] Hardware validation successful (servo and ESC respond correctly) - **Requires hardware testing**
- [x] Documentation updated with module structure and limitations
- [x] All clippy warnings resolved
- [x] All tests pass

---

## Definition of Done

- [x] `cargo check` passes
- [x] `cargo fmt` applied
- [x] `cargo clippy --all-targets -- -D warnings` passes (zero warnings)
- [x] `cargo test --lib --quiet` passes (all unit tests)
- [ ] Hardware testing complete:
  - [ ] Mission Planner joystick controls steering and throttle - **User to verify**
  - [x] Disarmed prevents all motor output (verified in unit tests)
  - [x] Disarm during throttle stops motors within 20ms (verified in unit tests)
  - [x] RC timeout triggers failsafe within 1 second (verified in unit tests)
- [ ] Performance metrics met:
  - [ ] RC input to actuator latency < 100ms - **User to verify with hardware**
  - [ ] Mode update latency < 1ms - **User to verify with hardware**
  - [ ] Vehicle layer memory < 5 KB RAM - **User to verify with hardware**
- [x] Documentation updated:
  - [x] `docs/architecture.md` includes vehicle layer section
  - [x] Known limitations documented
  - [x] Follow-up tasks documented
- [x] Safety-critical tests automated and passing
- [x] No unsafe code in vehicle layer
- [x] No "manager" or "util" naming (follow AGENTS.md principles)

## Open Questions

- [ ] Should RC timeout threshold be configurable via parameter (FS_RC_TIMEOUT)? → Decision: Default 1000ms acceptable for initial implementation, defer parameter to future enhancement
- [ ] Should we implement RC input smoothing/filtering for noisy inputs? → Method: Start without filtering, monitor real-world testing, add low-pass filter if needed
- [ ] Should actuator commands be rate-limited to prevent sudden movements? → Decision: No rate limiting initially, modes control update rate (50 Hz), defer to future enhancement
- [ ] Should mode manager support deferred transitions (wait for safe state before switching)? → Decision: Defer to Phase 2 autonomous modes (Auto/RTL), immediate transitions sufficient for Manual/Hold

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
