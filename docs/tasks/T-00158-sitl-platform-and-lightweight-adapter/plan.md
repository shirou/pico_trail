# T-00158 SITL Platform and Lightweight Adapter Plan

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement `SitlPlatform` and `LightweightAdapter` in 2 phases. Phase 1 creates the simulated platform with peripheral wrappers. Phase 2 implements the built-in kinematics adapter for CI-ready testing.

## Success Metrics

- [ ] `SitlPlatform` implements `Platform` trait
- [ ] Sensor injection and actuator collection work correctly
- [ ] `LightweightAdapter` passes kinematics tests
- [ ] Deterministic mode produces repeatable results
- [ ] No external dependencies required for testing

## Scope

- Goal: Minimum viable SITL with platform and lightweight adapter
- Non-Goals: GazeboAdapter, multi-vehicle routing, lockstep orchestration
- Assumptions: T-00157 core abstractions are complete
- Constraints: Must follow existing Platform trait patterns

## ADR & Legacy Alignment

- [ ] Follow ADR-00003 Platform trait patterns
- [ ] Follow MockPlatform patterns for peripheral wrappers
- [ ] No modifications to existing crates/core or crates/firmware

## Plan Summary

| Phase | Description        | Effort    |
| ----- | ------------------ | --------- |
| 1     | SitlPlatform       | 1 week    |
| 2     | LightweightAdapter | 1.5 weeks |

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: SitlPlatform

### Goal

- Implement `SitlPlatform` that implements `Platform` trait
- Sensor injection from bridge
- Actuator command collection for bridge
- Simulated UART, PWM, GPIO peripherals

### Inputs

- Files to Create:
  - `crates/sitl/src/platform/mod.rs`
  - `crates/sitl/src/platform/uart.rs`
  - `crates/sitl/src/platform/pwm.rs`
  - `crates/sitl/src/platform/gpio.rs`
  - `crates/sitl/src/platform/timer.rs`

### Tasks

- [ ] **Implement SitlTimeSource**
  - [ ] Wrap `Arc<AtomicU64>` for shared sim time
  - [ ] Implement `TimerInterface` trait (`now_us()`, `delay_us()`)
- [ ] **Implement SitlUart**
  - [ ] Ring buffer for RX/TX
  - [ ] `read()`, `write()` methods
  - [ ] Connect to MAVLink handler (future phase)
- [ ] **Implement SitlPwm**
  - [ ] Store duty cycle (0.0-1.0)
  - [ ] `set_duty()`, `get_duty()` methods
- [ ] **Implement SitlGpio**
  - [ ] Store pin state (high/low)
  - [ ] Input/output direction
- [ ] **Implement SitlPlatform**
  - [ ] Implement all `Platform` trait methods
  - [ ] `create_uart()` - return SitlUart
  - [ ] `create_pwm()` - return SitlPwm, track for actuator collection
  - [ ] `create_gpio()` - return SitlGpio
  - [ ] `read_battery_adc()` - return configurable value
  - [ ] `timer()` - return SitlTimeSource reference
- [ ] **Sensor injection**
  - [ ] `inject_sensors(&SensorData)` - update internal state
  - [ ] Thread-safe with Mutex
- [ ] **Actuator collection**
  - [ ] `collect_actuator_commands()` - read PWM values, return ActuatorCommands
- [ ] **Unit tests**
  - [ ] Test Platform trait implementation compiles
  - [ ] Test UART read/write
  - [ ] Test PWM duty cycle
  - [ ] Test sensor injection
  - [ ] Test actuator collection
- [ ] **Verification**
  - [ ] `cargo fmt`
  - [ ] `cargo clippy --all-targets -- -D warnings`
  - [ ] `cargo test -p pico_trail_sitl --lib`

### Deliverables

- `SitlPlatform` implementing `Platform` trait

### Acceptance Criteria (Phase Gate)

- FR-00154 requirements met
- SitlPlatform passes Platform trait bounds
- Sensor/actuator data flows correctly

### Rollback/Fallback

- SitlPlatform can return stub implementations

---

## Phase 2: LightweightAdapter

### Goal

- Implement `LightweightAdapter` with built-in differential drive kinematics
- Configurable sensor noise
- Deterministic mode with seeded RNG
- No external dependencies

### Inputs

- Files to Create:
  - `crates/sitl/src/adapter/lightweight.rs`

### Tasks

- [ ] **Define LightweightConfig**
  - [ ] `wheel_base: f32` (default 0.15)
  - [ ] `max_speed: f32` (default 1.0)
  - [ ] `max_turn_rate: f32` (default 2.0)
  - [ ] GPS noise, IMU noise, compass noise parameters
  - [ ] `seed: Option<u64>` for deterministic mode
- [ ] **Implement internal state**
  - [ ] Position (x, y) in meters
  - [ ] Heading in radians
  - [ ] Velocity vector
  - [ ] Motor commands (left, right)
- [ ] **Implement kinematics integration**
  - [ ] Differential drive model
  - [ ] `v = (v_left + v_right) / 2`
  - [ ] `omega = (v_right - v_left) / wheel_base`
  - [ ] Position/heading update
- [ ] **Implement sensor synthesis**
  - [ ] IMU from motion (accel from velocity change, gyro from rotation)
  - [ ] GPS from position with noise
  - [ ] Compass from heading with noise
- [ ] **Implement SimulatorAdapter trait**
  - [ ] `connect()` - initialize RNG, reset state
  - [ ] `receive_sensors()` - return synthesized sensors
  - [ ] `send_actuators()` - store motor commands
  - [ ] `step()` - integrate kinematics
  - [ ] `supports_lockstep()` - return true
- [ ] **Unit tests**
  - [ ] Test kinematics: straight line motion
  - [ ] Test kinematics: rotation in place
  - [ ] Test kinematics: arc turn
  - [ ] Test deterministic mode: same seed = same results
  - [ ] Test noise: outputs vary with noise enabled
- [ ] **Verification**
  - [ ] `cargo fmt`
  - [ ] `cargo clippy --all-targets -- -D warnings`
  - [ ] `cargo test -p pico_trail_sitl --lib`

### Deliverables

- `LightweightAdapter` ready for CI use

### Acceptance Criteria (Phase Gate)

- FR-00152 requirements met
- NFR-00096 requirements met (no external deps)
- Kinematics tests pass
- Deterministic mode works

### Rollback/Fallback

- Adapter can be disabled via feature flag

---

## Definition of Done

- [ ] All phases completed
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test -p pico_trail_sitl --lib`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover` (verify no impact on embedded)
- [ ] No `unsafe` code
- [ ] Plan checkboxes marked
- [ ] Task README status updated to Implementation Complete
- [ ] Traceability check: `bun scripts/trace-status.ts --check`
