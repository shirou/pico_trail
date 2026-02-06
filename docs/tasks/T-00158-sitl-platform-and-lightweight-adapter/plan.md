# T-00158 SITL Platform and Lightweight Adapter Plan

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement `SitlPlatform` and `LightweightAdapter` in 2 phases. Phase 1 creates the simulated platform with peripheral wrappers. Phase 2 implements the built-in kinematics adapter for CI-ready testing.

## Success Metrics

- [x] `SitlPlatform` implements `Platform` trait
- [x] Sensor injection and actuator collection work correctly
- [x] `LightweightAdapter` passes kinematics tests
- [x] Deterministic mode produces repeatable results
- [x] No external dependencies required for testing

## Scope

- Goal: Minimum viable SITL with platform and lightweight adapter
- Non-Goals: GazeboAdapter, multi-vehicle routing, lockstep orchestration
- Assumptions: T-00157 core abstractions are complete
- Constraints: Must follow existing Platform trait patterns

## ADR & Legacy Alignment

- [x] Follow ADR-00003 Platform trait patterns
- [x] Follow MockPlatform patterns for peripheral wrappers
- [x] No modifications to existing crates/core or crates/firmware

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

- [x] **Implement SitlTimeSource**
  - [x] Wrap `Arc<AtomicU64>` for shared sim time
  - [x] Implement `TimerInterface` trait (`now_us()`, `delay_us()`)
- [x] **Implement SitlUart**
  - [x] Ring buffer for RX/TX
  - [x] `read()`, `write()` methods
  - [x] Connect to MAVLink handler (future phase)
- [x] **Implement SitlPwm**
  - [x] Store duty cycle (0.0-1.0)
  - [x] `set_duty()`, `get_duty()` methods
- [x] **Implement SitlGpio**
  - [x] Store pin state (high/low)
  - [x] Input/output direction
- [x] **Implement SitlPlatform**
  - [x] Implement all `Platform` trait methods
  - [x] `create_uart()` - return SitlUart
  - [x] `create_pwm()` - return SitlPwm, track for actuator collection
  - [x] `create_gpio()` - return SitlGpio
  - [x] `read_battery_adc()` - return configurable value
  - [x] `timer()` - return SitlTimeSource reference
- [x] **Sensor injection**
  - [x] `inject_sensors(&SensorData)` - update internal state
  - [x] Thread-safe with Mutex
- [x] **Actuator collection**
  - [x] `collect_actuator_commands()` - read PWM values, return ActuatorCommands
- [x] **Unit tests**
  - [x] Test Platform trait implementation compiles
  - [x] Test UART read/write
  - [x] Test PWM duty cycle
  - [x] Test sensor injection
  - [x] Test actuator collection
- [x] **Verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `cargo test -p pico_trail_sitl --lib`

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

- [x] **Define LightweightConfig**
  - [x] `wheel_base: f32` (default 0.15)
  - [x] `max_speed: f32` (default 1.0)
  - [x] `max_turn_rate: f32` (default 2.0)
  - [x] GPS noise, IMU noise, compass noise parameters
  - [x] `seed: Option<u64>` for deterministic mode
- [x] **Implement internal state**
  - [x] Position (x, y) in meters
  - [x] Heading in radians
  - [x] Velocity vector
  - [x] Motor commands (left, right)
- [x] **Implement kinematics integration**
  - [x] Differential drive model
  - [x] `v = (v_left + v_right) / 2`
  - [x] `omega = (v_right - v_left) / wheel_base`
  - [x] Position/heading update
- [x] **Implement sensor synthesis**
  - [x] IMU from motion (accel from velocity change, gyro from rotation)
  - [x] GPS from position with noise
  - [x] Compass from heading with noise
- [x] **Implement SimulatorAdapter trait**
  - [x] `connect()` - initialize RNG, reset state
  - [x] `receive_sensors()` - return synthesized sensors
  - [x] `send_actuators()` - store motor commands
  - [x] `step()` - integrate kinematics
  - [x] `supports_lockstep()` - return true
- [x] **Unit tests**
  - [x] Test kinematics: straight line motion
  - [x] Test kinematics: rotation in place
  - [x] Test kinematics: arc turn
  - [x] Test deterministic mode: same seed = same results
  - [x] Test noise: outputs vary with noise enabled
- [x] **Verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `cargo test -p pico_trail_sitl --lib`

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

- [x] All phases completed
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test -p pico_trail_sitl --lib`
- [x] `./scripts/build-rp2350.sh pico_trail_rover` (verify no impact on embedded)
- [x] No `unsafe` code
- [x] Plan checkboxes marked
- [x] Task README status updated to Implementation Complete
- [x] Traceability check: `bun scripts/trace-status.ts --check`
