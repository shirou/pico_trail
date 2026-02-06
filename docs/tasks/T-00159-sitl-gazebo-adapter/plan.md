# T-00159 SITL Gazebo Adapter Plan

## Metadata

- Type: Implementation Plan
- Status: Implementation Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement `GazeboAdapter` with UDP/JSON protocol in a single phase. The adapter communicates with Gazebo Harmonic via the ardupilot_gazebo plugin protocol.

## Success Metrics

- [x] `GazeboAdapter` implements `SimulatorAdapter` trait
- [x] JSON protocol matches ardupilot_gazebo format
- [x] UDP socket management with timeout handling
- [x] Unit tests pass with mock sockets

## Scope

- Goal: GazeboAdapter ready for full physics simulation
- Non-Goals: LightweightAdapter, multi-vehicle, CI integration
- Assumptions: T-00157 core abstractions are complete
- Constraints: Unit tests must not require Gazebo installation

## ADR & Legacy Alignment

- [x] Confirm ADR-00156-sitl-pluggable-adapter-architecture is referenced
- [x] No modifications to existing crates/core or crates/firmware

## Plan Summary

| Phase | Description   | Effort  |
| ----- | ------------- | ------- |
| 1     | GazeboAdapter | 2 weeks |

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: GazeboAdapter

### Goal

- Implement `GazeboAdapter` with UDP/JSON protocol
- Compatible with ardupilot_gazebo plugin
- Connection/reconnection handling

### Inputs

- Files to Create:
  - `crates/sitl/src/adapter/gazebo.rs`
- External Dependencies:
  - Gazebo Harmonic (for integration testing only)
  - ardupilot_gazebo plugin

### Tasks

- [x] **Define GazeboConfig**
  - [x] `sensor_port: u16` (default 9002)
  - [x] `actuator_port: u16` (default 9003)
  - [x] `server_addr: SocketAddr`
  - [x] `ardupilot_compat: bool`
  - [x] `timeout_ms: u32`
- [x] **Implement UDP socket management**
  - [x] Async UDP with tokio
  - [x] Separate sockets for sensors and actuators
  - [x] Timeout handling
- [x] **Implement JSON parsing**
  - [x] Parse sensor JSON to `SensorData`
  - [x] Handle ardupilot_gazebo format
  - [x] Error handling for malformed JSON
- [x] **Implement JSON serialization**
  - [x] Serialize `ActuatorCommands` to JSON
  - [x] Convert normalized motor values to PWM (1000-2000)
- [x] **Implement SimulatorAdapter trait**
  - [x] `connect()` - bind sockets
  - [x] `disconnect()` - close sockets
  - [x] `receive_sensors()` - UDP recv + parse
  - [x] `send_actuators()` - serialize + UDP send
  - [x] `step()` - send step signal for lockstep
  - [x] `supports_lockstep()` - return true
- [x] **Unit tests (mock socket)**
  - [x] Test JSON parsing
  - [x] Test JSON serialization
  - [x] Test PWM conversion
- [ ] **Integration tests (optional, requires Gazebo)**
  - [ ] Document Gazebo setup
  - [ ] Test connection to real Gazebo
- [x] **Verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `cargo test -p pico_trail_sitl --lib`

### Deliverables

- `GazeboAdapter` ready for full simulation

### Acceptance Criteria (Phase Gate)

- FR-00151 requirements met
- JSON protocol matches ardupilot_gazebo
- Unit tests pass (mock socket)

### Rollback/Fallback

- GazeboAdapter can be disabled via feature flag
- LightweightAdapter (T-00158) provides fallback

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
