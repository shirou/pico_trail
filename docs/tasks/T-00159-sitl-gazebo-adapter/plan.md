# T-00159 SITL Gazebo Adapter Plan

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement `GazeboAdapter` with UDP/JSON protocol in a single phase. The adapter communicates with Gazebo Harmonic via the ardupilot_gazebo plugin protocol.

## Success Metrics

- [ ] `GazeboAdapter` implements `SimulatorAdapter` trait
- [ ] JSON protocol matches ardupilot_gazebo format
- [ ] UDP socket management with timeout handling
- [ ] Unit tests pass with mock sockets

## Scope

- Goal: GazeboAdapter ready for full physics simulation
- Non-Goals: LightweightAdapter, multi-vehicle, CI integration
- Assumptions: T-00157 core abstractions are complete
- Constraints: Unit tests must not require Gazebo installation

## ADR & Legacy Alignment

- [ ] Confirm ADR-00156-sitl-pluggable-adapter-architecture is referenced
- [ ] No modifications to existing crates/core or crates/firmware

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

- [ ] **Define GazeboConfig**
  - [ ] `sensor_port: u16` (default 9002)
  - [ ] `actuator_port: u16` (default 9003)
  - [ ] `server_addr: SocketAddr`
  - [ ] `ardupilot_compat: bool`
  - [ ] `timeout_ms: u32`
- [ ] **Implement UDP socket management**
  - [ ] Async UDP with tokio
  - [ ] Separate sockets for sensors and actuators
  - [ ] Timeout handling
- [ ] **Implement JSON parsing**
  - [ ] Parse sensor JSON to `SensorData`
  - [ ] Handle ardupilot_gazebo format
  - [ ] Error handling for malformed JSON
- [ ] **Implement JSON serialization**
  - [ ] Serialize `ActuatorCommands` to JSON
  - [ ] Convert normalized motor values to PWM (1000-2000)
- [ ] **Implement SimulatorAdapter trait**
  - [ ] `connect()` - bind sockets
  - [ ] `disconnect()` - close sockets
  - [ ] `receive_sensors()` - UDP recv + parse
  - [ ] `send_actuators()` - serialize + UDP send
  - [ ] `step()` - send step signal for lockstep
  - [ ] `supports_lockstep()` - return true
- [ ] **Unit tests (mock socket)**
  - [ ] Test JSON parsing
  - [ ] Test JSON serialization
  - [ ] Test PWM conversion
- [ ] **Integration tests (optional, requires Gazebo)**
  - [ ] Document Gazebo setup
  - [ ] Test connection to real Gazebo
- [ ] **Verification**
  - [ ] `cargo fmt`
  - [ ] `cargo clippy --all-targets -- -D warnings`
  - [ ] `cargo test -p pico_trail_sitl --lib`

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

- [ ] All phases completed
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test -p pico_trail_sitl --lib`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover` (verify no impact on embedded)
- [ ] No `unsafe` code
- [ ] Plan checkboxes marked
- [ ] Task README status updated to Implementation Complete
- [ ] Traceability check: `bun scripts/trace-status.ts --check`
