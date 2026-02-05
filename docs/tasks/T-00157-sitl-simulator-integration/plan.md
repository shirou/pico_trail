# T-00157 SITL Core Abstractions and Bridge Plan

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement the SITL crate foundation in 2 phases: core abstractions (trait, types, errors) followed by the bridge orchestrator (adapter registry, vehicle management, time mode).

## Success Metrics

- [ ] `SimulatorAdapter` trait defined and object-safe
- [ ] `SitlBridge` manages adapters and vehicles
- [ ] Normalized data types compile and can be instantiated
- [ ] Error types cover all failure modes
- [ ] New adapters require 0 lines of bridge change

## Scope

- Goal: SITL crate with core abstractions and bridge orchestrator
- Non-Goals: Adapter implementations, platform trait, CI integration
- Assumptions: Host-only (not embedded), tokio async runtime
- Constraints: Trait must be object-safe (Send + Sync)

## ADR & Legacy Alignment

- [ ] Confirm ADR-00156-sitl-pluggable-adapter-architecture is referenced
- [ ] Follow ADR-00003 Platform trait patterns
- [ ] No modifications to existing crates/core or crates/firmware

## Plan Summary

| Phase | Description       | Effort    |
| ----- | ----------------- | --------- |
| 1     | Core Abstractions | 1 week    |
| 2     | SITL Bridge       | 1.5 weeks |

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Core Abstractions

### Goal

- Create `crates/sitl` crate structure
- Define `SimulatorAdapter` trait
- Define normalized data types (`SensorData`, `ActuatorCommands`, `VehicleId`)
- Define error types

### Inputs

- Files to Create:
  - `crates/sitl/Cargo.toml`
  - `crates/sitl/src/lib.rs`
  - `crates/sitl/src/error.rs`
  - `crates/sitl/src/types.rs`
  - `crates/sitl/src/adapter/mod.rs`
  - `crates/sitl/src/adapter/capabilities.rs`

### Tasks

- [ ] **Create crate structure**
  - [ ] Create `crates/sitl/Cargo.toml` with dependencies (tokio, async-trait, serde, thiserror)
  - [ ] Create `crates/sitl/src/lib.rs` with module declarations
  - [ ] Add `sitl` to workspace in root `Cargo.toml`
- [ ] **Define error types**
  - [ ] Create `SimulatorError` enum in `error.rs`
  - [ ] Derive `thiserror::Error` for error messages
- [ ] **Define data types**
  - [ ] `VehicleId(u8)` with Debug, Clone, Copy, PartialEq, Eq, Hash
  - [ ] `SensorData` struct with optional sensor fields
  - [ ] `ImuData`, `GpsData`, `CompassData`, `BarometerData` structs
  - [ ] `ActuatorCommands` struct with motors/servos vectors
  - [ ] `GpsFixType` enum
- [ ] **Define SimulatorAdapter trait**
  - [ ] Use `#[async_trait]` for async methods
  - [ ] Add `Send + Sync` bounds for object safety
  - [ ] `adapter_type()`, `name()` for identification
  - [ ] `connect()`, `disconnect()`, `is_connected()` for lifecycle
  - [ ] `receive_sensors()`, `send_actuators()` for data exchange
  - [ ] `step()`, `sim_time_us()`, `supports_lockstep()` for time control
  - [ ] `capabilities()` for feature discovery
- [ ] **Define SimulatorCapabilities**
  - [ ] `SensorCapabilities` struct (imu, gps, compass, etc. booleans)
  - [ ] `SimulatorCapabilities` struct (sensors, max_rate_hz, multi_vehicle, terrain, wind)
- [ ] **Unit tests**
  - [ ] Test `VehicleId` derives work correctly
  - [ ] Test `SensorData` can be constructed and cloned
  - [ ] Test trait is object-safe (`Box<dyn SimulatorAdapter>` compiles)
- [ ] **Verification**
  - [ ] `cargo fmt`
  - [ ] `cargo clippy --all-targets -- -D warnings`
  - [ ] `cargo test -p pico_trail_sitl --lib`

### Deliverables

- New `crates/sitl/` crate with core abstractions

### Acceptance Criteria (Phase Gate)

- `SimulatorAdapter` trait is object-safe
- All data types compile and can be instantiated
- Unit tests pass
- No warnings from clippy

### Rollback/Fallback

- Phase is foundational; rollback = delete crate

---

## Phase 2: SITL Bridge

### Goal

- Implement `SitlBridge` orchestrator
- Adapter registry (register, unregister, list)
- Vehicle management (spawn, despawn, assign)
- Basic simulation step placeholder

### Inputs

- Files to Create:
  - `crates/sitl/src/bridge/mod.rs`
  - `crates/sitl/src/bridge/registry.rs`
  - `crates/sitl/src/bridge/time.rs`
  - `crates/sitl/src/vehicle/mod.rs`
  - `crates/sitl/src/vehicle/config.rs`

### Tasks

- [ ] **Define TimeMode enum**
  - [ ] `FreeRunning`
  - [ ] `Lockstep { step_size_us: u64 }`
  - [ ] `Scaled { factor: f32 }`
- [ ] **Define VehicleConfig struct**
  - [ ] `id: VehicleId`
  - [ ] `mavlink_port: u16` (default 14550 + id)
  - [ ] `vehicle_type: VehicleType`
  - [ ] `initial_position: Option<GeoPosition>`
- [ ] **Implement SitlBridge**
  - [ ] `new()` constructor
  - [ ] `register_adapter()` - add to HashMap, error on duplicate
  - [ ] `unregister_adapter()` - remove, error if not found
  - [ ] `list_adapters()` - return names
  - [ ] `get_adapter()` / `get_adapter_mut()` - lookup by name
- [ ] **Implement vehicle management**
  - [ ] `spawn_vehicle()` - create VehicleInstance, add to map
  - [ ] `despawn_vehicle()` - remove from map
  - [ ] `assign_vehicle_to_adapter()` - update mapping
  - [ ] `list_vehicles()` - return VehicleIds
- [ ] **Implement basic step**
  - [ ] `step()` - placeholder for single iteration
  - [ ] `set_time_mode()` - configure time mode
- [ ] **Unit tests**
  - [ ] Test adapter registration/unregistration
  - [ ] Test duplicate adapter name error
  - [ ] Test vehicle spawn/despawn
  - [ ] Test vehicle-adapter assignment
  - [ ] Test list operations
- [ ] **Verification**
  - [ ] `cargo fmt`
  - [ ] `cargo clippy --all-targets -- -D warnings`
  - [ ] `cargo test -p pico_trail_sitl --lib`

### Deliverables

- `SitlBridge` with adapter and vehicle management

### Acceptance Criteria (Phase Gate)

- FR-00149 adapter registry requirements met
- Vehicle management operations work correctly
- All unit tests pass

### Rollback/Fallback

- Bridge can operate with empty registries

---

## Definition of Done

- [ ] All phases completed
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test -p pico_trail_sitl --lib`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover` (verify no impact on embedded)
- [ ] No `unsafe` code in sitl crate
- [ ] Plan checkboxes marked
- [ ] Task README status updated to Implementation Complete
- [ ] Traceability check: `bun scripts/trace-status.ts --check`
