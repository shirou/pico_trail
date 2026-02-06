# T-00157 SITL Core Abstractions and Bridge Plan

## Metadata

- Type: Implementation Plan
- Status: Implementation Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement the SITL crate foundation in 2 phases: core abstractions (trait, types, errors) followed by the bridge orchestrator (adapter registry, vehicle management, time mode).

## Success Metrics

- [x] `SimulatorAdapter` trait defined and object-safe
- [x] `SitlBridge` manages adapters and vehicles
- [x] Normalized data types compile and can be instantiated
- [x] Error types cover all failure modes
- [x] New adapters require 0 lines of bridge change

## Scope

- Goal: SITL crate with core abstractions and bridge orchestrator
- Non-Goals: Adapter implementations, platform trait, CI integration
- Assumptions: Host-only (not embedded), tokio async runtime
- Constraints: Trait must be object-safe (Send + Sync)

## ADR & Legacy Alignment

- [x] Confirm ADR-00156-sitl-pluggable-adapter-architecture is referenced
- [x] Follow ADR-00003 Platform trait patterns
- [x] No modifications to existing crates/core or crates/firmware

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

- [x] **Create crate structure**
  - [x] Create `crates/sitl/Cargo.toml` with dependencies (tokio, async-trait, serde, thiserror)
  - [x] Create `crates/sitl/src/lib.rs` with module declarations
  - [x] Add `sitl` to workspace in root `Cargo.toml`
- [x] **Define error types**
  - [x] Create `SimulatorError` enum in `error.rs`
  - [x] Derive `thiserror::Error` for error messages
- [x] **Define data types**
  - [x] `VehicleId(u8)` with Debug, Clone, Copy, PartialEq, Eq, Hash
  - [x] `SensorData` struct with optional sensor fields
  - [x] `ImuData`, `GpsData`, `CompassData`, `BarometerData` structs
  - [x] `ActuatorCommands` struct with motors/servos vectors
  - [x] `GpsFixType` enum
- [x] **Define SimulatorAdapter trait**
  - [x] Use `#[async_trait]` for async methods
  - [x] Add `Send + Sync` bounds for object safety
  - [x] `adapter_type()`, `name()` for identification
  - [x] `connect()`, `disconnect()`, `is_connected()` for lifecycle
  - [x] `receive_sensors()`, `send_actuators()` for data exchange
  - [x] `step()`, `sim_time_us()`, `supports_lockstep()` for time control
  - [x] `capabilities()` for feature discovery
- [x] **Define SimulatorCapabilities**
  - [x] `SensorCapabilities` struct (imu, gps, compass, etc. booleans)
  - [x] `SimulatorCapabilities` struct (sensors, max_rate_hz, multi_vehicle, terrain, wind)
- [x] **Unit tests**
  - [x] Test `VehicleId` derives work correctly
  - [x] Test `SensorData` can be constructed and cloned
  - [x] Test trait is object-safe (`Box<dyn SimulatorAdapter>` compiles)
- [x] **Verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `cargo test -p pico_trail_sitl --lib`

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

- [x] **Define TimeMode enum**
  - [x] `FreeRunning`
  - [x] `Lockstep { step_size_us: u64 }`
  - [x] `Scaled { factor: f32 }`
- [x] **Define VehicleConfig struct**
  - [x] `id: VehicleId`
  - [x] `mavlink_port: u16` (default 14550 + id)
  - [x] `vehicle_type: VehicleType`
  - [x] `initial_position: Option<GeoPosition>`
- [x] **Implement SitlBridge**
  - [x] `new()` constructor
  - [x] `register_adapter()` - add to HashMap, error on duplicate
  - [x] `unregister_adapter()` - remove, error if not found
  - [x] `list_adapters()` - return names
  - [x] `get_adapter()` / `get_adapter_mut()` - lookup by name
- [x] **Implement vehicle management**
  - [x] `spawn_vehicle()` - create VehicleInstance, add to map
  - [x] `despawn_vehicle()` - remove from map
  - [x] `assign_vehicle_to_adapter()` - update mapping
  - [x] `list_vehicles()` - return VehicleIds
- [x] **Implement basic step**
  - [x] `step()` - placeholder for single iteration
  - [x] `set_time_mode()` - configure time mode
- [x] **Unit tests**
  - [x] Test adapter registration/unregistration
  - [x] Test duplicate adapter name error
  - [x] Test vehicle spawn/despawn
  - [x] Test vehicle-adapter assignment
  - [x] Test list operations
- [x] **Verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `cargo test -p pico_trail_sitl --lib`

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

- [x] All phases completed
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test -p pico_trail_sitl --lib`
- [x] `./scripts/build-rp2350.sh pico_trail_rover` (verify no impact on embedded)
- [x] No `unsafe` code in sitl crate
- [x] Plan checkboxes marked
- [x] Task README status updated to Implementation Complete
- [x] Traceability check: `bun scripts/trace-status.ts --check`
