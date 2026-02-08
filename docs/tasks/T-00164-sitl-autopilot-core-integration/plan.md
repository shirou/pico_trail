# T-00164 SITL Autopilot Core Integration

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Add embassy-sync and embassy-futures as optional dependencies of `pico_trail_core`, then migrate the autopilot integration layer from firmware to core in 5 phases. SITL consumes core's autopilot types to achieve full closed-loop GCS control with all rover modes.

## Success Metrics

- [ ] Core tests pass: `cargo test -p pico_trail_core --features embassy --lib`
- [ ] Core without embassy unchanged: `cargo test -p pico_trail_core --lib`
- [ ] Firmware builds: `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] SITL closed-loop: Mission Planner arms and steers rover in Gazebo
- [ ] All rover modes functional in SITL

## Scope

- Goal: Migrate autopilot layer from firmware to core; wire up SITL with core's autopilot
- Non-Goals: Embassy-time dependent tasks (parameter saver, arming tasks), firmware-specific param modules (WifiParams, BoardParams)
- Assumptions: T-00157 through T-00160 complete (SITL bridge, adapters, multi-vehicle, lockstep)
- Constraints: Core remains `no_std`; embassy deps are optional behind feature flag

## ADR & Legacy Alignment

- [ ] Confirm ADR-00161 (embassy-sync in core, full migration) is followed
- [ ] Verify core without `embassy` feature remains unchanged at each phase

## Plan Summary

| Phase | Description                              | Effort |
| ----- | ---------------------------------------- | ------ |
| 1     | Core embassy feature and sync primitives | 3 days |
| 2     | Autopilot state types migration          | 3 days |
| 3     | Communication layer migration            | 5 days |
| 4     | Mode management and implementations      | 4 days |
| 5     | SITL autopilot integration               | 5 days |

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Core Embassy Feature and Sync Primitives

### Goal

- Add embassy-sync and embassy-futures as optional dependencies of core behind `embassy` feature
- Migrate SharedState trait + EmbassyState + MockState from firmware to core
- Verify core compiles with and without the embassy feature

### Inputs

- Source Code to Read:
  - `crates/firmware/src/core/traits/sync.rs` — SharedState, EmbassyState, MockState
  - `crates/core/Cargo.toml` — current dependencies
  - `crates/firmware/Cargo.toml` — embassy git dependency specs
- Dependencies:
  - `embassy-sync` git dependency (same rev as firmware)
  - `embassy-futures` git dependency (same rev as firmware)
  - `critical-section` promoted from dev-dep to dep

### Tasks

- [ ] **Update core Cargo.toml**
  - [ ] Add `embassy` feature flag: `embassy = ["dep:embassy-sync", "dep:embassy-futures"]`
  - [ ] Add `embassy-sync` as optional dependency (same git rev as firmware)
  - [ ] Add `embassy-futures` as optional dependency (same git rev as firmware)
  - [ ] Promote `critical-section = "1.2"` from dev-dependency to dependency
  - [ ] Keep `critical-section = { features = ["std"] }` in dev-dependencies for host tests
- [ ] **Migrate SharedState trait to core**
  - [ ] Create `crates/core/src/traits/sync.rs`
  - [ ] Move `SharedState<T>` trait (not feature-gated — it's a pure trait)
  - [ ] Move `EmbassyState<T>` behind `#[cfg(feature = "embassy")]`
  - [ ] Move `MockState<T>` (not feature-gated — useful for all tests)
  - [ ] Add `impl SharedState<T> for EmbassyState<T>` behind embassy gate
  - [ ] Add `impl SharedState<T> for MockState<T>`
  - [ ] Export from `crates/core/src/traits/mod.rs`
- [ ] **Update core lib.rs**
  - [ ] Add `pub mod traits;` with `sync` submodule
  - [ ] Verify `#![no_std]` is preserved
- [ ] **Update firmware imports**
  - [ ] Change `firmware/src/core/traits/sync.rs` to re-export from core
  - [ ] Or update all firmware `use crate::core::traits::SharedState` to `use pico_trail_core::traits::sync::SharedState`
  - [ ] Update firmware Cargo.toml: `pico_trail_core = { path = "../core", features = ["embassy", "defmt"] }`
- [ ] **Unit tests**
  - [ ] Test MockState with/with_mut access
  - [ ] Test EmbassyState with/with_mut access (requires embassy feature in test)
  - [ ] Test SharedState trait object usage
- [ ] **Verification**
  - [ ] `cargo test -p pico_trail_core --lib --quiet` (without embassy — must pass unchanged)
  - [ ] `cargo test -p pico_trail_core --features embassy --lib --quiet`
  - [ ] `cargo test -p pico_trail_firmware --lib --quiet`
  - [ ] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- Core crate with `embassy` feature flag and SharedState trait
- Firmware re-exports or updated imports

### Acceptance Criteria (Phase Gate)

- Core compiles with and without `embassy` feature
- SharedState trait + implementations work on host
- Firmware builds for RP2350 without regression
- All existing tests pass

### Rollback/Fallback

- Remove embassy feature and revert sync.rs; no other crates affected

---

## Phase 2: Autopilot State Types Migration

### Goal

- Move GpsPosition from firmware to core
- Move SystemState and all supporting types to core
- Move VehicleType trait and implementations to core

### Inputs

- Dependencies:
  - Phase 1: SharedState trait in core
  - `GpsFixType` already in core (`core/src/navigation/types.rs`)
- Source Code to Migrate:
  - `firmware/src/devices/gps.rs` — `GpsPosition` struct
  - `firmware/src/communication/mavlink/state.rs` — `ArmedState`, `FlightMode`, `BatteryState`, `AttitudeState`, `HomePosition`, `SystemState`
  - `firmware/src/communication/mavlink/vehicle/` — `VehicleType`, `FlightModeOps`, `GroundRover`, `SurfaceBoat`

### Tasks

- [ ] **Move GpsPosition to core**
  - [ ] Add `GpsPosition` struct to `crates/core/src/navigation/types.rs` (alongside existing `GpsFixType`)
  - [ ] Change `defmt::Format` to `#[cfg_attr(feature = "defmt", derive(defmt::Format))]`
  - [ ] Update firmware to re-export or import from core
  - [ ] Update all firmware references to GpsPosition
- [ ] **Create autopilot module in core**
  - [ ] Create `crates/core/src/autopilot/mod.rs`
  - [ ] Create `crates/core/src/autopilot/state.rs`
  - [ ] Create `crates/core/src/autopilot/vehicle.rs`
  - [ ] Export from `crates/core/src/lib.rs`
- [ ] **Move state types to core**
  - [ ] Move `ArmedState` — no changes needed
  - [ ] Move `FlightMode` — change `defmt::Format` to `cfg_attr`
  - [ ] Move `BatteryState` — change `defmt::Format` to `cfg_attr`
  - [ ] Move `AttitudeState` — change `defmt::Format` to `cfg_attr`
  - [ ] Move `HomePosition` — no changes needed
  - [ ] Move `SystemState` — update GpsPosition import to core's
  - [ ] Move `FlightModeOps` trait — update imports
- [ ] **Move VehicleType to core**
  - [ ] Move `VehicleType` trait to `core/src/autopilot/vehicle.rs`
  - [ ] Move `FlightModeOps` trait
  - [ ] Move `GroundRover` and `SurfaceBoat` implementations
  - [ ] Update `defmt::Format` derives to `cfg_attr`
- [ ] **Update firmware imports**
  - [ ] Replace all `use crate::communication::mavlink::state::*` with core imports
  - [ ] Replace all `use crate::communication::mavlink::vehicle::*` with core imports
  - [ ] Replace all `use crate::devices::gps::GpsPosition` with core imports
  - [ ] Optionally keep firmware modules as re-exports for minimal diff
- [ ] **Unit tests**
  - [ ] Test SystemState default construction
  - [ ] Test FlightMode to/from custom_mode conversion
  - [ ] Test ArmedState transitions
  - [ ] Test GpsPosition creation with GpsFixType
- [ ] **Verification**
  - [ ] `cargo test -p pico_trail_core --features embassy --lib --quiet`
  - [ ] `cargo test -p pico_trail_core --lib --quiet` (without embassy)
  - [ ] `cargo test -p pico_trail_firmware --lib --quiet`
  - [ ] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- Core autopilot module with state types and vehicle types
- GpsPosition in core's navigation module
- Updated firmware imports

### Acceptance Criteria (Phase Gate)

- All autopilot state types accessible from core
- VehicleType trait and implementations in core
- Firmware compiles with updated imports
- All existing tests pass

### Rollback/Fallback

- State types are data-only — revert by restoring firmware copies and import paths

---

## Phase 3: Communication Layer Migration

### Goal

- Move FlashInterface trait, ParamHandler, MessageDispatcher and all handlers from firmware to core
- Move RC_INPUT global and status_notifier to core

### Inputs

- Dependencies:
  - Phase 2: SystemState, VehicleType in core
  - Phase 1: SharedState, EmbassyState in core
  - Mission types already in core (`core/src/mission/`)
  - RC types already in core (`core/src/rc/`)
  - ParameterStore already in core (`core/src/parameters/storage.rs`)
- Source Code to Migrate:
  - `firmware/src/platform/traits/flash.rs` — `FlashInterface` trait
  - `firmware/src/parameters/storage.rs` — `load_from_flash`, `save_to_flash` functions
  - `firmware/src/communication/mavlink/dispatcher.rs`
  - `firmware/src/communication/mavlink/status_notifier.rs`
  - `firmware/src/communication/mavlink/handlers/param.rs`
  - `firmware/src/communication/mavlink/handlers/command.rs`
  - `firmware/src/communication/mavlink/handlers/telemetry.rs`
  - `firmware/src/communication/mavlink/handlers/mission.rs`
  - `firmware/src/communication/mavlink/handlers/rc_input.rs`
  - `firmware/src/communication/mavlink/handlers/navigation.rs`

### Tasks

- [ ] **Move FlashInterface trait to core**
  - [ ] Create `crates/core/src/traits/flash.rs`
  - [ ] Move `FlashInterface` trait (pure interface — `read`, `write`, `erase`, `block_size`, `capacity`)
  - [ ] Export from `crates/core/src/traits/mod.rs`
  - [ ] Firmware re-exports or updates imports (implementations RP2350Flash, MockFlash stay in firmware)
- [ ] **Move Flash serialization functions to core**
  - [ ] Move `load_from_flash<F: FlashInterface>()` from `firmware/src/parameters/storage.rs` to `core/src/parameters/storage.rs`
  - [ ] Move `save_to_flash<F: FlashInterface>()` similarly
  - [ ] These only depend on `ParameterStore` (already in core) and `FlashInterface` (moved above)
- [ ] **Create communication module in core**
  - [ ] Create `crates/core/src/communication/mod.rs`
  - [ ] Create `crates/core/src/communication/dispatcher.rs`
  - [ ] Create `crates/core/src/communication/handlers/` directory and `mod.rs`
  - [ ] Export from `crates/core/src/lib.rs` behind `embassy` feature
- [ ] **Move RC_INPUT global to core**
  - [ ] Move RC_INPUT static (uses `EmbassyState<RcInput>`) to core (behind embassy feature)
  - [ ] Firmware re-exports from core
- [ ] **Move status_notifier to core**
  - [ ] Move `StatusNotifier` struct, `NOTIFIER` global, and all helper functions (`send_info`, `send_warning`, `send_error`, etc.)
  - [ ] Move `take_pending_statustext_messages()` and `chunk_message()`
  - [ ] No embassy deps — uses only `critical_section::Mutex` and `heapless` (both already in core)
- [ ] **Move ParamHandler to core**
  - [ ] Move `ParamHandler` struct to `core/src/communication/handlers/param.rs`
  - [ ] Refactor constructor: `new(store: ParameterStore)` — caller passes pre-initialized store
  - [ ] Keep `save_to_flash<F: FlashInterface>()` method generic over platform
  - [ ] Firmware-specific param modules (`WifiParams`, `BoardParams`) stay in firmware and call `register_defaults()` before constructing `ParamHandler`
- [ ] **Move other handler implementations**
  - [ ] Move `CommandHandler<V>` — depends on SystemState, VehicleType (now in core)
  - [ ] Move `TelemetryStreamer<V>` — depends on SystemState, VehicleType
  - [ ] Move `MissionHandler` — depends on mission types (already in core)
  - [ ] Move `RcInputHandler` — depends on RC_INPUT (moved above), embassy-sync
  - [ ] Move `NavigationHandler` — depends on navigation types (already in core)
- [ ] **Move MessageDispatcher**
  - [ ] Move `MessageDispatcher<V>` struct (now includes `ParamHandler`)
  - [ ] Move `dispatch()`, `update_telemetry()`, `process_rc_input()` methods
  - [ ] Move `DispatcherStats`, `ConnectionState`
- [ ] **Update firmware imports**
  - [ ] Replace all dispatcher/handler imports with core paths
  - [ ] Firmware constructs `ParameterStore` (load from Flash + register defaults), passes to `ParamHandler::new()`
  - [ ] Firmware re-exports FlashInterface implementations
- [ ] **Unit tests**
  - [ ] Test dispatcher routes messages to correct handlers
  - [ ] Test ParamHandler PARAM_REQUEST_LIST / PARAM_SET with MockFlash
  - [ ] Test CommandHandler ARM/DISARM produces correct ACK
  - [ ] Test CommandHandler SET_MODE updates SystemState
  - [ ] Test RcInputHandler normalizes channels correctly
  - [ ] Test TelemetryStreamer builds correct MAVLink messages
  - [ ] Test MissionHandler upload/download state machine
- [ ] **Verification**
  - [ ] `cargo test -p pico_trail_core --features embassy --lib --quiet`
  - [ ] `cargo test -p pico_trail_firmware --lib --quiet`
  - [ ] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- FlashInterface trait in core
- Flash serialization functions (`load_from_flash`, `save_to_flash`) in core
- Core communication module with dispatcher and all handlers (including ParamHandler)
- RC_INPUT global and status_notifier in core

### Acceptance Criteria (Phase Gate)

- MessageDispatcher and handlers compile and test in core
- Firmware compiles with updated dispatcher imports
- All existing firmware tests pass
- Embedded build succeeds

### Rollback/Fallback

- Restore firmware dispatcher/handlers, revert core communication module

---

## Phase 4: Mode Management and Implementations

### Goal

- Move ModeManager and all mode implementations from firmware to core
- ManualMode gated behind `embassy` feature (uses embassy-sync)

### Inputs

- Dependencies:
  - Phase 3: MessageDispatcher, RC_INPUT in core
  - Phase 2: SystemState in core
  - Phase 1: SharedState, EmbassyState in core
  - Mode trait already partially defined in core (`core/src/mode/`)
  - Navigation types already in core
- Source Code to Migrate:
  - `firmware/src/rover/mode_manager.rs` — ModeManager
  - `firmware/src/rover/mode/manual.rs` — ManualMode (embassy-sync)
  - `firmware/src/rover/mode/auto.rs` — AutoMode
  - `firmware/src/rover/mode/guided.rs` — GuidedMode
  - `firmware/src/rover/mode/rtl.rs` — RtlMode
  - `firmware/src/rover/mode/loiter.rs` — LoiterMode
  - `firmware/src/rover/mode/circle.rs` — CircleMode
  - `firmware/src/rover/mode/smartrtl.rs` — SmartRtlMode

### Tasks

- [ ] **Move ModeManager to core**
  - [ ] Move to `crates/core/src/mode/mode_executor.rs` (avoid "manager" naming per AGENTS.md)
  - [ ] Update SystemState reference to core's type
  - [ ] Gate behind `embassy` feature (ModeManager uses SystemState which may need embassy for globals)
- [ ] **Move ManualMode to core**
  - [ ] Move to `crates/core/src/mode/manual.rs`
  - [ ] Gate behind `embassy` feature (uses `embassy_sync::Mutex`, `embassy_futures::block_on`)
  - [ ] Update RC_INPUT import to core's global
- [ ] **Move non-embassy modes to core**
  - [ ] Move `AutoMode` to `crates/core/src/mode/auto.rs`
  - [ ] Move `GuidedMode` to `crates/core/src/mode/guided.rs`
  - [ ] Move `RtlMode` to `crates/core/src/mode/rtl.rs`
  - [ ] Move `LoiterMode` to `crates/core/src/mode/loiter.rs`
  - [ ] Move `CircleMode` to `crates/core/src/mode/circle.rs`
  - [ ] Move `SmartRtlMode` to `crates/core/src/mode/smartrtl.rs`
  - [ ] Update navigation/mission imports to core paths
- [ ] **Update firmware imports**
  - [ ] Replace all mode imports with core paths
  - [ ] Firmware's rover module delegates to core's mode implementations
- [ ] **Unit tests**
  - [ ] Test ModeExecutor mode transitions (enter/exit/fallback)
  - [ ] Test ManualMode update with mock RC input
  - [ ] Test AutoMode update with mock waypoints
  - [ ] Test mode execution at 50Hz timing
- [ ] **Verification**
  - [ ] `cargo test -p pico_trail_core --features embassy --lib --quiet`
  - [ ] `cargo test -p pico_trail_firmware --lib --quiet`
  - [ ] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- All mode implementations in core
- ModeExecutor in core
- Firmware delegates to core modes

### Acceptance Criteria (Phase Gate)

- All modes compile and test in core
- ModeExecutor transitions work correctly
- ManualMode works with embassy-sync on host
- Firmware compiles with updated mode imports
- Embedded build succeeds

### Rollback/Fallback

- Restore firmware mode implementations, revert core mode additions

---

## Phase 5: SITL Autopilot Integration

### Goal

- Wire SITL to use core's MessageDispatcher, ModeExecutor, and mode implementations
- Achieve closed-loop: GCS command → mode → actuator → Gazebo → sensor → telemetry → GCS
- Integration tests with LightweightAdapter

### Inputs

- Dependencies:
  - Phase 4: All modes in core
  - Phase 3: MessageDispatcher in core
  - Phase 2: SystemState, VehicleType in core
  - Phase 1: SharedState in core
  - Existing SITL infrastructure (SitlBridge, SitlPlatform, GcsLink, adapters)
- Source Code to Modify:
  - `crates/sitl/Cargo.toml` — add `embassy` feature to core dependency
  - `crates/sitl/src/bin/gazebo_bridge.rs` — integrate dispatcher and mode executor
  - `crates/sitl/src/gcs/mod.rs` — delegate incoming to dispatcher
  - `crates/sitl/src/platform/mod.rs` — `set_pwm_duty()` for motor output

### Tasks

- [ ] **Update SITL Cargo.toml**
  - [ ] Add `embassy` feature to core dependency: `pico_trail_core = { features = ["embassy"] }`
  - [ ] Add `critical-section = { features = ["std"] }` for host critical-section impl
- [ ] **Add SitlPlatform::set_pwm_duty()**
  - [ ] Add method to set PWM duty cycle by channel index
  - [ ] Unit test for set_pwm_duty
- [ ] **Integrate dispatcher into GcsLink**
  - [ ] Create `MessageDispatcher<GroundRover>` in vehicle setup
  - [ ] In `handle_incoming()`, delegate to `dispatcher.dispatch()`
  - [ ] Forward responses back to GCS
  - [ ] Process RC input via `dispatcher.process_rc_input()`
- [ ] **Integrate ModeExecutor into step loop**
  - [ ] Create `ModeExecutor` per vehicle with initial ManualMode
  - [ ] Call `mode_executor.execute(sim_time_us)` each step
  - [ ] Mode writes actuator outputs via SitlPlatform's actuator interface
- [ ] **Update gazebo_bridge main loop**
  - [ ] Process GCS commands → dispatcher before bridge.step()
  - [ ] Execute mode manager after command processing
  - [ ] bridge.step() sends actuators and receives sensors
  - [ ] Update SystemState with sensor data
  - [ ] Send telemetry from dispatcher
- [ ] **Update heartbeat with vehicle state**
  - [ ] Heartbeat reflects armed state from SystemState
  - [ ] Heartbeat reflects current mode from SystemState
- [ ] **Integration tests**
  - [ ] `test_closed_loop_control` — ARM, RC override forward, step, verify non-zero actuator commands
  - [ ] `test_disarmed_no_output` — DISARM, RC override, verify zero actuator output
  - [ ] `test_mode_change` — SET_MODE, verify mode in heartbeat
  - [ ] `test_multi_vehicle_routing` — Two vehicles, command one, verify other is unaffected
- [ ] **Documentation**
  - [ ] Update `crates/sitl/README.md` with GCS control section
  - [ ] Document supported commands and all modes
- [ ] **Build verification**
  - [ ] `cargo test -p pico_trail_sitl --lib --quiet`
  - [ ] `cargo test -p pico_trail_sitl --test '*' --quiet`
  - [ ] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- SITL integrated with core's autopilot (dispatcher, modes, state)
- Closed-loop GCS control working with Gazebo
- Integration tests demonstrating all functionality
- Updated SITL README

### Acceptance Criteria (Phase Gate)

- Mission Planner can arm/disarm SITL rover
- Joystick control moves rover in Gazebo
- All rover modes selectable and functional
- Multi-vehicle command routing works
- All tests pass (unit + integration)
- Embedded build still compiles

### Rollback/Fallback

- SITL integration changes are additive; existing GcsLink telemetry-only path can be restored

---

## Definition of Done

- [ ] Core crate: `cargo test -p pico_trail_core --features embassy --lib --quiet` passes
- [ ] Core crate (no embassy): `cargo test -p pico_trail_core --lib --quiet` passes
- [ ] Firmware crate: `cargo test -p pico_trail_firmware --lib --quiet` passes
- [ ] SITL crate: `cargo test -p pico_trail_sitl --lib --quiet` passes
- [ ] SITL integration: `cargo test -p pico_trail_sitl --test '*' --quiet` passes
- [ ] Embedded build: `./scripts/build-rp2350.sh pico_trail_rover` succeeds
- [ ] Code formatted: `cargo fmt`
- [ ] Linting passes: `cargo clippy --all-targets -- -D warnings`
- [ ] SITL README updated with GCS control documentation
- [ ] ADR-00161 scope fully implemented

## Open Questions

- [ ] Does Mission Planner send `MANUAL_CONTROL` or `RC_CHANNELS_OVERRIDE` for joystick? → Method: Test with live connection
- [ ] What ArduPilot custom_mode values for rover? → Method: Check ArduPilot source (0=Manual, 3=Steering, 4=Hold, 5=Loiter, 10=Auto, 15=Guided, 11=RTL)
- [x] Should `status_notifier` move to core or be abstracted? → **Move to core** — zero embassy deps (uses `critical_section::Mutex` + `heapless`), required by CommandHandler/MissionHandler
- [x] Should `ParamHandler` move to core? → **Yes** — `FlashInterface` is a pure trait (no firmware deps), `ParameterStore` is already in core, refactor constructor to accept pre-initialized store so firmware-specific param modules (`WifiParams`, `BoardParams`) stay in firmware
- [x] Which `SystemState` fields need firmware-only initialization? → **None** — `Default`/`init()` use constant values only. Runtime parameter values are written by `ParamHandler`, not at construction.
