# T-nazbq Firmware-to-Core Migration Plan

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [T-nazbq-design](./design.md)

## Overview

Phased migration of platform-independent code from `crates/firmware` to `crates/core`. Three phases: duplicate deletion, pure algorithm migration, and mixed module extraction.

## Success Metrics

- [x] Zero functionally identical code between crates
- [x] All migrated algorithms have unit tests in core
- [x] `cargo test -p pico_trail_core --lib --quiet` passes
- [x] `./scripts/build-rp2350.sh pico_trail_rover` compiles successfully
- [x] `./scripts/check-core-no-cfg.sh` passes

## Scope

- Goal: Move \~1,600 lines of platform-independent code from firmware to core
- Non-Goals: Rover mode refactoring, MAVLink refactoring, device driver changes
- Assumptions: Core crate already has the module structure from T-3n2ej
- Constraints: Zero `cfg` in core, `no_std` only, firmware build must not break

## ADR & Legacy Alignment

- [x] T-3n2ej workspace separation is complete (all 8 phases done)
- [ ] Verify firmware imports are updated after each phase to avoid duplicate symbol errors

## Plan Summary

- Phase 1 – Delete duplicate code from firmware
- Phase 2 – Migrate pure algorithms (DCM, geographic calculations)
- Phase 3 – Extract mixed module logic (path recorder, navigation controller, AHRS traits, heading source, mission state)

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: Delete Duplicate Code

### Goal

- Eliminate code that exists identically in both `crates/core` and `crates/firmware`, establishing core as the single source of truth

### Inputs

- Source Code to Modify:
  - `crates/firmware/src/core/arming/error.rs` – Delete (duplicate of core)
  - `crates/firmware/src/core/arming/mod.rs` – Remove `error` module declaration
  - `crates/firmware/src/libraries/kinematics/differential_drive.rs` – Delete (duplicate of core)
  - `crates/firmware/src/libraries/kinematics/mod.rs` – Remove module declaration or delete directory
  - `crates/firmware/src/core/parameters/block.rs` – Delete (duplicate of core)
  - `crates/firmware/src/core/parameters/crc.rs` – Delete (duplicate of core)
  - `crates/firmware/src/core/parameters/mod.rs` – Remove `block` and `crc` module declarations, import from core

### Tasks

- [x] **Delete arming/error.rs duplicate**
  - [x] Verify `crates/core/src/arming/error.rs` contains the canonical implementation
  - [x] Delete `crates/firmware/src/core/arming/error.rs`
  - [x] Update `crates/firmware/src/core/arming/mod.rs` to import `ArmingError`, `DisarmError`, `CheckCategory` from `pico_trail_core::arming`
  - [x] Fix all firmware references that used the local `error` module

- [x] **Delete kinematics duplicate**
  - [x] Verify `crates/core/src/kinematics/differential_drive.rs` contains the canonical implementation
  - [x] Delete `crates/firmware/src/libraries/kinematics/differential_drive.rs`
  - [x] Update or delete `crates/firmware/src/libraries/kinematics/mod.rs`
  - [x] Fix all firmware references to use `pico_trail_core::kinematics::DifferentialDrive`

- [x] **Delete parameters/block.rs and crc.rs duplicates**
  - [x] Verify `crates/core/src/parameters/block.rs` and `crc.rs` contain the canonical implementations
  - [x] Delete `crates/firmware/src/core/parameters/block.rs`
  - [x] Delete `crates/firmware/src/core/parameters/crc.rs`
  - [x] Update `crates/firmware/src/core/parameters/mod.rs` to import block/crc types from `pico_trail_core::parameters`
  - [x] Fix all firmware references

### Deliverables

- 4 duplicate files deleted from firmware
- Firmware imports updated to use `pico_trail_core`
- \~560 lines of duplication eliminated

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/check-core-no-cfg.sh
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- No duplicate implementations between crates
- All firmware code compiles using core imports
- Existing tests pass unchanged
- Embedded build succeeds

### Rollback/Fallback

- `git revert` to restore deleted files if firmware build breaks

---

## Phase 2: Migrate Pure Algorithms

### Phase 2 Goal

- Move algorithms with zero platform dependencies from firmware to core

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Duplicate deletion complete
- Source Code to Migrate:
  - `crates/firmware/src/subsystems/ahrs/dcm.rs` → `crates/core/src/ahrs/dcm.rs`
  - `crates/firmware/src/subsystems/navigation/geo.rs` → `crates/core/src/navigation/geo.rs`
- Source Code to Modify:
  - `crates/core/src/mode/nav.rs` – Remove duplicated functions after consolidation
  - `crates/core/src/ahrs/mod.rs` – Add `dcm` module declaration
  - `crates/core/src/navigation/mod.rs` – Add `geo` module declaration

### Phase 2 Tasks

- [x] **Migrate DCM algorithm**
  - [x] Copy `crates/firmware/src/subsystems/ahrs/dcm.rs` to `crates/core/src/ahrs/dcm.rs`
  - [x] Remove `crate::log_warn!()` call (replace with silent fallback or remove guard)
  - [x] Update imports to use `libm` instead of `micromath::F32Ext` for host compatibility
  - [x] Add `pub mod dcm;` to `crates/core/src/ahrs/mod.rs`
  - [x] Verify unit tests compile and pass on host
  - [x] Update firmware to import DCM from `pico_trail_core::ahrs::dcm`
  - [x] Replace firmware's local `dcm.rs` with re-export

- [x] **Consolidate geographic algorithms**
  - [x] Create `crates/core/src/navigation/geo.rs` with functions from firmware's `geo.rs`
  - [x] Include: `calculate_bearing`, `calculate_distance`, `wrap_180`, `wrap_360`, `offset_position`
  - [x] Add `pub mod geo;` to `crates/core/src/navigation/mod.rs`
  - ~~Define `Position` type~~ — Deferred to Phase 3; `PositionTarget` already exists in `types.rs`
  - [x] Identify duplicate functions in `crates/core/src/mode/nav.rs` (`haversine_distance_bearing`, `normalize_angle`)
  - [x] Update `mode/nav.rs` to delegate to `navigation/geo` (re-exports for backwards compatibility)
  - [x] Move existing unit tests from firmware geo.rs to core
  - [x] Update firmware to import from `pico_trail_core::navigation::geo`
  - [x] Replace firmware's local `geo.rs` with re-export

### Phase 2 Deliverables

- DCM algorithm in `crates/core/src/ahrs/dcm.rs`
- Consolidated geographic functions in `crates/core/src/navigation/geo.rs`
- `Position` type in core (if not already present)
- \~350 lines migrated

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/check-core-no-cfg.sh
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- DCM algorithm tests pass on host
- Geographic function tests pass on host
- No duplicate navigation math functions
- Firmware builds using core imports
- Zero `cfg` in core

### Phase 2 Rollback/Fallback

- Revert individual file moves; firmware keeps local copies until verified

---

## Phase 3: Extract Mixed Module Logic

### Phase 3 Goal

- Decouple platform-independent business logic from state management wrappers and move to core

### Phase 3 Inputs

- Dependencies:
  - Phase 2: Geographic algorithms and DCM in core (used by controller, heading, path recorder)
- Source Code to Refactor:
  - `crates/firmware/src/subsystems/navigation/path_recorder.rs`
  - `crates/firmware/src/subsystems/navigation/controller.rs`
  - `crates/firmware/src/subsystems/navigation/heading.rs`
  - `crates/firmware/src/subsystems/ahrs/traits.rs`
  - `crates/firmware/src/core/mission/state.rs`

### Phase 3 Tasks

- [x] **Extract AHRS traits**
  - [x] Create `crates/core/src/ahrs/traits.rs` with `Ahrs` trait and `AhrsState` struct
  - [x] Remove `SharedAhrsState` (critical-section wrapper) from extracted code — keep in firmware
  - [x] Add `pub mod traits;` to `crates/core/src/ahrs/mod.rs`
  - [x] Update firmware `ahrs/traits.rs` to import core traits and only define `SharedAhrsState` locally
  - [x] Add unit tests for `AhrsState` in core

- [x] **Extract path recorder algorithm**
  - [x] Create `crates/core/src/navigation/path_recorder.rs` with `PathPoint` and `PathRecorder`
  - [x] Replace `crate::log_info!()` / `crate::log_debug!()` calls with no-ops or remove
  - [x] Update distance calculation to use `crate::navigation::geo::calculate_distance`
  - [x] Add `pub mod path_recorder;` to `crates/core/src/navigation/mod.rs`
  - [x] Keep `PATH_RECORDER` global static in firmware
  - [x] Update firmware to import `PathRecorder` from core
  - [x] Add unit tests for path recording and simplification in core

- [x] **Extract navigation controller**
  - [x] Create `crates/core/src/navigation/controller.rs` with `NavigationController` trait and `SimpleNavigationController`
  - [x] Use core's `Position` type instead of firmware's `GpsPosition`
  - [x] Use core's `navigation::geo` functions for bearing/distance
  - [x] Add `pub mod controller;` to `crates/core/src/navigation/mod.rs`
  - [x] Update firmware to create `SimpleNavigationController` using core type, with conversion from `GpsPosition` to `Position`
  - [x] Add unit tests for steering and throttle calculations in core

- [x] **Extract heading source abstractions**
  - [x] Create `crates/core/src/navigation/heading.rs` with `HeadingSource` trait and `HeadingSourceType` enum
  - [x] Extract `FusedHeadingSource` algorithm to core (replace `SharedAhrsState` dependency with generic trait bound)
  - [x] Add `pub mod heading;` to `crates/core/src/navigation/mod.rs`
  - [x] Keep firmware-specific heading source implementations that use `SharedAhrsState`
  - [x] Add unit tests for heading fusion logic in core

- [x] **Extract mission state machine**
  - [x] Create `crates/core/src/mission/state.rs` with mission state transition logic
  - [x] Remove `EmbassyState` dependency from extracted code
  - [x] Add `pub mod state;` to `crates/core/src/mission/mod.rs`
  - [x] Keep `EmbassyState`-wrapped global in firmware
  - [x] Update firmware `mission/state.rs` to use core state types
  - [x] Add unit tests for state transitions in core

### Phase 3 Deliverables

- AHRS traits in core (without `SharedAhrsState`)
- Path recorder algorithm in core
- Navigation controller trait and implementation in core
- Heading source abstractions in core
- Mission state machine logic in core
- \~700 lines migrated
- Firmware uses core imports for all extracted logic

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/check-core-no-cfg.sh
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- All extracted traits and algorithms have unit tests in core
- Firmware compiles using core imports
- No platform-specific code in core (no `critical_section`, no `EmbassyState`, no logging macros)
- Zero `cfg` in core
- Embedded build succeeds

### Phase 3 Rollback/Fallback

- Each extraction is independent; can revert individual modules without affecting others
- Firmware keeps local copies until each migration is verified

---

## Definition of Done

- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/check-core-no-cfg.sh`
- [x] `./scripts/build-rp2350.sh pico_trail_rover`
- [x] No functionally identical code in both crates
- [x] All migrated algorithms have unit tests in core
- [x] `docs/architecture.md` updated with new module locations
- [x] No `unsafe` in migrated code
- [x] No vague naming ("manager"/"util")

## Open Questions

- [ ] Should `mode/nav.rs` functions be removed or re-exported from `navigation/geo`? → Next step: Evaluate during Phase 2 to check if any external code depends on `mode::nav` paths
- [ ] Should firmware's `GpsPosition` be fully replaced by core's `Position`? → Next step: Count firmware usages and assess impact during Phase 3
- [ ] Does core need `nalgebra` as a dependency for DCM? → Next step: Check if `nalgebra` with `no_std` feature is acceptable for core's Cargo.toml
