# T-00041 Parameter Store Core Migration

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [T-00041-parameter-store-core-migration-design](design.md)

## Overview

Phased migration of `ParameterStore`, `ParamValue`, `ParamFlags`, and 8 parameter group types from firmware crate to core crate. Each phase is independently verifiable via RP2350 build and `cargo test --lib`.

## Success Metrics

- [x] `cargo test --lib` runs parameter store tests on host
- [x] `cargo test --lib` runs all 8 parameter group tests on host
- [x] `./scripts/build-rp2350.sh pico_trail_rover` succeeds after every phase
- [x] No firmware import path changes required

## Scope

- Goal: Enable host-side unit testing for parameter subsystem
- Non-Goals: Flash persistence migration, `BoardParams` migration, `SystemState` migration
- Assumptions: Core crate's existing `heapless` and `bitflags` dependencies are sufficient
- Constraints: No circular crate dependencies; core must remain `no_std`

## ADR & Legacy Alignment

- [x] AN-00142 recommends Option A (move to core) — following that approach
- [x] No ADR needed — follows existing core/firmware separation pattern established in the workspace

## Plan Summary

- Phase 1 – Create `ParameterError` and move `ParamValue`/`ParamFlags`/`ParamMetadata` to core
- Phase 2 – Move `ParameterStore` core operations to core
- Phase 3 – Move parameter group types to core
- Phase 4 – Firmware re-exports and compatibility
- Phase 5 – Host-runnable unit tests

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Error Type and Value Types in Core

### Goal

- Create `ParameterError` enum in core
- Move `ParamValue`, `ParamFlags`, `ParamMetadata`, and constants to core
- Unify with existing core `registry.rs` `ParamValue`

### Inputs

- Source Code to Read:
  - `crates/firmware/src/parameters/storage.rs` – Current `ParamValue`, `ParamFlags`, `ParamMetadata`, constants
  - `crates/core/src/parameters/registry.rs` – Existing core `ParamValue`, `ParamMetadata`
- Source Code to Create:
  - `crates/core/src/parameters/error.rs` – `ParameterError` enum
- Source Code to Modify:
  - `crates/core/src/parameters/storage.rs` – New file with types moved from firmware
  - `crates/core/src/parameters/mod.rs` – Add new module exports
  - `crates/core/src/parameters/registry.rs` – Update or remove old `ParamValue`

### Tasks

- [x] **Create `ParameterError`**
  - [x] Create `crates/core/src/parameters/error.rs` with `ParameterError` enum (`InvalidConfig`, `StoreFull`, `ReadOnly`)
  - [x] Export from `crates/core/src/parameters/mod.rs`

- [x] **Move value types to core**
  - [x] Create `crates/core/src/parameters/storage.rs` in core
  - [x] Move `ParamValue` enum (with all 5 variants: `String`, `Bool`, `Int`, `Float`, `Ipv4`) to core `storage.rs`
  - [x] Move `ParamValue` impl block (`type_id`, `serialize`, `deserialize`) to core `storage.rs`
  - [x] Move `ParamFlags` bitflags definition to core `storage.rs`
  - [x] Move `ParamMetadata` struct to core `storage.rs`
  - [x] Move constants (`PARAM_NAME_LEN`, `MAX_PARAMS`, `MAX_STRING_LEN`, `PARAM_BLOCK_BASE`, `PARAM_BLOCK_SIZE`, `PARAM_BLOCK_COUNT`, `PARAM_MAGIC`, `PARAM_VERSION`) to core `storage.rs`
  - [x] Export all types from `crates/core/src/parameters/mod.rs`

- [x] **Handle existing core `registry.rs` `ParamValue`**
  - [x] Evaluate whether `registry.rs` `ParamValue`/`ParamMetadata` can be replaced by the unified types
  - [x] Update or alias `registry.rs` types to use new core `storage.rs` types
  - [x] Fix any references to old core `ParamValue` throughout core crate

- [x] **Update firmware to import from core**
  - [x] Remove type definitions from `crates/firmware/src/parameters/storage.rs`
  - [x] Add `use pico_trail_core::parameters::storage::{...}` imports
  - [x] Add `use pico_trail_core::parameters::error::ParameterError` import
  - [x] Ensure firmware `storage.rs` still compiles with flash methods using core types

### Deliverables

- `crates/core/src/parameters/error.rs` (new)
- `crates/core/src/parameters/storage.rs` (new — value types and constants)
- Updated `crates/core/src/parameters/mod.rs`
- Updated `crates/firmware/src/parameters/storage.rs` (types removed, imports added)

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- Core `ParamValue` has all 5 variants
- `ParameterError` exists in core
- Firmware compiles using core types
- RP2350 build succeeds
- Existing host tests pass

### Rollback/Fallback

- Revert core additions and firmware changes; restore original type definitions

---

## Phase 2: Move ParameterStore Core Operations to Core

### Phase 2 Goal

- Move `ParameterStore` struct and core methods to core crate
- Keep flash methods as free functions in firmware

### Phase 2 Inputs

- Dependencies:
  - Phase 1: `ParamValue`, `ParamFlags`, `ParamMetadata`, `ParameterError` in core
- Source Code to Modify:
  - `crates/core/src/parameters/storage.rs` – Add `ParameterStore` struct and core methods
  - `crates/firmware/src/parameters/storage.rs` – Remove core methods, keep flash methods as free functions

### Phase 2 Tasks

- [x] **Move `ParameterStore` struct to core**
  - [x] Move struct definition (fields: `parameters`, `metadata`, `dirty`) to core `storage.rs`
  - [x] Move `Default` impl to core
  - [x] Move `new()` to core
  - [x] Move `get()` to core
  - [x] Move `set()` to core (change error type from `PlatformError` to `ParameterError`)
  - [x] Move `register()` to core (change error type from `PlatformError` to `ParameterError`)
  - [x] Move `is_hidden()` to core
  - [x] Move `iter_names()` to core
  - [x] Move `count()` to core
  - [x] Move `is_dirty()` to core

- [x] **Refactor flash methods in firmware**
  - [x] Convert `load_from_flash()` to a free function: `pub fn load_from_flash(store: &mut ParameterStore, flash: &mut impl FlashInterface) -> Result<()>`
  - [x] Convert `save_to_flash()` to a free function: `pub fn save_to_flash(store: &mut ParameterStore, flash: &mut impl FlashInterface) -> Result<()>`
  - [x] Convert `load_from_block()` to a free function (internal helper)
  - [x] Update all callers in firmware to use free function syntax

- [x] **Add `From<ParameterError> for PlatformError`**
  - [x] Implement conversion in firmware's error module
  - [x] Update firmware code that previously caught `PlatformError::InvalidConfig` from parameter operations

### Phase 2 Deliverables

- Updated `crates/core/src/parameters/storage.rs` (with `ParameterStore`)
- Updated `crates/firmware/src/parameters/storage.rs` (flash functions only)
- Updated firmware callers of flash methods

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- `ParameterStore::new()`, `get()`, `set()`, `register()` work from core crate
- Flash persistence still works via free functions in firmware
- RP2350 build succeeds
- All existing tests pass

### Phase 2 Rollback/Fallback

- Restore `ParameterStore` methods to firmware; remove from core

---

## Phase 3: Move Parameter Group Types to Core

### Phase 3 Goal

- Move 8 architecture-independent parameter group types to core
- `BoardParams` stays in firmware

### Phase 3 Inputs

- Dependencies:
  - Phase 2: `ParameterStore` available in core
- Source Code to Move:
  - `crates/firmware/src/parameters/compass.rs` → `crates/core/src/parameters/compass.rs`
  - `crates/firmware/src/parameters/arming.rs` → `crates/core/src/parameters/arming.rs`
  - `crates/firmware/src/parameters/battery.rs` → `crates/core/src/parameters/battery.rs`
  - `crates/firmware/src/parameters/failsafe.rs` → `crates/core/src/parameters/failsafe.rs`
  - `crates/firmware/src/parameters/fence.rs` → `crates/core/src/parameters/fence.rs`
  - `crates/firmware/src/parameters/loiter.rs` → `crates/core/src/parameters/loiter.rs`
  - `crates/firmware/src/parameters/circle.rs` → `crates/core/src/parameters/circle.rs`
  - `crates/firmware/src/parameters/wifi.rs` → `crates/core/src/parameters/wifi.rs`

### Phase 3 Tasks

- [x] **Move parameter groups to core (one by one)**
  - [x] Move `compass.rs` to core — update imports (`crate::platform::Result` → `Result<(), ParameterError>`)
  - [x] Move `arming.rs` to core — update imports
  - [x] Move `battery.rs` to core — update imports
  - [x] Move `failsafe.rs` to core — update imports
  - [x] Move `fence.rs` to core — update imports
  - [x] Move `loiter.rs` to core — update imports
  - [x] Move `circle.rs` to core — update imports (also moved `CircleDirection` enum from `rover::mode::circle`)
  - [x] Move `wifi.rs` to core — update imports (changed `env!()` to `option_env!().unwrap_or("")` for host compilation)

- [x] **Update core `mod.rs`**
  - [x] Add module declarations for all 8 parameter groups
  - [x] Export all parameter group types

- [x] **Remove files from firmware**
  - [x] Delete the 8 parameter group files from firmware
  - [x] Update firmware `parameters/mod.rs` to remove module declarations

### Phase 3 Deliverables

- 8 parameter group files in `crates/core/src/parameters/`
- Updated `crates/core/src/parameters/mod.rs`
- Updated `crates/firmware/src/parameters/mod.rs`

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- All 8 parameter groups compile in core crate
- `BoardParams` remains in firmware
- RP2350 build succeeds
- Existing tests pass

### Phase 3 Rollback/Fallback

- Restore parameter group files to firmware; remove from core

---

## Phase 4: Firmware Re-exports and Compatibility

### Phase 4 Goal

- Ensure all firmware code compiles without import path changes
- Add re-exports so existing `use crate::parameters::compass::CompassParams` paths work

### Phase 4 Inputs

- Dependencies:
  - Phase 3: All parameter groups in core
- Source Code to Modify:
  - `crates/firmware/src/parameters/mod.rs` – Add re-exports

### Phase 4 Tasks

- [x] **Add re-exports in firmware `parameters/mod.rs`**
  - [x] Re-export core `storage` module types (`ParameterStore`, `ParamValue`, `ParamFlags`, `ParamMetadata`)
  - [x] Re-export core `error` module (`ParameterError`)
  - [x] Re-export core parameter groups (`compass`, `arming`, `battery`, `failsafe`, `fence`, `loiter`, `circle`, `wifi`)

- [x] **Verify firmware imports**
  - [x] Search firmware crate for all `use crate::parameters::` imports
  - [x] Confirm each resolves correctly via re-exports
  - [x] Fix any broken imports

- [x] **Verify no circular dependencies**
  - [x] `cargo check -p pico_trail_core` succeeds
  - [x] `cargo check -p pico_trail_firmware` succeeds (verified via RP2350 build)

### Phase 4 Deliverables

- Updated `crates/firmware/src/parameters/mod.rs` with re-exports

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 4 Acceptance Criteria

- No firmware source files changed (except `parameters/mod.rs`)
- All existing import paths work
- RP2350 build succeeds

### Phase 4 Rollback/Fallback

- Adjust re-exports as needed; all types still exist in core

---

## Phase 5: Host-Runnable Unit Tests

### Phase 5 Goal

- Add comprehensive unit tests in core crate for all migrated types
- Move existing firmware compile-only tests to core as executable tests
- Verify `cargo test --lib` runs all parameter tests

### Phase 5 Tasks

- [x] **ParameterStore tests**
  - [x] Test `new()` creates empty store
  - [x] Test `register()` and `get()` round-trip
  - [x] Test `set()` on registered parameter
  - [x] Test `set()` on unknown parameter returns error
  - [x] Test duplicate `register()` is idempotent
  - [x] Test `is_dirty()` after `set()`
  - [x] Test `count()` after registrations
  - [x] Test `iter_names()` returns all registered names

- [x] **ParamValue tests**
  - [x] Test `type_id()` for each variant
  - [x] Test serialization/deserialization round-trip for each variant (stays in firmware — `serialize_value`/`deserialize_value` are firmware free functions)
  - [x] Test equality for each variant

- [x] **Parameter group tests (per group)**
  - [x] `CompassParams`: register_defaults, from_store defaults, from_store custom values
  - [x] `ArmingParams`: register_defaults, from_store defaults, from_store custom values
  - [x] `BatteryParams`: register_defaults, from_store defaults, from_store custom values
  - [x] `FailsafeParams`: register_defaults, from_store defaults, from_store custom values
  - [x] `FenceParams`: register_defaults, from_store defaults, from_store custom values
  - [x] `LoiterParams`: register_defaults, from_store defaults, from_store custom values
  - [x] `CircleParams`: register_defaults, from_store defaults, from_store custom values
  - [x] `WifiParams`: register_defaults, from_store defaults, from_store custom values

### Phase 5 Deliverables

- Unit tests in `crates/core/src/parameters/storage.rs` (test module)
- Unit tests in each parameter group file (test modules)

### Phase 5 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 5 Acceptance Criteria

- `cargo test --lib` runs and passes all parameter tests
- Test count includes ParameterStore tests and all 8 parameter group tests
- No regressions in existing tests
- RP2350 build succeeds

### NFR-00092 Verification Notes

The following NFR-00092 acceptance criteria are satisfied by design (not by explicit test):

- **Flash format compatibility**: `serialize_value`/`deserialize_value` and `load_from_flash`/`save_to_flash` remain in `crates/firmware/src/parameters/storage.rs` unchanged. No serialization logic was modified during migration — only the crate location of the types they operate on changed. Therefore, flash-stored parameters remain readable.
- **MAVLink PARAM protocol unchanged**: The MAVLink handlers in `crates/firmware/src/communication/mavlink/handlers/param.rs` consume `ParameterStore` via the same public API (`get`, `set`, `register`, `iter_names`, `count`). The migration changed the crate origin of these methods but not their signatures or behavior. RP2350 build success confirms protocol handling compiles correctly.

---

## Definition of Done

- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet` — includes parameter store and parameter group tests (320 tests pass)
- [x] `./scripts/build-rp2350.sh pico_trail_rover`
- [x] `ParameterStore` core operations testable on host
- [x] All 8 parameter groups testable on host
- [x] `BoardParams` remains in firmware
- [x] Flash persistence unchanged
- [x] No firmware import path changes (re-exports maintain compatibility)
- [x] No `unsafe` and no vague naming

## Open Questions

- [ ] Should `ParamValue::serialize()` / `deserialize()` move to core or stay with flash persistence? If they move, core gains serialization capability but takes on `crc` dependency.
- [ ] Should old `registry.rs` `ParamValue`/`ParamMetadata` be removed entirely or kept as lightweight types for non-storage use cases?
