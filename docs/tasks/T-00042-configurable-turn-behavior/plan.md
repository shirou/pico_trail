# T-00042 Configurable Turn Behavior

## Metadata

- Type: Implementation Plan
- Status: Implementation Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Add `pivot_turn_angle` and `arc_turn_min_throttle` to `SimpleNavConfig`, modify `calculate_throttle()` to enforce a throttle floor during arc turns, and register all navigation parameters in the parameter store as `NavigationParams`.

## Success Metrics

- [x] `pivot_turn_angle` config field with default 60.0°
- [x] `arc_turn_min_throttle` config field with default 0.15
- [x] Throttle floor enforced when heading error < pivot angle and not at target
- [x] Transition continuous at boundary (< 0.1 delta)
- [x] `pivot_turn_angle=0` preserves current behavior
- [x] All existing tests pass; no regressions
- [x] Embedded build compiles
- [x] All 12 `SimpleNavConfig` fields registered in parameter store
- [x] Parameters configurable at runtime via MAVLink GCS

## Scope

- Goal: Configurable pivot turn threshold with arc turn throttle floor; navigation parameter store registration
- Non-Goals: WP_PIVOT_RATE, DifferentialDrive changes
- Assumptions: All autonomous modes share `SimpleNavigationController`
- Constraints: no_std, f32, < 1ms per update on RP2350

## ADR & Legacy Alignment

- [x] Confirm ADR-00022-navigation-controller-architecture is referenced
- [x] Existing `calculate_throttle()` signature unchanged (internal logic only)
- [x] No changes to `NavigationController` trait interface

## Plan Summary

- Phase 1 -- Add config fields, implement throttle floor, unit tests, verify build **(complete)**
- Phase 2 -- Create `NavigationParams` parameter group, register in store, unit tests, verify build **(complete)**

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Config Fields + Throttle Floor + Tests (Complete)

### Goal

- Add `pivot_turn_angle` and `arc_turn_min_throttle` to `SimpleNavConfig`
- Modify `calculate_throttle()` to apply throttle floor
- Add unit tests for all acceptance criteria
- Verify embedded build

### Inputs

- Source Code to Modify:
  - `crates/core/src/navigation/types.rs` -- Add config fields and defaults
  - `crates/core/src/navigation/controller.rs` -- Modify `calculate_throttle()`, add tests

### Tasks

- [x] **Add config fields to `SimpleNavConfig`**
  - [x] Add `pivot_turn_angle: f32` field with doc comment
  - [x] Add `arc_turn_min_throttle: f32` field with doc comment
  - [x] Update `Default` impl: `pivot_turn_angle: 60.0`, `arc_turn_min_throttle: 0.15`
- [x] **Modify `calculate_throttle()`**
  - [x] After existing throttle calculation, add floor condition
  - [x] `if heading_error_abs < pivot_turn_angle && distance > wp_radius`
  - [x] `throttle = throttle.max(arc_turn_min_throttle)`
- [x] **Update existing test helpers** (if `SimpleNavConfig` construction in tests needs new fields)
  - [x] Update any `SimpleNavConfig { .. }` literals in tests to include new fields
- [x] **Unit tests: pivot turn threshold (FR-00144)**
  - [x] Test: heading error below pivot angle → throttle >= arc_turn_min_throttle
  - [x] Test: heading error above pivot angle → throttle follows existing curve
  - [x] Test: `pivot_turn_angle=0` → floor never applies (current behavior)
  - [x] Test: `pivot_turn_angle=180` → floor always applies when not at target
- [x] **Unit tests: arc turn minimum throttle (FR-00145)**
  - [x] Test: floor applies when heading error < pivot angle AND distance > wp_radius
  - [x] Test: floor does NOT apply at target (distance <= wp_radius)
  - [x] Test: floor uses `max()` — does not reduce natural throttle when it is higher
- [x] **Unit tests: transition continuity (NFR-00093)**
  - [x] Test: throttle at 59.9° vs 60.1° with default config → delta < 0.1
  - [x] Test: throttle at boundary with non-default config → delta < 0.1
- [x] **Verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `cargo test --lib --quiet`
  - [x] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- Modified `crates/core/src/navigation/types.rs`
- Modified `crates/core/src/navigation/controller.rs`

### Acceptance Criteria (Phase Gate)

- All FR-00144 acceptance criteria met (threshold behavior, defaults, edge cases)
- All FR-00145 acceptance criteria met (floor activation, target guard, spin-cap interaction)
- NFR-00093 continuity met (delta < 0.1 at boundary)
- All existing navigation tests pass unchanged
- Embedded build succeeds

### Rollback/Fallback

- `pivot_turn_angle = 0.0` disables the feature entirely (current behavior)
- `arc_turn_min_throttle = 0.0` makes the floor a no-op

---

## Phase 2: Navigation Parameter Store Registration

### Goal

- Create `NavigationParams` parameter group in `crates/core/src/parameters/navigation.rs`
- Register all 12 `SimpleNavConfig` fields in the parameter store
- Add `to_nav_config()` conversion and `is_valid()` validation
- Register in `ParamHandler::new()` for MAVLink GCS access
- Unit tests for registration, loading, clamping, conversion, and validation

### Inputs

- Source Code to Create:
  - `crates/core/src/parameters/navigation.rs` -- New parameter group
- Source Code to Modify:
  - `crates/core/src/parameters/mod.rs` -- Add `navigation` module and re-export
  - `crates/firmware/src/communication/mavlink/handlers/param.rs` -- Register `NavigationParams`

### Tasks

- [x] **Create `NavigationParams` struct**
  - [x] Define struct with fields matching all 12 `SimpleNavConfig` fields
  - [x] Add constants for defaults, min/max ranges
  - [x] Implement `Default`
- [x] **Implement `register_defaults()`**
  - [x] Register all 12 parameters with ArduPilot-compatible names
  - [x] Use `ParamValue::Float` for all parameters
- [x] **Implement `from_store()`**
  - [x] Load each parameter with range clamping
  - [x] Handle missing values with defaults
  - [x] Handle both `Float` and `Int` variants (MAVLink PARAM_SET sends floats)
- [x] **Implement `to_nav_config()`**
  - [x] Convert `NavigationParams` → `SimpleNavConfig`
- [x] **Implement `is_valid()`**
  - [x] Validate range constraints
  - [x] Validate consistency (e.g., `approach_dist > wp_radius`)
- [x] **Register in `parameters/mod.rs`**
  - [x] Add `pub mod navigation;`
  - [x] Re-export `NavigationParams`
- [x] **Register in `ParamHandler::new()`**
  - [x] Add `NavigationParams::register_defaults(&mut store)` call
- [x] **Unit tests (FR-00146)**
  - [x] Test: `register_defaults()` populates all 12 parameters
  - [x] Test: `from_store()` reads default values correctly
  - [x] Test: `from_store()` reads custom values correctly
  - [x] Test: Out-of-range values are clamped
  - [x] Test: `to_nav_config()` produces matching `SimpleNavConfig`
  - [x] Test: `is_valid()` accepts valid parameters
  - [x] Test: `is_valid()` rejects invalid parameters
- [x] **Verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `cargo test --lib --quiet`
  - [x] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- New `crates/core/src/parameters/navigation.rs`
- Modified `crates/core/src/parameters/mod.rs`
- Modified `crates/firmware/src/communication/mavlink/handlers/param.rs`

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- All FR-00146 acceptance criteria met (12 parameters registered, configurable via GCS)
- FR-00144 parameter store criteria met (WP_PIVOT_ANGLE in store)
- FR-00145 parameter store criteria met (WP_ARC_THR in store)
- All existing tests pass unchanged
- Embedded build succeeds

### Rollback/Fallback

- If parameter store fails to load, `SimpleNavConfig::default()` provides safe fallback
- Individual parameters fall back to defaults if missing from store

---

## Definition of Done

- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh pico_trail_rover` builds successfully
- [x] No `unsafe` code
- [x] No vague naming ("manager"/"util")
- [x] Plan checkboxes marked
- [x] Task README status updated to Implementation Complete
- [x] Traceability check: `bun scripts/trace-status.ts --check`
