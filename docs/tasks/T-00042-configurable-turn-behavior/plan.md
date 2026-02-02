# T-00042 Configurable Turn Behavior

## Metadata

- Type: Implementation Plan
- Status: Implementation Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Add `pivot_turn_angle` and `arc_turn_min_throttle` to `SimpleNavConfig`, and modify `calculate_throttle()` to enforce a throttle floor during arc turns. Single phase — the change is small and self-contained.

## Success Metrics

- [x] `pivot_turn_angle` config field with default 60.0°
- [x] `arc_turn_min_throttle` config field with default 0.15
- [x] Throttle floor enforced when heading error < pivot angle and not at target
- [x] Transition continuous at boundary (< 0.1 delta)
- [x] `pivot_turn_angle=0` preserves current behavior
- [x] All existing tests pass; no regressions
- [x] Embedded build compiles

## Scope

- Goal: Configurable pivot turn threshold with arc turn throttle floor
- Non-Goals: WP_PIVOT_RATE, DifferentialDrive changes, MAVLink parameter exposure
- Assumptions: All autonomous modes share `SimpleNavigationController`
- Constraints: no_std, f32, < 1ms per update on RP2350

## ADR & Legacy Alignment

- [x] Confirm ADR-00022-navigation-controller-architecture is referenced
- [x] Existing `calculate_throttle()` signature unchanged (internal logic only)
- [x] No changes to `NavigationController` trait interface

## Plan Summary

- Phase 1 -- Add config fields, implement throttle floor, unit tests, verify build

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Config Fields + Throttle Floor + Tests

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

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

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
