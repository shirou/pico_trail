# T-00171 Hold Mode Implementation

## Metadata

- Type: Implementation Plan
- Status: Completed

## Links

- Associated Design Document:
  - [T-00171-hold-mode-design](design.md)

## Overview

Implement `HoldMode` struct in `crates/core/src/mode/hold.rs` with `Mode` trait implementation. Single-phase task due to minimal scope.

## Success Metrics

- [x] `HoldMode` passes all unit tests
- [x] `cargo test -p pico_trail_core --lib --quiet` passes
- [x] `./scripts/build-rp2350.sh pico_trail_rover` succeeds
- [x] `cargo clippy --all-targets -- -D warnings` passes
- [x] All existing tests pass; no regressions in mode system

## Scope

- Goal: Working `HoldMode` struct with Mode trait, unit tests, and passing builds
- Non-Goals: Mode switching integration, SITL changes, parameter additions
- Assumptions: `MockActuator` pattern exists in `manual.rs`/`auto.rs` test modules (define locally in `hold.rs` tests)
- Constraints: Must compile on both host (tests) and RP2350 (embedded)

## ADR & Legacy Alignment

- [x] No ADRs govern this work (confirmed in analysis AN-00167)
- [x] No legacy patterns to retire (new file only)

## Plan Summary

- Phase 1 – HoldMode implementation, module registration, and unit tests

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

> **⚠ Phase Gate Rule**: After completing each phase, you MUST stop and request explicit user approval before proceeding to the next phase. Do not continue automatically.

---

## Phase 1: HoldMode Implementation and Tests

### Goal

- Create `HoldMode` struct, implement `Mode` trait, register module, add unit tests, verify all builds pass

### Inputs

- Documentation:
  - `docs/tasks/T-00171-hold-mode/design.md` – Struct design and test plan
  - `docs/analysis/AN-00167-hold-mode.md` – Analysis and raw data (implementation sketch)
- Source Code to Modify:
  - `crates/core/src/mode/mod.rs` – Add `pub mod hold;` and re-export
- Source Code to Create:
  - `crates/core/src/mode/hold.rs` – HoldMode struct and Mode impl
- Dependencies:
  - Internal: `crates/core/src/mode/traits.rs` – `Mode` trait
  - Internal: `crates/core/src/servo/mod.rs` – `ActuatorInterface` trait
  - Reference: `crates/core/src/mode/manual.rs` tests – `MockActuator` pattern to replicate

### Tasks

- [x] **Create HoldMode struct**
  - [x] Create `crates/core/src/mode/hold.rs`
  - [x] Define `HoldMode` struct with `actuators: Box<dyn ActuatorInterface>` field
  - [x] Implement `HoldMode::new()` constructor
  - [x] Implement `Mode` trait: `enter()`, `update()`, `exit()`, `name()`
- [x] **Register module**
  - [x] Add `pub mod hold;` to `crates/core/src/mode/mod.rs`
  - [x] Add `pub use hold::HoldMode;` re-export
- [x] **Add unit tests**
  - [x] `test_hold_mode_enter_exit` — enter/exit succeed, actuators zeroed
  - [x] `test_hold_mode_update_zeros_actuators` — update zeros non-zero actuators
  - [x] `test_hold_mode_name` — name() returns "Hold"
  - [x] `test_hold_mode_size` — size_of::\<HoldMode>() <= 32
- [x] **Verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `cargo test -p pico_trail_core --lib --quiet`
  - [x] `./scripts/build-rp2350.sh pico_trail_rover`
- [x] **Documentation updates**
  - [x] Update AN-00167 status
  - [x] Update FR-00168, FR-00169, NFR-00170 Related Tasks links
  - [x] Update this plan with checkboxes
  - [x] Run `bun scripts/trace-status.ts --check`, `bun format`, `bun lint`

### Deliverables

- `crates/core/src/mode/hold.rs` — HoldMode struct + Mode impl + unit tests
- Updated `crates/core/src/mode/mod.rs` — module registration

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test -p pico_trail_core --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
bun scripts/trace-status.ts --check
bun format
bun lint
```

### Acceptance Criteria (Phase Gate)

- All 4 unit tests pass
- `cargo clippy` reports no warnings
- RP2350 embedded build succeeds
- Traceability check passes

### Rollback/Fallback

- Delete `crates/core/src/mode/hold.rs` and revert `mod.rs` changes. No other files are affected by the implementation itself.

---

## Definition of Done

- [x] Build succeeds with no errors
- [x] Code formatted per project standards (`cargo fmt`)
- [x] Linting passes with no warnings (`cargo clippy --all-targets -- -D warnings`)
- [x] Unit tests pass (`cargo test -p pico_trail_core --lib --quiet`)
- [x] Embedded build verified (`./scripts/build-rp2350.sh pico_trail_rover`)
- [x] Documentation traceability verified (`bun scripts/trace-status.ts --check`)
- [x] Plan checkboxes updated

## Open Questions

None – all questions resolved during analysis phase.
