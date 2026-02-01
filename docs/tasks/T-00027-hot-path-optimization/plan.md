# T-00027 Hot Path Optimization

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [T-00027-hot-path-optimization-design](design.md)

## Overview

Implement three hot path optimizations: replace custom clamp with standard library, define constant Z-unit vector for DCM, and gate determinant check to debug builds only.

## Success Metrics

- [ ] All existing tests pass without modification
- [ ] `cargo clippy` reports no warnings
- [ ] Embedded build succeeds
- [ ] No behavioral changes

## Scope

- Goal: Reduce unnecessary CPU operations in 100Hz control loops
- Non-Goals: Algorithm changes, benchmark infrastructure
- Assumptions: nalgebra supports const Vector3 construction
- Constraints: Must maintain identical behavior

## ADR & Legacy Alignment

- [x] No ADRs required for this task
- [x] No legacy patterns to migrate

## Plan Summary

- Phase 1 – Navigation controller clamp optimization
- Phase 2 – DCM constant vector and debug-only determinant

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Navigation Controller Optimization

### Goal

- Replace custom `clamp` function with `f32::clamp()` standard library method

### Inputs

- Source Code to Modify:
  - `src/subsystems/navigation/controller.rs` – Remove custom clamp, update usages

### Tasks

- [x] **Remove custom clamp function**
  - [x] Delete `fn clamp(value: f32, min: f32, max: f32) -> f32` function
  - [x] Update `sanitize_output()` to use `value.clamp(min, max)`
  - [x] Search for any other `clamp()` calls and update to method syntax

- [x] **Verify changes**
  - [x] Run `cargo fmt`
  - [x] Run `cargo clippy --all-targets -- -D warnings`
  - [x] Run `cargo test --lib --quiet` - all navigation tests pass

### Deliverables

- `controller.rs` using standard library clamp

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet navigation
```

### Acceptance Criteria (Phase Gate)

- Custom `clamp` function removed
- All tests pass
- No clippy warnings

### Rollback/Fallback

- Revert changes with `git checkout src/subsystems/navigation/controller.rs`

---

## Phase 2: DCM Module Optimization

### Phase 2 Goal

- Define constant Z-unit vector
- Gate determinant check to debug builds only

### Phase 2 Inputs

- Dependencies:
  - Phase 1 complete (independent but sequential for review clarity)
- Source Code to Modify:
  - `src/subsystems/ahrs/dcm.rs` – Add const, update orthonormalize

### Phase 2 Tasks

- [x] **Add Z-unit vector constant**
  - [x] Define `const Z_UNIT: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);` at module level
  - [x] Update `Dcm::update()` to use `Z_UNIT` instead of inline `Vector3::new(...)`
  - [x] Add doc comment explaining the constant's purpose

- [x] **Gate determinant check**
  - [x] Wrap determinant calculation block with `#[cfg(debug_assertions)]`
  - [x] Verify debug builds still log drift warnings
  - [x] Verify release builds skip the calculation

- [x] **Verify changes**
  - [x] Run `cargo fmt`
  - [x] Run `cargo clippy --all-targets -- -D warnings`
  - [x] Run `cargo test --lib --quiet ahrs` - all DCM tests pass
  - [x] Run `./scripts/build-rp2350.sh pico_trail_rover` - embedded build succeeds

### Phase 2 Deliverables

- `dcm.rs` with const vector and debug-gated determinant

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- `Z_UNIT` constant defined and used
- Determinant check gated with `#[cfg(debug_assertions)]`
- All tests pass
- Embedded build succeeds

### Phase 2 Rollback/Fallback

- Revert changes with `git checkout src/subsystems/ahrs/dcm.rs`

---

## Definition of Done

- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover` - embedded build succeeds
- [ ] Requirements documents updated with task links
- [ ] Analysis document updated with task link

## Open Questions

None - implementation is straightforward.
