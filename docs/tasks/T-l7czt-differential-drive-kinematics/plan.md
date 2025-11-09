# T-l7czt Differential Drive Kinematics

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement pure, platform-independent differential drive kinematics conversion from steering/throttle inputs to left/right motor speeds with normalization. This task creates a standalone kinematics library module with no dependencies on platform code, hardware abstractions, or system state, enabling unit testing on host machines while supporting embedded targets.

## Success Metrics

- [ ] All unit tests pass on host machine (cargo test --lib kinematics)
- [ ] All unit tests match ArduPilot reference outputs
- [ ] Property-based tests confirm outputs in \[-1.0, +1.0] range
- [ ] Module compiles for embedded target (cargo check --target thumbv8m.main-none-eabihf)
- [ ] Zero dependencies on platform modules (cargo tree shows no platform imports)
- [ ] No heap allocations (verified with #!\[no_std])
- [ ] All existing tests pass; no regressions in other modules

## Scope

- Goal: Create standalone kinematics library with differential drive mixing algorithm
- Non-Goals:
  - Motor driver integration (separate task)
  - Control mode integration (separate task)
  - Other kinematics models (Ackermann, mecanum) - future work
  - Advanced features (per-motor trim, feedback control) - future work
- Assumptions:
  - f32 arithmetic available (ARM Cortex-M33+ has hardware FPU)
  - Unit tests run on host machines with std available
  - ArduPilot mixing algorithm is the reference implementation
- Constraints:
  - no_std embedded environment (RP2350)
  - Zero dependencies on platform/hardware code
  - Pure functions (no mutable state, no I/O)

## ADR & Legacy Alignment

- [x] Confirm ADR-2l5fh-differential-drive-kinematics is referenced above
- [x] No legacy kinematics system exists (new feature)
- [ ] Motor driver integration will be handled in separate task (FR-higbc)

## Plan Summary

- Phase 1 – Core Kinematics Module (foundation scaffolding)
- Phase 2 – Unit Tests and Validation (test implementation)
- Phase 3 – Platform Verification (embedded build verification)

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: Core Kinematics Module

### Goal

- Create kinematics library module with differential drive mixing implementation

### Inputs

- Documentation:
  - `docs/adr/ADR-2l5fh-differential-drive-kinematics.md` – Architecture decision
  - `docs/requirements/FR-41nab-differential-drive-kinematics.md` – Functional requirements
  - `docs/analysis/AN-qlnt3-freenove-hardware-support.md` – Hardware context
- Source Code to Modify:
  - N/A (new files)
- Dependencies:
  - Internal: None (foundation layer)
  - External crates: None (core library only)

### Tasks

- [x] **Create kinematics library module**
  - [x] Create `src/libraries/kinematics/` directory
  - [x] Create `src/libraries/kinematics/mod.rs` (no_std attribute removed per clippy warning)
  - [x] Add `pub mod kinematics;` to `src/libraries/mod.rs`
  - [x] Verify module compiles: `cargo check`

- [x] **Implement differential drive mixing**
  - [x] Create `src/libraries/kinematics/differential_drive.rs`
  - [x] Define `DifferentialDrive` struct (zero-sized marker)
  - [x] Implement `mix(steering: f32, throttle: f32) -> (f32, f32)` method
  - [x] Add normalization logic (max magnitude > 1.0 → divide both by max)
  - [x] Add rustdoc comments with examples
  - [x] Re-export from `mod.rs`: `pub use differential_drive::DifferentialDrive;`

- [x] **Verify no platform dependencies**
  - [x] Run `cargo tree --edges normal | grep kinematics` to check dependencies
  - [x] Confirm no imports from `src/platform/`, `src/core/`, or `src/communication/`
  - [x] Confirm only `core` library used (no `std`, no `alloc`)

### Deliverables

- `src/libraries/kinematics/mod.rs` with no_std attribute
- `src/libraries/kinematics/differential_drive.rs` with mixing implementation
- Compiles successfully on host with `cargo check`

### Verification

```bash
# Build and checks
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings

# Verify no dependencies on platform code
cargo tree --edges normal | grep kinematics
```

### Acceptance Criteria (Phase Gate)

- Module structure created and compiles without errors
- `DifferentialDrive::mix()` implementation complete with rustdoc
- No dependencies on platform/hardware/system state code
- Code formatted and clippy clean

### Rollback/Fallback

- Delete `src/libraries/kinematics/` directory if implementation fails
- Defer to manual mixing in control modes if kinematics approach proves infeasible

---

## Phase 2: Unit Tests and Validation

### Phase 2 Goal

- Create comprehensive unit tests matching ArduPilot reference implementation

### Phase 2 Inputs

- Dependencies:
  - Phase 1: `DifferentialDrive::mix()` implementation
  - ArduPilot reference: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsUGV.cpp>
- Source Code to Modify:
  - `src/libraries/kinematics/differential_drive.rs` (add tests module)

### Phase 2 Tasks

- [x] **Add basic unit tests**
  - [x] Create `#[cfg(test)] mod tests` in `differential_drive.rs`
  - [x] Test: Straight forward (steering=0.0, throttle=0.5 → left=0.5, right=0.5)
  - [x] Test: Straight reverse (steering=0.0, throttle=-0.5 → left=-0.5, right=-0.5)
  - [x] Test: Spin right in place (steering=1.0, throttle=0.0 → left=1.0, right=-1.0)
  - [x] Test: Spin left in place (steering=-1.0, throttle=0.0 → left=-1.0, right=1.0)
  - [x] Test: Zero inputs (steering=0.0, throttle=0.0 → left=0.0, right=0.0)

- [x] **Add normalization tests**
  - [x] Test: Forward right turn normalized (steering=0.5, throttle=0.5 → left=1.0, right=0.0)
  - [x] Test: Forward left turn normalized (steering=-0.3, throttle=0.8 → left≈0.5, right≈1.0)
  - [x] Test: Full throttle + full steering (steering=1.0, throttle=1.0 → left=1.0, right=0.0)
  - [x] Test: Full reverse + full steering (steering=1.0, throttle=-1.0 → left=0.0, right=-1.0)

- [x] **Add property-based tests**
  - [x] Test: Output range property (all combinations produce outputs in \[-1.0, +1.0])
  - [x] Test: Symmetry property (steering=-x produces opposite results to steering=+x)
  - [x] Test: Zero throttle property (throttle=0.0 produces opposite left/right speeds)

- [x] **Cross-reference ArduPilot outputs**
  - [x] All test cases match ArduPilot SkidSteer mixing behavior
  - [x] All test cases from FR-41nab acceptance criteria pass (12 tests total)

### Phase 2 Deliverables

- Comprehensive unit test suite with 10+ test cases
- All tests pass on host machine
- Test outputs match ArduPilot reference implementation

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings

# Run kinematics tests specifically
cargo test --lib kinematics --quiet

# Run all tests to check for regressions
cargo test --lib --quiet
```

### Phase 2 Acceptance Criteria

- All unit tests pass with 100% success rate
- Test outputs match ArduPilot reference values
- Property-based tests confirm output range guarantees
- No test failures or panics

### Phase 2 Rollback/Fallback

- If tests fail to match ArduPilot, review mixing algorithm and normalization logic
- If property tests fail, investigate edge cases and adjust normalization

---

## Phase 3: Platform Verification

### Phase 3 Goal

- Verify module compiles for embedded target and has no platform dependencies

### Phase 3 Tasks

- [x] **Embedded build verification**
  - [x] Compile for RP2350 target: `cargo check --target thumbv8m.main-none-eabihf --lib`
  - [x] Verify no_std compatibility (no build errors related to std library)

- [x] **Dependency verification**
  - [x] Run `cargo tree --edges normal | grep kinematics`
  - [x] Confirm only `core` library dependency (no external dependencies found)
  - [x] Confirm no platform-specific imports in source code

- [x] **Documentation update**
  - [x] Add inline documentation examples to rustdoc (included in implementation)
  - [x] Usage examples included in module and function rustdoc comments

### Phase 3 Deliverables

- Kinematics module compiles for embedded target
- Documentation complete with usage examples
- Dependency tree verified (core library only)

### Phase 3 Verification

```bash
# Embedded build check
cargo check --target thumbv8m.main-none-eabihf --lib

# Verify dependencies
cargo tree --target thumbv8m.main-none-eabihf --edges normal | grep kinematics

# Check documentation builds
cargo doc --lib --no-deps --open

# Format and lint
cargo fmt
cargo clippy --all-targets -- -D warnings

# Final test run
cargo test --lib --quiet
```

### Phase 3 Acceptance Criteria

- Embedded build succeeds without errors or warnings
- Dependency tree shows only `core` library
- Documentation renders correctly with examples
- All tests still pass

---

## Definition of Done

- [x] `cargo check` (host)
- [x] `cargo check --target thumbv8m.main-none-eabihf --lib` (embedded)
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet` (all tests pass - 12 kinematics tests)
- [x] Rustdoc comments with examples added
- [x] No dependencies on platform/hardware/system state code
- [x] Property-based tests confirm output range guarantees
- [x] Test outputs match ArduPilot reference implementation
- [x] No heap allocations (pure functions, no_std compatible)
- [x] Module exports added to `src/libraries/mod.rs`

## Open Questions

- [ ] Should we add support for configurable mixing ratios in the future?
  - Next step: Defer to future enhancement (create issue if users request)
- [ ] Should we benchmark performance on RP2350 hardware?
  - Next step: Only if performance issues reported (unlikely for pure math)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
