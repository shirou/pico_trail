# T-d9rim Trait-Based Async Abstraction Plan

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [T-d9rim-design](./design.md)

## Overview

Implement trait-based abstractions for async runtime operations to reduce feature gates from \~150 to ≤60, enabling host testing without embedded dependencies while maintaining embedded functionality.

## Success Metrics

- [ ] Feature gate count ≤60 (measured by `grep -r "#\[cfg(feature" src/ | wc -l`)
- [ ] `cargo test --lib` passes without feature flags
- [ ] `./scripts/build-rp2350.sh pico_trail_rover` succeeds
- [ ] Control loop maintains 50Hz (≤20ms average period)
- [ ] All existing tests pass; no regressions

## Scope

- Goal: Decouple core logic from Embassy async runtime via traits
- Non-Goals: Workspace split, ESP32 support, async trait methods
- Assumptions: Embassy remains the target embedded runtime
- Constraints: Must maintain backward compatibility with examples

## ADR & Legacy Alignment

- [ ] Confirm ADR-3ciu6 is referenced and followed
- [ ] Note: Current code uses direct Embassy types; migration to traits required

## Plan Summary

- Phase 1 – Core Traits: Define `TimeSource` and `SharedState<T>` traits with implementations
- Phase 2 – Module Migration: Migrate high-impact modules to use traits
- Phase 3 – Cleanup: Remove duplicate implementations, reclassify feature gates
- Phase 4 – Verification: Comprehensive testing and CI enforcement

---

## Phase 1: Core Traits

### Goal

- Define `TimeSource` and `SharedState<T>` traits in `src/core/traits/`
- Implement Embassy and Mock versions
- Establish foundation for module migration

### Inputs

- Documentation:
  - `docs/adr/ADR-3ciu6-trait-based-async-abstraction.md` – Architecture decision
  - `docs/tasks/T-d9rim-trait-based-async-abstraction/design.md` – Detailed design
- Source Code to Modify:
  - `src/core/mod.rs` – Add `traits` module export
  - `src/lib.rs` – Export traits for external use
- Dependencies:
  - Internal: `embassy_time`, `embassy_sync` (feature-gated)
  - External crates: None new

### Tasks

- [ ] **Create traits module structure**
  - [ ] Create `src/core/traits/mod.rs`
  - [ ] Create `src/core/traits/time.rs`
  - [ ] Create `src/core/traits/sync.rs`
  - [ ] Export from `src/core/mod.rs`

- [ ] **Implement TimeSource trait**
  - [ ] Define `TimeSource` trait with `now_ms()`, `now_us()`, `elapsed_since()`
  - [ ] Implement `EmbassyTime` with `#[cfg(feature = "embassy")]`
  - [ ] Implement `MockTime` (always available)
  - [ ] Add unit tests for `MockTime`

- [ ] **Implement SharedState trait**
  - [ ] Define `SharedState<T>` trait with `with()`, `with_mut()`
  - [ ] Implement `EmbassyState<T>` with `#[cfg(feature = "embassy")]`
  - [ ] Implement `MockState<T>` (always available)
  - [ ] Add unit tests for `MockState<T>`

### Deliverables

- `src/core/traits/` module with complete trait definitions
- Unit tests for mock implementations
- Documentation comments on all public items

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet core::traits
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- Traits compile on both host and embedded targets
- Mock implementations have 100% test coverage
- Embassy implementations compile with `pico2_w` feature

### Rollback/Fallback

- Traits are additive; can remove without affecting existing code

---

## Phase 2: Module Migration

### Phase 2 Goal

- Migrate high-impact modules to use trait abstractions
- Start with `rc_channel` as POC, then `mission/state.rs`

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Core traits defined and tested
- Source Code to Modify:
  - `src/libraries/rc_channel/mod.rs` – RC input processing (3 gates)
  - `src/core/mission/state.rs` – Mission state management (18 gates)
  - `src/core/log_router.rs` – Log routing (10 gates)

### Phase 2 Tasks

- [ ] **Migrate rc_channel module (POC)**
  - [ ] Replace direct `Mutex` usage with `SharedState<RcInput>`
  - [ ] Update `RC_INPUT` static to use `EmbassyState`
  - [ ] Add generic type parameters where needed
  - [ ] Verify existing tests pass
  - [ ] Verify embedded build works

- [ ] **Migrate mission/state.rs**
  - [ ] Replace `MISSION_STATE` with trait-based state
  - [ ] Replace `MISSION_STORAGE` with trait-based state
  - [ ] Remove duplicate `_sync` function variants
  - [ ] Update callers in `mavlink/handlers/`

- [ ] **Migrate log_router.rs**
  - [ ] Replace `LOG_ROUTER` static with trait-based state
  - [ ] Remove feature-gated function duplicates

- [ ] **Migrate navigation module**
  - [ ] Replace `NAV_TARGET`, `NAV_OUTPUT` with trait-based state
  - [ ] Update `REPOSITION_TARGET` handling

### Phase 2 Deliverables

- Migrated modules using trait abstractions
- Reduced feature gate count in migrated modules
- Updated caller code

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- Each migrated module has fewer feature gates than before
- All existing functionality preserved
- No performance regression in control loop

### Phase 2 Rollback/Fallback

- Revert individual module changes if issues found
- Keep old implementation behind feature flag during transition

---

## Phase 3: Cleanup and Reclassification

### Phase 3 Goal

- Remove duplicate implementations throughout codebase
- Reclassify `pico2_w` gates to `embassy` where appropriate
- Enforce `pico2_w` only in `src/platform/`

### Phase 3 Tasks

- [ ] **Remove duplicate function implementations**
  - [ ] `mavlink/handlers/rc_input.rs` – Remove stub handlers
  - [ ] `mavlink/handlers/navigation.rs` – Remove stub handlers
  - [ ] `mavlink/handlers/command.rs` – Consolidate implementations
  - [ ] `mavlink/transport/udp.rs` – Unify transport implementations
  - [ ] `rover/mode/manual.rs` – Remove duplicate constructors

- [ ] **Reclassify feature gates**
  - [ ] Change `#[cfg(feature = "pico2_w")]` to `#[cfg(feature = "embassy")]` for async code
  - [ ] Move `#[embassy_executor::task]` to `src/platform/` or use `embassy-executor` feature
  - [ ] Ensure `pico2_w` only appears in `src/platform/rp2350/`

- [ ] **Move HAL-dependent code to platform**
  - [ ] Review `src/devices/imu/` for HAL leakage
  - [ ] Review `src/core/parameters/saver.rs` for platform-specific code

### Phase 3 Deliverables

- Zero `pico2_w` gates outside `src/platform/`
- Consolidated implementations (no duplicates)
- Feature gate count approaching target

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover

# Verify platform isolation
grep -r '#\[cfg(feature = "pico2_w"' src/ | grep -v "src/platform/" | wc -l
# Expected: 0
```

### Phase 3 Acceptance Criteria

- No `pico2_w` gates outside `src/platform/`
- No duplicate implementations remain
- Feature gate count ≤80 (intermediate target)

---

## Phase 4: Verification and CI Enforcement

### Phase 4 Goal

- Achieve final feature gate target (≤60)
- Add CI enforcement scripts
- Complete documentation updates

### Phase 4 Tasks

- [ ] **Final cleanup**
  - [ ] Review all remaining feature gates
  - [ ] Remove any unnecessary gates
  - [ ] Consolidate related gates where possible

- [ ] **CI enforcement**
  - [ ] Create `scripts/check-feature-gates.sh`
  - [ ] Add to pre-commit checks or CI pipeline
  - [ ] Document enforcement in CLAUDE.md

- [ ] **Documentation**
  - [ ] Update `docs/architecture.md` with trait layer
  - [ ] Update `CLAUDE.md` feature gate guidelines
  - [ ] Add examples of trait usage

- [ ] **Final verification**
  - [ ] Run full test suite on host
  - [ ] Run embedded build and flash test
  - [ ] Measure control loop performance

### Phase 4 Deliverables

- `scripts/check-feature-gates.sh` CI script
- Updated documentation
- Final feature gate count ≤60

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
./scripts/check-feature-gates.sh
```

### Phase 4 Acceptance Criteria

- Feature gate count ≤60
- CI script passes
- All documentation updated
- No performance regression confirmed

---

## Definition of Done

- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] Feature gate count ≤60
- [ ] `pico2_w` gates only in `src/platform/`
- [ ] `docs/architecture.md` updated
- [ ] ADR-3ciu6 status updated to Approved
- [ ] No duplicate implementations remain

## Open Questions

- [x] Should backward-compatible global statics be maintained? → Yes, during transition
- [ ] How to handle executor task macros that require platform feature? → Move to `src/platform/` or use wrapper
