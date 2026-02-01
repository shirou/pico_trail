# T-f4r7a Break Spin-in-Place Positive Feedback Loop

## Metadata

- Type: Implementation Plan
- Status: Implementation Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement four mitigations to break the spin-in-place positive feedback loop discovered during field testing after T-w8x3p: enable slew rate limiting, cap spin-in-place steering, add heading EMA filter, and use gyroscope yaw rate for D-term.

## Success Metrics

- [x] Slew rate limiting active by default (`max_steering_rate = 2.0`)
- [x] D-gain effective at normal heading rates (`steering_d_gain = 0.05`)
- [x] Spin-in-place steering capped when throttle near zero
- [x] Heading EMA filter smooths noise before navigation controller
- [x] D-term uses gyroscope yaw rate when available
- [x] All existing tests pass; no regressions
- [x] Embedded build compiles successfully

## Scope

- Goal: Break the spin-vibration-heading-corruption feedback loop
- Non-Goals: Full EKF, heading source blending, L1 navigation
- Assumptions: GPS at \~1 Hz, AHRS at 100 Hz, nav loop at 50 Hz, BNO086 gyroscope available
- Constraints: no_std, f32, < 1ms per update on RP2350

## ADR & Legacy Alignment

- [x] Confirm ADR-wrcuk-navigation-controller-architecture is referenced
- [x] Confirm ADR-h3k9f-heading-source-integration is referenced
- [x] Existing `NavigationController` trait interface updated (yaw_rate parameter added)
- [x] All callers of `NavigationController::update()` updated

## Plan Summary

- Phase 1 -- Fix defaults and add spin-in-place cap (`crates/core`)
- Phase 2 -- Heading EMA filter (`crates/core`)
- Phase 3 -- Gyroscope yaw rate for D-term (`crates/core` + `crates/firmware`)
- Phase 4 -- Integration verification

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Fix Defaults and Spin-in-Place Cap

### Goal

- Change `max_steering_rate` default from 0.0 to 2.0
- Change `steering_d_gain` default from 0.005 to 0.05
- Add spin-in-place steering cap config fields
- Implement steering cap in controller update

### Inputs

- Source Code to Modify:
  - `crates/core/src/navigation/types.rs` -- Fix defaults, add `max_spin_steering`, `spin_throttle_threshold`
  - `crates/core/src/navigation/controller.rs` -- Add spin-in-place cap after steering/throttle calculation
- Documentation:
  - `docs/analysis/AN-27568-position-target-navigation.md` -- Root Causes 6, 7, 8

### Tasks

- [x] **Fix `SimpleNavConfig` defaults**
  - [x] Change `max_steering_rate` from 0.0 to 2.0
  - [x] Change `steering_d_gain` from 0.005 to 0.05
- [x] **Add spin-in-place config fields to `SimpleNavConfig`**
  - [x] Add `max_spin_steering: f32` field (default: 0.3)
  - [x] Add `spin_throttle_threshold: f32` field (default: 0.1)
  - [x] Update `Default` impl with new field defaults
- [x] **Implement spin-in-place cap in `update()`**
  - [x] After computing steering and throttle, check if throttle < `spin_throttle_threshold`
  - [x] If so, clamp steering to `[-max_spin_steering, max_spin_steering]`
  - [x] Return capped steering in `NavigationOutput`
- [x] **Unit tests**
  - [x] Test: `max_steering_rate` default is 2.0
  - [x] Test: `steering_d_gain` default is 0.05
  - [x] Test: steering capped when throttle < spin_throttle_threshold
  - [x] Test: steering not capped when throttle >= spin_throttle_threshold
  - [x] Test: `max_spin_steering = 1.0` effectively disables cap
  - [x] Test: existing tests updated for new D-gain default if needed

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

- Slew rate limiting is active by default
- Spin-in-place cap limits steering when throttle is near zero
- All existing navigation tests pass (adjusted for new defaults where needed)
- Embedded build succeeds

### Rollback/Fallback

- `max_steering_rate = 0.0` disables slew rate limiting
- `max_spin_steering = 1.0` disables spin-in-place cap

---

## Phase 2: Heading EMA Filter

### Phase 2 Goal

- Implement angle-aware exponential moving average filter for heading
- Add config field for filter alpha
- Integrate filter into `SimpleNavigationController`

### Phase 2 Inputs

- Source Code to Create:
  - `crates/core/src/navigation/heading_filter.rs` -- New file for `HeadingFilter` struct
- Source Code to Modify:
  - `crates/core/src/navigation/mod.rs` -- Add `heading_filter` module
  - `crates/core/src/navigation/types.rs` -- Add `heading_filter_alpha` config field
  - `crates/core/src/navigation/controller.rs` -- Integrate filter into controller
- Dependencies:
  - Phase 1: config structure changes

### Phase 2 Tasks

- [x] **Create `HeadingFilter` struct**
  - [x] Create `crates/core/src/navigation/heading_filter.rs`
  - [x] Implement `new(alpha)`, `apply(heading)`, `reset()` methods
  - [x] Use angle-aware interpolation with `wrap_180`/`wrap_360`
- [x] **Add module to navigation**
  - [x] Add `pub mod heading_filter;` to `crates/core/src/navigation/mod.rs`
- [x] **Add config field to `SimpleNavConfig`**
  - [x] Add `heading_filter_alpha: f32` field (default: 0.3)
  - [x] Update `Default` impl
- [x] **Integrate filter into `SimpleNavigationController`**
  - [x] Add `heading_filter: HeadingFilter` field to struct
  - [x] Apply filter to heading in `update()` before calculating heading error
  - [x] Reset filter in `reset()`
  - [x] Initialize filter from config in constructors
- [x] **Unit tests**
  - [x] Test: filter smooths noisy heading input
  - [x] Test: angle wrapping (350 -> 10 degrees transition)
  - [x] Test: alpha=1.0 passes heading through unchanged
  - [x] Test: alpha=0.0 holds first heading indefinitely
  - [x] Test: reset clears filter state
  - [x] Test: first call returns raw heading (no smoothing)

### Phase 2 Deliverables

- New `crates/core/src/navigation/heading_filter.rs`
- Modified `crates/core/src/navigation/mod.rs`
- Modified `crates/core/src/navigation/types.rs`
- Modified `crates/core/src/navigation/controller.rs`

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- HeadingFilter reduces noise amplitude in unit tests
- Filter handles angle wrapping correctly
- Controller uses filtered heading for navigation
- All tests pass

### Phase 2 Rollback/Fallback

- Set `heading_filter_alpha = 1.0` to disable filtering (pass-through)

---

## Phase 3: Gyroscope Yaw Rate for D-Term

### Phase 3 Goal

- Add `yaw_rate_dps` parameter to `NavigationController::update()` trait
- Use gyroscope yaw rate for D-term when available
- Update firmware navigation task to pass gyroscope data
- Fall back to differenced heading when gyroscope unavailable

### Phase 3 Inputs

- Source Code to Modify:
  - `crates/core/src/navigation/controller.rs` -- Change `update()` and `calculate_steering()` signatures, use yaw_rate for D-term
  - `crates/firmware/examples/pico_trail_rover.rs` -- Pass gyroscope yaw rate to controller
- Dependencies:
  - Phase 2: controller changes

### Phase 3 Tasks

- [x] **Update `NavigationController` trait**
  - [x] Add `yaw_rate_dps: Option<f32>` parameter to `update()`
- [x] **Update `SimpleNavigationController::update()`**
  - [x] Accept `yaw_rate_dps` parameter
  - [x] Pass through to `calculate_steering()`
- [x] **Update `calculate_steering()` D-term logic**
  - [x] If `yaw_rate_dps` is Some, use it directly (negate sign: positive yaw rate = turning right = reduce positive steering)
  - [x] If None, fall back to differenced heading error (existing logic)
  - [x] Scale by `steering_d_gain` in both cases
- [x] **Update firmware navigation task**
  - [x] Read `AHRS_STATE.get_angular_rate().z.to_degrees()` (already available for logging)
  - [x] Pass as `Some(yaw_rate_dps)` to `controller.update()`
- [x] **Update all callers of `NavigationController::update()` (6 modes + 1 example + tests)**
  - [x] `crates/firmware/src/rover/mode/guided.rs` -- Pass `None` (no yaw rate provider yet)
  - [x] `crates/firmware/src/rover/mode/auto.rs` -- Pass `None` (no yaw rate provider yet)
  - [x] `crates/firmware/src/rover/mode/circle.rs` -- Pass `None` (no yaw rate provider yet)
  - [x] `crates/firmware/src/rover/mode/loiter.rs` -- Pass `None` (no yaw rate provider yet)
  - [x] `crates/firmware/src/rover/mode/rtl.rs` -- Pass `None` (no yaw rate provider yet)
  - [x] `crates/firmware/src/rover/mode/smartrtl.rs` -- Pass `None` (no yaw rate provider yet)
  - [x] `crates/firmware/examples/pico_trail_rover.rs` -- Pass `Some(yaw_rate_dps)` in navigation_task
  - [x] `crates/core/src/navigation/controller.rs` -- Update \~25 unit test calls to pass `None` for yaw_rate
- [x] **Unit tests**
  - [x] Test: D-term uses yaw_rate when provided (opposes turning direction)
  - [x] Test: D-term falls back to differenced heading when yaw_rate is None
  - [x] Test: positive yaw_rate reduces positive steering output
  - [x] Test: D-term magnitude meaningful at 30 deg/s with gain=0.05

### Phase 3 Deliverables

- Modified `crates/core/src/navigation/controller.rs` (trait + impl + tests)
- Modified `crates/firmware/examples/pico_trail_rover.rs`
- Modified `crates/firmware/src/rover/mode/guided.rs`
- Modified `crates/firmware/src/rover/mode/auto.rs`
- Modified `crates/firmware/src/rover/mode/circle.rs`
- Modified `crates/firmware/src/rover/mode/loiter.rs`
- Modified `crates/firmware/src/rover/mode/rtl.rs`
- Modified `crates/firmware/src/rover/mode/smartrtl.rs`

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- D-term uses gyroscope yaw rate when available
- D-term falls back to differenced heading when gyroscope unavailable
- D-term produces meaningful damping at normal heading rates (10-50 deg/s)
- All tests pass
- Embedded build succeeds

### Phase 3 Rollback/Fallback

- Pass `None` for yaw_rate to revert to differenced heading behavior
- Set `steering_d_gain = 0.0` to disable D-term entirely

---

## Phase 4: Integration Verification

### Phase 4 Goal

- Verify all changes compile and work together
- Run full test suite
- Verify embedded build

### Phase 4 Tasks

- [x] **Full test suite**
  - [x] Run `cargo test --lib --quiet`
  - [x] Run `cargo clippy --all-targets -- -D warnings`
- [x] **Embedded build**
  - [x] Run `./scripts/build-rp2350.sh pico_trail_rover`
- [x] **Documentation**
  - [x] Update plan.md phase checkboxes
  - [x] Update task status in README.md
  - [x] Update traceability: `bun scripts/trace-status.ts --check`

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
bun scripts/trace-status.ts --check
```

### Phase 4 Acceptance Criteria

- All tests pass
- No clippy warnings
- Embedded build succeeds
- Phase checkboxes complete
- Traceability check passes

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh pico_trail_rover` builds successfully
- [x] No `unsafe` code (except existing test helpers with `static mut`)
- [x] No vague naming ("manager"/"util")

## Open Questions

- [ ] Optimal `max_spin_steering` value: start with 0.3, tune in field
- [ ] Optimal `heading_filter_alpha`: start with 0.3, adjust based on vibration level
- [ ] Optimal `steering_d_gain` with gyroscope: start with 0.05, may need different tuning
