# T-00038 Fix Guided Mode Heading Oscillation

## Metadata

- Type: Implementation Plan
- Status: Implemented

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Fix the rover spinning in circles during Guided mode by adding PD steering with rate limiting, heading-error-based throttle reduction, and heading source hysteresis. These changes address the five root causes identified in AN-00044.

## Success Metrics

- [ ] Steering output is damped: change per cycle does not exceed `max_steering_rate * dt`
- [ ] PD controller opposes rapid heading error changes (D-term test)
- [ ] Throttle reduces to 0% at 90+ degree heading error
- [ ] Heading source switching uses hysteresis (no flip-flop at threshold)
- [ ] All existing tests pass; no regressions
- [ ] Embedded build compiles successfully

## Scope

- Goal: Eliminate heading oscillation loop in Guided mode
- Non-Goals: Full PID, heading source blending, L1 navigation
- Assumptions: GPS at \~1 Hz, AHRS at 100 Hz, nav loop at 50 Hz
- Constraints: no_std, f32, < 1ms per update on RP2350

## ADR & Legacy Alignment

- [ ] Confirm ADR-00022-navigation-controller-architecture is referenced
- [ ] Confirm ADR-00033-heading-source-integration is referenced
- [ ] Existing `NavigationController` trait interface preserved (backward-compatible)

## Plan Summary

- Phase 1 -- PD steering controller with rate limiting (`crates/core`)
- Phase 2 -- Heading-error-based throttle reduction (`crates/core`)
- Phase 3 -- Heading source hysteresis (`crates/firmware`)
- Phase 4 -- Integration verification

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: PD Steering Controller with Rate Limiting

### Goal

- Replace P-only steering with PD controller
- Add slew rate limiting to steering output
- Add state fields to `SimpleNavigationController`
- Add config fields to `SimpleNavConfig`

### Inputs

- Source Code to Modify:
  - `crates/core/src/navigation/types.rs` -- Add `steering_d_gain`, `max_steering_rate` to `SimpleNavConfig`
  - `crates/core/src/navigation/controller.rs` -- Add state fields, modify `calculate_steering`, update `update()` to pass `dt`
- Documentation:
  - `docs/analysis/AN-00044-guided-mode-heading-oscillation.md` -- Root Cause 1

### Tasks

- [x] **Add config fields to `SimpleNavConfig`**
  - [x] Add `steering_d_gain: f32` field (default: 0.05)
  - [x] Add `max_steering_rate: f32` field (default: 2.0)
  - [x] Update `Default` impl with new field defaults
- [x] **Add state fields to `SimpleNavigationController`**
  - [x] Add `prev_heading_error: f32` field (init: 0.0)
  - [x] Add `prev_steering: f32` field (init: 0.0)
  - [x] Update `new()` and `with_config()` constructors
- [x] **Implement PD steering in `calculate_steering`**
  - [x] Change signature to `&mut self, heading_error: f32, dt: f32`
  - [x] Add proportional term (existing logic)
  - [x] Add derivative term using `prev_heading_error`
  - [x] Store `prev_heading_error` for next cycle
- [x] **Implement steering slew rate limiting**
  - [x] Compute `max_change = max_steering_rate * dt`
  - [x] Clamp delta from previous steering
  - [x] Store `prev_steering` for next cycle
- [x] **Update `reset()` to clear state**
  - [x] Reset `prev_heading_error` to 0.0
  - [x] Reset `prev_steering` to 0.0
- [x] **Update `update()` method**
  - [x] Pass `dt` to `calculate_steering`
- [x] **Unit tests**
  - [x] Test: D-term opposes increasing heading error
  - [x] Test: slew rate limits steering change per cycle
  - [x] Test: `reset()` clears derivative state
  - [x] Test: `dt = 0` disables D-term safely
  - [x] Test: existing tests still pass with new defaults

### Deliverables

- Modified `crates/core/src/navigation/types.rs`
- Modified `crates/core/src/navigation/controller.rs`

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet navigation
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- PD steering produces damped output under oscillating heading input
- Slew rate prevents steering from changing faster than configured rate
- All existing navigation tests pass without modification
- Embedded build succeeds

### Rollback/Fallback

- Set `steering_d_gain = 0.0` and `max_steering_rate = 0.0` to revert to P-only behavior

---

## Phase 2: Heading-Error-Based Throttle Reduction

### Phase 2 Goal

- Reduce throttle proportionally to heading error
- Prevent full-speed driving while heading is far off target
- Add config field for heading error throttle threshold

### Phase 2 Inputs

- Source Code to Modify:
  - `crates/core/src/navigation/types.rs` -- Add `throttle_heading_error_max`
  - `crates/core/src/navigation/controller.rs` -- Modify `calculate_throttle`, update `update()`
- Dependencies:
  - Phase 1: controller state changes

### Phase 2 Tasks

- [x] **Add config field to `SimpleNavConfig`**
  - [x] Add `throttle_heading_error_max: f32` field (default: 90.0)
  - [x] Update `Default` impl
- [x] **Modify `calculate_throttle`**
  - [x] Add `heading_error_abs: f32` parameter
  - [x] Calculate heading error scaling factor
  - [x] Multiply distance-based throttle by heading error scale
- [x] **Update `update()` method**
  - [x] Pass `heading_error.abs()` to `calculate_throttle`
- [x] **Unit tests**
  - [x] Test: throttle = 0 when heading error = 90 degrees
  - [x] Test: throttle = 100% when heading error = 0 degrees
  - [x] Test: throttle scales linearly between 0 and 90 degrees
  - [x] Test: throttle at 180 degrees heading error = 0%
  - [x] Test: existing distance-based throttle tests still pass

### Phase 2 Deliverables

- Modified `crates/core/src/navigation/types.rs`
- Modified `crates/core/src/navigation/controller.rs`

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet navigation
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- Throttle is 0% when heading error >= 90 degrees
- Throttle is 100% when heading error = 0 degrees (and far from target)
- Distance-based approach slowdown still works
- All tests pass

### Phase 2 Rollback/Fallback

- Set `throttle_heading_error_max = 180.0` to effectively disable (heading error never exceeds 180)

---

## Phase 3: Heading Source Hysteresis

### Phase 3 Goal

- Add hysteresis band to `FusedHeadingSource` speed threshold
- Prevent rapid GPS COG / AHRS switching near 1.0 m/s
- Track current heading source state

### Phase 3 Inputs

- Source Code to Modify:
  - `crates/firmware/src/subsystems/navigation/heading.rs` -- Modify `FusedHeadingSource`
- Documentation:
  - `docs/analysis/AN-00044-guided-mode-heading-oscillation.md` -- Root Cause 2

### Phase 3 Tasks

- [x] **Add hysteresis state to `FusedHeadingSource`**
  - [x] Add `speed_hysteresis: f32` field (default: 0.3)
  - [x] Add `using_gps_cog: Cell<bool>` field (default: false, interior mutability for `&self` trait)
  - [x] Update constructors (`new`, `with_defaults`)
- [x] **Modify `get_gps_heading`**
  - [x] Compute effective threshold based on `using_gps_cog` state
  - [x] Update `using_gps_cog` when source changes
- [x] **Update `source_type`**
  - [x] Apply hysteresis to speed threshold in `source_type()` method
- [x] **Unit tests**
  - [x] Test: GPS COG activated at `threshold + hysteresis` (1.3 m/s)
  - [x] Test: GPS COG deactivated at `threshold - hysteresis` (0.7 m/s)
  - [x] Test: no switching when speed oscillates between 0.8 and 1.2 m/s (within band)
  - [x] Test: existing heading source tests still pass

### Phase 3 Deliverables

- Modified `crates/firmware/src/subsystems/navigation/heading.rs`

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- Heading source does not switch when speed is within hysteresis band
- GPS COG engages at higher threshold, disengages at lower threshold
- Existing heading source behavior preserved outside hysteresis band
- All tests pass

### Phase 3 Rollback/Fallback

- Set `speed_hysteresis = 0.0` to revert to hard-switch behavior

---

## Phase 4: Integration Verification

### Phase 4 Goal

- Verify all changes compile and work together
- Run full test suite
- Verify embedded build

### Phase 4 Tasks

- [x] **Full test suite**
  - [x] Run `cargo test --lib --quiet` -- 257 passed, 0 failed
  - [x] Run `cargo clippy --all-targets -- -D warnings` -- 0 warnings
- [x] **Embedded build**
  - [x] Run `./scripts/build-rp2350.sh pico_trail_rover` -- success
- [x] **Documentation**
  - [x] Update plan.md phase checkboxes
  - [ ] Mark task status in README.md

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 4 Acceptance Criteria

- All tests pass
- No clippy warnings
- Embedded build succeeds
- Phase checkboxes complete

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

- [ ] Optimal `steering_d_gain` value: start with 0.005, tune in field
- [ ] Optimal `speed_hysteresis` value: start with 0.3 m/s, adjust based on GPS noise
- [ ] Should `max_steering_rate` be exposed as MAVLink parameter? Defer to post-fix
