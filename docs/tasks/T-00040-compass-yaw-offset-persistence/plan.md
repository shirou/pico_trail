# T-00040 Compass Yaw Offset Persistence

## Metadata

- Type: Implementation Plan
- Status: Done

## Links

- Associated Design Document:
  - [T-00040-compass-yaw-offset-persistence-design](design.md)

## Overview

Persist `compass_yaw_offset` across reboots by adding `COMPASS_DEC` to the existing `CompassParams` / `ParameterStore` infrastructure. On boot, the offset is loaded from flash. After calibration via `MAV_CMD_FIXED_MAG_CAL_YAW`, the offset is saved to flash via the `ParamHandler`.

## Success Metrics

- [x] `COMPASS_DEC` visible in Mission Planner parameter list
- [x] Calibration offset persists across simulated reboot (unit test)
- [x] `SystemState.compass_yaw_offset` initialized from flash on boot
- [x] All existing tests pass; RP2350 build succeeds

## Scope

- Goal: Persist compass yaw offset and fix doc comment
- Non-Goals: Hard iron offsets, WMM integration, MAV_CMD_FIXED_MAG_CAL (42004)
- Assumptions: Existing ParameterStore and flash persistence infrastructure is functional
- Constraints: Use `COMPASS_DEC` parameter name per ADR-00035

## ADR & Legacy Alignment

- [x] ADR-00035 defines the approach: yaw offset with `COMPASS_DEC` parameter
- [x] No legacy patterns to migrate

## Plan Summary

- Phase 1 – Add `COMPASS_DEC` to CompassParams and boot loading
- Phase 2 – Save offset after calibration and fix doc comment
- Phase 3 – Unit tests

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Add COMPASS_DEC Parameter and Boot Loading

### Goal

- Register `COMPASS_DEC` in `CompassParams`
- Load the offset into `SystemState.compass_yaw_offset` on boot

### Inputs

- Source Code to Modify:
  - `crates/firmware/src/parameters/compass.rs` – Add `declination` field and parameter registration
  - `crates/firmware/src/communication/mavlink/state.rs` – Load `compass_yaw_offset` from `CompassParams`

### Tasks

- [x] **Add `COMPASS_DEC` to CompassParams**
  - [x] Add `declination: f32` field to `CompassParams` struct
  - [x] Add `declination: 0.0` to `Default` impl
  - [x] Register `COMPASS_DEC` with `ParamValue::Float(0.0)` in `register_defaults()`
  - [x] Read `COMPASS_DEC` in `from_store()` and assign to `declination`
  - [x] Update module doc comment to list `COMPASS_DEC`

- [x] **Load offset on boot**
  - [x] In `SystemState::from_param_store()`, read `CompassParams::from_store()` and use `compass_params.declination` for `compass_yaw_offset` instead of hardcoded `0.0`

### Deliverables

- Modified `compass.rs` with `COMPASS_DEC` support
- Modified `state.rs` with boot-time offset loading

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- `COMPASS_DEC` parameter registered successfully
- `SystemState::from_param_store()` uses the stored value
- Existing tests pass; RP2350 build succeeds

### Rollback/Fallback

- Revert `compass.rs` and `state.rs` changes

---

## Phase 2: Save Offset After Calibration and Fix Doc Comment

### Phase 2 Goal

- Save `compass_yaw_offset` to `COMPASS_DEC` in ParameterStore after successful `MAV_CMD_FIXED_MAG_CAL_YAW`
- Fix the doc comment in `handle_fixed_mag_cal_yaw`

### Phase 2 Inputs

- Dependencies:
  - Phase 1: `COMPASS_DEC` parameter registered
  - `MessageDispatcher` has access to both `CommandHandler` and `ParamHandler`
- Source Code to Modify:
  - `crates/firmware/src/communication/mavlink/dispatcher.rs` – Save COMPASS_DEC after MagCal command
  - `crates/firmware/src/communication/mavlink/handlers/command.rs` – Fix doc comment

### Phase 2 Tasks

- [x] **Save offset in dispatcher after MagCal**
  - [x] In the `COMMAND_LONG` dispatch path, after `handle_command_long()` returns ACCEPTED for `MAV_CMD_FIXED_MAG_CAL_YAW`:
    - Read `compass_yaw_offset` from `SYSTEM_STATE`
    - Set `COMPASS_DEC` in `param_handler.store_mut()`
    - The parameter will be saved to flash by the existing dirty-check/save mechanism (or explicitly trigger save)
  - [x] Add log message confirming parameter save

- [x] **Fix doc comment**
  - [x] In `handle_fixed_mag_cal_yaw`, change `magnetic north = 0` to `true north = 0` in the doc comment (line \~505)

### Phase 2 Deliverables

- Modified `dispatcher.rs` with MagCal parameter save logic
- Fixed doc comment in `command.rs`

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- After calibration, `COMPASS_DEC` is set in ParameterStore
- Doc comment correctly states "true north = 0"
- No regressions

### Phase 2 Rollback/Fallback

- Revert dispatcher changes; doc comment fix is safe to keep

---

## Phase 3: Unit Tests

### Phase 3 Goal

- Add unit tests verifying parameter registration, boot loading, and round-trip persistence

### Phase 3 Tasks

- [x] **CompassParams tests**
  - [x] Test `COMPASS_DEC` is registered in `register_defaults()`
  - [x] Test `from_store()` returns stored `COMPASS_DEC` value
  - [x] Test `from_store()` returns `0.0` when `COMPASS_DEC` not set
  - [x] Test `from_store()` with custom `COMPASS_DEC` value (e.g., `0.126` rad ≈ 7.2°)

- [x] **SystemState boot loading test**
  - [x] Test `from_param_store()` initializes `compass_yaw_offset` from `COMPASS_DEC`
  - [x] Test `from_param_store()` with `COMPASS_DEC = 0.0` (no calibration)
  - [x] Test `from_param_store()` with `COMPASS_DEC = 0.126` (calibrated)

### Phase 3 Deliverables

- Unit tests in `compass.rs` and `state.rs`

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- All new tests pass
- All existing tests pass
- No regressions

---

## Definition of Done

- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh pico_trail_rover`
- [x] `COMPASS_DEC` registered and accessible via MAVLink PARAM protocol
- [x] Offset loaded from flash on boot
- [x] Offset saved to flash after calibration
- [x] Doc comment fixed
- [x] No `unsafe` and no vague naming

## Open Questions

- [x] How does `handle_fixed_mag_cal_yaw` access ParameterStore?
  - **Answer:** It doesn't directly. The `MessageDispatcher` has access to both `CommandHandler` and `ParamHandler`. The save is performed in the dispatcher after the command handler returns ACCEPTED.
