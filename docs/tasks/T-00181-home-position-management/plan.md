# T-00181 Home Position Management Implementation

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement home position management in 4 phases: core state + shared message builder, auto-set + broadcast + disarmed update + lock, command handlers + pre-arm check, and telemetry fix.

## Success Metrics

- [ ] Home auto-set on first GPS 3D fix with HOME_POSITION broadcast
- [ ] Home refined at 1 Hz while disarmed if moved >0.5m (skipped if locked)
- [ ] GCS-set home locked from auto-update
- [ ] Arming blocked with "waiting for home" if home not set
- [ ] REQUEST_MESSAGE(242) and GET_HOME_POSITION(410) return HOME_POSITION
- [ ] `relative_alt` computed from home altitude
- [ ] All tests pass, clippy clean, embedded build succeeds

## Scope

- Goal: Complete home position lifecycle matching ArduPilot behavior
- Non-Goals: EKF origin, GPS_GLOBAL_ORIGIN, DO_SET_HOME via COMMAND_LONG, home persistence
- Assumptions: Existing MavlinkLoopRunner and CommandHandler patterns are stable
- Constraints: Core remains `no_std`; all new logic must be testable on host without embassy

## ADR & Legacy Alignment

- [x] Follows existing `MavlinkLoopRunner` control loop pattern (battery/GCS failsafe)
- [x] Follows existing `PreArmCheck` trait pattern for arming checks
- [x] Follows existing `CommandHandler` pattern for MAV_CMD handling
- [x] No new parameters needed (all behavior is architectural, matching ArduPilot)

## Plan Summary

| Phase | Description                                                       | Effort  |
| ----- | ----------------------------------------------------------------- | ------- |
| 1     | SystemState `home_locked` + shared `to_mavlink_message()`         | 1 hour  |
| 2     | Auto-set + broadcast + disarmed update + home lock in DO_SET_HOME | 4 hours |
| 3     | REQUEST_MESSAGE(242) + GET_HOME_POSITION(410) + pre-arm check     | 3 hours |
| 4     | GLOBAL_POSITION_INT `relative_alt` fix                            | 1 hour  |

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: SystemState Changes and Shared Message Builder

### Goal

- Add `home_locked: bool` field to `SystemState`
- Move `build_home_position_message()` to `HomePosition::to_mavlink_message()`
- Update `CommandHandler` to delegate to new method

### Inputs

- Source Code to Read:
  - `crates/core/src/autopilot/state.rs` -- `SystemState`, `HomePosition`
  - `crates/core/src/communication/handlers/command.rs` -- `build_home_position_message()`, `handle_set_home()`
- Dependencies: None

### Tasks

- [ ] **Add `home_locked: bool` to `SystemState`**
  - [ ] Add field with default `false` in `Default` impl
  - [ ] Add field in `init()` and `new()` constructors
- [ ] **Add `HomePosition::to_mavlink_message()` method**
  - [ ] Move message construction logic from `CommandHandler::build_home_position_message()`
  - [ ] Return `HOME_POSITION_DATA`
  - [ ] Add unit test for degE7/mm conversion accuracy
- [ ] **Update `CommandHandler`**
  - [ ] Change `build_home_position_message()` to delegate to `HomePosition::to_mavlink_message()`
  - [ ] Verify all existing tests still pass
- [ ] **Verification**
  - [ ] `cargo test -p pico_trail_core --lib --quiet`
  - [ ] `cargo clippy -p pico_trail_core --all-targets -- -D warnings`
  - [ ] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- `SystemState.home_locked` field
- `HomePosition::to_mavlink_message()` shared method
- `CommandHandler` delegates to new method (backward compatible)

### Acceptance Criteria (Phase Gate)

- `home_locked` defaults to `false`
- `to_mavlink_message()` produces same output as old `build_home_position_message()`
- All existing tests pass unchanged
- Embedded build succeeds

### Rollback/Fallback

- Remove `home_locked` field and revert method move; no functional impact

---

## Phase 2: Auto-Set, Broadcast, Disarmed Update, and Home Lock

### Goal

- Auto-set home on first GPS 3D fix in control loop
- Broadcast HOME_POSITION on every home change
- Update home at 1 Hz while disarmed if moved >0.5m
- Home lock: DO_SET_HOME sets `home_locked = true`

### Inputs

- Source Code to Read:
  - `crates/core/src/autopilot/mavlink_runner.rs` -- `MavlinkLoopRunner`
  - `crates/firmware/examples/pico_trail_rover.rs` -- firmware control loop caller
  - `crates/sitl/src/autopilot.rs` -- SITL control loop caller
  - `crates/core/src/communication/handlers/command.rs` -- `handle_set_home()`
- Dependencies:
  - Phase 1: `home_locked` field and `to_mavlink_message()` method

### Tasks

- [ ] **Add home management helpers to MavlinkLoopRunner**
  - [ ] Add `check_home_auto_set(has_home, gps_position, gps_fix_type) -> bool`
  - [ ] Add `check_home_update(home, gps, home_locked) -> bool` with 0.5m threshold
  - [ ] Add distance calculation function (flat-earth approximation)
  - [ ] Define `DISTANCE_HOME_MINCHANGE: f32 = 0.5`
- [ ] **Unit tests for home management helpers**
  - [ ] `check_home_auto_set`: true when no home + Fix3D, false when home exists, false when no GPS
  - [ ] `check_home_update`: true when >0.5m and unlocked, false when locked, false when <0.5m
  - [ ] Distance calculation: verify accuracy at equator and mid-latitudes
  - [ ] Home persists through GPS loss: set home, clear GPS, verify home remains Some
  - [ ] Auto-set fires only once: verify second call with home already set returns false
- [ ] **Set `home_locked = true` in `handle_set_home()`**
  - [ ] Both branches (use_current and specified location) set `home_locked = true`
  - [ ] Add test: DO_SET_HOME sets `home_locked = true`
- [ ] **Wire auto-set and disarmed update in firmware caller**
  - [ ] In `pico_trail_rover.rs` control loop: check `check_home_auto_set()`, if true set home, set `home_locked = false`, and broadcast HOME_POSITION
  - [ ] Implement 1 Hz rate limiter for disarmed update (counter or elapsed time check against control loop tick rate)
  - [ ] At 1 Hz while disarmed: check `check_home_update()`, if true update home and broadcast
  - [ ] Use `HomePosition::to_mavlink_message()` for broadcast message
  - [ ] Log "Home set to {lat}, {lon}" on auto-set
- [ ] **Wire auto-set and disarmed update in SITL caller**
  - [ ] In SITL `autopilot.rs`: same logic as firmware caller (auto-set with `home_locked = false`, 1 Hz disarmed update)
  - [ ] Add SITL integration test: auto-set on GPS fix
- [ ] **NFR verification (NFR-00179, NFR-00180)**
  - [ ] Verify auto-set operation has no heap allocations (code review: only struct copies)
  - [ ] Verify HOME_POSITION is not added to any `StreamConfig` in TelemetryStreamer (negative check)
  - [ ] Verify on-change-only pattern: no periodic timer for HOME_POSITION exists
- [ ] **Verification**
  - [ ] `cargo test -p pico_trail_core --lib --quiet`
  - [ ] `cargo test -p pico_trail_core --features embassy --lib --quiet`
  - [ ] `cargo clippy --all-targets -- -D warnings`
  - [ ] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- Home auto-set and disarmed update logic in MavlinkLoopRunner
- HOME_POSITION broadcast on every home change
- Home lock set by DO_SET_HOME
- Firmware and SITL callers wired

### Acceptance Criteria (Phase Gate)

- Home auto-set fires exactly once on first GPS 3D fix
- HOME_POSITION broadcast after auto-set and disarmed update
- Disarmed update skipped when `home_locked == true`
- Disarmed update skipped when moved <0.5m
- DO_SET_HOME sets `home_locked = true`
- All tests pass, embedded build succeeds

### Rollback/Fallback

- Remove home management methods from MavlinkLoopRunner; revert caller changes. Home remains None until GCS sets it (current behavior)

---

## Phase 3: Command Handlers and Pre-Arm Check

### Goal

- Support `MAV_CMD_REQUEST_MESSAGE` with param1=242 (HOME_POSITION)
- Support `MAV_CMD_GET_HOME_POSITION` (command 410, deprecated)
- Add "waiting for home" pre-arm check in GPS category

### Inputs

- Source Code to Read:
  - `crates/core/src/communication/handlers/command.rs` -- `handle_command_long`, `handle_request_message`
  - `crates/core/src/arming/checks.rs` -- `PreArmCheck` trait, `create_default_checker()`
  - `crates/core/src/arming/error.rs` -- `CheckCategory::Gps`
- Dependencies:
  - Phase 1: `to_mavlink_message()` method
  - Phase 2: Home auto-set (home is typically set before arm attempt)

### Tasks

- [ ] **Add REQUEST_MESSAGE handling for message ID 242**
  - [ ] Intercept `MavCmd::MAV_CMD_REQUEST_MESSAGE` at `handle_command_long` level for msg_id 242
  - [ ] Return HOME_POSITION in extra_messages Vec when home is set
  - [ ] Return `MAV_RESULT_ACCEPTED` when home set
  - [ ] Return `MAV_RESULT_FAILED` when home is None
  - [ ] Delegate all other message IDs to existing `handle_request_message()`
- [ ] **Add MAV_CMD_GET_HOME_POSITION handler**
  - [ ] Add `MavCmd::MAV_CMD_GET_HOME_POSITION` match arm in `handle_command_long`
  - [ ] Return HOME_POSITION in extra_messages Vec when home is set
  - [ ] Return `MAV_RESULT_ACCEPTED` when home set
  - [ ] Return `MAV_RESULT_FAILED` when home is None
- [ ] **Unit tests for command handlers**
  - [ ] REQUEST_MESSAGE param1=242 returns HOME_POSITION when home set
  - [ ] REQUEST_MESSAGE param1=242 returns FAILED when home not set
  - [ ] REQUEST_MESSAGE param1=148 still works (regression test)
  - [ ] GET_HOME_POSITION returns HOME_POSITION when home set
  - [ ] GET_HOME_POSITION returns FAILED when home not set
- [ ] **Add HomePositionCheck pre-arm check**
  - [ ] Create `HomePositionCheck` struct implementing `PreArmCheck`
  - [ ] Category: `CheckCategory::Gps`
  - [ ] Fails with "waiting for home" when `state.home_position.is_none()`
  - [ ] Register in `create_default_checker()`
- [ ] **Unit tests for pre-arm check**
  - [ ] Check passes when home_position is Some
  - [ ] Check fails with "waiting for home" when home_position is None
  - [ ] Check is skipped when GPS category is disabled in ARMING_CHECK
- [ ] **Verification**
  - [ ] `cargo test -p pico_trail_core --lib --quiet`
  - [ ] `cargo test -p pico_trail_core --features embassy --lib --quiet`
  - [ ] `cargo clippy --all-targets -- -D warnings`
  - [ ] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- REQUEST_MESSAGE(242) returns HOME_POSITION
- GET_HOME_POSITION(410) returns HOME_POSITION
- HomePositionCheck pre-arm check in GPS category

### Acceptance Criteria (Phase Gate)

- GCS can request home position via both standard and deprecated commands
- Arming blocked with "waiting for home" when home is not set
- ARMING_CHECK GPS bit disables the home check
- Existing command handler tests pass unchanged
- All tests pass, embedded build succeeds

### Rollback/Fallback

- Remove new match arms from command handler; remove HomePositionCheck from checker. Commands return UNSUPPORTED, arming proceeds without home check

---

## Phase 4: GLOBAL_POSITION_INT `relative_alt` Fix

### Goal

- Compute `relative_alt` in GLOBAL_POSITION_INT as altitude relative to home

### Inputs

- Source Code to Read:
  - `crates/core/src/communication/handlers/telemetry.rs` -- `build_global_position_int()` (line 308)
- Dependencies:
  - Phase 1: `home_position` field in SystemState (already exists)

### Tasks

- [ ] **Fix `relative_alt` computation**
  - [ ] If home set and GPS available: `relative_alt = ((gps_alt - home_alt) * 1000.0) as i32`
  - [ ] If home not set: `relative_alt = 0` (unchanged fallback)
  - [ ] Update `build_global_position_int()` in telemetry handler
- [ ] **Unit tests**
  - [ ] `relative_alt` correct when home and GPS available (positive and negative altitude difference)
  - [ ] `relative_alt` is 0 when home not set
  - [ ] `relative_alt` is 0 when GPS not available
- [ ] **Verification**
  - [ ] `cargo test -p pico_trail_core --lib --quiet`
  - [ ] `cargo clippy -p pico_trail_core --all-targets -- -D warnings`
  - [ ] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- Correct `relative_alt` in GLOBAL_POSITION_INT telemetry

### Acceptance Criteria (Phase Gate)

- `relative_alt` shows altitude difference from home in millimeters
- Fallback to 0 when home not set (no regression)
- All tests pass, embedded build succeeds

### Rollback/Fallback

- Revert to `relative_alt: 0` (current behavior, no safety impact)

---

## Definition of Done

- [ ] `home_locked` field added to SystemState with correct defaults
- [ ] `HomePosition::to_mavlink_message()` shared method
- [ ] Home auto-set on first GPS 3D fix with HOME_POSITION broadcast
- [ ] Home refined at 1 Hz while disarmed (>0.5m threshold, skip if locked)
- [ ] DO_SET_HOME sets `home_locked = true`
- [ ] REQUEST_MESSAGE(242) returns HOME_POSITION
- [ ] GET_HOME_POSITION(410) returns HOME_POSITION
- [ ] HomePositionCheck pre-arm check registered in GPS category
- [ ] `relative_alt` computed from home altitude
- [ ] Firmware and SITL callers wired for auto-set and disarmed update
- [ ] Core tests pass: `cargo test -p pico_trail_core --lib`
- [ ] Core (embassy) passes: `cargo test -p pico_trail_core --features embassy --lib`
- [ ] Embedded build: `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] Clippy clean: `cargo clippy --all-targets -- -D warnings`
- [ ] Code formatted: `cargo fmt`

## Open Questions

- [x] Should home auto-set and disarmed update be in MavlinkLoopRunner or separate struct? --> Methods on MavlinkLoopRunner, following the pattern of `check_battery` and `check_gcs_failsafe`. Keeps all control loop checks co-located
- [x] Should distance calculation live in MavlinkLoopRunner or a shared location? --> Private function in `mavlink_runner.rs` or `state.rs`. Only used for 0.5m threshold, no need for a shared utility
