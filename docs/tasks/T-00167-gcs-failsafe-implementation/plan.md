# T-00167 GCS Communication Lost Failsafe Implementation

## Metadata

- Type: Implementation Plan
- Status: Done

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement GCS communication lost failsafe in 3 phases: parameter alignment, checker implementation with runner integration, and firmware cleanup with STATUSTEXT notifications.

## Success Metrics

- [x] `FailsafeAction` enum matches ArduPilot (0=None, 1=RTL, 2=Hold)
- [x] Two-stage timeout works: `FS_GCS_TIMEOUT` (5.0s) + `FS_TIMEOUT` (1.5s)
- [x] GCS failsafe triggers Hold/RTL when heartbeat lost while armed
- [x] Mode exemption returns `Activated` in Hold/RTL/SmartRTL (STATUSTEXT sent, no mode change)
- [x] No trigger when disarmed, GCS never seen, or already in safe mode
- [x] `FS_ACTION=3` (Disarm) falls back to Hold for GCS failsafe
- [x] Recovery clears immediately on heartbeat resume
- [x] All core and firmware tests pass
- [x] SITL E2E integration test passes
- [x] Embedded build succeeds

## Scope

- Goal: End-to-end GCS failsafe from detection to action execution
- Non-Goals: RC failsafe, unified failsafe framework, continue-mission mode, FS_OPTIONS
- Assumptions: Battery failsafe pattern is stable and proven
- Constraints: Core remains `no_std`; checker must be testable on host without embassy

## ADR & Legacy Alignment

- [x] Follows `BatteryFailsafeChecker` pattern (checker struct + runner integration)
- [x] `FailsafeAction` enum aligned with ArduPilot numbering
- [x] `FailsafeParams` aligned with ArduPilot two-timeout system

## Plan Summary

| Phase | Description                                          | Effort  |
| ----- | ---------------------------------------------------- | ------- |
| 1     | Parameter alignment and FailsafeAction fix           | 2 hours |
| 2     | GcsFailsafeChecker and MavlinkLoopRunner integration | 4 hours |
| 3     | Firmware cleanup and STATUSTEXT notifications        | 2 hours |

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Parameter Alignment and FailsafeAction Fix

### Goal

- Fix `FailsafeAction` enum numbering to match ArduPilot (0=None, 1=RTL, 2=Hold)
- Rename `FS_TIMEOUT` (5.0s) to `FS_GCS_TIMEOUT`, add new `FS_TIMEOUT` (1.5s)
- Update `FailsafeParams` struct with `gcs_timeout` field
- Update all references to match new enum values

### Inputs

- Source Code to Read:
  - `crates/core/src/parameters/failsafe.rs` -- `FailsafeAction`, `FailsafeParams`
  - `crates/core/src/autopilot/mavlink_runner.rs` -- references to `FailsafeAction`
  - Any other files referencing `FailsafeAction` enum values
- Dependencies: None

### Tasks

- [x] **Fix `FailsafeAction` enum numbering**
  - [x] Change `RTL = 1` (was 2), `Hold = 2` (was 1)
  - [x] Keep `None = 0`, `Disarm = 3` unchanged
  - [x] Update all match arms and references across the codebase
- [x] **Update `FailsafeParams` struct**
  - [x] Add `gcs_timeout: f32` field
  - [x] Update field comments to reflect ArduPilot semantics
  - [x] `timeout` = `FS_TIMEOUT` (persistence, 1.5s default)
  - [x] `gcs_timeout` = `FS_GCS_TIMEOUT` (detection, 5.0s default)
- [x] **Update `register_defaults()`**
  - [x] `FS_ACTION`: change default to `FailsafeAction::Hold as i32` (now 2)
  - [x] `FS_TIMEOUT`: change default from 5.0 to 1.5
  - [x] Add `FS_GCS_TIMEOUT` registration with default 5.0
- [x] **Update `from_store()`**
  - [x] Read `FS_GCS_TIMEOUT` into `gcs_timeout` field
  - [x] Update `FS_TIMEOUT` fallback default to 1.5
- [x] **Document parameter migration**
  - [x] Add comment in `from_store()` noting the FailsafeAction enum reorder (Hold: 1→2, RTL: 2→1) is a breaking change for stored parameters
  - [x] Ensure invalid `FS_ACTION` values fall back to Hold (safe default)
- [x] **Update tests**
  - [x] Fix `test_failsafe_params_defaults` for new defaults
  - [x] Fix `test_from_store_defaults` for new field and defaults
  - [x] Fix `test_from_store_custom_values` for new field
  - [x] Add test for `FS_GCS_TIMEOUT` parameter
  - [x] Verify `FailsafeAction` enum values match ArduPilot
- [x] **Verification**
  - [x] `cargo test -p pico_trail_core --lib --quiet`
  - [x] `cargo test -p pico_trail_core --features embassy --lib --quiet`
  - [x] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- `FailsafeAction` enum with ArduPilot-aligned numbering
- `FailsafeParams` with two-timeout fields (`timeout` + `gcs_timeout`)
- Updated parameter registration and loading

### Acceptance Criteria (Phase Gate)

- `FailsafeAction::RTL == 1`, `FailsafeAction::Hold == 2`
- `FS_TIMEOUT` defaults to 1.5s, `FS_GCS_TIMEOUT` defaults to 5.0s
- Invalid `FS_ACTION` values fall back to Hold (safe default)
- All existing tests pass with updated values
- Embedded build succeeds

### Rollback/Fallback

- Revert enum values and parameter defaults; no external API changes

---

## Phase 2: GcsFailsafeChecker and MavlinkLoopRunner Integration

### Goal

- Create `GcsFailsafeChecker` in `crates/core/src/autopilot/gcs_failsafe.rs`
- Extend `MavlinkLoopRunner` with `check_gcs_failsafe()` method
- Comprehensive unit tests for all failsafe scenarios

### Inputs

- Source Code to Read:
  - `crates/core/src/autopilot/battery.rs` -- `BatteryFailsafeChecker` pattern reference
  - `crates/core/src/autopilot/mavlink_runner.rs` -- runner to extend
  - `crates/core/src/communication/dispatcher.rs` -- `ConnectionState` fields
  - `crates/core/src/autopilot/state.rs` -- `FlightMode` enum
- Dependencies:
  - Phase 1: Updated `FailsafeAction` and `FailsafeParams`

### Tasks

- [x] **Create `GcsFailsafeChecker` module**
  - [x] Create `crates/core/src/autopilot/gcs_failsafe.rs`
  - [x] Define `GcsFailsafeAction` enum (None, SetMode, Activated, Cleared)
  - [x] Define `GcsFailsafeConfig` struct (enabled, gcs_timeout_us, persistence_us, action)
  - [x] Implement `GcsFailsafeConfig::from_params()` -- map `FailsafeAction::Disarm` to Hold (safe fallback for GCS failsafe)
  - [x] Define `GcsFailsafeChecker` struct (config, condition_detected, condition_start_us, failsafe_active)
  - [x] Implement `new()`, `check()`, `reset()`, `reset_with_config()`
  - [x] Export from `crates/core/src/autopilot/mod.rs`
- [x] **Implement `check()` method**
  - [x] Armed guard: return None if `!is_armed`
  - [x] Disabled guard: return None if `!config.enabled`
  - [x] Never-seen guard: return None if `heartbeat_count == 0`
  - [x] Compute heartbeat age with `current_time_us.saturating_sub(last_heartbeat_us)` (clock safety)
  - [x] Stage 1: detect condition when `heartbeat_age >= gcs_timeout_us`
  - [x] Stage 2: require persistence for `persistence_us` before action
  - [x] Mode exemption: return `Activated` (not `SetMode`) if Hold/RTL/SmartRTL
  - [x] Sticky: only return SetMode/Activated once per activation
  - [x] Recovery: clear state and return Cleared when heartbeat resumes
  - [x] Disarmed resets: clear condition_detected when `!is_armed`
- [x] **Extend `MavlinkLoopRunner`**
  - [x] Add `gcs_failsafe: GcsFailsafeChecker` field
  - [x] Update `new()` to accept `GcsFailsafeConfig`
  - [x] Update all existing callers of `MavlinkLoopRunner::new()` (firmware and SITL) for new signature
  - [x] Add `check_gcs_failsafe()` method with arm-transition config reload
  - [x] Handle `Activated` variant in caller: send STATUSTEXT warning but skip `set_mode()`
  - [x] Add `reset_gcs_failsafe()` for explicit reset if needed
- [x] **Unit tests for `GcsFailsafeChecker`**
  - [x] Normal heartbeats -- no trigger
  - [x] Two-stage timeout -- condition detected then persists to trigger
  - [x] Condition clears before persistence -- no trigger (transient dropout)
  - [x] Armed guard -- disarmed never triggers
  - [x] Never-seen guard -- heartbeat_count==0 never triggers
  - [x] Disabled guard -- config.enabled==false never triggers
  - [x] Mode exemption -- Hold/RTL/SmartRTL return `Activated` (not `SetMode`)
  - [x] Mode exemption STATUSTEXT -- `Activated` then `Cleared` on recovery (both notifications)
  - [x] Sticky trigger -- fires only once per activation
  - [x] Recovery -- returns Cleared on heartbeat resume
  - [x] Re-trigger after recovery -- new failsafe after clear and re-loss
  - [x] Arm transition reset -- reset clears state
  - [x] Rapid arm/disarm cycle -- arm → timeout starts → disarm → re-arm → state correctly reset
  - [x] Disarm action fallback -- `FS_ACTION=3` maps to Hold in `GcsFailsafeConfig::from_params()`
  - [x] Clock non-monotonicity -- `current_time_us < last_heartbeat_us` does not false-trigger
- [x] **Unit tests for `MavlinkLoopRunner` GCS integration**
  - [x] Normal heartbeats through runner -- no action
  - [x] Timeout through runner -- returns SetMode
  - [x] Arm transition resets GCS failsafe
  - [x] Config reload from param store on arm
- [x] **Verification**
  - [x] `cargo test -p pico_trail_core --lib --quiet`
  - [x] `cargo test -p pico_trail_core --features embassy --lib --quiet`
  - [x] `cargo clippy -p pico_trail_core --all-targets -- -D warnings`
  - [x] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- `GcsFailsafeChecker` with full two-stage detection logic
- `MavlinkLoopRunner` extended with GCS failsafe
- Comprehensive unit tests

### Acceptance Criteria (Phase Gate)

- GCS failsafe triggers after `FS_GCS_TIMEOUT + FS_TIMEOUT`
- All guards work (armed, never-seen, disabled, mode exemption)
- Mode exemption returns `Activated` (caller can send STATUSTEXT without mode change)
- `FS_ACTION=3` (Disarm) safely falls back to Hold for GCS failsafe
- Recovery returns `Cleared` on heartbeat resume
- All tests pass, clippy clean, embedded build succeeds

### Rollback/Fallback

- Remove `gcs_failsafe.rs` and revert `MavlinkLoopRunner` changes; battery failsafe unaffected

---

## Phase 3: Firmware Cleanup, STATUSTEXT Notifications, and SITL Integration

### Goal

- Remove duplicate GCS heartbeat tracking from firmware `ArmedStateMonitor`
- Add STATUSTEXT notifications for failsafe activation and recovery
- Wire GCS failsafe check in SITL control loop
- SITL end-to-end integration tests

**Phase 2/3 boundary rationale**: Phase 2 focuses on core crate logic (checker + runner) with host-only unit tests. Phase 3 handles cross-crate integration (firmware cleanup, SITL wiring) and runtime behavior (STATUSTEXT) that depend on the full runner being available. This separation allows Phase 2 to be reviewed and tested in isolation before touching firmware or SITL code.

### Inputs

- Source Code to Read:
  - `crates/firmware/src/core/arming/monitoring.rs` -- `ArmedStateMonitor` GCS fields
  - `crates/core/src/communication/status_notifier.rs` -- notification API
  - `crates/sitl/src/autopilot.rs` -- SITL control loop
- Dependencies:
  - Phase 2: `GcsFailsafeChecker` and `MavlinkLoopRunner` integration complete

### Tasks

- [x] **Remove GCS tracking from `ArmedStateMonitor`**
  - [x] Remove `gcs_last_heartbeat_ms` field
  - [x] Remove GCS-related logic in `update_slow()` (the TODO for failsafe action)
  - [x] Remove any `FailsafeReason::GcsLoss` references if present
  - [x] Update tests if affected
- [x] **Add STATUSTEXT notifications in caller**
  - [x] On `GcsFailsafeAction::SetMode(mode)`: call `send_warning()` with "Failsafe: GCS Lost - Hold" or "- RTL"
  - [x] On `GcsFailsafeAction::Activated`: call `send_warning()` with "Failsafe: GCS Lost - already in {mode}" (mode exempt, no mode change)
  - [x] On `GcsFailsafeAction::Cleared`: call `send_info()` with "GCS Failsafe Cleared"
  - [x] Add `log_warn!` / `log_info!` calls for local logging
- [x] **Wire GCS failsafe in SITL control loop**
  - [x] Add `check_gcs_failsafe()` call in SITL's autopilot loop (if applicable)
  - [x] Pass `ConnectionState` data from dispatcher to runner
- [x] **SITL integration tests**
  - [x] Heartbeat loss E2E: stop heartbeat → wait `FS_GCS_TIMEOUT + FS_TIMEOUT` → verify mode changed to Hold
  - [x] Heartbeat recovery E2E: resume heartbeat after failsafe → verify failsafe cleared
- [x] **Verification**
  - [x] `cargo test -p pico_trail_core --lib --quiet`
  - [x] `cargo test -p pico_trail_core --features embassy --lib --quiet`
  - [x] `cargo test -p pico_trail_firmware --lib --quiet`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- Clean `ArmedStateMonitor` without duplicate GCS tracking
- STATUSTEXT notifications on failsafe events (including `Activated` for mode-exempt cases)
- SITL integration with GCS failsafe
- SITL E2E integration tests

### Acceptance Criteria (Phase Gate)

- `ArmedStateMonitor` has no GCS heartbeat fields
- STATUSTEXT messages queued on failsafe activation (both `SetMode` and `Activated`) and recovery
- SITL E2E test: heartbeat loss → Hold mode → heartbeat resume → failsafe cleared
- All crate tests pass
- Embedded build succeeds

### Rollback/Fallback

- Restore `ArmedStateMonitor` fields; notifications are additive and can be removed

---

## Definition of Done

- [x] `FailsafeAction` enum matches ArduPilot numbering
- [x] `FS_GCS_TIMEOUT` and `FS_TIMEOUT` registered with correct defaults
- [x] Invalid `FS_ACTION` values fall back to Hold; `FS_ACTION=3` (Disarm) mapped to Hold for GCS failsafe
- [x] `GcsFailsafeChecker` fully tested (guards, timeout, recovery, mode exemption `Activated`, edge cases)
- [x] `MavlinkLoopRunner` integrates GCS failsafe alongside battery
- [x] Firmware `ArmedStateMonitor` cleaned of GCS tracking
- [x] STATUSTEXT notifications sent on failsafe events (`SetMode`, `Activated`, `Cleared`)
- [x] SITL E2E integration test passes (heartbeat loss → Hold → heartbeat resume → cleared)
- [x] Core tests pass: `cargo test -p pico_trail_core --features embassy --lib`
- [x] Core (no embassy) passes: `cargo test -p pico_trail_core --lib`
- [x] Firmware tests pass: `cargo test -p pico_trail_firmware --lib`
- [x] Embedded build: `./scripts/build-rp2350.sh pico_trail_rover`
- [x] Clippy clean: `cargo clippy --all-targets -- -D warnings`
- [x] Code formatted: `cargo fmt`

## Open Questions

- [x] Should GCS failsafe interact with battery failsafe priority? --> Defer to unified framework. Current behavior: both default to Hold (no conflict). If configured differently (e.g., GCS=RTL + Battery=Hold), last-write-wins via `set_mode()`. This is safe because all supported actions are safe modes. See design.md "Concurrent Failsafe Behavior" for details
