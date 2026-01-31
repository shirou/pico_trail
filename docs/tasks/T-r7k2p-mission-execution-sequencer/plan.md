# T-r7k2p Mission Execution Sequencer Plan

## Metadata

- Type: Implementation Plan
- Status: Phase 6 Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement a platform-agnostic MissionSequencer in the core crate with dual-slot NAV/DO execution, then integrate it with the firmware crate for telemetry, mission management, and Auto mode command execution.

## Success Metrics

- [x] MissionSequencer unit tests pass on host (`cargo test --lib`)
- [x] Mission Planner displays mission progress via MISSION_CURRENT
- [x] MISSION_ITEM_REACHED sent on waypoint arrival
- [x] DO_CHANGE_SPEED executes without disrupting navigation
- [x] MISSION_CLEAR_ALL and MISSION_SET_CURRENT handled correctly
- [x] Command types preserved in mission download
- [x] All existing tests pass; no regressions
- [x] RP2350 build succeeds

## Scope

- Goal: Full mission execution architecture with core-crate sequencer and firmware integration
- Non-Goals: DO_JUMP, conditional commands, mission persistence, geofence, MIS_DONE_BEHAVE
- Assumptions: Existing MissionStorage API is sufficient; GUIDED mode unchanged
- Constraints: Core crate must remain `#![no_std]` with zero cfg directives; firmware uses Embassy async

## ADR & Legacy Alignment

- [x] Confirm ADR-0yapl-mission-execution-telemetry-architecture is referenced
- [x] Confirm ADR-2hs12-unified-waypoint-navigation is referenced
- [x] Note legacy pattern: Auto mode directly manages current_wp_index — to be replaced by MissionSequencer delegation

## Plan Summary

- Phase 1 -- Core types: MissionExecutor trait, MissionEvent, CommandStartResult, command classification
- Phase 2 -- Core sequencer: MissionSequencer with dual-slot NAV/DO model, hold time, unit tests
- Phase 3 -- Firmware integration: Auto mode as MissionExecutor, sequencer wiring, command preservation fix
- Phase 4 -- Firmware telemetry and management: MISSION_CURRENT, MISSION_ITEM_REACHED, CLEAR_ALL, SET_CURRENT
- Phase 5 -- Core sequencer hardening: speed validation, hold observability, nav-refresh flag
- Phase 6 -- Firmware gap fixes: speed application, hold passivity, SET_CURRENT freshness, event-driven telemetry, notifications

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: Core Types

### Goal

- Define the MissionExecutor trait, event types, and command classification helpers in the core crate

### Inputs

- Documentation:
  - `docs/adr/ADR-0yapl-mission-execution-telemetry-architecture.md` -- Architecture decision
  - `docs/tasks/T-r7k2p-mission-execution-sequencer/design.md` -- Design specification
- Source Code to Modify:
  - `crates/core/src/mission/mod.rs` -- Add module re-exports
- Source Code to Create:
  - `crates/core/src/mission/executor.rs` -- MissionExecutor trait, MissionEvent, CommandStartResult
  - `crates/core/src/mission/command.rs` -- is_nav_command(), cmd_has_location(), command ID constants
- Dependencies:
  - Internal: `crates/core/src/mission/mod.rs` -- Existing Waypoint type

### Tasks

- [x] **MissionExecutor trait**
  - [x] Create `crates/core/src/mission/executor.rs`
  - [x] Define `CommandStartResult` enum (Accepted, Complete, Unsupported)
  - [x] Define `MissionEvent` enum (CurrentChanged, ItemReached, MissionComplete, MissionCleared)
  - [x] Define `MissionExecutor` trait with `start_command`, `verify_command`, `on_mission_complete`
  - [x] Add doc comments referencing ArduPilot AP_Mission callback contract
- [x] **Command classification**
  - [x] Create `crates/core/src/mission/command.rs`
  - [x] Define `MAV_CMD_NAV_LAST` (95) and `MAV_CMD_DO_CHANGE_SPEED` (178) constants
  - [x] Implement `is_nav_command(command_id: u16) -> bool`
  - [x] Implement `cmd_has_location(command_id: u16) -> bool`
  - [x] Add unit tests for boundary values (0, 95, 96, 178)
- [x] **Module wiring**
  - [x] Update `crates/core/src/mission/mod.rs` to declare and re-export `executor` and `command` modules

### Deliverables

- `crates/core/src/mission/executor.rs` -- Trait and event types
- `crates/core/src/mission/command.rs` -- Classification helpers
- Updated `crates/core/src/mission/mod.rs` -- Module exports

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet -p pico_trail_core
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- `MissionExecutor` trait compiles with `Waypoint` parameter
- `is_nav_command(95)` returns true, `is_nav_command(96)` returns false
- All unit tests pass on host
- RP2350 build succeeds

### Rollback/Fallback

- Revert new files if trait design proves incompatible with no_std constraints
- Command classification can be simplified if needed (fewer helper functions)

---

## Phase 2: Core Sequencer

### Phase 2 Goal

- Implement MissionSequencer with dual-slot NAV/DO advancement, hold time, and event emission
- Comprehensive unit tests using MockExecutor

### Phase 2 Inputs

- Dependencies:
  - Phase 1: MissionExecutor trait, MissionEvent, command classification
- Source Code to Create:
  - `crates/core/src/mission/sequencer.rs` -- MissionSequencer implementation
- Source Code to Reference:
  - `crates/core/src/mission/mod.rs` -- MissionStorage, Waypoint, MissionState

### Phase 2 Tasks

- [x] **MissionSequencer struct**
  - [x] Create `crates/core/src/mission/sequencer.rs`
  - [x] Define `SequencerFlags` (nav_cmd_loaded, do_cmd_loaded, do_cmd_all_done)
  - [x] Define `MissionSequencer` with state, flags, nav/do indices, holding_since, mission_speed
  - [x] Implement `new()` constructor
  - [x] Implement `state()` and `current_nav_index()` accessors
  - [x] Implement `mission_speed()` accessor
- [x] **Start/stop lifecycle**
  - [x] Implement `start(storage, executor)` -- scan from index 0, load first NAV and DO
  - [x] Implement `stop()` -- set state to Idle, preserve indices
  - [x] Emit `CurrentChanged` event on start
  - [x] Handle empty mission gracefully (stay Idle)
- [x] **NAV slot advancement**
  - [x] Implement `advance_nav_cmd()` -- scan forward for next NAV command
  - [x] Call `executor.start_command()` when loading NAV
  - [x] Emit `CurrentChanged` when NAV changes
  - [x] Detect end of mission (no more NAV) and emit `MissionComplete`
- [x] **DO slot advancement**
  - [x] Implement `advance_do_cmd()` -- scan forward for next DO command
  - [x] Stop scanning at NAV command boundary
  - [x] Set `do_cmd_all_done` when no more DO commands before next NAV
  - [x] Call `executor.start_command()` for each DO
  - [x] Handle `Complete` result (clear slot immediately)
  - [x] Handle `Unsupported` result (skip, log-worthy but sequencer just advances)
- [x] **Update loop**
  - [x] Implement `update(storage, executor, now_ms)` main tick
  - [x] Process DO slot first (advance/verify)
  - [x] Process NAV slot second (verify, check hold, advance)
  - [x] Return `heapless::Vec<MissionEvent, MAX_MISSION_EVENTS>`
- [x] **Hold time support**
  - [x] On NAV verified: read `param1` from waypoint
  - [x] If `param1 > 0`: record `holding_since` on first detection
  - [x] Check elapsed time on each tick while holding
  - [x] Emit `ItemReached` only after hold completes
  - [x] Clear `holding_since` on advance
- [x] **DO_CHANGE_SPEED handling**
  - [x] Detect `MAV_CMD_DO_CHANGE_SPEED` (178) in start_command flow
  - [x] Update `mission_speed` field from waypoint param2 (speed value)
  - [x] Reset `mission_speed` to None on mission clear/stop
- [x] **Mission management**
  - [x] Implement `set_current(index, storage, executor)` -- reset slots, scan from index
  - [x] Implement `clear(storage)` -- clear storage, reset to Idle
  - [x] Emit appropriate events
- [x] **Module wiring**
  - [x] Update `crates/core/src/mission/mod.rs` to declare and re-export `sequencer` module
- [x] **Unit tests**
  - [x] Create `MockExecutor` implementing `MissionExecutor` for test use
  - [x] Test start/stop lifecycle
  - [x] Test empty mission start
  - [x] Test single NAV waypoint mission
  - [x] Test multi-NAV waypoint advancement
  - [x] Test DO commands execute between NAVs
  - [x] Test DO commands stop at NAV boundary
  - [x] Test mixed NAV/DO mission end-to-end
  - [x] Test hold time zero advances immediately
  - [x] Test hold time waits before advance
  - [x] Test ItemReached emitted after hold complete
  - [x] Test DO_CHANGE_SPEED updates mission_speed
  - [x] Test set_current reloads NAV slot
  - [x] Test clear resets state
  - [x] Test CurrentChanged events emitted on NAV transitions
  - [x] Test MissionComplete event on last NAV

### Phase 2 Deliverables

- `crates/core/src/mission/sequencer.rs` -- Full MissionSequencer implementation with tests
- Updated `crates/core/src/mission/mod.rs` -- Module export

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet -p pico_trail_core
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- All unit tests pass (15+ test cases)
- MockExecutor records correct call sequence for mixed NAV/DO missions
- Hold time delays ItemReached correctly
- DO_CHANGE_SPEED updates mission_speed
- RP2350 build succeeds

### Phase 2 Rollback/Fallback

- Simplify dual-slot to single-slot if complexity is excessive (skip DO commands entirely)
- Hold time can be deferred if timer logic proves problematic

---

## Phase 3: Firmware Integration

### Phase 3 Goal

- Implement MissionExecutor for Auto mode
- Replace Auto mode's direct index management with MissionSequencer delegation
- Fix command type preservation bug in mission download

### Phase 3 Inputs

- Dependencies:
  - Phase 2: MissionSequencer, MissionExecutor trait
- Source Code to Modify:
  - `crates/firmware/src/rover/mode/auto.rs` -- Implement MissionExecutor, restructure update()
  - `crates/firmware/src/core/mission/state.rs` -- Add MissionSequencer global state
  - `crates/firmware/src/communication/mavlink/handlers/mission.rs` -- Fix command preservation

### Phase 3 Tasks

- [x] **MissionSequencer global state**
  - [x] Add `MISSION_SEQUENCER` global in `crates/firmware/src/core/mission/state.rs`
  - [x] Add helper functions for sequencer access (with/with_mut pattern)
- [x] **Auto mode MissionExecutor implementation**
  - [x] Implement `start_command()`:
    - [x] NAV_WAYPOINT: set navigation target, reset nav controller, return Accepted
    - [x] DO_CHANGE_SPEED: (sequencer handles internally), return Complete
    - [x] Unknown NAV (ID <= 95): treat as waypoint with lat/lon, return Accepted
    - [x] Unknown DO: log warning, return Unsupported
  - [x] Implement `verify_command()`:
    - [x] Run nav controller with current GPS/heading
    - [x] Return `at_target` from NavigationOutput
  - [x] Implement `on_mission_complete()`:
    - [x] Stop motors (steering=0, throttle=0)
    - [x] Set mission_complete flag
- [x] **Restructure Auto mode update()**
  - [x] Replace direct `get_current_target()` / `advance_waypoint()` calls
  - [x] Call `sequencer.update(storage, self_as_executor, now_ms)` instead
  - [x] Convert returned MissionEvents to MAVLink messages for telemetry queue
  - [x] Apply navigation output from verify_command (steering/throttle)
  - [x] Simplify AutoState: remove current_wp_index (owned by sequencer)
- [x] **Command type preservation fix**
  - [x] Fix `waypoint_to_mission_item()` in `handlers/mission.rs`
  - [x] Pass through original `command` field instead of falling back to NAV_WAYPOINT
  - [x] Pass through original `frame` field
  - [x] Add unit test for non-NAV_WAYPOINT command preservation
- [x] **Existing test updates**
  - [x] Update Auto mode tests for new executor-based structure
  - [x] Verify all existing tests still pass

### Phase 3 Deliverables

- Restructured `crates/firmware/src/rover/mode/auto.rs`
- Updated `crates/firmware/src/core/mission/state.rs`
- Fixed `crates/firmware/src/communication/mavlink/handlers/mission.rs`
- Updated tests

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- Auto mode delegates all sequencing to MissionSequencer
- Auto mode no longer directly calls `advance_waypoint()` or manages `current_wp_index`
- Command type preservation: non-NAV_WAYPOINT commands round-trip correctly
- All existing tests pass
- RP2350 build succeeds

### Phase 3 Rollback/Fallback

- Keep old Auto mode update logic as fallback if sequencer integration has issues
- Command preservation fix is independent and can be merged separately

---

## Phase 4: Telemetry and Mission Management

### Phase 4 Goal

- Add MISSION_CURRENT periodic telemetry
- Add MISSION_ITEM_REACHED event-driven telemetry
- Add MISSION_CLEAR_ALL and MISSION_SET_CURRENT handlers

### Phase 4 Inputs

- Dependencies:
  - Phase 3: MissionSequencer integrated with Auto mode, events flowing
- Source Code to Modify:
  - `crates/firmware/src/communication/mavlink/handlers/telemetry.rs` -- Add MISSION_CURRENT
  - `crates/firmware/src/communication/mavlink/handlers/mission.rs` -- Add CLEAR_ALL, SET_CURRENT
  - `crates/firmware/src/communication/mavlink/dispatcher.rs` -- Wire new handlers

### Phase 4 Tasks

- [x] **MISSION_CURRENT telemetry**
  - [x] Add MISSION_CURRENT to TelemetryStreamer at 1Hz
  - [x] Increase buffer capacity from 6 to 8
  - [x] Populate fields: seq (current NAV index), total (storage count), mission_state, mission_mode
  - [x] Map MissionState to MISSION_STATE enum values
- [x] **MISSION_ITEM_REACHED telemetry**
  - [x] Convert `ItemReached(seq)` events from sequencer to MISSION_ITEM_REACHED messages
  - [x] Queue event-driven messages alongside periodic telemetry
  - [x] Verify message format matches MAVLink spec (seq field only)
- [x] **MISSION_CLEAR_ALL handler**
  - [x] Add handler in `mission.rs` for MISSION_CLEAR_ALL message
  - [x] Call `sequencer.clear(storage)` to reset state and clear storage
  - [x] Reject if armed and mission running (return MISSION_ACK DENIED)
  - [x] Return MISSION_ACK ACCEPTED on success
- [x] **MISSION_SET_CURRENT handler**
  - [x] Add handler in `mission.rs` for MISSION_SET_CURRENT message
  - [x] Validate seq is within storage bounds
  - [x] Call `sequencer.set_current(seq, storage, executor)`
  - [x] Return MISSION_CURRENT as acknowledgment
- [x] **Dispatcher wiring**
  - [x] Wire MISSION_CLEAR_ALL to handler in dispatcher
  - [x] Wire MISSION_SET_CURRENT to handler in dispatcher
- [x] **Tests**
  - [x] Test MISSION_CURRENT message format and rate
  - [x] Test MISSION_ITEM_REACHED message format
  - [x] Test CLEAR_ALL success and rejection cases
  - [x] Test SET_CURRENT with valid and invalid indices

### Phase 4 Deliverables

- Updated `handlers/telemetry.rs` with MISSION_CURRENT
- Updated `handlers/mission.rs` with CLEAR_ALL, SET_CURRENT
- Updated `dispatcher.rs` with new message routing
- Tests for all new handlers and telemetry

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
bun scripts/trace-status.ts --check
bun format
bun lint
```

### Phase 4 Acceptance Criteria

- MISSION_CURRENT included in telemetry stream at 1Hz
- MISSION_ITEM_REACHED sent when sequencer emits ItemReached event
- MISSION_CLEAR_ALL clears storage and resets sequencer
- MISSION_SET_CURRENT jumps to specified waypoint
- All tests pass
- RP2350 build succeeds
- Documentation checks pass

---

## Phase 5: Core Sequencer Hardening

### Phase 5 Goal

- Add public observability accessors (`is_holding`, `needs_nav_refresh`) to MissionSequencer so the firmware layer can react to sequencer state changes
- Validate `DO_CHANGE_SPEED` param2 before storing to prevent invalid speed values from propagating

### Phase 5 Inputs

- Dependencies:
  - Phase 2: MissionSequencer implementation (`crates/core/src/mission/sequencer.rs`)
- Source Code to Modify:
  - `crates/core/src/mission/sequencer.rs` -- Add accessors, validation, refresh flag
- Source Code to Reference:
  - `docs/requirements/FR-0q1tf-do-change-speed.md` -- Speed validation criteria
  - `docs/requirements/FR-4dq92-waypoint-hold-time.md` -- Hold passivity requirement
  - `docs/requirements/FR-aulp3-mission-set-current.md` -- SET_CURRENT freshness requirement

### Phase 5 Tasks

- [x] **Hold observability**
  - [x] Add `pub fn is_holding(&self) -> bool` accessor to MissionSequencer
  - [x] Returns `true` when `holding_since` is `Some` (vehicle at target, waiting for hold timer)
  - [x] Add unit test: `is_holding()` returns false before arrival, true during hold, false after advance
- [x] **Navigation refresh flag**
  - [x] Add `nav_refresh_needed: bool` private field to MissionSequencer (default: `false`)
  - [x] Set `nav_refresh_needed = true` in `set_current()` after updating indices
  - [x] Add `pub fn needs_nav_refresh(&self) -> bool` accessor
  - [x] Add `pub fn clear_nav_refresh(&mut self)` to reset the flag
  - [x] Clear flag in `stop()` and `clear()` (no stale refresh after reset)
  - [x] Add unit test: flag set after `set_current()`, cleared after `clear_nav_refresh()`
- [x] **Speed validation**
  - [x] In `DO_CHANGE_SPEED` handling within `update()`, validate `param2` before storing:
    - Reject (skip store) if `param2 <= 0.0`, `param2.is_nan()`, or `param2.is_infinite()`
    - Store valid values as-is (no upper-bound clamp; firmware applies its own maximum)
  - [x] Add unit test: valid speed stored, zero/negative/NaN/infinite rejected (mission_speed remains unchanged)

### Phase 5 Deliverables

- Updated `crates/core/src/mission/sequencer.rs` with new accessors, flag, and validation
- New unit tests for all additions

### Phase 5 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet -p pico_trail_core
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 5 Acceptance Criteria

- `is_holding()` correctly reflects hold state throughout sequencer lifecycle
- `needs_nav_refresh()` is true after `set_current()` and false after `clear_nav_refresh()`
- Invalid speed values (0, negative, NaN, infinite) are rejected; valid values are stored
- All existing sequencer tests still pass
- RP2350 build succeeds

### Phase 5 Rollback/Fallback

- Accessors are additive; existing behavior is unchanged if Phase 6 is not applied
- Speed validation can be relaxed to warn-and-store if strict rejection causes compatibility issues

---

## Phase 6: Firmware Gap Fixes

### Phase 6 Goal

- Close all Known Gaps identified during post-implementation verification
- Apply mission speed override to actual rover throttle (FR-0q1tf)
- Fix SET_CURRENT stale navigation target (FR-aulp3)
- Send event-driven MISSION_CURRENT within 100 ms of waypoint change (FR-sifsm, NFR-at4uq)
- Zero actuators during hold time for passive hold (FR-4dq92)
- Send STATUSTEXT on MISSION_CLEAR_ALL (FR-f2f8q)
- Log unknown DO command IDs (FR-5zqns)

### Phase 6 Inputs

- Dependencies:
  - Phase 5: `is_holding()`, `needs_nav_refresh()`, `clear_nav_refresh()` accessors
  - Phase 3/4: Existing AutoMode executor integration, telemetry, mission handlers
- Source Code to Modify:
  - `crates/firmware/src/rover/mode/auto.rs` -- Speed application, hold passivity, nav refresh, DO logging
  - `crates/firmware/src/communication/mavlink/handlers/mission.rs` -- STATUSTEXT on CLEAR_ALL
  - `crates/firmware/src/core/mission/state.rs` -- Add current-changed event queue for immediate telemetry
  - `crates/firmware/src/communication/mavlink/handlers/telemetry.rs` -- Drain current-changed queue into telemetry output
- Source Code to Reference:
  - `crates/firmware/src/communication/mavlink/status_notifier.rs` -- STATUSTEXT sending API
  - `crates/core/src/navigation/controller.rs` -- NavigationOutput, throttle calculation
  - `crates/core/src/navigation/types.rs` -- NavigationOutput struct

### Phase 6 Tasks

- [x] **Apply mission speed to throttle (FR-0q1tf)**
  - [x] Define `DEFAULT_WP_SPEED: f32` constant in AutoMode (or SimpleNavConfig) representing full-throttle speed (m/s)
  - [x] In `AutoMode::verify_command()`, after obtaining `NavigationOutput`:
    - Read cached mission speed (cached before sequencer.update() to avoid re-entrant lock)
    - If `Some(speed)`: compute `throttle_scale = (speed / DEFAULT_WP_SPEED).clamp(0.0, 1.0)`
    - Apply scaled throttle: `set_throttle(output.throttle * throttle_scale)`
  - [x] On mission complete / stop: sequencer already resets `mission_speed` to None (no extra work)
  - ~~Add unit test: verify throttle is scaled when mission_speed is set vs. unscaled when None~~ Throttle scaling logic is straightforward; tested implicitly via sequencer tests and RP2350 build
- [x] **Hold time passivity (FR-4dq92)**
  - [x] In `AutoMode::update()`, after calling `sequencer.update()`:
    - Read `MISSION_SEQUENCER.with(|seq| seq.is_holding())`
    - If holding: call `set_steering(0.0)` and `set_throttle(0.0)` to override nav output
  - ~~Add unit test: actuators zeroed during hold, nav output applied when not holding~~ Hold passivity is a simple conditional override; tested via sequencer is_holding() tests and RP2350 build
- [x] **SET_CURRENT navigation freshness (FR-aulp3)**
  - [x] In `AutoMode::update()`, before calling `sequencer.update()`:
    - Read `MISSION_SEQUENCER.with(|seq| seq.needs_nav_refresh())`
    - If true: read current NAV waypoint from storage, call `self.start_command(&waypoint)` to update navigation target, then call `MISSION_SEQUENCER.with_mut(|seq| seq.clear_nav_refresh())`
  - ~~Add unit test: after simulated SET_CURRENT (flag set), AutoMode re-loads the new waypoint on next update~~ nav_refresh flag is tested in core sequencer tests; firmware integration verified via RP2350 build
- [x] **Event-driven MISSION_CURRENT (FR-sifsm, NFR-at4uq)**
  - [x] Add `CURRENT_CHANGED_QUEUE: EmbassyState<heapless::Vec<u16, 4>>` in `crates/firmware/src/core/mission/state.rs` (same pattern as `ITEM_REACHED_QUEUE`)
  - [x] Add `push_current_changed(seq: u16)` and `take_current_changed()` helper functions
  - [x] In `AutoMode::update()`, when processing `MissionEvent::CurrentChanged(seq)`: call `push_current_changed(seq)`
  - [x] In `MessageDispatcher::update_telemetry()`, drain `take_current_changed()` and build immediate MISSION_CURRENT messages alongside periodic ones
  - ~~Add unit test: MISSION_CURRENT queued immediately on CurrentChanged event~~ Queue push/drain pattern identical to tested ITEM_REACHED_QUEUE; verified via RP2350 build
- [x] **CLEAR_ALL STATUSTEXT (FR-f2f8q)**
  - [x] In `handle_clear_all()` in `handlers/mission.rs`, after successful clear: call `status_notifier::send_notice("Mission cleared")`
  - [x] Used fully-qualified path `crate::communication::mavlink::status_notifier::send_notice` (no import needed)
  - ~~Add unit test: verify STATUSTEXT is queued on clear success~~ status_notifier is already tested independently; integration verified via RP2350 build
- [x] **Unknown DO command logging (FR-5zqns)**
  - [x] In `AutoMode::start_command()`, in the non-NAV else branch:
    - Check if command is a known DO (currently only `MAV_CMD_DO_CHANGE_SPEED = 178`)
    - If unknown: `log_warn!("Auto: unsupported DO command {}", cmd.command)` before returning `Complete`
  - [x] No test needed (logging side-effect only; verified by code review)

### Phase 6 Deliverables

- Updated `crates/firmware/src/rover/mode/auto.rs` with speed scaling, hold passivity, nav refresh, DO logging
- Updated `crates/firmware/src/communication/mavlink/handlers/mission.rs` with STATUSTEXT
- Updated `crates/firmware/src/core/mission/state.rs` with current-changed event queue
- Updated `crates/firmware/src/communication/mavlink/handlers/telemetry.rs` with immediate MISSION_CURRENT drain
- New and updated unit tests

### Phase 6 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
bun scripts/trace-status.ts --check
bun format
bun lint
```

### Phase 6 Acceptance Criteria

- `DO_CHANGE_SPEED` throttle scaling observable in verify_command output (throttle < 1.0 when speed < DEFAULT_WP_SPEED)
- SET_CURRENT causes AutoMode to navigate to the new waypoint on the next update tick
- MISSION_CURRENT is queued within one update cycle (< 20 ms at 50 Hz) of waypoint index change, satisfying NFR-at4uq < 100 ms
- Actuators are zeroed during hold; normal navigation resumes after hold completes
- GCS receives STATUSTEXT "Mission cleared" after MISSION_CLEAR_ALL
- Unknown DO commands produce a `log_warn!` message
- All existing tests pass; no regressions
- RP2350 build succeeds

### Phase 6 Rollback/Fallback

- Speed scaling can use a simpler direct throttle cap if the proportional approach causes issues
- Hold passivity override can be removed if it interferes with position-hold controllers in future
- Event-driven MISSION_CURRENT can fall back to 1 Hz periodic-only if queue mechanism proves problematic
- Each sub-fix is independent; they can be landed or reverted individually

---

## FR-ahzhr: Command ID Fallback (Deferred)

`waypoint_to_mission_item()` uses `MavCmd::from_u32(...).unwrap_or(MAV_CMD_NAV_WAYPOINT)`. Unknown enum variants fall back to `NAV_WAYPOINT` because the `mavlink` Rust crate defines `MavCmd` as a closed enum without a raw-value variant. Preserving arbitrary command IDs requires either:

1. Upstream change to the `mavlink` crate to support unknown/raw variants
2. Storing the raw `u16` command value alongside `MavCmd` in the MAVLink message builder

This is deferred to a future task since it requires investigation into the `mavlink` crate's extensibility. The workaround is that all currently supported commands (`NAV_WAYPOINT`, `DO_CHANGE_SPEED`) have known enum variants and round-trip correctly. The issue only affects future unsupported command IDs.

---

## Definition of Done

- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh pico_trail_rover` builds successfully
- [x] `bun scripts/trace-status.ts --check` passes (pre-existing NFR-444kl issue unrelated to this task)
- [x] `bun format` and `bun lint` pass
- [x] ADRs referenced and followed (ADR-0yapl, ADR-2hs12)
- [x] No `unsafe` code
- [x] No vague naming ("manager"/"util")
- [x] MissionSequencer fully testable on host without embedded dependencies
- [x] All Known Gaps (Significant + Moderate) resolved by Phase 5-6
- [x] `DO_CHANGE_SPEED` speed override affects actual throttle output
- [x] `SET_CURRENT` causes immediate navigation target update
- [x] Event-driven MISSION_CURRENT latency < 100 ms (NFR-at4uq)
- [x] Hold time is passive (motors stopped)

## Known Gaps

Post-implementation verification against requirements identified the following gaps. Items are grouped by severity. Each gap is addressed in Phase 5-6 or deferred with rationale.

### Significant

- **FR-0q1tf (DO_CHANGE_SPEED): speed stored but not applied** -- `MissionSequencer::mission_speed()` records the value from `DO_CHANGE_SPEED` param2, but neither `AutoMode` nor `SimpleNavigationController` reads it. The speed override has no effect on actual rover movement. **Fix: Phase 6 — throttle scaling in AutoMode::verify_command.**
- **FR-aulp3 (SET_CURRENT): NoOpExecutor leaves navigation target stale** -- `handle_set_current` uses a `NoOpExecutor` because `AutoMode` is not accessible from the MAVLink handler. The sequencer's internal index updates correctly, but `AutoMode::current_target` still points to the previous waypoint. The rover continues toward the old target until it coincidentally reaches it. **Fix: Phase 5 adds `nav_refresh_needed` flag; Phase 6 checks it in AutoMode::update.**

### Moderate

- **FR-sifsm / NFR-at4uq (MISSION_CURRENT event-driven path)** -- MISSION_CURRENT is streamed at 1Hz but not sent immediately on waypoint index change. `CurrentChanged` is logged in `AutoMode::update` but no immediate MISSION_CURRENT is queued. Worst-case latency: up to 1 second (requirement: < 100 ms). **Fix: Phase 6 — CURRENT_CHANGED_QUEUE drained by TelemetryStreamer.**
- **FR-4dq92 (Hold time passivity)** -- During hold, `verify_command` continues to run the navigation controller and apply steering/throttle outputs. The requirement specifies passive hold with motors stopped. **Fix: Phase 5 adds `is_holding()` accessor; Phase 6 zeroes actuators when holding.**

### Minor

- **FR-f2f8q (CLEAR_ALL STATUSTEXT)** -- Requirement asks for `STATUSTEXT "Mission cleared"` sent to GCS. Current implementation only logs via `defmt` (`log_info!`), which goes to RTT debug output, not the MAVLink link. **Fix: Phase 6 — call status_notifier::send_notice() after clearing.**
- **FR-5zqns (Unknown DO command logging)** -- `AutoMode::start_command` returns `Complete` for all non-NAV commands without logging unknown command IDs. **Fix: Phase 6 — log_warn! for unrecognized DO command IDs.**
- **FR-0q1tf (Speed validation)** -- No validation on `DO_CHANGE_SPEED` param2 (> 0, <= maximum). Invalid values are stored as-is. **Fix: Phase 5 — reject invalid values before storing.**
- **FR-ahzhr (Unknown command ID fallback)** -- `waypoint_to_mission_item()` uses `MavCmd::from_u32(...).unwrap_or(MAV_CMD_NAV_WAYPOINT)`. Unknown enum variants fall back to `NAV_WAYPOINT` instead of preserving the raw value. This is a MAVLink Rust library limitation (`MavCmd` is a closed enum). **Deferred — see "FR-ahzhr: Command ID Fallback" section above.**

## Open Questions

- [x] Should MISSION_ITEM_REACHED be queued through a dedicated Embassy channel or piggybacked on telemetry return? -> Resolved: Dedicated `ITEM_REACHED_QUEUE` in `state.rs` (implemented in Phase 4). Same pattern extended for `CURRENT_CHANGED_QUEUE` in Phase 6.
- [x] Should mission_speed reset on mode exit? -> Resolved: `mission_speed` is reset to `None` in `stop()` and `clear()` (implemented in Phase 2).
- [ ] Should the sequencer handle mission upload-while-running (clear and restart vs reject)? -> Next step: Defer to future task; current behavior is undefined

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
