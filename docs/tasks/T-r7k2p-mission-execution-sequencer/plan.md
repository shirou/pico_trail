# T-r7k2p Mission Execution Sequencer Plan

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement a platform-agnostic MissionSequencer in the core crate with dual-slot NAV/DO execution, then integrate it with the firmware crate for telemetry, mission management, and Auto mode command execution.

## Success Metrics

- [ ] MissionSequencer unit tests pass on host (`cargo test --lib`)
- [ ] Mission Planner displays mission progress via MISSION_CURRENT
- [ ] MISSION_ITEM_REACHED sent on waypoint arrival
- [ ] DO_CHANGE_SPEED executes without disrupting navigation
- [ ] MISSION_CLEAR_ALL and MISSION_SET_CURRENT handled correctly
- [ ] Command types preserved in mission download
- [ ] All existing tests pass; no regressions
- [ ] RP2350 build succeeds

## Scope

- Goal: Full mission execution architecture with core-crate sequencer and firmware integration
- Non-Goals: DO_JUMP, conditional commands, mission persistence, geofence, MIS_DONE_BEHAVE
- Assumptions: Existing MissionStorage API is sufficient; GUIDED mode unchanged
- Constraints: Core crate must remain `#![no_std]` with zero cfg directives; firmware uses Embassy async

## ADR & Legacy Alignment

- [ ] Confirm ADR-0yapl-mission-execution-telemetry-architecture is referenced
- [ ] Confirm ADR-2hs12-unified-waypoint-navigation is referenced
- [ ] Note legacy pattern: Auto mode directly manages current_wp_index — to be replaced by MissionSequencer delegation

## Plan Summary

- Phase 1 -- Core types: MissionExecutor trait, MissionEvent, CommandStartResult, command classification
- Phase 2 -- Core sequencer: MissionSequencer with dual-slot NAV/DO model, hold time, unit tests
- Phase 3 -- Firmware integration: Auto mode as MissionExecutor, sequencer wiring, command preservation fix
- Phase 4 -- Firmware telemetry and management: MISSION_CURRENT, MISSION_ITEM_REACHED, CLEAR_ALL, SET_CURRENT

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

- [ ] **MissionExecutor trait**
  - [ ] Create `crates/core/src/mission/executor.rs`
  - [ ] Define `CommandStartResult` enum (Accepted, Complete, Unsupported)
  - [ ] Define `MissionEvent` enum (CurrentChanged, ItemReached, MissionComplete, MissionCleared)
  - [ ] Define `MissionExecutor` trait with `start_command`, `verify_command`, `on_mission_complete`
  - [ ] Add doc comments referencing ArduPilot AP_Mission callback contract
- [ ] **Command classification**
  - [ ] Create `crates/core/src/mission/command.rs`
  - [ ] Define `MAV_CMD_NAV_LAST` (95) and `MAV_CMD_DO_CHANGE_SPEED` (178) constants
  - [ ] Implement `is_nav_command(command_id: u16) -> bool`
  - [ ] Implement `cmd_has_location(command_id: u16) -> bool`
  - [ ] Add unit tests for boundary values (0, 95, 96, 178)
- [ ] **Module wiring**
  - [ ] Update `crates/core/src/mission/mod.rs` to declare and re-export `executor` and `command` modules

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

- [ ] **MissionSequencer struct**
  - [ ] Create `crates/core/src/mission/sequencer.rs`
  - [ ] Define `SequencerFlags` (nav_cmd_loaded, do_cmd_loaded, do_cmd_all_done)
  - [ ] Define `MissionSequencer` with state, flags, nav/do indices, holding_since, mission_speed
  - [ ] Implement `new()` constructor
  - [ ] Implement `state()` and `current_nav_index()` accessors
  - [ ] Implement `mission_speed()` accessor
- [ ] **Start/stop lifecycle**
  - [ ] Implement `start(storage, executor)` -- scan from index 0, load first NAV and DO
  - [ ] Implement `stop()` -- set state to Idle, preserve indices
  - [ ] Emit `CurrentChanged` event on start
  - [ ] Handle empty mission gracefully (stay Idle)
- [ ] **NAV slot advancement**
  - [ ] Implement `advance_nav_cmd()` -- scan forward for next NAV command
  - [ ] Call `executor.start_command()` when loading NAV
  - [ ] Emit `CurrentChanged` when NAV changes
  - [ ] Detect end of mission (no more NAV) and emit `MissionComplete`
- [ ] **DO slot advancement**
  - [ ] Implement `advance_do_cmd()` -- scan forward for next DO command
  - [ ] Stop scanning at NAV command boundary
  - [ ] Set `do_cmd_all_done` when no more DO commands before next NAV
  - [ ] Call `executor.start_command()` for each DO
  - [ ] Handle `Complete` result (clear slot immediately)
  - [ ] Handle `Unsupported` result (skip, log-worthy but sequencer just advances)
- [ ] **Update loop**
  - [ ] Implement `update(storage, executor, now_ms)` main tick
  - [ ] Process DO slot first (advance/verify)
  - [ ] Process NAV slot second (verify, check hold, advance)
  - [ ] Return `heapless::Vec<MissionEvent, MAX_MISSION_EVENTS>`
- [ ] **Hold time support**
  - [ ] On NAV verified: read `param1` from waypoint
  - [ ] If `param1 > 0`: record `holding_since` on first detection
  - [ ] Check elapsed time on each tick while holding
  - [ ] Emit `ItemReached` only after hold completes
  - [ ] Clear `holding_since` on advance
- [ ] **DO_CHANGE_SPEED handling**
  - [ ] Detect `MAV_CMD_DO_CHANGE_SPEED` (178) in start_command flow
  - [ ] Update `mission_speed` field from waypoint param2 (speed value)
  - [ ] Reset `mission_speed` to None on mission clear/stop
- [ ] **Mission management**
  - [ ] Implement `set_current(index, storage, executor)` -- reset slots, scan from index
  - [ ] Implement `clear(storage)` -- clear storage, reset to Idle
  - [ ] Emit appropriate events
- [ ] **Module wiring**
  - [ ] Update `crates/core/src/mission/mod.rs` to declare and re-export `sequencer` module
- [ ] **Unit tests**
  - [ ] Create `MockExecutor` implementing `MissionExecutor` for test use
  - [ ] Test start/stop lifecycle
  - [ ] Test empty mission start
  - [ ] Test single NAV waypoint mission
  - [ ] Test multi-NAV waypoint advancement
  - [ ] Test DO commands execute between NAVs
  - [ ] Test DO commands stop at NAV boundary
  - [ ] Test mixed NAV/DO mission end-to-end
  - [ ] Test hold time zero advances immediately
  - [ ] Test hold time waits before advance
  - [ ] Test ItemReached emitted after hold complete
  - [ ] Test DO_CHANGE_SPEED updates mission_speed
  - [ ] Test set_current reloads NAV slot
  - [ ] Test clear resets state
  - [ ] Test CurrentChanged events emitted on NAV transitions
  - [ ] Test MissionComplete event on last NAV

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

- [ ] **MissionSequencer global state**
  - [ ] Add `MISSION_SEQUENCER` global in `crates/firmware/src/core/mission/state.rs`
  - [ ] Add helper functions for sequencer access (with/with_mut pattern)
- [ ] **Auto mode MissionExecutor implementation**
  - [ ] Implement `start_command()`:
    - [ ] NAV_WAYPOINT: set navigation target, reset nav controller, return Accepted
    - [ ] DO_CHANGE_SPEED: (sequencer handles internally), return Complete
    - [ ] Unknown NAV (ID <= 95): treat as waypoint with lat/lon, return Accepted
    - [ ] Unknown DO: log warning, return Unsupported
  - [ ] Implement `verify_command()`:
    - [ ] Run nav controller with current GPS/heading
    - [ ] Return `at_target` from NavigationOutput
  - [ ] Implement `on_mission_complete()`:
    - [ ] Stop motors (steering=0, throttle=0)
    - [ ] Set mission_complete flag
- [ ] **Restructure Auto mode update()**
  - [ ] Replace direct `get_current_target()` / `advance_waypoint()` calls
  - [ ] Call `sequencer.update(storage, self_as_executor, now_ms)` instead
  - [ ] Convert returned MissionEvents to MAVLink messages for telemetry queue
  - [ ] Apply navigation output from verify_command (steering/throttle)
  - [ ] Simplify AutoState: remove current_wp_index (owned by sequencer)
- [ ] **Command type preservation fix**
  - [ ] Fix `waypoint_to_mission_item()` in `handlers/mission.rs`
  - [ ] Pass through original `command` field instead of falling back to NAV_WAYPOINT
  - [ ] Pass through original `frame` field
  - [ ] Add unit test for non-NAV_WAYPOINT command preservation
- [ ] **Existing test updates**
  - [ ] Update Auto mode tests for new executor-based structure
  - [ ] Verify all existing tests still pass

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

- [ ] **MISSION_CURRENT telemetry**
  - [ ] Add MISSION_CURRENT to TelemetryStreamer at 1Hz
  - [ ] Increase buffer capacity from 6 to 8
  - [ ] Populate fields: seq (current NAV index), total (storage count), mission_state, mission_mode
  - [ ] Map MissionState to MISSION_STATE enum values
- [ ] **MISSION_ITEM_REACHED telemetry**
  - [ ] Convert `ItemReached(seq)` events from sequencer to MISSION_ITEM_REACHED messages
  - [ ] Queue event-driven messages alongside periodic telemetry
  - [ ] Verify message format matches MAVLink spec (seq field only)
- [ ] **MISSION_CLEAR_ALL handler**
  - [ ] Add handler in `mission.rs` for MISSION_CLEAR_ALL message
  - [ ] Call `sequencer.clear(storage)` to reset state and clear storage
  - [ ] Reject if armed and mission running (return MISSION_ACK DENIED)
  - [ ] Return MISSION_ACK ACCEPTED on success
- [ ] **MISSION_SET_CURRENT handler**
  - [ ] Add handler in `mission.rs` for MISSION_SET_CURRENT message
  - [ ] Validate seq is within storage bounds
  - [ ] Call `sequencer.set_current(seq, storage, executor)`
  - [ ] Return MISSION_CURRENT as acknowledgment
- [ ] **Dispatcher wiring**
  - [ ] Wire MISSION_CLEAR_ALL to handler in dispatcher
  - [ ] Wire MISSION_SET_CURRENT to handler in dispatcher
- [ ] **Tests**
  - [ ] Test MISSION_CURRENT message format and rate
  - [ ] Test MISSION_ITEM_REACHED message format
  - [ ] Test CLEAR_ALL success and rejection cases
  - [ ] Test SET_CURRENT with valid and invalid indices

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

## Definition of Done

- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover` builds successfully
- [ ] `bun scripts/trace-status.ts --check` passes
- [ ] `bun format` and `bun lint` pass
- [ ] ADRs referenced and followed (ADR-0yapl, ADR-2hs12)
- [ ] No `unsafe` code
- [ ] No vague naming ("manager"/"util")
- [ ] MissionSequencer fully testable on host without embedded dependencies

## Open Questions

- [ ] Should MISSION_ITEM_REACHED be queued through a dedicated Embassy channel or piggybacked on telemetry return? -> Next step: Evaluate in Phase 4 based on telemetry buffer constraints
- [ ] Should mission_speed reset on mode exit? -> Method: Check ArduPilot behavior; tentatively reset on stop/clear
- [ ] Should the sequencer handle mission upload-while-running (clear and restart vs reject)? -> Next step: Defer to future task; current behavior is undefined

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
