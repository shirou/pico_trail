# T-c26bh Unified Waypoint Navigation Plan

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement unified waypoint navigation using MissionStorage as the single source of truth. This enables Mission Planner's MISSION_ITEM workflow to control rover navigation in both GUIDED and AUTO modes.

## Success Metrics

- [x] Rover navigates when armed in GUIDED mode with uploaded waypoint
- [x] AUTO mode navigates through sequential waypoints
- [x] MissionState correctly tracks Idle/Running/Completed states
- [x] SET_POSITION_TARGET integrates with MissionStorage
- [x] All existing tests pass (no regressions)

## Scope

- Goal: Unify navigation target source to MissionStorage for GUIDED and AUTO modes
- Non-Goals: Mission pause/resume, complex mission commands, RTL mode
- Assumptions: Existing MissionStorage and NavigationController implementations work correctly
- Constraints: Maintain backward compatibility with SET_POSITION_TARGET

## ADR & Legacy Alignment

- [ ] Confirm ADR-2hs12-unified-waypoint-navigation is referenced
- [ ] Verify MissionStorage API is sufficient for navigation needs
- [ ] Check existing NAV_TARGET usage patterns for migration

## Plan Summary

- Phase 1 – Add MissionState and global MISSION_STORAGE
- Phase 2 – Modify navigation_task to read from MissionStorage
- Phase 3 – Implement GUIDED mode ARM-triggered navigation
- Phase 4 – Implement AUTO mode and MAV_CMD_MISSION_START handler
- Phase 5 – Integrate SET_POSITION_TARGET with MissionStorage

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: MissionState and Global Storage

### Goal

- Add MissionState enum for tracking mission execution status
- Create global MISSION_STORAGE accessible from navigation and handlers

### Inputs

- Documentation:
  - `docs/adr/ADR-2hs12-unified-waypoint-navigation.md` – Architecture decision
  - `docs/tasks/T-c26bh-unified-waypoint-navigation/design.md` – Design specification
- Source Code to Reference:
  - `src/core/mission/mod.rs` – Existing MissionStorage
  - `src/subsystems/navigation/mod.rs` – Current NAV_TARGET global

### Tasks

- [x] **MissionState enum**
  - [x] Create `src/core/mission/state.rs`
  - [x] Define `MissionState` enum (Idle, Running, Completed)
  - [x] Implement Default trait (Idle as default)
  - [x] Add unit tests for state transitions
- [x] **Global state definitions**
  - [x] Add `MISSION_STATE` global in `src/core/mission/state.rs`
  - [x] Add `MISSION_STORAGE` global in `src/core/mission/state.rs`
  - [x] Add const constructor `MissionStorage::new_const()` for static init
  - [x] Export from `src/core/mission/mod.rs`
- [x] **Helper functions**
  - [x] `get_mission_state() -> MissionState`
  - [x] `set_mission_state(state: MissionState)`
  - [x] `get_current_target() -> Option<PositionTarget>`

### Deliverables

- `src/core/mission/state.rs`
- Updated `src/core/mission/mod.rs`
- Unit tests for MissionState

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet mission
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- MissionState enum compiles and tests pass
- Global MISSION_STORAGE and MISSION_STATE accessible
- Const initialization works for static globals

### Rollback/Fallback

- Revert commits if global state causes lifetime issues
- Consider alternative state management if Mutex overhead is problematic

---

## Phase 2: Navigation Task Integration

### Phase 2 Goal

- Modify navigation_task to read from MISSION_STORAGE instead of NAV_TARGET
- Add waypoint-to-PositionTarget conversion

### Phase 2 Inputs

- Dependencies:
  - Phase 1: MISSION_STORAGE global, MissionState
- Source Code to Modify:
  - `examples/pico_trail_rover.rs` – navigation_task function
  - `src/subsystems/navigation/types.rs` – PositionTarget conversion

### Phase 2 Tasks

- [x] **Waypoint conversion**
  - [x] Implement `From<&Waypoint> for PositionTarget`
  - [x] Add unit tests for conversion accuracy
- [x] **Navigation task changes**
  - [x] Import MISSION_STORAGE and MissionState
  - [x] Replace NAV_TARGET read with MISSION_STORAGE.current_waypoint()
  - [x] Only navigate when MissionState::Running
  - [x] Update at_target detection to check MissionState
- [x] **Deprecate NAV_TARGET**
  - [x] Add deprecation comment to NAV_TARGET
  - [x] Update any remaining NAV_TARGET references

### Phase 2 Deliverables

- Updated `src/subsystems/navigation/types.rs`
- Updated navigation_task in rover example
- Unit tests for waypoint conversion

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet navigation
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- navigation_task reads from MISSION_STORAGE
- Waypoint coordinates correctly converted to PositionTarget
- Navigation only occurs when MissionState::Running

### Phase 2 Rollback/Fallback

- Keep NAV_TARGET as backup if MISSION_STORAGE access has issues
- Gradual migration path if needed

---

## Phase 3: GUIDED Mode ARM-Triggered Navigation

### Phase 3 Goal

- Start navigation automatically on ARM in GUIDED mode if waypoint exists
- Handle mode changes properly

### Phase 3 Inputs

- Dependencies:
  - Phase 2: Navigation reads from MISSION_STORAGE
- Source Code to Modify:
  - Mode handling code (arming logic)
  - `src/communication/mavlink/handlers/` – ARM command handler

### Phase 3 Tasks

- [x] **ARM handler enhancement**
  - [x] Detect GUIDED mode on ARM command
  - [x] Check if MISSION_STORAGE has waypoint
  - [x] If waypoint exists: Set MissionState::Running
  - [x] If no waypoint: Remain Idle (no navigation)
- [x] **Waypoint arrival handling**
  - [x] On at_target in GUIDED mode: Set MissionState::Completed
  - [x] Hold position after completion
- [x] **Mode change handling**
  - [x] Reset MissionState to Idle on mode change
  - [x] Clear navigation state on mode exit

### Phase 3 Deliverables

- Updated ARM command handling
- GUIDED mode navigation logic
- Mode change handling for MissionState

### Phase 3 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- ARM in GUIDED with waypoint starts navigation
- ARM in GUIDED without waypoint does nothing
- Waypoint arrival stops navigation (MissionState::Completed)

### Phase 3 Rollback/Fallback

- Disable auto-start on ARM if issues arise
- Require explicit MISSION_START for all modes

---

## Phase 4: AUTO Mode and MISSION_START Handler

### Phase 4 Goal

- Implement MAV_CMD_MISSION_START handler
- Add waypoint advance logic for AUTO mode
- Handle mission completion

### Phase 4 Inputs

- Dependencies:
  - Phase 3: GUIDED mode working
- Source Code to Modify:
  - `src/communication/mavlink/handlers/command.rs` – Command handler

### Phase 4 Tasks

- [x] **MISSION_START handler**
  - [x] Create handler for MAV_CMD_MISSION_START (command ID 300)
  - [x] Verify mission not empty
  - [x] Set current_index to 0
  - [x] Set MissionState::Running
  - [x] Return MAV_RESULT_ACCEPTED on success
  - [x] Return MAV_RESULT_FAILED if mission empty
- [x] **Waypoint advance logic**
  - [x] On waypoint arrival in AUTO mode:
    - [x] Increment current_index
    - [x] If more waypoints: continue navigation
    - [x] If last waypoint: MissionState::Completed
- [x] **Mission completion**
  - [x] Transition to Hold mode on completion
  - [x] Send appropriate MAVLink status message

### Phase 4 Deliverables

- MAV_CMD_MISSION_START handler
- AUTO mode waypoint advance logic
- Mission completion handling

### Phase 4 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 4 Acceptance Criteria

- MISSION_START command starts AUTO mode navigation
- Waypoints advance automatically in AUTO mode
- Mission completes and transitions to Hold

### Phase 4 Rollback/Fallback

- Disable AUTO mode if waypoint advance is buggy
- Manual waypoint advance via GCS commands

---

## Phase 5: SET_POSITION_TARGET Integration

### Phase 5 Goal

- Modify SET_POSITION_TARGET to update MissionStorage
- Maintain backward compatibility with existing workflow

### Phase 5 Inputs

- Dependencies:
  - Phase 4: AUTO mode working
- Source Code to Modify:
  - `src/communication/mavlink/handlers/navigation.rs` – Position target handler

### Phase 5 Tasks

- [x] **Handler modification**
  - [x] On SET_POSITION_TARGET received:
    - [x] Clear MISSION_STORAGE
    - [x] Create Waypoint from position data
    - [x] Add waypoint to MISSION_STORAGE
    - [x] If in GUIDED mode: Set MissionState::Running
- [x] **Remove NAV_TARGET usage**
  - [x] Remove direct NAV_TARGET updates
  - [x] Keep REPOSITION_TARGET for sync-to-async bridge (DO_REPOSITION)
  - [x] Update handler documentation
- [x] **Backward compatibility tests**
  - [x] Test SET_POSITION_TARGET still works for GUIDED mode
  - [x] Verify existing tests pass (no regressions)

### Phase 5 Deliverables

- Updated SET_POSITION_TARGET handler
- Removed or deprecated NAV_TARGET
- Integration tests

### Phase 5 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 5 Acceptance Criteria

- SET_POSITION_TARGET updates MissionStorage
- GUIDED mode navigation works via SET_POSITION_TARGET
- No regressions in existing functionality

### Phase 5 Rollback/Fallback

- Keep NAV_TARGET as parallel path if integration issues
- Document migration path for external users

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh pico_trail_rover` builds successfully
- [x] Documentation updated if needed
- [x] ADRs referenced and followed
- [x] No `unsafe` code
- [x] No vague naming ("manager"/"util")

## Open Questions

- [ ] Should MISSION_ITEM_REACHED be sent on waypoint arrival? -> Defer to Phase 4
- [ ] How to handle mission upload while running? -> Clear and restart vs reject
- [ ] Need to support mission pause/resume? -> Deferred to future enhancement

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
