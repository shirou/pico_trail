# T-bo6xc RTL and SmartRTL Mode | Implementation Plan

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement RTL (Return to Launch) and SmartRTL modes for the pico_trail rover. SmartRTL retraces the recorded path for safe navigation, while direct RTL provides straight-line fallback navigation to home.

## Success Metrics

- [ ] SmartRTL retraces recorded path successfully
- [ ] Direct RTL navigates to home when path unavailable
- [ ] Path recording uses < 10 KB RAM
- [ ] Navigation updates complete within 1ms at 50 Hz
- [ ] All existing tests pass; no regressions

## Scope

- Goal: Implement autonomous return-to-home capability with safe path retracing
- Non-Goals: RTL_SPEED parameter, RTL_OPTIONS bitmask, path persistence
- Assumptions: GPS and home position available, SimpleNavigationController working
- Constraints: < 10 KB RAM for path buffer, no_std environment

## ADR & Legacy Alignment

- [ ] Confirm ADR-cg5iz-rtl-smartrtl-architecture governs this work
- [ ] Confirm ADR-w9zpl-control-mode-architecture for Mode trait compliance

## Plan Summary

- Phase 1 – Path Recording Infrastructure
- Phase 2 – Direct RTL Mode Implementation
- Phase 3 – SmartRTL Mode Implementation
- Phase 4 – Integration and Testing

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Path Recording Infrastructure

### Goal

- Create PathRecorder for recording GPS positions during armed operation

### Inputs

- Documentation:
  - `docs/adr/ADR-cg5iz-rtl-smartrtl-architecture.md` – Architecture reference
- Source Code to Modify:
  - `src/subsystems/navigation/mod.rs` – Export PathRecorder
- Dependencies:
  - Internal: `src/devices/gps/` – GPS position types
  - External crates: `embassy-sync` – Mutex for thread safety

### Tasks

- [ ] **Create PathRecorder module**
  - [ ] Create `src/subsystems/navigation/path_recorder.rs`
  - [ ] Define `PathPoint` struct (lat, lon, timestamp_ms)
  - [ ] Define `PathRecorder` struct with ring buffer
  - [ ] Implement `PathRecorder::new()` constructor
  - [ ] Implement `PathRecorder::start()` (called on arm)
  - [ ] Implement `PathRecorder::stop()` (called on disarm)
  - [ ] Implement `PathRecorder::record()` with distance/time threshold
  - [ ] Implement `PathRecorder::get_return_path()` iterator
  - [ ] Implement `PathRecorder::has_path()` check
  - [ ] Implement `PathRecorder::count()` for waypoint count
- [ ] **Global PathRecorder instance**
  - [ ] Create static `PATH_RECORDER` with Mutex
  - [ ] Export from `src/subsystems/navigation/mod.rs`
- [ ] **Path simplification**
  - [ ] Implement basic distance-based simplification
  - [ ] Use SRTL_ACCURACY_M constant for threshold
- [ ] **Unit tests**
  - [ ] Test path recording with mock positions
  - [ ] Test ring buffer overflow behavior
  - [ ] Test distance/time threshold logic
  - [ ] Test path retrieval in reverse order

### Deliverables

- `src/subsystems/navigation/path_recorder.rs` with PathRecorder implementation
- Unit tests for path recording logic

### Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet path_recorder
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- PathRecorder records positions during armed operation
- Ring buffer correctly handles overflow
- Path retrieval returns points in reverse order
- Unit tests pass

### Rollback/Fallback

- Revert path_recorder.rs if blocking issues discovered

---

## Phase 2: Direct RTL Mode Implementation

### Phase 2 Goal

- Implement direct RTL mode as fallback navigation to home

### Phase 2 Inputs

- Dependencies:
  - Phase 1: PATH_RECORDER available (but not required for direct RTL)
  - `src/rover/mode/mod.rs` – Mode trait definition
  - `src/subsystems/navigation/controller.rs` – SimpleNavigationController
  - `src/communication/mavlink/state.rs` – HomePosition, SYSTEM_STATE

### Phase 2 Tasks

- [ ] **Create RtlMode module**
  - [ ] Create `src/rover/mode/rtl.rs`
  - [ ] Define `RtlMode` struct with nav_controller, target, arrived
  - [ ] Implement `RtlMode::new()` constructor
  - [ ] Implement `RtlMode::can_enter()` validation (GPS, home)
- [ ] **Implement Mode trait for RtlMode**
  - [ ] Implement `enter()` - validate, set target to home, reset controller
  - [ ] Implement `update()` - navigate, check arrival
  - [ ] Implement `exit()` - cleanup logging
  - [ ] Implement `name()` - return "RTL"
- [ ] **Export and register**
  - [ ] Export `RtlMode` from `src/rover/mode/mod.rs`
  - [ ] Add RTL to mode switching logic (if applicable)
- [ ] **Unit tests**
  - [ ] Test entry validation (GPS required, home required)
  - [ ] Test navigation toward target
  - [ ] Test arrival detection

### Phase 2 Deliverables

- `src/rover/mode/rtl.rs` with direct RTL implementation
- Unit tests for RTL mode

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet rtl
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- RtlMode validates GPS and home before entry
- Navigation proceeds toward home position
- Arrival detected when within WP_RADIUS
- Unit tests pass

### Phase 2 Rollback/Fallback

- Revert rtl.rs if blocking issues discovered

---

## Phase 3: SmartRTL Mode Implementation

### Phase 3 Goal

- Implement SmartRTL mode with path retracing

### Phase 3 Inputs

- Dependencies:
  - Phase 1: PATH_RECORDER with recorded path
  - Phase 2: RtlMode for fallback and can_enter() validation
  - `src/rover/mode/mod.rs` – Mode trait

### Phase 3 Tasks

- [ ] **Create SmartRtlMode module**
  - [ ] Create `src/rover/mode/smartrtl.rs`
  - [ ] Define `SmartRtlMode` struct with nav_controller, waypoint state
  - [ ] Implement `SmartRtlMode::new()` constructor
  - [ ] Implement `SmartRtlMode::can_enter()` (GPS, home, path available)
- [ ] **Implement Mode trait for SmartRtlMode**
  - [ ] Implement `enter()` - load waypoints from PathRecorder
  - [ ] Implement `update()` - navigate to current waypoint, advance on arrival
  - [ ] Implement `exit()` - cleanup logging
  - [ ] Implement `name()` - return "SmartRTL"
- [ ] **Waypoint management**
  - [ ] Load path points into local array on enter
  - [ ] Track current waypoint index
  - [ ] Advance to next waypoint on arrival
  - [ ] Complete when all waypoints visited
- [ ] **Export and register**
  - [ ] Export `SmartRtlMode` from `src/rover/mode/mod.rs`
- [ ] **Unit tests**
  - [ ] Test entry validation (path required)
  - [ ] Test waypoint progression
  - [ ] Test completion when all waypoints visited

### Phase 3 Deliverables

- `src/rover/mode/smartrtl.rs` with SmartRTL implementation
- Unit tests for SmartRTL mode

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet smartrtl
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- SmartRtlMode loads waypoints from PathRecorder
- Navigation progresses through waypoints in reverse order
- Mode completes when reaching home
- Unit tests pass

### Phase 3 Rollback/Fallback

- Fall back to direct RTL if SmartRTL issues discovered

---

## Phase 4: Integration and Testing

### Phase 4 Goal

- Integrate RTL/SmartRTL with mode manager and test end-to-end

### Phase 4 Tasks

- [ ] **RTL mode selection**
  - [ ] Implement `select_rtl_mode()` function
  - [ ] Try SmartRTL first, fall back to direct RTL
  - [ ] Log which mode was selected
- [ ] **GPS loss handling**
  - [ ] Detect GPS loss in update()
  - [ ] Return error to trigger mode manager transition
  - [ ] Log GPS loss event
- [ ] **Integration with arming**
  - [ ] Start path recording on arm
  - [ ] Stop path recording on disarm
  - [ ] Clear path buffer on arm (fresh path each arm)
- [ ] **Integration tests**
  - [ ] Test RTL mode selection logic
  - [ ] Test fallback from SmartRTL to direct RTL
  - [ ] Test GPS loss handling
- [ ] **Documentation**
  - [ ] Update requirements status if needed

### Phase 4 Deliverables

- RTL mode selection function
- GPS loss handling integrated
- Arming integration for path recording
- Integration tests

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 4 Acceptance Criteria

- RTL mode correctly selects SmartRTL or direct RTL based on path
- GPS loss transitions to Hold mode
- Path recording integrates with arming lifecycle
- All tests pass

---

## Definition of Done

- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] Path recording infrastructure complete
- [ ] Direct RTL mode implemented and tested
- [ ] SmartRTL mode implemented and tested
- [ ] GPS loss handling working
- [ ] No `unsafe` code
- [ ] Logging uses abstracted macros

## Open Questions

- [x] SmartRTL or direct RTL as default? → SmartRTL default
- [ ] Path simplification complexity? → Start with basic distance-based, enhance later if needed

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
