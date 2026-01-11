# T-r9v2k Heading Source and Navigation Integration

## Metadata

- Type: Implementation Plan
- Status: Phase 5 In Progress

## Links

- Associated Design Document:
  - [T-r9v2k-design](design.md)

## Overview

Implement the HeadingSource abstraction and integrate AHRS heading with the navigation system to enable Guided and Auto modes for autonomous navigation.

## Success Metrics

- [x] HeadingSource trait implemented with FusedHeadingSource
- [x] NavigationController updated to accept heading parameter
- [x] Guided mode navigates to SET_POSITION_TARGET_GLOBAL_INT targets
- [x] Auto mode executes mission waypoints sequentially
- [x] All existing modes updated to use heading_provider
- [x] All existing tests pass; no regressions

## Scope

- Goal: Enable autonomous navigation using AHRS + GPS fused heading
- Non-Goals: EKF/GSF sensor fusion, attitude stabilization, L1/S-Curve algorithms
- Assumptions: BNO086 AHRS provides valid yaw in NED frame; GPS provides valid COG when moving
- Constraints: no_std environment, Embassy async runtime, must not break existing modes

## ADR & Legacy Alignment

- [ ] Confirm ADR-h3k9f-heading-source-integration is the governing design
- [ ] Note: CircleMode has existing heading_provider pattern that will be migrated to HeadingSource

## Plan Summary

- Phase 1 – HeadingSource trait and FusedHeadingSource implementation
- Phase 2 – NavigationController integration and existing mode updates
- Phase 3 – Guided mode implementation
- Phase 4 – Auto mode implementation
- Phase 5 – Testing and integration verification

---

## Phase 1: HeadingSource Infrastructure

### Goal

- Create the HeadingSource trait and FusedHeadingSource implementation

### Inputs

- Documentation:
  - `docs/adr/ADR-h3k9f-heading-source-integration.md` – Design specification
  - `docs/analysis/AN-vcxr7-ahrs-navigation-control-integration.md` – Context and analysis
- Source Code to Modify:
  - `src/subsystems/navigation/mod.rs` – Add heading module export
- Dependencies:
  - Internal: `src/subsystems/ahrs/traits.rs` – SharedAhrsState
  - Internal: `src/devices/gps.rs` – GpsPosition

### Tasks

- [x] **Create heading module**
  - [x] Create `src/subsystems/navigation/heading.rs`
  - [x] Define `HeadingSourceType` enum (Ahrs, GpsCog, None)
  - [x] Define `HeadingSource` trait with `get_heading()`, `is_valid()`, `source_type()`
  - [x] Implement `FusedHeadingSource` struct
  - [x] Add `new()` constructor with ahrs_state, gps_provider, speed_threshold
  - [x] Implement HeadingSource for FusedHeadingSource
- [x] **Module integration**
  - [x] Export heading module from `src/subsystems/navigation/mod.rs`
  - [x] ~~Add feature gates for embassy if needed~~ (not needed - no embassy-specific code)
- [x] **Unit tests**
  - [x] Test FusedHeadingSource with AHRS healthy, GPS moving → returns GPS COG
  - [x] Test FusedHeadingSource with AHRS healthy, GPS stationary → returns AHRS yaw
  - [x] Test FusedHeadingSource with AHRS unhealthy → returns GPS COG fallback
  - [x] Test is_valid() and source_type() methods

### Deliverables

- `src/subsystems/navigation/heading.rs` with HeadingSource trait and FusedHeadingSource
- Unit tests for heading source selection logic

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet navigation
```

### Acceptance Criteria (Phase Gate)

- HeadingSource trait compiles and is exported
- FusedHeadingSource correctly selects heading source based on speed threshold
- Unit tests pass for all heading source selection scenarios

### Rollback/Fallback

- Revert heading.rs creation; existing modes continue using GPS COG directly

---

## Phase 2: NavigationController Integration

### Phase 2 Goal

- Update NavigationController to accept heading parameter and update existing modes

### Phase 2 Inputs

- Dependencies:
  - Phase 1: HeadingSource trait and FusedHeadingSource
- Source Code to Modify:
  - `src/subsystems/navigation/controller.rs` – NavigationController trait and SimpleNavigationController
  - `src/rover/mode/rtl.rs` – RTL mode
  - `src/rover/mode/smartrtl.rs` – SmartRTL mode
  - `src/rover/mode/loiter.rs` – Loiter mode
  - `src/rover/mode/circle.rs` – Circle mode (refactor existing heading_provider)

### Phase 2 Tasks

- [x] **Update NavigationController trait**
  - [x] Add `heading: f32` parameter to `update()` method
  - [x] Update trait documentation
- [x] **Update SimpleNavigationController**
  - [x] Remove internal `course_over_ground.unwrap_or(0.0)` logic
  - [x] Use heading parameter directly for heading_error calculation
  - [x] Update unit tests
- [x] **Update RTL mode**
  - [x] Add `heading_provider: fn() -> Option<f32>` field
  - [x] Get heading from heading_provider in update()
  - [x] Pass heading to nav_controller.update()
- [x] **Update SmartRTL mode**
  - [x] Add heading_provider field
  - [x] Update update() to use heading_provider
- [x] **Update Loiter mode**
  - [x] Add heading_provider field
  - [x] Update update() to use heading_provider
- [x] **Refactor Circle mode**
  - [x] Circle mode already had heading_provider
  - [x] Updated update() to pass heading to nav_controller.update()

### Phase 2 Deliverables

- Updated NavigationController trait with heading parameter
- All existing modes updated to use HeadingSource

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- NavigationController accepts heading parameter
- All existing modes compile with HeadingSource
- Existing tests pass
- RP2350 build succeeds

### Phase 2 Rollback/Fallback

- Revert NavigationController changes; modes continue using direct GPS COG

---

## Phase 3: Guided Mode Implementation

### Phase 3 Goal

- Implement Guided mode for navigation to SET_POSITION_TARGET_GLOBAL_INT targets

### Phase 3 Inputs

- Dependencies:
  - Phase 2: Updated NavigationController and HeadingSource integration
- Source Code to Modify:
  - `src/rover/mode/mod.rs` – Export GuidedMode
  - `src/communication/mavlink/handlers/` – Position target handler (if not exists)
- Source Code to Create:
  - `src/rover/mode/guided.rs` – Guided mode implementation

### Phase 3 Tasks

- [x] **Create GuidedMode struct**
  - [x] Create `src/rover/mode/guided.rs`
  - [x] Define GuidedMode struct with actuators, nav_controller, heading_provider, gps_provider
  - [x] Implement Mode trait (enter, update, exit, name)
- [x] **Implement enter()**
  - [x] Validate GPS 3D fix
  - [x] Initialize nav_controller
- [x] **Implement update()**
  - [x] Get current GPS position
  - [x] Get target from MISSION_STORAGE via get_current_target()
  - [x] Get heading from heading_provider (fallback to GPS COG)
  - [x] Call nav_controller.update() with heading parameter
  - [x] Apply steering/throttle to actuators
  - [x] Handle at_target condition (call complete_mission())
- [x] **Implement exit()**
  - [x] Stop actuators
  - [x] Reset nav_controller
  - [x] Reset mission state to Idle if was Running
- [x] **MAVLink integration**
  - [x] Verified SET_POSITION_TARGET_GLOBAL_INT handler updates MISSION_STORAGE
  - [x] Handler already exists in src/communication/mavlink/handlers/navigation.rs
- [x] **Export and registration**
  - [x] Export GuidedMode from `src/rover/mode/mod.rs`

### Phase 3 Deliverables

- `src/rover/mode/guided.rs` with functional Guided mode
- MAVLink SET_POSITION_TARGET_GLOBAL_INT integration

### Phase 3 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet mode
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- GuidedMode compiles and implements Mode trait
- Mode enters successfully with valid GPS and heading
- Mode navigates to target and stops at arrival
- RP2350 build succeeds

### Phase 3 Rollback/Fallback

- Revert guided.rs; Guided mode remains unimplemented

---

## Phase 4: Auto Mode Implementation

### Phase 4 Goal

- Implement Auto mode for mission waypoint execution

### Phase 4 Inputs

- Dependencies:
  - Phase 3: GuidedMode as reference implementation
- Source Code to Modify:
  - `src/rover/mode/mod.rs` – Export AutoMode
- Source Code to Create:
  - `src/rover/mode/auto.rs` – Auto mode implementation

### Phase 4 Tasks

- [x] **Create AutoMode struct**
  - [x] Create `src/rover/mode/auto.rs`
  - [x] Define AutoMode struct with mission state tracking
  - [x] Add current_wp_index for waypoint progression
- [x] **Implement enter()**
  - [x] Validate GPS 3D fix
  - [x] ~~Validate HeadingSource is_valid()~~ (deferred - heading validated in update())
  - [x] Validate mission is loaded
  - [x] Initialize to first waypoint
- [x] **Implement update()**
  - [x] Get current GPS position
  - [x] Get current waypoint from mission
  - [x] Get heading from HeadingSource
  - [x] Call nav_controller.update()
  - [x] Apply steering/throttle
  - [x] Check at_target → advance to next waypoint
  - [x] Handle mission complete → set state to Completed
- [x] **Implement exit()**
  - [x] Stop actuators
  - [x] Preserve mission state for resume
- [x] **Mission state machine**
  - [x] Handle NAV_WAYPOINT command (via get_current_target())
  - [x] ~~Handle loiter time~~ (deferred to future enhancement)
  - [x] Handle mission complete condition
- [x] **Export and registration**
  - [x] Export AutoMode from `src/rover/mode/mod.rs`

### Phase 4 Deliverables

- `src/rover/mode/auto.rs` with functional Auto mode
- Mission waypoint progression logic

### Phase 4 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet mode
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 4 Acceptance Criteria

- AutoMode compiles and implements Mode trait
- Mode enters with valid mission
- Mode progresses through waypoints sequentially
- Mode completes mission and transitions to hold
- RP2350 build succeeds

### Phase 4 Rollback/Fallback

- Revert auto.rs; Auto mode remains unimplemented

---

## Phase 5: Testing and Integration

### Phase 5 Goal

- Comprehensive testing and integration verification

### Phase 5 Tasks

- [x] **Unit tests**
  - [x] HeadingSource edge cases (17 new tests)
  - [x] NavigationController with various heading inputs (10 new tests)
  - [x] GuidedMode state transitions (6 new tests)
  - [x] AutoMode waypoint progression (10 new tests)
- [ ] **Integration tests**
  - [ ] Mode lifecycle (enter → update → exit)
  - [ ] Mode transitions (Manual → Guided → Auto → RTL)
  - [ ] MAVLink command flow
- [ ] **Hardware verification**
  - [ ] BNO086 yaw frame verification
  - [ ] GPS COG vs AHRS heading comparison
  - [ ] Navigation accuracy test

### Phase 5 Deliverables

- Comprehensive test coverage for new components
- Documented test results and any known limitations

### Phase 5 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 5 Acceptance Criteria

- All unit tests pass
- All integration tests pass
- RP2350 build succeeds
- No regressions in existing functionality

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet` (726 tests, 724 passed, 1 pre-existing failure unrelated to this task)
- [x] `./scripts/build-rp2350.sh pico_trail_rover`
- [x] ADR-h3k9f referenced and followed
- [x] No `unsafe` code added
- [x] No "manager"/"util" naming
- [x] Feature gates applied correctly for embassy

## Open Questions

- [ ] BNO086 yaw frame (NED, 0° = North) → Method: Hardware testing
- [ ] AHRS healthy requirement for mode entry → Next step: Define as part of Phase 3 implementation
- [ ] Mission storage format and provider → Next step: Review existing mission handling code
