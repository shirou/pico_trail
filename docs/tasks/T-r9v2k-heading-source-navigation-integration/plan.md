# T-r9v2k Heading Source and Navigation Integration

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [T-r9v2k-design](design.md)

## Overview

Implement the HeadingSource abstraction and integrate AHRS heading with the navigation system to enable Guided and Auto modes for autonomous navigation.

## Success Metrics

- [ ] HeadingSource trait implemented with FusedHeadingSource
- [ ] NavigationController updated to accept heading parameter
- [ ] Guided mode navigates to SET_POSITION_TARGET_GLOBAL_INT targets
- [ ] Auto mode executes mission waypoints sequentially
- [ ] All existing modes updated to use HeadingSource
- [ ] All existing tests pass; no regressions

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

- [ ] **Create heading module**
  - [ ] Create `src/subsystems/navigation/heading.rs`
  - [ ] Define `HeadingSourceType` enum (Ahrs, GpsCog, None)
  - [ ] Define `HeadingSource` trait with `get_heading()`, `is_valid()`, `source_type()`
  - [ ] Implement `FusedHeadingSource` struct
  - [ ] Add `new()` constructor with ahrs_state, gps_provider, speed_threshold
  - [ ] Implement HeadingSource for FusedHeadingSource
- [ ] **Module integration**
  - [ ] Export heading module from `src/subsystems/navigation/mod.rs`
  - [ ] Add feature gates for embassy if needed
- [ ] **Unit tests**
  - [ ] Test FusedHeadingSource with AHRS healthy, GPS moving → returns GPS COG
  - [ ] Test FusedHeadingSource with AHRS healthy, GPS stationary → returns AHRS yaw
  - [ ] Test FusedHeadingSource with AHRS unhealthy → returns GPS COG fallback
  - [ ] Test is_valid() and source_type() methods

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

- [ ] **Update NavigationController trait**
  - [ ] Add `heading: f32` parameter to `update()` method
  - [ ] Update trait documentation
- [ ] **Update SimpleNavigationController**
  - [ ] Remove internal `course_over_ground.unwrap_or(0.0)` logic
  - [ ] Use heading parameter directly for heading_error calculation
  - [ ] Update unit tests
- [ ] **Update RTL mode**
  - [ ] Add `heading_source: &'a dyn HeadingSource` field
  - [ ] Get heading from HeadingSource in update()
  - [ ] Pass heading to nav_controller.update()
- [ ] **Update SmartRTL mode**
  - [ ] Add HeadingSource field
  - [ ] Update update() to use HeadingSource
- [ ] **Update Loiter mode**
  - [ ] Add HeadingSource field
  - [ ] Update update() to use HeadingSource
- [ ] **Refactor Circle mode**
  - [ ] Replace heading_provider function pointer with HeadingSource
  - [ ] Simplify heading logic using HeadingSource

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

- [ ] **Create GuidedMode struct**
  - [ ] Create `src/rover/mode/guided.rs`
  - [ ] Define GuidedMode struct with actuators, nav_controller, heading_source, gps_provider, target_provider
  - [ ] Implement Mode trait (enter, update, exit, name)
- [ ] **Implement enter()**
  - [ ] Validate GPS 3D fix
  - [ ] Validate HeadingSource is_valid()
  - [ ] Initialize nav_controller
- [ ] **Implement update()**
  - [ ] Get current GPS position
  - [ ] Get target from NAV_TARGET global state
  - [ ] Get heading from HeadingSource
  - [ ] Call nav_controller.update()
  - [ ] Apply steering/throttle to actuators
  - [ ] Handle at_target condition (hold position)
- [ ] **Implement exit()**
  - [ ] Stop actuators
  - [ ] Reset nav_controller
- [ ] **MAVLink integration**
  - [ ] Verify SET_POSITION_TARGET_GLOBAL_INT handler updates NAV_TARGET
  - [ ] Add handler if not present
- [ ] **Export and registration**
  - [ ] Export GuidedMode from `src/rover/mode/mod.rs`
  - [ ] Add to mode registry/dispatcher if applicable

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

- [ ] **Create AutoMode struct**
  - [ ] Create `src/rover/mode/auto.rs`
  - [ ] Define AutoMode struct with mission state tracking
  - [ ] Add current_wp_index for waypoint progression
- [ ] **Implement enter()**
  - [ ] Validate GPS 3D fix
  - [ ] Validate HeadingSource is_valid()
  - [ ] Validate mission is loaded
  - [ ] Initialize to first waypoint
- [ ] **Implement update()**
  - [ ] Get current GPS position
  - [ ] Get current waypoint from mission
  - [ ] Get heading from HeadingSource
  - [ ] Call nav_controller.update()
  - [ ] Apply steering/throttle
  - [ ] Check at_target → advance to next waypoint
  - [ ] Handle mission complete → transition to Hold/Loiter
- [ ] **Implement exit()**
  - [ ] Stop actuators
  - [ ] Preserve mission state for resume
- [ ] **Mission state machine**
  - [ ] Handle NAV_WAYPOINT command
  - [ ] Handle loiter time (if applicable)
  - [ ] Handle mission complete condition
- [ ] **Export and registration**
  - [ ] Export AutoMode from `src/rover/mode/mod.rs`

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

- [ ] **Unit tests**
  - [ ] HeadingSource edge cases
  - [ ] NavigationController with various heading inputs
  - [ ] GuidedMode state transitions
  - [ ] AutoMode waypoint progression
- [ ] **Integration tests**
  - [ ] Mode lifecycle (enter → update → exit)
  - [ ] Mode transitions (Manual → Guided → Auto → RTL)
  - [ ] MAVLink command flow
- [ ] **Hardware verification** (if available)
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

- [ ] `cargo check`
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] ADR-h3k9f referenced and followed
- [ ] No `unsafe` code added
- [ ] No "manager"/"util" naming
- [ ] Feature gates applied correctly for embassy

## Open Questions

- [ ] BNO086 yaw frame (NED, 0° = North) → Method: Hardware testing
- [ ] AHRS healthy requirement for mode entry → Next step: Define as part of Phase 3 implementation
- [ ] Mission storage format and provider → Next step: Review existing mission handling code
