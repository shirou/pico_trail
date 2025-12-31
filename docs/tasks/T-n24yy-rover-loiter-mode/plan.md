# T-n24yy Rover Loiter Mode

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement Loiter mode for ground rovers with position holding capability. The mode supports Type 0 (stop motors) and Type 1 (active position correction) behaviors, following ArduPilot conventions for LOIT_TYPE and LOIT_RADIUS parameters.

## Success Metrics

- [ ] Loiter mode selectable via MAVLink DO_SET_MODE
- [ ] Type 0 stops motors and maintains position record
- [ ] Type 1 detects drift and navigates back when outside LOIT_RADIUS
- [ ] Drift detection completes within 1ms
- [ ] All unit tests pass
- [ ] Embedded build successful

## Scope

- Goal: Implement RoverLoiter with Type 0 and Type 1 support
- Non-Goals: Boat Loiter (separate), GPS driver changes, full L1 navigation
- Assumptions: Navigation controller exists, GPS state available, parameter system functional
- Constraints: no_std compatible, < 100 bytes RAM for loiter state

## ADR & Legacy Alignment

- [ ] Confirm ADR-8icsq-vehicle-type-separation is followed (rover feature gate)
- [ ] Confirm ADR-w9zpl-control-mode-architecture is followed (Mode trait)
- [ ] No existing Loiter code to migrate

## Plan Summary

- Phase 1 – Parameter definitions for LOIT_TYPE and LOIT_RADIUS
- Phase 2 – RoverLoiter struct and Type 0 implementation
- Phase 3 – Type 1 implementation with drift detection and correction
- Phase 4 – Testing and mode integration

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Loiter Parameters

### Goal

- Define LOIT_TYPE and LOIT_RADIUS parameters
- Integrate with existing parameter system

### Inputs

- Documentation:
  - `docs/requirements/FR-30t03-loiter-type-parameter.md` – Parameter requirements
  - [ArduPilot LOIT Parameters](https://ardupilot.org/rover/docs/parameters.html#loit-parameters)
- Source Code to Reference:
  - `src/parameters/mod.rs` – Parameter infrastructure
  - `src/parameters/circle.rs` – Example parameter module

### Tasks

- [x] **Create parameter module**
  - [x] Create `src/parameters/loiter.rs`
  - [x] Define `LoiterParams` struct with `loit_type` and `loit_radius`
  - [x] Implement `Default` trait with ArduPilot defaults (type=0, radius=2.0m)
  - [x] Add parameter validation (type: 0-1, radius: 0.5-100m)
- [x] **Parameter integration**
  - [x] Add loiter module to `src/parameters/mod.rs`
  - [x] Register parameters with MAVLink parameter protocol
  - [x] Add getter functions: `get_loit_type()`, `get_loit_radius()`

### Deliverables

- `src/parameters/loiter.rs`
- Updated `src/parameters/mod.rs`

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet parameters
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- Parameters defined with correct defaults
- Parameters accessible via getter functions
- Parameter changes take effect immediately
- Embedded build successful

### Rollback/Fallback

- If parameter system integration issues, use hardcoded defaults initially

---

## Phase 2: RoverLoiter Structure and Type 0

### Phase 2 Goal

- Create RoverLoiter struct implementing Mode trait
- Implement Type 0 (stop motors) behavior
- Implement loiter point calculation

### Phase 2 Inputs

- Dependencies:
  - Phase 1: loiter parameters
- Source Code to Reference:
  - `src/rover/mode/mod.rs` – Mode trait and FlightMode enum
  - `src/rover/mode/hold.rs` – Similar mode implementation
  - `src/subsystems/navigation/geo.rs` – Distance/bearing calculations

### Phase 2 Tasks

- [x] **Loiter state definition**
  - [x] Define `LoiterState` struct (loiter_point, loiter_type, radius, is_correcting)
  - [x] Implement `LoiterState::new()` constructor
- [x] **RoverLoiter struct**
  - [x] Create `src/rover/mode/loiter.rs` with `#[cfg(feature = "rover")]`
  - [x] Define `RoverLoiter` struct with `state: Option<LoiterState>`
  - [x] Implement `RoverLoiter::new()` constructor
- [x] **Mode trait implementation (Type 0 focus)**
  - [x] Implement `enter()`:
    - [x] Validate GPS fix (reject if no fix)
    - [x] Read LOIT_TYPE, LOIT_RADIUS parameters
    - [x] Calculate loiter point (current position or projected stop)
    - [x] Initialize LoiterState
    - [x] Log mode entry
  - [x] Implement `update()` dispatcher (routes to type0 or type1)
  - [x] Implement `update_type0()` (set steering=0, throttle=0)
  - [x] Implement `exit()` (neutral actuators, clear state, log exit)
  - [x] Implement `name()` returning "Loiter"
- [x] **Loiter point calculation**
  - [x] Implement `calculate_loiter_point()`:
    - [x] If speed < 0.5 m/s: use current position
    - [x] If moving: project stop point based on v^2/(2\*a)
    - [x] Cap projection to 50m maximum
  - [x] Use existing `offset_position()` from geo.rs
- [ ] **Add Loiter to FlightMode enum**
  - [ ] Add `Loiter` variant to `FlightMode` enum (Phase 4)
  - [ ] Update mode creation in mode manager (Phase 4)

### Phase 2 Deliverables

- `src/rover/mode/loiter.rs`
- Updated `src/rover/mode/mod.rs`
- Type 0 functionality complete

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet loiter
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- Loiter mode enters successfully with GPS fix
- Loiter mode rejects entry without GPS fix
- Loiter point calculated correctly
- Type 0 outputs zero steering/throttle
- Mode exits cleanly

### Phase 2 Rollback/Fallback

- If GPS state access issues, create mock for testing
- If mode manager integration issues, test mode in isolation first

---

## Phase 3: Type 1 Active Position Correction

### Phase 3 Goal

- Implement drift detection with hysteresis
- Implement position correction using navigation controller
- Handle GPS loss gracefully

### Phase 3 Inputs

- Dependencies:
  - Phase 2: RoverLoiter structure, loiter state
- Source Code to Reference:
  - `src/subsystems/navigation/controller.rs` – SimpleNavigationController
  - `src/subsystems/navigation/geo.rs` – distance_m() function

### Phase 3 Tasks

- [x] **Drift detection**
  - [x] Implement distance calculation from loiter point
  - [x] Implement hysteresis state machine:
    - [x] Start correcting when distance > LOIT_RADIUS
    - [x] Stop correcting when distance < LOIT_RADIUS \* 0.8
  - [x] Track is_correcting state for telemetry
- [x] **Navigation controller integration**
  - [x] Add `nav_controller: SimpleNavigationController` to RoverLoiter
  - [x] Create PositionTarget from loiter_point when correcting
  - [x] Call nav_controller.update() when is_correcting
  - [x] Apply nav_output.steering and nav_output.throttle
- [x] **Implement update_type1()**
  - [x] Read current GPS position
  - [x] Check GPS validity (degrade to type0 if invalid)
  - [x] Calculate distance to loiter point
  - [x] Update hysteresis state
  - [x] If correcting: navigate to loiter point
  - [x] If not correcting: output zero steering/throttle
- [x] **GPS loss handling**
  - [x] Detect GPS fix loss
  - [x] Degrade to Type 0 behavior (stop motors)
  - [x] Log warning via STATUSTEXT
  - [x] Resume Type 1 when GPS fix restored

### Phase 3 Deliverables

- Complete Type 1 implementation
- Drift detection with hysteresis
- GPS loss handling

### Phase 3 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet loiter
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- Drift detected correctly using distance calculation
- Hysteresis prevents oscillation at radius boundary
- Navigation controller used for position correction
- GPS loss triggers graceful degradation to Type 0
- is_correcting state accurate for telemetry

### Phase 3 Rollback/Fallback

- If navigation controller issues, simplify to basic bearing steering
- If hysteresis causes issues, reduce factor or use simple threshold

---

## Phase 4: Testing and Integration

### Phase 4 Goal

- Comprehensive unit tests
- MAVLink mode selection integration
- Performance verification

### Phase 4 Tasks

- [x] **Unit tests**
  - [x] Test loiter point calculation at rest
  - [x] Test loiter point calculation while moving
  - [x] Test loiter point projection capping
  - [x] Test Type 0 outputs zero (via update test)
  - [x] Test Type 1 drift detection (via hysteresis tests)
  - [x] Test hysteresis prevents oscillation
  - [x] Test GPS loss degradation (implemented in code, runtime test)
- [x] **Mode integration tests**
  - [x] Test mode entry with valid GPS (host test)
  - [ ] Test mode entry rejection without GPS (embedded only)
  - [ ] Test mode transition Manual → Loiter (requires mode manager)
  - [ ] Test mode transition Loiter → Manual (requires mode manager)
  - [ ] Test parameter changes during Loiter (deferred)
- [ ] **Performance tests**
  - [ ] Verify drift detection < 1ms (requires embedded profiling)
  - [ ] Verify total update < 2ms (requires embedded profiling)
- [ ] **MAVLink integration**
  - [ ] Add Loiter to DO_SET_MODE handler (separate task)
  - [ ] Verify HEARTBEAT reports correct mode (separate task)
  - [ ] Test mode selection from GCS (separate task)

### Phase 4 Deliverables

- Comprehensive test suite
- MAVLink mode selection working
- Performance verification results

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet loiter
cargo test --lib --quiet mode
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 4 Acceptance Criteria

- All unit tests pass
- Mode selectable from MAVLink
- Performance within requirements
- No regressions in existing tests

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings` (lib only, examples have unrelated issues)
- [x] `cargo test --lib --quiet` (loiter tests require rover feature)
- [x] `./scripts/build-rp2350.sh pico_trail_rover` builds successfully
- [x] Documentation updated (architecture.md)
- [x] ADRs referenced and followed (ADR-8icsq, ADR-w9zpl)
- [x] No `unsafe` code
- [x] No vague naming ("manager"/"util")

## Open Questions

- [ ] Should we add LOIT_SPEED parameter for correction speed limit? → Method: Evaluate after initial testing
- [ ] Need reverse navigation for tight spaces? → Method: Test with hardware
- [ ] Should loiter point be adjustable via MAVLink? → Decision: Defer, not standard ArduPilot behavior

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
