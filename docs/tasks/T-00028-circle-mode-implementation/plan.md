# T-00028 Circle Mode Implementation Plan

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement Circle mode that enables the rover to orbit around a fixed center point at configurable radius and speed. Uses hybrid approach: continuous circle generator feeding look-ahead targets to L1 navigation controller.

## Success Metrics

- [ ] Circle mode calculates target points within 1ms on RP2350
- [ ] Path following error < 2m RMS at typical speeds
- [ ] All acceptance criteria from FR-00112 satisfied
- [ ] All existing tests pass; no regressions in rover subsystem

## Scope

- Goal: Implement functional Circle mode with ArduPilot-compatible parameters
- Non-Goals: Dynamic center updates, altitude control, advanced patterns
- Assumptions: L1 navigation controller is functional; GPS state available
- Constraints: RP2350 performance budget (1ms for circle calculations)

## ADR & Legacy Alignment

- [x] Confirm the latest ADRs/design documents that govern this work are referenced above (update `Related ADRs` if needed).
- [ ] Note any known gaps between existing code/dependencies and the approved approach; add explicit subtasks in the phase checklists to retire or migrate those legacy patterns.

## Plan Summary

- Phase 1 – Circle calculation utilities (geodetic functions)
- Phase 2 – CircleMode struct and Mode trait implementation
- Phase 3 – Parameter integration and testing

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: Circle Calculation Utilities

### Goal

- Implement geodetic utility functions required for circle path generation

### Inputs

- Documentation:
  - `docs/adr/ADR-00029-circle-mode-path-generation.md` – Architecture decision
  - `docs/requirements/FR-00111-circle-center-point.md` – Center calculation spec
- Source Code to Modify:
  - `src/core/math/` – Math utilities (add geodetic functions if not present)
  - `src/subsystems/navigation/geo.rs` – Existing geodetic calculations
- Dependencies:
  - External crates: `libm` – Trigonometric functions for no_std

### Tasks

- [x] **Verify existing geodetic utilities**
  - [x] Check if `calculate_bearing()` exists in navigation module
  - [x] Check if `offset_position()` exists
  - [x] Check if `wrap_180()` / `wrap_360()` exists
- [x] **Add missing geodetic functions (if needed)**
  - [x] Implement `offset_position(origin, distance, bearing) -> GpsPosition`
  - [x] Ensure bearing calculations handle all quadrants
  - [x] Add unit tests for geodetic functions
- [x] **Add CircleDirection enum**
  - [x] Create `src/rover/mode/circle.rs` with `CircleDirection` enum
  - [x] Add `Clockwise` and `CounterClockwise` variants

### Deliverables

- Complete geodetic utility functions
- `CircleDirection` enum defined
- Unit tests for all geodetic calculations

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet navigation
cargo test --lib --quiet circle
```

### Acceptance Criteria (Phase Gate)

- All geodetic functions have unit tests
- Tests pass for known coordinate pairs (Tokyo, etc.)
- `offset_position` correctly handles all bearings (0°, 90°, 180°, 270°)

### Rollback/Fallback

- Revert new files if issues found
- Geodetic functions are isolated, no impact on existing code

---

## Phase 2: CircleMode Implementation

### Phase 2 Goal

- Implement `CircleMode` struct with full Mode trait implementation

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Geodetic utilities complete
  - `src/rover/mode/mod.rs` – Mode trait definition
  - `src/subsystems/navigation/` – Navigation controller
- Source Code to Modify:
  - `src/rover/mode/mod.rs` – Add circle module export
  - `src/rover/mode/circle.rs` – Circle mode implementation

### Phase 2 Tasks

- [x] **Create CircleMode struct**
  - [x] Define struct fields: center, radius, speed, direction
  - [x] Add references to navigation controller, GPS state, AHRS
  - [x] Implement `CircleMode::new()` constructor
- [x] **Implement Mode trait**
  - [x] `enter()`: Validate GPS, calculate center, load parameters
  - [x] `update(dt)`: Calculate target, delegate to L1 controller
  - [x] `exit()`: Set actuators to neutral, clear state
  - [x] `name()`: Return "Circle"
- [x] **Implement calculate_target()**
  - [x] Calculate current angle from center to vehicle
  - [x] Apply angular velocity for look-ahead
  - [x] Handle clockwise/counterclockwise direction
  - [x] Return target point on circle perimeter
- [x] **Handle special cases**
  - [x] Stationary mode (CIRC_RADIUS = 0)
  - [x] GPS fix lost during operation (transition to Hold - stub for now)
- [x] **Export CircleMode from module**
  - [x] Add `pub mod circle;` to `src/rover/mode/mod.rs`
  - [x] Add `pub use circle::CircleMode;`

### Phase 2 Deliverables

- Functional `CircleMode` implementing `Mode` trait
- Circle target calculation working
- Mode entry/update/exit lifecycle complete

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet circle
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- CircleMode compiles and passes trait bounds
- Target calculation produces correct angles for CW/CCW
- Embedded build succeeds

### Phase 2 Rollback/Fallback

- CircleMode is isolated; can be removed without affecting Manual mode
- If navigation controller integration fails, stub the navigation calls

---

## Phase 3: Parameters and Testing

### Phase 3 Goal

- Add CIRC\_\* parameters and comprehensive tests

### Phase 3 Tasks

- [x] **Add Circle parameters to parameter system**
  - [x] Register CIRC_RADIUS (f32, default 20.0, range 0-1000)
  - [x] Register CIRC_SPEED (f32, default 2.0, range 0-10)
  - [x] Register CIRC_DIR (i8, default 0, range 0-1)
  - [x] Add parameter validation
- [x] **Update CircleMode to read parameters**
  - [x] Add `CircleConfig::from_store()` method
  - [x] Add `CircleConfig::from_params()` method
- [x] **Comprehensive unit tests**
  - [x] Test clockwise orbit target calculation
  - [x] Test counterclockwise orbit target calculation
  - [x] Test center point calculation
  - [x] Test CircleConfig from params
  - [x] Test CircleConfig from store
  - [x] Test parameter validation
- [ ] **Performance verification**
  - [ ] Add timing measurement for target calculation
  - [ ] Verify < 1ms on RP2350 (manual test on hardware)
- [ ] **Documentation updates**
  - [ ] Update `docs/architecture.md` if needed
  - [ ] Add Circle mode to rover documentation

### Phase 3 Deliverables

- CIRC\_\* parameters registered and working
- Comprehensive test coverage for Circle mode
- Performance verified on target hardware

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- All FR-00112 acceptance criteria satisfied
- Parameters readable/writable via MAVLink
- All tests pass on host and embedded build succeeds

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] All FR-00112 acceptance criteria marked complete
- [ ] Performance measured on hardware (< 1ms target calculation)
- [ ] No `unsafe` and no vague naming (no "manager"/"util")

## Open Questions

- [ ] Optimal look-ahead time for different speeds? → Method: Empirical testing on hardware after implementation
- [ ] Integration with mode controller for mode switching? → Next step: Coordinate with T-xxxx mode controller task when created
