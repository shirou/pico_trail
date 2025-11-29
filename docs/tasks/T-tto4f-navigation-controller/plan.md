# T-tto4f Navigation Controller

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement a navigation controller subsystem that converts GPS position targets into steering and throttle commands. This is the foundation for all autonomous navigation modes (Guided, Auto, RTL).

## Success Metrics

- [x] Navigation calculations complete within 2ms on RP2350 (estimated \~300µs)
- [x] Steering output always in \[-1.0, +1.0], throttle in \[0.0, 1.0]
- [x] Unit tests pass for all navigation functions (48 tests)
- [x] All existing tests pass; no regressions in vehicle control

## Scope

- Goal: Implement SimpleNavigationController with bearing-based steering
- Non-Goals: Full L1 controller (future), mode integration (separate task)
- Assumptions: GPS state available via GpsNavigationState
- Constraints: no_std compatible, < 500 bytes RAM for controller state

## ADR & Legacy Alignment

- [x] Confirm ADR-wrcuk-navigation-controller-architecture is referenced
- [x] No existing navigation code to migrate

## Plan Summary

- Phase 1 – Module structure and geo calculations (bearing, distance, wrap)
- Phase 2 – NavigationController trait and SimpleNavigationController
- Phase 3 – Testing and verification

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Module Structure and Geo Calculations

### Goal

- Create navigation module structure
- Implement geographic calculation functions

### Inputs

- Documentation:
  - `docs/adr/ADR-wrcuk-navigation-controller-architecture.md` – Architecture reference
  - `docs/requirements/FR-2vbe8-navigation-controller.md` – Requirements
- Source Code to Reference:
  - `src/devices/gps.rs` – GpsPosition struct
  - `src/subsystems/` – Existing subsystem patterns
- Dependencies:
  - External crates: `libm` – Trigonometric functions for no_std

### Tasks

- [x] **Module scaffolding**
  - [x] Create `src/subsystems/navigation/mod.rs`
  - [x] Create `src/subsystems/navigation/types.rs`
  - [x] Create `src/subsystems/navigation/geo.rs`
  - [x] Add navigation module to `src/subsystems/mod.rs`
- [x] **Type definitions**
  - [x] Define `PositionTarget` struct
  - [x] Define `NavigationOutput` struct
  - [x] Define `SimpleNavConfig` struct with defaults
- [x] **Geo calculations**
  - [x] Implement `calculate_bearing(lat1, lon1, lat2, lon2) -> f32`
  - [x] Implement `calculate_distance(lat1, lon1, lat2, lon2) -> f32`
  - [x] Implement `wrap_180(angle: f32) -> f32`
  - [x] Implement `wrap_360(angle: f32) -> f32`
- [x] **Unit tests for geo functions**
  - [x] Test bearing: north, south, east, west directions
  - [x] Test bearing: diagonal directions
  - [x] Test distance: known reference points
  - [x] Test wrap_180: boundary cases
  - [x] Test wrap_360: boundary cases

### Deliverables

- `src/subsystems/navigation/mod.rs`
- `src/subsystems/navigation/types.rs`
- `src/subsystems/navigation/geo.rs`
- Unit tests for all geo functions

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet navigation
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- All geo functions implemented with correct formulas
- Unit tests pass for bearing, distance, and angle wrapping
- Module compiles for embedded target
- No clippy warnings

### Rollback/Fallback

- Revert commits if calculations are incorrect
- Reference ArduPilot source for formula verification

---

## Phase 2: NavigationController Implementation

### Phase 2 Goal

- Implement NavigationController trait
- Implement SimpleNavigationController (L1-lite)

### Phase 2 Inputs

- Dependencies:
  - Phase 1: geo.rs functions, types.rs structs
- Source Code to Create:
  - `src/subsystems/navigation/controller.rs` – Controller implementation

### Phase 2 Tasks

- [x] **NavigationController trait**
  - [x] Define `NavigationController` trait with `update()` and `reset()` methods
  - [x] Document trait contract and usage
- [x] **SimpleNavigationController**
  - [x] Implement `SimpleNavigationController` struct
  - [x] Implement `new()` with default config
  - [x] Implement `with_config()` for custom configuration
  - [x] Implement `update()` method:
    - [x] Calculate bearing to target
    - [x] Calculate distance to target
    - [x] Calculate heading error (wrapped to ±180°)
    - [x] Calculate steering from heading error
    - [x] Calculate throttle with approach slowdown
    - [x] Detect arrival (within WP_RADIUS)
  - [x] Implement `reset()` method
- [x] **Output range safety**
  - [x] Ensure steering clamped to \[-1.0, +1.0]
  - [x] Ensure throttle clamped to \[0.0, 1.0]
  - [x] Handle NaN/infinity inputs safely

### Phase 2 Deliverables

- `src/subsystems/navigation/controller.rs`
- NavigationController trait
- SimpleNavigationController implementation

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet navigation
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- Controller calculates correct steering for various heading errors
- Controller calculates correct throttle with approach slowdown
- Controller detects arrival within WP_RADIUS
- All outputs within specified ranges

### Phase 2 Rollback/Fallback

- If steering calculation incorrect, verify against ArduPilot formulas
- Simplify to pure proportional control if issues arise

---

## Phase 3: Testing and Integration

### Phase 3 Goal

- Comprehensive unit tests
- Property-based tests for range guarantees
- Embedded build verification

### Phase 3 Tasks

- [x] **Controller unit tests**
  - [x] Test at_target detection (within/outside WP_RADIUS)
  - [x] Test steering: target ahead (0° error)
  - [x] Test steering: target to left (negative error)
  - [x] Test steering: target to right (positive error)
  - [x] Test steering: target behind (180° error)
  - [x] Test throttle: far from target (full throttle)
  - [x] Test throttle: in approach zone (reduced)
  - [x] Test throttle: at target (zero)
- [x] **Property-based tests** (pseudo-property tests via iteration)
  - [x] Steering always in \[-1.0, +1.0] for any heading/bearing
  - [x] Throttle always in \[0.0, 1.0] for any distance
  - [x] wrap_180 always returns value in \[-180, 180]
- [x] **Edge cases**
  - [x] Current position equals target position
  - [x] Very large distances (>1000km)
  - [x] Positions near poles (high latitude)
  - [x] Positions crossing date line (longitude wrap)
- [x] **Documentation**
  - [x] Add docstrings to all public functions
  - [x] Add usage example in module documentation

### Phase 3 Deliverables

- Comprehensive test suite
- Property-based tests (if applicable)
- Documentation for public API

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet navigation
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- All unit tests pass
- Property tests verify output ranges
- Edge cases handled correctly
- Module compiles and links for embedded target

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

- [ ] Should GPS heading (COG) filtering be added to controller? → Method: Test with real GPS, observe noise level
- [ ] Need to add heading hold when stationary? → Next step: Defer to Guided mode implementation

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
