# T-tto4f Navigation Controller

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement a navigation controller subsystem that converts GPS position targets into steering and throttle commands. This is the foundation for all autonomous navigation modes (Guided, Auto, RTL).

## Success Metrics

- [ ] Navigation calculations complete within 2ms on RP2350
- [ ] Steering output always in \[-1.0, +1.0], throttle in \[0.0, 1.0]
- [ ] Unit tests pass for all navigation functions
- [ ] All existing tests pass; no regressions in vehicle control

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

- [ ] **Module scaffolding**
  - [ ] Create `src/subsystems/navigation/mod.rs`
  - [ ] Create `src/subsystems/navigation/types.rs`
  - [ ] Create `src/subsystems/navigation/geo.rs`
  - [ ] Add navigation module to `src/subsystems/mod.rs`
- [ ] **Type definitions**
  - [ ] Define `PositionTarget` struct
  - [ ] Define `NavigationOutput` struct
  - [ ] Define `SimpleNavConfig` struct with defaults
- [ ] **Geo calculations**
  - [ ] Implement `calculate_bearing(lat1, lon1, lat2, lon2) -> f32`
  - [ ] Implement `calculate_distance(lat1, lon1, lat2, lon2) -> f32`
  - [ ] Implement `wrap_180(angle: f32) -> f32`
  - [ ] Implement `wrap_360(angle: f32) -> f32`
- [ ] **Unit tests for geo functions**
  - [ ] Test bearing: north, south, east, west directions
  - [ ] Test bearing: diagonal directions
  - [ ] Test distance: known reference points
  - [ ] Test wrap_180: boundary cases
  - [ ] Test wrap_360: boundary cases

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

- [ ] **NavigationController trait**
  - [ ] Define `NavigationController` trait with `update()` and `reset()` methods
  - [ ] Document trait contract and usage
- [ ] **SimpleNavigationController**
  - [ ] Implement `SimpleNavigationController` struct
  - [ ] Implement `new()` with default config
  - [ ] Implement `with_config()` for custom configuration
  - [ ] Implement `update()` method:
    - [ ] Calculate bearing to target
    - [ ] Calculate distance to target
    - [ ] Calculate heading error (wrapped to ±180°)
    - [ ] Calculate steering from heading error
    - [ ] Calculate throttle with approach slowdown
    - [ ] Detect arrival (within WP_RADIUS)
  - [ ] Implement `reset()` method
- [ ] **Output range safety**
  - [ ] Ensure steering clamped to \[-1.0, +1.0]
  - [ ] Ensure throttle clamped to \[0.0, 1.0]
  - [ ] Handle NaN/infinity inputs safely

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

- [ ] **Controller unit tests**
  - [ ] Test at_target detection (within/outside WP_RADIUS)
  - [ ] Test steering: target ahead (0° error)
  - [ ] Test steering: target to left (negative error)
  - [ ] Test steering: target to right (positive error)
  - [ ] Test steering: target behind (180° error)
  - [ ] Test throttle: far from target (full throttle)
  - [ ] Test throttle: in approach zone (reduced)
  - [ ] Test throttle: at target (zero)
- [ ] **Property-based tests** (if proptest available)
  - [ ] Steering always in \[-1.0, +1.0] for any heading/bearing
  - [ ] Throttle always in \[0.0, 1.0] for any distance
  - [ ] wrap_180 always returns value in \[-180, 180]
- [ ] **Edge cases**
  - [ ] Current position equals target position
  - [ ] Very large distances (>1000km)
  - [ ] Positions near poles (high latitude)
  - [ ] Positions crossing date line (longitude wrap)
- [ ] **Documentation**
  - [ ] Add docstrings to all public functions
  - [ ] Add usage example in module documentation

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
- [ ] Documentation updated if needed
- [ ] ADRs referenced and followed
- [ ] No `unsafe` code
- [ ] No vague naming ("manager"/"util")

## Open Questions

- [ ] Should GPS heading (COG) filtering be added to controller? → Method: Test with real GPS, observe noise level
- [ ] Need to add heading hold when stationary? → Next step: Defer to Guided mode implementation

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
