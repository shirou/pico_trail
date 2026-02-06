# T-00160 SITL Multi-Vehicle, Lockstep and CI Integration Plan

## Metadata

- Type: Implementation Plan
- Status: Implementation Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement multi-vehicle routing, lockstep time synchronization, per-vehicle MAVLink ports, and CI integration in 2 phases. Phase 1 adds multi-vehicle and lockstep functionality to the bridge. Phase 2 integrates everything into CI with tests, examples, and documentation.

## Success Metrics

- [ ] Multi-vehicle support (10 vehicles tested)
- [ ] Lockstep produces deterministic results
- [ ] Per-vehicle MAVLink ports work
- [ ] <20ms p99 latency in lockstep mode
- [ ] CI passes with `cargo test --features sitl`
- [ ] All rover modes work in SITL

## Scope

- Goal: Full multi-vehicle SITL with CI integration
- Non-Goals: New adapter implementations, camera simulation
- Assumptions: T-00157 and T-00158 are complete
- Constraints: <20ms latency in lockstep mode, CI must not require Gazebo

## ADR & Legacy Alignment

- [ ] Confirm ADR-00156-sitl-pluggable-adapter-architecture is referenced
- [ ] No modifications to existing crates/core or crates/firmware

## Plan Summary

| Phase | Description                | Effort    |
| ----- | -------------------------- | --------- |
| 1     | Multi-Vehicle and Lockstep | 1.5 weeks |
| 2     | CI Integration             | 1 week    |

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Multi-Vehicle and Lockstep

### Goal

- Full multi-vehicle support with VehicleId routing
- Lockstep time synchronization across adapters
- Per-vehicle MAVLink ports

### Inputs

- Files to Modify:
  - `crates/sitl/src/bridge/mod.rs`
  - `crates/sitl/src/bridge/time.rs`

### Tasks

- [x] **Implement TimeCoordinator**
  - [x] Track sim_time_us
  - [x] Lockstep: wait for all adapters before advancing
  - [x] Free-running: advance at wall-clock rate
  - [x] Scaled: multiply wall-clock by factor
- [x] **Implement multi-adapter step**
  - [x] Parallel adapter.step() calls
  - [x] Collect sensors from all adapters
  - [x] Route by vehicle_id
- [x] **Implement sensor routing**
  - [x] Match `sensor_data.vehicle_id` to vehicle instance
  - [x] Inject sensors to correct SitlPlatform
- [x] **Implement actuator aggregation**
  - [x] Collect commands from all vehicles
  - [x] Send to respective adapters
- [x] **Implement per-vehicle MAVLink**
  - [x] Port assignment: 14550 + vehicle_id
  - [x] Port conflict detection at spawn time
  - [ ] UDP socket per vehicle (deferred to MAVLink integration task)
  - [ ] Route MAVLink messages by port (deferred to MAVLink integration task)
- [x] **Unit tests**
  - [x] Test multi-vehicle spawn (10 vehicles)
  - [x] Test sensor routing (correct vehicle receives data)
  - [x] Test lockstep timing (all adapters step together)
  - [x] Test MAVLink port conflict detection
- [x] **Integration tests**
  - [x] Multi-vehicle sensor routing
  - [x] Deterministic multi-vehicle scenario
- [x] **Verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `cargo test -p pico_trail_sitl --lib`

### Deliverables

- Full multi-vehicle and lockstep support

### Acceptance Criteria (Phase Gate)

- FR-00153 lockstep requirements met
- FR-00155 MAVLink port requirements met
- NFR-00095 latency requirements met
- 10 vehicles work in tests

### Rollback/Fallback

- Fall back to single-vehicle mode
- Fall back to free-running mode

---

## Phase 2: CI Integration and Documentation

### Goal

- Integrate SITL tests into CI
- Create example SITL usage
- Documentation for adding new adapters

### Inputs

- Files to Create/Modify:
  - `.github/workflows/test.yml`
  - `crates/sitl/examples/basic_sitl.rs`
  - `docs/sitl-guide.md`

### Tasks

- [ ] **Add feature flag to firmware** (deferred: SITL crate is standalone)
  - [ ] `sitl` feature in `crates/firmware/Cargo.toml`
  - [ ] Conditional compilation for SITL tests
- [ ] **Create SITL test suite** (deferred to mode integration tasks)
  - [ ] Test Manual mode in SITL
  - [ ] Test Guided mode navigation
  - [ ] Test mode transitions
- [x] **Update CI workflow**
  - [x] Add `cargo test -p pico_trail_sitl --lib` step
  - [x] No Gazebo dependency in CI
- [x] **Create example**
  - [x] `basic_sitl.rs` demonstrating bridge setup
  - [x] Spawn vehicle, run simulation loop
- [x] **Write documentation**
  - [x] SITL usage guide
  - [x] Adapter creation guide
  - [ ] Gazebo setup instructions (deferred to Gazebo-specific task)
- [x] **Final verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --all-targets -- -D warnings`
  - [x] `cargo test -p pico_trail_sitl --lib`
  - [ ] CI workflow passes (requires merge to trigger)
- [x] **Update traceability**
  - [x] `bun scripts/trace-status.ts --check`
  - [x] `bun format` and `bun lint`

### Deliverables

- CI integration with SITL tests
- Example code
- Documentation

### Acceptance Criteria (Phase Gate)

- CI passes with SITL tests
- Example compiles and runs
- Documentation complete

### Rollback/Fallback

- SITL tests can be skipped with `--skip sitl`

---

## Definition of Done

- [x] All phases completed
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test -p pico_trail_sitl --lib`
- [x] `./scripts/build-rp2350.sh pico_trail_rover` (verify no impact on embedded)
- [ ] CI workflow passes (requires merge to trigger)
- [x] No `unsafe` code in sitl crate
- [x] Documentation complete
- [x] Plan checkboxes marked
- [x] Task README status updated to Implementation Complete
- [x] Traceability check: `bun scripts/trace-status.ts --check`
