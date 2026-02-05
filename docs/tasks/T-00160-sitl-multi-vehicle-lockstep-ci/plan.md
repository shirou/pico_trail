# T-00160 SITL Multi-Vehicle, Lockstep and CI Integration Plan

## Metadata

- Type: Implementation Plan
- Status: Draft

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

- [ ] **Implement TimeCoordinator**
  - [ ] Track sim_time_us
  - [ ] Lockstep: wait for all adapters before advancing
  - [ ] Free-running: advance at wall-clock rate
  - [ ] Scaled: multiply wall-clock by factor
- [ ] **Implement multi-adapter step**
  - [ ] Parallel adapter.step() calls
  - [ ] Collect sensors from all adapters
  - [ ] Route by vehicle_id
- [ ] **Implement sensor routing**
  - [ ] Match `sensor_data.vehicle_id` to vehicle instance
  - [ ] Inject sensors to correct SitlPlatform
- [ ] **Implement actuator aggregation**
  - [ ] Collect commands from all vehicles
  - [ ] Send to respective adapters
- [ ] **Implement per-vehicle MAVLink**
  - [ ] Port assignment: 14550 + vehicle_id
  - [ ] UDP socket per vehicle
  - [ ] Route MAVLink messages by port
- [ ] **Unit tests**
  - [ ] Test multi-vehicle spawn (10 vehicles)
  - [ ] Test sensor routing (correct vehicle receives data)
  - [ ] Test lockstep timing (all adapters step together)
  - [ ] Test MAVLink port assignment
- [ ] **Integration tests**
  - [ ] 3 vehicles on LightweightAdapter
  - [ ] Deterministic multi-vehicle scenario
- [ ] **Verification**
  - [ ] `cargo fmt`
  - [ ] `cargo clippy --all-targets -- -D warnings`
  - [ ] `cargo test -p pico_trail_sitl --lib`

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

- [ ] **Add feature flag to firmware**
  - [ ] `sitl` feature in `crates/firmware/Cargo.toml`
  - [ ] Conditional compilation for SITL tests
- [ ] **Create SITL test suite**
  - [ ] Test Manual mode in SITL
  - [ ] Test Guided mode navigation
  - [ ] Test mode transitions
- [ ] **Update CI workflow**
  - [ ] Add `cargo test --features sitl` step
  - [ ] No Gazebo dependency in CI
- [ ] **Create example**
  - [ ] `basic_sitl.rs` demonstrating bridge setup
  - [ ] Spawn vehicle, run simulation loop
- [ ] **Write documentation**
  - [ ] SITL usage guide
  - [ ] Adapter creation guide
  - [ ] Gazebo setup instructions
- [ ] **Final verification**
  - [ ] `cargo fmt`
  - [ ] `cargo clippy --all-targets -- -D warnings`
  - [ ] `cargo test --all-features`
  - [ ] CI workflow passes
- [ ] **Update traceability**
  - [ ] `bun scripts/trace-status.ts --check`
  - [ ] `bun scripts/trace-status.ts --write`

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

- [ ] All phases completed
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --all-features`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover` (verify no impact on embedded)
- [ ] CI workflow passes
- [ ] No `unsafe` code in sitl crate
- [ ] Documentation complete
- [ ] Plan checkboxes marked
- [ ] Task README status updated to Implementation Complete
- [ ] Traceability check: `bun scripts/trace-status.ts --check`
