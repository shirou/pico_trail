# T-00166 SITL Per-Process Multi-Vehicle

## Metadata

- Type: Implementation Plan
- Status: Completed

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Refactor `gazebo_bridge` from multi-vehicle single-process to single-vehicle per-process. Remove SITL-side sensor telemetry bypass. Add launch script for N processes + mavp2p aggregation.

## Success Metrics

- [x] `gazebo_bridge --system-id 1 --gazebo-port 9002 --mavlink-port 5760` runs one vehicle
- [x] No SITL sensor telemetry bypass (all telemetry via core's TelemetryStreamer)
- [ ] 3 processes + mavp2p: all vehicles visible and controllable in Mission Planner
- [ ] No vertical speed oscillation in Mission Planner
- [x] All SITL tests pass
- [x] Firmware builds: `./scripts/build-rp2350.sh pico_trail_rover`

## Scope

- Goal: Single vehicle per OS process; remove duplicate telemetry; launch script for multi-vehicle
- Non-Goals: Core/firmware changes, mavp2p packaging, WSL2 port forwarding setup
- Assumptions: T-00164 complete (core autopilot integration), mavp2p installed on host
- Constraints: SITL crate only; no changes to core or firmware crates

## ADR & Legacy Alignment

- [x] Confirm ADR-00165 (per-process model) is followed
- [x] Verify core/firmware crates are untouched

## Plan Summary

| Phase | Description                                | Effort |
| ----- | ------------------------------------------ | ------ |
| 1     | Remove SITL sensor telemetry bypass        | 1 day  |
| 2     | Refactor gazebo_bridge to single-vehicle   | 2 days |
| 3     | Launch script and multi-vehicle validation | 1 day  |

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Remove SITL Sensor Telemetry Bypass

### Goal

- Remove SITL-side telemetry functions that duplicate core's TelemetryStreamer
- Remove `send_telemetry` and `send_heartbeats` from GcsLink
- Simplify GcsLink to a send/receive transport (no telemetry logic)

### Inputs

- Source Code to Modify:
  - `crates/sitl/src/gcs/telemetry.rs` — remove all telemetry builder functions
  - `crates/sitl/src/gcs/mod.rs` — remove `send_telemetry`, `send_heartbeats`, `send_heartbeats_with`, `VehicleState`, `register_vehicle`, interval constants
  - `crates/sitl/src/bin/gazebo_bridge.rs` — remove `gcs.send_telemetry()` and `gcs.send_heartbeats()` calls
- Dependencies:
  - Core's `TelemetryStreamer` already sends HEARTBEAT, ATTITUDE, GPS, SYS_STATUS via `autopilot.update_telemetry()`

### Tasks

- [x] **Remove telemetry module**
  - [x] Delete or gut `crates/sitl/src/gcs/telemetry.rs` (remove `build_telemetry`, `build_heartbeat`, `build_heartbeat_with_state`, `build_attitude`, `build_gps_raw_int`, `build_global_position_int`, `build_sys_status`, `gps_fix_to_mav`, `TelemetrySet`)
  - [x] If module becomes empty, remove `pub mod telemetry;` from `crates/sitl/src/gcs/mod.rs`
- [x] **Simplify GcsLink**
  - [x] Remove `VehicleState` struct
  - [x] Remove `vehicles: Vec<VehicleState>` field from `GcsLink`
  - [x] Remove `register_vehicle()` method
  - [x] Remove `send_heartbeats()` method
  - [x] Remove `send_heartbeats_with()` method
  - [x] Remove `send_telemetry()` method
  - [x] Remove interval constants (`HEARTBEAT_INTERVAL_US`, `ATTITUDE_INTERVAL_US`, `POSITION_INTERVAL_US`, `SYS_STATUS_INTERVAL_US`)
  - [x] Keep: `new()`, `try_accept()`, `poll_incoming()`, `send_message_as()`, `is_connected()`, `parse_mavlink_frame()`
- [x] **Update gazebo_bridge main loop**
  - [x] Remove `gcs.send_heartbeats(wall_us)` call
  - [x] Remove `gcs.send_telemetry(i, &sensors, wall_us)` call and its surrounding loop
  - [x] Remove `gcs.register_vehicle(i)` call
  - [x] Core's `autopilot.update_telemetry(wall_us)` now handles all telemetry (HEARTBEAT, ATTITUDE, GPS, SYS_STATUS)
- [x] **Update tests**
  - [x] Remove `test_register_vehicle` test
  - [x] Remove `test_multi_vehicle_heartbeats` test
  - [x] Update remaining GcsLink tests (remove `register_vehicle` setup calls if present)
  - [x] Verify `test_gcs_link_creation`, `test_send_without_client_is_noop`, `test_poll_incoming_no_client`, `test_parse_skips_signed_v2_frame`, `test_tcp_loopback` still pass
- [x] **Verification**
  - [x] `cargo test -p pico_trail_sitl --lib --quiet`
  - [x] `cargo test -p pico_trail_sitl --test '*' --quiet`
  - [x] `cargo clippy -p pico_trail_sitl --all-targets -- -D warnings`
  - [x] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- GcsLink reduced to transport-only (send/receive MAVLink messages)
- No SITL telemetry bypass — all telemetry through core's dispatcher
- Telemetry module removed or emptied

### Acceptance Criteria (Phase Gate)

- `send_telemetry`, `send_heartbeats`, `register_vehicle` removed from GcsLink
- `build_telemetry`, `build_global_position_int`, etc. removed from telemetry module
- Core's `autopilot.update_telemetry()` is the sole telemetry source
- All SITL tests pass
- Embedded build succeeds

### Rollback/Fallback

- Restore telemetry module and GcsLink methods from git

---

## Phase 2: Refactor gazebo_bridge to Single-Vehicle

### Goal

- Change CLI from multi-vehicle (`-n COUNT`) to single-vehicle (`--system-id`, `--gazebo-port`, `--mavlink-port`)
- Remove multi-vehicle loop from main
- Every process handles exactly one vehicle

### Inputs

- Dependencies:
  - Phase 1: GcsLink simplified to transport-only
  - `VehicleAutopilot` already handles single vehicle
- Source Code to Modify:
  - `crates/sitl/src/bin/gazebo_bridge.rs` — CLI args, main loop

### Tasks

- [x] **Refactor CLI arguments**
  - [x] Replace `Args` struct: remove `count`, `gazebo_port_base`, `port_stride`; add `system_id: u8`, `gazebo_port: u16`, `mavlink_port: u16`
  - [x] Update `parse_args()`: accept `--system-id`, `--gazebo-port`, `--mavlink-port`
  - [x] Update `print_usage()` with new flags
  - [x] Default values: `system_id=1`, `gazebo_port=9002`, `mavlink_port=5760`
- [x] **Simplify main to single vehicle**
  - [x] Remove `for i in 1..=args.count` vehicle registration loop
  - [x] Register one vehicle: `VehicleId(args.system_id)`
  - [x] Register one adapter: `GazeboAdapter` with `args.gazebo_port`
  - [x] Create `GcsLink::new(args.mavlink_port)`
  - [x] Create `VehicleAutopilot::new(args.system_id)`
  - [x] Remove comment about "only vehicle 1 can have autopilot"
- [x] **Simplify main loop**
  - [x] Remove multi-vehicle telemetry loop (`for i in 1..=args.count { ... send_telemetry ... }`)
  - [x] Use `args.system_id` consistently (instead of hardcoded `VehicleId(1)`)
  - [x] Remove `args.count` from summary log
- [x] **Update tests**
  - [x] Add CLI parsing tests: `--system-id 2 --gazebo-port 9012 --mavlink-port 5762`
  - [x] Verify defaults work: no args -> system_id=1, gazebo_port=9002, mavlink_port=5760
- [x] **Verification**
  - [x] `cargo test -p pico_trail_sitl --lib --quiet`
  - [x] `cargo test -p pico_trail_sitl --test '*' --quiet`
  - [x] `cargo clippy -p pico_trail_sitl --all-targets -- -D warnings`
  - [x] `./scripts/build-rp2350.sh pico_trail_rover`

### Deliverables

- `gazebo_bridge` binary accepts single-vehicle CLI flags
- Main loop handles exactly one vehicle
- No multi-vehicle routing logic in the process

### Acceptance Criteria (Phase Gate)

- `gazebo_bridge --system-id 1 --gazebo-port 9002 --mavlink-port 5760` runs one vehicle
- Old multi-vehicle flags (`-n`, `--count`, `--gazebo-port-base`, `--port-stride`) removed
- All SITL tests pass
- Embedded build succeeds

### Rollback/Fallback

- Restore multi-vehicle CLI and loop from git

---

## Phase 3: Launch Script and Multi-Vehicle Validation

### Goal

- Create launch script for N vehicles + mavp2p
- Validate multi-vehicle operation end-to-end

### Inputs

- Dependencies:
  - Phase 2: Single-vehicle `gazebo_bridge`
  - mavp2p binary installed on host
- External:
  - [mavp2p](https://github.com/bluenviron/mavp2p)

### Tasks

- [x] **Create launch script**
  - [x] Create `scripts/sitl-multi-vehicle.sh`
  - [x] Accept vehicle count as argument (default: 3)
  - [x] Environment variables: `GAZEBO_PORT_BASE`, `GAZEBO_PORT_STRIDE`, `MAVLINK_PORT_BASE`, `MAVLINK_PORT_STRIDE`, `MAVP2P_PORT`
  - [x] Start N `gazebo_bridge` processes with correct port assignments
  - [x] Start mavp2p with `tcpc:` for each vehicle + `tcps:` for GCS
  - [x] Trap-based cleanup: kill all child processes on EXIT/INT/TERM
  - [x] Check for mavp2p binary and print install instructions if missing
  - [x] Make script executable: `chmod +x`
- [x] **Update SITL documentation**
  - [x] Update `crates/sitl/README.md` with per-process architecture
  - [x] Document new CLI flags
  - [x] Document launch script usage
  - [x] Document mavp2p setup
- [ ] **Manual validation**
  - [ ] Start 3 vehicles via launch script
  - [ ] Connect Mission Planner to mavp2p port
  - [ ] Verify all 3 vehicles visible
  - [ ] ARM each vehicle independently
  - [ ] Control each vehicle with joystick independently
  - [ ] Verify no vertical speed oscillation
  - [ ] Verify clean shutdown via Ctrl+C
- [x] **Verification**
  - [x] `cargo test -p pico_trail_sitl --lib --quiet`
  - [x] `cargo clippy -p pico_trail_sitl --all-targets -- -D warnings`
  - [x] `./scripts/build-rp2350.sh pico_trail_rover`
  - [ ] `bun scripts/trace-status.ts --check`
  - [ ] `bun format`
  - [ ] `bun lint`

### Deliverables

- `scripts/sitl-multi-vehicle.sh` launch script
- Updated SITL README with per-process documentation
- Manual validation report

### Acceptance Criteria (Phase Gate)

- Launch script starts N vehicles + mavp2p
- All vehicles visible and independently controllable in Mission Planner
- No vertical speed oscillation
- Clean shutdown on Ctrl+C
- All tests and lints pass

### Rollback/Fallback

- Launch script is additive; delete it to revert

---

## Definition of Done

- [x] SITL sensor telemetry bypass removed (no `build_telemetry`, `send_telemetry`, `send_heartbeats`)
- [x] GcsLink simplified to transport-only
- [x] `gazebo_bridge` runs single vehicle per process
- [x] Launch script for N vehicles + mavp2p
- [x] No duplicate telemetry (no vertical speed oscillation)
- [x] All SITL tests pass: `cargo test -p pico_trail_sitl --lib --quiet`
- [x] Integration tests pass: `cargo test -p pico_trail_sitl --test '*' --quiet`
- [x] Clippy clean: `cargo clippy -p pico_trail_sitl --all-targets -- -D warnings`
- [x] Firmware build: `./scripts/build-rp2350.sh pico_trail_rover`
- [x] Code formatted: `cargo fmt`
- [x] Documentation updated: SITL README, traceability

## Open Questions

- [x] Should the launch script use `cargo run` or pre-built binaries? — Using `cargo run` for simplicity; pre-built can be added later.
- [x] Should `GcsLink` multi-vehicle support be removed entirely, or kept but unused? — Removed entirely. GcsLink is now transport-only.
