# T-00166 SITL Per-Process Multi-Vehicle

## Metadata

- Type: Task
- Status: Completed

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../../analysis/AN-00147-sitl-simulator-integration.md)
- Related Requirements:
  - [FR-00150-sitl-multi-vehicle-instances](../../requirements/FR-00150-sitl-multi-vehicle-instances.md)
  - [FR-00155-sitl-vehicle-mavlink-ports](../../requirements/FR-00155-sitl-vehicle-mavlink-ports.md)
  - [FR-00163-sitl-autopilot-loop-integration](../../requirements/FR-00163-sitl-autopilot-loop-integration.md)
- Related ADRs:
  - [ADR-00165-sitl-per-process-multi-vehicle](../../adr/ADR-00165-sitl-per-process-multi-vehicle.md)
  - [ADR-00161-sitl-autopilot-integration-layer](../../adr/ADR-00161-sitl-autopilot-integration-layer.md)
  - [ADR-00156-sitl-pluggable-adapter-architecture](../../adr/ADR-00156-sitl-pluggable-adapter-architecture.md)
- Prerequisite Tasks:
  - [T-00164-sitl-autopilot-core-integration](../T-00164-sitl-autopilot-core-integration/README.md)
- Dependent Tasks: N/A
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Refactor `gazebo_bridge` from a multi-vehicle single-process model to a single-vehicle per-process model. Each process owns one vehicle with its own autopilot state (SYSTEM_STATE, RC_INPUT, etc.), eliminating global state conflicts and duplicate telemetry. Remove SITL-side sensor telemetry bypass so all telemetry flows through core's dispatcher. Add a launch script to start N processes plus mavp2p for GCS aggregation.

## Scope

- In scope:
  - Refactor `gazebo_bridge` CLI to accept `--system-id`, `--gazebo-port`, `--mavlink-port` for single-vehicle operation
  - Remove multi-vehicle loop and multi-vehicle registration from `gazebo_bridge`
  - Remove SITL sensor telemetry functions (`build_telemetry`, `build_global_position_int`, `build_attitude`, `build_gps_raw_int`, `TelemetrySet`) from `sitl/gcs/telemetry.rs`
  - Remove `send_telemetry` method from `GcsLink`
  - Remove `send_heartbeats` / `send_heartbeats_with` from `GcsLink` (core dispatcher sends HEARTBEAT via TelemetryStreamer)
  - Simplify `GcsLink` to single-vehicle (remove `VehicleState` vector, multi-vehicle heartbeat logic)
  - Remove `register_vehicle` from `GcsLink` (single vehicle per process)
  - Create launch script for N processes + mavp2p aggregation
  - Update existing tests to reflect single-vehicle-per-process model
- Out of scope:
  - Changes to `pico_trail_core` or `pico_trail_firmware`
  - Changes to `SitlBridge` multi-vehicle API (still usable within single process for one vehicle)
  - mavp2p installation or packaging
  - WSL2 port forwarding configuration

## Success Metrics

| Metric                        | Target                                                                                 |
| ----------------------------- | -------------------------------------------------------------------------------------- |
| Single-vehicle per process    | `gazebo_bridge --system-id 1 --gazebo-port 9002 --mavlink-port 5760` runs one vehicle  |
| No duplicate telemetry        | Only core's TelemetryStreamer sends ATTITUDE, GPS, HEARTBEAT (no SITL bypass)          |
| Multi-vehicle via mavp2p      | 3 processes + mavp2p: all vehicles visible in Mission Planner with independent control |
| Independent autopilot         | Each vehicle can ARM, change modes, and execute missions independently                 |
| No vertical speed oscillation | Mission Planner shows stable vertical speed (no alternating relative_alt values)       |
| Firmware compatibility        | `./scripts/build-rp2350.sh pico_trail_rover` compiles without regression               |
| SITL tests pass               | `cargo test -p pico_trail_sitl --lib --quiet` and integration tests pass               |
