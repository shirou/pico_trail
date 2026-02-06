# T-00160 SITL Multi-Vehicle, Lockstep and CI Integration

## Metadata

- Type: Task
- Status: Implementation Complete

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../../analysis/AN-00147-sitl-simulator-integration.md)
- Related Requirements:
  - [FR-00150-sitl-multi-vehicle-instances](../../requirements/FR-00150-sitl-multi-vehicle-instances.md)
  - [FR-00153-sitl-lockstep-synchronization](../../requirements/FR-00153-sitl-lockstep-synchronization.md)
  - [FR-00155-sitl-vehicle-mavlink-ports](../../requirements/FR-00155-sitl-vehicle-mavlink-ports.md)
  - [NFR-00095-sitl-latency](../../requirements/NFR-00095-sitl-latency.md)
- Related ADRs:
  - [ADR-00156-sitl-pluggable-adapter-architecture](../../adr/ADR-00156-sitl-pluggable-adapter-architecture.md)
- Prerequisite Tasks:
  - [T-00157-sitl-simulator-integration](../T-00157-sitl-simulator-integration/README.md)
  - [T-00158-sitl-platform-and-lightweight-adapter](../T-00158-sitl-platform-and-lightweight-adapter/README.md)
- Dependent Tasks: None
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement multi-vehicle support with VehicleId routing, lockstep time synchronization across adapters, per-vehicle MAVLink ports, and CI integration. This task brings together all SITL components into a fully functional simulation environment with deterministic testing capabilities.

## Scope

- In scope:
  - Multi-vehicle sensor routing by VehicleId
  - Lockstep time synchronization (TimeCoordinator)
  - Per-vehicle MAVLink UDP ports (14550 + vehicle_index)
  - Multi-adapter parallel step execution
  - Actuator command aggregation from multiple vehicles
  - CI workflow integration (`cargo test --features sitl`)
  - SITL example code (`basic_sitl.rs`)
  - SITL usage and adapter creation documentation
- Out of scope:
  - New adapter implementations (JSBSim, AirSim)
  - Vehicle-to-vehicle communication
  - Camera/vision simulation
  - Real-time performance guarantees

## Success Metrics

| Metric                | Target                                          |
| --------------------- | ----------------------------------------------- |
| Multi-vehicle support | Up to 10 vehicles tested                        |
| Lockstep determinism  | Same seed produces identical multi-step results |
| Latency (lockstep)    | <20ms p99 sensor-to-actuator (NFR-00095)        |
| CI integration        | `cargo test --features sitl` without Gazebo     |
| MAVLink ports         | Independent UDP port per vehicle                |
| Mode coverage         | All rover modes work in SITL                    |
