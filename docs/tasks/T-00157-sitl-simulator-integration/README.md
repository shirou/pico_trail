# T-00157 SITL Core Abstractions and Bridge

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../../analysis/AN-00147-sitl-simulator-integration.md)
- Related Requirements:
  - [FR-00148-sitl-simulator-adapter-trait](../../requirements/FR-00148-sitl-simulator-adapter-trait.md)
  - [FR-00149-sitl-multi-adapter-registration](../../requirements/FR-00149-sitl-multi-adapter-registration.md)
  - [NFR-00094-sitl-adapter-trait-safety](../../requirements/NFR-00094-sitl-adapter-trait-safety.md)
  - [NFR-00097-sitl-adapter-extensibility](../../requirements/NFR-00097-sitl-adapter-extensibility.md)
- Related ADRs:
  - [ADR-00156-sitl-pluggable-adapter-architecture](../../adr/ADR-00156-sitl-pluggable-adapter-architecture.md)
- Prerequisite Tasks: None
- Dependent Tasks:
  - [T-00158-sitl-platform-and-lightweight-adapter](../T-00158-sitl-platform-and-lightweight-adapter/README.md)
  - [T-00159-sitl-gazebo-adapter](../T-00159-sitl-gazebo-adapter/README.md)
  - [T-00160-sitl-multi-vehicle-lockstep-ci](../T-00160-sitl-multi-vehicle-lockstep-ci/README.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Create the `crates/sitl` crate with core abstractions (`SimulatorAdapter` trait, normalized data types, error types) and the `SitlBridge` orchestrator (adapter registry, vehicle management, time mode). This task establishes the foundation that all subsequent SITL tasks depend on.

## Scope

- In scope:
  - New `crates/sitl` crate structure and workspace integration
  - `SimulatorAdapter` trait with async methods (object-safe, Send + Sync)
  - Normalized data types (`SensorData`, `ActuatorCommands`, `VehicleId`)
  - `SimulatorCapabilities` for feature discovery
  - `SimulatorError` enum with `thiserror`
  - `SitlBridge` with adapter registry (register, unregister, list)
  - Vehicle management (spawn, despawn, assign to adapter)
  - `TimeMode` enum (FreeRunning, Lockstep, Scaled)
  - `VehicleConfig` and `VehicleInstance` structs
- Out of scope:
  - Platform trait implementation (T-00158)
  - Adapter implementations (T-00158, T-00159)
  - Multi-vehicle routing and lockstep logic (T-00160)
  - CI integration (T-00160)

## Success Metrics

| Metric              | Target                                        |
| ------------------- | --------------------------------------------- |
| Trait object safety | `Box<dyn SimulatorAdapter>` compiles          |
| Adapter registry    | Register, unregister, list adapters           |
| Vehicle management  | Spawn, despawn, assign vehicles               |
| New adapter effort  | 0 lines core change (NFR-00097)               |
| Error handling      | All error cases covered with `SimulatorError` |
