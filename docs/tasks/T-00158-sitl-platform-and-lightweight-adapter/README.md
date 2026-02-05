# T-00158 SITL Platform and Lightweight Adapter

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../../analysis/AN-00147-sitl-simulator-integration.md)
- Related Requirements:
  - [FR-00152-sitl-lightweight-adapter](../../requirements/FR-00152-sitl-lightweight-adapter.md)
  - [FR-00154-sitl-platform-trait](../../requirements/FR-00154-sitl-platform-trait.md)
  - [NFR-00096-sitl-lightweight-no-deps](../../requirements/NFR-00096-sitl-lightweight-no-deps.md)
- Related ADRs:
  - [ADR-00156-sitl-pluggable-adapter-architecture](../../adr/ADR-00156-sitl-pluggable-adapter-architecture.md)
- Prerequisite Tasks:
  - [T-00157-sitl-simulator-integration](../T-00157-sitl-simulator-integration/README.md)
- Dependent Tasks:
  - [T-00160-sitl-multi-vehicle-lockstep-ci](../T-00160-sitl-multi-vehicle-lockstep-ci/README.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement `SitlPlatform` (Platform trait for simulation) and `LightweightAdapter` (built-in differential drive kinematics with no external dependencies). Together these provide the minimum viable SITL: autopilot code runs unchanged against a simulated environment that can be tested in CI without any external simulator.

## Scope

- In scope:
  - `SitlPlatform` implementing `Platform` trait for simulation
  - Simulated peripherals: `SitlUart`, `SitlPwm`, `SitlGpio`, `SitlTimeSource`
  - Sensor injection from bridge to platform
  - Actuator command collection from platform to bridge
  - `LightweightAdapter` with differential drive kinematics
  - Configurable sensor noise (GPS, IMU, compass)
  - Deterministic mode with seeded RNG
  - Unit tests for kinematics and platform
- Out of scope:
  - GazeboAdapter (T-00159)
  - Multi-vehicle routing (T-00160)
  - Lockstep time synchronization logic (T-00160)
  - CI integration (T-00160)

## Success Metrics

| Metric              | Target                                       |
| ------------------- | -------------------------------------------- |
| Platform trait      | `SitlPlatform` passes Platform trait bounds  |
| Sensor injection    | Data flows from bridge to platform           |
| Actuator collection | Commands flow from platform to bridge        |
| Kinematics accuracy | Straight line, rotation, arc turn tests pass |
| Deterministic mode  | Same seed produces identical results         |
| No external deps    | `cargo test` sufficient (NFR-00096)          |
