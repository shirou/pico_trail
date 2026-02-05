# T-00159 SITL Gazebo Adapter

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../../analysis/AN-00147-sitl-simulator-integration.md)
- Related Requirements:
  - [FR-00151-sitl-gazebo-adapter](../../requirements/FR-00151-sitl-gazebo-adapter.md)
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

Implement `GazeboAdapter` that communicates with Gazebo Harmonic via UDP/JSON protocol, compatible with the ardupilot_gazebo plugin. This adapter enables full physics simulation for realistic testing of navigation, sensor fusion, and control loops. This task can be implemented in parallel with T-00158.

## Scope

- In scope:
  - `GazeboAdapter` implementing `SimulatorAdapter` trait
  - UDP socket management for sensor and actuator channels
  - JSON parsing for ardupilot_gazebo sensor format
  - JSON serialization for actuator commands
  - Normalized-to-PWM motor value conversion
  - Connection/reconnection/timeout handling
  - Unit tests with mock sockets
  - Gazebo setup documentation
- Out of scope:
  - LightweightAdapter (T-00158)
  - SitlPlatform (T-00158)
  - Multi-vehicle routing (T-00160)
  - CI integration (Gazebo not available in CI)

## Success Metrics

| Metric                 | Target                                   |
| ---------------------- | ---------------------------------------- |
| Protocol compatibility | Works with ardupilot_gazebo plugin       |
| JSON parsing           | All sensor types parsed correctly        |
| PWM conversion         | Normalized \[-1,1] ↔ PWM \[1000,2000]   |
| Timeout handling       | Configurable timeout with error recovery |
| Unit tests             | JSON parse/serialize tests pass (mock)   |
