# T-00164 SITL Autopilot Core Integration

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../../analysis/AN-00147-sitl-simulator-integration.md)
- Related Requirements:
  - [FR-00162-sitl-gcs-command-reception](../../requirements/FR-00162-sitl-gcs-command-reception.md)
  - [FR-00163-sitl-autopilot-loop-integration](../../requirements/FR-00163-sitl-autopilot-loop-integration.md)
  - [FR-00154-sitl-platform-trait](../../requirements/FR-00154-sitl-platform-trait.md)
- Related ADRs:
  - [ADR-00161-sitl-autopilot-integration-layer](../../adr/ADR-00161-sitl-autopilot-integration-layer.md)
  - [ADR-00156-sitl-pluggable-adapter-architecture](../../adr/ADR-00156-sitl-pluggable-adapter-architecture.md)
- Prerequisite Tasks:
  - [T-00160-sitl-multi-vehicle-lockstep-ci](../T-00160-sitl-multi-vehicle-lockstep-ci/README.md)
- Dependent Tasks: N/A
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Add `embassy-sync` and `embassy-futures` as optional dependencies of `pico_trail_core` behind an `embassy` feature flag, then migrate the autopilot integration layer (SharedState, SystemState, VehicleType, MessageDispatcher, ModeManager, mode implementations) from firmware to core. This enables SITL to use the same autopilot logic as firmware, providing full GCS command reception and closed-loop control with all rover modes.

## Scope

- In scope:
  - Add `embassy` feature flag to core crate with embassy-sync and embassy-futures
  - Migrate SharedState trait + EmbassyState + MockState to core
  - Migrate SystemState and supporting types (ArmedState, FlightMode, BatteryState, AttitudeState, HomePosition, GpsPosition) to core
  - Migrate VehicleType trait + FlightModeOps + implementations to core
  - Migrate FlashInterface trait to core (pure interface; implementations stay in firmware)
  - Migrate ParamHandler to core (refactored constructor, generic over FlashInterface)
  - Migrate MessageDispatcher + all handlers (param, command, telemetry, mission, rc_input, navigation) to core
  - Migrate status_notifier to core
  - Migrate ModeManager and mode implementations (Manual, Auto, Guided, RTL, Loiter, Circle, SmartRTL) to core
  - Update firmware imports to use core modules
  - SITL integration: GcsLink delegates to core's MessageDispatcher
  - Closed-loop integration: GCS command → mode → actuator → Gazebo → sensor → telemetry → GCS
  - Integration tests with LightweightAdapter
- Out of scope:
  - Firmware-specific parameter modules (WifiParams, BoardParams — register_defaults stays in firmware)
  - Embassy tasks that depend on embassy-time (parameter saver, arming tasks)
  - Platform trait and device drivers (remain in firmware)
  - Transport layers (UART, UDP, WiFi) remain platform-specific

## Success Metrics

| Metric                  | Target                                                                    |
| ----------------------- | ------------------------------------------------------------------------- |
| Core portability        | `cargo test -p pico_trail_core --features embassy --lib` passes           |
| Firmware compatibility  | `./scripts/build-rp2350.sh pico_trail_rover` compiles without regression  |
| GCS command reception   | ARM/DISARM, SET_MODE, RC_CHANNELS_OVERRIDE, MANUAL_CONTROL via dispatcher |
| All rover modes in SITL | Manual, Auto, Guided, RTL, Loiter, Circle, SmartRTL work                  |
| End-to-end latency      | <100ms GCS command → telemetry update at 100Hz                            |
| Multi-vehicle control   | Independent control of 3+ vehicles from one GCS                           |
| Gazebo validation       | Vehicle moves in Gazebo when steered from Mission Planner                 |
