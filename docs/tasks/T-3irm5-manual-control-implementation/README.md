# T-3irm5 Manual Control Implementation

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-yqeju-manual-control-implementation](../../analysis/AN-yqeju-manual-control-implementation.md)
- Related Requirements:
  - [FR-q2sjt-control-mode-framework](../../requirements/FR-q2sjt-control-mode-framework.md)
  - [FR-uo1p5-actuator-abstraction](../../requirements/FR-uo1p5-actuator-abstraction.md)
  - [FR-993xy-rc-channels-processing](../../requirements/FR-993xy-rc-channels-processing.md)
  - [FR-uk0us-manual-mode](../../requirements/FR-uk0us-manual-mode.md)
  - [NFR-v6kvd-vehicle-layer-memory](../../requirements/NFR-v6kvd-vehicle-layer-memory.md)
  - [NFR-jng15-actuator-failsafe](../../requirements/NFR-jng15-actuator-failsafe.md)
  - [NFR-kqvyf-manual-control-latency](../../requirements/NFR-kqvyf-manual-control-latency.md)
- Related ADRs:
  - [ADR-w9zpl-control-mode-architecture](../../adr/ADR-w9zpl-control-mode-architecture.md)
  - [ADR-b8snw-actuator-abstraction-rover](../../adr/ADR-b8snw-actuator-abstraction-rover.md)
  - [ADR-ea7fw-rc-input-processing](../../adr/ADR-ea7fw-rc-input-processing.md)
- Associated Design Document:
  - [design.md](./design.md)
- Associated Plan Document:
  - [plan.md](./plan.md)

## Summary

Implement manual control capability for rover vehicles, including RC input processing from MAVLink RC_CHANNELS messages, trait-based control mode framework, actuator abstraction with safety enforcement, and Manual mode implementation enabling operator control via Mission Planner joystick.

## Scope

- In scope:
  - Control mode trait and framework (`VehicleMode` trait, `ModeManager`)
  - RC input processing from MAVLink RC_CHANNELS messages
  - RC channel normalization (0-65535 → -1.0 to +1.0)
  - RC timeout detection and failsafe (1 second timeout)
  - Actuator abstraction trait (`ActuatorInterface`)
  - Normalized actuator commands (steering/throttle -1.0 to +1.0)
  - PWM conversion (normalized → 1000-2000 μs pulse width → duty cycle)
  - Armed state enforcement at actuator layer
  - Actuator calibration parameters (min/max/trim)
  - Manual mode implementation (direct RC pass-through)
  - Vehicle control task (50 Hz mode execution)
  - MAVLink DO_SET_MODE command handler
  - Mode transition validation and logging
  - Safety: All actuators neutral when disarmed
  - Safety: Actuators neutral on RC timeout
- Out of scope:
  - Physical RC receiver support (SBUS/PPM) - MAVLink RC only
  - Other control modes (Hold, Auto, RTL, Guided) - deferred to future tasks
  - RC input filtering/smoothing - simple pass-through initially
  - Advanced actuator features (rate limiting, deadband) - deferred
  - Differential steering or skid-steer mixing - Ackermann only
  - RC override (switch from Auto to Manual via RC) - deferred

## Success Metrics

- **Latency**: RC input to actuator response < 100ms (NFR-kqvyf)
- **Safety**: 100% of actuator commands check armed state (automated test required)
- **Safety**: Actuators neutral within 20ms of disarm event (NFR-jng15)
- **Memory**: Vehicle layer RAM usage < 5 KB (NFR-v6kvd)
- **Compatibility**: Mission Planner joystick controls steering and throttle
- **Reliability**: RC timeout detected within 1 second, fail-safe to neutral
- **Code Quality**: All tests pass (`cargo test --lib --quiet`), no clippy warnings
- **Mode Execution**: Active mode updates at 50 Hz without errors

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../../templates/README.md#task-template-taskmd) in the templates README.
