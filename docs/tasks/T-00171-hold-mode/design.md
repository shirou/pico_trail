# T-00171 Hold Mode Design

## Metadata

- Type: Design
- Status: Completed

## Links

- Associated Plan Document:
  - [T-00171-hold-mode-plan](plan.md)

## Overview

Implement `HoldMode` struct that implements the `Mode` trait with the simplest possible behavior: zero all actuator outputs. The `FlightMode::Hold` enum variant, MAVLink mapping, and `MotorControlRunner` handling already exist. This task fills the gap of a missing `Mode` trait implementation for use with `ModeExecutor`.

## Success Metrics

- [x] `HoldMode` struct implements `Mode` trait (enter/update/exit/name)
- [x] All unit tests pass with no regressions
- [x] Embedded build succeeds
- [x] `core::mem::size_of::<HoldMode>() <= 32` bytes

## Background and Current State

- Context: Hold mode is the simplest control mode and the primary failsafe fallback. It must always succeed entry and produce zero motor output.
- Current behavior: `FlightMode::Hold` enum exists at `crates/core/src/autopilot/state.rs:44`. `MotorControlRunner::step()` returns zero output for Hold at `crates/core/src/motor/runner.rs:73`. Battery failsafe uses `BatteryFailsafeAction::Hold` at `crates/core/src/autopilot/mavlink_runner.rs:77`. No `HoldMode` struct exists.
- Pain points: Cannot use Hold mode with `ModeExecutor::set_mode()` because there is no `Mode` trait implementation.
- Constraints: Must follow existing mode patterns (`ManualMode` as reference). Must work on embedded (RP2350) and host test targets.
- Related ADRs: N/A

## Proposed Design

### High-Level Architecture

```text
ModeExecutor
    │
    ├── set_mode(Box<dyn Mode>)
    │       │
    │       ▼
    │   HoldMode::enter()  ──► actuators.set_steering(0.0)
    │                       ──► actuators.set_throttle(0.0)
    │
    ├── execute(timestamp)
    │       │
    │       ▼
    │   HoldMode::update(dt) ──► actuators.set_steering(0.0)
    │                         ──► actuators.set_throttle(0.0)
    │
    └── set_mode(other_mode)
            │
            ▼
        HoldMode::exit()  ──► actuators.set_steering(0.0)
                           ──► actuators.set_throttle(0.0)
```

### Components

- `crates/core/src/mode/hold.rs`: New file containing `HoldMode` struct and `Mode` implementation
- `crates/core/src/mode/mod.rs`: Add `pub mod hold;` and re-export

### Data Flow

1. `ModeExecutor::set_mode()` calls `HoldMode::enter()` → zeros actuators
2. At 50 Hz, `ModeExecutor::execute()` calls `HoldMode::update(dt)` → zeros actuators
3. On mode exit, `ModeExecutor::set_mode()` calls `HoldMode::exit()` → zeros actuators

### Data Models and Types

```rust
use crate::servo::ActuatorInterface;
use super::traits::Mode;

pub struct HoldMode {
    actuators: Box<dyn ActuatorInterface>,
}
```

No additional types, enums, or state machines are needed.

### Error Handling

- `enter()`: Sets actuators to zero. Actuator write errors are propagated via `?`. No precondition checks (no GPS, RC, etc.).
- `update()`: Propagates actuator write errors via `?`.
- `exit()`: Propagates actuator write errors via `?`.
- No custom error types needed; uses existing `&'static str` error pattern from `Mode` trait.

### Security Considerations

N/A – No network, filesystem, or user input involved.

### Performance Considerations

- `update()` performs exactly 2 method calls per cycle (set_steering + set_throttle)
- No heap allocations during update
- No conditional branches dependent on external state
- Struct size: 16 bytes (one `Box<dyn ActuatorInterface>` fat pointer)

### Platform Considerations

N/A – Platform agnostic. Uses `ActuatorInterface` abstraction which has implementations for RP2350, mock (tests), and SITL.

## Alternatives Considered

1. Shared NullMode / StoppedMode generic
   - Pros: Could share code if multiple modes need zero output
   - Cons: Over-engineering; each mode has unique lifecycle; adds unnecessary abstraction
2. No struct (keep using MotorControlRunner only)
   - Pros: Zero implementation effort
   - Cons: Cannot use Hold with ModeExecutor; inconsistent with other modes; blocks proper mode transition lifecycle

Decision Rationale

- Standalone `HoldMode` struct is the simplest approach that follows existing patterns. The implementation is \~50 lines including tests. No abstraction needed for a one-time trivial struct.

## Migration and Compatibility

- Backward compatible: No existing behavior changes
- No migration needed: New file only, no modifications to existing mode behavior
- `MotorControlRunner` continues to handle `FlightMode::Hold` as before (defense-in-depth)

## Testing Strategy

### Unit Tests

Tests in `crates/core/src/mode/hold.rs` (inline `#[cfg(test)]` module):

- `test_hold_mode_enter_exit`: Verify enter/exit succeed and actuators are zeroed
- `test_hold_mode_update_zeros_actuators`: Verify update zeros actuators even with non-zero initial values
- `test_hold_mode_name`: Verify `name()` returns `"Hold"`
- `test_hold_mode_size`: Verify `size_of::<HoldMode>() <= 32`

Define a local `MockActuator` in the `#[cfg(test)]` module, following the same pattern used in `manual.rs` and `auto.rs`.

### Integration Tests

N/A – Existing `MotorControlRunner` tests already cover `FlightMode::Hold` behavior. `ModeExecutor` tests cover mode transition lifecycle.

## Documentation Impact

- Update AN-00167 status and task link
- Update FR-00168, FR-00169, NFR-00170 task links

## External References

- [ArduPilot Hold Mode (Rover)](https://ardupilot.org/rover/docs/hold-mode.html)

## Open Questions

None – all questions resolved during analysis phase.
