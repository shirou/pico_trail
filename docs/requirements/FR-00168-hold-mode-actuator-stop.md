# FR-00168 Hold Mode Actuator Stop

## Metadata

- Type: Functional Requirement
- Status: Implemented

## Links

- Prerequisite Requirements:
  - [FR-00062-control-modes](FR-00062-control-modes.md)
- Dependent Requirements:
  - [FR-00169-hold-mode-identity](FR-00169-hold-mode-identity.md)
  - [NFR-00170-hold-mode-performance](NFR-00170-hold-mode-performance.md)
- Related Tasks:
  - [T-00171-hold-mode](../tasks/T-00171-hold-mode/README.md)

## Requirement Statement

The system shall provide a Hold mode that implements the `Mode` trait, always succeeds on entry, and sets all actuator outputs (steering and throttle) to zero on entry, every update cycle, and exit.

## Rationale

Hold mode is the simplest and most reliable control mode. It must:

- Serve as the default failsafe fallback when other modes cannot be entered (e.g., no GPS for RTL/Loiter)
- Provide a safe, predictable stopped state for battery failsafe (`BatteryFailsafeAction::Hold`)
- Allow GCS to command a full stop via standard ArduPilot SET_MODE (custom mode 4)
- Work without any external dependencies (no GPS, no RC input, no navigation)

The `FlightMode::Hold` enum variant and MAVLink mapping already exist, but no `Mode` trait implementation struct exists to use with `ModeExecutor`.

## User Story

As a failsafe system, I want Hold mode to always succeed entry and produce zero motor output, so that the vehicle reliably stops in any failure scenario.

## Acceptance Criteria

- [x] `HoldMode` struct exists and implements `Mode` trait
- [x] `HoldMode::new()` constructor accepts `Box<dyn ActuatorInterface>`
- [x] `enter()` always returns `Ok(())` with no preconditions
- [x] `enter()` sets steering to 0.0 and throttle to 0.0
- [x] `update()` sets steering to 0.0 and throttle to 0.0
- [x] `exit()` sets steering to 0.0 and throttle to 0.0
- [x] `HoldMode` can be passed to `ModeExecutor::set_mode()`

## Technical Details

### Functional Requirement Details

**Mode Entry:**

1. Set steering to 0.0 via `ActuatorInterface::set_steering()`
2. Set throttle to 0.0 via `ActuatorInterface::set_throttle()`
3. Return `Ok(())` unconditionally (no GPS, RC, or other checks)

**Mode Update (50 Hz):**

1. Set steering to 0.0
2. Set throttle to 0.0
3. Ignore `dt` parameter (no time-dependent behavior)

**Mode Exit:**

1. Set steering to 0.0
2. Set throttle to 0.0

**Defense-in-Depth:**

`MotorControlRunner::step()` already returns zero output for `FlightMode::Hold`. The `HoldMode` trait implementation provides a second layer of safety, ensuring actuators are zeroed regardless of motor control path.

## Platform Considerations

N/A - Platform agnostic (uses `ActuatorInterface` abstraction)

## Risks & Mitigation

| Risk                          | Impact | Likelihood | Mitigation                                    | Validation                   |
| ----------------------------- | ------ | ---------- | --------------------------------------------- | ---------------------------- |
| ActuatorInterface write fails | Low    | Very Low   | Log error, continue (vehicle already stopped) | Unit test with mock actuator |

## Implementation Notes

- Create `crates/core/src/mode/hold.rs` following the pattern of `manual.rs`
- Add `pub mod hold;` and re-export in `crates/core/src/mode/mod.rs`
- No parameters needed (ArduPilot Hold mode has no dedicated parameters)
- No `can_enter()` needed (no preconditions)

## External References

- [ArduPilot Hold Mode (Rover)](https://ardupilot.org/rover/docs/hold-mode.html) - Official documentation
