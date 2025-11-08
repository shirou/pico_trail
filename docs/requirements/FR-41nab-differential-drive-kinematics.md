# FR-41nab Differential Drive Kinematics Conversion

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-higbc-hbridge-motor-control](../requirements/FR-higbc-hbridge-motor-control.md)
- Dependent Requirements: N/A - Not yet defined
- Related Tasks: N/A - Tasks will be created after requirements approval

## Requirement Statement

The system shall convert steering and throttle inputs into left and right motor group speeds for differential drive (skid-steer) vehicles, with automatic normalization to prevent output values from exceeding the valid \[-1.0, +1.0] range.

## Rationale

Differential drive vehicles (like the Freenove 4WD Car) control direction by varying the speed of left and right motor groups independently, rather than using a separate steering mechanism. Control modes output high-level commands (steering: -1.0 left to +1.0 right, throttle: -1.0 reverse to +1.0 forward), but motor drivers require individual motor speeds. Without proper kinematics conversion, commands like "full throttle + full right turn" (throttle=1.0, steering=1.0) would produce mathematically invalid motor speeds (left=2.0, right=0.0). This requirement ensures valid motor commands through proper mixing and normalization.

## User Story (if applicable)

As a rover control system, I want to convert steering/throttle commands into left/right motor speeds with proper normalization, so that differential drive vehicles can execute maneuvers without exceeding motor speed limits.

## Acceptance Criteria

**Functional Requirements:**

- [ ] System provides conversion function accepting steering (-1.0 to +1.0) and throttle (-1.0 to +1.0) inputs
- [ ] System returns left and right motor speeds normalized to \[-1.0, +1.0] range
- [ ] Forward motion (steering=0.0, throttle=0.5) produces equal left and right speeds (0.5, 0.5)
- [ ] Right turn (steering=0.5, throttle=0.5) produces valid normalized output (e.g., left=1.0, right=0.0)
- [ ] In-place rotation (steering=1.0, throttle=0.0) produces opposite motor directions (left=1.0, right=-1.0)
- [ ] Normalization maintains proportional relationship between left and right speeds
- [ ] Conversion behavior matches ArduPilot's differential drive mixing for consistency

**Reusability Requirements:**

- [ ] Kinematics code has zero dependencies on platform-specific modules (`src/platform/*/`)
- [ ] Kinematics code has zero dependencies on hardware abstractions (GPIO, PWM, motor drivers)
- [ ] Kinematics code has zero dependencies on system state (armed state, failsafe, parameters)
- [ ] Unit tests for kinematics run on host machine (`cargo test --lib`) without embedded target
- [ ] Kinematics functions are pure (same inputs always produce same outputs, no side effects)
- [ ] `no_std` compatible for embedded use while remaining testable with `std` on host
- [ ] Same kinematics implementation works for different motor counts (2-motor, 4-motor, 6-motor)

## Technical Details

### Functional Requirement Details

**Kinematics Conversion Interface:**

```rust
pub struct DifferentialDrive;

impl DifferentialDrive {
    /// Convert steering/throttle to left/right motor speeds
    ///
    /// # Arguments
    /// * `steering` - -1.0 (full left) to +1.0 (full right)
    /// * `throttle` - -1.0 (full reverse) to +1.0 (full forward)
    ///
    /// # Returns
    /// (left_speed, right_speed) normalized to [-1.0, +1.0]
    pub fn mix(steering: f32, throttle: f32) -> (f32, f32);
}
```

**Inputs:**

- `steering`: f32 in range \[-1.0, +1.0]
  - Negative: turn left
  - Positive: turn right
  - Zero: straight line
- `throttle`: f32 in range \[-1.0, +1.0]
  - Negative: reverse
  - Positive: forward
  - Zero: stationary

**Outputs:**

- `(left_speed, right_speed)`: Tuple of f32 values in range \[-1.0, +1.0]
  - Values represent motor group speeds
  - Normalized to maintain proportional relationship without exceeding limits

**Conversion Algorithm:**

```rust
let mut left = throttle + steering;
let mut right = throttle - steering;

// Normalize if either exceeds [-1.0, +1.0] range
let max_magnitude = left.abs().max(right.abs());
if max_magnitude > 1.0 {
    left /= max_magnitude;
    right /= max_magnitude;
}

(left, right)
```

**Test Cases (from ArduPilot reference):**

| Steering | Throttle | Left Speed | Right Speed | Behavior                        |
| -------- | -------- | ---------- | ----------- | ------------------------------- |
| 0.0      | 0.5      | 0.5        | 0.5         | Straight forward                |
| 0.5      | 0.5      | 1.0        | 0.0         | Forward right turn (normalized) |
| 1.0      | 0.0      | 1.0        | -1.0        | Spin right in place             |
| -1.0     | 0.0      | -1.0       | 1.0         | Spin left in place              |
| 0.0      | -0.5     | -0.5       | -0.5        | Straight reverse                |
| -0.3     | 0.8      | 0.5        | 1.0         | Forward left turn (normalized)  |

**Error Conditions:**

- None - all f32 inputs accepted, normalization handles edge cases

### Non-Functional Requirement Details

**Reliability:**

- Pure functions: No mutable state, no I/O, deterministic outputs
- Stack-only computation: No heap allocations
- No panics: All code paths safe for embedded `no_std` environment

**Compatibility:**

- `no_std` compatible with conditional compilation for test environments:

```rust
#[cfg(test)]
mod tests {
    use super::*;  // std available in tests

    #[test]
    fn test_straight_forward() {
        let (left, right) = DifferentialDrive::mix(0.0, 0.5);
        assert_eq!(left, 0.5);
        assert_eq!(right, 0.5);
    }
}
```

- Works with f32 arithmetic (native on ARM Cortex-M)
- No external crate dependencies beyond `core`/`libm` for embedded math

**Usability:**

- Simple API: Single function call for conversion
- Clear documentation with test case examples
- Matches ArduPilot behavior for user familiarity

## Platform Considerations

### Cross-Platform

- Platform-independent: Pure math with no platform-specific code
- Host testing: Tests run on Linux/macOS/Windows development machines
- Embedded targets: Works on RP2350, ESP32, STM32 without changes
- Future extensibility: Easy to add other kinematics models (Ackermann steering, mecanum wheels) following same pattern

## Risks & Mitigation

| Risk                                                   | Impact | Likelihood | Mitigation                                    | Validation                                         |
| ------------------------------------------------------ | ------ | ---------- | --------------------------------------------- | -------------------------------------------------- |
| Normalization changes turn radius unexpectedly         | Medium | Medium     | Document behavior, match ArduPilot reference  | Unit tests compare against ArduPilot outputs       |
| Floating-point precision errors in normalization       | Low    | Low        | Use f32 max operation, test edge cases        | Unit tests with extreme values (±1.0 combinations) |
| Behavior differs from ArduPilot causing user confusion | Medium | Low        | Implement identical algorithm to ArduPilot    | Cross-reference ArduPilot SkidSteer output code    |
| Coupling to platform code reduces portability          | High   | Medium     | Code review to prevent platform dependencies  | CI checks kinematics compiles standalone           |
| Heap allocations break `no_std` compatibility          | Medium | Low        | Use `#![no_std]` attribute, avoid collections | Compile tests for `no_std` target                  |
| Testing requires hardware mocks (not pure functions)   | Medium | Medium     | Design as pure functions from start           | Unit tests run without any mocks                   |

## Implementation Notes

**Preferred Approach:**

- Create standalone library module `src/libraries/kinematics/` with no dependencies
- Implement as pure functions (no `&mut self`, no state)
- Use `#![no_std]` attribute with conditional `std` for tests
- Separate kinematics (math), motor_driver (hardware), control modes (logic)
- Match ArduPilot's mixing algorithm exactly for consistency

**Verification Commands:**

```bash
# Run kinematics tests on host (fast iteration)
cargo test --lib kinematics

# Verify no_std compatibility
cargo check --target thumbv8m.main-none-eabihf --lib

# Check for platform dependencies
cargo tree --package pico_trail --edges normal | grep kinematics
```

**Known Pitfalls:**

- Do not implement in motor_driver (couples math to hardware)
- Do not duplicate across control modes (centralize in kinematics library)
- Ensure normalization maintains proportional relationship (not clamping)
- Avoid importing anything from `src/platform/` in kinematics code
- Avoid importing hardware abstractions (`motor_driver`, `srv_channel`)
- Avoid importing system state (`SystemState`, `Parameters`)
- Keep dependencies minimal (only `core` for embedded)

**Related Code Areas:**

- `src/libraries/kinematics/` - Kinematics implementation (new)
- `src/libraries/motor_driver/` - Consumes kinematics output (separate concern)
- `src/rover/modes/` - Provides steering/throttle inputs (separate concern)

**Suggested Testing:**

- Unit tests: All basic cases (straight, turns, spins) from test case table
- Property-based testing: Output always in \[-1.0, +1.0] for any valid input
- Cross-platform testing: Same tests pass on host and embedded targets

## External References

- [ArduPilot SkidSteer Output](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsUGV.cpp) - Reference differential drive mixing implementation (lines 400-450)
- [ROS Differential Drive Kinematics](http://wiki.ros.org/diff_drive_controller) - Standard differential drive mathematics
- [Differential Drive Kinematics Tutorial](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf) - Theory and mathematics
- [Rust Embedded Book - Design Philosophy](https://docs.rust-embedded.org/book/design-patterns/hal.html) - Separation of platform-independent and platform-specific code
- [ArduPilot Architecture](https://ardupilot.org/dev/docs/apmcopter-code-overview.html) - Reference for separating kinematics from motors
- [Rust `no_std` Documentation](https://docs.rust-embedded.org/book/intro/no-std.html) - Guidelines for platform-independent embedded code

---

## Template Usage

This requirement follows the structure defined in [docs/templates/requirements.md](../templates/requirements.md).
