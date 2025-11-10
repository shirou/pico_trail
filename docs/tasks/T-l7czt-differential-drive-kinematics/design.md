# T-l7czt Differential Drive Kinematics

## Metadata

- Type: Design
- Status: Complete

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design implements a pure, platform-independent differential drive kinematics library that converts steering and throttle inputs into left and right motor speeds with normalization. The system provides a standalone kinematics module (`src/libraries/kinematics/differential_drive.rs`) with no dependencies on platform code, hardware abstractions, or system state, enabling unit testing on host machines while supporting embedded targets. The implementation matches ArduPilot's differential drive behavior for user familiarity and correctness validation.

## Success Metrics

- [ ] All unit tests pass on host machine (cargo test --lib kinematics)
- [ ] All unit tests match ArduPilot reference outputs for identical inputs
- [ ] Property-based tests confirm outputs always in \[-1.0, +1.0] range
- [ ] Module compiles for embedded target (cargo check --target thumbv8m.main-none-eabihf)
- [ ] Zero dependencies on platform modules (verified via cargo tree)
- [ ] No heap allocations (verified with #!\[no_std] attribute)
- [ ] Pure functions (same inputs → same outputs, no side effects)

## Background and Current State

- Context: Differential drive vehicles (Freenove 4WD Car) use independent left/right motor groups for steering, requiring conversion from high-level steering/throttle commands to motor speeds
- Current behavior: No kinematics implementation exists; control modes cannot output motor commands
- Pain points:
  - Commands like "full throttle + full turn" (throttle=1.0, steering=1.0) produce mathematically invalid motor speeds (left=2.0, right=0.0) without normalization
  - No clear ownership of steering/throttle → left/right conversion logic
  - Tight coupling risk if kinematics implemented in motor_driver or control modes
- Constraints:
  - Must work in no_std embedded environment (RP2350)
  - Must be unit testable on host machines (Linux/macOS/Windows)
  - Must match ArduPilot behavior for consistency
- Related ADRs: [ADR-2l5fh-differential-drive-kinematics](../../adr/ADR-2l5fh-differential-drive-kinematics.md)

## Proposed Design

### High-Level Architecture

```text
Control Mode (Manual/Guided/Auto)
  ↓ (outputs steering: -1.0..+1.0, throttle: -1.0..+1.0)
DifferentialDrive::mix(steering, throttle)
  ↓ (pure function, no state, no I/O)
(left_speed, right_speed) normalized to [-1.0, +1.0]
  ↓ (consumed by)
Motor Driver (hbridge, motor groups)
  ↓ (outputs)
GPIO PWM signals
```

### Components

- **`src/libraries/kinematics/mod.rs`**:
  - Module declaration and re-exports
  - `#![no_std]` attribute for embedded compatibility
  - Conditional `std` import for tests

- **`src/libraries/kinematics/differential_drive.rs`**:
  - `DifferentialDrive` struct (zero-sized marker)
  - `mix(steering: f32, throttle: f32) -> (f32, f32)` method
  - Normalization logic to maintain proportional relationship
  - No dependencies on platform code, hardware, or system state

- **`src/libraries/kinematics/differential_drive.rs` tests**:
  - Unit tests with ArduPilot reference values
  - Property-based tests for output range guarantees
  - Edge case tests (zero inputs, extreme values, boundary conditions)

### Data Flow

1. Control mode outputs steering (-1.0 to +1.0) and throttle (-1.0 to +1.0)
2. `DifferentialDrive::mix()` receives inputs
3. Calculate raw motor speeds: `left = throttle + steering`, `right = throttle - steering`
4. Normalize if either exceeds \[-1.0, +1.0]: divide both by `max(abs(left), abs(right))`
5. Return normalized (left_speed, right_speed) tuple
6. Motor driver consumes speeds for hardware control

### Data Models and Types

````rust
// src/libraries/kinematics/differential_drive.rs

/// Differential drive kinematics converter (zero-sized type)
pub struct DifferentialDrive;

impl DifferentialDrive {
    /// Convert steering/throttle to left/right motor speeds with normalization
    ///
    /// # Arguments
    /// * `steering` - -1.0 (full left) to +1.0 (full right)
    /// * `throttle` - -1.0 (full reverse) to +1.0 (full forward)
    ///
    /// # Returns
    /// (left_speed, right_speed) normalized to [-1.0, +1.0]
    ///
    /// # Examples
    /// ```
    /// let (left, right) = DifferentialDrive::mix(0.0, 0.5);
    /// assert_eq!(left, 0.5);   // straight forward
    /// assert_eq!(right, 0.5);
    ///
    /// let (left, right) = DifferentialDrive::mix(1.0, 0.0);
    /// assert_eq!(left, 1.0);   // spin right in place
    /// assert_eq!(right, -1.0);
    /// ```
    pub fn mix(steering: f32, throttle: f32) -> (f32, f32) {
        let mut left = throttle + steering;
        let mut right = throttle - steering;

        // Normalize if either exceeds [-1.0, +1.0] range
        let max_magnitude = left.abs().max(right.abs());
        if max_magnitude > 1.0 {
            left /= max_magnitude;
            right /= max_magnitude;
        }

        (left, right)
    }
}
````

### Error Handling

- No error conditions: All f32 inputs accepted
- Normalization handles edge cases (NaN, infinity handled by f32 arithmetic)
- Pure function with deterministic outputs

### Security Considerations

- N/A - Pure math function with no I/O or external dependencies

### Performance Considerations

- Stack-only computation: No heap allocations
- Inline candidate: Small function suitable for #\[inline] attribute
- f32 arithmetic native on ARM Cortex-M (no soft-float overhead)
- Estimated cost: \~10 CPU cycles (2 additions, 2 abs, 1 max, 1 comparison, optional 2 divisions)

### Platform Considerations

#### Embedded (RP2350)

- `no_std` compatible: Uses only `core` library
- f32 operations native on Cortex-M33+ (hardware FPU)
- Zero heap allocations (stack only)
- Works in interrupt contexts (pure function, no locks)

#### Cross-Platform

- Unit tests run on host machines (Linux/macOS/Windows) with `std` available
- Same code compiles for embedded (RP2350) and future platforms (ESP32, STM32)
- Conditional compilation for test imports:

```rust
#![no_std]

#[cfg(test)]
extern crate std;  // std available in host tests

#[cfg(test)]
mod tests {
    use super::*;
    // tests here
}
```

## Alternatives Considered

1. **Implement in motor_driver library**
   - Pros: Co-located with motor control code
   - Cons: Couples pure math to hardware abstraction, harder to test, not reusable for other kinematics models
   - Decision: Rejected - violates separation of concerns

2. **Implement in control modes (manual/guided/auto)**
   - Pros: Close to input source
   - Cons: Duplicates logic across 3+ modes, inconsistent behavior risk
   - Decision: Rejected - violates DRY principle

3. **Use external crate (e.g., differential-drive-rs)**
   - Pros: Potentially well-tested implementation
   - Cons: No such crate exists, would add dependency, may not match ArduPilot behavior
   - Decision: Rejected - simple enough to implement, ArduPilot compatibility critical

4. **Standalone kinematics library (chosen)**
   - Pros: Clean separation, reusable, testable, matches ArduPilot architecture
   - Cons: Slightly more initial scaffolding (new module)
   - Decision: Approved - best long-term design

Decision Rationale: Standalone library provides clean separation between kinematics (math), motor_driver (hardware), and control modes (logic), enabling independent testing and future extensibility (Ackermann, mecanum). Matches ArduPilot's architecture philosophy.

## Migration and Compatibility

- Backward compatibility: N/A (new feature)
- Forward compatibility: Interface stable, future kinematics models (Ackermann, mecanum) will be separate modules
- Rollout plan: Implement kinematics library first, integrate with motor_driver and control modes in separate tasks

## Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_straight_forward() {
        let (left, right) = DifferentialDrive::mix(0.0, 0.5);
        assert_eq!(left, 0.5);
        assert_eq!(right, 0.5);
    }

    #[test]
    fn test_forward_right_turn_normalized() {
        let (left, right) = DifferentialDrive::mix(0.5, 0.5);
        assert_eq!(left, 1.0);   // normalized from 1.0
        assert_eq!(right, 0.0);  // normalized from 0.0
    }

    #[test]
    fn test_spin_right_in_place() {
        let (left, right) = DifferentialDrive::mix(1.0, 0.0);
        assert_eq!(left, 1.0);
        assert_eq!(right, -1.0);
    }

    #[test]
    fn test_spin_left_in_place() {
        let (left, right) = DifferentialDrive::mix(-1.0, 0.0);
        assert_eq!(left, -1.0);
        assert_eq!(right, 1.0);
    }

    #[test]
    fn test_straight_reverse() {
        let (left, right) = DifferentialDrive::mix(0.0, -0.5);
        assert_eq!(left, -0.5);
        assert_eq!(right, -0.5);
    }

    #[test]
    fn test_forward_left_turn_normalized() {
        let (left, right) = DifferentialDrive::mix(-0.3, 0.8);
        assert!((left - 0.5).abs() < 0.01);   // ~0.5 after normalization
        assert!((right - 1.0).abs() < 0.01);  // ~1.0 after normalization
    }

    // Property-based test: outputs always in [-1.0, +1.0]
    #[test]
    fn test_output_range_property() {
        for steering in [-1.0, -0.5, 0.0, 0.5, 1.0] {
            for throttle in [-1.0, -0.5, 0.0, 0.5, 1.0] {
                let (left, right) = DifferentialDrive::mix(steering, throttle);
                assert!(left >= -1.0 && left <= 1.0, "left={}", left);
                assert!(right >= -1.0 && right <= 1.0, "right={}", right);
            }
        }
    }
}
```

### Integration Tests

- N/A - Pure math function, no integration points
- Integration with motor_driver tested in motor_driver module tests

### External API Parsing

- N/A - No external API

### Performance & Benchmarks

- Optional: Benchmark with `cargo bench` if performance concerns arise
- Expected: < 100ns per call on RP2350 @ 150 MHz (< 15 clock cycles)

## Documentation Impact

- Update `src/libraries/README.md` (or create if missing) to document kinematics library
- Add inline documentation with examples (rustdoc comments)
- No user-facing documentation needed (internal library)

## External References

- [ArduPilot SkidSteer Output](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsUGV.cpp) - Reference differential drive mixing implementation (lines 400-450)
- [ROS Differential Drive Kinematics](http://wiki.ros.org/diff_drive_controller) - Standard differential drive mathematics
- [Differential Drive Kinematics Tutorial](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf) - Theory and mathematics

## Open Questions

- [ ] Should we support configurable mixing ratios (e.g., 0.5 \* steering instead of 1.0 \* steering)?
  - Next step: Defer to future enhancement if users request it
- [ ] Should we provide separate functions for arcade vs tank mixing?
  - Next step: Start with arcade mixing (throttle + steering), add tank mixing if needed

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
