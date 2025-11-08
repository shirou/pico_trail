# ADR-2l5fh Differential Drive Kinematics Implementation

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-41nab-differential-drive-kinematics](../requirements/FR-41nab-differential-drive-kinematics.md)
- Related Analyses:
  - [AN-qlnt3-freenove-hardware-support](../analysis/AN-qlnt3-freenove-hardware-support.md)
- Related ADRs:
  - [ADR-4hx3d-motor-driver-abstraction](../adr/ADR-4hx3d-motor-driver-abstraction.md)
- Related Tasks: N/A - Tasks will be created after requirements

## Context

The Freenove 4WD Car uses a differential drive (skid-steer) configuration with 4 DC motors arranged in left/right groups. The system must convert high-level steering/throttle commands into left/right motor group speeds.

**Current State:**

- No differential drive implementation exists
- Control modes (manual, guided, auto) output steering/throttle commands
- No conversion logic from steering/throttle to motor speeds

**New Requirements:**

- Convert steering (-1.0 left, +1.0 right) and throttle (-1.0 reverse, +1.0 forward) to left/right motor speeds
- Normalize output to prevent values exceeding ±1.0 range
- Handle edge cases (full steering + full throttle, zero-radius turns)
- Match ArduPilot's differential drive behavior for consistency

**Forces in Tension:**

- **Modularity**: Kinematics logic should be reusable for different hardware platforms
- **Integration**: Needs to work with motor_driver library and control modes
- **Coupling**: Avoid tight coupling between kinematics and motor driver implementation
- **Testability**: Kinematics math should be unit testable without hardware dependencies

**Pain Points:**

- Mixing kinematics into motor_driver couples math to hardware abstraction
- Implementing in control modes duplicates logic across manual/guided/auto modes
- No clear ownership of the steering/throttle → left/right conversion

## Success Metrics

- **Correctness**: Unit tests validate against ArduPilot reference implementation (matching output for identical inputs)
- **Reusability**: Same kinematics code works for different motor configurations (2-motor, 4-motor, mecanum wheels)
- **Performance**: Zero-cost abstraction - kinematics calls compile to simple arithmetic (verified via `cargo asm`)

## Decision

We will implement differential drive kinematics as a standalone library module at `src/libraries/kinematics/differential_drive.rs`, separate from both the motor_driver library and control mode logic.

### Decision Drivers

- **Separation of Concerns**: Kinematics math is independent of motor hardware and control logic
- **Reusability**: Kinematics library can be used by any vehicle type (rovers, tracked vehicles, boats)
- **Testability**: Pure math functions easily unit tested without mocking hardware or system state
- **Future Extensibility**: Easy to add other kinematics models (Ackermann steering, mecanum wheels)

### Considered Options

- **Option A**: Implement in motor_driver library (tightly coupled to motor control)
- **Option B**: Implement in rover/mode layer (control logic)
- **Option C**: Standalone kinematics library (Selected)

### Option Analysis

- **Option A** — Pros: Co-located with motor usage | Cons: Couples math to hardware abstraction, not reusable for other drive types
- **Option B** — Pros: Close to control logic | Cons: Duplicates across modes, mixes control flow with kinematics math
- **Option C** — Pros: Reusable, testable, clear responsibility, extensible | Cons: Adds another module (minimal cost)

## Rationale

Option C provides the cleanest architecture:

1. **Pure Math Separation**: Kinematics is pure mathematics (steering/throttle → left/right), independent of hardware (PWM, GPIO) and control logic (modes, failsafe).

2. **Reusability Across Vehicle Types**: A standalone kinematics library can be reused for:
   - Rovers with different motor counts (2-motor, 4-motor, 6-motor)
   - Tracked vehicles (differential drive with tracks)
   - Boats with differential thrust
   - Future drive types (mecanum wheels, Ackermann steering)

3. **Testability**: Pure functions with no dependencies on hardware or system state:

   ```rust
   #[test]
   fn test_straight_forward() {
       let (left, right) = differential_drive(0.0, 0.5);
       assert_eq!(left, 0.5);
       assert_eq!(right, 0.5);
   }
   ```

4. **Clear Ownership**: `src/libraries/kinematics/` owns the math, `motor_driver` owns hardware, control modes own decision logic.

5. **ArduPilot Consistency**: ArduPilot implements kinematics in a separate mixing layer (not in motors or control modes).

Option A was rejected because it couples kinematics to motor hardware abstraction. Option B was rejected because it duplicates logic across control modes and mixes concerns.

## Consequences

### Positive

- Kinematics logic reusable across vehicle types and platforms
- Pure math functions easily unit tested without hardware mocks
- Clear separation: kinematics (math), motor_driver (hardware), modes (control logic)
- Easy to add new kinematics models (Ackermann, mecanum) without refactoring
- Matches ArduPilot's architectural pattern for mixing/kinematics

### Negative

- Adds another library module (minimal maintenance cost)
- Developers must understand three layers: kinematics → motor_driver → platform

### Neutral

- Control modes call kinematics library, then pass results to motor_driver
- Future kinematics models (Ackermann steering) will live in same library

## Implementation Notes

### Module Structure

```
src/
├── libraries/
│   ├── kinematics/            # NEW: Drive kinematics
│   │   ├── mod.rs             # Kinematics trait and common types
│   │   └── differential.rs    # Differential drive mixing
│   ├── motor_driver/          # From ADR-4hx3d
│   └── srv_channel/           # Existing
```

### Core API

```rust
// src/libraries/kinematics/differential.rs
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
    pub fn mix(steering: f32, throttle: f32) -> (f32, f32) {
        let mut left = throttle + steering;
        let mut right = throttle - steering;

        // Normalize to [-1.0, +1.0] range
        let max_magnitude = left.abs().max(right.abs());
        if max_magnitude > 1.0 {
            left /= max_magnitude;
            right /= max_magnitude;
        }

        (left, right)
    }
}
```

### Usage Pattern

```rust
// In control mode (e.g., manual mode)
let steering = rc_input.get_normalized(1); // -1.0 to +1.0
let throttle = rc_input.get_normalized(2); // -1.0 to +1.0

let (left_speed, right_speed) = DifferentialDrive::mix(steering, throttle);

motor_group_left.set_speed(left_speed)?;
motor_group_right.set_speed(right_speed)?;
```

### Unit Test Coverage

Reference ArduPilot's differential drive behavior:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_forward() {
        let (left, right) = DifferentialDrive::mix(0.0, 0.5);
        assert_eq!(left, 0.5);
        assert_eq!(right, 0.5);
    }

    #[test]
    fn forward_right_turn() {
        let (left, right) = DifferentialDrive::mix(0.5, 0.5);
        assert_eq!(left, 1.0);  // Normalized
        assert_eq!(right, 0.0);
    }

    #[test]
    fn spin_right_in_place() {
        let (left, right) = DifferentialDrive::mix(1.0, 0.0);
        assert_eq!(left, 1.0);
        assert_eq!(right, -1.0);
    }

    #[test]
    fn spin_left_in_place() {
        let (left, right) = DifferentialDrive::mix(-1.0, 0.0);
        assert_eq!(left, -1.0);
        assert_eq!(right, 1.0);
    }
}
```

## Platform Considerations

- **Platform-Independent**: Pure math with no platform-specific code
- **Float Performance**: Uses f32 arithmetic (native on ARM Cortex-M, efficient on RP2350)
- **No Allocation**: Stack-only computation, suitable for `no_std` embedded

## Security & Privacy

N/A - Pure math functions with no security or privacy implications.

## Open Questions

- [ ] Should kinematics handle deadband/trim adjustments, or leave that to motor_driver/parameter layer? → Next step: Determine during motor group abstraction requirements (FR-DRAFT-3)
- [ ] Do we need rate limiting in kinematics (e.g., max acceleration), or handle in control modes? → Next step: Defer to control mode implementation, kinematics stays pure math

## External References

- [ArduPilot SkidSteer Output](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Motors/AP_MotorsUGV.cpp) - Reference differential drive mixing implementation
- [ROS Differential Drive Kinematics](http://wiki.ros.org/diff_drive_controller) - Standard differential drive math

---

## Template Usage

This ADR follows the structure defined in [docs/templates/adr.md](../templates/adr.md).
