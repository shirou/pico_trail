# T-vf57h H-bridge Motor Control

## Metadata

- Type: Design
- Status: Implemented

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design implements a zero-cost H-bridge motor driver abstraction supporting forward, reverse, brake, and coast operations with PWM-based speed control. The system provides a platform-independent `Motor` trait with RP2350 implementation using DRV8837 H-bridge drivers (2-pin PWM control), motor groups for coordinating left/right pairs in differential drive configuration, and hardware-enforced armed state checks. The design prioritizes zero-cost abstraction with inline trait methods, no heap allocations, and compile-time polymorphism.

## Success Metrics

- [ ] Motor trait methods compile to direct GPIO writes (verified with `cargo asm`)
- [ ] No heap allocations during motor operations (verified with `#![no_std]`)
- [ ] Armed state enforcement prevents motors running when disarmed (100% test coverage)
- [ ] Motor control latency < 100ns overhead vs direct HAL calls
- [ ] Release builds with LTO show zero function call overhead
- [ ] Motor groups coordinate 4 motors for differential drive (Freenove 4WD Car)

## Background and Current State

- Context: Freenove 4WD Car uses 4 DC motors with DRV8837 H-bridge drivers, requiring 2-pin PWM control per motor for variable speed and direction
- Current behavior: No motor driver abstraction exists; `srv_channel` designed for PWM actuators (servos/ESCs) with pulse-width semantics
- Pain points:
  - No H-bridge control implementation (IN1/IN2 pin pairs, not single PWM output)
  - No differential drive motor coordination (left/right motor groups)
  - No armed state enforcement at motor driver level
  - `srv_channel` naming and semantics don't match motor control paradigm
- Constraints:
  - Zero-cost abstraction required (embedded performance)
  - RP2350 has 12 PWM slices (sufficient for 8 motor pins = 4 motors × 2 pins)
  - DRV8837 requires specific IN1/IN2 pin combinations for each state
- Related ADRs: [ADR-4hx3d-motor-driver-abstraction](../../adr/ADR-4hx3d-motor-driver-abstraction.md)

## Proposed Design

### High-Level Architecture

```text
Control Mode (Manual/Guided/Auto)
  ↓ (outputs left_speed, right_speed via DifferentialDrive::mix)
MotorGroup (left: M1+M2, right: M3+M4)
  ↓ (enforces armed state, distributes speed to motors)
Motor trait (HBridgeMotor<IN1, IN2>)
  ↓ (converts speed to PWM duty cycle, controls IN1/IN2 pins)
RP2350 PWM HAL (rp235x-hal::pwm)
  ↓ (outputs PWM signals)
DRV8837 H-bridge → DC Motor
```

### Components

- **`src/libraries/motor_driver/mod.rs`**:
  - `Motor` trait: `set_speed(speed: f32)`, `stop()`, `brake()`
  - `MotorError` enum: `NotArmed`, `InvalidSpeed`, `HardwareFault`
  - `MotorGroup` struct: Coordinates multiple motors with same speed
  - Re-exports and module organization

- **`src/libraries/motor_driver/hbridge.rs`**:
  - `HBridgeMotor<IN1, IN2>` struct (generic over GPIO pins)
  - DRV8837 control logic (truth table implementation)
  - PWM duty cycle conversion from f32 speed
  - `#[inline]` attribute on all hot-path methods

- **`src/platform/rp2350/motor.rs`**:
  - RP2350-specific PWM configuration (frequency, slice allocation)
  - GPIO pin initialization for motor control
  - Platform-specific safety (PWM channel validation)

- **Safety Enforcement** (integrated across components):
  - Armed state check in `MotorGroup::set_speed()`
  - Error propagation via `Result<(), MotorError>`
  - Cannot bypass safety (enforced at trait implementation)

### Data Flow

1. Control mode outputs left/right speeds (-1.0 to +1.0)
2. `MotorGroup` receives speeds and checks armed state
3. If armed, distribute speed to each motor in the group
4. Each `HBridgeMotor` converts speed to IN1/IN2 pin states
5. PWM hardware outputs signals to DRV8837 H-bridge
6. DC motors rotate at commanded speed/direction

### Data Models and Types

```rust
// src/libraries/motor_driver/mod.rs

/// Motor control error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorError {
    /// System not armed - motors cannot run
    NotArmed,
    /// Speed value outside [-1.0, +1.0] range
    InvalidSpeed,
    /// Hardware PWM channel unavailable
    HardwareFault,
}

/// Motor control trait (platform-independent)
pub trait Motor {
    /// Set motor speed: -1.0 (full reverse) to +1.0 (full forward)
    fn set_speed(&mut self, speed: f32) -> Result<(), MotorError>;

    /// Stop motor (coast - high-Z state)
    fn stop(&mut self) -> Result<(), MotorError>;

    /// Brake motor (short brake - both pins low)
    fn brake(&mut self) -> Result<(), MotorError>;
}

/// Motor group for coordinating multiple motors (left/right sides)
pub struct MotorGroup<M: Motor> {
    motors: [M; 4],  // Or use heapless::Vec for variable count
    armed: fn() -> bool,  // Function pointer to check armed state
}

impl<M: Motor> MotorGroup<M> {
    /// Create motor group with armed state checker
    pub fn new(motors: [M; 4], armed_check: fn() -> bool) -> Self {
        Self { motors, armed: armed_check }
    }

    /// Set speed for all motors in group (with armed check)
    pub fn set_group_speed(&mut self, speeds: [f32; 4]) -> Result<(), MotorError> {
        if !(self.armed)() {
            return Err(MotorError::NotArmed);
        }

        for (motor, &speed) in self.motors.iter_mut().zip(&speeds) {
            motor.set_speed(speed)?;
        }
        Ok(())
    }
}
```

```rust
// src/libraries/motor_driver/hbridge.rs

/// DRV8837 H-bridge motor driver (2-pin PWM control)
pub struct HBridgeMotor<IN1, IN2>
where
    IN1: PwmPin,
    IN2: PwmPin,
{
    in1: IN1,
    in2: IN2,
}

impl<IN1, IN2> HBridgeMotor<IN1, IN2>
where
    IN1: PwmPin,
    IN2: PwmPin,
{
    /// Create new H-bridge motor with initialized PWM pins
    pub fn new(in1: IN1, in2: IN2) -> Self {
        Self { in1, in2 }
    }
}

impl<IN1, IN2> Motor for HBridgeMotor<IN1, IN2>
where
    IN1: PwmPin,
    IN2: PwmPin,
{
    #[inline]
    fn set_speed(&mut self, speed: f32) -> Result<(), MotorError> {
        if speed < -1.0 || speed > 1.0 {
            return Err(MotorError::InvalidSpeed);
        }

        // DRV8837 truth table implementation
        if speed > 0.0 {
            // Forward: IN1=PWM, IN2=LOW
            self.in1.set_duty(speed)?;
            self.in2.set_duty(0.0)?;
        } else if speed < 0.0 {
            // Reverse: IN1=LOW, IN2=PWM
            self.in1.set_duty(0.0)?;
            self.in2.set_duty(speed.abs())?;
        } else {
            // Coast: IN1=LOW, IN2=LOW
            self.in1.set_duty(0.0)?;
            self.in2.set_duty(0.0)?;
        }
        Ok(())
    }

    #[inline]
    fn stop(&mut self) -> Result<(), MotorError> {
        // Coast: IN1=LOW, IN2=LOW (High-Z)
        self.in1.set_duty(0.0)?;
        self.in2.set_duty(0.0)?;
        Ok(())
    }

    #[inline]
    fn brake(&mut self) -> Result<(), MotorError> {
        // Brake: IN1=HIGH, IN2=HIGH (short brake)
        self.in1.set_duty(1.0)?;
        self.in2.set_duty(1.0)?;
        Ok(())
    }
}
```

**DRV8837 Truth Table** (from datasheet):

| IN1 | IN2 | Motor State                                |
| --- | --- | ------------------------------------------ |
| 0   | 0   | Coast (High-Z, motor freewheels)           |
| PWM | 0   | Forward (speed = PWM duty cycle)           |
| 0   | PWM | Reverse (speed = PWM duty cycle)           |
| 1   | 1   | Brake (short brake, both terminals to GND) |

### Error Handling

- `MotorError::NotArmed`: Return when attempting motor operations while disarmed
- `MotorError::InvalidSpeed`: Return when speed outside \[-1.0, +1.0]
- `MotorError::HardwareFault`: Return when PWM channels unavailable or initialization fails
- All errors propagate via `Result<(), MotorError>` (no panics)

### Security Considerations

- Armed state check prevents unauthorized motor operation
- Cannot bypass safety via trait interface (enforced at all call sites)
- Invalid pin states (e.g., conflicting IN1/IN2) prevented by truth table logic

### Performance Considerations

- **Zero-cost abstraction**:
  - `#[inline]` on all trait methods
  - Generic bounds resolved at compile time (monomorphization)
  - LTO eliminates function call overhead
- **Stack-only operations**: No heap allocations (no `Box`, `Vec`, `String`)
- **Compile-time polymorphism**: Use generics (`Motor` trait with generic pins), not dynamic dispatch (`dyn Motor`)
- **PWM frequency**: 20-50 kHz (avoid audible noise, balance switching losses)

### Platform Considerations

#### Embedded (RP2350)

- PWM slices: 12 available (8 needed for 4 motors × 2 pins)
- PWM frequency: Configurable 20-50 kHz
- GPIO requirements: 8 PWM-capable pins (M1: GPIO18/19, M2: GPIO20/21, M3: GPIO6/7, M4: GPIO8/9)
- HAL integration: `rp235x-hal::pwm::Slice` for PWM control
- Initialization: Configure PWM slices at startup, store in motor structs

#### Cross-Platform

- Motor trait platform-independent (works on ESP32, STM32)
- Platform-specific code isolated to `src/platform/*/motor.rs`
- PWM abstraction via embedded-hal traits (future: use embedded-hal PwmPin)

## Alternatives Considered

1. **Extend srv_channel for motors**
   - Pros: Reuses existing actuator abstraction
   - Cons: Mixes incompatible paradigms (pulse width vs duty cycle), confusing naming
   - Decision: Rejected - violates separation of concerns

2. **Direct platform implementation (no trait)**
   - Pros: Simple, no abstraction overhead
   - Cons: Not portable, duplicates safety logic across platforms
   - Decision: Rejected - violates platform abstraction principles

3. **Dynamic dispatch (`Box<dyn Motor>`)**
   - Pros: Flexible runtime motor selection
   - Cons: Heap allocations, vtable overhead, not suitable for embedded
   - Decision: Rejected - violates zero-cost abstraction requirement

4. **Separate motor_driver library (chosen)**
   - Pros: Clean separation, zero-cost abstraction, portable, extensible
   - Cons: Slightly more initial implementation effort
   - Decision: Approved - best long-term design

Decision Rationale: Standalone motor_driver library with trait-based abstraction provides clean separation between actuator types (PWM servos vs H-bridge motors), enables zero-cost abstraction via generics and inline, and allows future motor types (DShot, stepper) without refactoring control logic.

## Migration and Compatibility

- Backward compatibility: N/A (new feature)
- Forward compatibility: Motor trait stable, new motor types implement same trait
- Rollout plan: Implement motor_driver first, integrate with differential drive kinematics and control modes in separate tasks

## Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    // Mock motor for testing
    struct MockMotor {
        speed: f32,
        armed: bool,
    }

    impl Motor for MockMotor {
        fn set_speed(&mut self, speed: f32) -> Result<(), MotorError> {
            if !self.armed {
                return Err(MotorError::NotArmed);
            }
            if speed < -1.0 || speed > 1.0 {
                return Err(MotorError::InvalidSpeed);
            }
            self.speed = speed;
            Ok(())
        }

        fn stop(&mut self) -> Result<(), MotorError> {
            self.speed = 0.0;
            Ok(())
        }

        fn brake(&mut self) -> Result<(), MotorError> {
            self.speed = 0.0;
            Ok(())
        }
    }

    #[test]
    fn test_motor_armed_check() {
        let mut motor = MockMotor { speed: 0.0, armed: false };
        assert_eq!(motor.set_speed(0.5), Err(MotorError::NotArmed));

        motor.armed = true;
        assert!(motor.set_speed(0.5).is_ok());
    }

    #[test]
    fn test_invalid_speed() {
        let mut motor = MockMotor { speed: 0.0, armed: true };
        assert_eq!(motor.set_speed(1.5), Err(MotorError::InvalidSpeed));
        assert_eq!(motor.set_speed(-1.5), Err(MotorError::InvalidSpeed));
    }
}
```

### Integration Tests

- Test with actual RP2350 hardware (oscilloscope verification)
- Verify PWM frequency (20-50 kHz range)
- Measure motor control latency (target < 100ns overhead)
- Verify armed state enforcement prevents motor operation

### Performance & Benchmarks

```bash
# Verify zero-cost abstraction
cargo asm --lib --release motor_driver::hbridge::set_speed

# Check binary size impact
cargo bloat --release --target thumbv8m.main-none-eabihf

# Runtime benchmarks (on hardware)
# - Motor control latency measurement
# - PWM frequency stability
# - CPU overhead during continuous motor updates
```

## Documentation Impact

- Update `src/libraries/README.md` to document motor_driver library
- Add inline rustdoc comments with examples
- Create example: `examples/motor_test.rs` for hardware verification

## External References

- [DRV8837 Datasheet](https://www.ti.com/product/DRV8837) - H-bridge specifications and truth table
- [ArduPilot Motors Library](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Motors) - Reference motor abstraction
- [rp235x-hal PWM Documentation](https://docs.rs/rp235x-hal/latest/rp235x_hal/pwm/) - RP2350 PWM implementation
- [embedded-hal Digital Traits](https://docs.rs/embedded-hal/latest/embedded_hal/digital/) - Standard embedded GPIO traits

## Open Questions

- [ ] Should motor groups support variable motor count (2, 4, 6 motors)?
  - Next step: Start with fixed 4-motor array, add heapless::Vec if needed
- [ ] Should we implement current limiting / over-current protection?
  - Next step: Defer to future enhancement (requires ADC current sensing)
- [ ] Should brake use HIGH/HIGH or LOW/LOW (DRV8837 supports both)?
  - Next step: Use HIGH/HIGH (short brake, faster stopping)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
