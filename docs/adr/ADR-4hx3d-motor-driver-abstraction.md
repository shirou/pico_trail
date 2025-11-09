# ADR-4hx3d Motor Driver Abstraction Strategy

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-higbc-hbridge-motor-control](../requirements/FR-higbc-hbridge-motor-control.md)
- Related Analyses:
  - [AN-qlnt3-freenove-hardware-support](../analysis/AN-qlnt3-freenove-hardware-support.md)
- Related Tasks:
  - [T-vf57h-hbridge-motor-control](../tasks/T-vf57h-hbridge-motor-control/README.md)

## Context

The pico_trail project currently supports PWM-based servo and ESC control via `src/libraries/srv_channel/`, designed for hobby RC vehicles with 2-channel control (steering + throttle). To support the Freenove 4WD Car hardware platform, we need to add H-bridge motor driver control for 4 independent DC motors using DRV8837 H-bridge drivers with PWM-based speed control.

**Current State:**

- `src/libraries/srv_channel/`: PWM actuator abstraction for servo/ESC control
- Output: PWM duty cycle (1000-2000μs pulse widths → 0.0-1.0 duty cycle)
- Model: Direct actuator control (steering/throttle → PWM output)

**New Requirements:**

- H-bridge motor control: 2 GPIO pins per motor (IN1, IN2) with PWM speed control
- 4 motors arranged in differential drive configuration (left/right groups)
- Control states: Forward, Reverse, Brake, Coast
- PWM duty cycle: 0-100% speed range (not pulse width timing)

**Forces in Tension:**

- **Separation of Concerns**: PWM actuators (servo/ESC) vs H-bridge motors are different control paradigms
- **Code Reusability**: Share common patterns (safety checks, armed state) without coupling incompatible abstractions
- **Future Extensibility**: Support additional motor types (DShot ESCs, stepper motors) without refactoring
- **Implementation Effort**: Balance architectural purity with time to first working implementation

**Pain Points:**

- `srv_channel` naming assumes servo control, misleading for motor drivers
- PWM pulse width semantics (1000-2000μs) don't map naturally to H-bridge control (0-100% duty cycle)
- Mixing two actuator paradigms in one module reduces clarity

## Success Metrics

- Zero-cost abstraction: H-bridge trait calls compile to direct GPIO writes (verified via `cargo asm`)
- Code reuse: Safety enforcement (armed checks) shared between srv_channel and motor_driver
- Extensibility: Adding a new motor type (e.g., DShot ESC) requires only implementing the motor trait, no changes to control logic

## Decision

We will create a new library module `src/libraries/motor_driver/` for H-bridge motor control, separate from the existing `srv_channel` module.

### Decision Drivers

- **Conceptual Clarity**: Separate modules for distinct actuator types (PWM vs H-bridge)
- **Future Extensibility**: Easy to add new motor types without refactoring existing code
- **Safety Reuse**: Share armed state enforcement pattern across actuator types
- **Platform Abstraction**: Motor driver traits independent of RP2350-specific code

### Considered Options

- **Option A**: Extend srv_channel for H-bridge Support
- **Option B**: New Motor Driver Module (Selected)
- **Option C**: Platform-Specific Motor Implementation

### Option Analysis

- **Option A** — Pros: Reuses existing abstraction, minimal changes | Cons: Mixes incompatible paradigms, misleading naming, couples PWM and H-bridge concepts
- **Option B** — Pros: Clean separation, clear naming, extensible for future motor types | Cons: Requires new module and traits, slightly more initial effort
- **Option C** — Pros: Fast to implement for testing | Cons: Not portable, violates platform abstraction, creates technical debt

## Rationale

Option B provides the best long-term architecture:

1. **Separation of Concerns**: `srv_channel` remains focused on PWM actuators (servo/ESC), while `motor_driver` handles H-bridge motors. This prevents mixing incompatible control paradigms.

2. **Clear Naming**: `motor_driver` accurately describes its purpose, unlike extending a module named `srv_channel` to control DC motors.

3. **Extensibility**: Future motor types (DShot ESCs, stepper motors, brushless motors) can be added to `motor_driver/` without refactoring the servo control code.

4. **Platform Abstraction Alignment**: Follows the project's platform trait pattern, allowing motor driver implementations to work across different platforms (RP2350, ESP32, STM32).

5. **Safety Pattern Reuse**: Both `srv_channel` and `motor_driver` can share the armed state enforcement pattern from `SystemState`, avoiding code duplication while maintaining separation.

Option A was rejected because mixing PWM pulse-width semantics (1000-2000μs) with H-bridge duty cycle control (0-100%) creates conceptual confusion. Option C was rejected as it violates platform abstraction principles and creates technical debt.

## Consequences

### Positive

- Clean separation between PWM actuators and motor drivers
- Easy to add new motor types (DShot, stepper) without refactoring
- Clear module naming that reflects purpose
- Portable motor driver abstraction across platforms
- Shared safety patterns (armed checks) reduce code duplication

### Negative

- Requires creating new module and trait definitions (moderate initial effort)
- Two separate actuator abstractions to maintain
- Developers must understand when to use srv_channel vs motor_driver

### Neutral

- Platform-specific implementations required for both srv_channel and motor_driver
- ADR-2l5fh will determine where differential drive kinematics live (motor_driver vs separate library)

## Implementation Notes

### Module Structure

```
src/
├── libraries/
│   ├── motor_driver/          # NEW: H-bridge motor control
│   │   ├── mod.rs             # Motor traits and safety
│   │   └── hbridge.rs         # H-bridge implementation
│   └── srv_channel/           # EXISTING: PWM actuators (unchanged)
```

### Core Traits

```rust
// src/libraries/motor_driver/mod.rs
pub trait Motor {
    fn set_speed(&mut self, speed: f32) -> Result<(), MotorError>; // -1.0 to +1.0
    fn stop(&mut self) -> Result<(), MotorError>;
    fn brake(&mut self) -> Result<(), MotorError>;
}

pub trait MotorSafety {
    fn is_armed(&self) -> bool;
    fn enforce_armed(&self) -> Result<(), MotorError>;
}
```

### Safety Enforcement Pattern

```rust
impl<P: PlatformTrait> HBridgeMotor<P> {
    pub fn set_speed(&mut self, speed: f32) -> Result<(), MotorError> {
        self.enforce_armed()?; // Shared safety pattern
        // H-bridge control logic
    }
}
```

### Platform Implementation

```rust
// src/platform/rp2350/motor.rs
impl Motor for RP2350HBridgeMotor {
    fn set_speed(&mut self, speed: f32) -> Result<(), MotorError> {
        // RP2350-specific PWM control for H-bridge
    }
}
```

## Platform Considerations

- **RP2350**: Requires 2 PWM-capable GPIO pins per motor (8 pins total for 4 motors)
- **PWM Frequency**: Typically 20-50 kHz for DC motors (avoid audible noise)
- **Hardware PWM Channels**: RP2350 has 12 PWM channels, sufficient for 8 motor control pins
- **Future Platforms**: ESP32, STM32 can implement the same Motor trait with platform-specific PWM control

## Security & Privacy

- **Armed State Enforcement**: Motors must not run when system is disarmed (hard safety requirement)
- **Invalid State Prevention**: H-bridge trait prevents invalid state combinations (e.g., both IN pins high during transitions)

## Open Questions

- [ ] Should motor trim/calibration be part of the motor_driver library or a separate parameter management layer? → Next step: Determine during FR-DRAFT-3 (Motor Group Abstraction) requirements refinement
- [ ] How to share safety enforcement pattern between srv_channel and motor_driver without code duplication? → Method: Consider extracting common safety trait to `src/core/safety/actuator.rs`

## External References

- [DRV8837 Datasheet](https://www.ti.com/product/DRV8837) - Texas Instruments Low-Voltage H-Bridge
- [ArduPilot Motors Library](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Motors) - Reference motor abstraction architecture
- [embedded-hal Digital Traits](https://docs.rs/embedded-hal/latest/embedded_hal/digital/) - Rust embedded GPIO abstraction

---

## Template Usage

This ADR follows the structure defined in [docs/templates/adr.md](../templates/adr.md).
