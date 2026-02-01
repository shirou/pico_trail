# FR-00064 H-bridge Motor Control Capability

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements: N/A - This is a foundational requirement
- Dependent Requirements:
  - [FR-00063-differential-drive-kinematics](../requirements/FR-00063-differential-drive-kinematics.md)
  - [FR-00065-pin-configuration-management](../requirements/FR-00065-pin-configuration-management.md)
- Related Tasks:
  - [T-00012-hbridge-motor-control](../tasks/T-00012-hbridge-motor-control/README.md)

## Requirement Statement

The system shall provide motor control functionality for H-bridge motor drivers, supporting forward, reverse, brake, and coast operations with PWM-based speed control ranging from -1.0 (full reverse) to +1.0 (full forward).

## Rationale

The Freenove 4WD Car uses DRV8837 H-bridge motor drivers for 4 independent DC motors. Each motor requires 2 GPIO pins (IN1, IN2) with PWM control to achieve variable speed and direction. This capability is essential for differential drive vehicle control and differs from the existing servo/ESC PWM pulse-width control (1000-2000μs). Providing a dedicated H-bridge abstraction enables clean separation from servo control while supporting future motor types (DShot ESCs, stepper motors).

## User Story (if applicable)

As a rover control system, I want to control H-bridge motor drivers with variable speed and direction, so that I can implement differential drive locomotion for the Freenove 4WD Car.

## Acceptance Criteria

**Functional Requirements:**

- [ ] System provides `Motor` trait with `set_speed(speed: f32)` method accepting -1.0 to +1.0 range
- [ ] System provides `stop()` method that immediately halts motor movement (coast or brake)
- [ ] System provides `brake()` method that actively brakes the motor (both H-bridge pins high or low)
- [ ] Motor control enforces armed state check - motors shall not run when system is disarmed
- [ ] Invalid speed values outside \[-1.0, +1.0] range are rejected with error
- [ ] H-bridge control prevents invalid state combinations (e.g., simultaneous forward/reverse signals)
- [ ] Platform-specific implementations support RP2350 with 2 PWM-capable GPIO pins per motor

**Performance Requirements:**

- [ ] Motor trait method calls (`set_speed`, `stop`, `brake`) compile to direct GPIO writes (verified via `cargo asm`)
- [ ] No heap allocations during motor control operations (verified via `#![no_std]` compatibility)
- [ ] Motor control latency equivalent to direct platform HAL calls (measured via oscilloscope)
- [ ] Release builds with LTO show no function call overhead for trait methods (verified via disassembly)
- [ ] Stack usage for motor operations remains constant and minimal (< 100 bytes per call)

## Technical Details

### Functional Requirement Details

**Motor Control Interface:**

```rust
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

**Inputs:**

- `speed`: f32 value in range \[-1.0, +1.0]
  - Negative values: reverse direction
  - Positive values: forward direction
  - Magnitude: motor speed (0.0 = stopped, ±1.0 = full speed)

**Outputs:**

- H-bridge GPIO control: 2 pins (IN1, IN2) with PWM duty cycle
- Error codes: `MotorError::NotArmed`, `MotorError::InvalidSpeed`, `MotorError::HardwareFault`

**Error Conditions:**

- System not armed: Return `MotorError::NotArmed`
- Speed out of range: Return `MotorError::InvalidSpeed`
- Hardware failure (PWM channel unavailable): Return `MotorError::HardwareFault`

**Safety Enforcement:**

- All motor control methods must check armed state before execution
- Disarmed state takes precedence over all other commands
- Motor operations while disarmed return error without affecting hardware

### Non-Functional Requirement Details

**Performance:**

- Latency: Motor control commands shall execute within same instruction count as direct HAL calls (target: < 10 CPU cycles overhead)
- Throughput: Support control loop frequencies up to 400 Hz without CPU saturation
- Memory: Zero heap allocations during runtime motor control (compile-time allocations acceptable)

**Reliability:**

- Trait abstractions shall inline completely in release builds with LTO enabled
- No dynamic dispatch (no `dyn Trait` usage) in performance-critical motor control paths
- Compile-time polymorphism via generics preferred over runtime polymorphism

**Compatibility:**

- `no_std` compatible for embedded targets (RP2350, ESP32, STM32)
- Works with Rust embedded ecosystem (`embedded-hal`, platform HALs)

## Platform Considerations

### Embedded (RP2350)

- Requires 2 PWM-capable GPIO pins per motor (8 pins total for 4 motors)
- PWM frequency: 20-50 kHz (avoid audible noise)
- Hardware PWM channels: RP2350 has 12 PWM slices, sufficient for 8 motor pins
- `no_std` compatibility required
- Cortex-M33+ CPU at 150 MHz provides \~6.67ns per clock cycle
- Target < 100ns overhead for motor control call (< 15 clock cycles)
- Verify via `probe-rs` or logic analyzer timing measurements

### Cross-Platform

- Motor trait shall be platform-independent (implementable on ESP32, STM32, etc.)
- Platform-specific code limited to `src/platform/*/motor.rs` implementations
- Safety enforcement pattern shared across all platforms
- Zero-cost abstraction applies to all platforms (RP2350, ESP32, STM32)
- Platform-specific optimizations (e.g., DMA for PWM) allowed if maintaining trait interface
- Performance validation required per platform

## Risks & Mitigation

| Risk                                                       | Impact | Likelihood | Mitigation                                               | Validation                                         |
| ---------------------------------------------------------- | ------ | ---------- | -------------------------------------------------------- | -------------------------------------------------- |
| Motors run while disarmed (safety hazard)                  | High   | Medium     | Enforce armed check in every motor control method        | Unit tests verify `NotArmed` error when disarmed   |
| Invalid H-bridge state causes hardware damage              | High   | Low        | Validate IN1/IN2 combinations before GPIO writes         | Review H-bridge state machine in code review       |
| PWM frequency too low (audible noise)                      | Low    | Medium     | Default to 20+ kHz PWM frequency                         | Verify with oscilloscope during hardware testing   |
| Trait method calls not inlined (runtime overhead)          | High   | Low        | Use `#[inline]` attribute, enable LTO in release profile | Verify with `cargo asm` and disassembly inspection |
| Abstraction layer adds latency affecting control stability | High   | Low        | Profile with oscilloscope, benchmark against direct HAL  | Measure GPIO toggle latency in hardware tests      |
| Generic code bloat increases binary size                   | Medium | Medium     | Use monomorphization selectively, measure binary size    | Monitor flash usage with `cargo bloat`             |

## Implementation Notes

**Preferred Approach:**

- Create new library module `src/libraries/motor_driver/` separate from `srv_channel`
- Follow platform trait pattern: trait in library, implementation in `src/platform/*/`
- Share safety enforcement pattern with `srv_channel` via common trait or base implementation
- Use static dispatch (generics) instead of dynamic dispatch (`dyn Trait`)
- Apply `#[inline]` attributes to trait methods
- Enable LTO (Link-Time Optimization) in release profile:

```toml
[profile.release]
lto = true
opt-level = "z"  # or "s" for size optimization
```

**Verification Commands:**

```bash
# Functional verification
cargo test --lib motor_driver

# Performance verification
cargo asm --lib --release motor_driver::hbridge::set_speed  # Inspect generated assembly
cargo bloat --release  # Measure binary size impact
```

**Known Pitfalls:**

- Do not mix PWM pulse-width semantics (1000-2000μs) with H-bridge duty cycle (0-100%)
- Ensure armed state check is consistent across all actuator types (motors, servos)
- Validate PWM channel availability at initialization, not during runtime control
- Debug builds will not inline - always verify performance in release mode
- Avoid heap allocations (`Box`, `Vec`, `String`) in motor control hot paths
- Generic bounds should be minimal to reduce monomorphization overhead

**Related Code Areas:**

- `src/libraries/srv_channel/` - Reference for safety enforcement pattern
- `src/core/system_state.rs` - Armed state management
- `src/platform/rp2350/platform.rs` - Platform trait implementation
- `src/libraries/motor_driver/` - Motor trait definitions
- `src/platform/rp2350/motor.rs` - Platform implementation
- `Cargo.toml` - Release profile optimization settings

**Suggested Libraries:**

- `embedded-hal` digital traits for GPIO abstraction
- RP2350 HAL PWM module for hardware PWM control

## External References

- [DRV8837 Datasheet](https://www.ti.com/product/DRV8837) - H-bridge motor driver specifications and control logic
- [ArduPilot Motors Library](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Motors) - Reference motor abstraction architecture
- [embedded-hal Digital Traits](https://docs.rs/embedded-hal/latest/embedded_hal/digital/) - Rust embedded GPIO abstraction
- [Rust Performance Book - Zero-Cost Abstractions](https://nnethercote.github.io/perf-book/inlining.html) - Guidelines for zero-cost abstractions in Rust
- [cargo-asm Documentation](https://github.com/gnzlbg/cargo-asm) - Tool for inspecting generated assembly
- [embedded-hal Design Philosophy](https://docs.rs/embedded-hal/latest/embedded_hal/#design-goals) - Zero-cost abstraction principles for embedded Rust
