# FR-uo1p5 Actuator Abstraction for Rover

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements: N/A - Foundation layer
- Dependent Requirements:
  - [FR-uk0us-manual-mode](FR-uk0us-manual-mode.md)
  - [FR-sp3at-vehicle-modes](FR-sp3at-vehicle-modes.md)
  - [NFR-jng15-actuator-failsafe](NFR-jng15-actuator-failsafe.md)
- Related Tasks:
  - [T-3irm5-manual-control-implementation](../tasks/T-3irm5-manual-control-implementation/README.md)

## Requirement Statement

The system shall provide an actuator abstraction layer that accepts normalized steering and throttle commands (-1.0 to +1.0), converts them to PWM signals (1000-2000 μs pulse width), and enforces critical safety checks by disabling all actuator output when the vehicle is disarmed.

## Rationale

Actuator abstraction decouples vehicle control logic from hardware PWM details, providing:

- **Hardware independence**: Vehicle modes work with normalized values, not PWM specifics
- **Safety enforcement**: Primary layer for armed state check (defense in depth)
- **Calibration centralization**: Min/max/trim values managed in one place
- **Testability**: Vehicle modes can be unit tested without hardware
- **Maintainability**: PWM changes don't affect mode implementations

The actuator layer is the **primary enforcement point** for safety checks, ensuring no motor output when disarmed regardless of mode behavior.

## User Story (if applicable)

As a vehicle control mode, I want to command actuators with normalized values (-1.0 to +1.0), so that I can remain independent of PWM hardware details and rely on the actuator layer to enforce safety checks.

## Acceptance Criteria

- [ ] Provide `set_steering(f32)` interface accepting -1.0 (left) to +1.0 (right)
- [ ] Provide `set_throttle(f32)` interface accepting -1.0 (reverse) to +1.0 (forward)
- [ ] Normalize inputs: clamp to \[-1.0, +1.0] range
- [ ] Convert normalized values to PWM pulse width (1000-2000 μs)
- [ ] Convert PWM pulse width to duty cycle for 50 Hz PWM (5-10% duty cycle)
- [ ] **CRITICAL SAFETY: Check armed state before EVERY actuator command**
- [ ] **CRITICAL SAFETY: Output neutral PWM (1500 μs) when disarmed, ignore all commands**
- [ ] **CRITICAL SAFETY: On disarm event, immediately set neutral PWM (< 20ms)**
- [ ] Support per-actuator calibration: min, max, trim (stored as parameters)
- [ ] Default calibration: 1000 μs (min), 1500 μs (neutral), 2000 μs (max)
- [ ] Actuator state queryable: get current steering/throttle values

## Technical Details (if applicable)

### Functional Requirement Details

**Actuator Interface:**

```rust
/// Actuator abstraction for rover steering and throttle
pub trait ActuatorInterface {
    /// Set steering command (-1.0 left, 0.0 center, +1.0 right)
    /// SAFETY: Enforces armed check, outputs neutral if disarmed
    fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str>;

    /// Set throttle command (-1.0 reverse, 0.0 stop, +1.0 forward)
    /// SAFETY: Enforces armed check, outputs neutral if disarmed
    fn set_throttle(&mut self, normalized: f32) -> Result<(), &'static str>;

    /// Get current steering value
    fn get_steering(&self) -> f32;

    /// Get current throttle value
    fn get_throttle(&self) -> f32;
}
```

**Actuator Implementation:**

```rust
pub struct Actuators {
    steering_pwm: &mut dyn PwmInterface,
    throttle_pwm: &mut dyn PwmInterface,
    system_state: &SystemState,
    config: ActuatorConfig,
}

pub struct ActuatorConfig {
    /// Steering calibration (pulse width in μs)
    pub steering_min: u16,    // Default: 1000
    pub steering_neutral: u16, // Default: 1500
    pub steering_max: u16,    // Default: 2000

    /// Throttle calibration (pulse width in μs)
    pub throttle_min: u16,    // Default: 1000
    pub throttle_neutral: u16, // Default: 1500
    pub throttle_max: u16,    // Default: 2000
}

impl ActuatorInterface for Actuators {
    fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str> {
        // CRITICAL SAFETY CHECK: Only output if armed
        let output = if self.system_state.is_armed() {
            normalized.clamp(-1.0, 1.0)
        } else {
            0.0 // Neutral steering when disarmed
        };

        // Convert to PWM
        let pulse_us = self.normalized_to_pulse(
            output,
            self.config.steering_min,
            self.config.steering_neutral,
            self.config.steering_max,
        );
        let duty = Self::pulse_to_duty_cycle(pulse_us);
        self.steering_pwm.set_duty_cycle(duty)?;
        Ok(())
    }

    fn set_throttle(&mut self, normalized: f32) -> Result<(), &'static str> {
        // CRITICAL SAFETY CHECK: Only output if armed
        let output = if self.system_state.is_armed() {
            normalized.clamp(-1.0, 1.0)
        } else {
            0.0 // Zero throttle when disarmed
        };

        // Convert to PWM
        let pulse_us = self.normalized_to_pulse(
            output,
            self.config.throttle_min,
            self.config.throttle_neutral,
            self.config.throttle_max,
        );
        let duty = Self::pulse_to_duty_cycle(pulse_us);
        self.throttle_pwm.set_duty_cycle(duty)?;
        Ok(())
    }
}

impl Actuators {
    /// Convert normalized value (-1.0 to +1.0) to PWM pulse width (μs)
    fn normalized_to_pulse(&self, normalized: f32, min: u16, neutral: u16, max: u16) -> u16 {
        let clamped = normalized.clamp(-1.0, 1.0);

        if clamped < 0.0 {
            // Negative: interpolate between min and neutral
            let range = (neutral - min) as f32;
            neutral - (clamped.abs() * range) as u16
        } else {
            // Positive: interpolate between neutral and max
            let range = (max - neutral) as f32;
            neutral + (clamped * range) as u16
        }
    }

    /// Convert PWM pulse width (μs) to duty cycle (50 Hz)
    fn pulse_to_duty_cycle(pulse_us: u16) -> f32 {
        const PERIOD_US: f32 = 20_000.0; // 50 Hz = 20 ms
        (pulse_us as f32) / PERIOD_US
    }
}
```

**Calibration Parameters:**

Actuator calibration stored as parameters (see FR-a1cuu):

- `SERVO_STEER_MIN`: Steering minimum PWM (default 1000)
- `SERVO_STEER_TRIM`: Steering neutral PWM (default 1500)
- `SERVO_STEER_MAX`: Steering maximum PWM (default 2000)
- `SERVO_THROT_MIN`: Throttle minimum PWM (default 1000)
- `SERVO_THROT_TRIM`: Throttle neutral PWM (default 1500)
- `SERVO_THROT_MAX`: Throttle maximum PWM (default 2000)

**Safety Event Handling:**

On disarm event:

1. System state transitions to `ArmedState::Disarmed`
2. Next `set_steering()` / `set_throttle()` call checks armed state
3. Actuators immediately output neutral PWM (1500 μs)
4. Latency: < 20ms (one control loop cycle at 50 Hz)

## Platform Considerations

### Pico W (RP2040)

- PWM hardware: RP2040 has 8 PWM slices (16 channels)
- Steering: GPIO pin configured as PWM output
- Throttle: GPIO pin configured as PWM output
- 50 Hz PWM frequency: Standard for servos and ESCs

### Pico 2 W (RP2350)

- PWM hardware: RP2350 has 12 PWM slices (24 channels)
- Same interface as RP2040

### Cross-Platform

Actuator abstraction is cross-platform via `PwmInterface` trait. Platform-specific details handled by platform layer (`src/platform/*/pwm.rs`).

## Risks & Mitigation

| Risk                                               | Impact   | Likelihood | Mitigation                                                | Validation                                        |
| -------------------------------------------------- | -------- | ---------- | --------------------------------------------------------- | ------------------------------------------------- |
| **Armed check bypassed, motors run when disarmed** | CRITICAL | Low        | Armed check in every actuator method, unit tests verify   | Automated test: call actuators while disarmed     |
| Calibration incorrect (servo damage)               | High     | Medium     | Safe defaults (1000-2000), parameter validation           | Test with safe range, verify servo movement       |
| PWM duty cycle calculation error                   | Medium   | Low        | Unit test conversion functions with boundary values       | Test: 1000 μs = 5%, 1500 μs = 7.5%, 2000 μs = 10% |
| Concurrent access (race condition)                 | High     | Low        | Actuators owned by vehicle control task (single-threaded) | Design review: verify single owner                |

## Implementation Notes

Preferred approaches:

- **Safety first**: Armed check is **non-negotiable**, always enforced
- **Single owner**: Actuators owned by vehicle control task, no shared mutable state
- **Calibration via parameters**: Use parameter system (FR-a1cuu) for servo calibration
- **Simple conversion**: Direct linear interpolation for normalized → PWM

Known pitfalls:

- **Normalization range**: Always clamp to \[-1.0, +1.0] to prevent overflow
- **Integer overflow**: Use `u16` for pulse width, avoid `u8` (max 255 < 2000)
- **Floating point precision**: PWM conversion uses `f32`, acceptable for 50 Hz
- **Calibration order**: min < neutral < max, validate on parameter load

Related code areas:

- `src/vehicle/actuators.rs` - Actuator abstraction implementation
- `src/platform/traits/pwm.rs` - Platform PWM interface
- `src/communication/mavlink/state.rs` - System state (armed state)
- `src/parameters/` - Servo calibration parameters

## External References

- Servo PWM Standard: <https://www.servocity.com/how-do-servos-work>
- ArduPilot Servo Functions: <https://ardupilot.org/copter/docs/common-servo.html>
- RC PWM Protocol: <https://www.rcgroups.com/forums/showthread.php?1323503-PWM-to-PPM-encoder-(Tutorial)>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
