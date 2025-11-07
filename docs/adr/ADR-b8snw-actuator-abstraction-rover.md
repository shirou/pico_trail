# ADR-b8snw Actuator Abstraction: Normalized Commands with PWM Conversion

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-uo1p5-actuator-abstraction](../requirements/FR-uo1p5-actuator-abstraction.md)
  - [FR-uk0us-manual-mode](../requirements/FR-uk0us-manual-mode.md)
  - [NFR-jng15-actuator-failsafe](../requirements/NFR-jng15-actuator-failsafe.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-3irm5-manual-control-implementation](../tasks/T-3irm5-manual-control-implementation/README.md)

## Context

Rover vehicles require precise control of two primary actuators:

- **Steering Servo**: Ackermann steering, controls front wheel angle
- **Throttle Motor/ESC**: Differential drive or single motor, controls forward/reverse motion

Vehicle control modes (Manual, Auto, RTL) must command these actuators to achieve desired motion. However, modes should not be coupled to hardware PWM details (pulse width, duty cycle, frequency).

### Problem

We need an actuator abstraction that:

- Decouples vehicle logic from PWM hardware details
- Accepts normalized commands (-1.0 to +1.0 range) from modes
- Converts normalized values to PWM signals (1000-2000 μs pulse width)
- Enforces critical safety checks (armed state, neutral fail-safe)
- Supports per-actuator calibration (min/max/trim values)
- Works across different rover platforms (Ackermann, skid-steer)

### Constraints

- **Safety Critical**: Actuator layer is primary enforcement point for armed state check
- **Real-time**: Conversion must be fast (< 100 μs per command)
- **Memory Budget**: Actuator state < 500 bytes RAM
- **PWM Standard**: 50 Hz frequency, 1000-2000 μs pulse width (servo/ESC standard)

### Prior Art

**ArduPilot SRV_Channel Class**:

```cpp
class SRV_Channel {
public:
    void set_output_norm(float value);  // -1.0 to +1.0
    void set_output_pwm(uint16_t pwm);  // 1000-2000 μs
    void set_trim(uint16_t trim);       // Neutral position
};

class AP_Motors {
    void set_steering(float normalized);  // -1.0 to +1.0
    void set_throttle(float normalized);
};
```

**PX4 Actuator Mixer**:

- Mixer converts control allocations to PWM outputs
- Supports multiple mixing strategies (Ackermann, differential, omnidirectional)
- Calibration stored in parameters

## Success Metrics

- **Safety**: 100% of commands check armed state (no bypasses)
- **Accuracy**: Normalized → PWM conversion accurate to ±1 μs
- **Performance**: Command processing < 100 μs
- **Testability**: Modes can be unit tested without hardware (mock actuators)

## Decision

**We will implement an ActuatorInterface trait that accepts normalized commands (-1.0 to +1.0), enforces armed state checks, converts to PWM pulse width, and delegates to platform PwmInterface.**

### Architecture

```
┌─────────────────────────────────────────────┐
│         Control Modes                       │
│  (Manual, Auto, RTL)                        │
│  Commands: set_steering(f32), set_throttle(f32) │
└───────────────┬─────────────────────────────┘
                │ Normalized values (-1.0 to +1.0)
┌───────────────▼─────────────────────────────┐
│      Actuator Abstraction Layer             │
│  - SAFETY: Enforce armed check              │
│  - Convert normalized → PWM pulse width     │
│  - Apply calibration (min/max/trim)         │
│  - Convert pulse width → duty cycle         │
└───────────────┬─────────────────────────────┘
                │ PWM duty cycle (5-10%)
┌───────────────▼─────────────────────────────┐
│       Platform PWM Interface                │
│  (PwmInterface trait, RP2040/RP2350 impl)   │
└─────────────────────────────────────────────┘
```

### Decision Drivers

1. **Safety First**: Armed check is critical, must be enforced at actuator layer
2. **Hardware Independence**: Modes work with normalized values, not PWM details
3. **Calibration**: Servo/ESC trim values vary, need per-actuator calibration
4. **Testability**: Mock actuators for unit testing modes without hardware
5. **ArduPilot Compatibility**: Similar pattern, proven in production systems

### Considered Options

- **Option A: Actuator Trait with Mixer** ⭐ Selected
- **Option B: Direct PWM Control from Modes**
- **Option C: Centralized Mixer with Allocation Matrix**

### Option Analysis

**Option A: Actuator Trait with Mixer**

- **Pros**:
  - Clean separation: modes → normalized, actuator → PWM
  - Armed check enforced at actuator layer (defense in depth)
  - Easy to add calibration (per-actuator parameters)
  - Testable: mock actuator trait for mode unit tests
  - Straightforward implementation (simple linear interpolation)
- **Cons**:
  - Additional abstraction layer (minimal overhead)
  - Each actuator needs calibration parameters
- **Estimated Overhead**: \~400 bytes RAM, \~5 KB Flash

**Option B: Direct PWM Control from Modes**

- **Pros**:
  - Simplest approach (no abstraction)
  - Minimal code
  - Direct hardware access
- **Cons**:
  - Modes coupled to PWM hardware details
  - Armed check must be in every mode (error-prone)
  - Hard to add calibration (duplicated logic)
  - Not testable (modes require hardware)
  - Calibration changes require mode modifications
- **Estimated Overhead**: \~200 bytes RAM, \~3 KB Flash

**Option C: Centralized Mixer with Allocation Matrix**

- **Pros**:
  - Supports complex mixing (differential steering, omnidirectional)
  - Single mixing logic for all vehicle types
  - Matches PX4 architecture
- **Cons**:
  - Overkill for simple Ackermann rover (2 actuators)
  - Higher memory overhead (matrix storage)
  - More complex than needed
  - Harder to understand and maintain
- **Estimated Overhead**: \~1 KB RAM, \~10 KB Flash

## Rationale

**Actuator Trait with Mixer** was selected for:

1. **Safety**: Armed check enforced at single point (actuator layer)
2. **Separation of Concerns**: Modes work with normalized values, actuators handle PWM
3. **Calibration**: Per-actuator min/max/trim parameters
4. **Testability**: Modes can be unit tested with mock actuators
5. **Simplicity**: Ackermann rover needs simple linear interpolation, not complex mixer

### Trade-offs Accepted

- **Abstraction Overhead**: \~400 bytes RAM (acceptable within NFR-v6kvd budget)
- **Conversion Cost**: \~100 μs per command (negligible at 50 Hz)

**Decision**: We accept minimal overhead for better safety and testability.

## Consequences

### Positive

- **Safety**: Single enforcement point for armed check (defense in depth)
- **Hardware Independence**: Modes independent of PWM details
- **Calibration**: Per-actuator trim values for servo/ESC tuning
- **Testability**: Modes can be unit tested without hardware
- **Maintainability**: PWM changes don't affect mode implementations

### Negative

- **Abstraction**: Additional layer adds \~400 bytes RAM
- **Conversion Cost**: \~100 μs per command (acceptable)

### Neutral

- **Linear Interpolation**: Simple approach, sufficient for Ackermann steering
- **Calibration via Parameters**: Uses parameter system (FR-a1cuu)

## Implementation Notes

### ActuatorInterface Trait

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

### Actuator Implementation

```rust
pub struct Actuators {
    steering_pwm: &'static mut dyn PwmInterface,
    throttle_pwm: &'static mut dyn PwmInterface,
    system_state: &'static SystemState,
    config: ActuatorConfig,
    /// Current commanded values (for telemetry)
    current_steering: f32,
    current_throttle: f32,
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

impl Default for ActuatorConfig {
    fn default() -> Self {
        Self {
            steering_min: 1000,
            steering_neutral: 1500,
            steering_max: 2000,
            throttle_min: 1000,
            throttle_neutral: 1500,
            throttle_max: 2000,
        }
    }
}

impl ActuatorInterface for Actuators {
    fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str> {
        // CRITICAL SAFETY CHECK: Only output if armed
        let output = if self.system_state.is_armed() {
            normalized.clamp(-1.0, 1.0)
        } else {
            0.0 // Neutral steering when disarmed
        };

        // Store for telemetry
        self.current_steering = output;

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

        // Store for telemetry
        self.current_throttle = output;

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

    fn get_steering(&self) -> f32 {
        self.current_steering
    }

    fn get_throttle(&self) -> f32 {
        self.current_throttle
    }
}

impl Actuators {
    /// Create new actuator controller
    pub fn new(
        steering_pwm: &'static mut dyn PwmInterface,
        throttle_pwm: &'static mut dyn PwmInterface,
        system_state: &'static SystemState,
        config: ActuatorConfig,
    ) -> Self {
        Self {
            steering_pwm,
            throttle_pwm,
            system_state,
            config,
            current_steering: 0.0,
            current_throttle: 0.0,
        }
    }

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

### Calibration Parameters

```rust
// Load calibration from parameter system (FR-a1cuu)
pub fn load_actuator_config(params: &Parameters) -> ActuatorConfig {
    ActuatorConfig {
        steering_min: params.get_u16("SERVO_STEER_MIN").unwrap_or(1000),
        steering_neutral: params.get_u16("SERVO_STEER_TRIM").unwrap_or(1500),
        steering_max: params.get_u16("SERVO_STEER_MAX").unwrap_or(2000),
        throttle_min: params.get_u16("SERVO_THROT_MIN").unwrap_or(1000),
        throttle_neutral: params.get_u16("SERVO_THROT_TRIM").unwrap_or(1500),
        throttle_max: params.get_u16("SERVO_THROT_MAX").unwrap_or(2000),
    }
}
```

### Safety Event Handling

```rust
// In system state module
impl SystemState {
    /// Set armed state
    pub fn set_armed(&mut self, armed: bool) {
        let prev_armed = self.armed;
        self.armed = armed;

        if prev_armed && !armed {
            // Disarm event occurred
            defmt::info!("Vehicle disarmed");
            // Note: Actuators check armed state on next command
            // Latency: < 20ms (one control loop cycle at 50 Hz)
        }
    }
}
```

### Mock Actuator for Testing

```rust
#[cfg(test)]
pub struct MockActuators {
    steering: f32,
    throttle: f32,
    is_armed: bool,
}

#[cfg(test)]
impl ActuatorInterface for MockActuators {
    fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str> {
        if self.is_armed {
            self.steering = normalized.clamp(-1.0, 1.0);
        } else {
            self.steering = 0.0;
        }
        Ok(())
    }

    fn set_throttle(&mut self, normalized: f32) -> Result<(), &'static str> {
        if self.is_armed {
            self.throttle = normalized.clamp(-1.0, 1.0);
        } else {
            self.throttle = 0.0;
        }
        Ok(())
    }

    fn get_steering(&self) -> f32 {
        self.steering
    }

    fn get_throttle(&self) -> f32 {
        self.throttle
    }
}
```

### Unit Test Example

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_manual_mode_when_disarmed() {
        let mut mock_actuators = MockActuators {
            steering: 0.0,
            throttle: 0.0,
            is_armed: false, // DISARMED
        };

        let mut manual_mode = ManualMode::new(&mock_rc, &mut mock_actuators);

        // Simulate full steering and throttle commands
        mock_rc.set_channel(1, 1.0);  // Full right steering
        mock_rc.set_channel(3, 1.0);  // Full forward throttle

        // Update mode
        manual_mode.update(0.02).unwrap();

        // CRITICAL TEST: Actuators should be neutral when disarmed
        assert_eq!(mock_actuators.get_steering(), 0.0, "Steering should be neutral when disarmed");
        assert_eq!(mock_actuators.get_throttle(), 0.0, "Throttle should be zero when disarmed");
    }

    #[test]
    fn test_normalized_to_pulse_conversion() {
        let config = ActuatorConfig::default();
        let actuators = Actuators::new(/* ... */);

        // Test neutral
        let pulse = actuators.normalized_to_pulse(0.0, config.steering_min, config.steering_neutral, config.steering_max);
        assert_eq!(pulse, 1500, "Neutral should be 1500 μs");

        // Test full left
        let pulse = actuators.normalized_to_pulse(-1.0, config.steering_min, config.steering_neutral, config.steering_max);
        assert_eq!(pulse, 1000, "Full left should be 1000 μs");

        // Test full right
        let pulse = actuators.normalized_to_pulse(1.0, config.steering_min, config.steering_neutral, config.steering_max);
        assert_eq!(pulse, 2000, "Full right should be 2000 μs");
    }
}
```

## Platform Considerations

### Pico W (RP2040)

- PWM hardware: 8 PWM slices (16 channels)
- Steering: GPIO configured as PWM output
- Throttle: GPIO configured as PWM output
- 50 Hz PWM frequency: Standard for servos and ESCs

### Pico 2 W (RP2350)

- PWM hardware: 12 PWM slices (24 channels)
- Same interface as RP2040

### Cross-Platform

Actuator abstraction is platform-independent via `PwmInterface` trait. Platform-specific details handled by `src/platform/*/pwm.rs`.

## Security & Privacy

- No sensitive data handling
- Armed state check prevents unauthorized actuator control

## Monitoring & Logging

- **Actuator Commands**: Log steering/throttle values at 1 Hz for debugging
- **Armed State**: Log disarm events immediately
- **Calibration**: Log loaded calibration parameters on boot
- **Safety Violations**: Log attempts to command actuators when disarmed (should never occur)

## Open Questions

- [ ] Should actuator commands be rate-limited? → Decision: No, modes control update rate (50 Hz)
- [ ] Do we need actuator deadband (near-zero values)? → Method: Start without deadband, add if servos jitter
- [ ] Should we support asymmetric calibration (different min/max ranges)? → Decision: Yes, already supported (separate min/max parameters)

## External References

- Servo PWM Standard: <https://www.servocity.com/how-do-servos-work>
- ArduPilot Servo Functions: <https://ardupilot.org/copter/docs/common-servo.html>
- RC PWM Protocol: <https://www.rcgroups.com/forums/showthread.php?1323503-PWM-to-PPM-encoder-(Tutorial)>
- PX4 Actuator Mixer: <https://docs.px4.io/main/en/concept/mixing.html>

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
