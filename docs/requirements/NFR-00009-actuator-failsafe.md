# NFR-00009 Actuator Fail-Safe (CRITICAL SAFETY)

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-00012-actuator-abstraction](FR-00012-actuator-abstraction.md)
  - [FR-00013-manual-mode](FR-00013-manual-mode.md)
- Dependent Requirements: N/A
- Related Tasks:
  - [T-00007-manual-control-implementation](../tasks/T-00007-manual-control-implementation/README.md)

## Requirement Statement

All actuators (steering and throttle) shall immediately fail-safe to neutral position (steering centered at 1500 μs PWM, throttle zero at 1500 μs PWM) within 20ms when the vehicle is disarmed or RC input is lost, enforced via multi-layer defense in depth architecture.

## Rationale

**CRITICAL SAFETY REQUIREMENT**: Unintended vehicle motion poses severe risks:

- **Runaway vehicle**: Throttle output while disarmed can cause vehicle to drive away
- **Injury risk**: Spinning motors or moving vehicle can injure operators or bystanders
- **Property damage**: Uncontrolled motion can damage vehicle or surroundings
- **Development safety**: Testing requires confidence that disarm immediately stops motion

Multi-layer defense (mode layer + actuator layer + system layer) ensures safety even if one layer fails.

## User Story (if applicable)

The system shall immediately disable all actuator outputs (steering centered, throttle zero) when disarmed or RC lost within 20ms (one control loop cycle) to ensure operators can safely interact with the vehicle and prevent unintended motion.

## Acceptance Criteria

- [ ] **CRITICAL: All actuator outputs neutral (1500 μs PWM) when vehicle disarmed**
- [ ] **CRITICAL: Disarm command disables actuators within 20ms (one control loop cycle at 50 Hz)**
- [ ] **CRITICAL: Armed state checked before EVERY actuator command (defense layer 1)**
- [ ] **CRITICAL: Actuator layer enforces armed check, ignores commands when disarmed (defense layer 2)**
- [ ] **CRITICAL: System layer immediately sets neutral PWM on disarm event (defense layer 3)**
- [ ] RC timeout (> 1 second) triggers fail-safe: neutral actuator outputs
- [ ] Mode exit (e.g., Manual → Hold) sets neutral outputs before transition
- [ ] Neutral PWM values: 1500 μs (7.5% duty cycle at 50 Hz)
- [ ] Fail-safe latency measured: disarm → neutral PWM < 20ms
- [ ] **CRITICAL: Automated safety test: disarm during full throttle, verify immediate stop**
- [ ] **CRITICAL: Automated safety test: send actuator command while disarmed, verify ignored**

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Multi-Layer Defense Architecture:**

```
                User/GCS Disarm Command
                         |
                         v
    ┌────────────────────────────────────────────┐
    │ System Layer (Defense Layer 3)            │
    │ - Receives disarm event                    │
    │ - Immediately broadcasts to all subsystems │
    │ - Latency: < 1ms                           │
    └────────────────────────────────────────────┘
                         |
                         v
    ┌────────────────────────────────────────────┐
    │ Mode Layer (Defense Layer 1)              │
    │ - Checks armed state before commands       │
    │ - Sets neutral on RC loss or mode exit     │
    │ - Latency: 0-20ms (next control loop)      │
    └────────────────────────────────────────────┘
                         |
                         v
    ┌────────────────────────────────────────────┐
    │ Actuator Layer (Defense Layer 2 - PRIMARY)│
    │ - Enforces armed check on EVERY command    │
    │ - Ignores all commands when disarmed       │
    │ - Outputs neutral PWM (1500 μs)            │
    │ - Latency: < 1ms                           │
    └────────────────────────────────────────────┘
                         |
                         v
              Neutral PWM Output (SAFE)
```

**Defense Layer 1 (Mode Layer):**

```rust
impl Mode for ManualMode {
    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        // Check RC status
        if self.rc_input.status == RcStatus::Lost {
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        }

        // Normal operation (armed check in actuator layer)
        let steering = self.rc_input.channel(0);
        let throttle = self.rc_input.channel(2);
        self.actuators.set_steering(steering)?;
        self.actuators.set_throttle(throttle)?;
        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        // Set neutral on mode exit
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }
}
```

**Defense Layer 2 (Actuator Layer - PRIMARY):**

```rust
impl ActuatorInterface for Actuators {
    fn set_steering(&mut self, normalized: f32) -> Result<(), &'static str> {
        // CRITICAL SAFETY CHECK: Primary enforcement point
        let output = if self.system_state.is_armed() {
            normalized.clamp(-1.0, 1.0)
        } else {
            0.0 // Neutral steering when disarmed
        };

        let pulse_us = self.normalized_to_pulse(output, /* ... */);
        let duty = Self::pulse_to_duty_cycle(pulse_us);
        self.steering_pwm.set_duty_cycle(duty)?;
        Ok(())
    }

    fn set_throttle(&mut self, normalized: f32) -> Result<(), &'static str> {
        // CRITICAL SAFETY CHECK: Primary enforcement point
        let output = if self.system_state.is_armed() {
            normalized.clamp(-1.0, 1.0)
        } else {
            0.0 // Zero throttle when disarmed
        };

        let pulse_us = self.normalized_to_pulse(output, /* ... */);
        let duty = Self::pulse_to_duty_cycle(pulse_us);
        self.throttle_pwm.set_duty_cycle(duty)?;
        Ok(())
    }
}
```

**Defense Layer 3 (System Layer):**

```rust
impl SystemState {
    pub fn disarm(&mut self) -> Result<(), &'static str> {
        if !self.is_armed() {
            return Err("Already disarmed");
        }

        // Transition to disarmed state
        self.armed = ArmedState::Disarmed;

        // Broadcast disarm event to all subsystems
        // (Actuator layer will check armed state on next command)

        Ok(())
    }
}
```

**Fail-Safe Latency Target:**

- **Target**: < 20ms (one control loop cycle at 50 Hz)
- **Measurement**: Timestamp disarm command → log PWM change to 1500 μs
- **Validation**: Automated test runs disarm at various throttle levels, measures latency

**Automated Safety Tests:**

Test 1: Disarm during full throttle

```rust
#[test]
fn test_disarm_stops_actuators() {
    let mut system = TestSystem::new();

    // Arm vehicle
    system.arm();

    // Set full throttle in Manual mode
    system.send_rc_channels(steering: 0.0, throttle: 1.0);
    system.wait_control_loop(); // 20ms

    // Verify throttle active
    assert_eq!(system.get_throttle_pwm(), 2000); // Full forward

    // Disarm vehicle
    let disarm_time = system.now();
    system.disarm();

    // Wait one control loop
    system.wait_control_loop(); // 20ms

    // Verify throttle neutral
    let pwm_change_time = system.now();
    assert_eq!(system.get_throttle_pwm(), 1500); // Neutral
    assert!(pwm_change_time - disarm_time < 20_000); // < 20ms
}
```

Test 2: Actuator commands ignored when disarmed

```rust
#[test]
fn test_actuator_ignores_commands_when_disarmed() {
    let mut system = TestSystem::new();

    // Ensure disarmed
    assert!(!system.is_armed());

    // Attempt to set full throttle
    system.actuators.set_throttle(1.0);

    // Verify throttle remains neutral
    assert_eq!(system.get_throttle_pwm(), 1500); // Neutral, not 2000
}
```

**Neutral PWM Values:**

- **1500 μs pulse width** = 7.5% duty cycle at 50 Hz (20ms period)
- Servo neutral: 1500 μs = center position (straight ahead)
- ESC neutral: 1500 μs = stop (no throttle)

## Platform Considerations

### Pico W (RP2040)

- PWM hardware supports neutral output (1500 μs = 7.5% duty cycle)
- Control loop at 50 Hz ensures 20ms latency target achievable

### Pico 2 W (RP2350)

- Same PWM interface as RP2040
- Faster CPU may reduce latency further

### Cross-Platform

Fail-safe mechanism is cross-platform via `PwmInterface` trait.

## Risks & Mitigation

| Risk                                                  | Impact   | Likelihood | Mitigation                                         | Validation                                       |
| ----------------------------------------------------- | -------- | ---------- | -------------------------------------------------- | ------------------------------------------------ |
| **Armed check bypassed (code bug)**                   | CRITICAL | Low        | Multi-layer defense, automated tests, code review  | Safety tests run on every build                  |
| **Disarm event not received (communication failure)** | CRITICAL | Low        | Actuator layer checks armed state on EVERY command | Test: disconnect MAVLink, verify neutral output  |
| **PWM hardware failure (stuck at high duty)**         | CRITICAL | Very Low   | Hardware watchdog (future), emergency stop button  | Manual test: verify PWM output with oscilloscope |
| Latency exceeds 20ms (control loop overrun)           | High     | Low        | Profile control loop, ensure < 10ms execution time | Measure: control loop period + jitter            |

## Implementation Notes

Preferred approaches:

- **Defense in depth**: Never rely on single safety check
- **Actuator layer is primary**: Enforce armed check at lowest level
- **Automated testing**: Safety tests MUST run on every build
- **Fail-safe by design**: Default state (boot, error) is neutral PWM

Known pitfalls:

- **Single point of failure**: Don't rely only on mode layer armed check
- **Optimizing away checks**: Never remove armed check for "performance"
- **Test coverage**: Ensure safety tests cover all paths (normal, error, edge cases)
- **Hardware assumptions**: Don't assume PWM hardware fails safe (it doesn't)

Related code areas:

- `src/vehicle/actuators.rs` - Actuator layer (primary safety enforcement)
- `src/vehicle/modes/` - Mode layer (secondary safety check)
- `src/communication/mavlink/state.rs` - System state (armed/disarmed)
- `tests/safety/` - Automated safety tests (CRITICAL)

## External References

- IEC 61508 Functional Safety: <https://en.wikipedia.org/wiki/IEC_61508>
- Defense in Depth: <https://en.wikipedia.org/wiki/Defense_in_depth_(computing)>
- ArduPilot Arming Checks: <https://ardupilot.org/copter/docs/arming_the_motors.html>
