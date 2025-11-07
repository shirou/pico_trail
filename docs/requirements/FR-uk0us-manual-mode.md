# FR-uk0us Manual Mode Implementation

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
  - [FR-993xy-rc-channels-processing](FR-993xy-rc-channels-processing.md)
  - [FR-q2sjt-control-mode-framework](FR-q2sjt-control-mode-framework.md)
  - [FR-uo1p5-actuator-abstraction](FR-uo1p5-actuator-abstraction.md)

- Dependent Requirements:
  - [FR-sp3at-control-modes](FR-sp3at-control-modes.md)
  - [NFR-jng15-actuator-failsafe](NFR-jng15-actuator-failsafe.md)
  - [NFR-kqvyf-manual-control-latency](NFR-kqvyf-manual-control-latency.md)

- Related Tasks:
  - [T-3irm5-manual-control-implementation](../tasks/T-3irm5-manual-control-implementation/README.md)

## Requirement Statement

The system shall implement Manual mode that provides direct pass-through control from RC input (channel 1 for steering, channel 3 for throttle) to rover actuators without stabilization or filtering, with all actuators disabled when the vehicle is disarmed.

## Rationale

Manual mode is the fundamental vehicle control mode, providing:

- **Direct operator control**: Joystick/RC input directly controls steering and throttle
- **Testing foundation**: Validates actuators (servos, motors) before implementing autonomous modes
- **Safety override**: Operator can take direct control in any situation
- **Development workflow**: Essential for hardware validation and integration testing

Manual mode is required by FR-sp3at (Vehicle Operational Modes) and is the simplest mode, making it the ideal starting point for vehicle control implementation.

## User Story (if applicable)

As an operator, I want to control the rover directly with my joystick in Manual mode, so that I can drive the vehicle manually for testing, troubleshooting, and emergency override situations.

## Acceptance Criteria

- [ ] Manual mode reads RC channel 1 (steering) and channel 3 (throttle)
- [ ] Steering input maps directly to steering actuator (-1.0 left, 0.0 center, +1.0 right)
- [ ] Throttle input maps directly to throttle actuator (-1.0 reverse, 0.0 stop, +1.0 forward)
- [ ] No filtering, rate limiting, or stabilization applied (pure pass-through)
- [ ] **CRITICAL SAFETY: All actuator outputs are zero when vehicle is disarmed**
- [ ] **CRITICAL SAFETY: Disarm command immediately stops all actuators (< 20ms)**
- [ ] **CRITICAL SAFETY: Armed state checked before EVERY actuator command**
- [ ] Mode update rate: 50 Hz minimum (per NFR-ukjvr)
- [ ] RC timeout triggers fail-safe: set actuators to neutral (0.0)
- [ ] Mode entry: No pre-conditions (can always enter Manual mode)
- [ ] Mode exit: Set actuators to neutral before transitioning to another mode
- [ ] Mode visible in MAVLink HEARTBEAT message (custom_mode = 0 for Manual)

## Technical Details (if applicable)

### Functional Requirement Details

**RC Channel Mapping:**

- **Channel 1 (Steering)**: Left/right control
  - -1.0 = Full left
  - 0.0 = Center (straight ahead)
  - +1.0 = Full right
- **Channel 3 (Throttle)**: Forward/reverse control
  - -1.0 = Full reverse (if supported by ESC)
  - 0.0 = Stop
  - +1.0 = Full forward

**Manual Mode Logic:**

```rust
pub struct ManualMode {
    rc_input: &RcInput,
    actuators: &Actuators,
}

impl Mode for ManualMode {
    fn enter(&mut self) -> Result<(), &'static str> {
        // No initialization needed for Manual mode
        Ok(())
    }

    fn update(&mut self, _dt: f32) -> Result<(), &'static str> {
        // Read RC inputs
        let steering = self.rc_input.channel(0); // Channel 1 (0-indexed)
        let throttle = self.rc_input.channel(2); // Channel 3 (0-indexed)

        // Check RC status
        if self.rc_input.status == RcStatus::Lost {
            // RC lost: fail-safe to neutral
            self.actuators.set_steering(0.0)?;
            self.actuators.set_throttle(0.0)?;
            return Ok(());
        }

        // Direct pass-through to actuators
        // Note: Actuator layer enforces armed check (defense in depth)
        self.actuators.set_steering(steering)?;
        self.actuators.set_throttle(throttle)?;

        Ok(())
    }

    fn exit(&mut self) -> Result<(), &'static str> {
        // Set actuators to neutral on mode exit
        self.actuators.set_steering(0.0)?;
        self.actuators.set_throttle(0.0)?;
        Ok(())
    }

    fn name(&self) -> &'static str {
        "Manual"
    }
}
```

**Safety Enforcement:**

Manual mode safety is enforced at **multiple layers** (defense in depth):

1. **Mode Layer**: Checks RC status, sets neutral on RC loss
2. **Actuator Layer**: Enforces armed check, ignores commands when disarmed (see FR-uo1p5)
3. **System Layer**: Disarm event immediately triggers actuator shutdown

**Fail-Safe Behavior:**

- **RC Lost**: Set actuators to neutral (0.0)
- **Disarmed**: Set actuators to neutral (0.0)
- **Mode Exit**: Set actuators to neutral (0.0)

**MAVLink Integration:**

- Manual mode = `custom_mode` 0 in HEARTBEAT message
- Mode change via COMMAND_LONG (MAV_CMD_DO_SET_MODE)
- Mode transitions logged via STATUSTEXT message

## Platform Considerations

### Pico W (RP2040)

N/A - Platform agnostic (mode logic independent of hardware)

### Pico 2 W (RP2350)

N/A - Platform agnostic

### Cross-Platform

Manual mode implementation is fully cross-platform. Platform-specific details are abstracted via:

- `RcInput` trait (RC source abstraction)
- `Actuators` trait (actuator output abstraction)

## Risks & Mitigation

| Risk                                            | Impact   | Likelihood | Mitigation                                                             | Validation                                       |
| ----------------------------------------------- | -------- | ---------- | ---------------------------------------------------------------------- | ------------------------------------------------ |
| **PWM output to motors while disarmed**         | CRITICAL | Medium     | Multi-layer defense: armed check in mode + actuator layer + auto tests | Automated test: send throttle while disarmed     |
| **Disarm command ignored, motors keep running** | CRITICAL | Low        | Actuator layer subscribes to armed state changes, immediate shutdown   | Test: disarm during full throttle, verify < 20ms |
| RC timeout not detected                         | High     | Low        | Check RC status on every update (50 Hz)                                | Test: stop RC_CHANNELS, verify neutral output    |
| Mode transition during full throttle            | Medium   | Low        | Exit method sets neutral before transition                             | Test: mode change at full throttle               |

## Implementation Notes

Preferred approaches:

- **Simple pass-through**: No complex logic, direct mapping RC → actuators
- **Fail-safe first**: Check RC status and armed state before actuator commands
- **Defense in depth**: Safety checks at multiple layers (mode, actuator, system)
- **Stateless mode**: Manual mode has no internal state, only reads RC and writes actuators

Known pitfalls:

- **RC array indexing**: Channel 1 = index 0, Channel 3 = index 2 (off-by-one error)
- **Actuator abstraction leakage**: Do NOT call PWM directly, always use actuator interface
- **Armed check in mode**: Defense in depth, but actuator layer is primary enforcement point
- **Mode exit cleanup**: Always set neutral in `exit()` to prevent stuck throttle

Related code areas:

- `src/vehicle/modes/manual.rs` - Manual mode implementation
- `src/vehicle/mode_manager.rs` - Mode switching logic
- `src/communication/mavlink/handlers/command.rs` - DO_SET_MODE command
- `src/vehicle/actuators.rs` - Actuator interface (see FR-uo1p5)

## External References

- ArduPilot Rover Manual Mode: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_manual.cpp>
- ArduPilot Mode Documentation: <https://ardupilot.org/rover/docs/rover-modes.html#manual-mode>
- MAVLink DO_SET_MODE: <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
