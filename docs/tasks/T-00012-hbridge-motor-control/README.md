# T-00012 H-bridge Motor Control

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-00018-freenove-hardware-support](../../analysis/AN-00018-freenove-hardware-support.md)
- Related Requirements:
  - [FR-00064-hbridge-motor-control](../../requirements/FR-00064-hbridge-motor-control.md)
  - [FR-00065-pin-configuration-management](../../requirements/FR-00065-pin-configuration-management.md)
- Related ADRs:
  - [ADR-00015-motor-driver-abstraction](../../adr/ADR-00015-motor-driver-abstraction.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement H-bridge motor driver abstraction with zero-cost trait interface supporting forward, reverse, brake, and coast operations with PWM-based speed control, enabling differential drive vehicles like the Freenove 4WD Car to control individual DC motors with hardware safety enforcement.

## Scope

- In scope:
  - Create `src/libraries/motor_driver/` module with Motor trait
  - Implement H-bridge motor control with 2-pin PWM interface (IN1, IN2)
  - RP2350 platform implementation using rp235x-hal PWM
  - Motor groups for coordinating left/right motor pairs
  - Armed state enforcement (motors disabled when disarmed)
  - Speed control with \[-1.0, +1.0] range (PWM duty cycle)
  - DRV8837 H-bridge control logic (forward, reverse, brake, coast)
- Out of scope:
  - Advanced motor control (PID, current limiting, feedback control)
  - Motor encoder support (future work)
  - Other motor types (DShot ESCs, stepper motors) - future work
  - Per-motor trim/calibration (future enhancement)

## Success Metrics

- Zero-cost abstraction: Motor trait methods inline to direct GPIO writes (cargo asm verification)
- Safety: Motors cannot run when system is disarmed (enforced at trait level)
- Performance: Motor control latency < 100ns overhead vs direct HAL calls
- Portability: Motor trait independent of platform implementation (works on RP2350, future ESP32/STM32)
