# T-00011 Differential Drive Kinematics

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-00018-freenove-hardware-support](../../analysis/AN-00018-freenove-hardware-support.md)
- Related Requirements:
  - [FR-00063-differential-drive-kinematics](../../requirements/FR-00063-differential-drive-kinematics.md)
- Related ADRs:
  - [ADR-00014-differential-drive-kinematics](../../adr/ADR-00014-differential-drive-kinematics.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement pure, platform-independent differential drive kinematics conversion from steering/throttle inputs to left/right motor speeds with normalization, enabling differential drive vehicles like the Freenove 4WD Car to execute maneuvers without exceeding motor speed limits.

## Scope

- In scope:
  - Create `src/libraries/kinematics/` module with differential drive implementation
  - Implement steering/throttle → left/right motor speeds conversion
  - Normalization algorithm to keep outputs in \[-1.0, +1.0] range
  - Pure functions with no platform dependencies (no_std compatible)
  - Unit tests matching ArduPilot reference implementation
  - Property-based tests for output range guarantees
- Out of scope:
  - Motor driver integration (separate concern handled by motor_driver library)
  - Control mode implementation (manual/guided/auto modes call kinematics)
  - Other kinematics models (Ackermann steering, mecanum wheels) - future work
  - Advanced features (per-motor trim, feedback control) - future work

## Success Metrics

- Correctness: Unit tests match ArduPilot reference outputs for all test cases
- Portability: Tests pass on both host (cargo test --lib) and embedded (cargo check --target thumbv8m.main-none-eabihf)
- Zero dependencies: Module depends only on core (no platform modules, no hardware abstractions)
- Performance: Pure functions with no heap allocations (verified with #!\[no_std])
