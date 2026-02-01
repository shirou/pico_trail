# T-00039 Break Spin-in-Place Positive Feedback Loop

## Metadata

- Type: Task
- Status: Implementation Complete

## Links

- Related Analyses:
  - [AN-00023-position-target-navigation](../../analysis/AN-00023-position-target-navigation.md)
  - [AN-00044-guided-mode-heading-oscillation](../../analysis/AN-00044-guided-mode-heading-oscillation.md)
- Related Requirements:
  - [FR-00083-guided-mode-navigation](../../requirements/FR-00083-guided-mode-navigation.md)
  - [FR-00084-navigation-controller](../../requirements/FR-00084-navigation-controller.md)
  - [NFR-00069-navigation-controller-performance](../../requirements/NFR-00069-navigation-controller-performance.md)
- Related ADRs:
  - [ADR-00022-navigation-controller-architecture](../../adr/ADR-00022-navigation-controller-architecture.md)
  - [ADR-00033-heading-source-integration](../../adr/ADR-00033-heading-source-integration.md)
- Prerequisite Tasks:
  - [T-00038-fix-guided-mode-heading-oscillation](../T-00038-fix-guided-mode-heading-oscillation/README.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Post-field-testing follow-up to T-00038. Field testing confirmed that PD steering, heading-error throttle reduction, and heading source hysteresis (T-00038) are insufficient to eliminate spinning. The root cause is a self-sustaining positive feedback loop: large heading error produces throttle=0 with steering=1.0, causing maximum-speed spin in place, which induces vibration that corrupts AHRS yaw, which sustains the large heading error.

This task implements four mitigations identified in AN-00023 (Field Testing: Post-T-00038 Analysis): enable slew rate limiting (fix default), cap spin-in-place steering speed, add heading EMA filter, and use gyroscope yaw rate for D-term.

## Scope

- In scope:
  - Fix `max_steering_rate` default from 0.0 to 2.0 (enables slew rate limiting)
  - Add spin-in-place steering cap: limit steering output when throttle is near zero
  - Implement heading exponential moving average (EMA) filter to smooth AHRS noise
  - Use BNO086 gyroscope yaw rate directly for D-term instead of differencing heading
  - Increase `steering_d_gain` default from 0.005 to a more effective value
  - Unit tests for all new behaviors
- Out of scope:
  - Full EKF heading estimation
  - Weighted heading source blending / interpolation
  - L1 or S-curve path following
  - Removal of Priority 3 heading fallback (separate evaluation needed)

## Success Metrics

- Spin-in-place steering is capped: when throttle < 0.1, steering magnitude does not exceed configured limit
- Slew rate limiting is active by default (non-zero `max_steering_rate`)
- Heading EMA filter smooths vibration-induced noise measurably in unit tests
- D-term uses gyroscope yaw rate, providing meaningful damping at normal heading rates
- All existing navigation tests continue to pass
- Embedded build compiles successfully
