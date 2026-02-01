# T-00038 Fix Guided Mode Heading Oscillation

## Metadata

- Type: Task
- Status: Implemented

## Links

- Related Analyses:
  - [AN-00044-guided-mode-heading-oscillation](../../analysis/AN-00044-guided-mode-heading-oscillation.md)
- Related ADRs:
  - [ADR-00033-heading-source-integration](../../adr/ADR-00033-heading-source-integration.md)
  - [ADR-00022-navigation-controller-architecture](../../adr/ADR-00022-navigation-controller-architecture.md)
- Dependent Tasks:
  - [T-00039-break-spin-feedback-loop](../T-00039-break-spin-feedback-loop/README.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Fix a field-observed bug where the rover spins in circles during Guided mode navigation. The analysis (AN-00044) identified five root causes: P-only steering with no damping, heading source hard-switching without hysteresis, GPS COG timing lag, throttle independent of heading error, and no heading source blending. This task implements the immediate mitigations (steering rate limiting, PD steering, heading-error-based throttle reduction, heading source hysteresis) to eliminate the oscillation loop.

## Scope

- In scope:
  - Add steering rate limiting (slew rate) to `SimpleNavigationController`
  - Upgrade steering from P-only to PD (proportional-derivative) control
  - Reduce throttle proportionally to heading error
  - Add hysteresis band to `FusedHeadingSource` speed threshold
  - Add configuration fields for new parameters
  - Unit tests for all new behaviors
- Out of scope:
  - Full PID controller with integral term (future tuning if needed)
  - Weighted heading source blending / interpolation (future optimization)
  - L1 or S-curve path following (separate feature)
  - Full EKF heading estimation

## Success Metrics

- Steering output is damped: steering change per control cycle does not exceed configured slew rate
- Heading error reduces throttle: throttle is 0% at 90+ degree heading error
- Heading source does not oscillate at speed threshold: hysteresis prevents switching more than once per second under steady conditions
- All existing navigation tests continue to pass
- Embedded build compiles successfully
