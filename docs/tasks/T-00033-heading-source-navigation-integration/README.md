# T-00033 Heading Source and Navigation Integration

## Metadata

- Type: Task
- Status: In Progress

## Links

- Related Analyses:
  - [AN-00037-ahrs-navigation-control-integration](../../analysis/AN-00037-ahrs-navigation-control-integration.md)
- Related Requirements:
  - [FR-00084-navigation-controller](../../requirements/FR-00084-navigation-controller.md)
  - [FR-00001-ahrs-attitude-estimation](../../requirements/FR-00001-ahrs-attitude-estimation.md)
  - [FR-00083-guided-mode-navigation](../../requirements/FR-00083-guided-mode-navigation.md)
  - [FR-00082-auto-mode-mission-execution](../../requirements/FR-00082-auto-mode-mission-execution.md)
- Related ADRs:
  - [ADR-00033-heading-source-integration](../../adr/ADR-00033-heading-source-integration.md)
  - [ADR-00032-ahrs-abstraction-architecture](../../adr/ADR-00032-ahrs-abstraction-architecture.md)
- Associated Design Document:
  - [T-00033-design](design.md)
- Associated Plan Document:
  - [T-00033-plan](plan.md)

## Summary

Integrate AHRS heading data with the navigation system to enable autonomous navigation in Guided and Auto modes. Implement the `HeadingSource` trait to provide fused heading (AHRS + GPS COG) and update the NavigationController to use this abstraction.

## Scope

- In scope:
  - HeadingSource trait and FusedHeadingSource implementation
  - NavigationController update to accept heading parameter
  - Guided mode implementation with SET_POSITION_TARGET_GLOBAL_INT support
  - Auto mode implementation with mission waypoint execution
  - Update existing modes (RTL, SmartRTL, Loiter) to use HeadingSource
- Out of scope:
  - EKF/GSF sensor fusion (deferred to SoftwareAhrs implementation)
  - Attitude stabilization using angular rates
  - L1/S-Curve advanced navigation algorithms

## Success Metrics

- `Heading availability`: Valid heading available within 1 second of AHRS initialization
- `Stationary navigation`: Vehicle can begin navigation from stationary position without heading discontinuity
- `Guided mode`: Vehicle navigates to SET_POSITION_TARGET_GLOBAL_INT target and stops within WP_RADIUS
- `Auto mode`: Vehicle executes uploaded mission visiting all waypoints in sequence
