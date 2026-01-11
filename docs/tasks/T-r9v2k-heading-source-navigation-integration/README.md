# T-r9v2k Heading Source and Navigation Integration

## Metadata

- Type: Task
- Status: In Progress

## Links

- Related Analyses:
  - [AN-vcxr7-ahrs-navigation-control-integration](../../analysis/AN-vcxr7-ahrs-navigation-control-integration.md)
- Related Requirements:
  - [FR-2vbe8-navigation-controller](../../requirements/FR-2vbe8-navigation-controller.md)
  - [FR-eyuh8-ahrs-attitude-estimation](../../requirements/FR-eyuh8-ahrs-attitude-estimation.md)
  - [FR-erpze-guided-mode-navigation](../../requirements/FR-erpze-guided-mode-navigation.md)
  - [FR-jm7mj-auto-mode-mission-execution](../../requirements/FR-jm7mj-auto-mode-mission-execution.md)
- Related ADRs:
  - [ADR-h3k9f-heading-source-integration](../../adr/ADR-h3k9f-heading-source-integration.md)
  - [ADR-nzvfy-ahrs-abstraction-architecture](../../adr/ADR-nzvfy-ahrs-abstraction-architecture.md)
- Associated Design Document:
  - [T-r9v2k-design](design.md)
- Associated Plan Document:
  - [T-r9v2k-plan](plan.md)

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
