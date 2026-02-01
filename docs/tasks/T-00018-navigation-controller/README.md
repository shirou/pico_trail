# T-00018 Navigation Controller

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00023-position-target-navigation](../../analysis/AN-00023-position-target-navigation.md)
  - [AN-00003-navigation-approach](../../analysis/AN-00003-navigation-approach.md)
- Related Requirements:
  - [FR-00084-navigation-controller](../../requirements/FR-00084-navigation-controller.md)
  - [NFR-00069-navigation-controller-performance](../../requirements/NFR-00069-navigation-controller-performance.md)
- Related ADRs:
  - [ADR-00022-navigation-controller-architecture](../../adr/ADR-00022-navigation-controller-architecture.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement a navigation controller that calculates steering and throttle commands to navigate from the current GPS position to a target position, using bearing-based steering (L1-lite) for the initial implementation with an upgrade path to full L1 controller.

## Scope

- In scope:
  - Create `src/subsystems/navigation/` module structure
  - Implement `NavigationController` trait for extensibility
  - Implement `SimpleNavigationController` (L1-lite) with bearing-based steering
  - Geo calculation functions (bearing, distance, angle wrapping)
  - `NavigationOutput` struct for steering/throttle/status
  - Unit tests for navigation calculations with known GPS positions
  - Property-based tests for output range guarantees
- Out of scope:
  - Full L1 controller with cross-track error (future Phase 2)
  - Mode integration (Guided/Auto modes will use controller separately)
  - Path following between waypoints (point-to-point only)
  - GPS driver implementation (uses existing GpsNavigationState)

## Success Metrics

- Accuracy: Navigation calculations match expected values for test positions
- Performance: Navigation update completes within 2ms on RP2350 (NFR-00069)
- Portability: Tests pass on both host and embedded target
- Range Compliance: Steering output always in \[-1.0, +1.0], throttle in \[0.0, 1.0]
- Zero Regressions: All existing tests pass
