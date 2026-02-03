# T-00042 Configurable Turn Behavior

## Metadata

- Type: Task
- Status: Implementation Complete

## Links

- Related Analyses:
  - [AN-00045-configurable-turn-behavior](../../analysis/AN-00045-configurable-turn-behavior.md)
  - [AN-00044-guided-mode-heading-oscillation](../../analysis/AN-00044-guided-mode-heading-oscillation.md)
- Related Requirements:
  - [FR-00144-configurable-pivot-turn-threshold](../../requirements/FR-00144-configurable-pivot-turn-threshold.md)
  - [FR-00145-arc-turn-minimum-throttle](../../requirements/FR-00145-arc-turn-minimum-throttle.md)
  - [NFR-00093-arc-pivot-transition-continuity](../../requirements/NFR-00093-arc-pivot-transition-continuity.md)
  - [FR-00146-navigation-parameter-store](../../requirements/FR-00146-navigation-parameter-store.md)
  - [FR-00084-navigation-controller](../../requirements/FR-00084-navigation-controller.md)
- Related ADRs:
  - [ADR-00022-navigation-controller-architecture](../../adr/ADR-00022-navigation-controller-architecture.md)
- Prerequisite Tasks:
  - [T-00039-break-spin-feedback-loop](../T-00039-break-spin-feedback-loop/README.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Add configurable pivot turn angle threshold and arc turn minimum throttle to `SimpleNavigationController`. When heading error is below the threshold, the rover maintains forward throttle and turns in arcs; above it, pivot turns are allowed (current behavior). This follows ArduPilot's `WP_PIVOT_ANGLE` concept and applies to all autonomous modes (Guided, Auto, RTL, SmartRTL, Loiter, Circle).

## Scope

- In scope:
  - Add `pivot_turn_angle` config field (default 60.0°)
  - Add `arc_turn_min_throttle` config field (default 0.15)
  - Modify `calculate_throttle()` to apply throttle floor during arc turns
  - Unit tests for boundary behavior, continuity, and edge cases
  - Verify all autonomous modes use the shared controller (no per-mode changes needed)
  - Register all `SimpleNavConfig` fields in parameter store as `NavigationParams`
  - Parameters configurable at runtime via MAVLink GCS
- Out of scope:
  - `WP_PIVOT_RATE` (turn rate during pivot turns)
  - `DifferentialDrive::mix()` modifications
  - Full turn rate PID controller

## Success Metrics

- Heading error < `pivot_turn_angle` → throttle >= `arc_turn_min_throttle` (arc turn)
- Heading error >= `pivot_turn_angle` → throttle follows existing curve (pivot allowed)
- `pivot_turn_angle=0` preserves current behavior exactly
- Transition at boundary is continuous (throttle delta < 0.1)
- All existing navigation tests pass
- All 12 `SimpleNavConfig` fields registered in parameter store
- Parameters visible and editable in GCS
- Embedded build compiles successfully
