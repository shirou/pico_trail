# T-00028 Circle Mode Implementation

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00033-circle-mode](../../analysis/AN-00033-circle-mode.md)
- Related Requirements:
  - [FR-00112-circle-mode-implementation](../../requirements/FR-00112-circle-mode-implementation.md)
  - [FR-00111-circle-center-point](../../requirements/FR-00111-circle-center-point.md)
  - [FR-00113-circle-mode-parameters](../../requirements/FR-00113-circle-mode-parameters.md)
  - [NFR-00080-circle-path-calculation-performance](../../requirements/NFR-00080-circle-path-calculation-performance.md)
- Related ADRs:
  - [ADR-00029-circle-mode-path-generation](../../adr/ADR-00029-circle-mode-path-generation.md)
  - [ADR-00022-navigation-controller-architecture](../../adr/ADR-00022-navigation-controller-architecture.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement Circle mode that orbits around a fixed center point at a configurable radius and speed, using the hybrid approach (continuous circle generator + L1 navigation controller) as specified in ADR-00029.

## Scope

- In scope:
  - Create `CircleMode` struct implementing `Mode` trait
  - Implement center point calculation on mode entry
  - Implement continuous look-ahead target point generation
  - Integrate with existing L1 navigation controller
  - Add CIRC_RADIUS, CIRC_SPEED, CIRC_DIR parameters
  - Handle stationary mode (CIRC_RADIUS=0)
  - GPS fix validation on mode entry
  - Unit tests for circle calculations
- Out of scope:
  - Dynamic center point updates via MAVLink (future enhancement)
  - Support for non-circular patterns (figure-8, etc.)
  - Circle mode with altitude control (2D only for rovers)

## Success Metrics

- Path following error < 2m RMS at normal speeds
- Circle calculation overhead < 1ms on RP2350
- All acceptance criteria from FR-00112 satisfied
- Zero regressions in existing tests
