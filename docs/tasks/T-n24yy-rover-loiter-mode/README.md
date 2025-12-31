# T-n24yy Rover Loiter Mode

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-lgfjh-loiter-mode](../../analysis/AN-lgfjh-loiter-mode.md)
- Related Requirements:
  - [FR-aw3h3-rover-loiter-mode](../../requirements/FR-aw3h3-rover-loiter-mode.md)
  - [FR-tzlof-loiter-point-calculation](../../requirements/FR-tzlof-loiter-point-calculation.md)
  - [FR-30t03-loiter-type-parameter](../../requirements/FR-30t03-loiter-type-parameter.md)
  - [FR-w0spp-position-drift-detection](../../requirements/FR-w0spp-position-drift-detection.md)
  - [FR-9s9th-position-correction-navigation](../../requirements/FR-9s9th-position-correction-navigation.md)
  - [NFR-biag9-loiter-drift-detection-performance](../../requirements/NFR-biag9-loiter-drift-detection-performance.md)
  - [NFR-32ade-position-hold-accuracy](../../requirements/NFR-32ade-position-hold-accuracy.md)
- Related ADRs:
  - [ADR-8icsq-vehicle-type-separation](../../adr/ADR-8icsq-vehicle-type-separation.md)
  - [ADR-w9zpl-control-mode-architecture](../../adr/ADR-w9zpl-control-mode-architecture.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement Loiter mode for ground rovers that enables position holding at a fixed GPS point. The mode supports two behavioral types: Type 0 (stop motors, no correction) and Type 1 (active position correction when drift exceeds LOIT_RADIUS threshold). This is distinct from Hold mode which simply stops motors without GPS-based position tracking.

## Scope

- In scope:
  - Create `src/rover/mode/loiter.rs` with `#[cfg(feature = "rover")]` gate
  - Implement `RoverLoiter` struct implementing the `Mode` trait
  - Loiter point calculation on mode entry (current position or projected stop point)
  - LOIT_TYPE parameter support (0=stop, 1=active hold)
  - LOIT_RADIUS parameter support (drift threshold in meters)
  - Position drift detection using Haversine distance calculation
  - Position correction navigation (Type 1) using existing navigation controller
  - Hysteresis to prevent oscillation at radius boundary
  - Unit tests for loiter state machine and calculations
- Out of scope:
  - Boat Loiter mode (separate implementation per ADR-8icsq)
  - Full L1 navigation (uses existing SimpleNavigationController)
  - GPS driver changes (uses existing GpsNavigationState)
  - Parameter persistence (uses existing parameter system)

## Success Metrics

- Functionality: Loiter mode selectable via MAVLink DO_SET_MODE command
- Type 0: Vehicle stops within 2 seconds of mode entry
- Type 1: Vehicle returns to within LOIT_RADIUS when drift exceeds threshold
- Performance: Drift detection completes within 1ms per update (NFR-biag9)
- Accuracy: Position hold within LOIT_RADIUS + GPS accuracy (NFR-32ade)
- Portability: Tests pass on both host and embedded target
- Zero Regressions: All existing tests pass

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../../templates/README.md#task-template-taskmd) in the templates README.
