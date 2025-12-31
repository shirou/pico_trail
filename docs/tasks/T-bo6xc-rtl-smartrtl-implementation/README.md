# T-bo6xc RTL and SmartRTL Mode Implementation

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-75408-rtl-mode](../../analysis/AN-75408-rtl-mode.md)
- Related Requirements:
  - [FR-lia7r-rtl-navigate-home](../../requirements/FR-lia7r-rtl-navigate-home.md)
  - [FR-bwqq7-rtl-entry-validation](../../requirements/FR-bwqq7-rtl-entry-validation.md)
  - [FR-zlza4-rtl-arrival-stop](../../requirements/FR-zlza4-rtl-arrival-stop.md)
  - [FR-crqdd-rtl-gps-loss-handling](../../requirements/FR-crqdd-rtl-gps-loss-handling.md)
  - [FR-nywcm-smartrtl-path-recording](../../requirements/FR-nywcm-smartrtl-path-recording.md)
  - [FR-me7q8-smartrtl-path-simplification](../../requirements/FR-me7q8-smartrtl-path-simplification.md)
  - [FR-8dug4-smartrtl-return-navigation](../../requirements/FR-8dug4-smartrtl-return-navigation.md)
  - [FR-hibx1-smartrtl-rtl-fallback](../../requirements/FR-hibx1-smartrtl-rtl-fallback.md)
  - [NFR-u885h-rtl-update-rate](../../requirements/NFR-u885h-rtl-update-rate.md)
  - [NFR-dwe8t-rtl-memory-overhead](../../requirements/NFR-dwe8t-rtl-memory-overhead.md)
  - [NFR-nm9hf-rtl-entry-validation-time](../../requirements/NFR-nm9hf-rtl-entry-validation-time.md)
  - [NFR-leuwp-smartrtl-memory-budget](../../requirements/NFR-leuwp-smartrtl-memory-budget.md)
- Related ADRs:
  - [ADR-cg5iz-rtl-smartrtl-architecture](../../adr/ADR-cg5iz-rtl-smartrtl-architecture.md)
  - [ADR-w9zpl-control-mode-architecture](../../adr/ADR-w9zpl-control-mode-architecture.md)
  - [ADR-wrcuk-navigation-controller-architecture](../../adr/ADR-wrcuk-navigation-controller-architecture.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement RTL (Return to Launch) and SmartRTL modes for the pico_trail rover. SmartRTL is the default mode that retraces the recorded path in reverse for safe navigation, while direct RTL serves as a fallback providing straight-line navigation to home when no recorded path is available.

## Scope

- In scope:
  - Create `src/subsystems/navigation/path_recorder.rs` for path recording infrastructure
  - Create `src/rover/mode/rtl.rs` with direct RTL mode implementation
  - Create `src/rover/mode/smartrtl.rs` with SmartRTL mode implementation
  - Implement path recording during armed operation (ring buffer)
  - Implement path simplification using distance-based algorithm
  - RTL entry validation (GPS fix, home position)
  - GPS loss handling (transition to Hold mode)
  - Arrival detection and stop behavior
  - RTL mode selection logic (SmartRTL if path available, else direct RTL)
  - Unit tests for path recording, navigation, and mode logic
- Out of scope:
  - RTL_SPEED parameter (use WP_SPEED)
  - RTL_OPTIONS bitmask (defer to future enhancement)
  - Path persistence across disarm (clear on arm per ArduPilot behavior)
  - Obstacle avoidance (relies on recorded path safety)

## Success Metrics

- Functionality: RTL mode selectable via MAVLink DO_SET_MODE command
- SmartRTL: Vehicle retraces recorded path to home position
- Direct RTL: Vehicle navigates straight to home when path unavailable
- GPS Loss: Vehicle transitions to Hold mode within 100ms of GPS loss
- Arrival: Vehicle stops within WP_RADIUS of home position
- Memory: Path buffer uses less than 10 KB RAM (NFR-leuwp)
- Performance: Navigation update completes within 20ms (50 Hz) (NFR-u885h)
- Entry validation: Completes within 1ms (NFR-nm9hf)
- Zero Regressions: All existing tests pass

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../../templates/README.md#task-template-taskmd) in the templates README.
