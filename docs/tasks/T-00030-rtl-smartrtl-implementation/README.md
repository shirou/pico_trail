# T-00030 RTL and SmartRTL Mode Implementation

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00036-rtl-mode](../../analysis/AN-00036-rtl-mode.md)
- Related Requirements:
  - [FR-00122-rtl-navigate-home](../../requirements/FR-00122-rtl-navigate-home.md)
  - [FR-00120-rtl-entry-validation](../../requirements/FR-00120-rtl-entry-validation.md)
  - [FR-00119-rtl-arrival-stop](../../requirements/FR-00119-rtl-arrival-stop.md)
  - [FR-00121-rtl-gps-loss-handling](../../requirements/FR-00121-rtl-gps-loss-handling.md)
  - [FR-00123-smartrtl-path-recording](../../requirements/FR-00123-smartrtl-path-recording.md)
  - [FR-00124-smartrtl-path-simplification](../../requirements/FR-00124-smartrtl-path-simplification.md)
  - [FR-00125-smartrtl-return-navigation](../../requirements/FR-00125-smartrtl-return-navigation.md)
  - [FR-00126-smartrtl-rtl-fallback](../../requirements/FR-00126-smartrtl-rtl-fallback.md)
  - [NFR-00085-rtl-update-rate](../../requirements/NFR-00085-rtl-update-rate.md)
  - [NFR-00084-rtl-memory-overhead](../../requirements/NFR-00084-rtl-memory-overhead.md)
  - [NFR-00083-rtl-entry-validation-time](../../requirements/NFR-00083-rtl-entry-validation-time.md)
  - [NFR-00086-smartrtl-memory-budget](../../requirements/NFR-00086-smartrtl-memory-budget.md)
- Related ADRs:
  - [ADR-00031-rtl-smartrtl-architecture](../../adr/ADR-00031-rtl-smartrtl-architecture.md)
  - [ADR-00013-control-mode-architecture](../../adr/ADR-00013-control-mode-architecture.md)
  - [ADR-00022-navigation-controller-architecture](../../adr/ADR-00022-navigation-controller-architecture.md)
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
- Memory: Path buffer uses less than 10 KB RAM (NFR-00086)
- Performance: Navigation update completes within 20ms (50 Hz) (NFR-00085)
- Entry validation: Completes within 1ms (NFR-00083)
- Zero Regressions: All existing tests pass
