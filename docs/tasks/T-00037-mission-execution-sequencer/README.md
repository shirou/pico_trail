# T-00037 Mission Execution Sequencer

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00042-mission-execution-gaps](../../analysis/AN-00042-mission-execution-gaps.md)
- Related Requirements:
  - [FR-00136-mission-current-telemetry](../../requirements/FR-00136-mission-current-telemetry.md)
  - [FR-00137-mission-item-reached](../../requirements/FR-00137-mission-item-reached.md)
  - [FR-00134-mission-clear-all](../../requirements/FR-00134-mission-clear-all.md)
  - [FR-00138-mission-set-current](../../requirements/FR-00138-mission-set-current.md)
  - [FR-00139-nav-do-command-separation](../../requirements/FR-00139-nav-do-command-separation.md)
  - [FR-00133-do-change-speed](../../requirements/FR-00133-do-change-speed.md)
  - [FR-00140-waypoint-hold-time](../../requirements/FR-00140-waypoint-hold-time.md)
  - [FR-00135-mission-command-preservation](../../requirements/FR-00135-mission-command-preservation.md)
  - [NFR-00091-mission-telemetry-latency](../../requirements/NFR-00091-mission-telemetry-latency.md)
  - [NFR-00090-mission-command-execution-overhead](../../requirements/NFR-00090-mission-command-execution-overhead.md)
- Related ADRs:
  - [ADR-00034-mission-execution-telemetry-architecture](../../adr/ADR-00034-mission-execution-telemetry-architecture.md)
  - [ADR-00023-unified-waypoint-navigation](../../adr/ADR-00023-unified-waypoint-navigation.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement a platform-agnostic MissionSequencer in the core crate following ArduPilot's AP_Mission architecture: a sequencing engine that drives mission execution through a MissionExecutor trait. Integrate with firmware for telemetry (MISSION_CURRENT, MISSION_ITEM_REACHED), mission management (CLEAR_ALL, SET_CURRENT), and NAV/DO dual-slot command execution.

## Scope

- In scope:
  - MissionExecutor trait (start_command, verify_command, on_mission_complete) in core crate
  - MissionEvent enum and CommandStartResult in core crate
  - Command classification helpers (is_nav_command, cmd_has_location) in core crate
  - MissionSequencer with NAV/DO dual-slot model and hold time in core crate
  - Auto mode MissionExecutor implementation in firmware crate
  - MISSION_CURRENT periodic telemetry (1Hz) in firmware crate
  - MISSION_ITEM_REACHED event-driven telemetry in firmware crate
  - MISSION_CLEAR_ALL and MISSION_SET_CURRENT handlers in firmware crate
  - Command type preservation fix in waypoint_to_mission_item()
  - DO_CHANGE_SPEED execution support
- Out of scope:
  - DO_JUMP and mission loops (complex state, defer)
  - Conditional commands (CONDITION_DELAY, CONDITION_DISTANCE)
  - Mission persistence to flash storage
  - Geofence integration during missions
  - MIS_DONE_BEHAVE parameter (Hold is sufficient)
  - GUIDED mode changes (single waypoint, no sequencing needed)

## Success Metrics

- Host testability: MissionSequencer unit tests pass with `cargo test --lib` (no embedded deps)
- GCS compatibility: Mission Planner displays mission progress via MISSION_CURRENT
- Waypoint feedback: MISSION_ITEM_REACHED sent within 100ms of arrival detection
- NAV/DO separation: DO_CHANGE_SPEED executes without affecting navigation timing
- DO command overhead: Execution completes within a single update cycle (< 20ms)
- Command preservation: GCS receives correct command types during mission download
- Zero regressions: All existing tests pass, RP2350 build succeeds
