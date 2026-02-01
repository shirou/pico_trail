# T-00019 Unified Waypoint Navigation

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-00024-mission-execution](../../analysis/AN-00024-mission-execution.md)
- Related Requirements:
  - [FR-00090-mission-waypoint-navigation](../../requirements/FR-00090-mission-waypoint-navigation.md)
  - [FR-00087-guided-mode-arm-start](../../requirements/FR-00087-guided-mode-arm-start.md)
  - [FR-00089-mission-start-command](../../requirements/FR-00089-mission-start-command.md)
  - [FR-00088-mission-execution-state](../../requirements/FR-00088-mission-execution-state.md)
  - [FR-00091-position-target-mission-integration](../../requirements/FR-00091-position-target-mission-integration.md)
  - [NFR-00070-navigation-update-rate](../../requirements/NFR-00070-navigation-update-rate.md)
- Related ADRs:
  - [ADR-00023-unified-waypoint-navigation](../../adr/ADR-00023-unified-waypoint-navigation.md)
- Associated Design Document:
  - [design.md](design.md)
- Associated Plan Document:
  - [plan.md](plan.md)

## Summary

Implement unified waypoint navigation using MissionStorage as the single source of truth for both GUIDED and AUTO modes. This enables Mission Planner's standard MISSION_ITEM workflow to control rover navigation.

## Scope

- In scope:
  - Add MissionState global for Idle/Running/Completed tracking
  - Modify navigation_task to read from MissionStorage instead of NAV_TARGET
  - Implement GUIDED mode navigation start on ARM when waypoint exists
  - Add MAV_CMD_MISSION_START handler for AUTO mode
  - Integrate SET_POSITION_TARGET to update MissionStorage
  - Implement waypoint advance logic for AUTO mode
  - Add mission completion detection and mode transition to Hold
- Out of scope:
  - Mission pause/resume functionality (future enhancement)
  - Complex mission commands beyond NAV_WAYPOINT (future phases)
  - RTL mode integration (separate task)

## Success Metrics

- Compatibility: Rover moves when armed in GUIDED mode with Mission Planner-uploaded waypoint
- AUTO mode: Rover navigates through sequential waypoints when MAV_CMD_MISSION_START received
- State tracking: MissionState correctly reflects Idle/Running/Completed
- Backward compatibility: SET_POSITION_TARGET still works via MissionStorage integration
- Zero regressions: All existing tests pass
