# T-c26bh Unified Waypoint Navigation

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-zd6uw-mission-execution](../../analysis/AN-zd6uw-mission-execution.md)
- Related Requirements:
  - [FR-m2c9z-mission-waypoint-navigation](../../requirements/FR-m2c9z-mission-waypoint-navigation.md)
  - [FR-pbuh7-guided-mode-arm-start](../../requirements/FR-pbuh7-guided-mode-arm-start.md)
  - [FR-w893v-mission-start-command](../../requirements/FR-w893v-mission-start-command.md)
  - [FR-v6571-mission-execution-state](../../requirements/FR-v6571-mission-execution-state.md)
  - [FR-evl5k-position-target-mission-integration](../../requirements/FR-evl5k-position-target-mission-integration.md)
  - [NFR-iuk5h-navigation-update-rate](../../requirements/NFR-iuk5h-navigation-update-rate.md)
- Related ADRs:
  - [ADR-2hs12-unified-waypoint-navigation](../../adr/ADR-2hs12-unified-waypoint-navigation.md)
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

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../../templates/README.md#task-template-taskmd) in the templates README.
