# T-00034 Compass Calibration via Mission Planner

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-00038-compass-calibration-via-mission-planner](../../analysis/AN-00038-compass-calibration-via-mission-planner.md)
- Related Requirements:
  - [FR-00127-compass-calibration-capability](../../requirements/FR-00127-compass-calibration-capability.md)
  - [FR-00128-fixed-mag-cal-yaw-handler](../../requirements/FR-00128-fixed-mag-cal-yaw-handler.md)
  - [FR-00129-magcal-gps-validation](../../requirements/FR-00129-magcal-gps-validation.md)
  - [NFR-00087-magcal-response-time](../../requirements/NFR-00087-magcal-response-time.md)
- Related ADRs:
  - N/A - Simple implementation, no architectural decisions required
- Associated Design Document:
  - [T-00034-compass-calibration-design](design.md)
- Associated Plan Document:
  - [T-00034-compass-calibration-plan](plan.md)

## Summary

Enable Mission Planner's "Large Vehicle MagCal" feature by advertising compass calibration capability and implementing the `MAV_CMD_FIXED_MAG_CAL_YAW` command handler. This Phase 1 implementation trusts the BNO086's internal magnetometer calibration, requiring only GPS validation before accepting the calibration command.

## Scope

- In scope:
  - Add `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` to AUTOPILOT_VERSION capabilities
  - Implement `MAV_CMD_FIXED_MAG_CAL_YAW` (42006) command handler
  - Validate GPS fix availability before accepting calibration
  - Send STATUSTEXT feedback on success/failure
  - Log calibration attempts for debugging
- Out of scope:
  - WMM (World Magnetic Model) implementation (Phase 2: FR-00102)
  - Hard iron offset calculation and storage (Phase 2)
  - Full onboard calibration (`MAV_CMD_DO_START_MAG_CAL`)
  - MAG_CAL_PROGRESS/REPORT messages
  - Multi-compass support

## Success Metrics

- Capability visible: Mission Planner shows enabled "Large Vehicle MagCal" button when connected
- Command acceptance: `MAV_CMD_FIXED_MAG_CAL_YAW` returns `MAV_RESULT_ACCEPTED` with GPS fix
- GPS validation: Command returns `MAV_RESULT_DENIED` without GPS fix
- Response time: COMMAND_ACK sent within 1 second of command reception
- User feedback: STATUSTEXT message displayed in Mission Planner console
