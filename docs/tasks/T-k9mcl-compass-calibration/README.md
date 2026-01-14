# T-k9mcl Compass Calibration via Mission Planner

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-fgiit-compass-calibration-via-mission-planner](../../analysis/AN-fgiit-compass-calibration-via-mission-planner.md)
- Related Requirements:
  - [FR-m4gcl-compass-calibration-capability](../../requirements/FR-m4gcl-compass-calibration-capability.md)
  - [FR-v8fmy-fixed-mag-cal-yaw-handler](../../requirements/FR-v8fmy-fixed-mag-cal-yaw-handler.md)
  - [FR-t3gpv-magcal-gps-validation](../../requirements/FR-t3gpv-magcal-gps-validation.md)
  - [NFR-q7kcr-magcal-response-time](../../requirements/NFR-q7kcr-magcal-response-time.md)
- Related ADRs:
  - N/A - Simple implementation, no architectural decisions required
- Associated Design Document:
  - [T-k9mcl-compass-calibration-design](design.md)
- Associated Plan Document:
  - [T-k9mcl-compass-calibration-plan](plan.md)

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
  - WMM (World Magnetic Model) implementation (Phase 2: FR-e2urj)
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
