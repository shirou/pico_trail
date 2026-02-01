# T-00040 Compass Yaw Offset Persistence

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00043-compass-calibration-true-north-handling](../../analysis/AN-00043-compass-calibration-true-north-handling.md)
- Related Requirements:
  - [FR-00141-compass-yaw-offset-persistence](../../requirements/FR-00141-compass-yaw-offset-persistence.md)
  - [FR-00128-fixed-mag-cal-yaw-handler](../../requirements/FR-00128-fixed-mag-cal-yaw-handler.md)
  - [FR-00022-configuration-persistence](../../requirements/FR-00022-configuration-persistence.md)
- Related ADRs:
  - [ADR-00035-compass-yaw-offset-calibration](../../adr/ADR-00035-compass-yaw-offset-calibration.md)
- Associated Design Document:
  - [T-00040-compass-yaw-offset-persistence-design](design.md)
- Associated Plan Document:
  - [T-00040-compass-yaw-offset-persistence-plan](plan.md)

## Summary

Persist the `compass_yaw_offset` across reboots by adding a `COMPASS_DEC` parameter to the existing `CompassParams` and `ParameterStore` infrastructure. This ensures that compass calibration performed via Mission Planner's Large Vehicle MagCal survives power cycles. Also fix the doc comment in `handle_fixed_mag_cal_yaw` that incorrectly refers to "magnetic north" instead of "true north".

## Scope

- In scope:
  - Add `COMPASS_DEC` parameter to `CompassParams` struct and registration
  - Load `COMPASS_DEC` on boot and initialize `SystemState.compass_yaw_offset`
  - Save `COMPASS_DEC` to ParameterStore after successful `MAV_CMD_FIXED_MAG_CAL_YAW` calibration
  - Fix doc comment in `handle_fixed_mag_cal_yaw` ("magnetic north = 0" → "true north = 0")
  - Add unit tests for parameter registration, save, and restore
- Out of scope:
  - Hard iron offset calibration (FR-00102)
  - WMM table integration
  - `MAV_CMD_FIXED_MAG_CAL` (42004) support
  - Soft iron calibration

## Success Metrics

- `COMPASS_DEC` parameter is registered and visible via MAVLink PARAM protocol
- Calibration offset persists across simulated reboot in unit tests
- `SystemState.compass_yaw_offset` is restored from `COMPASS_DEC` on boot (not hardcoded to `0.0`)
- Doc comment correctly states "true north = 0"
- All existing tests pass; no regressions
- RP2350 build succeeds
