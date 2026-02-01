# FR-00046 Mode Entry Sensor Requirement Validation When Armed

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P0
- Category: Mode Management / Safety

## Links

- Parent Analysis: [AN-00013-mode-entry-validation](../analysis/AN-00013-mode-entry-validation.md)
- Prerequisite Requirements:
  - [FR-00044-mode-capability-declaration](FR-00044-mode-capability-declaration.md)
- Related Requirements: [FR-00045-mode-capability-queries](FR-00045-mode-capability-queries.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  - [FR-00058-...](FR-00058-validation-capability-enforcement.md)
  - [FR-00031-...](FR-00031-ekf-health-validation.md)
  - [FR-00059-...](FR-00059-validation-failure-reporting.md)
  - [FR-00039-...](FR-00039-fallback-mode-selection.md)
  - [FR-00048-mode-lifecycle-management](FR-00048-mode-lifecycle-management.md)
  - [FR-00030-...](FR-00030-disarmed-validation-exception.md)
  - [NFR-00057-...](NFR-00057-validation-timing-performance.md)
  - [NFR-00056-...](NFR-00056-validation-memory-overhead.md)
  - [NFR-00054-...](NFR-00054-validation-attempt-logging.md)

## Requirement Statement

The system shall validate sensor requirements before allowing mode entry when armed by checking position estimate if mode requires_position(), velocity estimate if mode requires_velocity(), GPS availability for autopilot modes, and return specific error message for each validation failure.

## Rationale

Preventing mode changes that will fail due to missing sensors is critical for flight safety. When armed, the vehicle must not enter modes that require sensors which are unavailable (e.g., entering Auto mode without GPS fix). Validation prevents unsafe autonomous operation.

## Acceptance Criteria

1. Mode validation function checks sensor requirements when vehicle is armed:
   - If `requires_position()` returns true, check `has_position_estimate()`
   - If `requires_velocity()` returns true, check `has_velocity_estimate()`
   - If `is_autopilot_mode()` returns true, check `has_gps_fix()`
   - Check `has_imu()` for all modes requiring sensors
2. Validation returns specific error for each failure type:
   - `DeniedNoPosition`: "Cannot enter mode: No position estimate"
   - `DeniedNoVelocity`: "Cannot enter mode: No velocity estimate"
   - `DeniedNoGPS`: "Cannot enter mode: GPS not available"
   - `DeniedNoIMU`: "Cannot enter mode: IMU not available"
3. Denied mode changes logged with mode name, reason, timestamp
4. GCS notified via STATUSTEXT message with failure reason

## Success Metrics

- Mode changes denied when sensor requirements not met
- Validation errors include specific sensor missing
- 100% of unsafe mode transitions prevented in testing
- All validation failures logged for post-flight analysis

## Dependencies

- Mode capability queries (FR-00045)
- Sensor health monitoring system (AN-00009)
- GPS system providing fix status
- IMU system providing health status
- EKF/AHRS providing position/velocity estimates

## ArduPilot References

This requirement is based on ArduPilot Rover's mode entry validation:

**File**: `Rover/mode.cpp` (lines 21-54)

```cpp
bool Mode::enter()
{
    nav_filter_status filt_status;
    rover.ahrs.get_filter_status(filt_status);

    const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;

    if (requires_position() && !position_ok) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires position");
        return false;
    }

    if (requires_velocity() && !position_ok && !filt_status.flags.horiz_vel) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires velocity");
        return false;
    }

    return _enter();
}
```

**Related ArduPilot Parameters**: None - Validation logic is hardcoded, not configurable

## Verification Methods

- Unit tests: Mock sensor availability, verify validation denies mode entry
- Integration tests: Test each failure mode (no GPS, no position, no velocity, no IMU)
- HITL testing: Verify validation prevents unsafe mode changes on hardware
- Log analysis: Confirm all denied transitions logged with specific reason

## Notes

- Validation timing budget: < 1ms per mode change
- Validation must complete synchronously before mode change
- Sensor health checks should query monitoring system (AN-00009)
- Phase 1: Basic GPS/IMU checks; Phase 2: Add EKF health validation
