# FR-00129 MagCal GPS Fix Validation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00038-compass-calibration-via-mission-planner](../analysis/AN-00038-compass-calibration-via-mission-planner.md)
- Prerequisite Requirements:
  - [FR-00128-fixed-mag-cal-yaw-handler](FR-00128-fixed-mag-cal-yaw-handler.md)
  - [FR-00076-gps-operation-data-management](FR-00076-gps-operation-data-management.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00034-compass-calibration](../tasks/T-00034-compass-calibration/README.md)

## Requirement Statement

The system shall reject magnetometer calibration commands when a valid GPS fix is unavailable, returning an appropriate error status and user feedback.

## Rationale

Large Vehicle MagCal requires the vehicle to be pointed in a known direction relative to true north:

- **Position Verification**: GPS fix confirms the vehicle is stationary and location is known
- **Procedure Compliance**: Ensures user has completed prerequisite steps (acquiring GPS lock)
- **Future WMM Support**: GPS position will be required for Phase 2 WMM-based calibration
- **User Feedback**: Clear error message helps operators troubleshoot calibration issues

Without GPS validation, calibration could be attempted in invalid conditions leading to poor results or user confusion.

## User Story (if applicable)

As an operator, I want clear feedback when calibration cannot proceed due to missing GPS fix, so that I can wait for GPS lock or troubleshoot GPS issues before retrying.

## Acceptance Criteria

- [ ] Check GPS fix status before processing calibration command
- [ ] Reject calibration with `MAV_RESULT_DENIED` when no GPS fix
- [ ] Send STATUSTEXT warning message explaining the failure reason
- [ ] Accept calibration only when 3D GPS fix (or better) is available
- [ ] Return `MAV_RESULT_ACCEPTED` when GPS fix is available

## Technical Details (if applicable)

### Functional Requirement Details

**GPS Fix Check:**

```rust
fn has_gps_fix(&self) -> bool {
    let gps_state = GPS_STATE.lock().await;
    matches!(
        gps_state.fix_type,
        GpsFixType::Fix3D | GpsFixType::DgpsFix | GpsFixType::RtkFloat | GpsFixType::RtkFixed
    )
}
```

**Calibration Rejection Flow:**

```rust
fn handle_fixed_mag_cal_yaw(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
    // GPS validation - required for calibration
    if !self.has_gps_fix() {
        status_notifier::send_warning("Mag cal failed: No GPS fix");
        return MavResult::MAV_RESULT_DENIED;
    }

    // Proceed with calibration...
    MavResult::MAV_RESULT_ACCEPTED
}
```

**STATUSTEXT Message:**

| Condition     | Severity | Message                      |
| ------------- | -------- | ---------------------------- |
| No GPS fix    | Warning  | "Mag cal failed: No GPS fix" |
| GPS available | Info     | "Mag cal accepted"           |

**GPS Fix Types (acceptable for calibration):**

| Fix Type  | Value | Acceptable |
| --------- | ----- | ---------- |
| No Fix    | 0     | No         |
| No GPS    | 1     | No         |
| 2D Fix    | 2     | No         |
| 3D Fix    | 3     | Yes        |
| DGPS      | 4     | Yes        |
| RTK Float | 5     | Yes        |
| RTK Fixed | 6     | Yes        |

## Platform Considerations

### Cross-Platform

N/A - Platform agnostic (GPS state access via shared module)

## Risks & Mitigation

| Risk                        | Impact | Likelihood | Mitigation                              | Validation                   |
| --------------------------- | ------ | ---------- | --------------------------------------- | ---------------------------- |
| GPS state stale or outdated | Low    | Low        | Check GPS timestamp freshness           | Verify with simulated GPS    |
| 2D fix might be sufficient  | Low    | Low        | Document 3D requirement, allow override | Test with 2D and 3D fixes    |
| STATUSTEXT not reaching GCS | Medium | Low        | Ensure STATUSTEXT handler is active     | Verify message in MP console |

## Implementation Notes

- GPS state access via `GPS_STATE` mutex in `src/navigation/gps/state.rs`
- STATUSTEXT messages via `status_notifier` module
- This validation is part of the `handle_fixed_mag_cal_yaw` function
- Consider adding GPS timestamp check to detect stale fix data

## External References

- [MAVLink GPS_FIX_TYPE](https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE)
- [Large Vehicle MagCal documentation](https://ardupilot.org/rover/docs/common-compass-calibration-in-mission-planner.html#large-vehicle-magcal)
