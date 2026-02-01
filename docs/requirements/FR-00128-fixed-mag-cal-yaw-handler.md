# FR-00128 Fixed Mag Cal Yaw Command Handler

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00038-compass-calibration-via-mission-planner](../analysis/AN-00038-compass-calibration-via-mission-planner.md)
  - [AN-00028-bno086-imu-integration](../analysis/AN-00028-bno086-imu-integration.md)
- Prerequisite Requirements:
  - [FR-00127-compass-calibration-capability](FR-00127-compass-calibration-capability.md)
  - [FR-00005-mavlink-protocol](FR-00005-mavlink-protocol.md)
  - [FR-00076-gps-operation-data-management](FR-00076-gps-operation-data-management.md)
- Dependent Requirements:
  - [FR-00129-magcal-gps-validation](FR-00129-magcal-gps-validation.md)
  - [FR-00141-compass-yaw-offset-persistence](FR-00141-compass-yaw-offset-persistence.md)
  - [NFR-00087-magcal-response-time](NFR-00087-magcal-response-time.md)
- Related ADRs:
  - N/A - To be created after approval
- Related Tasks:
  - [T-00034-compass-calibration](../tasks/T-00034-compass-calibration/README.md)
  - [T-00040-compass-yaw-offset-persistence](../tasks/T-00040-compass-yaw-offset-persistence/README.md)

## Requirement Statement

The system shall handle the `MAV_CMD_FIXED_MAG_CAL_YAW` (ID 42006) command by validating GPS availability and acknowledging the command, trusting the BNO086's internal magnetometer calibration.

## Rationale

This is a Phase 1 implementation for Mission Planner's "Large Vehicle MagCal" feature:

- **BNO086 Internal Calibration**: The BNO086 continuously calibrates its magnetometer internally and persists calibration state in sensor memory
- **Simplified Workflow**: By trusting BNO086's internal calibration, we avoid implementing WMM tables while still enabling the Mission Planner calibration UI
- **GPS Validation**: Requiring GPS fix ensures the user is following the proper calibration procedure (pointing vehicle in known direction)
- **Standard Protocol**: Uses MAVLink common dialect - no ardupilotmega feature required

Future Phase 2 (FR-00102) may implement full WMM-based calibration if field testing shows BNO086 internal calibration is insufficient.

## User Story (if applicable)

As an operator using Mission Planner, I want to use the Large Vehicle MagCal feature, so that I can verify my compass is calibrated without physically rotating the rover.

## Acceptance Criteria

- [x] Handle `MAV_CMD_FIXED_MAG_CAL_YAW` (command ID 42006) in command dispatcher
- [x] Validate GPS fix is available before accepting command
- [x] Return `MAV_RESULT_ACCEPTED` when GPS fix available
- [x] Return `MAV_RESULT_DENIED` when GPS fix unavailable
- [x] Send STATUSTEXT message indicating calibration result
- [x] Log the provided yaw angle for debugging purposes
- [x] Extract yaw parameter (param1) from command

## Technical Details (if applicable)

### Functional Requirement Details

**MAVLink Command (common.xml):**

```
MAV_CMD_FIXED_MAG_CAL_YAW (42006)
├── Param1: Yaw (deg) - Vehicle yaw in earth frame (0-360)
├── Param2: CompassMask - Target compasses (0 = all)
├── Param3: Latitude (deg) - 0 = use current GPS
├── Param4: Longitude (deg) - 0 = use current GPS
└── Param5-7: Empty
```

**Command Handler Implementation:**

```rust
// In handle_command_long match statement
MavCmd::MAV_CMD_FIXED_MAG_CAL_YAW => {
    (self.handle_fixed_mag_cal_yaw(cmd), false, Vec::new())
}

fn handle_fixed_mag_cal_yaw(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
    // Check GPS fix availability
    if !self.has_gps_fix() {
        status_notifier::send_warning("Mag cal failed: No GPS fix");
        return MavResult::MAV_RESULT_DENIED;
    }

    let yaw_deg = cmd.param1;
    crate::log_info!("Large Vehicle MagCal: yaw={} deg", yaw_deg);

    // BNO086 handles calibration internally
    // Just acknowledge the command
    status_notifier::send_info("Mag cal accepted (BNO086 internal)");
    MavResult::MAV_RESULT_ACCEPTED
}
```

**Protocol Flow:**

```
Mission Planner                          pico_trail
      │                                       │
      │  MAV_CMD_FIXED_MAG_CAL_YAW            │
      │  param1: 0.0 (North)                  │
      │ ─────────────────────────────────────>│
      │                                       │
      │                                       │ 1. Verify GPS fix
      │                                       │ 2. Accept calibration
      │                                       │
      │  COMMAND_ACK (ACCEPTED)               │
      │ <─────────────────────────────────────│
      │                                       │
      │  STATUSTEXT "Mag cal accepted"        │
      │ <─────────────────────────────────────│
```

**BNO086 Calibration Note:**

The BNO086 sensor:

- Runs continuous internal magnetometer calibration
- Calibration improves as sensor experiences different orientations
- Calibration state persists in sensor memory
- No external intervention needed for basic operation

## Platform Considerations

### Cross-Platform

N/A - Platform agnostic (MAVLink command handling)

## Risks & Mitigation

| Risk                                  | Impact | Likelihood | Mitigation                        | Validation                        |
| ------------------------------------- | ------ | ---------- | --------------------------------- | --------------------------------- |
| BNO086 calibration insufficient       | Medium | Low        | Phase 2: implement WMM (FR-00102) | Field testing with compass errors |
| Mission Planner expects more response | Low    | Low        | Test with actual Mission Planner  | Connect and verify workflow       |
| GPS unavailable during calibration    | Low    | Medium     | Return DENIED with STATUSTEXT     | Test with GPS disconnected        |

## Implementation Notes

- Add command case to `handle_command_long` match statement
- Location: `src/communication/mavlink/handlers/command.rs`
- Use `status_notifier::send_info()` and `send_warning()` for feedback
- Reference GPS state from GPS module to check fix availability

**Difference from FR-00102:**

| Aspect          | This FR (Phase 1)         | FR-00102 (Phase 2)           |
| --------------- | ------------------------- | ---------------------------- |
| WMM Tables      | Not required              | Required                     |
| Offset Storage  | None                      | COMPASS_OFS_X/Y/Z parameters |
| Implementation  | \~20 lines                | \~200+ lines                 |
| BNO086 specific | Yes (trusts internal cal) | No (generic approach)        |

## External References

- [MAV_CMD_FIXED_MAG_CAL_YAW](https://mavlink.io/en/messages/common.html#MAV_CMD_FIXED_MAG_CAL_YAW)
- [ArduPilot PR #12863 - Fixed yaw mag calibration](https://github.com/ArduPilot/ardupilot/pull/12863)
- [Large Vehicle MagCal documentation](https://ardupilot.org/rover/docs/common-compass-calibration-in-mission-planner.html#large-vehicle-magcal)
