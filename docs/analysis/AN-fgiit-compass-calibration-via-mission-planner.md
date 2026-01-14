# AN-fgiit Compass Calibration via Mission Planner

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-srhcj-bno086-imu-integration](../analysis/AN-srhcj-bno086-imu-integration.md)
- Related Requirements:
  - [FR-m4gcl-compass-calibration-capability](../requirements/FR-m4gcl-compass-calibration-capability.md)
  - [FR-v8fmy-fixed-mag-cal-yaw-handler](../requirements/FR-v8fmy-fixed-mag-cal-yaw-handler.md)
  - [FR-t3gpv-magcal-gps-validation](../requirements/FR-t3gpv-magcal-gps-validation.md)
  - [NFR-q7kcr-magcal-response-time](../requirements/NFR-q7kcr-magcal-response-time.md)
  - [FR-e2urj-large-vehicle-magcal](../requirements/FR-e2urj-large-vehicle-magcal.md) (Phase 2: WMM-based)
- Related ADRs:
  - N/A - To be created after approval
- Related Tasks:
  - [T-k9mcl-compass-calibration](../tasks/T-k9mcl-compass-calibration/README.md)

## Executive Summary

This analysis investigates implementing compass calibration for pico_trail via Mission Planner, focusing specifically on the **Large Vehicle MagCal** feature. This approach is ideal for rovers that cannot be easily rotated on all axes.

**Key Findings:**

- Mission Planner disables calibration UI when `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` is not advertised
- The capability flag is defined in common.xml - no MAVLink feature changes needed
- `MAV_CMD_FIXED_MAG_CAL_YAW` (ID 42006) is also in common.xml
- Implementation requires only: capability flag addition + command handler
- **ardupilotmega feature is NOT required**

## Problem Space

### Current State

The pico_trail autopilot uses BNO086 for AHRS with the following characteristics:

1. **Compass Calibration UI**: Mission Planner calibration buttons are **disabled**
2. **Capability Advertisement**: `COMPASS_CALIBRATION` capability is not set in `AUTOPILOT_VERSION`
3. **MAVLink Configuration**: Uses "common" dialect only
4. **BNO086 Internal Calibration**: Runs continuously but not exposed to GCS

**Current Capabilities** (`src/communication/mavlink/handlers/command.rs:698-703`):

```rust
let capabilities = MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_INT
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT;
// Missing: MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION
```

### Desired State

- Mission Planner "Large Vehicle MagCal" button is enabled and functional
- Users can calibrate compass by pointing rover in known direction
- Calibration completes without physical rotation of the rover

### Gap Analysis

| Component                      | Current State  | Required State            |
| ------------------------------ | -------------- | ------------------------- |
| COMPASS_CALIBRATION capability | Not advertised | Add to AUTOPILOT_VERSION  |
| MAV_CMD_FIXED_MAG_CAL_YAW      | Not handled    | Handler implemented       |
| MAVLink dialect                | common         | common (no change needed) |

## Stakeholder Analysis

| Stakeholder           | Interest/Need                     | Impact | Priority |
| --------------------- | --------------------------------- | ------ | -------- |
| Rover Operator        | Calibration without lifting rover | High   | P0       |
| Navigation System     | Accurate heading data             | High   | P0       |
| GCS (Mission Planner) | Standard capability advertisement | High   | P0       |

## Research & Discovery

### Mission Planner Calibration UI Enable Condition

Mission Planner checks the `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` flag in the `AUTOPILOT_VERSION` message to determine whether to enable calibration UI.

From [Mission Planner PR #1550](https://github.com/ArduPilot/MissionPlanner/pull/1550):

| Capability Flag | Mission Planner Behavior                              |
| --------------- | ----------------------------------------------------- |
| Present         | Shows Onboard Calibration option                      |
| Absent          | Shows both Onboard and Live Calibration (or disables) |

**MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION** is defined in `common.xml`:

```xml
<entry value="4096" name="MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION">
  <description>Autopilot supports onboard compass calibration.</description>
</entry>
```

### Large Vehicle MagCal Protocol

Unlike onboard calibration (which requires rotating the vehicle), Large Vehicle MagCal works by:

1. User points vehicle in a **known direction** (e.g., North)
2. GCS sends `MAV_CMD_FIXED_MAG_CAL_YAW` with the known yaw angle
3. Autopilot uses **GPS position** + **WMM tables** to calculate expected magnetic field
4. Compares expected field with measured field to compute offsets
5. Saves offsets to parameters

**MAV_CMD_FIXED_MAG_CAL_YAW (ID 42006):**

| Parameter | Type  | Description                             |
| --------- | ----- | --------------------------------------- |
| param1    | float | Yaw of vehicle in earth frame (degrees) |
| param2    | float | CompassMask, 0 for all                  |
| param3    | float | Latitude (0 = use current GPS)          |
| param4    | float | Longitude (0 = use current GPS)         |
| param5-7  | -     | Empty                                   |

**Key Advantage:** Both the capability flag and command are in `common.xml`, so **no MAVLink feature changes are required**.

### Protocol Flow

```
Mission Planner                          pico_trail
      │                                       │
      │  Request AUTOPILOT_VERSION            │
      │ ─────────────────────────────────────>│
      │                                       │
      │  AUTOPILOT_VERSION                    │
      │  capabilities: COMPASS_CALIBRATION    │
      │ <─────────────────────────────────────│
      │                                       │
      │  [Calibration UI becomes enabled]     │
      │                                       │
      │  User points rover North              │
      │  and clicks "Large Vehicle MagCal"    │
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
      │  STATUSTEXT "Mag cal complete"        │
      │ <─────────────────────────────────────│
```

### BNO086 Considerations

The BNO086 has internal auto-calibration:

- Continuously calibrates its magnetometer internally
- Calibration improves as the sensor experiences different orientations
- Calibration state persists in sensor memory

**Implementation Strategy:** Trust BNO086's internal calibration. The `MAV_CMD_FIXED_MAG_CAL_YAW` handler will verify GPS fix and return ACCEPTED, allowing the user to trigger BNO086's calibration improvement through normal vehicle movement.

## Discovered Requirements

### Functional Requirements

- [x] **[FR-m4gcl](../requirements/FR-m4gcl-compass-calibration-capability.md)**: System shall advertise MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION
  - Rationale: Required for Mission Planner to enable calibration UI
  - Acceptance Criteria: Capability flag set in AUTOPILOT_VERSION message

- [x] **[FR-v8fmy](../requirements/FR-v8fmy-fixed-mag-cal-yaw-handler.md)**: System shall handle MAV_CMD_FIXED_MAG_CAL_YAW command
  - Rationale: Required for Mission Planner Large Vehicle MagCal
  - Acceptance Criteria: Command returns ACCEPTED when GPS fix available

- [x] **[FR-t3gpv](../requirements/FR-t3gpv-magcal-gps-validation.md)**: System shall reject calibration when GPS fix is unavailable
  - Rationale: Large Vehicle MagCal requires known position
  - Acceptance Criteria: Returns DENIED with STATUSTEXT explanation

### Non-Functional Requirements

- [x] **[NFR-q7kcr](../requirements/NFR-q7kcr-magcal-response-time.md)**: Calibration response shall complete within 1 second
  - Category: Performance
  - Rationale: Simple validation, no complex calculation in Phase 1
  - Target: <1s from command to ACK

## Design Considerations

### Technical Constraints

**Hardware:**

- BNO086 provides fused quaternion output with internal calibration
- GPS provides position for validation

**Software:**

- MAVLink "common" dialect includes both capability flag and command
- No additional MAVLink features required

### Implementation (Phase 1: BNO086 Trust Mode)

**1. Add Capability Flag:**

```rust
// src/communication/mavlink/handlers/command.rs
let capabilities = MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MISSION_INT
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT
    | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION;  // ADD
```

**2. Add Command Handler:**

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

### Future Enhancement (Phase 2)

If field testing shows BNO086 internal calibration is insufficient:

- Implement WMM declination lookup
- Store declination offset in parameters
- Apply correction to heading output

## Risk Assessment

| Risk                                  | Probability | Impact | Mitigation Strategy              |
| ------------------------------------- | ----------- | ------ | -------------------------------- |
| GPS unavailable during calibration    | Medium      | Low    | Check GPS fix, return DENIED     |
| BNO086 calibration insufficient       | Low         | Medium | Phase 2: add WMM declination     |
| Mission Planner expects more response | Low         | Low    | Test with actual Mission Planner |

## Open Questions

- [x] Is MAV_CMD_FIXED_MAG_CAL_YAW in common.xml?
  - **Answer:** Yes, command ID 42006 is in common.xml
- [x] Is MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION in common.xml?
  - **Answer:** Yes, value 4096 is in common.xml
- [x] Does Mission Planner require ardupilotmega for calibration?
  - **Answer:** No, common dialect is sufficient for Large Vehicle MagCal
- [x] Does Mission Planner require specific response beyond COMMAND_ACK?
  - **Answer:** Yes. See [Implementation Notes](#implementation-notes) below.

## Recommendations

### Implementation Checklist

1. [x] Add `MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION` to capabilities
2. [x] Add `MAV_CMD_FIXED_MAG_CAL_YAW` command handler
3. [x] Verify GPS fix before accepting calibration
4. [x] Send STATUSTEXT on success/failure
5. [x] Test with Mission Planner

### Next Steps

1. [x] Create formal requirements: See [Discovered Requirements](#discovered-requirements)
2. [x] Create task: [T-k9mcl-compass-calibration](../tasks/T-k9mcl-compass-calibration/README.md)

### Out of Scope

- **Full onboard calibration** (MAV_CMD_DO_START_MAG_CAL): Requires ardupilotmega dialect
- **MAG_CAL_PROGRESS/REPORT messages**: Not needed for Large Vehicle MagCal
- **Multi-compass support**: pico_trail has single compass
- **WMM implementation**: Phase 2 if needed

## Appendix

### References

**MAVLink:**

- [MAV_PROTOCOL_CAPABILITY enum](https://mavlink.io/en/messages/common.html)
- [MAV_CMD_FIXED_MAG_CAL_YAW](https://mavlink.io/en/messages/common.html)
- [ArduPilot PR #12863 - Fixed yaw mag calibration](https://github.com/ArduPilot/ardupilot/pull/12863)

**Mission Planner:**

- [PR #1550 - Compass calibration capability check](https://github.com/ArduPilot/MissionPlanner/pull/1550)
- [Large Vehicle MagCal documentation](https://ardupilot.org/rover/docs/common-compass-calibration-in-mission-planner.html#large-vehicle-magcal)

### Capability Flag Definition

From `common.xml`:

```xml
<enum name="MAV_PROTOCOL_CAPABILITY" bitmask="true">
  ...
  <entry value="4096" name="MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION">
    <description>Autopilot supports onboard compass calibration.</description>
  </entry>
  ...
</enum>
```

### Command Definition

From `common.xml`:

```xml
<entry value="42006" name="MAV_CMD_FIXED_MAG_CAL_YAW">
  <description>Magnetometer calibration based on provided known yaw.
  This allows for fast calibration using WMM field tables in the vehicle,
  given only the known yaw of the vehicle. If Latitude and longitude are
  both zero then use the current vehicle location.</description>
  <param index="1" label="Yaw" units="deg">Yaw of vehicle in earth frame.</param>
  <param index="2" label="CompassMask">CompassMask, 0 for all.</param>
  <param index="3" label="Latitude" units="deg">Latitude.</param>
  <param index="4" label="Longitude" units="deg">Longitude.</param>
  <param index="5">Empty.</param>
  <param index="6">Empty.</param>
  <param index="7">Empty.</param>
</entry>
```

### Summary: What's Needed vs Not Needed

| Item                              | Needed?      | Notes                             |
| --------------------------------- | ------------ | --------------------------------- |
| ardupilotmega MAVLink feature     | **No**       | common is sufficient              |
| COMPASS_CALIBRATION capability    | **Yes**      | Enable Mission Planner UI         |
| MAV_CMD_FIXED_MAG_CAL_YAW handler | **Yes**      | Large Vehicle MagCal              |
| MAV_CMD_DO_START_MAG_CAL          | No           | Full onboard cal (out of scope)   |
| MAG_CAL_PROGRESS message          | No           | Not needed for Large Vehicle      |
| MAG_CAL_REPORT message            | No           | Not needed for Large Vehicle      |
| WMM implementation                | No (Phase 1) | BNO086 internal cal is sufficient |

## Implementation Notes

**Completed in Task T-k9mcl**

During implementation, the following additional requirements were discovered beyond the original analysis:

### 1. COMPASS_OFS_X Parameter Requirement

Mission Planner's `ConfigHWCompass.cs` checks for the existence of `COMPASS_OFS_X` parameter before enabling the compass configuration UI. The capability flag alone is insufficient.

**Solution:** Added `COMPASS_OFS_X`, `COMPASS_OFS_Y`, `COMPASS_OFS_Z` parameters in `src/parameters/compass.rs`.

### 2. SYS_STATUS Sensor Flags

Mission Planner also checks `SYS_STATUS.onboard_control_sensors_present` for `MAV_SYS_STATUS_SENSOR_3D_MAG` flag.

**Solution:** Added the 3D magnetometer flag to SYS_STATUS in `src/communication/mavlink/handlers/telemetry.rs:390-395`.

### 3. Yaw Offset Application to Telemetry

The original analysis assumed accepting the calibration command was sufficient. However, for the calibration to have visible effect (correct heading display in GCS), the yaw offset must be calculated and applied to heading reports.

**Solution:**

- Added `compass_yaw_offset` field to `SystemState` (`src/communication/mavlink/state.rs:474-477`)
- Updated `handle_fixed_mag_cal_yaw()` to calculate offset: `offset = true_yaw - ahrs_yaw`
- Applied offset to `ATTITUDE.yaw` and `GLOBAL_POSITION_INT.hdg` messages

**Implementation Details:**

```rust
// In handle_fixed_mag_cal_yaw():
let true_yaw_rad = true_yaw_degrees.to_radians();
let yaw_offset = true_yaw_rad - current_ahrs_yaw;
let yaw_offset = Self::normalize_angle(yaw_offset);  // Normalize to [-π, π]
state.compass_yaw_offset = yaw_offset;

// In build_attitude():
let corrected_yaw = Self::normalize_yaw(state.attitude.yaw + state.compass_yaw_offset);

// In build_global_position_int():
let corrected_heading = Self::normalize_yaw(state.attitude.yaw + state.compass_yaw_offset);
let hdg_cdeg = (corrected_heading.to_degrees() * 100.0) as u16 % 36000;
```

### Updated Protocol Flow

The actual implementation flow differs from the original analysis:

```
Mission Planner                          pico_trail
      │                                       │
      │  Request AUTOPILOT_VERSION            │
      │ ─────────────────────────────────────>│
      │                                       │
      │  AUTOPILOT_VERSION                    │
      │  capabilities: COMPASS_CALIBRATION    │
      │ <─────────────────────────────────────│
      │                                       │
      │  PARAM_REQUEST_LIST                   │
      │ ─────────────────────────────────────>│
      │                                       │
      │  PARAM_VALUE (COMPASS_OFS_X, etc.)    │
      │ <─────────────────────────────────────│
      │                                       │
      │  SYS_STATUS (sensors: 3D_MAG)         │
      │ <─────────────────────────────────────│
      │                                       │
      │  [Calibration UI becomes enabled]     │
      │                                       │
      │  MAV_CMD_FIXED_MAG_CAL_YAW            │
      │  param1: 0.0 (North)                  │
      │ ─────────────────────────────────────>│
      │                                       │
      │                                       │ 1. Verify GPS 3D fix
      │                                       │ 2. Calculate yaw offset
      │                                       │ 3. Store in compass_yaw_offset
      │                                       │
      │  COMMAND_ACK (ACCEPTED)               │
      │ <─────────────────────────────────────│
      │                                       │
      │  STATUSTEXT "MagCal: Calibrated"      │
      │ <─────────────────────────────────────│
      │                                       │
      │  ATTITUDE (yaw includes offset)       │
      │ <─────────────────────────────────────│
      │                                       │
      │  [Heading display shows corrected yaw]│
```

### Files Modified

| File                                              | Changes                                                                                    |
| ------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| `src/communication/mavlink/handlers/command.rs`   | Added capability flag, command handler, yaw offset calculation, 4 unit tests               |
| `src/communication/mavlink/handlers/telemetry.rs` | Added yaw offset application to ATTITUDE and GLOBAL_POSITION_INT, added 3D_MAG sensor flag |
| `src/communication/mavlink/state.rs`              | Added `compass_yaw_offset` field to SystemState                                            |
| `src/parameters/compass.rs`                       | New file with COMPASS_OFS_X/Y/Z parameters                                                 |
| `src/parameters/mod.rs`                           | Added compass module export                                                                |
| `src/communication/mavlink/handlers/param.rs`     | Registered compass parameters                                                              |

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
