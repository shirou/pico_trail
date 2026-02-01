# AN-2m7km Compass Calibration True North Handling

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-fgiit-compass-calibration-via-mission-planner](AN-fgiit-compass-calibration-via-mission-planner.md)
  - [AN-vcxr7-ahrs-navigation-control-integration](AN-vcxr7-ahrs-navigation-control-integration.md)
  - [AN-lsq0s-guided-mode-heading-oscillation](AN-lsq0s-guided-mode-heading-oscillation.md)
- Related Requirements:
  - [FR-v8fmy-fixed-mag-cal-yaw-handler](../requirements/FR-v8fmy-fixed-mag-cal-yaw-handler.md)
  - [FR-e2urj-large-vehicle-magcal](../requirements/FR-e2urj-large-vehicle-magcal.md)
- Related ADRs:
  - N/A
- Related Tasks:
  - [T-k9mcl-compass-calibration](../tasks/T-k9mcl-compass-calibration/README.md)

## Executive Summary

This analysis investigates whether the current compass calibration implementation correctly handles the **True North** heading input from Mission Planner's Large Vehicle MagCal feature. The investigation confirms that the calibration logic is functionally correct but identifies documentation errors and architectural differences from ArduPilot.

**Key Findings:**

1. **Calibration logic is correct** - The yaw offset calculation (`true_heading - ahrs_yaw`) correctly captures magnetic declination and residual compass error
2. **Doc comment is wrong** - The handler documents param1 as "magnetic north = 0" when it should be "true north = 0"
3. **Approach differs from ArduPilot** - pico_trail stores a yaw correction offset; ArduPilot computes magnetometer hard iron offsets using WMM tables
4. **Offset is volatile** - `compass_yaw_offset` is lost on reboot (not persisted to parameters)
5. **No explicit declination handling** - Declination is implicitly captured in the yaw offset, with no separate `COMPASS_DEC` parameter

## Problem Space

### Current State

Mission Planner's Large Vehicle MagCal sends `MAV_CMD_FIXED_MAG_CAL_YAW` with param1 defined in the MAVLink specification as:

> param1: Yaw of vehicle in earth frame (degrees)

"Earth frame" means **True North** reference (geographic north, not magnetic north).

The current handler at `crates/firmware/src/communication/mavlink/handlers/command.rs:499-564` processes this value as follows:

```rust
let true_yaw_degrees = cmd.param1;
let true_yaw_rad = true_yaw_degrees.to_radians();
let yaw_offset = true_yaw_rad - current_ahrs_yaw;
let yaw_offset = Self::normalize_angle(yaw_offset);
state.compass_yaw_offset = yaw_offset;
```

### Doc Comment Error

The handler's doc comment incorrectly describes param1:

```
/// - param1: Yaw angle in degrees (0-360, magnetic north = 0)  <-- WRONG
```

Per the MAVLink specification and Mission Planner behavior, param1 is **True North** referenced. The user enters the vehicle's heading relative to True North (geographic north).

### Why the Logic Is Correct

The DCM AHRS (`crates/core/src/ahrs/dcm.rs:167-183`) uses magnetometer correction in `update_with_mag()`:

```rust
let mag_heading = atan2f(mag_earth.y, mag_earth.x);
let heading_error = normalize_angle_rad(mag_heading - yaw);
self.state.omega_p.z += heading_error * self.config.kp_yaw;
```

This means `current_ahrs_yaw` converges to **Magnetic North** reference. Therefore:

```
yaw_offset = true_north_heading - magnetic_north_heading
           = magnetic_declination + residual_compass_error
```

When applied to future AHRS output:

```
corrected_heading = ahrs_yaw(magnetic) + yaw_offset
                  = magnetic_heading + declination + residual_error
                  = true_north_heading
```

This correctly converts Magnetic North-referenced AHRS output to True North-referenced heading.

### Where the Offset Is Applied

The `compass_yaw_offset` is applied in three locations:

| Location                    | File                    | Line    | Usage                                               |
| --------------------------- | ----------------------- | ------- | --------------------------------------------------- |
| ATTITUDE message            | `handlers/telemetry.rs` | 291     | `corrected_yaw = attitude.yaw + compass_yaw_offset` |
| GLOBAL_POSITION_INT heading | `handlers/telemetry.rs` | 354     | `corrected_yaw = attitude.yaw + compass_yaw_offset` |
| Navigation heading source   | `navigation/heading.rs` | 121-122 | `corrected_yaw_rad = yaw_rad + offset_rad`          |

## Comparison with ArduPilot

### ArduPilot Approach

When ArduPilot receives `MAV_CMD_FIXED_MAG_CAL_YAW`:

1. Looks up magnetic declination from WMM (World Magnetic Model) tables using GPS position
2. Computes expected magnetic heading: `expected_mag = true_heading - declination`
3. Compares expected magnetic field vector with actual magnetometer reading
4. Calculates magnetometer **hard iron offsets** (stored in `COMPASS_OFS_X/Y/Z`)
5. Persists offsets to EEPROM parameters

### pico_trail Approach

1. Reads current AHRS yaw (Magnetic North referenced)
2. Computes `yaw_offset = true_heading - ahrs_yaw`
3. Stores offset in `SystemState.compass_yaw_offset` (RAM only)
4. Applies offset to all heading outputs

### Functional Comparison

| Aspect               | ArduPilot                      | pico_trail                        |
| -------------------- | ------------------------------ | --------------------------------- |
| What is calibrated   | Magnetometer hard iron offsets | AHRS yaw output offset            |
| Declination handling | Explicit via WMM lookup        | Implicitly included in yaw offset |
| Persistence          | Stored in EEPROM parameters    | Lost on reboot                    |
| Accuracy             | Corrects raw magnetometer data | Corrects heading output only      |
| Soft iron correction | Partial (via offsets)          | None                              |
| Location dependency  | Uses GPS position for WMM      | No location-specific model        |

## Identified Issues

### Issue 1: Incorrect Doc Comment (Low Impact)

**File**: `crates/firmware/src/communication/mavlink/handlers/command.rs:505`

```
Current:  /// - param1: Yaw angle in degrees (0-360, magnetic north = 0)
Correct:  /// - param1: Yaw angle in degrees (0-360, true north = 0)
```

This is a documentation-only issue. The code correctly treats param1 as True North.

### Issue 2: Volatile Offset (Medium Impact)

`compass_yaw_offset` is stored in `SystemState` (RAM) and initialized to `0.0` on every boot. This means:

- Calibration must be repeated after every power cycle
- No way to persist the calibration result across reboots

ArduPilot stores this as persistent `COMPASS_OFS_*` parameters.

### Issue 3: No Explicit Declination Parameter (Low Impact)

ArduPilot provides `COMPASS_DEC` for manual declination entry. pico_trail has no equivalent. The yaw offset approach implicitly includes declination, which is acceptable for single-location operation but becomes invalid if the vehicle moves to a significantly different geographic location (where declination differs).

For a rover operating in a limited area, this is not a practical concern.

## Stakeholder Analysis

| Stakeholder       | Interest/Need                                         | Impact | Priority |
| ----------------- | ----------------------------------------------------- | ------ | -------- |
| Rover Operator    | Correct heading after calibration                     | High   | P0       |
| Navigation System | True North referenced heading for waypoint navigation | High   | P0       |
| Field Operations  | Calibration persists across reboots                   | Medium | P1       |

## Risk Assessment

| Risk                                             | Probability | Impact | Mitigation Strategy                                         |
| ------------------------------------------------ | ----------- | ------ | ----------------------------------------------------------- |
| Doc comment causes confusion during maintenance  | Medium      | Low    | Fix the comment (trivial change)                            |
| Calibration lost on reboot in field              | High        | Medium | Persist `compass_yaw_offset` as parameter                   |
| Offset invalid after geographic relocation       | Low         | Low    | Acceptable for rover operating area; re-calibrate if needed |
| AHRS yaw drift makes offset inaccurate over time | Low         | Medium | Magnetometer correction in DCM maintains yaw stability      |

## Discovered Requirements

### Functional Requirements

- [ ] **FR-TBD-1**: `compass_yaw_offset` shall be persisted across reboots
  - Rationale: Users should not need to re-calibrate after every power cycle
  - Acceptance Criteria: Offset value is saved to a parameter and restored on boot

### Non-Functional Requirements

- None identified

## Recommendations

### Immediate (Low Effort)

1. **Fix doc comment** in `handle_fixed_mag_cal_yaw` to correctly state "true north = 0" instead of "magnetic north = 0"

### Future Enhancement

2. **Persist compass_yaw_offset** as a parameter (e.g., `COMPASS_YAW_OFS` or store via existing `COMPASS_OFS_*` mechanism) so calibration survives reboots
3. **Consider WMM integration** (Phase 2 from AN-fgiit) if operating in areas with significant declination variation

### Out of Scope

- Full WMM table implementation (memory-intensive for embedded)
- Soft iron calibration matrix (would require full 3D rotation calibration)
- Multi-compass support (single compass hardware)

## Open Questions

- [x] Does the calibration correctly handle True North input from Mission Planner?
  - **Answer:** Yes. The offset calculation `true_heading - ahrs_yaw` correctly captures declination and residual error. The code logic is correct despite the misleading doc comment.
- [ ] Should `compass_yaw_offset` be persisted as a new parameter or stored in existing `COMPASS_OFS_*`?
  - Note: `COMPASS_OFS_*` are magnetometer hard iron offsets (different concept). A separate parameter may be clearer.
- [ ] Is the current approach sufficient for the project's operational area?
  - Note: For single-location rover operation, the implicit declination handling is adequate.

## Appendix

### MAVLink Specification Reference

From `common.xml`:

```xml
<entry value="42006" name="MAV_CMD_FIXED_MAG_CAL_YAW">
  <param index="1" label="Yaw" units="deg">Yaw of vehicle in earth frame.</param>
</entry>
```

"Earth frame" = True North reference (geographic north).

### Calibration Data Flow

```text
Mission Planner                                  pico_trail
     │                                                │
     │  User enters True North heading (e.g., 0°)    │
     │  MAV_CMD_FIXED_MAG_CAL_YAW param1=0.0         │
     │ ──────────────────────────────────────────────>│
     │                                                │
     │                                                │ AHRS yaw = -7.2° (mag north ref)
     │                                                │ offset = 0.0 - (-7.2°) = +7.2°
     │                                                │ (≈ magnetic declination at location)
     │                                                │
     │                                                │ Store: compass_yaw_offset = +7.2°
     │                                                │
     │  COMMAND_ACK (ACCEPTED)                        │
     │ <──────────────────────────────────────────────│
     │                                                │
     │  Subsequent ATTITUDE messages:                 │
     │  yaw = ahrs_yaw + 7.2° = True North heading   │
     │ <──────────────────────────────────────────────│
```
