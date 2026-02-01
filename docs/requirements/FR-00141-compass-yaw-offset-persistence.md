# FR-00141 Compass Yaw Offset Persistence

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-00043-compass-calibration-true-north-handling](../analysis/AN-00043-compass-calibration-true-north-handling.md)
  - [AN-00038-compass-calibration-via-mission-planner](../analysis/AN-00038-compass-calibration-via-mission-planner.md)
- Prerequisite Requirements:
  - [FR-00128-fixed-mag-cal-yaw-handler](FR-00128-fixed-mag-cal-yaw-handler.md)
  - [FR-00022-configuration-persistence](FR-00022-configuration-persistence.md)
  - [FR-00006-runtime-parameters](FR-00006-runtime-parameters.md)
- Dependent Requirements: None
- Related ADRs:
  - [ADR-00035-compass-yaw-offset-calibration](../adr/ADR-00035-compass-yaw-offset-calibration.md)
- Related Tasks:
  - [T-00034-compass-calibration](../tasks/T-00034-compass-calibration/README.md)
  - [T-00040-compass-yaw-offset-persistence](../tasks/T-00040-compass-yaw-offset-persistence/README.md)

## Requirement Statement

The system shall persist the `compass_yaw_offset` value across reboots so that compass calibration performed via `MAV_CMD_FIXED_MAG_CAL_YAW` survives power cycles without requiring the operator to recalibrate.

## Rationale

The current implementation (FR-00128) stores `compass_yaw_offset` in RAM only (`SystemState`), initialized to `0.0` on every boot. This means:

- **Calibration is lost on every power cycle**, forcing operators to recalibrate before each mission
- **Field operations are impacted** when the rover is powered off between missions
- **ArduPilot parity**: ArduPilot persists calibration results (as `COMPASS_OFS_*` hard iron offsets) to EEPROM, so operators expect calibration to survive reboots

The yaw offset captures both magnetic declination and residual compass error in a single value. Persisting it eliminates the most impactful usability gap identified in AN-00043.

## User Story (if applicable)

As a rover operator, I want compass calibration to persist across reboots, so that I do not need to recalibrate heading every time the rover is powered on.

## Acceptance Criteria

- [x] `compass_yaw_offset` is saved to non-volatile storage after successful `MAV_CMD_FIXED_MAG_CAL_YAW` calibration
- [x] The stored offset is restored on boot before the navigation system begins using heading data
- [x] After power cycle, ATTITUDE and GLOBAL_POSITION_INT messages reflect the previously calibrated offset
- [x] Navigation heading source uses the restored offset without requiring recalibration
- [x] If no calibration has been performed (first boot or parameter reset), the offset defaults to `0.0`
- [x] The offset can be inspected and modified via MAVLink `PARAM_SET` / `PARAM_VALUE` protocol
- [ ] Saving the offset does not block the scheduler for more than 10ms

## Technical Details (if applicable)

### Functional Requirement Details

**Current Offset Usage (3 locations per AN-00043):**

| Location                    | File                    | Usage                                               |
| --------------------------- | ----------------------- | --------------------------------------------------- |
| ATTITUDE message            | `handlers/telemetry.rs` | `corrected_yaw = attitude.yaw + compass_yaw_offset` |
| GLOBAL_POSITION_INT heading | `handlers/telemetry.rs` | `corrected_yaw = attitude.yaw + compass_yaw_offset` |
| Navigation heading source   | `navigation/heading.rs` | `corrected_yaw_rad = yaw_rad + offset_rad`          |

**Boot Sequence Integration:**

```text
Boot
 ├── Load parameters from flash (including compass yaw offset)
 ├── Initialize SystemState with restored compass_yaw_offset
 ├── Start AHRS / navigation
 └── Heading output uses restored offset immediately
```

**Calibration Save Flow:**

```text
MAV_CMD_FIXED_MAG_CAL_YAW received
 ├── Calculate: offset = true_heading - ahrs_yaw
 ├── Store in SystemState.compass_yaw_offset (RAM)
 ├── Save to ParameterStore (flash)
 └── Send COMMAND_ACK (ACCEPTED)
```

### Parameter Storage Approach

Per [ADR-00035](../adr/ADR-00035-compass-yaw-offset-calibration.md), the offset is stored using the standard ArduPilot parameter `COMPASS_DEC`. While ArduPilot uses this parameter for pure magnetic declination, pico_trail's yaw offset captures declination plus residual compass error. This semantic broadening is documented but does not affect operator workflow. The offset is stored in radians, consistent with ArduPilot's `COMPASS_DEC` unit.

## Platform Considerations

### Cross-Platform

- Flash storage applies only to embedded targets (RP2350)
- Host tests shall use in-memory parameter storage (existing pattern per NFR-00062)
- The offset is a single `f32` value - minimal storage overhead

## Risks & Mitigation

| Risk                                        | Impact | Likelihood | Mitigation                                                 | Validation                        |
| ------------------------------------------- | ------ | ---------- | ---------------------------------------------------------- | --------------------------------- |
| Stale offset after geographic relocation    | Low    | Low        | Acceptable for rover operating area; recalibrate if needed | Document in operator manual       |
| Flash write failure during save             | Medium | Low        | Fallback to `0.0` default; warn operator via STATUSTEXT    | Test with simulated flash failure |
| Offset restored before AHRS is ready        | Medium | Low        | Load offset during parameter init, apply after AHRS boot   | Verify boot sequence ordering     |
| COMPASS_DEC semantics differ from ArduPilot | Low    | Low        | Documented in ADR-00035; acceptable for project scope      | Review operator documentation     |

## Implementation Notes

- Integrate with existing `ParameterStore` infrastructure (per FR-00022)
- The `handle_fixed_mag_cal_yaw` handler (FR-00128) already computes the offset; add a save step after computing
- On boot, `SystemState::compass_yaw_offset` should be initialized from stored parameter instead of hardcoded `0.0`
- The doc comment in the handler should also be corrected as part of implementation ("magnetic north = 0" → "true north = 0", per AN-00043 Issue 1)

## External References

- [MAV_CMD_FIXED_MAG_CAL_YAW](https://mavlink.io/en/messages/common.html#MAV_CMD_FIXED_MAG_CAL_YAW)
- [ArduPilot COMPASS_DEC Parameter](https://ardupilot.org/rover/docs/parameters.html#compass-dec)
- [Large Vehicle MagCal Documentation](https://ardupilot.org/rover/docs/common-compass-calibration-in-mission-planner.html#large-vehicle-magcal)
