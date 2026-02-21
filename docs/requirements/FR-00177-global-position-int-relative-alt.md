# FR-00177 GLOBAL_POSITION_INT Relative Altitude

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00171-home-auto-set-gps-fix](FR-00171-home-auto-set-gps-fix.md)
  - [FR-00080-gps-mavlink-telemetry](FR-00080-gps-mavlink-telemetry.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00047-home-position-management](../analysis/AN-00047-home-position-management.md)
- Related Tasks:
  - [T-00181-home-position-management](../tasks/T-00181-home-position-management/README.md)

## Requirement Statement

The system shall compute the `relative_alt` field in GLOBAL_POSITION_INT as the altitude relative to the home position (current GPS altitude minus home altitude), expressed in millimeters. If home is not set, `relative_alt` shall be 0 (current fallback behavior).

## Rationale

ArduPilot computes `relative_alt` as current altitude minus home altitude (`GCS_Common.cpp:6105-6113`). Mission Planner uses this field for the altitude display on the HUD. The current pico_trail implementation hardcodes `relative_alt: 0` (`telemetry.rs:308`), which causes Mission Planner to always show 0m altitude. For a ground rover this is less critical than for a copter, but correct altitude reporting improves GCS accuracy and operator situational awareness, especially on hilly terrain.

## User Story

As an operator, I want the GCS to display the correct altitude above my launch point, so that I can assess terrain changes during rover navigation.

## Acceptance Criteria

- [ ] If home is set: `relative_alt = (current_gps_alt - home_alt) * 1000` (meters to millimeters, as int32)
- [ ] If home is not set: `relative_alt = 0` (unchanged fallback)
- [ ] Update `build_global_position_int()` in `TelemetryStreamer` to use computed value
- [ ] Values are consistent with HOME_POSITION altitude format (both in mm)

## Technical Details

### Functional Requirement Details

**Updated telemetry computation:**

```rust
fn build_global_position_int(state: &SystemState) -> GlobalPositionInt {
    let relative_alt = if let Some(home) = &state.home_position {
        if let Some(gps) = &state.gps_position {
            ((gps.altitude - home.altitude) * 1000.0) as i32  // meters to mm
        } else {
            0
        }
    } else {
        0  // No home set, fallback
    };

    GlobalPositionInt {
        // ... existing fields ...
        relative_alt,
        // ...
    }
}
```

**ArduPilot reference:**

- `GCS_MAVLINK::global_position_int_relative_alt()` at `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:6105-6138`
- Uses `get_relative_position_D_home()` which computes position relative to home
- Result is negated (NED to up) and converted to millimeters

**Current state:**

- `crates/core/src/communication/handlers/telemetry.rs:308` hardcodes `relative_alt: 0`
- Fix is a simple computation change in `build_global_position_int()`

## Platform Considerations

N/A - Platform agnostic (core crate logic)

## Risks & Mitigation

| Risk                                            | Impact | Likelihood | Mitigation                                                 | Validation           |
| ----------------------------------------------- | ------ | ---------- | ---------------------------------------------------------- | -------------------- |
| GPS altitude noise causes jitter                | Low    | Medium     | Normal for GPS altitude; filtering is a future enhancement | Test with real GPS   |
| Integer overflow on extreme altitude difference | Low    | Very Low   | +-2147m range in i32 mm is sufficient for rovers           | Range check in tests |

## Implementation Notes

- Single-line change in `build_global_position_int()` in `TelemetryStreamer`
- Requires `SystemState` to include `home_position` (already exists)
- GPS altitude and home altitude must use the same reference (MSL)

## External References

- ArduPilot `global_position_int_relative_alt()`: `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:6105-6138`
- MAVLink GLOBAL_POSITION_INT: <https://mavlink.io/en/messages/common.html#GLOBAL_POSITION_INT>
