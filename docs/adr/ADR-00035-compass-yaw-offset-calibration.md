# ADR-00035 Compass Yaw Offset Calibration Approach

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-00141-compass-yaw-offset-persistence](../requirements/FR-00141-compass-yaw-offset-persistence.md)
  - [FR-00128-fixed-mag-cal-yaw-handler](../requirements/FR-00128-fixed-mag-cal-yaw-handler.md)
  - [FR-00102-large-vehicle-magcal](../requirements/FR-00102-large-vehicle-magcal.md)
- Related ADRs:
  - [ADR-00033-heading-source-integration](ADR-00033-heading-source-integration.md)
  - [ADR-00004-storage-strategy](ADR-00004-storage-strategy.md)
- Related Analysis:
  - [AN-00043-compass-calibration-true-north-handling](../analysis/AN-00043-compass-calibration-true-north-handling.md)
  - [AN-00038-compass-calibration-via-mission-planner](../analysis/AN-00038-compass-calibration-via-mission-planner.md)
- Related Tasks:
  - [T-00034-compass-calibration](../tasks/T-00034-compass-calibration/README.md)
  - [T-00040-compass-yaw-offset-persistence](../tasks/T-00040-compass-yaw-offset-persistence/README.md)

## Context

Mission Planner's Large Vehicle MagCal sends `MAV_CMD_FIXED_MAG_CAL_YAW` (42006) with the vehicle's True North heading. The autopilot must use this to correct compass-based heading output.

AN-00043 confirmed that the current pico_trail implementation correctly calculates a yaw offset (`true_heading - ahrs_yaw`) that captures both magnetic declination and residual compass error. However, this offset is stored in RAM only and lost on reboot (FR-00141).

Two fundamental approaches exist for compass calibration:

1. **Hard iron offset correction** (ArduPilot approach) — Correct raw magnetometer data using WMM-derived expected field vectors, producing `COMPASS_OFS_X/Y/Z` offsets
2. **Yaw offset correction** (current pico_trail approach) — Correct AHRS heading output by adding a scalar offset

The choice affects accuracy, memory usage, and implementation complexity.

### Mission Planner Protocol Constraint

Mission Planner's "Large Vehicle MagCal" button sends `MAV_CMD_FIXED_MAG_CAL_YAW` (42006) with **only the yaw angle** as param1. It does **not** send WMM field parameters. The alternative command `MAV_CMD_FIXED_MAG_CAL` (42004), which includes declination, inclination, and intensity from the GCS, is not used by Mission Planner's UI.

Source: [Mission Planner ConfigHWCompass.cs](https://github.com/ArduPilot/MissionPlanner/blob/master/GCSViews/ConfigurationView/ConfigHWCompass.cs)

This means implementing hard iron offset correction requires embedding WMM tables in the firmware.

## Decision

We will **retain the yaw offset approach** and persist the offset as a custom parameter.

### Decision Drivers

- Mission Planner sends only yaw angle (42006), requiring firmware-side WMM for hard iron offsets
- WMM tables cost 2–15 KB of flash, significant on RP2350
- The rover operates in a limited geographic area where the yaw offset approach is adequate
- Current implementation is proven correct per AN-00043
- Simplicity and minimal resource usage are priorities for this embedded platform

### Considered Options

- Option A: Yaw offset with persistence (current approach + parameter storage)
- Option B: Hard iron offsets with embedded WMM tables (ArduPilot approach)
- Option C: Hard iron offsets via `MAV_CMD_FIXED_MAG_CAL` (42004) from GCS

### Option Analysis

**Option A — Yaw offset with persistence**

- Pros: Simple (1 × f32), proven correct, no WMM dependency, minimal flash usage
- Cons: Cannot correct heading-dependent (soft iron) errors; bundles declination and compass error into one value; invalid after large geographic relocation

**Option B — Hard iron offsets with embedded WMM**

- Pros: Corrects raw magnetometer data, improves overall AHRS accuracy, handles soft iron partially, separates declination from compass error
- Cons: Requires 2–15 KB WMM tables in flash, complex implementation (\~200+ lines), WMM epoch expires (currently WMM2020, valid until 2025)

**Option C — Hard iron offsets via MAV_CMD_FIXED_MAG_CAL (42004)**

- Pros: No WMM tables needed in firmware (GCS provides field parameters), full hard iron correction
- Cons: **Mission Planner does not use this command** — the "Large Vehicle MagCal" UI sends 42006 only; would require custom GCS tooling or MAVProxy workflow

**Selected: Option A**

### Parameter Storage

The yaw offset requires a parameter name for MAVLink `PARAM_SET`/`PARAM_VALUE` protocol access. ArduPilot has no direct equivalent (ArduPilot uses `COMPASS_OFS_X/Y/Z` for hard iron offsets, which is a different concept).

**Chosen parameter name: `COMPASS_DEC`**

Rationale:

- `COMPASS_DEC` is a standard ArduPilot parameter for magnetic declination
- The pico_trail yaw offset is semantically close: it captures declination plus residual compass error
- Using a standard parameter name avoids creating a custom parameter exception
- Operators familiar with ArduPilot will understand the purpose
- The semantic difference (pure declination vs. declination + residual error) is documented but does not affect the operator workflow

The offset is stored in radians, consistent with ArduPilot's `COMPASS_DEC` unit.

## Rationale

Option A was selected because:

1. **Protocol constraint**: Mission Planner only sends 42006, making Option C impractical without custom tooling
2. **Resource constraint**: WMM tables (Option B) consume 2–15 KB of flash on a memory-constrained platform
3. **Proven correctness**: AN-00043 verified the yaw offset calculation is mathematically sound
4. **Operational scope**: The rover operates in a limited area where declination variation is negligible
5. **Incremental path**: If soft iron errors are observed in field testing, Option B can be implemented later (FR-00102) without changing the MAVLink interface

The `COMPASS_DEC` parameter name was chosen to stay within ArduPilot conventions while acknowledging the slight semantic broadening in documentation.

## Consequences

### Positive

- Calibration persists across reboots (FR-00141 satisfied)
- No additional flash usage beyond one f32 parameter
- Implementation requires minimal code changes (save/restore in existing ParameterStore)
- Compatible with Mission Planner's standard Large Vehicle MagCal workflow
- Standard ArduPilot parameter name recognizable by operators

### Negative

- Soft iron errors (heading-dependent) cannot be corrected with a scalar offset
- Offset becomes invalid if the rover relocates to a region with significantly different magnetic declination
- `COMPASS_DEC` semantics differ slightly from ArduPilot (declination + residual error vs. pure declination)
- AHRS magnetometer correction loop continues to work with uncorrected data

### Neutral

- FR-00102 (full WMM-based calibration) remains available as a future enhancement if field testing reveals inadequacy
- Doc comment fix in `handle_fixed_mag_cal_yaw` ("magnetic north" → "true north") should be applied during implementation

## Implementation Notes

**Parameter Integration:**

```rust
// Save after calibration
let yaw_offset = true_yaw_rad - current_ahrs_yaw;
let yaw_offset = Self::normalize_angle(yaw_offset);
state.compass_yaw_offset = yaw_offset;
params.set_f32("COMPASS_DEC", yaw_offset)?;
params.save()?;
```

**Boot Restore:**

```rust
// During parameter initialization
let compass_dec = params.get_f32("COMPASS_DEC").unwrap_or(0.0);
state.compass_yaw_offset = compass_dec;
```

**Offset Application Points (unchanged):**

| Location                    | File                    | Usage                                               |
| --------------------------- | ----------------------- | --------------------------------------------------- |
| ATTITUDE message            | `handlers/telemetry.rs` | `corrected_yaw = attitude.yaw + compass_yaw_offset` |
| GLOBAL_POSITION_INT heading | `handlers/telemetry.rs` | `corrected_yaw = attitude.yaw + compass_yaw_offset` |
| Navigation heading source   | `navigation/heading.rs` | `corrected_yaw_rad = yaw_rad + offset_rad`          |
