# FR-e2urj Large Vehicle MagCal

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-t47be-mpu9250-imu-and-ekf-integration](../analysis/AN-t47be-mpu9250-imu-and-ekf-integration.md)
- Prerequisite Requirements:
  - [FR-slm3x-icm20948-i2c-driver](FR-slm3x-icm20948-i2c-driver.md)
  - [FR-oqxl8-mpu9250-i2c-driver](FR-oqxl8-mpu9250-i2c-driver.md)
  - [FR-soukr-imu-calibration-interface](FR-soukr-imu-calibration-interface.md)
  - [FR-3ik7l-gps-operation-data-management](FR-3ik7l-gps-operation-data-management.md)
  - [FR-gpzpz-mavlink-protocol](FR-gpzpz-mavlink-protocol.md)
- Dependent Requirements: None
- Related ADRs:
  - [ADR-t5cq4-mpu9250-i2c-driver-architecture](../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)

## Requirement Statement

The system shall support Large Vehicle MagCal via MAVLink, allowing magnetometer calibration without rotating the vehicle by using GPS position, World Magnetic Model (WMM), and a user-provided heading.

## Rationale

Large or heavy vehicles (rovers, boats) are impractical to rotate on all axes for traditional magnetometer calibration:

- **Physical Constraints**: Rovers and boats cannot be easily lifted and rotated
- **Mission Planner Integration**: Standard ArduPilot GCS workflow for field calibration
- **Accuracy**: WMM-based calibration provides accurate hard iron offsets when GPS is available
- **Convenience**: Single-position calibration vs multi-axis rotation

This feature enables field calibration using Mission Planner's "Large Vehicle MagCal" option.

## User Story (if applicable)

As an operator, I want to calibrate the magnetometer using Mission Planner's Large Vehicle MagCal, so that I can accurately calibrate compass offsets without physically rotating the vehicle.

## Acceptance Criteria

- [ ] Handle `MAV_CMD_FIXED_MAG_CAL_YAW` (42006) MAVLink command
- [ ] Require valid 3D GPS fix before accepting calibration command
- [ ] Use WMM tables to compute expected magnetic field at current GPS position
- [ ] Accept TRUE heading (not magnetic) from user input
- [ ] Calculate and store hard iron offsets (`COMPASS_OFS_X/Y/Z`)
- [ ] Return `COMMAND_ACK` with success/failure status
- [ ] Reject calibration if GPS fix unavailable with appropriate error
- [ ] Support compass mask parameter for multi-compass systems (future)
- [ ] Persist calibration to parameter storage

## Technical Details (if applicable)

### Functional Requirement Details

**MAVLink Command:**

```
MAV_CMD_FIXED_MAG_CAL_YAW (42006)
├── Param1: Yaw (deg) - Vehicle TRUE heading (0-360)
├── Param2: CompassMask - Target compasses (0 = all)
├── Param3: Latitude (deg) - Current GPS latitude (optional, use current if 0)
├── Param4: Longitude (deg) - Current GPS longitude (optional, use current if 0)
└── Param5-7: Empty
```

**Alternative Command (if WMM not available):**

```
MAV_CMD_FIXED_MAG_CAL (42004)
├── Param1: Declination (deg) - Magnetic declination
├── Param2: Inclination (deg) - Magnetic inclination
├── Param3: Intensity (mGauss) - Field intensity
├── Param4: Yaw (deg) - Vehicle TRUE heading
└── Param5-7: Empty
```

**Calibration Algorithm:**

```rust
pub fn fixed_yaw_mag_cal(
    measured_mag: Vector3<f32>,  // Current mag reading (µT)
    yaw_deg: f32,                // User-provided TRUE heading
    latitude: f64,               // GPS latitude
    longitude: f64,              // GPS longitude
) -> Result<Vector3<f32>, CalError> {
    // 1. Get expected field from WMM at this location
    let wmm_field = wmm::get_field(latitude, longitude)?;
    // wmm_field: (intensity, declination, inclination)

    // 2. Convert WMM field to NED frame
    let expected_ned = wmm_to_ned(
        wmm_field.intensity,
        wmm_field.inclination,
    );

    // 3. Rotate expected field to body frame using yaw
    let yaw_rad = yaw_deg.to_radians();
    let expected_body = rotate_ned_to_body(expected_ned, yaw_rad);

    // 4. Calculate offset (measured - expected)
    let offset = measured_mag - expected_body;

    Ok(offset)  // Hard iron offset to store in COMPASS_OFS_*
}
```

**WMM Integration:**

World Magnetic Model provides magnetic field parameters for any location:

- **Declination**: Angle between magnetic north and true north
- **Inclination**: Angle of field dip below horizontal
- **Intensity**: Total field strength

Options for WMM:

1. **Embedded WMM Table** (\~15KB ROM): Full WMM2020 coefficients
2. **Simplified Grid** (\~2KB ROM): Pre-computed grid with interpolation
3. **External Command**: Use `MAV_CMD_FIXED_MAG_CAL` with GCS-provided values

**Parameters Set:**

| Parameter     | Description        | Unit   |
| ------------- | ------------------ | ------ |
| COMPASS_OFS_X | Hard iron offset X | mGauss |
| COMPASS_OFS_Y | Hard iron offset Y | mGauss |
| COMPASS_OFS_Z | Hard iron offset Z | mGauss |

**Mission Planner Workflow:**

1. User opens Setup → Mandatory Hardware → Compass
2. User selects "Large Vehicle MagCal"
3. User enters vehicle's TRUE heading (from map landmark or phone + declination)
4. Mission Planner sends `MAV_CMD_FIXED_MAG_CAL_YAW`
5. Autopilot calculates offsets and stores to parameters
6. Mission Planner receives `COMMAND_ACK` with result

## Platform Considerations

### Cross-Platform

- WMM tables stored in ROM (flash)
- Same algorithm on both RP2040 and RP2350
- Parameter storage via existing parameter system

### Memory Considerations

| Component               | Size   | Notes                       |
| ----------------------- | ------ | --------------------------- |
| WMM coefficients (full) | \~15KB | WMM2020 spherical harmonics |
| WMM grid (simplified)   | \~2KB  | Pre-computed 10° grid       |
| Runtime calculation     | \~200B | Stack for computation       |

## Risks & Mitigation

| Risk                                         | Impact | Likelihood | Mitigation                     | Validation                   |
| -------------------------------------------- | ------ | ---------- | ------------------------------ | ---------------------------- |
| Poor GPS position degrades calibration       | High   | Low        | Require 3D fix with HDOP < 2.0 | Check GPS quality before cal |
| User enters magnetic heading instead of true | High   | Medium     | Document clearly, warn in GCS  | Add heading sanity check     |
| WMM tables outdated (valid until 2025)       | Medium | Low        | Update with firmware           | Check WMM epoch              |
| Soft iron not corrected                      | Medium | Medium     | Document limitation            | Note: hard iron only         |

## Implementation Notes

**MAVLink Handler:**

```rust
fn handle_fixed_mag_cal_yaw(
    &mut self,
    yaw: f32,
    compass_mask: u8,
    lat: f64,
    lon: f64,
) -> MavResult {
    // Check GPS fix
    let gps = GPS_STATE.lock().await;
    if gps.fix_type < GpsFixType::Fix3D {
        return MavResult::Denied; // No GPS fix
    }

    // Use provided lat/lon or current GPS
    let (lat, lon) = if lat == 0.0 && lon == 0.0 {
        (gps.latitude, gps.longitude)
    } else {
        (lat, lon)
    };

    // Read current magnetometer
    let mag = self.imu.read_mag().await?;

    // Calculate offset using WMM
    let offset = fixed_yaw_mag_cal(mag, yaw, lat, lon)?;

    // Store to parameters
    params.set_f32("COMPASS_OFS_X", offset.x)?;
    params.set_f32("COMPASS_OFS_Y", offset.y)?;
    params.set_f32("COMPASS_OFS_Z", offset.z)?;
    params.save()?;

    // Update driver calibration
    self.imu.set_mag_offset(offset);

    MavResult::Accepted
}
```

**WMM Library Options:**

- `wmm` crate (if available for no_std)
- Custom implementation with coefficient table
- Pre-computed lookup table with bilinear interpolation

**Heading Validation:**

```rust
fn validate_heading(yaw_deg: f32, gps_course: f32) -> bool {
    // If vehicle is moving, compare with GPS course
    // Allow ±30° difference as sanity check
    if gps_speed > 1.0 {
        let diff = (yaw_deg - gps_course).abs();
        if diff > 30.0 && diff < 330.0 {
            log_warn!("Heading differs from GPS course by {}°", diff);
        }
    }
    true
}
```

## External References

- [ArduPilot Compass Calibration](https://ardupilot.org/copter/docs/common-compass-calibration-in-mission-planner.html)
- [ArduPilot Advanced Compass Setup](https://ardupilot.org/copter/docs/common-compass-setup-advanced.html)
- [MAVLink ArduPilotMega Dialect](https://mavlink.io/en/messages/ardupilotmega.html)
- [ArduPilot PR #12863 - Fixed yaw mag cal](https://github.com/ArduPilot/ardupilot/pull/12863)
- [World Magnetic Model (NOAA)](https://www.ngdc.noaa.gov/geomag/WMM/)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
