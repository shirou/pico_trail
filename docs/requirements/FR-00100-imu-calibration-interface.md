# FR-00100 IMU Calibration Interface

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00101-imu-sensor-trait](FR-00101-imu-sensor-trait.md)
  - [FR-00006-runtime-parameters](FR-00006-runtime-parameters.md)
- Dependent Requirements:
  - [FR-00104-quaternion-ekf-ahrs](FR-00104-quaternion-ekf-ahrs.md)
  - [FR-00102-large-vehicle-magcal](FR-00102-large-vehicle-magcal.md)
- Related ADRs:
  - [ADR-00026-mpu9250-i2c-driver-architecture](../adr/ADR-00026-mpu9250-i2c-driver-architecture.md)
- Related Tasks:
  - [T-00024-icm20948-driver-implementation](../tasks/T-00024-icm20948-driver-implementation/README.md)
  - [T-00023-mpu9250-driver-implementation](../tasks/T-00023-mpu9250-driver-implementation/README.md)

## Requirement Statement

The system shall provide an IMU calibration interface that loads calibration data from the parameter system and applies offset/scale corrections to raw sensor readings, outputting calibrated data in standard units.

## Rationale

Raw IMU sensor data contains systematic errors that must be corrected:

- **Gyroscope Bias**: Gyros report non-zero rate when stationary (drift source)
- **Accelerometer Offset/Scale**: Imperfect alignment, manufacturing tolerances
- **Magnetometer Hard/Soft Iron**: Environmental magnetic distortion

Calibration data stored in parameters allows:

- Persistence across power cycles
- Field adjustment without reflashing firmware
- Consistent handling across sensor types

## User Story (if applicable)

As an operator, I want IMU calibration data to be loaded from parameters and applied automatically, so that the autopilot provides accurate attitude estimates without manual recalibration on each power cycle.

## Acceptance Criteria

- [ ] Calibration data loaded from parameter system on driver initialization
- [ ] Gyroscope bias subtracted from raw readings
- [ ] Accelerometer offset subtracted, then scale applied per-axis
- [ ] Magnetometer hard iron offset subtracted
- [ ] Magnetometer soft iron correction matrix applied
- [ ] Calibration can be updated at runtime via `set_calibration()`
- [ ] Default calibration (identity) works for uncalibrated sensors
- [ ] Calibration data format documented for parameter system

## Technical Details (if applicable)

### Functional Requirement Details

**Calibration Data Structure:**

```rust
pub struct ImuCalibration {
    /// Gyroscope bias: rad/s offset to subtract
    /// Loaded from: INS_GYR*OFF* parameters
    pub gyro_bias: Vector3<f32>,

    /// Accelerometer offset: m/s² to subtract
    /// Loaded from: INS_ACC*OFFS* parameters
    pub accel_offset: Vector3<f32>,

    /// Accelerometer scale: per-axis scale factors
    /// Loaded from: INS_ACC*SCAL* parameters
    pub accel_scale: Vector3<f32>,

    /// Magnetometer hard iron offset: µT to subtract
    /// Loaded from: COMPASS_OFS* parameters
    pub mag_offset: Vector3<f32>,

    /// Magnetometer soft iron matrix: 3x3 correction matrix
    /// Loaded from: COMPASS_DIA*, COMPASS_ODI* parameters
    pub mag_scale: Matrix3<f32>,
}

impl Default for ImuCalibration {
    fn default() -> Self {
        Self {
            gyro_bias: Vector3::zeros(),
            accel_offset: Vector3::zeros(),
            accel_scale: Vector3::new(1.0, 1.0, 1.0),
            mag_offset: Vector3::zeros(),
            mag_scale: Matrix3::identity(),
        }
    }
}
```

**Calibration Application:**

```rust
impl Mpu9250Driver {
    fn apply_calibration(&self, raw: RawData) -> ImuReading {
        // Gyro: subtract bias
        let gyro = raw.gyro - self.calibration.gyro_bias;

        // Accel: subtract offset, apply scale
        let accel = (raw.accel - self.calibration.accel_offset)
            .component_mul(&self.calibration.accel_scale);

        // Mag: subtract hard iron, apply soft iron matrix
        let mag = self.calibration.mag_scale
            * (raw.mag - self.calibration.mag_offset);

        ImuReading {
            gyro,
            accel,
            mag,
            temperature: raw.temperature,
            timestamp_us: raw.timestamp_us,
        }
    }
}
```

**ArduPilot Parameter Mapping:**

| Calibration Field  | ArduPilot Parameters | Notes                  |
| ------------------ | -------------------- | ---------------------- |
| gyro_bias.x        | INS_GYR1OFFS_X       | Primary gyro X bias    |
| gyro_bias.y        | INS_GYR1OFFS_Y       | Primary gyro Y bias    |
| gyro_bias.z        | INS_GYR1OFFS_Z       | Primary gyro Z bias    |
| accel_offset.x     | INS_ACC1OFFS_X       | Primary accel X offset |
| accel_offset.y     | INS_ACC1OFFS_Y       | Primary accel Y offset |
| accel_offset.z     | INS_ACC1OFFS_Z       | Primary accel Z offset |
| accel_scale.x      | INS_ACC1SCAL_X       | Primary accel X scale  |
| accel_scale.y      | INS_ACC1SCAL_Y       | Primary accel Y scale  |
| accel_scale.z      | INS_ACC1SCAL_Z       | Primary accel Z scale  |
| mag_offset.x       | COMPASS_OFS_X        | Compass X hard iron    |
| mag_offset.y       | COMPASS_OFS_Y        | Compass Y hard iron    |
| mag_offset.z       | COMPASS_OFS_Z        | Compass Z hard iron    |
| mag_scale diagonal | COMPASS_DIA_X/Y/Z    | Compass diagonal scale |
| mag_scale off-diag | COMPASS_ODI_X/Y/Z    | Compass off-diagonal   |

## Platform Considerations

### Cross-Platform

- Calibration data format identical on all platforms
- Parameter storage uses existing parameter system
- Calibration application uses platform-independent math

## Risks & Mitigation

| Risk                                  | Impact | Likelihood | Mitigation                                                      | Validation                           |
| ------------------------------------- | ------ | ---------- | --------------------------------------------------------------- | ------------------------------------ |
| Invalid calibration degrades accuracy | High   | Medium     | Validate calibration sanity (scale near 1.0, offset reasonable) | Log warnings for suspect values      |
| Calibration not loaded on init        | High   | Low        | Log error if parameter load fails, use defaults                 | Verify calibration in startup log    |
| Temperature drift not compensated     | Medium | Medium     | Implement temperature calibration (future)                      | Monitor drift over temperature range |

## Implementation Notes

**Loading Calibration:**

```rust
pub fn load_calibration_from_params(params: &ParameterStore) -> ImuCalibration {
    ImuCalibration {
        gyro_bias: Vector3::new(
            params.get_f32("INS_GYR1OFFS_X").unwrap_or(0.0),
            params.get_f32("INS_GYR1OFFS_Y").unwrap_or(0.0),
            params.get_f32("INS_GYR1OFFS_Z").unwrap_or(0.0),
        ),
        accel_offset: Vector3::new(
            params.get_f32("INS_ACC1OFFS_X").unwrap_or(0.0),
            params.get_f32("INS_ACC1OFFS_Y").unwrap_or(0.0),
            params.get_f32("INS_ACC1OFFS_Z").unwrap_or(0.0),
        ),
        accel_scale: Vector3::new(
            params.get_f32("INS_ACC1SCAL_X").unwrap_or(1.0),
            params.get_f32("INS_ACC1SCAL_Y").unwrap_or(1.0),
            params.get_f32("INS_ACC1SCAL_Z").unwrap_or(1.0),
        ),
        // ... mag calibration
    }
}
```

**Calibration Validation:**

```rust
fn validate_calibration(cal: &ImuCalibration) -> bool {
    // Accel scale should be near 1.0
    let scale_ok = cal.accel_scale.iter()
        .all(|s| (0.9..1.1).contains(s));

    // Gyro bias should be small (< 0.1 rad/s)
    let bias_ok = cal.gyro_bias.norm() < 0.1;

    scale_ok && bias_ok
}
```

## External References

- [ArduPilot INS Parameters](https://ardupilot.org/rover/docs/parameters.html#ins-parameters)
- [ArduPilot Compass Calibration](https://ardupilot.org/rover/docs/common-compass-calibration-in-mission-planner.html)
