# ADR-00032 AHRS Abstraction Architecture: Software EKF and External AHRS Support

## Metadata

- Type: ADR
- Status: Draft

## Links

- Related Analyses:
  - [AN-00028-bno086-imu-integration](../analysis/AN-00028-bno086-imu-integration.md)
  - [AN-00030-imu-driver-hal-abstraction](../analysis/AN-00030-imu-driver-hal-abstraction.md)
- Impacted Requirements:
  - [FR-00001-ahrs-attitude-estimation](../requirements/FR-00001-ahrs-attitude-estimation.md)
  - [FR-00101-imu-sensor-trait](../requirements/FR-00101-imu-sensor-trait.md)
  - [FR-00107-imu-hal-abstraction](../requirements/FR-00107-imu-hal-abstraction.md)
  - [NFR-00002-imu-sampling-rate](../requirements/NFR-00002-imu-sampling-rate.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-00031-bno086-driver-implementation](../tasks/T-00031-bno086-driver-implementation/README.md)
  - [T-00032-ahrs-abstraction-layer](../tasks/T-00032-ahrs-abstraction-layer/README.md)
  - [T-00033-heading-source-navigation-integration](../tasks/T-00033-heading-source-navigation-integration/README.md)

## Context

### Problem

pico_trail needs to support multiple IMU sensor types with different capabilities:

1. **Raw IMU Sensors** (ICM-42688, BMI270, MPU6500): Output raw accelerometer and gyroscope data. Require software sensor fusion (EKF/DCM) to compute attitude.

2. **Fusion IMU Sensors** (BNO086, BNO085, BNO080): Have on-chip ARM Cortex-M0+ running sensor fusion. Output quaternion attitude directly.

Currently, the codebase has:

- `QuaternionSensor` trait for BNO086 quaternion output
- No unified interface for flight control to consume attitude data regardless of source

### Constraints

- Flight control code should not need to know whether attitude comes from software EKF or external sensor
- BNO086's on-chip fusion should be usable without running redundant software EKF
- Future raw IMU support (ICM-42688) should integrate with software EKF
- Memory budget: < 10 KB RAM for AHRS state
- CPU budget: AHRS update within 10ms at 100Hz

### Forces in Tension

| Force         | Software EKF                    | External AHRS            |
| ------------- | ------------------------------- | ------------------------ |
| CPU load      | High (matrix operations)        | Low (sensor does fusion) |
| Flexibility   | High (tunable, GPS integration) | Low (fixed algorithm)    |
| Sensor choice | Wide (any raw IMU)              | Limited (BNO08x family)  |
| Complexity    | High                            | Low                      |

### Prior Art

- **ArduPilot**: Supports both internal EKF3 and External AHRS (VectorNav, MicroStrain). Uses `AP_AHRS` base class with `AP_AHRS_DCM`, `AP_AHRS_NavEKF`, `AP_ExternalAHRS` implementations.
- **PX4**: Primarily uses internal EKF2, limited external AHRS support.

## Decision

**We will implement a unified AHRS abstraction layer that supports both Software EKF (for raw IMUs) and External AHRS (for fusion sensors like BNO086).**

### Architecture Overview

```
                           ┌──────────────────┐
                           │  Flight Control  │
                           │  (AttitudeController) │
                           └────────▲─────────┘
                                    │
                           ┌────────┴─────────┐
                           │    Ahrs Trait    │
                           │ (Common Interface)  │
                           └────────▲─────────┘
                                    │
          ┌─────────────────────────┼─────────────────────────┐
          │                         │                         │
 ┌────────┴────────┐       ┌────────┴────────┐       ┌────────┴────────┐
 │  SoftwareAhrs   │       │  ExternalAhrs   │       │  ExternalAhrs   │
 │  (EKF/DCM)      │       │  (BNO086)       │       │  (Future: VN-100) │
 └────────▲────────┘       └────────▲────────┘       └─────────────────┘
          │                         │
 ┌────────┴────────┐       ┌────────┴────────┐
 │   RawImu Trait  │       │ QuaternionSensor│
 │  (Raw Data Output) │       │  (Existing Trait) │
 └────────▲────────┘       └─────────────────┘
          │
 ┌────────┴────────┐
 │  ICM-42688      │
 │  BMI270 etc.    │
 └─────────────────┘
```

### Decision Drivers

1. **Separation of Concerns**: Flight control should depend on abstract `Ahrs` trait, not specific sensor implementations
2. **ArduPilot Compatibility**: Follow proven architecture pattern from ArduPilot
3. **Resource Efficiency**: BNO086 users should not pay CPU cost for unused software EKF
4. **Extensibility**: Easy to add new sensor types without modifying flight control

### Considered Options

- **Option A**: Unified `Ahrs` trait with multiple implementations (Selected)
- **Option B**: Separate code paths for raw IMU vs fusion sensors
- **Option C**: Always run software EKF, ignore sensor fusion output

### Option Analysis

- **Option A** — Pros: Clean abstraction, ArduPilot-like, extensible | Cons: Additional trait complexity
- **Option B** — Pros: Simple initial implementation | Cons: Code duplication, harder to maintain
- **Option C** — Pros: Consistent processing | Cons: Wastes BNO086 fusion, higher CPU usage

## Rationale

Option A was chosen because:

1. **Proven Pattern**: ArduPilot has successfully used this architecture for 10+ years
2. **Resource Efficiency**: Allows BNO086 to bypass software EKF entirely
3. **Future-Proof**: Adding ICM-42688 requires only implementing `RawImu` trait and wiring to `SoftwareAhrs`
4. **Testability**: Each component can be tested independently

## Consequences

### Positive

- Flight control code is sensor-agnostic
- BNO086 users get low CPU overhead (no redundant EKF)
- Clear upgrade path to raw IMU + software EKF
- Follows industry-standard architecture (ArduPilot)

### Negative

- Additional abstraction layer adds some complexity
- Two different code paths to maintain (SoftwareAhrs, ExternalAhrs)
- Need to ensure both paths produce consistent output format

### Neutral

- Existing `QuaternionSensor` trait will be wrapped by `ExternalAhrs` implementation

## Implementation Notes

### Core Traits

```rust
/// AHRS output - common interface for all attitude sources
pub trait Ahrs {
    /// Get current attitude as quaternion
    async fn get_attitude(&mut self) -> Result<AhrsState, AhrsError>;

    /// Check if AHRS is healthy and producing valid data
    fn is_healthy(&self) -> bool;

    /// Get AHRS type identifier (for logging/diagnostics)
    fn ahrs_type(&self) -> AhrsType;
}

/// AHRS state output
pub struct AhrsState {
    /// Attitude quaternion (NED frame)
    pub quaternion: Quaternion<f32>,

    /// Angular rates in body frame (rad/s)
    pub angular_rate: Vector3<f32>,

    /// Linear acceleration in body frame (m/s²)
    pub acceleration: Vector3<f32>,

    /// Timestamp (microseconds)
    pub timestamp_us: u64,
}

/// AHRS type for runtime identification
pub enum AhrsType {
    /// Software EKF/DCM processing raw IMU data
    Software,
    /// External AHRS (BNO086, VectorNav, etc.)
    External,
}
```

### Raw IMU Trait (for future ICM-42688 support)

```rust
/// Raw IMU sensor interface
pub trait RawImu {
    /// Read calibrated accelerometer data (m/s²)
    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read calibrated gyroscope data (rad/s)
    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read calibrated magnetometer data (optional)
    async fn read_mag(&mut self) -> Result<Option<Vector3<f32>>, ImuError>;

    /// Get sample rate (Hz)
    fn sample_rate(&self) -> u32;
}
```

### External AHRS Implementation (BNO086)

```rust
/// BNO086 as External AHRS
pub struct Bno086ExternalAhrs<T: ShtpTransport> {
    driver: Bno086Driver<T>,
}

impl<T: ShtpTransport> Ahrs for Bno086ExternalAhrs<T> {
    async fn get_attitude(&mut self) -> Result<AhrsState, AhrsError> {
        let reading = self.driver.read_quaternion().await?;
        Ok(AhrsState {
            quaternion: reading.quaternion,
            angular_rate: Vector3::zeros(), // BNO086 can provide this via separate report
            acceleration: Vector3::zeros(), // BNO086 can provide this via separate report
            timestamp_us: reading.timestamp_us,
        })
    }

    fn is_healthy(&self) -> bool {
        self.driver.is_healthy()
    }

    fn ahrs_type(&self) -> AhrsType {
        AhrsType::External
    }
}
```

### Software AHRS Implementation (Future)

```rust
/// Software EKF for raw IMU sensors
pub struct SoftwareAhrs<I: RawImu> {
    imu: I,
    ekf: EkfState,
}

impl<I: RawImu> Ahrs for SoftwareAhrs<I> {
    async fn get_attitude(&mut self) -> Result<AhrsState, AhrsError> {
        // Read raw sensor data
        let accel = self.imu.read_accel().await?;
        let gyro = self.imu.read_gyro().await?;

        // Update EKF
        self.ekf.predict(gyro, dt);
        self.ekf.update_accel(accel);

        Ok(AhrsState {
            quaternion: self.ekf.get_quaternion(),
            angular_rate: gyro,
            acceleration: accel,
            timestamp_us: timestamp_us(),
        })
    }

    fn is_healthy(&self) -> bool {
        self.ekf.is_converged()
    }

    fn ahrs_type(&self) -> AhrsType {
        AhrsType::Software
    }
}
```

### Usage in Flight Control

```rust
/// Flight control is AHRS-agnostic
pub async fn attitude_control_loop<A: Ahrs>(ahrs: &mut A) {
    loop {
        let state = ahrs.get_attitude().await?;

        // Same code works for BNO086 External AHRS or ICM-42688 + Software EKF
        let error = compute_attitude_error(state.quaternion, target);
        let output = pid_controller.update(error, state.angular_rate);

        apply_motor_output(output);
    }
}
```

### File Structure

```
src/
├── ahrs/
│   ├── mod.rs              # Ahrs trait definition
│   ├── state.rs            # AhrsState struct
│   ├── external/
│   │   ├── mod.rs
│   │   └── bno086.rs       # Bno086ExternalAhrs
│   └── software/
│       ├── mod.rs
│       └── ekf.rs          # SoftwareAhrs<I> (future)
├── devices/
│   ├── imu/
│   │   ├── bno086/         # Existing driver (unchanged)
│   │   └── icm42688/       # Future raw IMU driver
│   └── traits/
│       ├── mod.rs
│       ├── quaternion.rs   # Existing QuaternionSensor
│       └── raw_imu.rs      # New RawImu trait
```

## Open Questions

- [x] Should `AhrsState` include covariance/uncertainty estimates? → Resolved: Added `accuracy_rad: Option<f32>` field for external AHRS
- [x] How to handle BNO086 angular rate output (separate report)? → Resolved: Must enable GYROSCOPE_CALIBRATED (Report ID 0x05) alongside rotation vector. Angular rate is REQUIRED for PID D-term.
- [x] BNO086 coordinate frame conversion? → Resolved: BNO086 outputs Z-up frame, must convert to NED in `Bno086ExternalAhrs` implementation
- [ ] GPS velocity integration for yaw correction? → Next step: Design in separate ADR when GPS integration is planned

## External References

- [ArduPilot AP_AHRS Source](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_AHRS)
- [ArduPilot External AHRS Documentation](https://ardupilot.org/copter/docs/common-external-ahrs.html)
- [CEVA SH-2 Reference Manual](https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf)
- [BNO086 Datasheet](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf)
