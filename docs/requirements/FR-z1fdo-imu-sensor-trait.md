# FR-z1fdo ImuSensor Trait Interface

## Metadata

- Type: Functional Requirement
- Status: Draft

## Change History

- **2025-01-XX**: Updated to support ICM-20948 (primary) and MPU-9250 (backup) sensors.

## Links

- Prerequisite Requirements:
  - [FR-slm3x-icm20948-i2c-driver](FR-slm3x-icm20948-i2c-driver.md)
  - [FR-oqxl8-mpu9250-i2c-driver](FR-oqxl8-mpu9250-i2c-driver.md)
- Dependent Requirements:
  - [FR-3f2cn-quaternion-ekf-ahrs](FR-3f2cn-quaternion-ekf-ahrs.md)
  - [FR-soukr-imu-calibration-interface](FR-soukr-imu-calibration-interface.md)
- Related ADRs:
  - [ADR-t5cq4-mpu9250-i2c-driver-architecture](../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)
- Related Tasks:
  - [T-x8mq2-bno086-driver-implementation](../tasks/T-x8mq2-bno086-driver-implementation/README.md)
  - [T-7khm3-ahrs-abstraction-layer](../tasks/T-7khm3-ahrs-abstraction-layer/README.md)
  - ~~[T-0kbo4-icm20948-driver-implementation](../tasks/T-0kbo4-icm20948-driver-implementation/README.md)~~ (Cancelled)
  - ~~[T-kx79g-mpu9250-driver-implementation](../tasks/T-kx79g-mpu9250-driver-implementation/README.md)~~ (Cancelled)

## Requirement Statement

The system shall provide a device-independent `ImuSensor` trait interface that abstracts IMU hardware specifics, enabling the EKF subsystem to consume sensor data without knowledge of the underlying sensor implementation.

## Rationale

A trait-based abstraction layer provides:

- **Testability**: Mock implementations for unit tests without hardware
- **Sensor Independence**: EKF code unchanged when sensor hardware changes (ICM-20948 or MPU-9250)
- **Multiple Sensor Support**: ICM-20948 (primary) and MPU-9250 (backup) share same interface
- **Separation of Concerns**: Driver handles hardware, EKF handles algorithm

## User Story (if applicable)

As a developer, I want the EKF to consume IMU data through a trait interface, so that I can test the EKF with mock data and swap sensors without modifying EKF code.

## Acceptance Criteria

- [ ] `ImuSensor` trait defines async methods for reading sensor data
- [ ] `read_all()` returns combined gyro, accel, mag, temperature, and timestamp
- [ ] Individual read methods: `read_gyro()`, `read_accel()`, `read_mag()`
- [ ] Calibration interface: `set_calibration()` accepts calibration data
- [ ] Health query: `is_healthy()` returns sensor status
- [ ] `ImuReading` struct contains all 9-axis data with units specified
- [ ] `ImuError` enum covers all possible failure modes
- [ ] Mock implementation available for testing
- [ ] ICM-20948 driver implements `ImuSensor` trait (primary)
- [ ] MPU-9250 driver implements `ImuSensor` trait (backup)

## Technical Details (if applicable)

### Functional Requirement Details

**Trait Definition:**

```rust
/// Device-independent IMU interface for EKF
#[allow(async_fn_in_trait)]
pub trait ImuSensor {
    /// Read all 9 axes: gyro (rad/s), accel (m/s²), mag (µT)
    async fn read_all(&mut self) -> Result<ImuReading, ImuError>;

    /// Read gyroscope only (rad/s, body frame)
    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read accelerometer only (m/s², body frame)
    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read magnetometer only (µT, body frame)
    async fn read_mag(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Apply calibration data
    fn set_calibration(&mut self, calibration: ImuCalibration);

    /// Get sensor health status
    fn is_healthy(&self) -> bool;
}
```

**Data Structures:**

```rust
/// Combined IMU reading with calibration applied
pub struct ImuReading {
    /// Gyroscope: rad/s, body frame (X=right, Y=forward, Z=up)
    pub gyro: Vector3<f32>,

    /// Accelerometer: m/s², body frame
    pub accel: Vector3<f32>,

    /// Magnetometer: µT, body frame
    pub mag: Vector3<f32>,

    /// Temperature: °C
    pub temperature: f32,

    /// Timestamp: microseconds since boot
    pub timestamp_us: u64,
}

/// Calibration data for IMU sensors
pub struct ImuCalibration {
    /// Gyroscope bias: rad/s offset to subtract
    pub gyro_bias: Vector3<f32>,

    /// Accelerometer offset: m/s² to subtract
    pub accel_offset: Vector3<f32>,

    /// Accelerometer scale: per-axis scale factors
    pub accel_scale: Vector3<f32>,

    /// Magnetometer hard iron offset: µT to subtract
    pub mag_offset: Vector3<f32>,

    /// Magnetometer soft iron matrix: 3x3 correction matrix
    pub mag_scale: Matrix3<f32>,
}

/// IMU error types
pub enum ImuError {
    /// I2C/SPI communication failed
    I2cError,

    /// Data validation failed (e.g., stuck sensor)
    InvalidData,

    /// Driver not initialized
    NotInitialized,

    /// Sensor self-test failed
    SelfTestFailed,

    /// Magnetometer data not ready
    MagNotReady,
}
```

**Coordinate Frame:**

All data uses NED (North-East-Down) body frame convention:

- X: Right (starboard)
- Y: Forward (bow)
- Z: Down

Axis alignment must be handled in driver if sensor orientation differs.

## Platform Considerations

### Cross-Platform

- Trait uses `#[cfg(feature = "embassy")]` for async methods
- Host tests use synchronous mock implementation
- `Vector3` and `Matrix3` from `nalgebra` with `no_std` support

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                                                 | Validation                                  |
| ---------------------------------- | ------ | ---------- | ---------------------------------------------------------- | ------------------------------------------- |
| Trait overhead impacts performance | Low    | Low        | Inline methods, monomorphization optimizes                 | Benchmark trait call vs direct call         |
| Coordinate frame confusion         | High   | Medium     | Document frame clearly, validate with IMU on level surface | Test roll/pitch/yaw with known orientations |
| Mock diverges from real behavior   | Medium | Medium     | Keep mock simple, validate with hardware tests             | Integration test on hardware before release |

## Implementation Notes

**Module Location:**

```
src/devices/
├── traits/
│   └── imu.rs              # ImuSensor trait, ImuReading, ImuError
└── imu/
    ├── mod.rs              # Re-exports
    ├── icm20948/           # ICM-20948 implementation (primary)
    ├── mpu9250/            # MPU-9250 implementation (backup)
    └── mock.rs             # Mock for testing
```

**Mock Implementation:**

```rust
/// Mock IMU for testing
pub struct MockImu {
    readings: VecDeque<ImuReading>,
    healthy: bool,
}

impl ImuSensor for MockImu {
    async fn read_all(&mut self) -> Result<ImuReading, ImuError> {
        self.readings.pop_front().ok_or(ImuError::InvalidData)
    }
    // ... other methods
}
```

**Usage Pattern:**

```rust
// EKF is generic over ImuSensor
async fn run_ekf<I: ImuSensor>(mut imu: I, mut ekf: AhrsEkf) {
    loop {
        let reading = imu.read_all().await?;
        ekf.predict(reading.gyro, dt);
        // ...
    }
}

// Production: ICM-20948 (primary)
let imu = Icm20948Driver::new(i2c, config).await?;
run_ekf(imu, ekf).await;

// Production: MPU-9250 (backup)
let imu = Mpu9250Driver::new(i2c, config).await?;
run_ekf(imu, ekf).await;

// Testing: Mock
let imu = MockImu::with_readings(test_data);
run_ekf(imu, ekf).await;
```

## External References

- N/A - No external references

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
