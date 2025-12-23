# ADR-t5cq4 MPU-9250 I2C Driver Architecture

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [NFR-3wlo1-imu-sampling-rate](../requirements/NFR-3wlo1-imu-sampling-rate.md)
  - [FR-eyuh8-ahrs-attitude-estimation](../requirements/FR-eyuh8-ahrs-attitude-estimation.md)
  - [FR-oqxl8-mpu9250-i2c-driver](../requirements/FR-oqxl8-mpu9250-i2c-driver.md)
  - [FR-z1fdo-imu-sensor-trait](../requirements/FR-z1fdo-imu-sensor-trait.md)
  - [FR-soukr-imu-calibration-interface](../requirements/FR-soukr-imu-calibration-interface.md)
  - [NFR-ulsja-imu-i2c-read-latency](../requirements/NFR-ulsja-imu-i2c-read-latency.md)
  - [NFR-wkrr5-imu-read-reliability](../requirements/NFR-wkrr5-imu-read-reliability.md)
- Supersedes ADRs:
  - [ADR-wjcrn-imu-driver-architecture](../adr/ADR-wjcrn-imu-driver-architecture.md) (Deprecated - BMI088/SPI)
- Related Analyses:
  - [AN-t47be-mpu9250-imu-and-ekf-integration](../analysis/AN-t47be-mpu9250-imu-and-ekf-integration.md)
- Related Tasks:
  - [T-kx79g-mpu9250-driver-implementation](../tasks/T-kx79g-mpu9250-driver-implementation/README.md)

## Context

The pico_trail autopilot requires IMU sensor data for attitude estimation via EKF. The previous ADR (ADR-wjcrn) specified BMI088 with SPI interface, but project constraints now favor MPU-9250 with I2C.

**Problem Statement:**

- Provide 9-axis sensor data (gyro, accel, mag) to EKF at 400Hz
- Support both RP2040 (Pico W) and RP2350 (Pico 2 W) platforms
- Enable future sensor upgrades through hardware abstraction
- Provide calibration data interface for offset/scale corrections

**Constraints:**

- Embedded environment: No heap allocation, static buffers only
- I2C bus may be shared with other devices (GPS)
- Embassy async runtime for task management
- Must work without FPU on RP2040

**Assumptions:**

- Single IMU instance sufficient (no redundancy for rover/boat)
- Calibration data stored in parameter system
- EKF handles sensor fusion (driver provides raw calibrated data)

**Forces in Tension:**

1. **I2C vs SPI**: I2C simpler wiring vs SPI lower latency
2. **Abstraction vs Performance**: Trait overhead vs direct HAL access
3. **Magnetometer**: Integrated (MPU-9250) vs external (better placement)

## Success Metrics

- **Sampling Rate**: Sustained 400Hz ± 5Hz over 10-second window
- **Jitter**: < 1ms standard deviation measured via timestamps
- **I2C Latency**: Full 9-axis read completes in < 1.5ms
- **CPU Overhead**: < 15% CPU time on RP2040 @ 133MHz for IMU task
- **Reliability**: < 1 read error per 1000 samples under normal operation
- **Platform Support**: Works on both RP2040 and RP2350 without modification

## Decision

**We will implement an MPU-9250 driver using I2C communication with a trait-based abstraction layer to provide 9-axis sensor data to the EKF subsystem.**

### Core Decisions

1. **Sensor Selection**: MPU-9250 (9-axis IMU)
   - Rationale: Integrated magnetometer simplifies hardware; widely available

2. **Communication Interface**: I2C at 400kHz
   - Rationale: Simpler wiring (4 wires), adequate for 400Hz, shared bus capability

3. **Sampling Strategy**: Timer-based polling at 400Hz
   - Rationale: Simpler than interrupt-driven; MPU-9250 DRDY pin optional

4. **Driver Architecture**: Trait-based with two layers
   - `ImuSensor` trait: Device-independent interface for EKF
   - `Mpu9250Driver`: MPU-9250-specific implementation

5. **Magnetometer Handling**: Use integrated AK8963 at 100Hz
   - Rationale: Avoid external sensor complexity initially
   - Future: Can add external mag if interference issues arise

6. **Calibration**: Load from parameters, apply in driver
   - Rationale: Consistent with existing parameter system

### Decision Drivers

1. **Hardware Availability**: MPU-9250 readily available on breakout boards
2. **Wiring Simplicity**: I2C requires only 4 wires (VCC, GND, SDA, SCL)
3. **9-axis Integration**: Avoids separate magnetometer module
4. **Adequate Performance**: 400Hz achievable with I2C @ 400kHz
5. **Maintainability**: Trait abstraction enables testing and future sensors

### Considered Options

**Option A: MPU-9250 + I2C** (Selected)

- Pros: Simple wiring, integrated mag, widely available
- Cons: Higher latency than SPI, EOL product

**Option B: MPU-9250 + SPI**

- Pros: Lower latency (\~0.3ms), lower jitter
- Cons: More wiring (6+ wires), more complex integration

**Option C: ICM-20948 + I2C**

- Pros: Current production, better specs than MPU-9250
- Cons: Less common, different register map, higher cost

### Option Analysis

- **Option A** — Best balance of simplicity and capability | Adequate for 400Hz requirements
- **Option B** — Lower latency not needed for 400Hz | Added wiring complexity
- **Option C** — Future upgrade path if MPU-9250 unavailable | Deferred

## Rationale

**Why MPU-9250 + I2C?**

1. **Simplicity**: I2C requires 4 wires vs 6+ for SPI, reducing integration complexity and potential wiring errors.

2. **Adequate Performance**: At 400kHz I2C, a full 14-byte read (gyro + accel + temp) completes in \~1ms, well within the 2.5ms sample period.

3. **Integrated Magnetometer**: The AK8963 magnetometer eliminates need for a separate sensor, simplifying calibration and mounting.

4. **Compatibility**: I2C is the default interface on most MPU-9250 breakout boards, matching available hardware.

5. **Shared Bus**: I2C allows bus sharing with GPS or other sensors if needed, though dedicated bus is preferred.

**Why Not SPI?**

- SPI provides \~0.3ms read latency vs \~1.5ms for I2C, but this margin is not needed at 400Hz.
- SPI requires more GPIO pins and wiring complexity.
- Most MPU-9250 breakouts default to I2C configuration.

**Trade-offs Accepted:**

- **Higher Latency**: 1.5ms I2C read vs 0.3ms SPI (acceptable for 400Hz)
- **EOL Sensor**: MPU-9250 discontinued but widely available; ICM-20948 as backup
- **Bus Contention Risk**: Mitigated by dedicated I2C bus or careful timing

## Consequences

### Positive

- **Simple Integration**: 4-wire I2C connection to MPU-9250 breakout
- **9-axis Data**: Gyro, accel, mag from single sensor
- **Testable**: Mock `ImuSensor` implementation for unit tests
- **Future-Proof**: Trait abstraction allows sensor upgrades
- **Calibration Support**: Offset/scale parameters for all axes

### Negative

- **Higher Latency**: \~1.5ms per read vs \~0.3ms with SPI
- **Jitter Risk**: I2C clock stretching can cause timing variation
- **EOL Sensor**: MPU-9250 discontinued (mitigated by stocking)
- **Bus Contention**: Shared I2C may conflict with other devices

### Neutral

- **Magnetometer Quality**: AK8963 adequate for heading; external mag optional
- **DMP Unused**: MPU-9250 DMP available but not used (EKF handles fusion)

## Implementation Notes

### Module Structure

```
src/devices/
├── traits/
│   └── imu.rs                  # ImuSensor trait
└── imu/
    ├── mod.rs                  # Public exports
    ├── mpu9250/
    │   ├── mod.rs              # Driver public API
    │   ├── registers.rs        # Register definitions
    │   ├── config.rs           # Configuration structs
    │   └── driver.rs           # Core driver implementation
    └── mock.rs                 # Mock IMU for testing
```

### Interface Design

```rust
/// Device-independent IMU interface for EKF
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

/// Combined IMU reading
pub struct ImuReading {
    pub gyro: Vector3<f32>,      // rad/s
    pub accel: Vector3<f32>,     // m/s²
    pub mag: Vector3<f32>,       // µT
    pub temperature: f32,        // °C
    pub timestamp_us: u64,
}
```

### MPU-9250 Configuration

```rust
pub struct Mpu9250Config {
    pub gyro_range: GyroRange,      // ±250, ±500, ±1000, ±2000 °/s
    pub accel_range: AccelRange,    // ±2, ±4, ±8, ±16 g
    pub gyro_dlpf: DlpfConfig,      // Digital low-pass filter
    pub accel_dlpf: DlpfConfig,
    pub sample_rate_div: u8,        // ODR = 1kHz / (1 + div)
    pub mag_mode: MagMode,          // Single, continuous 8Hz/100Hz
}

impl Default for Mpu9250Config {
    fn default() -> Self {
        Self {
            gyro_range: GyroRange::Dps2000,
            accel_range: AccelRange::G8,
            gyro_dlpf: DlpfConfig::Bw184Hz,
            accel_dlpf: DlpfConfig::Bw184Hz,
            sample_rate_div: 0,     // 1kHz ODR
            mag_mode: MagMode::Continuous100Hz,
        }
    }
}
```

### I2C Communication

```rust
// MPU-9250 I2C addresses
const MPU9250_ADDR: u8 = 0x68;      // AD0 pin low
const MPU9250_ADDR_ALT: u8 = 0x69;  // AD0 pin high
const AK8963_ADDR: u8 = 0x0C;       // Magnetometer (via bypass)

// Read all sensor data in single transaction
async fn read_sensors(&mut self) -> Result<RawData, ImuError> {
    // Read 14 bytes: accel (6) + temp (2) + gyro (6)
    let mut buf = [0u8; 14];
    self.i2c.write_read(MPU9250_ADDR, &[ACCEL_XOUT_H], &mut buf).await?;

    // Read magnetometer separately (7 bytes including status)
    let mut mag_buf = [0u8; 7];
    self.i2c.write_read(AK8963_ADDR, &[AK8963_XOUT_L], &mut mag_buf).await?;

    Ok(RawData::from_bytes(&buf, &mag_buf))
}
```

### Calibration Integration

```rust
pub struct ImuCalibration {
    pub gyro_bias: Vector3<f32>,        // rad/s offset
    pub accel_offset: Vector3<f32>,     // m/s² offset
    pub accel_scale: Vector3<f32>,      // scale factor per axis
    pub mag_offset: Vector3<f32>,       // µT hard iron offset
    pub mag_scale: Matrix3<f32>,        // soft iron matrix
}

impl Mpu9250Driver {
    fn apply_calibration(&self, raw: RawData) -> ImuReading {
        let gyro = raw.gyro - self.calibration.gyro_bias;
        let accel = (raw.accel - self.calibration.accel_offset)
            .component_mul(&self.calibration.accel_scale);
        let mag = self.calibration.mag_scale * (raw.mag - self.calibration.mag_offset);

        ImuReading { gyro, accel, mag, .. }
    }
}
```

### Error Handling

```rust
pub enum ImuError {
    I2cError,           // I2C communication failed
    InvalidData,        // Data validation failed (e.g., stuck sensor)
    NotInitialized,     // Driver not initialized
    SelfTestFailed,     // Sensor self-test failed
    MagNotReady,        // Magnetometer data not ready
}

// Retry logic for transient errors
const MAX_RETRIES: u8 = 3;
```

## Platform Considerations

### RP2040 (Pico W)

- **No FPU**: Data conversion uses software floating-point
- **I2C**: Hardware I2C0/I2C1 with DMA support
- **Clock**: 133MHz sufficient for 400Hz with I2C overhead

### RP2350 (Pico 2 W)

- **Hardware FPU**: Faster data conversion
- **I2C**: Same hardware I2C support
- **Clock**: 150MHz provides more headroom

### Cross-Platform

- Use `embassy-rp` I2C traits for portability
- Conditional compilation for platform optimizations
- Same driver code works on both platforms

## Monitoring & Logging

```rust
// ERROR: Critical failures
log_error!("MPU-9250 initialization failed: {:?}", error);

// WARN: Recoverable issues
log_warn!("IMU read retry {}/3", attempt);
log_warn!("Magnetometer data not ready");

// INFO: Operational events
log_info!("MPU-9250 initialized: whoami={:#x}", whoami);

// DEBUG: Detailed diagnostics
log_debug!("IMU sample: dt={}us, jitter={}us", dt, jitter);

// TRACE: Raw data (high volume)
log_trace!("I2C read: addr={:#x}, len={}", addr, len);
```

## Open Questions

- [ ] Should we implement I2C DMA for lower CPU overhead? → Method: Benchmark CPU usage, add DMA if >20%
- [ ] Is bypass mode or master mode better for AK8963 access? → Decision: Start with bypass (simpler)
- [ ] Do we need MPU-9250 FIFO for data buffering? → Method: Test jitter without FIFO first
- [ ] Should we support ICM-20948 as drop-in replacement? → Next step: Research register compatibility

## External References

- [MPU-9250 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf)
- [MPU-9250 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [AK8963 Datasheet](https://www.akm.com/content/dam/documents/products/electronic-compass/ak8963c/ak8963c-en-datasheet.pdf)
- [Embassy RP I2C](https://docs.embassy.dev/embassy-rp/git/rp2040/i2c/index.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
