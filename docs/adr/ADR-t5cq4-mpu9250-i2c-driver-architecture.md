# ADR-t5cq4 9-Axis IMU I2C Driver Architecture

## Metadata

- Type: ADR
- Status: Draft

## Change History

- **2025-01-XX**: Renamed from "MPU-9250" to "9-Axis IMU" to reflect multi-sensor support. Added ICM-20948 as primary sensor option.

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
  - [T-0kbo4-icm20948-driver-implementation](../tasks/T-0kbo4-icm20948-driver-implementation/README.md) (Primary)
  - [T-kx79g-mpu9250-driver-implementation](../tasks/T-kx79g-mpu9250-driver-implementation/README.md) (Cancelled)

## Context

The pico_trail autopilot requires IMU sensor data for attitude estimation via EKF. The previous ADR (ADR-wjcrn) specified BMI088 with SPI interface, but project constraints now favor 9-axis IMUs (ICM-20948 or MPU-9250) with I2C.

**Problem Statement:**

- Provide 9-axis sensor data (gyro, accel, mag) to EKF at 400Hz
- Support both RP2040 (Pico W) and RP2350 (Pico 2 W) platforms
- Support multiple IMU sensors (ICM-20948 primary, MPU-9250 backup) through hardware abstraction
- Provide calibration data interface for offset/scale corrections

**Constraints:**

- Embedded environment: No heap allocation, static buffers only
- I2C bus may be shared with other devices (GPS)
- Embassy async runtime for task management
- Must work without FPU on RP2040
- ICM-20948 uses register bank switching (4 banks)
- MPU-9250 is EOL but hardware may still be available

**Assumptions:**

- Single IMU instance sufficient (no redundancy for rover/boat)
- Calibration data stored in parameter system
- EKF handles sensor fusion (driver provides raw calibrated data)
- Either ICM-20948 or MPU-9250 will be used (not both simultaneously)

**Forces in Tension:**

1. **I2C vs SPI**: I2C simpler wiring vs SPI lower latency
2. **Abstraction vs Performance**: Trait overhead vs direct HAL access
3. **Magnetometer**: Integrated (ICM-20948: AK09916, MPU-9250: AK8963) vs external (better placement)
4. **Code Reuse vs Complexity**: Shared trait interface vs sensor-specific optimizations

## Success Metrics

- **Sampling Rate**: Sustained 400Hz ± 5Hz over 10-second window
- **Jitter**: < 1ms standard deviation measured via timestamps
- **I2C Latency**: Full 9-axis read completes in < 1.5ms
- **CPU Overhead**: < 15% CPU time on RP2040 @ 133MHz for IMU task
- **Reliability**: < 1 read error per 1000 samples under normal operation
- **Platform Support**: Works on both RP2040 and RP2350 without modification

## Decision

**We will implement 9-axis IMU drivers (ICM-20948 primary, MPU-9250 backup) using I2C communication with a trait-based abstraction layer to provide sensor data to the EKF subsystem.**

### Core Decisions

1. **Sensor Selection**: ICM-20948 (primary), MPU-9250 (backup)
   - Rationale: ICM-20948 is actively produced; MPU-9250 is EOL but implementation retained

2. **Communication Interface**: I2C at 400kHz
   - Rationale: Simpler wiring (4 wires), adequate for 400Hz, shared bus capability

3. **Sampling Strategy**: Timer-based polling at 400Hz
   - Rationale: Simpler than interrupt-driven; DRDY pin optional on both sensors

4. **Driver Architecture**: Trait-based with sensor-specific implementations
   - `ImuSensor` trait: Device-independent interface for EKF
   - `Icm20948Driver`: ICM-20948-specific implementation (primary)
   - `Mpu9250Driver`: MPU-9250-specific implementation (backup)

5. **Magnetometer Handling**: Use integrated magnetometers at 100Hz
   - ICM-20948: AK09916 (via I2C master or bypass mode)
   - MPU-9250: AK8963 (via I2C bypass mode)
   - Future: Can add external mag if interference issues arise

6. **Calibration**: Load from parameters, apply in driver
   - Rationale: Consistent with existing parameter system

### Decision Drivers

1. **Hardware Availability**: ICM-20948 actively produced; MPU-9250 EOL
2. **Wiring Simplicity**: I2C requires only 4 wires (VCC, GND, SDA, SCL)
3. **9-axis Integration**: Avoids separate magnetometer module
4. **Adequate Performance**: 400Hz achievable with I2C @ 400kHz
5. **Maintainability**: Trait abstraction enables testing and sensor swapping

### Considered Options

**Option A: ICM-20948 + I2C** (Selected as Primary)

- Pros: Actively produced, better specs, simple wiring, integrated mag
- Cons: Register bank switching adds driver complexity, VDDIO voltage considerations

**Option B: MPU-9250 + I2C** (Retained as Backup)

- Pros: Simple wiring, integrated mag, simpler register map
- Cons: EOL product, higher latency than SPI

**Option C: MPU-9250 + SPI**

- Pros: Lower latency (\~0.3ms), lower jitter
- Cons: More wiring (6+ wires), more complex integration, EOL product

### Option Analysis

- **Option A (ICM-20948)** — **Recommended**: Actively produced, future-proof | Register bank complexity manageable
- **Option B (MPU-9250)** — Retained as backup for existing hardware | EOL limits long-term use
- **Option C (SPI)** — Lower latency not needed for 400Hz | Added wiring complexity

## Rationale

**Why ICM-20948 as Primary Sensor?**

1. **Active Production**: ICM-20948 is the official TDK InvenSense successor to MPU-9250, actively manufactured.

2. **Better Specifications**: Higher ODR capability (9000Hz gyro vs 8000Hz), same programmable ranges.

3. **Same Architecture**: Similar I2C interface (different registers), same general approach.

**Why Retain MPU-9250 Support?**

1. **Existing Hardware**: Users may have MPU-9250 modules that still work.

2. **Partial Implementation**: Code exists in `src/devices/imu/mpu9250/` that can be completed.

3. **Simpler Registers**: No bank switching required, useful as reference implementation.

**Why I2C for Both?**

1. **Simplicity**: I2C requires 4 wires vs 6+ for SPI, reducing integration complexity and potential wiring errors.

2. **Adequate Performance**: At 400kHz I2C, a full 14-byte read (gyro + accel + temp) completes in \~1ms, well within the 2.5ms sample period.

3. **Integrated Magnetometer**: AK09916 (ICM-20948) and AK8963 (MPU-9250) eliminate need for separate sensors.

4. **Compatibility**: I2C is the default interface on most breakout boards (Adafruit, SparkFun).

5. **Shared Bus**: I2C allows bus sharing with GPS or other sensors if needed, though dedicated bus is preferred.

**Why Not SPI?**

- SPI provides \~0.3ms read latency vs \~1.5ms for I2C, but this margin is not needed at 400Hz.
- SPI requires more GPIO pins and wiring complexity.
- Most breakout boards default to I2C configuration.

**Trade-offs Accepted:**

- **Higher Latency**: 1.5ms I2C read vs 0.3ms SPI (acceptable for 400Hz)
- **Register Complexity**: ICM-20948 bank switching adds driver complexity
- **Bus Contention Risk**: Mitigated by dedicated I2C bus or careful timing

## Consequences

### Positive

- **Simple Integration**: 4-wire I2C connection to breakout boards
- **9-axis Data**: Gyro, accel, mag from single sensor
- **Testable**: Mock `ImuSensor` implementation for unit tests
- **Future-Proof**: Trait abstraction allows sensor upgrades; ICM-20948 is actively produced
- **Calibration Support**: Offset/scale parameters for all axes
- **Sensor Choice**: Users can use ICM-20948 (recommended) or MPU-9250 (backup)

### Negative

- **Higher Latency**: \~1.5ms per read vs \~0.3ms with SPI
- **Jitter Risk**: I2C clock stretching can cause timing variation
- **Code Duplication**: Two separate driver implementations (ICM-20948, MPU-9250)
- **Bus Contention**: Shared I2C may conflict with other devices
- **ICM-20948 Complexity**: Register bank switching requires careful state management

### Neutral

- **Magnetometer Quality**: AK09916/AK8963 adequate for heading; external mag optional
- **DMP Unused**: DMP available on both sensors but not used (EKF handles fusion)
- **MPU-9250 EOL**: Implementation retained for existing hardware users

## Implementation Notes

### Module Structure

```
src/devices/
├── traits/
│   └── imu.rs                  # ImuSensor trait
└── imu/
    ├── mod.rs                  # Public exports
    ├── icm20948/               # ICM-20948 (Primary)
    │   ├── mod.rs              # Driver public API
    │   ├── registers.rs        # Register definitions (4 banks)
    │   ├── config.rs           # Configuration structs
    │   └── driver.rs           # Core driver implementation
    ├── mpu9250/                # MPU-9250 (Backup)
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
// ICM-20948 I2C addresses (Primary)
const ICM20948_ADDR: u8 = 0x69;     // AD0 pin low (default on most breakouts)
const ICM20948_ADDR_ALT: u8 = 0x68; // AD0 pin high
const AK09916_ADDR: u8 = 0x0C;      // Magnetometer (via bypass or I2C master)
const ICM20948_WHO_AM_I: u8 = 0xEA;

// MPU-9250 I2C addresses (Backup)
const MPU9250_ADDR: u8 = 0x68;      // AD0 pin low
const MPU9250_ADDR_ALT: u8 = 0x69;  // AD0 pin high
const AK8963_ADDR: u8 = 0x0C;       // Magnetometer (via bypass)
const MPU9250_WHO_AM_I: u8 = 0x71;

// ICM-20948: Read with bank switching
async fn read_sensors_icm20948(&mut self) -> Result<RawData, ImuError> {
    // Select Bank 0 for sensor data
    self.select_bank(0).await?;

    // Read 14 bytes: accel (6) + gyro (6) + temp (2)
    let mut buf = [0u8; 14];
    self.i2c.write_read(ICM20948_ADDR, &[ACCEL_XOUT_H], &mut buf).await?;

    // Read magnetometer via I2C master or bypass mode
    let mut mag_buf = [0u8; 8];
    self.i2c.write_read(AK09916_ADDR, &[AK09916_HXL], &mut mag_buf).await?;

    Ok(RawData::from_bytes(&buf, &mag_buf))
}

// MPU-9250: Read without bank switching
async fn read_sensors_mpu9250(&mut self) -> Result<RawData, ImuError> {
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
- [ ] Is bypass mode or master mode better for magnetometer access? → Decision: Start with bypass (simpler for both sensors)
- [ ] Do we need FIFO for data buffering? → Method: Test jitter without FIFO first
- [x] ~~Should we support ICM-20948 as drop-in replacement?~~ → **Decision: ICM-20948 is now primary sensor**

## External References

**ICM-20948 (Primary):**

- [ICM-20948 Datasheet](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000189-icm-20948-v1.5.pdf)
- [ICM-20948 Product Page](https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/)
- [Adafruit ICM-20948](https://www.adafruit.com/product/4554)
- [SparkFun ICM-20948 Breakout](https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html)

**MPU-9250 (Backup):**

- [MPU-9250 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf)
- [MPU-9250 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [AK8963 Datasheet](https://www.akm.com/content/dam/documents/products/electronic-compass/ak8963c/ak8963c-en-datasheet.pdf)

**Embassy:**

- [Embassy RP I2C](https://docs.embassy.dev/embassy-rp/git/rp2040/i2c/index.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
