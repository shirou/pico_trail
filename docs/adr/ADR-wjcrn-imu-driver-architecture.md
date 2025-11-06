# ADR-wjcrn IMU Driver Architecture

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [NFR-3wlo1-imu-sampling-rate](../requirements/NFR-3wlo1-imu-sampling-rate.md)
  - [FR-eyuh8-ahrs-attitude-estimation](../requirements/FR-eyuh8-ahrs-attitude-estimation.md)
- Related Analyses:
  - [AN-yhnjd-imu-sensor-selection](../analysis/AN-yhnjd-imu-sensor-selection.md)
- Related ADRs:
  - [ADR-6twis-ahrs-algorithm-selection](../adr/ADR-6twis-ahrs-algorithm-selection.md)
  - [ADR-oa2qa-platform-abstraction](../adr/ADR-oa2qa-platform-abstraction.md)
- Related Tasks:
  - [T-49k7n-ahrs-dcm-implementation](../tasks/T-49k7n-ahrs-dcm-implementation/README.md)
  - [T-qwvco-bmi088-imu-driver-implementation](../tasks/T-qwvco-bmi088-imu-driver-implementation/README.md)

## Context

The AHRS subsystem requires IMU sensor data at 400Hz with low jitter (<1ms) to provide accurate attitude estimation. The system must:

**Problem Statement:**

- Provide gyroscope and accelerometer data to AHRS at 400Hz
- Support optional magnetometer for heading correction
- Minimize sampling jitter for stable attitude estimation
- Operate reliably on both RP2040 (Pico W) and RP2350 (Pico 2 W)
- Keep CPU overhead low (<0.5ms per sample) for other tasks

**Constraints:**

- Embedded environment: No heap allocation, static buffers only
- Real-time: 400Hz sampling must not be interrupted
- Resource-limited: RP2040 has no FPU, limited CPU cycles
- Platform abstraction: Must use existing SPI/I2C/GPIO traits
- Embassy async: Must integrate with Embassy runtime

**Assumptions:**

- AHRS DCM algorithm is implemented and ready (ADR-6twis)
- Platform abstraction layer provides SPI/I2C/GPIO traits (ADR-oa2qa)
- Single IMU instance sufficient (no redundancy for rover/boat)
- Calibration data will be stored in parameter system

**Forces in Tension:**

1. **Performance vs Simplicity**: High-rate sampling requires careful timing, but driver should remain simple
2. **Portability vs Optimization**: Trait-based abstraction adds overhead vs direct HAL access
3. **Sensor Choice vs Cost**: BMI088 is best performance but higher cost than MPU6050
4. **Interrupt vs Polling**: Interrupts provide deterministic timing but add complexity

## Success Metrics

- **Sampling Rate**: Sustained 400Hz ± 5Hz over 10-second window
- **Jitter**: Standard deviation < 1ms measured via timestamps
- **Latency**: Sensor read completes in < 0.5ms (SPI transaction + conversion)
- **CPU Overhead**: < 10% CPU time on RP2040 @ 133MHz for IMU task
- **Reliability**: < 1 read error per 1000 samples under normal operation
- **AHRS Integration**: AHRS achieves ±2° roll/pitch accuracy with IMU data

## Decision

**We will implement a BMI088-based IMU driver using SPI communication, interrupt-driven sampling, and a trait-based abstraction layer.**

### Core Decisions

1. **Sensor Selection**: BMI088 (6-axis high-performance IMU)
   - Rationale: Meets all requirements (1600Hz ODR, SPI, low jitter)
   - External magnetometer (QMC5883L/HMC5883L) added separately for heading

2. **Communication Interface**: SPI at 1-8 MHz
   - Rationale: Low latency (<0.3ms), minimal jitter, deterministic timing
   - I2C rejected due to jitter concerns (clock stretching)

3. **Sampling Strategy**: Interrupt-driven (data-ready pin)
   - Rationale: Hardware-triggered sampling ensures 400Hz precision
   - Gyro INT1 and Accel INT2 used to synchronize reads

4. **Driver Architecture**: Trait-based with three layers
   - `ImuSensor` trait: Device-independent interface for AHRS
   - `Bmi088Driver`: BMI088-specific implementation
   - `SpiInterface` trait: Platform abstraction (existing)

5. **Error Handling**: Retry with exponential backoff, sensor reset on persistent failure
   - Rationale: Transient SPI errors should not hang the system
   - Max 3 retries before reporting error to AHRS (graceful degradation)

### Decision Drivers

1. **400Hz Sampling Requirement**: NFR-3wlo1 mandates 400Hz with <1ms jitter
2. **CPU Budget**: Must leave cycles for AHRS (100Hz), control loops (50Hz), telemetry
3. **ArduPilot Compatibility**: Use proven sensor and architecture patterns
4. **Maintainability**: Trait-based design allows future sensor support (ICM-42688, etc.)
5. **Testability**: Mock implementations enable unit testing without hardware

### Considered Options

**Option A: BMI088 + SPI + Interrupt** (Selected)

- Pros: Best performance, deterministic timing, industry standard
- Cons: Higher cost, external magnetometer needed, moderate driver complexity

**Option B: MPU9250 + SPI + Interrupt**

- Pros: Integrated magnetometer, good performance, widely available
- Cons: EOL product (discontinued 2020), magnetometer quality issues

**Option C: MPU6050 + I2C + Polling**

- Pros: Low cost, simple driver, many examples available
- Cons: I2C jitter risk, polling overhead, no magnetometer, entry-level performance

**Option D: Multiple IMU Redundancy**

- Pros: Fault tolerance, sensor fusion across multiple IMUs
- Cons: Excessive complexity for rover/boat, high cost, not required

### Option Analysis

- **Option A** — Best technical fit, proven in ArduPilot/PX4, meets all requirements | Higher cost, external mag
- **Option B** — Good all-in-one solution | EOL risk, no long-term availability
- **Option C** — Cheapest, simplest | Jitter concerns, may not meet NFR-3wlo1
- **Option D** — Overkill for application | Unnecessary complexity and cost

## Rationale

**Why BMI088 + SPI + Interrupt?**

1. **Technical Requirements**: BMI088 is the only option that guarantees <1ms jitter via SPI and hardware interrupts. MPU6050 I2C is too risky for jitter compliance.

2. **ArduPilot Validation**: BMI088 is proven in production autopilots (ArduPilot, PX4), reducing integration risk. We benefit from their extensive testing.

3. **Performance Headroom**: 1600Hz ODR provides 4x margin over 400Hz requirement, allowing for future optimization (e.g., hardware FIFO, decimation filters).

4. **Deterministic Timing**: Hardware data-ready interrupts eliminate polling jitter and CPU waste. AHRS gets samples at precisely 400Hz.

5. **Scalability**: Trait-based design allows easy addition of ICM-42688 or other sensors in future without changing AHRS code.

**Why Not Alternatives?**

- **MPU9250**: EOL status means uncertain long-term availability, not suitable for production
- **MPU6050**: I2C jitter measured at 0.5-2ms on RP2040, fails NFR-3wlo1
- **Multi-IMU**: Adds complexity without benefit for ground vehicles (aerial drones need redundancy)

**Trade-offs Accepted:**

- **Cost**: \~$12 vs \~$3 for MPU6050, justified by technical superiority
- **External Magnetometer**: Adds wiring complexity but enables better sensor placement away from interference
- **Driver Complexity**: BMI088 has separate gyro/accel datasheets, but trait abstraction hides complexity from AHRS

## Consequences

### Positive

- **Meets All Requirements**: 400Hz, <1ms jitter, <0.5ms latency guaranteed by design
- **Future-Proof**: Trait abstraction allows sensor upgrades without AHRS changes
- **Testable**: Mock IMU trait implementation enables unit testing
- **Industry-Standard**: BMI088 is well-documented, proven in production systems
- **Low CPU Overhead**: Interrupt-driven, DMA-capable SPI minimizes CPU time
- **Portable**: Platform abstraction via traits works on RP2040, RP2350, future targets

### Negative

- **Higher BOM Cost**: $12 BMI088 + $3 magnetometer vs $3 all-in-one MPU6050
- **Additional Wiring**: External magnetometer requires separate I2C bus
- **Driver Complexity**: BMI088 configuration more complex than MPU6050
- **Lead Time**: BMI088 less common, 2-4 week sourcing vs immediate availability
- **Calibration Burden**: Separate gyro, accel, mag calibration vs simpler integrated solution

### Neutral

- **No Magnetometer Integration**: External mag is separate task, can be deferred
- **SPI Bus Sharing**: BMI088 requires dedicated CS pin, but SPI bus is shared
- **Embassy Async**: Fits naturally into Embassy runtime, but locks us into async model

## Implementation Notes

### Module Structure

```
src/devices/
├── traits/
│   └── imu.rs                  # ImuSensor trait
└── imu/
    ├── mod.rs                  # Public exports
    ├── bmi088/
    │   ├── mod.rs              # Driver public API
    │   ├── registers.rs        # Register definitions
    │   ├── config.rs           # Configuration structs
    │   └── driver.rs           # Core driver implementation
    └── mock.rs                 # Mock IMU for testing
```

### Interface Design

```rust
/// Device-independent IMU interface for AHRS
pub trait ImuSensor {
    /// Read gyroscope (rad/s, body frame)
    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read accelerometer (m/s², body frame)
    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read magnetometer if available (µT, body frame)
    async fn read_mag(&mut self) -> Result<Option<Vector3<f32>>, ImuError>;

    /// Configure sensor (ODR, range, filters)
    async fn configure(&mut self, config: ImuConfig) -> Result<(), ImuError>;

    /// Perform self-test
    async fn self_test(&mut self) -> Result<SelfTestResult, ImuError>;
}

/// BMI088-specific configuration
pub struct Bmi088Config {
    pub gyro_range: GyroRange,      // ±125, ±250, ±500, ±1000, ±2000 °/s
    pub gyro_odr: GyroOdr,          // 100, 200, 400, 1000, 2000 Hz
    pub accel_range: AccelRange,    // ±3, ±6, ±12, ±24 g
    pub accel_odr: AccelOdr,        // 12.5, 25, 50, 100, 200, 400, 800, 1600 Hz
    pub enable_fifo: bool,
}
```

### SPI Configuration

```rust
// BMI088 SPI settings
let spi_config = SpiConfig {
    frequency: 1_000_000,  // 1 MHz (safe), up to 8 MHz
    mode: SpiMode::Mode3,  // CPOL=1, CPHA=1
    bit_order: BitOrder::MsbFirst,
};
```

### Interrupt Configuration

```rust
// Gyro data-ready on INT1
gpio.configure_interrupt(
    gyro_int1_pin,
    EdgeTrigger::Rising,
    priority: InterruptPriority::High,
);

// Accel data-ready on INT2
gpio.configure_interrupt(
    accel_int2_pin,
    EdgeTrigger::Rising,
    priority: InterruptPriority::High,
);
```

### Error Handling Strategy

```rust
pub enum ImuError {
    SpiError(SpiError),
    InvalidData,
    SelfTestFailed,
    ConfigurationError,
    TimeoutError,
}

// Retry logic
const MAX_RETRIES: u8 = 3;
for attempt in 0..MAX_RETRIES {
    match read_sensor().await {
        Ok(data) => return Ok(data),
        Err(e) if attempt < MAX_RETRIES - 1 => {
            Timer::after(Duration::from_micros(100 * (1 << attempt))).await;
            continue;
        }
        Err(e) => return Err(e),
    }
}
```

### Calibration Integration

```rust
// Load calibration from parameter system
let calibration = CalibrationData::load_from_params(&registry);

// Apply to driver
let mut imu = Bmi088Driver::new(spi, cs_pin, config);
imu.set_gyro_bias(calibration.gyro_bias);
imu.set_accel_calibration(calibration.accel_offset, calibration.accel_scale);
```

### AHRS Integration

```rust
// AHRS task reads IMU via trait
async fn ahrs_task(mut imu: impl ImuSensor) {
    loop {
        // Wait for data-ready interrupt (400Hz)
        wait_for_imu_interrupt().await;

        let gyro = imu.read_gyro().await?;
        let accel = imu.read_accel().await?;

        // Feed to DCM algorithm
        dcm.update(gyro, accel, 0.0025);  // dt = 2.5ms

        // Publish to shared state
        ATTITUDE.update_attitude(roll, pitch, yaw, timestamp_ms);
    }
}
```

## Examples

### Driver Initialization

```rust
use pico_trail::devices::imu::bmi088::{Bmi088Driver, Bmi088Config, GyroRange, AccelRange};
use pico_trail::platform::SpiInterface;

// Initialize SPI
let spi = platform.init_spi(spi_config)?;

// Create BMI088 driver
let config = Bmi088Config {
    gyro_range: GyroRange::Dps2000,
    gyro_odr: GyroOdr::Hz400,
    accel_range: AccelRange::G24,
    accel_odr: AccelOdr::Hz400,
    enable_fifo: false,
};

let mut imu = Bmi088Driver::new(spi, cs_gyro_pin, cs_accel_pin, config);
imu.configure(config).await?;

// Verify communication
let gyro_chip_id = imu.read_gyro_chip_id().await?;
assert_eq!(gyro_chip_id, 0x0F);  // BMI088 gyro chip ID
```

### Reading Sensor Data

```rust
// Synchronous read (blocking)
let gyro = imu.read_gyro().await?;
let accel = imu.read_accel().await?;

println!("Gyro: {:?} rad/s", gyro);
println!("Accel: {:?} m/s²", accel);

// Combined read (optimized)
let (gyro, accel) = imu.read_gyro_accel().await?;
```

### Mock Implementation for Testing

```rust
struct MockImu {
    gyro_data: Vector3<f32>,
    accel_data: Vector3<f32>,
}

impl ImuSensor for MockImu {
    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError> {
        Ok(self.gyro_data)
    }

    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError> {
        Ok(self.accel_data)
    }
}

// Use in tests
#[tokio::test]
async fn test_ahrs_with_mock_imu() {
    let mock_imu = MockImu {
        gyro_data: Vector3::zeros(),
        accel_data: Vector3::new(0.0, 0.0, 9.81),
    };

    let mut dcm = Dcm::new(DcmConfig::default());
    // ... test AHRS with predictable IMU data
}
```

## Platform Considerations

### RP2040 (Pico W) Specifics

- **No FPU**: Data conversion uses software floating-point (slower)
- **Clock Speed**: 133 MHz, sufficient for 400Hz with optimization
- **SPI Speed**: Use 1-2 MHz to stay within timing budget
- **DMA**: Use DMA for SPI reads to reduce CPU overhead
- **Memory**: 264KB RAM, allocate static 1KB buffer for IMU FIFO

### RP2350 (Pico 2 W) Specifics

- **Hardware FPU**: Faster data conversion (3-5x speedup)
- **Clock Speed**: 150 MHz, more headroom for other tasks
- **SPI Speed**: Can use 4-8 MHz safely
- **Memory**: 520KB RAM, no buffer constraints

### Cross-Platform Compatibility

- Use platform traits (`SpiInterface`, `GpioInterface`) for portability
- Avoid platform-specific HAL calls in driver code
- Feature flags for platform-specific optimizations (e.g., `target_arch = "arm"`)

## Security & Privacy

- **No Security Risks**: IMU data is not sensitive, no encryption needed
- **Error Logging**: Log sensor read errors for debugging, no PII involved
- **Calibration Data**: Stored in parameter system, no security implications

## Monitoring & Logging

### Verbosity Levels

```rust
// ERROR: Critical failures (sensor not responding)
defmt::error!("IMU initialization failed: {:?}", error);

// WARN: Recoverable errors (transient SPI errors)
defmt::warn!("IMU read retry {}/3", attempt);

// INFO: Operational events (initialization, configuration changes)
defmt::info!("BMI088 initialized: gyro_id={:#x}, accel_id={:#x}", gyro_id, accel_id);

// DEBUG: Detailed diagnostics (sampling rates, timestamps)
defmt::debug!("IMU sample: dt={}us, jitter={}us", dt, jitter);

// TRACE: Raw data dumps (register reads, SPI transactions)
defmt::trace!("SPI read: addr={:#x}, data={:?}", addr, data);
```

### Performance Metrics

```rust
// Track sampling statistics
struct ImuStats {
    sample_count: u64,
    error_count: u32,
    jitter_us: RingBuffer<u32, 100>,
    last_sample_time: u64,
}

// Log every 10 seconds
if sample_count % 4000 == 0 {
    defmt::info!("IMU stats: samples={}, errors={}, jitter_avg={}us",
        sample_count, error_count, jitter_us.average());
}
```

## Open Questions

- [ ] Should we implement hardware FIFO buffering for robustness? → Next step: Prototype FIFO mode in Phase 2, measure benefits
- [ ] What DMA strategy is optimal for RP2040? → Method: Benchmark DMA vs interrupt-driven SPI
- [ ] Should magnetometer be on same I2C bus as other peripherals? → Decision: Defer to magnetometer integration task
- [ ] Do we need gyro temperature compensation? → Next step: Measure temperature drift on hardware, add if > 0.01°/s/°C
- [ ] Should we support runtime sensor re-initialization? → Method: Add reset API, test recovery from sensor failures

## External References

**Datasheets:**

- [BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
- [BMI088 Application Note](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bmi088-an000.pdf)

**Driver References:**

- [embassy-rp SPI Examples](https://github.com/embassy-rs/embassy/tree/main/examples/rp)
- [BMI088 Arduino Library](https://github.com/bolderflight/bmi088-arduino) (reference only)

**ArduPilot Integration:**

- [ArduPilot IMU Backend](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_InertialSensor)
- [ArduPilot BMI088 Driver](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_InertialSensor/AP_InertialSensor_Invensense.cpp)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
