# T-00009 BMI088 IMU Driver Implementation

## Metadata

- Type: Design
- Status: Deprecated
  <!-- Deprecated: BMI088 replaced by MPU-9250. See AN-00027-mpu9250-imu-and-ekf-integration. -->

## Links

- Associated Plan Document:
  - [plan.md](./plan.md)

## Overview

This design document describes the implementation of a BMI088-based IMU driver for pico_trail. The driver provides high-rate (400Hz) gyroscope and accelerometer data to the AHRS subsystem with low jitter (<1ms) for accurate attitude estimation. The design follows a trait-based abstraction layer to decouple AHRS from specific sensor implementations, enabling future sensor upgrades and comprehensive unit testing via mock implementations.

## Success Metrics

- [ ] Sampling rate: 400Hz ± 5Hz sustained over 10-second window
- [ ] Jitter: Standard deviation < 1ms measured via timestamps
- [ ] Latency: Sensor read completes in < 0.5ms
- [ ] CPU overhead: < 10% on RP2040 @ 133MHz
- [ ] Reliability: < 1 read error per 1000 samples
- [ ] AHRS integration: ±2° roll/pitch accuracy achieved
- [ ] Platform support: Works on both RP2040 and RP2350 without modification

## Background and Current State

- Context: AHRS subsystem requires IMU sensor data to provide attitude estimation for autonomous navigation and control. The IMU driver is a critical component in the sensor pipeline, feeding data to the DCM-based AHRS algorithm.
- Current behavior: No IMU driver currently exists. AHRS task (T-00005) is implemented but requires IMU sensor interface.
- Pain points: Need deterministic 400Hz sampling with low jitter to meet AHRS accuracy requirements. I2C-based sensors (MPU6050) have jitter concerns due to clock stretching.
- Constraints:
  - Embedded environment: No heap allocation, static buffers only
  - Real-time: 400Hz sampling must not be interrupted
  - Resource-limited: RP2040 has no FPU, limited CPU cycles
  - Platform abstraction: Must use existing SPI/I2C/GPIO traits
  - Embassy async: Must integrate with Embassy runtime
- Related ADRs:
  - [ADR-00006-imu-driver-architecture](../../adr/ADR-00006-imu-driver-architecture.md): Core architecture decision selecting BMI088 + SPI + interrupt
  - [ADR-00001-ahrs-algorithm-selection](../../adr/ADR-00001-ahrs-algorithm-selection.md): AHRS algorithm requiring 400Hz IMU data
  - [ADR-00003-platform-abstraction](../../adr/ADR-00003-platform-abstraction.md): Platform abstraction layer design

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                     AHRS Task (100Hz)                        │
│                  (DCM Algorithm Integration)                 │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       │ ImuSensor trait
                       │ (read_gyro, read_accel)
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                   Bmi088Driver (SPI)                         │
│  ┌──────────────────────┐  ┌───────────────────────────┐    │
│  │   Gyro Subsystem     │  │   Accel Subsystem         │    │
│  │  - INT1 (data-ready) │  │  - INT2 (data-ready)      │    │
│  │  - Range: ±2000°/s   │  │  - Range: ±24g            │    │
│  │  - ODR: 400Hz        │  │  - ODR: 400Hz             │    │
│  └──────────┬───────────┘  └────────┬──────────────────┘    │
│             │                       │                        │
│             └───────┬───────────────┘                        │
│                     │                                        │
└─────────────────────┼────────────────────────────────────────┘
                      │
                      │ SpiInterface trait
                      │ (platform abstraction)
                      ▼
┌─────────────────────────────────────────────────────────────┐
│             Platform SPI Implementation                      │
│              (RP2040 / RP2350 HAL)                           │
└─────────────────────────────────────────────────────────────┘
```

### Components

- **`ImuSensor` trait** (`src/devices/traits/imu.rs`):
  - Device-independent IMU interface for AHRS
  - Methods: `read_gyro()`, `read_accel()`, `read_mag()`, `configure()`, `self_test()`
  - Returns `Result<Vector3<f32>, ImuError>` for each axis
  - Enables mock implementations for testing
- **`Bmi088Driver` struct** (`src/devices/imu/bmi088/driver.rs`):
  - BMI088-specific implementation of `ImuSensor` trait
  - Manages separate gyro and accel chip select pins
  - Handles interrupt-driven data-ready events
  - Implements error retry logic with exponential backoff
  - Applies calibration data from parameter system
- **`Bmi088Config` struct** (`src/devices/imu/bmi088/config.rs`):
  - Configuration structure for gyro and accel ranges, ODR, FIFO settings
  - Default: ±2000°/s gyro, ±24g accel, 400Hz ODR
- **Register definitions** (`src/devices/imu/bmi088/registers.rs`):
  - BMI088 register addresses and bit masks
  - Separate constants for gyro and accel (dual datasheets)
- **`MockImu` struct** (`src/devices/imu/mock.rs`):
  - Mock implementation of `ImuSensor` trait for unit testing
  - Returns configurable static or dynamic sensor data
  - Enables AHRS testing without hardware

### Data Flow

1. **Initialization**:
   - Platform initializes SPI bus via `SpiInterface` trait
   - `Bmi088Driver::new()` configures gyro and accel (ODR, range, filters)
   - Driver verifies chip IDs (0x0F gyro, 0x1E accel)
   - Interrupt pins configured for data-ready (rising edge, high priority)
   - Calibration data loaded from parameter system
2. **Sampling Loop (400Hz)**:
   - Hardware generates data-ready interrupt on INT1 (gyro) or INT2 (accel)
   - Interrupt handler wakes IMU task
   - Driver reads gyro/accel via SPI (6-byte burst read per sensor)
   - Raw ADC values converted to physical units (rad/s, m/s²)
   - Calibration applied (gyro bias, accel offset/scale)
   - Data returned to AHRS via `ImuSensor` trait
3. **Error Handling**:
   - SPI errors trigger retry with exponential backoff (max 3 retries)
   - Persistent failures return `ImuError` to AHRS
   - AHRS degrades gracefully (gyro-only mode if accel fails)

### Data Models and Types

```rust
/// Device-independent IMU interface
pub trait ImuSensor {
    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError>;
    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError>;
    async fn read_mag(&mut self) -> Result<Option<Vector3<f32>>, ImuError>;
    async fn configure(&mut self, config: ImuConfig) -> Result<(), ImuError>;
    async fn self_test(&mut self) -> Result<SelfTestResult, ImuError>;
}

/// BMI088 driver configuration
pub struct Bmi088Config {
    pub gyro_range: GyroRange,      // ±125, ±250, ±500, ±1000, ±2000 °/s
    pub gyro_odr: GyroOdr,          // 100, 200, 400, 1000, 2000 Hz
    pub accel_range: AccelRange,    // ±3, ±6, ±12, ±24 g
    pub accel_odr: AccelOdr,        // 12.5, 25, 50, 100, 200, 400, 800, 1600 Hz
    pub enable_fifo: bool,
}

/// IMU error types
pub enum ImuError {
    SpiError(SpiError),
    InvalidData,
    SelfTestFailed,
    ConfigurationError,
    TimeoutError,
}

/// Calibration data from parameter system
pub struct CalibrationData {
    pub gyro_bias: Vector3<f32>,     // rad/s
    pub accel_offset: Vector3<f32>,  // m/s²
    pub accel_scale: Vector3<f32>,   // scale factors
}
```

### Error Handling

- English error messages for all failure modes
- `ImuError` enum provides type-safe error representation
- Retry logic for transient SPI errors:
  - Max 3 retries with exponential backoff (100µs, 200µs, 400µs)
  - Return error to AHRS after max retries
- Sensor reset on persistent failure (> 10 consecutive errors)
- AHRS implements graceful degradation:
  - Gyro-only mode if accelerometer fails
  - Dead-reckoning if all sensors fail

### Security Considerations

- No security risks: IMU data is not sensitive
- No network exposure
- No authentication/encryption required
- Error logging does not contain PII

### Performance Considerations

- **Hot paths**:
  - SPI read: < 0.3ms per sensor (1 MHz SPI clock)
  - Data conversion: < 0.1ms (fixed-point math on RP2040, FPU on RP2350)
  - Calibration application: < 0.05ms (3 multiplies + 3 adds)
- **Caching strategy**:
  - Configuration cached in driver struct (no repeated SPI writes)
  - Calibration data loaded once at initialization
- **Async/concurrency**:
  - Embassy async tasks for interrupt-driven sampling
  - No blocking waits in critical path
- **I/O**:
  - SPI burst reads minimize transaction overhead
  - Interrupt-driven (no polling waste)
- **Progress indicators**:
  - Not applicable for embedded sensor driver

### Platform Considerations

#### RP2040 (Pico W)

- No hardware FPU: Use software floating-point (slower)
- Clock speed: 133 MHz (sufficient for 400Hz with optimization)
- SPI speed: 1-2 MHz to stay within timing budget
- DMA: Use DMA for SPI reads to reduce CPU overhead (deferred to Phase 2)
- Memory: 264KB RAM, allocate static 1KB buffer for future FIFO support

#### RP2350 (Pico 2 W)

- Hardware FPU: Faster data conversion (3-5x speedup)
- Clock speed: 150 MHz (more headroom for other tasks)
- SPI speed: 4-8 MHz safe
- Memory: 520KB RAM (no constraints)

#### Cross-Platform Compatibility

- Use platform traits (`SpiInterface`, `GpioInterface`) for portability
- Avoid platform-specific HAL calls in driver code
- Feature flags for platform-specific optimizations

## Alternatives Considered

Alternatives were extensively evaluated in ADR-00006-imu-driver-architecture. Summary:

1. **MPU9250 + SPI + Interrupt**
   - Pros: Integrated magnetometer, good performance
   - Cons: EOL product (discontinued 2020), uncertain availability
2. **MPU6050 + I2C + Polling**
   - Pros: Low cost, simple driver
   - Cons: I2C jitter risk (0.5-2ms on RP2040), fails NFR-00002
3. **Multiple IMU Redundancy**
   - Pros: Fault tolerance
   - Cons: Excessive complexity for rover/boat, not required

Decision Rationale: BMI088 is the only option guaranteeing <1ms jitter via SPI and hardware interrupts. Proven in ArduPilot/PX4, providing 4x margin (1600Hz ODR) over 400Hz requirement.

## Migration and Compatibility

- Backward/forward compatibility: Not applicable (initial implementation)
- Rollout plan: Single-phase deployment after validation
- Deprecation plan: Not applicable

## Testing Strategy

### Unit Tests

- Test `Bmi088Config` default values and validation
- Test register address constants
- Test data conversion (raw ADC → physical units)
- Test calibration application (bias, offset, scale)
- Test `MockImu` implementation for AHRS testing

### Integration Tests

- Test SPI communication with real BMI088 hardware
- Test interrupt-driven sampling timing (400Hz ± 5Hz)
- Test jitter measurement (standard deviation < 1ms)
- Test error retry logic (simulate SPI failures)
- Test AHRS integration (±2° roll/pitch accuracy)

### External API Parsing (if applicable)

Not applicable (embedded sensor driver, no external APIs)

### Performance & Benchmarks (if applicable)

- Benchmark SPI read latency (target: < 0.3ms)
- Benchmark data conversion time (target: < 0.1ms on RP2040)
- Benchmark CPU overhead (target: < 10% @ 133MHz)
- Measure jitter distribution over 10-second window

## Documentation Impact

- Add module documentation for `src/devices/imu/bmi088/`
- Document `ImuSensor` trait in `src/devices/traits/imu.rs`
- Update architecture documentation with IMU driver design
- Add wiring diagram for BMI088 connections (SPI, CS, interrupts)

## External References

- [BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
- [BMI088 Application Note](https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bmi088-an000.pdf)
- [embassy-rp SPI Examples](https://github.com/embassy-rs/embassy/tree/main/examples/rp)
- [ArduPilot IMU Backend](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_InertialSensor)

## Open Questions

- [ ] Should we implement hardware FIFO buffering for robustness? → Next step: Prototype FIFO mode in Phase 2, measure benefits
- [ ] What DMA strategy is optimal for RP2040? → Method: Benchmark DMA vs interrupt-driven SPI in Phase 2
- [ ] Do we need gyro temperature compensation? → Next step: Measure temperature drift on hardware, add if > 0.01°/s/°C
