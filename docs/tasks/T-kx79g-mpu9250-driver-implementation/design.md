# T-kx79g MPU-9250 I2C Driver Implementation

## Metadata

- Type: Design
- Status: Cancelled

## Links

- Associated Plan Document:
  - [T-kx79g-mpu9250-driver-implementation-plan](./plan.md)

## Overview

This design implements an MPU-9250 9-axis IMU driver using I2C communication for the pico_trail autopilot. The driver provides calibrated sensor data through an abstracted `ImuSensor` trait, enabling the EKF AHRS subsystem to consume gyroscope, accelerometer, and magnetometer readings without hardware-specific knowledge. The implementation supports both RP2040 and RP2350 platforms via Embassy async runtime.

## Success Metrics

- [ ] MPU-9250 initialization with WHO_AM_I verification (0x71)
- [ ] AK8963 magnetometer access via bypass mode (WHO_AM_I: 0x48)
- [ ] Full 9-axis read latency < 1.5ms (95th percentile)
- [ ] Sustained 400Hz sampling rate with < 1 error per 1000 samples
- [ ] All unit tests pass on host; embedded build succeeds on RP2350

## Background and Current State

- Context: AHRS subsystem requires IMU sensor data for attitude estimation; GPS and other sensors already use similar driver patterns
- Current behavior: No physical IMU driver exists; `ImuData` interface defined but not implemented
- Pain points: BMI088 task (T-qwvco) was planned but MPU-9250 selected per AN-t47be analysis
- Constraints: no_std, no heap allocation, Embassy async runtime, I2C @ 400kHz
- Related ADRs: [ADR-t5cq4-mpu9250-i2c-driver-architecture](../../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────┐
│                    EKF AHRS Task                        │
│                         │                               │
│                         ▼                               │
│              ┌─────────────────────┐                    │
│              │   ImuSensor trait   │                    │
│              │  - read_all()       │                    │
│              │  - read_gyro()      │                    │
│              │  - read_accel()     │                    │
│              │  - read_mag()       │                    │
│              │  - set_calibration()│                    │
│              │  - is_healthy()     │                    │
│              └──────────┬──────────┘                    │
│                         │                               │
│         ┌───────────────┴───────────────┐               │
│         ▼                               ▼               │
│ ┌───────────────────┐       ┌───────────────────┐       │
│ │  Mpu9250Driver    │       │     MockImu       │       │
│ │  (Production)     │       │    (Testing)      │       │
│ └─────────┬─────────┘       └───────────────────┘       │
│           │                                             │
│           ▼                                             │
│ ┌─────────────────────────────────────────────┐         │
│ │              I2C Bus (400kHz)               │         │
│ │                                             │         │
│ │   ┌────────────┐      ┌────────────────┐    │         │
│ │   │  MPU-9250  │      │    AK8963      │    │         │
│ │   │  0x68      │      │    0x0C        │    │         │
│ │   │ Gyro+Accel │      │ Magnetometer   │    │         │
│ │   └────────────┘      └────────────────┘    │         │
│ └─────────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────┘
```

### Components

**`src/devices/traits/imu.rs`**:

- `ImuSensor` trait: Device-independent interface for EKF
- `ImuReading`: Combined 9-axis data with timestamp
- `ImuCalibration`: Offset/scale correction parameters
- `ImuError`: Error enumeration for failure modes

**`src/devices/imu/mpu9250/`**:

- `mod.rs`: Public API and driver struct
- `registers.rs`: MPU-9250 and AK8963 register definitions
- `config.rs`: Configuration structs (ranges, DLPF, sample rate)
- `driver.rs`: Core I2C communication and data conversion

**`src/devices/imu/mock.rs`**:

- `MockImu`: Test implementation with preset readings

### Data Flow

1. **Initialization**:
   - Verify MPU-9250 WHO_AM_I (0x71)
   - Reset device, wait 100ms
   - Configure gyro/accel ranges and DLPF
   - Enable I2C bypass for AK8963
   - Verify AK8963 WHO_AM_I (0x48)
   - Configure magnetometer for 100Hz continuous

2. **Read Cycle (400Hz)**:
   - Read 14 bytes from MPU-9250 (accel + temp + gyro)
   - Read 7 bytes from AK8963 (mag + status)
   - Convert raw values to SI units
   - Apply calibration corrections
   - Return `ImuReading` with timestamp

3. **Calibration**:
   - Load from parameter system on init
   - Apply gyro bias subtraction
   - Apply accel offset/scale corrections
   - Apply mag hard/soft iron corrections

### Data Models and Types

```rust
/// Combined IMU reading with calibration applied
pub struct ImuReading {
    pub gyro: [f32; 3],        // rad/s, body frame
    pub accel: [f32; 3],       // m/s², body frame
    pub mag: [f32; 3],         // µT, body frame
    pub temperature: f32,      // °C
    pub timestamp_us: u64,     // Microseconds since boot
}

/// Calibration parameters
pub struct ImuCalibration {
    pub gyro_bias: [f32; 3],        // rad/s offset
    pub accel_offset: [f32; 3],     // m/s² offset
    pub accel_scale: [f32; 3],      // Scale factors
    pub mag_offset: [f32; 3],       // µT hard iron
    pub mag_scale: [[f32; 3]; 3],   // Soft iron matrix
}

/// Driver configuration
pub struct Mpu9250Config {
    pub gyro_range: GyroRange,      // ±250/500/1000/2000 °/s
    pub accel_range: AccelRange,    // ±2/4/8/16 g
    pub gyro_dlpf: DlpfConfig,      // Low-pass filter bandwidth
    pub accel_dlpf: DlpfConfig,
    pub mag_mode: MagMode,          // Continuous 8Hz/100Hz
}

/// Error types
pub enum ImuError {
    I2cError,
    InvalidData,
    NotInitialized,
    SelfTestFailed,
    MagNotReady,
}
```

### Error Handling

- **I2C errors**: Retry up to 3 times, log warnings, mark sensor unhealthy after failures
- **Invalid data**: Detect stuck values (same reading repeated), mark as `InvalidData`
- **Initialization failures**: Log error, prevent reads, return `NotInitialized`
- **Magnetometer not ready**: Return cached value or skip mag in reading

### Security Considerations

- N/A - No external input processing; I2C communication is internal

### Performance Considerations

- **I2C Latency**: Target < 1.5ms for full read; use async to avoid blocking
- **CPU Overhead**: < 15% on RP2040 @ 133MHz for 400Hz task
- **Memory**: Fixed buffers, no heap allocation
- **Calibration**: Applied inline during data conversion (minimal overhead)

### Platform Considerations

#### RP2040 (Pico W)

- No FPU: Software floating-point for unit conversion
- I2C via `embassy_rp::i2c::I2c` async driver
- May need DMA if I2C blocking is excessive (defer)

#### RP2350 (Pico 2 W)

- Hardware FPU: Faster unit conversion
- Same I2C driver interface
- More CPU headroom for 400Hz

#### Filesystem

- N/A

## Alternatives Considered

1. **SPI Interface**
   - Pros: Lower latency (\~0.3ms vs 1.5ms)
   - Cons: More wiring (6+ pins), added complexity
   - Decision: I2C adequate for 400Hz per ADR decision

2. **Direct HAL Access (no trait)**
   - Pros: Slightly less overhead
   - Cons: No mock testing, coupled to hardware
   - Decision: Trait enables testing and future sensor swaps

Decision Rationale:

- I2C simplicity outweighs SPI latency benefits for 400Hz
- Trait abstraction essential for unit testing without hardware

## Migration and Compatibility

- Backward/forward compatibility: New module, no existing code affected
- Rollout plan: N/A - new feature
- Deprecation plan: T-qwvco (BMI088 task) superseded by this task

## Testing Strategy

### Unit Tests

- Test configuration struct defaults
- Test unit conversion math (raw to SI units)
- Test calibration application
- Test `MockImu` behavior

### Integration Tests

- Test with real I2C on hardware (manual verification)
- Test 400Hz sampling stability over extended period

### External API Parsing (if applicable)

- N/A

### Performance & Benchmarks (if applicable)

- Measure I2C read latency statistics
- Profile CPU usage during 400Hz operation

## Documentation Impact

- Add IMU driver documentation to `docs/architecture.md`
- Update module comments with usage examples

## External References

- [MPU-9250 Register Map (RM-MPU-9250A-00)](https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf)
- [MPU-9250 Datasheet (PS-MPU-9250A-01)](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [AK8963 Datasheet](https://www.akm.com/content/dam/documents/products/electronic-compass/ak8963c/ak8963c-en-datasheet.pdf)

## Open Questions

- [ ] Should we implement I2C DMA for lower CPU overhead? → Method: Benchmark CPU usage, add DMA if >20%
- [ ] Is bypass mode or master mode better for AK8963 access? → Decision: Start with bypass (simpler)

## Appendix

### Register Map Summary

| Register     | Address | Description          |
| ------------ | ------- | -------------------- |
| WHO_AM_I     | 0x75    | Device ID (0x71)     |
| PWR_MGMT_1   | 0x6B    | Power management     |
| CONFIG       | 0x1A    | DLPF configuration   |
| GYRO_CONFIG  | 0x1B    | Gyro range           |
| ACCEL_CONFIG | 0x1C    | Accel range          |
| INT_PIN_CFG  | 0x37    | I2C bypass enable    |
| ACCEL_XOUT_H | 0x3B    | Start of sensor data |

### Unit Conversion Formulas

```rust
// Gyro: ±2000°/s full scale, 16-bit signed
let gyro_scale = (2000.0 / 32768.0) * (PI / 180.0); // deg/s to rad/s

// Accel: ±8g full scale, 16-bit signed
let accel_scale = (8.0 / 32768.0) * 9.80665; // g to m/s²

// Mag: 16-bit mode, 4912 µT full scale
let mag_scale = 4912.0 / 32760.0; // µT
```

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
