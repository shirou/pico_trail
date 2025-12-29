# FR-slm3x ICM-20948 I2C Driver

## Metadata

- Type: Functional Requirement
- Status: Abandoned

## Abandonment Note

**Reason**: ICM-20948 hardware was counterfeit - magnetometer (AK09916) not functional.

The requirement is abandoned because the purchased ICM-20948 turned out to be counterfeit. The magnetometer does not work, making 9-axis functionality unusable. Source code in `src/devices/imu/icm20948/` is retained for potential future use with genuine hardware.

## Links

- Related Analyses:
  - [AN-t47be-mpu9250-imu-and-ekf-integration](../analysis/AN-t47be-mpu9250-imu-and-ekf-integration.md)
- Prerequisite Requirements:
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
  - [NFR-3wlo1-imu-sampling-rate](NFR-3wlo1-imu-sampling-rate.md)
- Dependent Requirements:
  - [FR-z1fdo-imu-sensor-trait](FR-z1fdo-imu-sensor-trait.md)
  - [FR-3f2cn-quaternion-ekf-ahrs](FR-3f2cn-quaternion-ekf-ahrs.md)
  - [FR-e2urj-large-vehicle-magcal](FR-e2urj-large-vehicle-magcal.md)
  - [NFR-ulsja-imu-i2c-read-latency](NFR-ulsja-imu-i2c-read-latency.md)
  - [NFR-wkrr5-imu-read-reliability](NFR-wkrr5-imu-read-reliability.md)
- Related ADRs:
  - [ADR-t5cq4-mpu9250-i2c-driver-architecture](../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)
- Related Tasks:
  - [T-0kbo4-icm20948-driver-implementation](../tasks/T-0kbo4-icm20948-driver-implementation/README.md)

## Requirement Statement

The system shall implement an ICM-20948 driver using I2C communication at 400kHz to provide 9-axis sensor data (gyroscope, accelerometer, magnetometer) for the EKF AHRS subsystem.

## Rationale

The ICM-20948 is TDK InvenSense's successor to the discontinued MPU-9250 and provides integrated 9-axis sensing (3-axis gyro, 3-axis accel, 3-axis mag via AK09916):

- **Active Production**: ICM-20948 is actively manufactured, unlike EOL MPU-9250
- **Integration**: Single sensor simplifies hardware, calibration, and mounting
- **I2C Simplicity**: 4-wire connection (VCC, GND, SDA, SCL) reduces wiring complexity
- **Availability**: Widely available on breakout boards (Adafruit, SparkFun)
- **Adequate Performance**: I2C at 400kHz sufficient for 400Hz sampling with \~1.5ms read latency

## User Story (if applicable)

As an autopilot system, I want to read 9-axis IMU data from the ICM-20948 sensor via I2C, so that the EKF can estimate accurate vehicle attitude.

## Acceptance Criteria

- [ ] ICM-20948 driver initializes successfully with WHO_AM_I verification (0xEA)
- [ ] Register bank switching implemented for accessing all 4 banks
- [ ] AK09916 magnetometer accessible via I2C bypass mode with WHO_AM_I verification (0x09)
- [ ] Gyroscope data readable at ±2000°/s full scale, converted to rad/s
- [ ] Accelerometer data readable at ±8g full scale, converted to m/s²
- [ ] Magnetometer data readable at 100Hz continuous mode, converted to µT
- [ ] All 9 axes read in single transaction sequence within 1.5ms
- [ ] Temperature data readable and convertible to °C
- [ ] Driver works on both RP2040 and RP2350 platforms
- [ ] Self-test function validates sensor health on startup
- [ ] Driver implements `ImuSensor` trait (FR-z1fdo)

## Technical Details (if applicable)

### Functional Requirement Details

**I2C Addresses:**

- ICM-20948: 0x69 (AD0 low, default on most breakouts) or 0x68 (AD0 high)
- AK09916: 0x0C (accessible via bypass mode)

**Register Banks:**

The ICM-20948 uses a 4-bank register architecture:

- Bank 0: User Bank (sensor data, config)
- Bank 1: Self-test data
- Bank 2: Sensor configuration
- Bank 3: I2C master configuration

```rust
// Key registers (Bank 0)
const WHO_AM_I: u8 = 0x00;          // Expected: 0xEA
const USER_CTRL: u8 = 0x03;         // User control
const PWR_MGMT_1: u8 = 0x06;        // Power management
const PWR_MGMT_2: u8 = 0x07;        // Power management 2
const INT_PIN_CFG: u8 = 0x0F;       // Bypass enable for AK09916
const ACCEL_XOUT_H: u8 = 0x2D;      // Start of accel data
const GYRO_XOUT_H: u8 = 0x33;       // Start of gyro data
const TEMP_OUT_H: u8 = 0x39;        // Temperature data
const REG_BANK_SEL: u8 = 0x7F;      // Bank select (all banks)

// Bank 2 registers
const GYRO_CONFIG_1: u8 = 0x01;     // Gyro full scale
const ACCEL_CONFIG: u8 = 0x14;      // Accel full scale
```

**Default Configuration:**

- Gyro range: ±2000°/s (for high-rate maneuvers)
- Accel range: ±8g (covers typical rover/boat accelerations)
- DLPF: 184Hz bandwidth (adequate anti-aliasing for 400Hz)
- Sample rate: 1.125kHz internal ODR, externally sampled at 400Hz
- Mag mode: Continuous 100Hz

**Data Read Sequence:**

1. Select Bank 0 for sensor data
2. Read 12 bytes: accel (6) + gyro (6)
3. Read 2 bytes: temperature
4. Read magnetometer via bypass mode (8 bytes including status)

**Unit Conversions:**

```rust
// Gyro: LSB to rad/s
let gyro_scale = (2000.0 / 32768.0) * (PI / 180.0); // deg/s to rad/s

// Accel: LSB to m/s²
let accel_scale = (8.0 / 32768.0) * 9.80665; // g to m/s²

// Mag: LSB to µT (AK09916)
let mag_scale = 4912.0 / 32752.0; // 16-bit mode, µT
```

## Platform Considerations

### Pico W (RP2040)

- I2C via `embassy_rp::i2c::I2c`
- 400kHz I2C clock (Fast Mode)
- No FPU: Software floating-point for unit conversions
- May need DMA for lower CPU overhead if I2C blocking is too long

### Pico 2 W (RP2350)

- Same I2C driver interface
- Hardware FPU accelerates unit conversions
- More headroom for I2C transaction timing

### Cross-Platform

- Use `embedded_hal_async::i2c::I2c` trait for portability
- Same driver code works on both platforms
- Platform-specific I2C initialization in `src/platform/`

## Risks & Mitigation

| Risk                             | Impact | Likelihood | Mitigation                                        | Validation                            |
| -------------------------------- | ------ | ---------- | ------------------------------------------------- | ------------------------------------- |
| I2C communication errors         | High   | Medium     | Implement retry logic (3 attempts), error logging | Test with long cable, EMI environment |
| ICM-20948 sensor stuck           | High   | Low        | Implement watchdog, check for stuck values        | Monitor sensor variance over time     |
| AK09916 data not ready           | Medium | Medium     | Check DRDY status bit before reading              | Log mag not-ready events              |
| Register bank switching overhead | Low    | Medium     | Cache current bank, minimize bank switches        | Profile I2C transaction times         |
| VDDIO voltage mismatch           | Medium | Low        | Use breakout boards with level shifters           | Verify with oscilloscope              |

## Implementation Notes

**Module Structure:**

```
src/devices/imu/
├── mod.rs              # Public exports
├── icm20948/
│   ├── mod.rs          # Driver public API
│   ├── registers.rs    # Register definitions (all 4 banks)
│   ├── config.rs       # Configuration structs
│   └── driver.rs       # Core driver implementation
```

**Initialization Sequence:**

1. Verify WHO_AM_I (0xEA)
2. Reset device (PWR_MGMT_1 = 0x80), wait 100ms
3. Wake device, auto-select clock (PWR_MGMT_1 = 0x01)
4. Select Bank 2, configure gyro/accel ranges and DLPF
5. Select Bank 0, enable I2C bypass for AK09916 (INT_PIN_CFG = 0x02)
6. Verify AK09916 WHO_AM_I (0x09)
7. Configure AK09916 for continuous 100Hz mode
8. Run self-test and log results

**Error Handling:**

- Transient I2C errors: Retry up to 3 times
- Persistent errors: Mark sensor unhealthy, log error
- Invalid data (stuck at same value): Detect and report
- Bank switch errors: Reset and retry

## External References

- [ICM-20948 Datasheet](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000189-icm-20948-v1.5.pdf)
- [ICM-20948 Product Page](https://invensense.tdk.com/products/motion-tracking/9-axis/icm-20948/)
- [Adafruit ICM-20948 Guide](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-tdk-invensense-icm-20948-9-dof-imu.pdf)
- [SparkFun ICM-20948 Breakout](https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
