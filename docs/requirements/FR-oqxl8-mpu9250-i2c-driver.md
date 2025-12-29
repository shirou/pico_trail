# FR-oqxl8 MPU-9250 I2C Driver

## Metadata

- Type: Functional Requirement
- Status: Deferred

## Change History

- **2025-01-XX**: Status changed to Deferred. MPU-9250 hardware unavailable; ICM-20948 is now primary sensor. Implementation retained for users with existing MPU-9250 hardware.

## Links

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
  - [T-kx79g-mpu9250-driver-implementation](../tasks/T-kx79g-mpu9250-driver-implementation/README.md)

## Requirement Statement

The system shall implement an MPU-9250 driver using I2C communication at 400kHz to provide 9-axis sensor data (gyroscope, accelerometer, magnetometer) for the EKF AHRS subsystem.

## Rationale

The MPU-9250 provides integrated 9-axis sensing (3-axis gyro, 3-axis accel, 3-axis mag via AK8963) in a single package:

- **Integration**: Single sensor simplifies hardware, calibration, and mounting
- **I2C Simplicity**: 4-wire connection (VCC, GND, SDA, SCL) reduces wiring complexity
- **Availability**: Widely available on breakout boards for prototyping
- **Adequate Performance**: I2C at 400kHz sufficient for 400Hz sampling with \~1.5ms read latency

## User Story (if applicable)

As an autopilot system, I want to read 9-axis IMU data from the MPU-9250 sensor via I2C, so that the EKF can estimate accurate vehicle attitude.

## Acceptance Criteria

- [ ] MPU-9250 driver initializes successfully with WHO_AM_I verification (0x71)
- [ ] AK8963 magnetometer accessible via I2C bypass mode with WHO_AM_I verification (0x48)
- [ ] Gyroscope data readable at ±2000°/s full scale, converted to rad/s
- [ ] Accelerometer data readable at ±8g full scale, converted to m/s²
- [ ] Magnetometer data readable at 100Hz continuous mode, converted to µT
- [ ] All 9 axes read in single transaction sequence within 1.5ms
- [ ] Temperature data readable and convertible to °C
- [ ] Driver works on both RP2040 and RP2350 platforms
- [ ] Self-test function validates sensor health on startup

## Technical Details (if applicable)

### Functional Requirement Details

**I2C Addresses:**

- MPU-9250: 0x68 (AD0 low) or 0x69 (AD0 high)
- AK8963: 0x0C (accessible via bypass mode)

**Register Configuration:**

```rust
// Key registers
const WHO_AM_I: u8 = 0x75;          // Expected: 0x71
const PWR_MGMT_1: u8 = 0x6B;        // Power management
const CONFIG: u8 = 0x1A;            // DLPF config
const GYRO_CONFIG: u8 = 0x1B;       // Gyro full scale
const ACCEL_CONFIG: u8 = 0x1C;      // Accel full scale
const INT_PIN_CFG: u8 = 0x37;       // Bypass enable for AK8963
const ACCEL_XOUT_H: u8 = 0x3B;      // Start of sensor data (14 bytes)
```

**Default Configuration:**

- Gyro range: ±2000°/s (for high-rate maneuvers)
- Accel range: ±8g (covers typical rover/boat accelerations)
- DLPF: 184Hz bandwidth (adequate anti-aliasing for 400Hz)
- Sample rate: 1kHz internal ODR, externally sampled at 400Hz
- Mag mode: Continuous 100Hz

**Data Read Sequence:**

1. Read 14 bytes from MPU-9250 starting at ACCEL_XOUT_H
   - Accel X/Y/Z (6 bytes), Temp (2 bytes), Gyro X/Y/Z (6 bytes)
2. Read 7 bytes from AK8963 starting at HXL
   - Mag X/Y/Z (6 bytes), Status2 (1 byte for overflow check)

**Unit Conversions:**

```rust
// Gyro: LSB to rad/s
let gyro_scale = (2000.0 / 32768.0) * (PI / 180.0); // deg/s to rad/s

// Accel: LSB to m/s²
let accel_scale = (8.0 / 32768.0) * 9.80665; // g to m/s²

// Mag: LSB to µT
let mag_scale = 4912.0 / 32760.0; // 16-bit mode, µT
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

| Risk                        | Impact | Likelihood | Mitigation                                        | Validation                            |
| --------------------------- | ------ | ---------- | ------------------------------------------------- | ------------------------------------- |
| I2C communication errors    | High   | Medium     | Implement retry logic (3 attempts), error logging | Test with long cable, EMI environment |
| MPU-9250 sensor stuck       | High   | Low        | Implement watchdog, check for stuck values        | Monitor sensor variance over time     |
| AK8963 data not ready       | Medium | Medium     | Check DRDY status bit before reading              | Log mag not-ready events              |
| I2C bus contention with GPS | Medium | Medium     | Use dedicated I2C bus or careful timing           | Test with GPS on same bus             |

## Implementation Notes

**Module Structure:**

```
src/devices/imu/
├── mod.rs              # Public exports
├── mpu9250/
│   ├── mod.rs          # Driver public API
│   ├── registers.rs    # Register definitions
│   ├── config.rs       # Configuration structs
│   └── driver.rs       # Core driver implementation
```

**Initialization Sequence:**

1. Verify WHO_AM_I (0x71)
2. Reset device (PWR_MGMT_1 = 0x80), wait 100ms
3. Wake device (PWR_MGMT_1 = 0x00)
4. Configure gyro/accel ranges and DLPF
5. Enable I2C bypass for AK8963 (INT_PIN_CFG = 0x02)
6. Verify AK8963 WHO_AM_I (0x48)
7. Configure AK8963 for continuous 100Hz mode
8. Run self-test and log results

**Error Handling:**

- Transient I2C errors: Retry up to 3 times
- Persistent errors: Mark sensor unhealthy, log error
- Invalid data (stuck at same value): Detect and report

## External References

- [MPU-9250 Register Map (RM-MPU-9250A-00)](https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf)
- [MPU-9250 Datasheet (PS-MPU-9250A-01)](https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [AK8963 Datasheet](https://www.akm.com/content/dam/documents/products/electronic-compass/ak8963c/ak8963c-en-datasheet.pdf)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
