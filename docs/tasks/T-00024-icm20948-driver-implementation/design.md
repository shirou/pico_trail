# T-00024 ICM-20948 Driver Implementation - Design

## Metadata

- Type: Task Design
- Status: Cancelled

## Links

- Parent Task:
  - [T-00024-icm20948-driver-implementation](./README.md)
- Associated Plan:
  - [T-00024-icm20948-driver-implementation-plan](./plan.md)

## Overview

This document describes the design for implementing the ICM-20948 9-axis IMU driver. The driver will provide I2C communication with the ICM-20948 sensor and implement the `ImuSensor` trait for integration with the EKF AHRS subsystem.

## Architecture

### Module Structure

```
src/devices/imu/
├── mod.rs                  # Public exports (add icm20948)
├── icm20948/
│   ├── mod.rs              # Driver public API
│   ├── registers.rs        # Register definitions (all 4 banks)
│   ├── config.rs           # Configuration structs
│   └── driver.rs           # Core driver implementation
├── mpu9250/                # Existing MPU-9250 (retained)
│   └── ...
└── mock.rs                 # Mock IMU for testing
```

### Component Diagram

```
┌──────────────────────────────────────────────────────────┐
│                      EKF AHRS                            │
│                  (consumes ImuSensor)                    │
└─────────────────────────┬────────────────────────────────┘
                          │ ImuSensor trait
                          ▼
┌──────────────────────────────────────────────────────────┐
│                   Icm20948Driver                         │
│  ┌──────────────┬───────────────┬──────────────────────┐│
│  │ Bank Manager │ Sensor Reader │ Calibration Applier  ││
│  └──────┬───────┴───────┬───────┴──────────┬───────────┘│
└─────────┼───────────────┼──────────────────┼────────────┘
          │               │                  │
          ▼               ▼                  ▼
┌──────────────────────────────────────────────────────────┐
│                    I2C Bus                               │
└─────────────────────────┬────────────────────────────────┘
                          │
          ┌───────────────┴───────────────┐
          ▼                               ▼
    ┌──────────┐                   ┌───────────┐
    │ICM-20948 │                   │  AK09916  │
    │  (0x69)  │                   │  (0x0C)   │
    └──────────┘                   └───────────┘
```

## Data Structures

### Configuration

```rust
/// ICM-20948 configuration
pub struct Icm20948Config {
    /// I2C address (0x68 or 0x69)
    pub address: u8,
    /// Gyroscope full scale range
    pub gyro_range: GyroRange,
    /// Accelerometer full scale range
    pub accel_range: AccelRange,
    /// Gyroscope digital low-pass filter
    pub gyro_dlpf: DlpfConfig,
    /// Accelerometer digital low-pass filter
    pub accel_dlpf: DlpfConfig,
    /// Sample rate divider (ODR = 1125Hz / (1 + div))
    pub sample_rate_div: u8,
    /// Magnetometer mode
    pub mag_mode: MagMode,
}

impl Default for Icm20948Config {
    fn default() -> Self {
        Self {
            address: 0x69,  // Default on most breakouts
            gyro_range: GyroRange::Dps2000,
            accel_range: AccelRange::G8,
            gyro_dlpf: DlpfConfig::Bw184Hz,
            accel_dlpf: DlpfConfig::Bw184Hz,
            sample_rate_div: 0,     // 1125Hz ODR
            mag_mode: MagMode::Continuous100Hz,
        }
    }
}
```

### Register Banks

```rust
/// Register bank selection
#[repr(u8)]
pub enum RegisterBank {
    Bank0 = 0x00,  // User bank (sensor data, config)
    Bank1 = 0x10,  // Self-test data
    Bank2 = 0x20,  // Sensor configuration
    Bank3 = 0x30,  // I2C master configuration
}
```

### Driver State

```rust
/// ICM-20948 driver
pub struct Icm20948Driver<I2C> {
    i2c: I2C,
    address: u8,
    config: Icm20948Config,
    calibration: ImuCalibration,
    current_bank: RegisterBank,
    healthy: bool,
    error_count: u32,
}
```

## Key Algorithms

### Register Bank Switching

```rust
async fn select_bank(&mut self, bank: RegisterBank) -> Result<(), ImuError> {
    if self.current_bank != bank {
        // REG_BANK_SEL is at address 0x7F in all banks
        self.write_register(0x7F, bank as u8).await?;
        self.current_bank = bank;
    }
    Ok(())
}
```

### Sensor Data Read

```rust
async fn read_raw(&mut self) -> Result<RawData, ImuError> {
    // Ensure Bank 0 for sensor data
    self.select_bank(RegisterBank::Bank0).await?;

    // Read accel + gyro + temp (14 bytes starting at 0x2D)
    let mut buf = [0u8; 14];
    self.i2c.write_read(self.address, &[ACCEL_XOUT_H], &mut buf).await?;

    // Parse big-endian data
    let accel = Vector3::new(
        i16::from_be_bytes([buf[0], buf[1]]) as f32,
        i16::from_be_bytes([buf[2], buf[3]]) as f32,
        i16::from_be_bytes([buf[4], buf[5]]) as f32,
    );
    let gyro = Vector3::new(
        i16::from_be_bytes([buf[6], buf[7]]) as f32,
        i16::from_be_bytes([buf[8], buf[9]]) as f32,
        i16::from_be_bytes([buf[10], buf[11]]) as f32,
    );
    let temp_raw = i16::from_be_bytes([buf[12], buf[13]]);

    // Read magnetometer via bypass
    let mag = self.read_magnetometer().await?;

    Ok(RawData { accel, gyro, mag, temp_raw })
}
```

### Magnetometer Access (Bypass Mode)

```rust
async fn read_magnetometer(&mut self) -> Result<Vector3<f32>, ImuError> {
    // AK09916 is at 0x0C when bypass is enabled
    let mut buf = [0u8; 8];  // ST1, HXL, HXH, HYL, HYH, HZL, HZH, ST2
    self.i2c.write_read(AK09916_ADDR, &[AK09916_ST1], &mut buf).await?;

    // Check DRDY (bit 0 of ST1)
    if buf[0] & 0x01 == 0 {
        return Err(ImuError::MagNotReady);
    }

    // Check overflow (bit 3 of ST2)
    if buf[7] & 0x08 != 0 {
        return Err(ImuError::MagOverflow);
    }

    // Parse little-endian data (AK09916 uses little-endian)
    let mag = Vector3::new(
        i16::from_le_bytes([buf[1], buf[2]]) as f32,
        i16::from_le_bytes([buf[3], buf[4]]) as f32,
        i16::from_le_bytes([buf[5], buf[6]]) as f32,
    );

    Ok(mag)
}
```

## Initialization Sequence

1. **WHO_AM_I Check**: Read 0x00, expect 0xEA
2. **Reset**: Write 0x80 to PWR_MGMT_1, wait 100ms
3. **Wake Up**: Write 0x01 to PWR_MGMT_1 (auto-select best clock)
4. **Configure Gyro** (Bank 2):
   - Set GYRO_CONFIG_1 for range and DLPF
   - Set GYRO_SMPLRT_DIV for sample rate
5. **Configure Accel** (Bank 2):
   - Set ACCEL_CONFIG for range and DLPF
   - Set ACCEL_SMPLRT_DIV for sample rate
6. **Enable Bypass** (Bank 0):
   - Set INT_PIN_CFG bit 1 (BYPASS_EN)
7. **Configure AK09916**:
   - Read WHO_AM_I (0x01), expect 0x09
   - Set CNTL2 to continuous mode (0x08 for 100Hz)
8. **Self-Test**: Verify readings change when sensor is moved

## Hardware Calibration (User Bank Offsets)

The ICM-20948 supports hardware-level calibration offsets stored in User Bank registers. These offsets are applied by the sensor before data output, providing more efficient calibration than software-based corrections.

### Register Layout

| Axis    | Bank   | High Byte | Low Byte | Bits                           |
| ------- | ------ | --------- | -------- | ------------------------------ |
| Gyro X  | Bank 2 | 0x03      | 0x04     | 16-bit signed                  |
| Gyro Y  | Bank 2 | 0x05      | 0x06     | 16-bit signed                  |
| Gyro Z  | Bank 2 | 0x07      | 0x08     | 16-bit signed                  |
| Accel X | Bank 1 | 0x14      | 0x15     | 15-bit signed (bit 0 reserved) |
| Accel Y | Bank 1 | 0x17      | 0x18     | 15-bit signed (bit 0 reserved) |
| Accel Z | Bank 1 | 0x1A      | 0x1B     | 15-bit signed (bit 0 reserved) |

### Calibration API

```rust
/// Write gyroscope offset to hardware (Bank 2)
pub async fn set_gyro_offset(&mut self, offset: [i16; 3]) -> Result<(), ImuError>;

/// Read gyroscope offset from hardware (Bank 2)
pub async fn get_gyro_offset(&mut self) -> Result<[i16; 3], ImuError>;

/// Write accelerometer offset to hardware (Bank 1)
/// Note: 15-bit values, bit 0 of low byte is reserved
pub async fn set_accel_offset(&mut self, offset: [i16; 3]) -> Result<(), ImuError>;

/// Read accelerometer offset from hardware (Bank 1)
pub async fn get_accel_offset(&mut self) -> Result<[i16; 3], ImuError>;
```

### Usage Example

```rust
// Apply gyroscope calibration offsets (raw LSB units)
imu.set_gyro_offset([100, -50, 25]).await?;

// Apply accelerometer calibration offsets (15-bit, raw LSB units)
imu.set_accel_offset([512, -256, 128]).await?;

// Verify offsets were written correctly
let gyro_offset = imu.get_gyro_offset().await?;
let accel_offset = imu.get_accel_offset().await?;
```

### Notes

- Gyroscope offsets are 16-bit signed values in raw LSB units
- Accelerometer offsets are 15-bit signed values; bit 0 of the low byte is reserved and preserved during writes
- These hardware offsets are separate from software calibration (`ImuCalibration` struct)
- Hardware offsets are applied before data output, reducing CPU overhead

## Error Handling

- **I2C Errors**: Retry up to 3 times, then mark unhealthy
- **Bank Switch Errors**: Reset current_bank to unknown, force re-select
- **Magnetometer Not Ready**: Return cached value or skip (for rate limiting)
- **Invalid Data**: Detect stuck values (variance < threshold), mark unhealthy

## Testing Strategy

- **Unit Tests**: Mock I2C to test register sequences
- **Integration Tests**: Verify on hardware with known orientations
- **Performance Tests**: Measure I2C timing, verify 400Hz achievable

## Dependencies

- `embassy-rp`: I2C driver for RP2040/RP2350
- `embedded-hal-async`: I2C trait abstraction
- `nalgebra`: Vector3 for sensor data
