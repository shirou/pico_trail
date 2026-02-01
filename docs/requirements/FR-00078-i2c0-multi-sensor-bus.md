# FR-00078 I2C0 Multi-Sensor Bus Initialization for GPS and IMU

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-00021-gps-hardware-integration](../analysis/AN-00021-gps-hardware-integration.md)
- Related ADRs:
  - [ADR-00019-i2c0-gps-imu-integration](../adr/ADR-00019-i2c0-gps-imu-integration.md)
- Prerequisite Requirements: N/A – No prerequisite requirements
- Dependent Requirements:
  - [FR-00075-gps-i2c-driver](FR-00075-gps-i2c-driver.md)
- Related Tasks:
  - [T-00015-i2c0-gps-imu-integration](../tasks/T-00015-i2c0-gps-imu-integration/README.md)

## Requirement Statement

The system shall initialize I2C0 bus on GPIO 0 (SDA) and GPIO 1 (SCL) to enable communication with multiple I2C devices including NEO-M8N GPS module (address 0x42) and BNO085 IMU (address 0x4A).

## Rationale

The Freenove Standard board has limited GPIO pin availability (only GPIO 0, 1, 17, 22 available). I2C0 on GPIO 0 and 1 enables multi-sensor integration (GPS + IMU) on a shared bus, which is more efficient than dedicating separate GPIO pairs to each sensor. This approach maximizes pin utilization while supporting the sensors required for navigation (FR-00004-gps-waypoint-navigation).

## User Story (if applicable)

As an autopilot system, I want to initialize an I2C bus that can communicate with multiple sensors simultaneously, so that I can gather GPS position and IMU orientation data for navigation without exhausting available GPIO pins.

## Acceptance Criteria

- [ ] I2C0 peripheral initialized with GPIO 0 (SDA) and GPIO 1 (SCL) on both RP2040 and RP2350 platforms
- [ ] I2C0 configured for Fast Mode operation (400 kHz clock speed) or Standard Mode (100 kHz) with configurable clock speed
- [ ] Platform abstraction trait `I2cInterface` implemented in `src/platform/traits.rs` to support multi-platform I2C access
- [ ] I2C0 initialization code added to `src/platform/rp2350/i2c.rs` and `src/platform/rp2040/i2c.rs`
- [ ] External pull-up resistors (4.7kΩ recommended) documented in hardware wiring specification
- [ ] Multi-device address arbitration supported (0x42 for GPS, 0x4A for IMU)
- [ ] I2C bus errors (NACK, timeout, arbitration loss) detected and reported
- [ ] Unit tests verify I2C0 initialization succeeds on both platforms

## Technical Details

### Functional Requirement Details

**I2C0 Configuration:**

- GPIO 0: I2C0 SDA (data line)
- GPIO 1: I2C0 SCL (clock line)
- Clock speed: 100 kHz (Standard Mode) or 400 kHz (Fast Mode)
- Pull-up resistors: 4.7kΩ external resistors on both SDA and SCL lines (NEO-M8N datasheet requirement)

**Platform Abstraction:**

```rust
pub trait I2cInterface {
    fn write(&mut self, address: u8, data: &[u8]) -> Result<(), I2cError>;
    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), I2cError>;
    fn write_read(&mut self, address: u8, write_data: &[u8], read_buffer: &mut [u8]) -> Result<(), I2cError>;
}
```

**Error Handling:**

- NACK (No Acknowledge): Device address not responding
- Timeout: Transaction exceeds configured timeout (e.g., 100ms)
- Arbitration Loss: Multi-master bus conflict (rare in single-master system)

**Multi-Device Support:**

- NEO-M8N GPS: I2C address 0x42 (fixed)
- BNO085 IMU: I2C address 0x4A (default, configurable)
- D_SEL pin on NEO-M8N: Must be HIGH or OPEN to enable I2C/DDC interface

## Platform Considerations

### Unix

N/A – Embedded targets only (RP2040, RP2350)

### Windows

N/A – Embedded targets only (RP2040, RP2350)

### Cross-Platform

- I2C0 initialization must work identically on Pico W (RP2040) and Pico 2 W (RP2350)
- `I2cInterface` trait provides uniform API across platforms
- Platform-specific implementations in `src/platform/rp2040/i2c.rs` and `src/platform/rp2350/i2c.rs`

## Risks & Mitigation

| Risk                                       | Impact | Likelihood | Mitigation                                                   | Validation                                    |
| ------------------------------------------ | ------ | ---------- | ------------------------------------------------------------ | --------------------------------------------- |
| Missing pull-up resistors cause bus errors | High   | Medium     | Document 4.7kΩ pull-ups in wiring guide; validate with scope | Test I2C communication with GPS and IMU       |
| I2C clock speed too high for GPS/IMU       | Medium | Low        | Start with 100 kHz, increase to 400 kHz only after testing   | Verify successful transactions at both speeds |
| GPIO 0, 1 conflict with existing code      | High   | Low        | Audit `boards/freenove_standard.hwdef` before implementation | Review board definition and platform code     |
| Multi-device address collision             | High   | Low        | Verify GPS (0x42) and IMU (0x4A) addresses are unique        | Check datasheets for both devices             |

## Implementation Notes

- Use `rp235x_hal::i2c` or `rp2040_hal::i2c` for platform-specific I2C peripheral access
- I2C0 is the only I2C bus available on the remaining GPIO pins (I2C1 requires GPIO 2, 3 which are allocated to motor control)
- Embassy async I2C driver may be preferred for non-blocking operation in async tasks
- Test I2C communication with a simple device scan (read from addresses 0x42 and 0x4A) before implementing full GPS/IMU drivers

## External References

- [NEO-M8 Series Datasheet (UBX-15031086)](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf) - Section 1.17.4 (I2C/DDC Interface)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - Chapter 4.3 (I2C)
- [I2C Specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf) - NXP I2C-bus specification
