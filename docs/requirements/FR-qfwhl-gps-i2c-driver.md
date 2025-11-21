# FR-qfwhl GPS I2C/DDC Driver Implementation for NEO-M8N

## Metadata

- Type: Functional Requirement
- Status: Superseded

**This requirement has been superseded by [FR-93b5v-gps-uart-driver](FR-93b5v-gps-uart-driver.md) due to hardware verification that the GPS module uses UART interface, not I2C.**

## Links

- Related Analyses:
  - [AN-xfiyr-gps-hardware-integration](../analysis/AN-xfiyr-gps-hardware-integration.md)
- Related ADRs:
  - [ADR-00mjv-i2c0-gps-imu-integration](../adr/ADR-00mjv-i2c0-gps-imu-integration.md)
- Prerequisite Requirements:
  - [FR-2f599-i2c0-multi-sensor-bus](FR-2f599-i2c0-multi-sensor-bus.md)
- Dependent Requirements:
  - [FR-3ik7l-gps-operation-data-management](FR-3ik7l-gps-operation-data-management.md)
- Related Tasks:
  - [T-meox8-i2c0-gps-imu-integration](../tasks/T-meox8-i2c0-gps-imu-integration/README.md)

## Requirement Statement

The GPS driver shall communicate with NEO-M8N GPS module via I2C/DDC interface (address 0x42) to receive NMEA sentences and extract position data.

## Rationale

The existing GPS driver in `src/devices/gps.rs` is UART-based. With GPIO 0, 1 allocated to I2C0 for multi-sensor integration, the GPS driver must be extended to support I2C/DDC protocol. NEO-M8N datasheet (UBX-15031086, Section 1.17.4) confirms I2C/DDC fully supports NMEA, UBX, and RTCM protocols with the same functionality as UART.

## User Story (if applicable)

As a GPS driver, I want to read NMEA sentences from NEO-M8N via I2C/DDC interface, so that I can provide position data to the navigation subsystem without requiring a dedicated UART port.

## Acceptance Criteria

- [ ] GPS driver extends `src/devices/gps.rs` with I2C/DDC interface support
- [ ] Driver implements I2C read from NEO-M8N at address 0x42 using DDC protocol
- [ ] NMEA sentences (GPGGA, GPRMC) successfully parsed from I2C data stream
- [ ] GPS driver uses `I2cInterface` trait for platform-independent I2C access
- [ ] Valid GPS position (latitude, longitude, altitude) extracted from NMEA sentences
- [ ] GPS fix status (no fix, 2D fix, 3D fix) correctly detected from NMEA data
- [ ] Unit tests verify I2C-based NMEA parsing with known GPS data
- [ ] Integration test confirms GPS position data retrieved via I2C on hardware

## Technical Details

### Functional Requirement Details

**NEO-M8N I2C/DDC Protocol:**

- I2C address: 0x42 (7-bit address)
- Interface: Fast Mode compliant (up to 400 kHz)
- Data format: NMEA sentences as ASCII text stream (same as UART)
- Register 0xFF: Number of bytes available to read (2-byte big-endian)
- Register 0xFD: Data stream (read sequentially)

**DDC Read Sequence:**

1. Write to register 0xFF to query available bytes
2. Read 2 bytes: high byte (MSB), low byte (LSB) = bytes available
3. If bytes > 0, read from register 0xFD up to available bytes
4. Parse NMEA sentences from received data

**NMEA Sentence Parsing:**

- Supported sentences: GPGGA (position, altitude, fix), GPRMC (position, velocity, timestamp)
- Sentence format: `$GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,,*hh\r\n`
- Checksum validation: XOR of characters between `$` and `*`
- Invalid sentences discarded

**Data Structure:**

```rust
pub struct GpsPosition {
    pub latitude: f32,      // Decimal degrees
    pub longitude: f32,     // Decimal degrees
    pub altitude: f32,      // Meters above sea level
    pub fix_type: GpsFixType, // NoFix, Fix2D, Fix3D
    pub satellites: u8,     // Number of satellites used
    pub timestamp: u32,     // Milliseconds since system start
}
```

## Platform Considerations

### Unix

N/A – Embedded targets only (RP2040, RP2350)

### Windows

N/A – Embedded targets only (RP2040, RP2350)

### Cross-Platform

- GPS I2C driver must work on both Pico W (RP2040) and Pico 2 W (RP2350)
- Platform abstraction via `I2cInterface` trait ensures portability
- Host tests use mock I2C interface with pre-recorded NMEA data

## Risks & Mitigation

| Risk                                     | Impact | Likelihood | Mitigation                                              | Validation                                 |
| ---------------------------------------- | ------ | ---------- | ------------------------------------------------------- | ------------------------------------------ |
| I2C read errors cause GPS data loss      | Medium | Medium     | Implement retry logic (up to 3 retries) on I2C errors   | Test with injected I2C errors              |
| NMEA sentence fragmentation across reads | Medium | High       | Buffer partial sentences, parse only complete sentences | Test with varying I2C read sizes           |
| GPS cold start delay (30+ seconds)       | Low    | High       | Log GPS status, delay navigation until fix acquired     | Test GPS startup sequence indoors/outdoors |
| D_SEL pin not configured correctly       | High   | Low        | Document D_SEL wiring (HIGH or OPEN) in hardware guide  | Verify I2C communication on first boot     |

## Implementation Notes

- Extend `src/devices/gps.rs` with `GpsI2c` struct implementing `GpsInterface` trait
- NEO-M8N DDC protocol details in datasheet Section 1.17.4, Page 13
- NMEA sentence parsing logic already exists in `src/devices/gps.rs`, reuse for I2C
- Use circular buffer to accumulate NMEA data across multiple I2C reads
- GPS update rate: 1Hz default, configurable up to 10Hz (requires UBX protocol, defer to future enhancement)
- Consider Embassy async I2C for non-blocking reads in async GPS task

## External References

- [NEO-M8 Series Datasheet (UBX-15031086)](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf) - Section 1.17.4 (I2C/DDC Interface)
- [NMEA 0183 Standard](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard)
- [NMEA Sentence Reference](https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_MessageOverview.html)
