# FR-00077 GPS UART Driver Implementation for NEO-M8N

## Metadata

- Type: Functional Requirement
- Status: Completed

## Links

- Related Analyses:
  - [AN-00021-gps-hardware-integration](../analysis/AN-00021-gps-hardware-integration.md)
- Prerequisite Requirements:
  - None (UART infrastructure already exists)
- Dependent Requirements:
  - [FR-00076-gps-operation-data-management](FR-00076-gps-operation-data-management.md)
  - [FR-00080-gps-mavlink-telemetry](FR-00080-gps-mavlink-telemetry.md)
  - [FR-00081-gps-navigation-state-access](FR-00081-gps-navigation-state-access.md)
  - [FR-00079-gps-course-over-ground](FR-00079-gps-course-over-ground.md)
  - [FR-00092-gps-initialization-separation](FR-00092-gps-initialization-separation.md)
- Related Tasks:
  - [T-00016-uart0-gps-integration](../tasks/T-00016-uart0-gps-integration/README.md)

## Requirement Statement

The GPS system shall communicate with the NEO-M8N GPS module via UART1 interface (GPIO 4 TX, GPIO 5 RX) at 9600 baud to receive NMEA sentences and extract position data for waypoint navigation.

## Rationale

The NEO-M8N GPS module in use connects via UART interface. The existing GPS driver in `src/devices/gps.rs` provides full UART-based NMEA parsing functionality. With GPIO 4, 5 allocated to UART1 for GPS communication, this requirement ensures reliable GPS position data reception for the navigation subsystem (FR-00004) while avoiding RP2350 boot conflicts discovered with UART0 (GPIO 0/1). UART1 allocation frees GPIO 0, 1 for future I2C0 IMU integration and maintains USB Serial as the primary debug logging interface.

## User Story (if applicable)

As a GPS driver, I want to read NMEA sentences from NEO-M8N via UART1 interface, so that I can provide position data to the navigation subsystem without boot conflicts and without requiring complex bus arbitration.

## Acceptance Criteria

- [ ] UART1 initialized with GPIO 4 (TX) and GPIO 5 (RX) for GPS communication
- [ ] UART configuration: 9600 baud, 8 data bits, no parity, 1 stop bit (8N1)
- [ ] GPS driver reads NMEA sentences (GPGGA, GPRMC) from UART1 data stream
- [ ] GPS driver uses `UartInterface` trait for platform-independent UART access
- [ ] Valid GPS position (latitude, longitude, altitude) extracted from NMEA sentences
- [ ] GPS fix status (no fix, 2D fix, 3D fix) correctly detected from NMEA data
- [ ] Unit tests verify UART-based NMEA parsing with known GPS data using MockUart
- [ ] Integration test confirms GPS position data retrieved via UART1 on hardware
- [ ] No boot hangs when GPS module connected and powered

## Technical Details

### Functional Requirement Details

**NEO-M8N UART Protocol:**

- Interface: UART (3.3V TTL serial)
- Default baud rate: 9600 bps (configurable: 4800, 9600, 19200, 38400, 57600, 115200)
- Data format: 8N1 (8 data bits, no parity, 1 stop bit)
- Output: NMEA 0183 sentences (GPGGA, GPRMC, etc.)
- Update rate: 1Hz default (configurable up to 10Hz with UBX protocol)

**UART1 GPIO Configuration:**

- GPIO 4: UART1 TX (connected to NEO-M8N RX)
- GPIO 5: UART1 RX (connected to NEO-M8N TX)
- Voltage: 3.3V TTL (compatible with RP2350)

**Boot Conflict Avoidance:**

- UART0 (GPIO 0/1) conflicts with RP2350 bootrom stdio initialization
- NEO-M8N transmits NMEA data immediately on power-up
- GPS data on GPIO 0 during boot causes Pico 2 W to hang/freeze
- UART1 (GPIO 4/5) avoids boot sequence conflicts

**NMEA Sentence Parsing:**

- Supported sentences: GPGGA (position, altitude, fix), GPRMC (position, velocity, timestamp)
- Sentence format: `$GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,,*hh\r\n`
- Checksum validation: XOR of characters between `$` and `*`
- Invalid sentences discarded with error logging

**Data Structure:**

```rust
pub struct GpsPosition {
    pub latitude: f32,      // Decimal degrees
    pub longitude: f32,     // Decimal degrees
    pub altitude: f32,      // Meters above sea level
    pub fix_type: GpsFixType, // NoFix, Fix2D, Fix3D
    pub satellites: u8,     // Number of satellites used
}
```

**Existing Implementation:**

The GPS UART driver already exists in `src/devices/gps.rs` with:

- Generic over `UartInterface` trait (platform-independent)
- NMEA sentence parsing (GPRMC, GPGGA)
- Coordinate conversion (NMEA format to decimal degrees)
- Fix type detection
- 256-byte internal buffer for sentence assembly
- Comprehensive unit tests with MockUart

## Platform Considerations

### Unix

N/A – Embedded targets only (RP2040, RP2350)

### Windows

N/A – Embedded targets only (RP2040, RP2350)

### Cross-Platform

- GPS UART driver must work on both Pico W (RP2040) and Pico 2 W (RP2350)
- Platform abstraction via `UartInterface` trait ensures portability
- Host tests use MockUart with pre-recorded NMEA data
- UART1 peripheral exists on both RP2040 and RP2350
- GPIO 4, 5 support UART1 function on both platforms

## Risks & Mitigation

| Risk                                     | Impact | Likelihood | Mitigation                                              | Validation                                 |
| ---------------------------------------- | ------ | ---------- | ------------------------------------------------------- | ------------------------------------------ |
| UART read errors cause GPS data loss     | Medium | Medium     | Implement retry logic and buffer overflow handling      | Test with injected UART errors             |
| NMEA sentence fragmentation across reads | Medium | High       | Buffer partial sentences, parse only complete sentences | Test with varying UART buffer sizes        |
| GPS cold start delay (30+ seconds)       | Low    | High       | Log GPS status, delay navigation until fix acquired     | Test GPS startup sequence indoors/outdoors |
| Incorrect baud rate causes garbled data  | High   | Low        | Use NEO-M8N default 9600 baud, validate with known NMEA | Test with real NEO-M8N module              |
| GPIO 4, 5 conflict with existing usage   | Low    | Low        | Audit GPIO usage - no conflicts (GPIO 4, 5 available)   | Verify hwdef and confirm no conflicts      |
| Boot hangs with UART0 GPS connection     | High   | N/A        | **Mitigated** - Use UART1 instead of UART0              | Hardware test confirms no boot hangs       |

## Implementation Notes

- Reuse existing GPS UART driver (`src/devices/gps.rs`) - no new driver implementation needed
- Implement GPS operation manager (`src/devices/gps_operation.rs`) to integrate UART GPS driver with:
  - Async polling loop with configurable rate (1Hz, 5Hz, 10Hz)
  - Error recovery with exponential backoff (3 retries)
  - Shared GPS state for navigation subsystem access
  - Failsafe triggering after 3 consecutive NoFix readings
- Update `boards/freenove_standard.hwdef` to document GPIO 4, 5 UART1 allocation
- GPS update rate: 1Hz default, configurable (10Hz requires UBX protocol, defer to future enhancement)
- Use Embassy async runtime for non-blocking UART reads in GPS task
- USB Serial remains primary debug logging interface (no conflict)
- GPIO 0, 1 freed for future I2C0 IMU integration

## External References

- [NEO-M8 Series Datasheet (UBX-15031086)](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf) - UART interface specification (Section 1.17.1)
- [NMEA 0183 Standard](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard) - NMEA sentence format specification
- [NMEA Sentence Reference](https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_MessageOverview.html) - GPGGA and GPRMC message details
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - UART peripheral specification (Chapter 4.2)
