# ADR-00019 I2C0 Multi-Sensor Bus for GPS and IMU Integration

## Metadata

- Type: ADR
- Status: Superseded

**This ADR has been superseded by [ADR-00020-uart0-gps-allocation](ADR-00020-uart0-gps-allocation.md) due to hardware verification that the GPS module uses UART interface, not I2C.**

## Links

- Impacted Requirements:
  - [FR-00078-i2c0-multi-sensor-bus](../requirements/FR-00078-i2c0-multi-sensor-bus.md)
  - [FR-00075-gps-i2c-driver](../requirements/FR-00075-gps-i2c-driver.md)
  - [FR-00076-gps-operation-data-management](../requirements/FR-00076-gps-operation-data-management.md)
  - [FR-00004-gps-waypoint-navigation](../requirements/FR-00004-gps-waypoint-navigation.md)
- Related Analyses:
  - [AN-00021-gps-hardware-integration](../analysis/AN-00021-gps-hardware-integration.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-00015-i2c0-gps-imu-integration](../tasks/T-00015-i2c0-gps-imu-integration/README.md)

## Context

The pico_trail autopilot requires GPS position data and IMU orientation data for navigation (FR-00004-gps-waypoint-navigation), but faces severe GPIO pin constraints on the Freenove Standard board. Only GPIO 0, 1, 17, and 22 are available after motor control (GPIO 2-21) and WiFi (GPIO 23-25, 29) allocations.

**Problem:**

- NEO-M8N GPS module and BNO085 9-axis IMU must be connected to Pico 2 W
- Available pins (GPIO 0, 1, 17, 22) support UART0 or I2C0 on GPIO 0 and 1
- Traditional UART approach requires 2 pins per device (4 pins total for GPS + IMU), exceeding availability
- Debug logging traditionally uses UART0 serial console on GPIO 0, 1

**Constraints:**

- GPIO 0, 1, 17, 22 only available pins for sensor integration
- NEO-M8N GPS supports both UART and I2C/DDC (address 0x42, 400 kHz, full protocol support)
- BNO085 IMU requires I2C (address 0x4A, 400 kHz)
- Debug logging must remain functional for development
- Solution must work on both RP2040 (Pico W) and RP2350 (Pico 2 W)

**Forces in tension:**

- Pin efficiency vs. implementation complexity
- Debug console convenience vs. sensor integration
- UART simplicity vs. I2C multi-device capability
- Standard approach (UART GPS) vs. pin-constrained reality

## Success Metrics

- GPS position data received at 1-10 Hz via I2C interface
- IMU sensor data received at 100+ Hz via I2C interface
- Debug logging functional via USB Serial (no UART0 console needed)
- I2C bus arbitration allows concurrent GPS and IMU communication
- Total I2C transaction latency <300ms for GPS data retrieval
- No I2C address conflicts between devices (0x42 vs. 0x4A)

## Decision

We will allocate GPIO 0 (SDA) and GPIO 1 (SCL) to I2C0 bus for multi-device communication with NEO-M8N GPS module (address 0x42) and BNO085 IMU (address 0x4A), using USB Serial (CDC-ACM) for debug logging instead of UART0 serial console.

### Decision Drivers

- Pin availability: Only GPIO 0, 1 support both UART0 and I2C0
- Multi-device requirement: GPS + IMU integration requires shared bus or dedicated pins per device
- NEO-M8N I2C/DDC capability: Official datasheet (UBX-15031086, Section 1.17.4) confirms full NMEA/UBX/RTCM protocol support via I2C at 400 kHz
- Existing USB Serial infrastructure: `usb_serial` feature in `src/core/logging.rs` provides debug logging without UART0
- Future sensor expansion: I2C bus allows additional sensors (magnetometer, barometer) without GPIO exhaustion

### Considered Options

- Option A: I2C0 Multi-Sensor Bus (GPIO 0, 1 for I2C0 SDA/SCL)
- Option B: Expand GPIO Pin Access (hardware redesign, out of scope)

### Option Analysis

- Option A — Pros: Pin efficient (2 pins for multiple devices), NEO-M8N I2C/DDC fully supported, USB Serial already implemented, future-proof for additional I2C sensors | Cons: I2C driver implementation required, bus arbitration complexity, external pull-up resistors needed (4.7kΩ)
- Option B — Pros: Preserves UART0 serial console, more flexible sensor expansion | Cons: Requires hardware redesign (custom PCB or breadboard rewiring), incompatible with user's constraint (GPIO 0, 1, 17, 22 only), delayed timeline, unnecessary given USB Serial alternative

## Rationale

Option A (I2C0 Multi-Sensor Bus) is selected because:

1. **Pin efficiency**: Two sensors (GPS + IMU) on two GPIO pins vs. four pins with separate UART/SPI interfaces
2. **NEO-M8N I2C/DDC capability verified**: Official u-blox datasheet confirms I2C/DDC interface provides identical functionality to UART (NMEA, UBX, RTCM protocols at 400 kHz)
3. **USB Serial debug logging functional**: Existing `usb_serial` feature in `src/core/logging.rs` eliminates need for UART0 console
4. **Multi-device support**: I2C bus arbitration (address-based) allows concurrent GPS and IMU communication
5. **Industry standard**: I2C is standard for multi-sensor integration in embedded systems (ArduPilot, PX4)
6. **Future expansion**: Leaves GPIO 17, 22 available for SPI peripherals or additional I2C devices

Option B (hardware redesign) is rejected because:

- User explicitly stated GPIO 0, 1, 17, 22 are the only available pins
- Hardware redesign adds weeks to project timeline
- USB Serial already provides debug interface, making UART0 console unnecessary

**Trade-offs accepted:**

- I2C bus speed (400 kHz) slower than UART (115200 baud) but sufficient for GPS (1-10 Hz) and IMU (100 Hz)
- External 4.7kΩ pull-up resistors required on SDA and SCL lines (standard I2C requirement)
- I2C driver implementation complexity (DDC protocol, bus arbitration) vs. UART simplicity

## Consequences

### Positive

- Two sensors (GPS + IMU) integrated with two GPIO pins (maximum pin efficiency)
- NEO-M8N GPS and BNO085 IMU coexist on same I2C0 bus (addresses 0x42 and 0x4A)
- USB Serial debug logging functional via `usb_serial` feature (no UART0 serial console needed)
- GPIO 17, 22 remain available for future expansion (SPI peripherals, additional I2C devices)
- Platform-independent I2C abstraction (`I2cInterface` trait) supports RP2040 and RP2350
- Standard I2C protocol simplifies sensor driver development and testing

### Negative

- I2C driver implementation required for GPS (currently UART-only in `src/devices/gps.rs`)
- External 4.7kΩ pull-up resistors required on SDA and SCL lines (hardware wiring complexity)
- I2C bus arbitration adds complexity to multi-device communication
- I2C transaction overhead may introduce latency (mitigated by 400 kHz Fast Mode)
- USB connection required for debug logging (no UART0 serial console fallback)

### Neutral

- NEO-M8N D_SEL pin must be HIGH or OPEN to enable I2C/DDC interface (documented in hardware guide)
- I2C clock speed configurable (100 kHz Standard Mode or 400 kHz Fast Mode)
- GPS and IMU drivers must use async I2C to avoid blocking other tasks

## Implementation Notes

### Hardware Wiring

- GPIO 0: I2C0 SDA (data line)
- GPIO 1: I2C0 SCL (clock line)
- External 4.7kΩ pull-up resistors on SDA and SCL lines (connect to 3.3V)
- NEO-M8N D_SEL pin: HIGH or OPEN (enables I2C/DDC interface)
- NEO-M8N I2C address: 0x42 (7-bit address)
- BNO085 I2C address: 0x4A (default, configurable)

### Software Implementation

1. **I2C0 Initialization** (FR-00078-i2c0-multi-sensor-bus):
   - Platform-specific I2C0 setup in `src/platform/rp2350/i2c.rs` and `src/platform/rp2040/i2c.rs`
   - `I2cInterface` trait in `src/platform/traits.rs` for platform-independent access
   - Clock speed: 400 kHz (Fast Mode) or 100 kHz (Standard Mode)

2. **GPS I2C Driver** (FR-00075-gps-i2c-driver):
   - Extend `src/devices/gps.rs` with I2C/DDC support
   - NEO-M8N DDC protocol: Read register 0xFF (bytes available), read register 0xFD (data stream)
   - NMEA sentence parsing (GPGGA, GPRMC) reused from existing UART driver

3. **GPS Operation** (FR-00076-gps-operation-data-management):
   - GPS polling task at 1Hz, 5Hz, or 10Hz
   - NMEA checksum validation, GPS fix type detection
   - Position data available to navigation subsystem within 300ms
   - Automatic recovery from I2C errors within 5 seconds

4. **IMU I2C Driver** (future implementation):
   - Complete BNO085 driver with I2C interface
   - 100+ Hz sampling rate for accelerometer, gyroscope, magnetometer
   - Shared I2C0 bus with GPS (address arbitration)

### Documentation Updates

- `boards/freenove_standard.hwdef`: Document GPIO 0, 1 allocation to I2C0 bus
- Hardware wiring guide: I2C0 connection diagram with pull-up resistors
- Developer guide: USB Serial logging workflow (already active via `usb_serial` feature)

## Platform Considerations

- I2C0 initialization identical on Pico W (RP2040) and Pico 2 W (RP2350)
- `rp2040_hal::i2c` and `rp235x_hal::i2c` provide platform-specific I2C peripheral access
- Embassy async I2C driver preferred for non-blocking operation in async tasks
- USB Serial logging uses `crate::log_*!` macros with `usb_serial` feature (works on both platforms)

## Security & Privacy

- No security concerns: I2C communication local to device (no network exposure)
- GPS position data stored in memory only (not persisted to storage)

## Monitoring & Logging

- I2C errors logged via `crate::log_error!` macros (USB Serial output)
- GPS fix status logged at startup and when fix acquired/lost
- I2C transaction latency measurable with `defmt::trace!` (conditional on debug assertions)

## Open Questions

- [ ] Should GPS task run at 1Hz, 5Hz, or 10Hz? → Next step: Benchmark CPU overhead at each rate on RP2040/RP2350 during implementation
- [ ] Should we implement UBX protocol for advanced GPS configuration (10Hz mode)? → Decision: Defer to future enhancement, NMEA sufficient for initial implementation

## External References

- [NEO-M8 Series Datasheet (UBX-15031086)](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf) - Section 1.17.4 (I2C/DDC Interface)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - Chapter 4.3 (I2C)
- [I2C Specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf) - NXP I2C-bus specification
- [ArduPilot GPS Configuration](https://ardupilot.org/rover/docs/common-gps-how-to.html) - GPS integration best practices
