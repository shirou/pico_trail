# T-meox8 I2C0 Multi-Sensor Bus Integration for GPS and IMU

## Metadata

- Type: Task
- Status: Completed

## Links

- Related Requirements:
  - [FR-2f599-i2c0-multi-sensor-bus](../../requirements/FR-2f599-i2c0-multi-sensor-bus.md)
  - [FR-qfwhl-gps-i2c-driver](../../requirements/FR-qfwhl-gps-i2c-driver.md)
  - [FR-3ik7l-gps-operation-data-management](../../requirements/FR-3ik7l-gps-operation-data-management.md)
  - [FR-333ym-gps-waypoint-navigation](../../requirements/FR-333ym-gps-waypoint-navigation.md)
- Related ADRs:
  - [ADR-00mjv-i2c0-gps-imu-integration](../../adr/ADR-00mjv-i2c0-gps-imu-integration.md)
- Related Analyses:
  - [AN-xfiyr-gps-hardware-integration](../../analysis/AN-xfiyr-gps-hardware-integration.md)
- Associated Design Document:
  - [design.md](./design.md)
- Associated Plan Document:
  - [plan.md](./plan.md)

## Summary

Implement I2C0 multi-sensor bus integration for NEO-M8N GPS module (address 0x42) and BNO085 9-axis IMU (address 0x4A) on GPIO 0 (SDA) and GPIO 1 (SCL). Enable GPS position data at 1-10 Hz via I2C/DDC interface with NMEA sentence parsing, supporting waypoint navigation (FR-333ym) under strict GPIO pin constraints (only GPIO 0, 1, 17, 22 available). Use USB Serial for debug logging instead of UART0 serial console.

## Implementation Phases

1. **Phase 1**: I2C0 Initialization and Platform Abstraction
2. **Phase 2**: GPS I2C/DDC Driver Implementation
3. **Phase 3**: GPS Operation and Data Management
4. **Phase 4**: Testing and Verification

## Success Criteria

- GPS position data received at 1-10 Hz via I2C interface with <300ms latency
- NMEA sentence parsing (GPGGA, GPRMC) extracts latitude, longitude, altitude, fix status
- I2C bus arbitration allows concurrent GPS (0x42) and IMU (0x4A) communication
- GPS driver automatically recovers from I2C errors within 5 seconds
- GPS fix type validation (NoFix, 2D, 3D) with failsafe trigger on loss
- USB Serial debug logging functional via `usb_serial` feature
- Host unit tests pass without hardware (`cargo test --lib`)
- Hardware validation confirms GPS and IMU coexistence on I2C0 bus

## Key Design Decisions

- **I2C0 Fast Mode (400 kHz)**: NEO-M8N supports Fast Mode per datasheet Section 1.17.4
- **I2cInterface trait**: Platform-independent abstraction for RP2040 and RP2350
- **NEO-M8N I2C/DDC protocol**: Read register 0xFF (bytes available), read register 0xFD (data stream)
- **NMEA parsing**: Reuse existing `src/devices/gps.rs` parser for GPGGA and GPRMC sentences
- **GPS polling rate**: Configurable 1Hz (default), 5Hz, or 10Hz via async task
- **Error recovery**: 3 retries with exponential backoff (100ms, 200ms, 400ms) for I2C errors
- **USB Serial logging**: Continue using `crate::log_*!` macros with `usb_serial` feature
- **External pull-ups**: 4.7kΩ resistors required on SDA and SCL lines (hardware requirement)
- **Embassy async I2C**: Non-blocking operation prevents control loop starvation

## Progress Tracking

See [Implementation Plan](plan.md) for detailed task checklists and phase status.

## Notes

- Follows ADR-00mjv: I2C0 Multi-Sensor Bus selected for pin efficiency (2 pins for 2 sensors)
- NEO-M8N D_SEL pin must be HIGH or OPEN to enable I2C/DDC interface
- GPS cold start can take 30+ seconds; 5-second recovery target applies to warm/hot start
- Indoor GPS testing may not achieve fix; use outdoor environment for validation
- IMU driver implementation deferred to future task (T-<future-id>)
- Compatible with RP2350 (Pico 2 W) and RP2040 (Pico W)
- GPIO 17, 22 remain available for future SPI peripherals or additional I2C devices
