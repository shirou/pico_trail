# T-meox8 I2C0 Multi-Sensor Bus Integration Design

## Metadata

- Type: Design
- Status: Completed

## Links

- Associated Plan Document:
  - [T-meox8-i2c0-gps-imu-integration-plan](./plan.md)

## Overview

Design I2C0 multi-sensor bus integration for NEO-M8N GPS module (address 0x42) and BNO085 IMU (address 0x4A) sharing GPIO 0 (SDA) and GPIO 1 (SCL). Implement platform-independent I2C abstraction, GPS I2C/DDC driver with NMEA parsing, and GPS operation with configurable polling (1-10 Hz) and automatic error recovery. Enable waypoint navigation under strict GPIO pin constraints while maintaining USB Serial debug logging.

## Success Metrics

- [ ] GPS position data latency <300ms (I2C read + NMEA parse + state update)
- [ ] GPS fix type validation (NoFix, 2D, 3D) with failsafe trigger on 3 consecutive NoFix
- [ ] I2C bus arbitration allows concurrent GPS and IMU communication without conflicts
- [ ] Automatic recovery from I2C errors within 5 seconds (3 retries with exponential backoff)
- [ ] Zero regressions in existing tests (`cargo test --lib --quiet`)

## Background and Current State

- Context: pico_trail autopilot requires GPS position data for waypoint navigation (FR-333ym) but faces GPIO pin constraints (only GPIO 0, 1, 17, 22 available after motor control and WiFi allocations)
- Current behavior: GPS driver in `src/devices/gps.rs` is UART-only with NMEA parsing (GPGGA, GPRMC); no I2C support
- Pain points: Traditional UART approach requires 2 pins per device (4 pins total for GPS + IMU), exceeding availability; UART0 serial console conflicts with sensor GPIO usage
- Constraints: RP2040/RP2350 platform, Embassy async runtime, no_std environment, 264-520 KB RAM budget
- Related ADRs: ADR-00mjv-i2c0-gps-imu-integration (I2C0 bus selected for pin efficiency)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                   pico_trail Firmware                       │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ Navigation Subsystem (FR-333ym)                       │  │
│  │  - Waypoint following                                 │  │
│  │  - S-curve path planning                              │  │
│  └────────────────┬──────────────────────────────────────┘  │
│                   │ GpsState                                 │
│  ┌────────────────▼──────────────────────────────────────┐  │
│  │ GPS Operation (src/devices/gps_operation.rs)          │  │
│  │  - Async polling task (1Hz/5Hz/10Hz)                  │  │
│  │  - NMEA validation (checksum, range)                  │  │
│  │  - Error recovery (3 retries, exponential backoff)    │  │
│  └────────────────┬──────────────────────────────────────┘  │
│                   │ I2cInterface                             │
│  ┌────────────────▼──────────────────────────────────────┐  │
│  │ GPS I2C Driver (src/devices/gps_i2c.rs)               │  │
│  │  - NEO-M8N DDC protocol (reg 0xFF, 0xFD)              │  │
│  │  - NMEA sentence buffering                            │  │
│  │  - Checksum validation                                │  │
│  └────────────────┬──────────────────────────────────────┘  │
│                   │ I2cInterface trait                       │
│  ┌────────────────▼──────────────────────────────────────┐  │
│  │ I2C Platform Abstraction                              │  │
│  │  - src/platform/traits.rs: I2cInterface trait         │  │
│  │  - src/platform/rp2350/i2c.rs: I2C0 impl (RP2350)     │  │
│  │  - src/platform/rp2040/i2c.rs: I2C0 impl (RP2040)     │  │
│  └────────────────┬──────────────────────────────────────┘  │
│                   │ embassy-rp I2C HAL                       │
└───────────────────┼──────────────────────────────────────────┘
                    │
    ┌───────────────▼───────────────┐  ┌─────────────────────┐
    │ I2C0 Bus (GPIO 0/1, 400 kHz)  ├──┤ 4.7kΩ pull-ups      │
    │  - SDA (GPIO 0)               │  │  - SDA → 3.3V       │
    │  - SCL (GPIO 1)               │  │  - SCL → 3.3V       │
    └───────┬────────────────┬──────┘  └─────────────────────┘
            │                │
    ┌───────▼───────┐  ┌────▼─────────┐
    │ NEO-M8N GPS   │  │ BNO085 IMU   │
    │  Addr: 0x42   │  │  Addr: 0x4A  │
    │  D_SEL: HIGH  │  │  (future)    │
    └───────────────┘  └──────────────┘
```

### Components

**1. I2C Platform Abstraction (`src/platform/traits.rs`, `src/platform/rp2350/i2c.rs`, `src/platform/rp2040/i2c.rs`)**

- `I2cInterface` trait: Platform-independent I2C operations (read, write, write_read)
- `I2cError` enum: I2C error types (NACK, Timeout, ArbitrationLoss)
- RP2350/RP2040 implementations: Wrap `embassy_rp::i2c::I2c` for async non-blocking I2C
- I2C0 initialization: GPIO 0 (SDA), GPIO 1 (SCL), 400 kHz Fast Mode clock

**2. GPS I2C Driver (`src/devices/gps_i2c.rs`)**

- `GpsI2c` struct: I2C/DDC interface to NEO-M8N GPS module
- NEO-M8N DDC protocol:
  - Register 0xFF: Number of bytes available (2-byte big-endian)
  - Register 0xFD: Data stream (read sequentially)
- NMEA sentence buffering: Circular buffer for partial sentence accumulation across reads
- Checksum validation: XOR of characters between `$` and `*`
- Reuses existing NMEA parser from `src/devices/gps.rs` (GPGGA, GPRMC)

**3. GPS Operation (`src/devices/gps_operation.rs`)**

- `GpsOperation` struct: Manages GPS polling, validation, error recovery
- Async polling task: Configurable interval (1Hz, 5Hz, 10Hz) using `embassy_time::Ticker`
- NMEA validation:
  - Checksum validation (XOR accumulator)
  - GPS fix type detection (GPGGA field 6, GPRMC field 2)
  - Position range validation (lat: -90 to 90, lon: -180 to 180, alt: -1000 to 10000)
- Error recovery:
  - I2C errors: 3 retries with exponential backoff (100ms, 200ms, 400ms)
  - Persistent I2C failure: Continue polling at 1Hz, log error state
  - NoFix status: Continue polling, trigger failsafe after 3 consecutive NoFix readings
- `GpsState`: Shared state (Mutex or Channel) for latest position, fix status, timestamp

**4. Board Configuration (`boards/freenove_standard.hwdef`)**

- Document GPIO 0, 1 allocation to I2C0 bus
- I2C device addresses: GPS (0x42), IMU (0x4A)
- Hardware notes: NEO-M8N D_SEL pin HIGH or OPEN, 4.7kΩ pull-ups on SDA/SCL

### Data Flow

1. **GPS Polling Task (Embassy async task)**:
   - Timer wakes task at configurable interval (1Hz/5Hz/10Hz)
   - Call `GpsI2c::read_nmea()` to retrieve NMEA sentences via I2C
   - If I2C error, retry with exponential backoff (up to 3 times)
   - If successful, parse NMEA sentences, validate checksum and GPS fix type

2. **NMEA Parsing**:
   - Extract GPGGA: latitude, longitude, altitude, fix quality, satellites
   - Extract GPRMC: latitude, longitude, timestamp, status (A=Active, V=Void)
   - Validate position ranges, discard invalid data

3. **State Update**:
   - Update `GpsState` with latest position, fix type, timestamp
   - Log warnings for invalid data, errors for I2C failures

4. **Navigation Access**:
   - Navigation subsystem reads `GpsState` via Mutex or Channel
   - Check position age (<2 seconds), GPS fix type (not NoFix)
   - Use validated position for waypoint navigation

### Data Models and Types

```rust
// I2C Platform Abstraction
pub trait I2cInterface {
    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), I2cError>;
    fn write(&mut self, address: u8, data: &[u8]) -> Result<(), I2cError>;
    fn write_read(&mut self, address: u8, write_data: &[u8], read_buffer: &mut [u8]) -> Result<(), I2cError>;
}

pub enum I2cError {
    Nack,         // No acknowledge from device
    Timeout,      // Transaction timeout
    ArbitrationLoss, // Multi-master bus conflict
}

// GPS I2C Driver
pub struct GpsI2c<I: I2cInterface> {
    i2c: I,
    address: u8,  // 0x42
    buffer: [u8; 256], // NMEA sentence buffer
    buffer_len: usize,
}

impl<I: I2cInterface> GpsI2c<I> {
    pub fn new(i2c: I, address: u8) -> Self { ... }
    pub fn read_nmea(&mut self) -> Result<Option<NmeaSentence>, I2cError> { ... }
}

// GPS Operation
pub struct GpsOperation<I: I2cInterface> {
    gps: GpsI2c<I>,
    polling_rate: PollingRate, // 1Hz, 5Hz, 10Hz
    retry_count: u8,
}

pub enum PollingRate {
    Rate1Hz,
    Rate5Hz,
    Rate10Hz,
}

// GPS State (shared with navigation)
pub struct GpsState {
    pub position: Option<GpsPosition>,
    pub last_update: Instant,
    pub fix_type: GpsFixType,
}

pub struct GpsPosition {
    pub latitude: f32,      // Decimal degrees
    pub longitude: f32,     // Decimal degrees
    pub altitude: f32,      // Meters above sea level
    pub fix_type: GpsFixType,
    pub satellites: u8,
    pub timestamp: u32,     // Milliseconds since system start
}

pub enum GpsFixType {
    NoFix,
    Fix2D,
    Fix3D,
}
```

### Error Handling

- English messages using `crate::log_error!`, `crate::log_warn!` macros
- I2C errors: Log error with retry count, enter error state after 3 failures
- NMEA validation errors: Log warning, discard invalid sentence, continue polling
- GPS fix loss: Log info when NoFix detected, trigger failsafe after 3 consecutive NoFix
- Position range errors: Log error, discard position, continue polling
- Error recovery: Automatic retry with exponential backoff, continue polling at reduced rate on persistent failure

### Security Considerations

- No security concerns: I2C communication local to device (no network exposure)
- GPS position data stored in memory only (not persisted to storage)

### Performance Considerations

- **Hot paths**:
  - I2C read (10-50ms): Async I2C prevents blocking other tasks
  - NMEA parsing (<10ms): Optimized parser, avoid allocations
  - State update (<5ms): Mutex lock and copy
- **Latency budget**: GPS fix (100-200ms) + I2C read (10-50ms) + parse (10ms) + state update (5ms) = 125-265ms (within 300ms target)
- **Concurrency**: Embassy async I2C allows GPS and IMU to share I2C0 bus without blocking
- **Progress indicators**: GPS fix status logged at startup and when fix acquired/lost

### Platform Considerations

#### Embedded (RP2040/RP2350)

- GPIO 0 (I2C0 SDA), GPIO 1 (I2C0 SCL) configured via `embassy_rp::i2c::I2c`
- I2C0 clock speed: 400 kHz (Fast Mode) or 100 kHz (Standard Mode)
- External 4.7kΩ pull-up resistors required on SDA and SCL lines
- NEO-M8N D_SEL pin: HIGH or OPEN to enable I2C/DDC interface
- Embassy async runtime provides non-blocking I2C operations

#### Host Tests

- `MockI2c` struct: Simulates I2C transactions with pre-recorded NMEA data
- Inject I2C errors (NACK, timeout) to test retry logic
- Inject invalid NMEA sentences to test validation and error handling

## Alternatives Considered

1. **UART GPS + Expand GPIO Pins (Rejected)**
   - Pros: Preserves traditional UART interface, familiar debugging
   - Cons: Requires hardware redesign, incompatible with user's constraint (GPIO 0, 1, 17, 22 only)
   - Decision Rationale: User explicitly stated pin constraint; ADR-00mjv selected I2C0 for pin efficiency

2. **SPI GPS Interface (Not Feasible)**
   - Pros: Higher throughput than I2C
   - Cons: NEO-M8N SPI requires GPIO 2/18 (SCK), 3/19 (MOSI), which are allocated to motor control
   - Decision Rationale: GPIO 2-21 reserved for motor control, SPI not available

## Migration and Compatibility

- Backward compatibility: Existing UART GPS driver (`src/devices/gps.rs`) remains available for boards with free UART pins
- Forward compatibility: I2C0 bus can accommodate additional I2C sensors (magnetometer, barometer) in future
- No breaking changes: GPS driver API remains consistent (returns `GpsPosition`, `GpsFixType`)

## Testing Strategy

### Unit Tests

- **I2C Platform Abstraction**:
  - `MockI2c` struct: Simulates I2C read, write, write_read operations
  - Test I2C errors: NACK, timeout, arbitration loss
- **GPS I2C Driver**:
  - Test NEO-M8N DDC protocol: Read register 0xFF, read register 0xFD
  - Test NMEA sentence buffering: Partial sentences across multiple reads
  - Test checksum validation: Valid and invalid checksums
- **GPS Operation**:
  - Test polling at 1Hz, 5Hz, 10Hz
  - Test NMEA validation: Checksum, GPS fix type, position ranges
  - Test error recovery: I2C retry with exponential backoff, persistent failure
  - Test failsafe trigger: 3 consecutive NoFix readings

### Integration Tests

- **Hardware Validation** (requires NEO-M8N GPS module):
  - Test GPS fix acquisition indoors (NoFix expected) and outdoors (3D fix expected)
  - Test I2C bus arbitration: Concurrent GPS and IMU communication
  - Test GPS cold start delay (up to 30 seconds)
  - Test GPS warm/hot start recovery (<5 seconds)
  - Measure I2C transaction latency: End-to-end GPS fix → position available in `GpsState`
- **Stress Testing**:
  - Disconnect GPS module during operation, verify automatic recovery
  - Inject I2C bus noise (short SDA/SCL lines), verify error handling
  - Run continuous GPS polling for 1 hour, verify stability

## Documentation Impact

- Update `docs/architecture.md`: Add I2C0 bus and GPS I2C driver to system architecture
- Update `boards/freenove_standard.hwdef`: Document GPIO 0, 1 allocation to I2C0 bus
- Add hardware wiring guide: I2C0 connection diagram with pull-up resistors, NEO-M8N D_SEL pin configuration

## External References

- [NEO-M8 Series Datasheet (UBX-15031086)](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf) - Section 1.17.4 (I2C/DDC Interface)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - Chapter 4.3 (I2C)
- [I2C Specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf) - NXP I2C-bus specification
- [NMEA 0183 Standard](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard)

## Open Questions

- [ ] Should GPS task run at 1Hz, 5Hz, or 10Hz? → Next step: Benchmark CPU overhead at each rate during Phase 3
- [ ] Should we implement UBX protocol for advanced GPS configuration (10Hz mode)? → Decision: Defer to future enhancement, NMEA sufficient for initial implementation
