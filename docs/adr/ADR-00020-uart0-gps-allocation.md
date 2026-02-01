# ADR-00020 UART1 Allocation for NEO-M8N GPS Module

## Metadata

- Type: ADR
- Status: Accepted

## Links

- Related Analyses:
  - [AN-00021-gps-hardware-integration](../analysis/AN-00021-gps-hardware-integration.md)
- Impacted Requirements:
  - [FR-00077-gps-uart-driver](../requirements/FR-00077-gps-uart-driver.md)
  - [FR-00004-gps-waypoint-navigation](../requirements/FR-00004-gps-waypoint-navigation.md)
  - [FR-00076-gps-operation-data-management](../requirements/FR-00076-gps-operation-data-management.md)
- Supersedes ADRs:
  - [ADR-00019-i2c0-gps-imu-integration](ADR-00019-i2c0-gps-imu-integration.md)
- Related Tasks:
  - [T-00016-uart0-gps-integration](../tasks/T-00016-uart0-gps-integration/README.md)

## Context

The pico_trail autopilot requires GPS position data for waypoint navigation (FR-00004). With limited GPIO pin availability (GPIO 0, 1, 17, 22), the hardware integration approach must balance sensor connectivity with debug logging requirements.

**Problem:**

- NEO-M8N GPS module in use connects via UART interface (verified during implementation)
- Previous decision (ADR-00019) assumed I2C0 multi-sensor bus for GPS + IMU
- Hardware verification revealed GPS module uses UART, not I2C/DDC

**Constraints:**

- Limited GPIO pin availability (GPIO 0-9, 16-22, 26 available on Freenove 4WD Car)
- GPIO 6-9 occupied by motors, GPIO 16 by WS2812, GPIO 26 by ADC
- GPIO 0, 1 support both UART0 and I2C0 functions
- GPIO 4, 5 support UART1 function
- USB Serial provides debug logging (no UART console needed)
- BNO085 IMU requires I2C interface (future integration)

**Boot Conflict Discovery:**

- UART0 (GPIO 0/1) conflicts with RP2350 bootrom stdio initialization
- NEO-M8N transmits NMEA data immediately on power-up
- GPS data on GPIO 0 during boot causes Pico 2 W to hang/freeze
- UART1 (GPIO 4/5) avoids boot sequence conflicts

**Assumptions:**

- NEO-M8N GPS configured for UART mode (9600 baud default)
- USB Serial remains primary debug interface (`usb_serial` feature)
- GPS update rate 1-10Hz sufficient for waypoint navigation
- I2C0 can be allocated to IMU on GPIO 0, 1 (now available)

**Existing Implementation:**

- `src/devices/gps.rs`: Complete UART-based GPS driver with NMEA parsing
- `src/platform/traits/uart.rs`: UartInterface trait (platform-independent)
- `src/platform/rp2350/uart.rs`: RP2350 UART implementation (supports UART0/UART1)
- `src/platform/mock/uart.rs`: MockUart for testing (functional)

## Success Metrics

- GPS position data received at 1-10Hz with <300ms latency
- NMEA sentence validation (checksum, fix type, position range) with 100% invalid data rejection
- Automatic UART error recovery within 5 seconds (3 retries with exponential backoff)
- All existing tests pass; no regressions in control loop timing
- Hardware validation confirms GPS NMEA reception via UART1
- No boot hangs when GPS module connected and powered

## Decision

**We will allocate UART1 (GPIO 4 TX, GPIO 5 RX) to NEO-M8N GPS communication at 9600 baud, 8N1 format.**

This decision supersedes ADR-00019 (I2C0 multi-sensor bus) based on hardware verification that the GPS module uses UART interface, and avoids UART0 boot conflicts discovered during hardware testing. GPIO 0, 1 become available for I2C0 IMU integration.

### Decision Drivers

1. **Boot conflict avoidance**: UART0 (GPIO 0/1) causes boot hangs when GPS transmits data during power-up
2. **Hardware compatibility**: NEO-M8N GPS module in use connects via UART interface
3. **Existing implementation**: UART GPS driver (`src/devices/gps.rs`) already functional with comprehensive tests
4. **Platform abstraction**: UartInterface trait supports both UART0 and UART1 (RP2040/RP2350 portability)
5. **Debug logging**: USB Serial provides standard terminal-compatible logging (no UART console needed)
6. **Pin efficiency**: GPIO 4, 5 available on Freenove 4WD Car; GPIO 0, 1 freed for I2C0 IMU

### Considered Options

- Option A: UART0 GPS on GPIO 0, 1
- Option B: Redesign hardware to use I2C GPS module
- **Option C: UART1 GPS on GPIO 4, 5 (Chosen)**

### Option Analysis

- Option A (UART0 GPS) — Pros: GPS hardware uses UART; existing driver functional; simple protocol | Cons: **Boot hangs due to GPIO 0 conflict with bootrom**; UART0 occupied; GPIO 0, 1 unavailable for I2C0
- Option B (I2C GPS) — Pros: Multi-sensor bus sharing | Cons: Requires different GPS hardware; existing I2C driver incomplete; hardware redesign cost; added complexity
- **Option C (UART1 GPS)** — Pros: **Avoids boot conflicts**; GPS hardware uses UART; existing driver functional; GPIO 0, 1 freed for I2C0 IMU; simple protocol | Cons: Requires GPIO 4, 5 (available on Freenove 4WD Car); single sensor per UART

## Rationale

The NEO-M8N GPS module in use connects via UART interface (verified during hardware testing). The existing UART GPS driver in `src/devices/gps.rs` provides complete NMEA parsing functionality with comprehensive unit tests using MockUart.

**Critical Discovery - UART0 Boot Conflict:**

During hardware testing, connecting the GPS module to UART0 (GPIO 0/1) caused the Pico 2 W to hang during boot. Investigation revealed:

- RP2350 bootrom uses GPIO 0 for stdio UART during initialization
- NEO-M8N transmits NMEA data immediately upon power-up (no delay)
- GPS data arriving on GPIO 0 during boot sequence interferes with bootrom logic
- This is a known issue: UART0 stdio conflicts with external UART devices

**Why UART1 over UART0:**

- **Boot reliability**: UART1 (GPIO 4/5) avoids bootrom stdio conflicts
- **Hardware testing**: UART0 confirmed to cause boot hangs with GPS connected
- **GPIO availability**: GPIO 4, 5 available on Freenove 4WD Car (not occupied by motors)
- **I2C0 liberation**: GPIO 0, 1 now available for BNO085 IMU on I2C0

**Why UART over I2C:**

- GPS hardware uses UART interface (not I2C/DDC)
- UART GPS driver already exists and is fully functional
- UART protocol simpler than I2C (no bus arbitration, no pull-up resistors)
- Direct point-to-point connection (no address conflicts)

**Why GPIO 4, 5:**

- GPIO 4, 5 support UART1 function on RP2350
- GPIO 4, 5 not occupied by motors, LEDs, or other peripherals
- GPIO 0, 1 freed for I2C0 BNO085 IMU integration

**Trade-offs Accepted:**

- UART1 dedicated to GPS (not available for other sensors)
- Single sensor per UART (cannot share like I2C bus)
- GPIO 4, 5 not available for other peripherals

## Consequences

### Positive

- **No boot hangs**: UART1 avoids RP2350 bootrom conflicts that caused system freezes with UART0
- **I2C0 now available**: GPIO 0, 1 freed for BNO085 IMU integration (originally intended purpose)
- Reuse existing, proven UART GPS driver (`src/devices/gps.rs`) with zero new driver development
- Simple UART protocol (no bus arbitration, no pull-up resistors, no address conflicts)
- Platform-independent via UartInterface trait (RP2040/RP2350 compatible)
- USB Serial debug logging remains primary interface (familiar terminal emulator workflow)
- NMEA parsing well-established with comprehensive unit tests

### Negative

- UART1 occupied by GPS (cannot be used for other sensors)
- Single sensor per UART (cannot share multiple devices like I2C bus)
- GPIO 4, 5 not available for other peripherals
- Limited to 9600 baud GPS update rate without UBX protocol configuration

### Neutral

- GPS update rate 1Hz default (sufficient for waypoint navigation per FR-00004)
- Hardware wiring requires GPIO 5 (RX) → NEO-M8N TX, GPIO 4 (TX) → NEO-M8N RX crossover
- Future GPS modules must support UART interface (standard across all u-blox modules)
- UART abstraction (UartInterface trait) allows easy UART instance changes

## Implementation Notes

**High-level Plan:**

1. **GPS Operation Manager** (`src/devices/gps_operation.rs`):
   - Integrate existing UART GPS driver (`GpsDriver` from `gps.rs`)
   - Implement async polling loop with configurable rate (1Hz/5Hz/10Hz)
   - Add error recovery with exponential backoff (3 retries: 100ms, 200ms, 400ms)
   - Implement shared GPS state for navigation subsystem access
   - Add failsafe triggering after 3 consecutive NoFix readings

2. **Board Configuration** (`boards/freenove_standard.hwdef`):
   - Document GPIO 4, 5 allocation to UART1 GPS
   - Specify NEO-M8N connection: GPIO 4 (TX), GPIO 5 (RX), 9600 baud, 8N1
   - Note I2C0 available for BNO085 IMU on GPIO 0, 1
   - Document boot conflict rationale for UART1 selection

3. **Platform Integration**:
   - UART1 initialization in platform code (use `src/platform/rp2350/uart.rs` with UART1 instance)
   - Ensure UartInterface trait usage for RP2040/RP2350 portability
   - GPIO 4 configured as UART1 TX, GPIO 5 as UART1 RX

**Error Handling:**

- UART read errors: Retry up to 3 times with exponential backoff
- Invalid NMEA sentences: Discard with error logging, continue polling
- GPS NoFix status: Continue polling, trigger failsafe after 3 consecutive NoFix readings
- GPS cold start: Log status, delay navigation mode until fix acquired (up to 30 seconds)

**Data Flow:**

```
NEO-M8N GPS (UART TX) → GPIO 5 (RX) → UART1 → GpsDriver → GpsOperation → GpsState → Navigation Subsystem
```

## Examples

**Hardware Wiring:**

```
NEO-M8N GPS Module    Pico 2 W (RP2350)
------------------    -----------------
TX    ------------>   GPIO 5 (UART1 RX)
RX    <------------   GPIO 4 (UART1 TX)
VCC   <------------   3.3V (Pin 36)
GND   ------------>   GND (Pin 3/8/13/18/23/28/33/38)
```

**NMEA Sentence Example:**

```
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
```

**GPS Initialization (Pseudo-code):**

```rust
// Platform creates UART1 interface
let uart1 = platform.create_uart(UartConfig {
    uart_id: 1,  // UART1
    baud_rate: 9600,
    data_bits: 8,
    parity: None,
    stop_bits: 1,
})?;

// GPS driver wraps UART interface
let gps_driver = GpsDriver::new(uart1);

// GPS operation manager handles polling and state
let mut gps_operation = GpsOperation::new(gps_driver, PollingRate::Rate1Hz);

// Spawn async task for GPS polling
spawner.spawn(async move {
    gps_operation.poll_loop().await;
}).ok();
```

## Platform Considerations

- **RP2040 (Pico W)**: UART1 on GPIO 4, 5 supported (same pinout as RP2350)
- **RP2350 (Pico 2 W)**: UART implementation in `src/platform/rp2350/uart.rs` supports both UART0 and UART1
- **Cross-platform**: UartInterface trait ensures portability across RP2040/RP2350 and UART instances
- **Host tests**: MockUart provides UART simulation for unit testing without hardware
- **Boot compatibility**: UART1 confirmed to avoid boot hangs observed with UART0

## Security & Privacy

- GPS position data logged via USB Serial (no sensitive data exposure beyond development)
- NMEA sentences contain geographic coordinates (publicly available satellite data)
- No GPS configuration changes via UBX protocol (prevents accidental misconfiguration)

## Monitoring & Logging

- GPS fix acquisition: `crate::log_info!("GPS: Fix acquired ({:?}, {} satellites)", fix_type, satellites)`
- GPS fix lost: `crate::log_info!("GPS: Fix lost (NoFix), continuing polling")`
- UART errors: `crate::log_error!("GPS: UART error after {} retries", MAX_RETRIES)`
- Invalid NMEA: `crate::log_warn!("GPS: NMEA parse error: {:?}", error)`
- GPS failsafe: `crate::log_warn!("GPS: Failsafe triggered (3 consecutive NoFix readings)")`

## Open Questions

- [ ] Should GPS task run at 1Hz, 5Hz, or 10Hz? → Next step: Benchmark CPU overhead at each rate during implementation
- [ ] Should we implement UBX protocol for advanced GPS configuration (10Hz mode)? → Decision: Defer to future enhancement, NMEA sufficient for initial implementation
- [x] Which GPIO pins should be used for I2C0 IMU integration? → **Answered**: GPIO 0, 1 now available for I2C0 BNO085 IMU
- [x] Does UART0 cause boot issues? → **Answered**: Yes, confirmed during hardware testing; UART1 chosen to avoid conflicts

## External References

- [NEO-M8 Series Datasheet (UBX-15031086)](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf) - UART interface specification (Section 1.17.1, Page 11)
- [NMEA 0183 Standard](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard) - NMEA sentence format specification
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - UART peripheral specification (Chapter 4.2)
- [ArduPilot GPS Configuration](https://ardupilot.org/rover/docs/common-gps-how-to.html) - GPS integration best practices
