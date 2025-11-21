# AN-xfiyr GPS and IMU Hardware Integration via I2C with Limited GPIO Pins

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-7ix56-navigation-approach](../analysis/AN-7ix56-navigation-approach.md)
- Related Requirements:
  - [FR-333ym-gps-waypoint-navigation](../requirements/FR-333ym-gps-waypoint-navigation.md)
  - [FR-2f599-i2c0-multi-sensor-bus](../requirements/FR-2f599-i2c0-multi-sensor-bus.md) (superseded)
  - [FR-qfwhl-gps-i2c-driver](../requirements/FR-qfwhl-gps-i2c-driver.md) (superseded)
  - [FR-93b5v-gps-uart-driver](../requirements/FR-93b5v-gps-uart-driver.md)
  - [FR-3ik7l-gps-operation-data-management](../requirements/FR-3ik7l-gps-operation-data-management.md)
- Related ADRs:
  - [ADR-00mjv-i2c0-gps-imu-integration](../adr/ADR-00mjv-i2c0-gps-imu-integration.md) (superseded)
  - [ADR-8tp69-uart0-gps-allocation](../adr/ADR-8tp69-uart0-gps-allocation.md)
- Related Tasks:
  - [T-meox8-i2c0-gps-imu-integration](../tasks/T-meox8-i2c0-gps-imu-integration/README.md) (superseded)
  - [T-vxtxn-uart0-gps-integration](../tasks/T-vxtxn-uart0-gps-integration/README.md)

## Discovery Update (2025-11-20)

**GPS Module Interface Clarification:**

During implementation of the I2C-based GPS integration (T-meox8), it was discovered that the NEO-M8N GPS module in use actually connects via **UART interface, not I2C/DDC**. This discovery requires revising the hardware integration approach from I2C0 multi-sensor bus to UART0 GPS with separate I2C0 for IMU.

**Key Changes:**

- GPS connection: UART0 (9600 baud, 8N1) on GPIO 0 (RX), GPIO 1 (TX)
- IMU connection: I2C0 remains available for BNO085 IMU on alternative GPIO pins
- I2cInterface trait remains intact for future IMU integration
- UART GPS driver already exists and is functional (`src/devices/gps.rs`)

**Impact on Previous Analysis:**

- Original analysis correctly identified UART0 as viable option (see "Option 1" below)
- NEO-M8N hardware supports both UART and I2C, but this specific module uses UART
- No changes needed to USB Serial logging approach (remains primary debug interface)

**Status:** This document has been updated to reflect UART0 as the primary GPS interface. I2C-specific sections are retained for historical context and future IMU integration reference.

## Executive Summary

This analysis evaluates the feasibility and approach for integrating a NEO-M8N GPS module and BNO085 9-axis IMU with the pico_trail autopilot under strict GPIO pin constraints. The available pins (GPIO 0, 1, 17, 22) support both UART0 and I2C0 interfaces on GPIO 0 and 1. **After hardware verification, the NEO-M8N GPS module in use connects via UART interface (9600 baud, 8N1 format).** **The recommended approach is UART0-based connection for GPS on GPIO 0 (RX) and GPIO 1 (TX)**, which provides reliable NMEA sentence reception for waypoint navigation (FR-333ym) while maintaining USB Serial for debug logging (`usb_serial` feature in `src/core/logging.rs`). I2C0 remains available for future BNO085 IMU integration on alternative GPIO pins.

## Problem Space

### Current State

The pico_trail project has the following GPS-related components in place:

- **GPS driver implementation**: `src/devices/gps.rs` provides a platform-independent UART-based NEO-M8N driver with NMEA sentence parsing (GPRMC, GPGGA)
- **Platform abstraction**: `UartInterface` trait enables GPS driver to work across RP2040 and RP2350
- **Navigation requirements**: FR-333ym specifies GPS waypoint navigation with S-curve path planning
- **No hardware integration specification**: No documentation or configuration exists for connecting physical GPS hardware to Pico 2 W

**Current GPIO usage (Freenove Standard board):**

- GPIO 0, 1: Reserved for UART0 (serial debug console)
- GPIO 2-21: Motor control (DRV8837 H-bridge drivers)
- GPIO 16, 18, 19: SPI0 (currently configured but potentially unused)
- GPIO 26: Battery ADC monitoring
- GPIO 23-25, 29: WiFi (CYW43439 on Pico 2 W)

**Available pins for GPS:**

- GPIO 0, 1, 17, 22
- GND, VCC (3.3V)

### Desired State

- Physical NEO-M8N GPS module connected to Pico 2 W via UART0 (GPIO 0, 1)
- GPS driver receiving NMEA sentences at 1-10Hz via UART interface (9600 baud, 8N1)
- BNO085 IMU available for future integration via I2C0 on alternative GPIO pins
- UART0 initialized and configured for GPS communication
- USB Serial debug logging maintained (via `usb_serial` feature in `src/core/logging.rs`)
- GPS position data available to navigation subsystem (FR-333ym)
- Reliable operation on both Pico W (RP2040) and Pico 2 W (RP2350)

### Gap Analysis

Missing components:

1. **Hardware wiring specification**: UART0 GPIO pin assignments for GPS connection
2. **UART0 initialization code**: Platform-specific UART0 setup in `src/platform/rp2350/uart.rs` (already exists)
3. **GPS operation manager**: Integration of existing GPS UART driver (`src/devices/gps.rs`) with polling loop, error recovery, and state management
4. **Board configuration update**: `boards/freenove_standard.hwdef` should document GPIO 0, 1 for UART0 GPS
5. **Hardware integration testing**: Validation of GPS NMEA reception via UART0
6. **Performance validation**: GPS update rate and UART latency measurement

## Stakeholder Analysis

| Stakeholder          | Interest/Need                                        | Impact | Priority |
| -------------------- | ---------------------------------------------------- | ------ | -------- |
| Navigation subsystem | GPS position data at 1-10Hz for waypoint navigation  | High   | P0       |
| AHRS subsystem       | GPS velocity for heading estimation (when available) | Medium | P1       |
| Control modes (Auto) | Reliable GPS data for autonomous operation           | High   | P0       |
| Developers           | Alternative debug logging after UART0 reallocation   | High   | P0       |
| System integrator    | Simple wiring, clear documentation                   | Medium | P1       |
| Hardware tester      | GPS signal validation in indoor/outdoor environments | Medium | P2       |

## Research & Discovery

### User Feedback

N/A - This is a hardware integration task driven by existing requirements (FR-333ym).

### Competitive Analysis

**ArduPilot GPS Integration:**

- Primary GPS interface: UART (serial)
- Typical baud rate: 9600 (u-blox default) or 38400 (high-rate mode)
- Supported protocols: NMEA (basic), u-blox UBX binary (advanced, requires driver extension)
- Update rate: 1Hz (standard), 5Hz (recommended), 10Hz (high-performance)
- GPS failsafe: Loss of GPS fix triggers Hold or RTL mode

**PX4 Autopilot:**

- Similar UART-based approach
- Uses dedicated GPS UART port separate from debug console
- Supports GPS+Compass combo modules (GPS with integrated magnetometer)

**Hobby/Educational Projects:**

- Most use NEO-M8N or NEO-M9N (u-blox GPS modules)
- UART interface universal across microcontrollers
- NMEA parsing in software (no dedicated GPS hardware accelerator)

### Technical Investigation

#### RP2350 Pin Constraints Analysis

**Available Communication Interfaces with GPIO 0, 1, 17, 22:**

| Interface | Pin Configuration     | Feasibility        | Notes                                                     |
| --------- | --------------------- | ------------------ | --------------------------------------------------------- |
| UART0     | GPIO 0 (RX), 1 (TX)   | ✅ Possible        | Primary UART, currently used for debug                    |
| UART1     | GPIO 4 (RX), 5 (TX)   | ❌ Not available   | GPIO 4, 5 not in available pin list                       |
| SPI0      | SCK, MOSI, MISO, CS   | ❌ Not possible    | Requires GPIO 2/18 (SCK), 3/19 (MOSI), 0/4/16 (MISO)      |
| SPI1      | SCK, MOSI, MISO, CS   | ❌ Not possible    | Requires GPIO 10/26 (SCK), 11/27 (MOSI), 8/12/28 (MISO)   |
| I2C0      | GPIO 0 (SDA), 1 (SCL) | ✅ Fully supported | NEO-M8N I2C/DDC fully functional (400 kHz, all protocols) |
| I2C1      | GPIO 2 (SDA), 3 (SCL) | ❌ Not available   | GPIO 2, 3 not in available pin list                       |

**Conclusion:** Both UART0 and I2C0 (GPIO 0, 1) are viable interfaces for GPS connection. I2C0 is preferable when sharing pins with other I2C devices (e.g., BNO085 IMU at address 0x4A).

#### NEO-M8N GPS Module Specifications

**Interface:**

- Primary: UART (3.3V TTL serial)
- Secondary: I2C/DDC (address 0x42, **fully functional**)
  - I2C Fast Mode compliant (up to 400 kHz)
  - All protocols supported: NMEA, UBX, RTCM
  - Same functionality as UART interface
  - Reference: NEO-M8 Datasheet Section 1.17.4 (Page 13), Section 1.16 (Page 12)
- Tertiary: SPI (slave mode, D_SEL=0)
  - Max transfer rate: 125 kB/s (max SPI clock 5.5 MHz)
  - All protocols supported: NMEA, UBX, RTCM
  - Reference: NEO-M8 Datasheet Section 1.17.3 (Page 12), Table 8 (Page 17)

**Interface Selection (D_SEL pin):**

- D_SEL = HIGH or OPEN: UART + I2C/DDC available (default for this project)
- D_SEL = GND: SPI available (UART and I2C disabled)
- Reference: NEO-M8 Datasheet Section 3.1 (Page 17), Table 8

**UART Parameters:**

- Default baud rate: 9600 bps
- Configurable baud rates: 4800, 9600, 19200, 38400, 57600, 115200
- Data format: 8N1 (8 data bits, no parity, 1 stop bit)
- Output: NMEA 0183 sentences (GPGGA, GPRMC, GPGSV, etc.)

**Update Rate:**

- Default: 1Hz (1 position per second)
- Configurable: Up to 10Hz (requires UBX configuration, not supported by current driver)

**Electrical:**

- Supply voltage: 3.3V (compatible with Pico 2 W)
- Current consumption: \~25mA (active), \~10mA (power save)

#### Debug Logging Alternatives

With UART0 allocated to GPS, the default serial console debug interface is unavailable. Alternative logging mechanisms:

| Method             | Description                            | Pros                                                 | Cons                                   | Feasibility     |
| ------------------ | -------------------------------------- | ---------------------------------------------------- | -------------------------------------- | --------------- |
| USB CDC-ACM        | Virtual serial port over USB           | No extra hardware, no GPIO pins, already implemented | Requires USB connection                | ✅ Recommended  |
| `defmt` + probe-rs | Structured logging via SWD debug probe | No GPIO pins required, fast                          | Requires hardware debugger (probe-rs)  | ⚠️ Fallback     |
| UART1              | Secondary UART on GPIO 4, 5            | Familiar serial console                              | GPIO 4, 5 not available                | ❌ Not possible |
| WiFi telemetry     | Log over WiFi (TCP/UDP)                | Wireless, flexible                                   | Complex, unreliable during development | ❌ Not suitable |

**Current project logging strategy:**

- `src/core/logging.rs` provides abstraction macros (`crate::log_info!`, etc.)
- Embedded targets with `usb_serial` feature (primary): USB CDC-ACM virtual serial port
- Embedded targets without `usb_serial` (fallback): `defmt` logging via probe-rs
- Host tests: Use `println!`
- **Recommendation:** Continue using USB Serial as primary debugging mechanism (standard terminal emulator compatible)

#### UART0 Initialization on RP2350

**Current platform UART implementation:**

- `src/platform/rp2350/uart.rs`: Wraps `rp235x_hal::uart::UartPeripheral`
- Supports configurable baud rate, parity, stop bits
- DMA support available but not required for 9600 baud GPS

**Required initialization for GPS:**

```rust
// Pseudo-code for UART0 GPS setup
let uart0 = hal::uart::UartPeripheral::new(
    pac.UART0,
    pins.gpio0, // RX
    pins.gpio1, // TX
    9600.Hz(),  // Baud rate
    &mut pac.RESETS,
);
```

### Data Analysis

**GPS Data Rate and Latency Budget:**

| Item                    | Value       | Notes                                         |
| ----------------------- | ----------- | --------------------------------------------- |
| GPS update rate         | 1Hz-10Hz    | NEO-M8N default 1Hz, configurable up to 10Hz  |
| NMEA sentence length    | 80 bytes    | Typical GPGGA or GPRMC sentence               |
| UART transmission time  | \~83ms      | At 9600 baud: 80 bytes \* 10 bits/byte / 9600 |
| GPS fix latency         | 100-200ms   | Internal processing delay                     |
| Total latency           | \~180-280ms | GPS fix + transmission                        |
| Navigation control loop | 50Hz        | FR-333ym specifies 50Hz trajectory updates    |

**Conclusion:** GPS latency (180-280ms) is acceptable for navigation control at 50Hz. Position prediction/interpolation may be needed for smooth trajectory following.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: UART0 GPS receiver initialization and configuration → Will become FR-\<id>
  - Rationale: GPS driver (`src/devices/gps.rs`) requires UART interface to receive NMEA sentences from NEO-M8N
  - Acceptance Criteria:
    - UART0 initialized with 9600 baud, 8N1 format
    - GPIO 0 (RX) and GPIO 1 (TX) configured for UART0 function
    - UART0 available to GPS driver via `UartInterface` trait
    - Works on both Pico W (RP2040) and Pico 2 W (RP2350)

- [ ] **FR-DRAFT-2**: GPS position data polling at configurable rate → Will become FR-\<id>
  - Rationale: Navigation subsystem needs GPS data at 1-10Hz for waypoint following (FR-333ym)
  - Acceptance Criteria:
    - GPS driver polls UART0 for NMEA sentences
    - Valid GPS position extracted when fix available
    - Position data accessible to navigation subsystem
    - Configurable update rate (1Hz, 5Hz, 10Hz)

- [ ] **FR-DRAFT-3**: GPS data validation and error handling → Will become FR-\<id>
  - Rationale: Invalid or missing GPS data should not crash the system or cause unsafe behavior
  - Acceptance Criteria:
    - GPS driver detects invalid NMEA sentences (checksum validation)
    - GPS driver detects loss of GPS fix (status field in GPRMC/GPGGA)
    - GPS driver reports error state to navigation subsystem
    - Navigation subsystem triggers failsafe on GPS loss (per FR-333ym)

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: GPS data reception latency shall not exceed 300ms → Will become NFR-\<id>
  - Category: Performance
  - Rationale: Navigation control loop (50Hz) requires timely GPS updates for trajectory following
  - Target: <300ms from GPS fix to position available in driver (includes UART transmission and parsing)

- [ ] **NFR-DRAFT-2**: Debug logging shall use USB Serial (CDC-ACM) on embedded targets → Will become NFR-\<id>
  - Category: Usability
  - Rationale: UART0 allocated to GPS, serial console unavailable; USB Serial provides standard logging interface
  - Target: All embedded logging uses `crate::log_*!` macros with `usb_serial` feature (defined in `src/core/logging.rs`)

- [ ] **NFR-DRAFT-3**: GPS driver shall recover from transient UART errors within 5 seconds → Will become NFR-\<id>
  - Category: Reliability
  - Rationale: Temporary GPS signal loss (e.g., tunnels, buildings) should not require system reboot
  - Target: GPS driver detects UART errors, resets internal buffer, and resumes parsing within 5 seconds

## Design Considerations

### Technical Constraints

**Hardware:**

- RP2040 (Pico W): Cortex-M0+, 133MHz, no FPU, 264KB RAM
- RP2350 (Pico 2 W): Cortex-M33, 150MHz, FPU, 520KB RAM
- GPIO pin availability: Only GPIO 0, 1, 17, 22 available for GPS

**Software:**

- Existing GPS driver: `src/devices/gps.rs` (UART-based, NMEA parsing)
- Platform abstraction: `UartInterface` trait in `src/platform/traits.rs`
- Logging: `defmt` on embedded targets, `println!` on host tests
- Embassy async runtime for task management

**Electrical:**

- NEO-M8N requires 3.3V power supply
- UART signal levels: 3.3V TTL (compatible with RP2350)
- Current draw: \~25mA active (negligible for USB-powered Pico)

### Potential Approaches

#### Option 1: UART0 GPS Connection (Recommended)

**Description:** Allocate GPIO 0, 1 to UART0 for NEO-M8N GPS communication (9600 baud, 8N1), using USB Serial for debug logging as already implemented in `src/core/logging.rs`. I2C0 remains available for future IMU integration on alternative GPIO pins.

**Pros:**

- **Feasible with available pins**: Only requires GPIO 0, 1
- **GPS driver already exists**: `src/devices/gps.rs` provides full UART NMEA parsing
- **UART platform abstraction complete**: `UartInterface` trait, RP2350 implementation, and MockUart all functional
- **No additional hardware**: USB Serial uses existing USB port (no extra GPIO or debug probe)
- **Industry standard**: UART is universal GPS interface across all autopilots (ArduPilot, PX4)
- **Familiar debugging**: Standard terminal emulator compatible with USB Serial
- **Simple protocol**: Direct NMEA sentence streaming, no bus arbitration needed
- **Future-proof**: Leaves GPIO 17, 22 available for additional peripherals; I2C0 available for IMU on other pins

**Cons:**

- **Limited expansion**: UART0 occupied by GPS, UART1 pins not available in current pin set
- **Single sensor per UART**: Cannot share UART bus like I2C (but IMU can use I2C0 separately)

**Effort:** Low

- UART0 initialization: 0 hours (already exists in `src/platform/rp2350/uart.rs`)
- GPS operation manager: 2-4 hours (integrate existing UART GPS driver with polling loop, error recovery)
- USB Serial logging: 0 hours (already project standard with `usb_serial` feature)
- Documentation: 1 hour (`boards/freenove_standard.hwdef` update for GPIO 0, 1 UART0 allocation)
- Testing: 2-3 hours (hardware validation, NMEA reception, indoor/outdoor GPS fix)

#### Option 2: Expand GPIO Pin Access (Out of Scope)

**Description:** Redesign hardware to expose additional GPIO pins (e.g., GPIO 4, 5, 8-15) to enable UART1 or SPI1 for GPS, leaving UART0 for debug console.

**Pros:**

- Preserves UART0 for traditional serial console debugging
- More flexible sensor expansion options (I2C, SPI, additional UARTs)

**Cons:**

- **Requires hardware redesign**: Breadboard rewiring or custom PCB
- **Not compatible with user's constraint**: User explicitly stated only GPIO 0, 1, 17, 22 available
- **Delayed timeline**: Hardware changes add weeks to project schedule
- **Unnecessary**: USB Serial logging already provides debug interface

**Effort:** High (out of scope)

### Architecture Impact

**ADR Required:**

- **ADR-\<id>-uart0-gps-allocation**: Documenting the decision to allocate UART0 to GPS and use USB Serial as primary debug interface
  - Context: Limited GPIO pins, GPS requires UART
  - Decision: UART0 for GPS, USB Serial for logging
  - Consequences: No UART0 serial console, requires USB connection for debugging

**Decisions to Make:**

1. ✅ UART0 for GPS (decided: yes, only viable option)
2. ✅ USB Serial for debug logging (decided: yes, already project standard with `usb_serial` feature)
3. ⏳ GPS baud rate: 9600 (default) or 38400 (higher throughput)? → Recommend 9600 for compatibility
4. ⏳ GPS task priority: Same as AHRS (400Hz) or lower (10Hz)? → Recommend separate 10Hz task
5. ⏳ GPS data buffering: Single-position or circular buffer? → Recommend single latest position (no buffering)

## Risk Assessment

| Risk                                              | Probability | Impact | Mitigation Strategy                                                                |
| ------------------------------------------------- | ----------- | ------ | ---------------------------------------------------------------------------------- |
| Loss of UART0 console complicates debugging       | Low         | Low    | Use USB Serial for logging (already implemented); standard terminal compatible     |
| GPS signal unavailable indoors during testing     | High        | Low    | Test outdoors or near window; use NMEA sentence injection for unit testing         |
| UART0 baud rate mismatch causes garbled data      | Low         | Medium | Configure UART0 to NEO-M8N default (9600 baud); validate with known NMEA data      |
| GPIO 0, 1 conflict with existing code             | Low         | High   | Audit `boards/freenove_standard.hwdef` and `src/platform/rp2350/uart.rs` first     |
| GPS driver parsing errors with NEO-M8N output     | Medium      | Medium | Validate parser with real NEO-M8N NMEA sentences; improve error handling           |
| USB Serial logging adds latency to critical paths | Low         | Low    | USB Serial runs in separate task; non-blocking channel prevents log-induced delays |

## Open Questions

- [ ] Should GPS task run at 1Hz, 5Hz, or 10Hz? → Next step: Benchmark CPU overhead at each rate on RP2040/RP2350
- [ ] Should we implement UBX protocol for advanced GPS configuration (10Hz mode)? → Decision: Defer to future enhancement, NMEA sufficient for initial implementation
- [ ] How to handle GPS cold start delay (up to 30 seconds)? → Method: Add GPS status monitoring, delay navigation mode entry until fix acquired
- [ ] Should GPS position be logged to storage for post-flight analysis? → Next step: Create requirement for telemetry logging (separate from this GPS integration)

## Recommendations

### Immediate Actions

1. **Select UART0 GPS connection approach** as primary implementation:
   - GPIO 0 (UART0 RX) → NEO-M8N TX
   - GPIO 1 (UART0 TX) → NEO-M8N RX
   - Baud rate: 9600 (NEO-M8N default), 8N1 format
   - Rationale: GPS module uses UART interface; UART GPS driver already exists and is functional
   - Continue using USB Serial (`usb_serial` feature) for debug logging (already project standard)

2. **Document hardware wiring** in `boards/freenove_standard.hwdef`:
   - Mark GPIO 0, 1 as allocated to UART0 (GPS)
   - Specify NEO-M8N connection: GPIO 0 (RX), GPIO 1 (TX), 9600 baud
   - Note I2C0 remains available for future BNO085 IMU on alternative GPIO pins

3. **Integrate existing UART GPS driver**:
   - Reuse `src/devices/gps.rs` (UART-based GPS driver with NMEA parsing)
   - Implement GPS operation manager with polling loop, error recovery, state management
   - Add unit tests for UART-based NMEA sentence reception and error handling

### Next Steps

1. [ ] Create formal requirements based on FR-DRAFT and NFR-DRAFT above (updated for UART)
2. [ ] Draft ADR: UART0 allocation to GPS (supersedes ADR-00mjv I2C0 decision)
3. [ ] Create task: T-\<id>-uart0-gps-integration
   - Phase 1: GPS operation manager implementation (polling, error recovery, state management)
   - Phase 2: Board configuration update (GPIO 0, 1 UART0 allocation)
   - Phase 3: Unit testing (NMEA parsing, error recovery, failsafe)
   - Phase 4: Hardware validation (indoor/outdoor GPS fix, latency measurement)
4. [ ] Update `boards/freenove_standard.hwdef` with GPIO 0, 1 allocation to UART0 GPS
5. [ ] Document USB Serial logging workflow in developer guide (already active via `usb_serial` feature)
6. [ ] Mark superseded documents (FR-qfwhl, ADR-00mjv, T-meox8) with "Superseded" status

### Out of Scope

- **Multi-GPS redundancy**: Single GPS sufficient for rover/boat (not aircraft)
- **UBX binary protocol**: NMEA parsing adequate for 1-10Hz updates
- **GPS+Compass combo module**: Magnetometer integration is separate concern (future work)
- **RTK GPS (centimeter accuracy)**: Waypoint navigation (FR-333ym) specifies 2m acceptance radius
- **Alternative debug interfaces (USB CDC-ACM)**: `defmt` + probe-rs is current standard, USB stack not yet implemented

## Appendix

### References

**Datasheets:**

- [u-blox NEO-M8 Series Datasheet (UBX-15031086)](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf)
  - Section 1.17.4 "Display data channel (DDC)" - Page 13: I2C/DDC interface specification
  - Section 1.16 "Protocols and interfaces" - Page 12: Confirms all protocols (NMEA, UBX, RTCM) available on I2C/DDC
  - Table 4 - Page 12: Protocol availability across all interfaces
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)

**NMEA Protocol:**

- [NMEA 0183 Standard](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard)
- [NMEA Sentence Reference](https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_MessageOverview.html)

**ArduPilot GPS Integration:**

- [ArduPilot GPS Configuration](https://ardupilot.org/rover/docs/common-gps-how-to.html)
- [ArduPilot GPS Selection Guide](https://ardupilot.org/copter/docs/common-choosing-a-gps.html)

**Embedded Rust Logging:**

- [defmt Framework](https://defmt.ferrous-systems.com/)
- [probe-rs Documentation](https://probe.rs/)

### Raw Data

**GPIO Pin Availability Confirmation (from initial investigation):**

```
Available GPIO pins: 0, 1, 17, 22
GND, VCC (3.3V) also available

RP2350 UART0 Pin Options:
- UART0 TX: GPIO 0, 12, 16, 28
- UART0 RX: GPIO 1, 13, 17, 29

Selected: GPIO 0 (RX), GPIO 1 (TX) → Only option within available pins
```

**NEO-M8N NMEA Output Example (captured from real module):**

```
$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
```

**UART Transmission Timing (9600 baud):**

- 1 character = 10 bits (8N1 format)
- 1 bit time = 1/9600 = 104 μs
- 1 character time = 1.04 ms
- 80 characters (typical NMEA sentence) = 83.2 ms

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
