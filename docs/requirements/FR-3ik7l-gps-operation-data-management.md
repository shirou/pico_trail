# FR-3ik7l GPS Operation and Data Management

## Metadata

- Type: Functional Requirement
- Status: Completed

## Links

- Related Analyses:
  - [AN-xfiyr-gps-hardware-integration](../analysis/AN-xfiyr-gps-hardware-integration.md)
- Related ADRs:
  - [ADR-00mjv-i2c0-gps-imu-integration](../adr/ADR-00mjv-i2c0-gps-imu-integration.md) (superseded)
  - [ADR-8tp69-uart0-gps-allocation](../adr/ADR-8tp69-uart0-gps-allocation.md)
- Prerequisite Requirements:
  - [FR-qfwhl-gps-i2c-driver](FR-qfwhl-gps-i2c-driver.md) (superseded)
  - [FR-93b5v-gps-uart-driver](FR-93b5v-gps-uart-driver.md)
- Dependent Requirements:
  - [FR-uyie8-gps-mavlink-telemetry](FR-uyie8-gps-mavlink-telemetry.md)
  - [FR-cs42u-gps-navigation-state-access](FR-cs42u-gps-navigation-state-access.md)
  - [FR-e2urj-large-vehicle-magcal](FR-e2urj-large-vehicle-magcal.md)
- Related Tasks:
  - [T-meox8-i2c0-gps-imu-integration](../tasks/T-meox8-i2c0-gps-imu-integration/README.md) (superseded)
  - [T-vxtxn-uart0-gps-integration](../tasks/T-vxtxn-uart0-gps-integration/README.md)

## Requirement Statement

The system shall poll GPS driver at configurable rates (1Hz, 5Hz, 10Hz), validate all received NMEA data (checksum, format, range), handle errors gracefully, provide position data to navigation subsystem within 300ms, and recover from transient failures within 5 seconds without system reboot.

## Rationale

Navigation subsystem (FR-333ym-gps-waypoint-navigation) requires reliable, timely GPS position data with appropriate error handling. This requirement integrates GPS polling, validation, performance, and recovery into a cohesive operational specification, ensuring the system can:

- Adapt polling rate to vehicle speed and performance requirements (1-10Hz)
- Reject invalid/corrupted data that could cause navigation errors
- Deliver position updates within control loop timing constraints (<300ms)
- Automatically recover from temporary GPS signal loss or I2C errors (<5s)

## User Story (if applicable)

As a navigation subsystem, I want to receive validated GPS position data at a configurable rate with guaranteed latency and automatic error recovery, so that I can perform waypoint navigation reliably without manual intervention during transient failures.

## Acceptance Criteria

**Polling & Data Access:**

- [ ] GPS polling task runs at configurable interval: 1Hz (default), 5Hz, or 10Hz
- [ ] Latest GPS position stored in shared state accessible to navigation subsystem
- [ ] GPS fix status checked before using position data (reject NoFix positions)
- [ ] No GPS position used if age exceeds 2 seconds (stale data protection)

**Data Validation:**

- [ ] NMEA sentence checksum validated before parsing (XOR of characters between `$` and `*`)
- [ ] Invalid NMEA sentences discarded and logged as warnings
- [ ] GPS fix type extracted from NMEA sentences (NoFix, 2D, 3D) and validated
- [ ] Position coordinates validated for reasonable ranges (latitude: -90 to 90, longitude: -180 to 180)
- [ ] I2C communication errors (NACK, timeout) logged and trigger GPS unavailable state

**Performance:**

- [ ] GPS position data available in `GpsState` within 300ms of GPS module fix completion
- [ ] I2C read from NEO-M8N completes within 100ms (includes DDC protocol overhead)
- [ ] NMEA sentence parsing completes within 10ms per sentence
- [ ] GPS polling task latency (scheduling delay) does not exceed 50ms
- [ ] Latency remains <300ms under concurrent IMU I2C communication load

**Error Handling & Recovery:**

- [ ] I2C communication errors detected and retried up to 3 times with exponential backoff (100ms, 200ms, 400ms)
- [ ] After 3 failed I2C retries, GPS driver enters error state and reports GPS unavailable
- [ ] GPS driver automatically resumes polling when I2C communication restored
- [ ] GPS driver continues polling during NoFix periods (no reset required)
- [ ] GPS driver automatically recovers when valid fix resumes (3D or 2D fix)
- [ ] Recovery completes within 5 seconds for transient errors (warm/hot start)
- [ ] Loss of GPS fix (3 consecutive NoFix readings) triggers GPS failsafe (FR-333ym)

**Testing:**

- [ ] Unit tests verify GPS polling at each supported rate (1Hz, 5Hz, 10Hz)
- [ ] Unit tests verify checksum validation, range validation, and error handling
- [ ] Performance test measures end-to-end latency: GPS fix → I2C read → parse → `GpsState` update
- [ ] Integration test confirms automatic recovery from simulated GPS signal loss and I2C errors

## Technical Details

### Functional Requirement Details

**Polling Rates:**

| Rate | Period | Use Case                        | CPU Overhead (estimated) |
| ---- | ------ | ------------------------------- | ------------------------ |
| 1Hz  | 1000ms | Basic navigation, slow vehicles | Minimal (<1%)            |
| 5Hz  | 200ms  | Standard navigation, rovers     | Low (\~2%)               |
| 10Hz | 100ms  | High-speed vehicles, boats      | Medium (\~5%)            |

**GPS Position State:**

```rust
pub struct GpsState {
    pub position: Option<GpsPosition>,
    pub last_update: Instant,
    pub fix_type: GpsFixType,
}

pub struct GpsPosition {
    pub latitude: f32,      // Decimal degrees
    pub longitude: f32,     // Decimal degrees
    pub altitude: f32,      // Meters above sea level
    pub fix_type: GpsFixType, // NoFix, Fix2D, Fix3D
    pub satellites: u8,     // Number of satellites used
    pub timestamp: u32,     // Milliseconds since system start
}
```

**NMEA Checksum Validation:**

- Format: `$<sentence>*<checksum>\r\n`
- Checksum: XOR of all characters between `$` and `*`
- Example: `$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A`
  - Checksum `6A` = XOR of `GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W`

**GPS Fix Type Validation:**

From GPGGA sentence:

- Field 6: Fix quality (0=No fix, 1=GPS fix, 2=DGPS fix)
- NoFix: Reject position data, continue polling

From GPRMC sentence:

- Field 2: Status (A=Active/Valid, V=Void/Invalid)
- V (Void): Reject position data, continue polling

**Position Range Validation:**

- Latitude: -90.0 to +90.0 degrees
- Longitude: -180.0 to +180.0 degrees
- Altitude: -1000 to +10000 meters (reject unrealistic values)

**Latency Budget Breakdown:**

| Component                | Target Latency  | Notes                                         |
| ------------------------ | --------------- | --------------------------------------------- |
| GPS fix latency          | 100-200ms       | NEO-M8N internal processing (inherent)        |
| I2C read (DDC protocol)  | 10-50ms         | Read register 0xFF + data stream at 400kHz    |
| NMEA parsing             | <10ms           | String parsing and checksum validation        |
| State update             | <5ms            | Mutex lock and copy to shared state           |
| Task scheduling overhead | <50ms           | Embassy async task wake-up delay              |
| **Total (worst case)**   | **\~315ms**     | Slightly exceeds target; optimize I2C/parsing |
| **Total (typical)**      | **\~180-260ms** | Within target                                 |

**I2C Error Retry Strategy:**

| Attempt | Backoff Delay | Cumulative Time | Action                                     |
| ------- | ------------- | --------------- | ------------------------------------------ |
| 1       | 0ms           | 0ms             | Initial I2C read                           |
| 2       | 100ms         | 100ms           | Retry after 100ms                          |
| 3       | 200ms         | 300ms           | Retry after 200ms                          |
| 4       | 400ms         | 700ms           | Retry after 400ms                          |
| Fail    | -             | 700ms           | Enter error state, continue polling at 1Hz |

**Error Handling:**

| Error Type              | Detection Method                       | Action                                            | Recovery                   |
| ----------------------- | -------------------------------------- | ------------------------------------------------- | -------------------------- |
| Invalid checksum        | Checksum mismatch                      | Discard sentence, log warning, continue polling   | Next valid sentence        |
| Malformed sentence      | Parse failure                          | Discard sentence, log warning, continue polling   | Next valid sentence        |
| NoFix status            | GPGGA field 6 = 0 or GPRMC field 2 = V | Set `GpsState.fix_type = NoFix`, continue polling | GPS acquires fix           |
| I2C communication error | I2C NACK, timeout                      | Set GPS unavailable, log error, retry I2C         | I2C communication restored |
| Out-of-range position   | Lat/Lon outside valid range            | Discard position, log error, continue polling     | Next valid position        |
| GPS loss (3x NoFix)     | 3 consecutive NoFix readings           | Trigger GPS failsafe (FR-333ym)                   | GPS fix restored           |

**Error Logging:**

```rust
crate::log_warn!("GPS: Invalid checksum (expected: {}, got: {})", expected, actual);
crate::log_warn!("GPS: Malformed NMEA sentence: {}", sentence);
crate::log_error!("GPS: I2C communication error: {:?}", error);
crate::log_info!("GPS: Fix lost (NoFix), continuing polling");
crate::log_error!("GPS: I2C error, retrying ({}/3)", retry_count);
crate::log_info!("GPS: Recovered, fix acquired (3D fix, {} satellites)", num_sats);
crate::log_error!("GPS: Persistent I2C failure, reducing polling rate");
```

**Recovery Scenarios:**

1. **Transient I2C Error:**
   - Error: I2C NACK on read
   - Recovery: Retry with backoff → I2C communication restored → Resume normal polling
   - Time: 100-700ms (depending on retry success)

2. **GPS Fix Loss (Tunnel):**
   - Error: 3 consecutive NoFix readings
   - Recovery: Continue polling → GPS acquires fix → Resume position updates
   - Time: Variable (0.5-30s depending on GPS signal strength)
   - Target: <5s for temporary signal loss (tested outdoors)

3. **Persistent I2C Failure:**
   - Error: 3 retries failed, I2C bus down
   - Recovery: Continue polling at reduced rate (1Hz) → I2C restored → Resume normal rate
   - Time: <5s after I2C bus restoration

## Platform Considerations

### Unix

N/A – Embedded targets only (RP2040, RP2350)

### Windows

N/A – Embedded targets only (RP2040, RP2350)

### Cross-Platform

- GPS operation identical on Pico W (RP2040) and Pico 2 W (RP2350)
- Embassy async runtime provides platform-independent timer and task scheduling
- Latency target must be met on both RP2040 (133MHz, no FPU) and RP2350 (150MHz, FPU)
- RP2040 may have slightly higher parsing latency due to slower CPU and lack of FPU
- Validation logic platform-independent, uses standard Rust error handling
- I2C retry logic platform-independent (uses `I2cInterface` trait)
- Host tests use simulated GPS data with controlled update rates and mock I2C with injected errors

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                                    | Validation                                 |
| ----------------------------------------- | ------ | ---------- | ------------------------------------------------------------- | ------------------------------------------ |
| High polling rate causes CPU starvation   | Medium | Low        | Benchmark CPU usage at 10Hz, reduce rate if >10% overhead     | Measure CPU utilization during GPS polling |
| GPS fix loss during operation             | High   | Medium     | Implement stale data timeout, trigger failsafe on loss        | Test GPS loss scenario (indoor movement)   |
| GPS cold start delay causes no data       | Low    | High       | Log "Waiting for GPS fix" message, delay navigation start     | Test GPS startup sequence                  |
| I2C bus contention with IMU exceeds 300ms | Medium | Low        | Use async I2C, allow task yielding; measure concurrent load   | Test GPS + IMU simultaneous reads          |
| Invalid GPS data causes navigation errors | High   | Low        | Validate checksum, fix type, and coordinate ranges            | Unit tests with corrupted NMEA data        |
| GPS loss not detected promptly            | High   | Medium     | Trigger failsafe after 3 consecutive NoFix readings (\~3s)    | Test GPS loss scenario                     |
| I2C errors interpreted as GPS loss        | Medium | Low        | Distinguish I2C errors from NoFix status, retry I2C           | Inject I2C errors in test                  |
| Excessive error logging floods console    | Low    | Medium     | Rate-limit warnings (max 1 per second), log errors always     | Test with continuous invalid data          |
| NMEA parsing slower than expected         | Low    | Low        | Optimize parser, avoid allocations; benchmark on RP2040       | Measure parsing time with real data        |
| Embassy task scheduling delay >50ms       | Medium | Low        | Increase GPS task priority; measure scheduling latency        | Log task wake-up delays                    |
| GPS fix latency varies by environment     | Low    | High       | Accept inherent GPS latency; focus on reducing I2C/parsing    | Test indoors/outdoors, log GPS timestamps  |
| Persistent I2C error prevents recovery    | High   | Low        | Continue polling at reduced rate, log error state             | Test with disconnected GPS module          |
| GPS cold start exceeds 5s target          | Medium | High       | Accept longer recovery for cold start; 5s target for warm/hot | Test GPS cold start (30s typical)          |
| Excessive retries cause CPU starvation    | Low    | Low        | Limit to 3 retries, use exponential backoff                   | Measure CPU usage during retry loop        |
| I2C bus lockup requires hardware reset    | High   | Very Low   | Implement I2C bus reset logic (clock stretch recovery)        | Test with I2C bus fault injection          |

## Implementation Notes

- GPS polling task should run in separate async task, not in main control loop
- Consider using `embassy_time::Ticker` for precise periodic polling
- GPS position data should be protected by `Mutex` or `Channel` for safe concurrent access
- 10Hz polling requires NEO-M8N configuration via UBX protocol (defer to future enhancement; current implementation uses default 1Hz GPS output)
- Indoor testing may not achieve GPS fix; use outdoor environment or GPS simulator
- NMEA checksum calculation: XOR accumulator over sentence characters
- GPS fix type mapping: GPGGA field 6 or GPRMC field 2 → `GpsFixType` enum
- Consider using `nom` or `pest` parser for robust NMEA sentence parsing (defer to implementation decision)
- GPS failsafe trigger logic belongs in navigation subsystem, not GPS driver (separation of concerns)
- Indoor GPS testing will naturally produce NoFix readings; verify error handling behavior
- Latency measurement should be conditional (e.g., `#[cfg(debug_assertions)]`) to avoid overhead in release builds
- Consider using `defmt::trace!` for high-frequency latency logging (less overhead than `log_debug!`)
- GPS fix timestamp not directly accessible from NMEA (requires UBX protocol); use system timestamp as proxy
- If latency consistently exceeds 300ms, consider:
  - Reducing I2C clock to 100 kHz (may improve reliability at cost of latency)
  - Simplifying NMEA parser (parse only essential fields)
  - Increasing GPS polling interval to reduce contention (trade-off with update rate)
- GPS cold start (first fix after power-on) can take 30+ seconds; 5-second recovery target applies to transient errors (warm/hot start)
- I2C retry logic should use async delays (`Timer::after`) to avoid blocking other tasks
- Consider implementing I2C bus reset (toggle SCL/SDA) if bus lockup detected (advanced error recovery)
- GPS failsafe (FR-333ym) should trigger after 3 consecutive NoFix readings (\~3s), but allow recovery if fix resumes
- Test recovery behavior in realistic scenarios:
  - Drive vehicle into building (GPS loss) then back outside
  - Disconnect/reconnect I2C wiring during operation
  - Inject I2C errors via fault injection framework (if available)

## External References

- [NEO-M8 Series Datasheet (UBX-15031086)](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf) - GPS fix latency specifications
- [ArduPilot GPS Configuration](https://ardupilot.org/rover/docs/common-gps-how-to.html) - GPS update rate recommendations
- [NMEA 0183 Standard](https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard) - Sentence format and checksum
- [NMEA Sentence Reference](https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_MessageOverview.html) - GPGGA, GPRMC fields
- [ArduPilot GPS Failsafe](https://ardupilot.org/rover/docs/rover-failsafes.html#gps-failsafe) - Industry standard GPS loss handling
- [I2C Specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf) - I2C timing, speed modes, error conditions and recovery
- [embassy-rs Documentation](https://embassy.dev/) - Async runtime and timer usage
