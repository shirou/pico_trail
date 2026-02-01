# T-00015 I2C0 Multi-Sensor Bus Integration Plan

## Metadata

- Type: Implementation Plan
- Status: Superseded

**This task has been superseded by [T-00016-uart0-gps-integration](../T-00016-uart0-gps-integration/README.md) due to hardware verification that the GPS module uses UART interface, not I2C.**

## Links

- Associated Design Document:
  - [T-00015-i2c0-gps-imu-integration-design](./design.md)

## Overview

Implement I2C0 multi-sensor bus for NEO-M8N GPS (0x42) and BNO085 IMU (0x4A) on GPIO 0/1. Enable GPS position data at 1-10 Hz with NMEA parsing, validation, and automatic error recovery. Support waypoint navigation under GPIO pin constraints while maintaining USB Serial debug logging.

## Success Metrics

- [ ] GPS position data received at 1-10 Hz with <300ms latency
- [ ] NMEA validation (checksum, fix type, position range) with 100% invalid data rejection
- [ ] Automatic I2C error recovery within 5 seconds (3 retries, exponential backoff)
- [ ] All existing tests pass; no regressions in control loop timing
- [ ] Hardware validation confirms GPS and IMU coexistence on I2C0 bus

## Scope

- Goal: Enable GPS position data via I2C0 bus for waypoint navigation (FR-00004)
- Non-Goals: IMU driver implementation (future task), UBX protocol support (future enhancement)
- Assumptions: NEO-M8N D_SEL pin HIGH/OPEN, 4.7kΩ pull-ups on SDA/SCL, outdoor GPS testing available
- Constraints: GPIO 0, 1, 17, 22 only; RP2040/RP2350 platform; Embassy async runtime; no_std environment

## ADR & Legacy Alignment

- [ ] Confirm ADR-00019-i2c0-gps-imu-integration governs this work
- [ ] Note: Existing GPS driver (`src/devices/gps.rs`) is UART-only; new I2C driver extends functionality

## Plan Summary

- Phase 1 – I2C0 Initialization and Platform Abstraction
- Phase 2 – GPS I2C/DDC Driver Implementation
- Phase 3 – GPS Operation and Data Management
- Phase 4 – Testing and Verification

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: I2C0 Initialization and Platform Abstraction

### Goal

- Initialize I2C0 bus on GPIO 0 (SDA) and GPIO 1 (SCL) with platform-independent abstraction supporting RP2040 and RP2350

### Inputs

- Documentation:
  - `/docs/adr/ADR-00019-i2c0-gps-imu-integration.md` – I2C0 bus decision
  - `/docs/requirements/FR-00078-i2c0-multi-sensor-bus.md` – I2C0 initialization requirements
  - [NEO-M8 Datasheet Section 1.17.4](https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf) – I2C/DDC specification
  - [RP2350 Datasheet Chapter 4.3](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) – I2C specification
- Source Code to Modify:
  - `/src/platform/traits.rs` – Add `I2cInterface` trait
  - `/src/platform/rp2350/i2c.rs` – RP2350 I2C0 implementation
  - `/src/platform/rp2040/i2c.rs` – RP2040 I2C0 implementation (if supporting Pico W)
- Dependencies:
  - External crates: `embassy-rp` (I2C HAL), `embassy-time` (async delays)

### Tasks

- [x] **Platform abstraction trait**
  - [x] Define `I2cInterface` trait in `src/platform/traits/i2c.rs` with methods: `read`, `write`, `write_read`, `set_frequency`
  - [x] Define `I2cConfig` struct with frequency and timeout settings
  - [x] Add trait documentation with I2C address arbitration notes
- [x] **RP2350 I2C0 implementation**
  - [x] Create `src/platform/rp2350/i2c.rs` wrapping `embassy_rp::i2c::I2c`
  - [x] Initialize I2C0 with GPIO 0 (SDA), GPIO 1 (SCL), 400 kHz clock support
  - [x] Implement `I2cInterface` trait for RP2350 I2C0 (async methods)
  - [x] Add error mapping from `embassy_rp` errors to `PlatformError::I2c`
- [x] **Mock I2C implementation**
  - [x] Create `src/platform/mock/i2c.rs` for testing
  - [x] Implement transaction logging and read data pre-programming
  - [x] Add unit tests (4 tests passing)
- [ ] **RP2040 I2C0 implementation (optional)** - Skipped (not required for current board)
- [x] **Board configuration**
  - [x] Update `boards/freenove_standard.hwdef` to document GPIO 0, 1 allocation to I2C0
  - [x] Add hardware notes: NEO-M8N D_SEL pin HIGH/OPEN, 4.7kΩ pull-ups required, device addresses (GPS 0x42, IMU 0x4A)

### Deliverables

- `I2cInterface` trait with RP2350/RP2040 implementations
- I2C0 initialized at 400 kHz on GPIO 0, 1
- `boards/freenove_standard.hwdef` updated with I2C0 pin assignments

### Verification

```bash
# Build embedded target
./scripts/build-rp2350.sh pico_trail_rover
# Check formatting and linting
cargo fmt
cargo clippy --all-targets -- -D warnings
# Unit tests (host)
cargo test --lib --quiet mock::i2c
```

**Results:**

- ✅ RP2350 embedded build: Success
- ✅ Code formatting: Clean
- ✅ Linting: No warnings
- ✅ Mock I2C unit tests: 4/4 passed

### Acceptance Criteria (Phase Gate)

- [x] I2C0 initialized successfully on RP2350
- [x] `I2cInterface` trait compiles without errors
- [x] Mock I2C implementation with unit tests
- [x] No regressions in existing tests

### Rollback/Fallback

- Revert commits in `src/platform/` if I2C initialization causes build failures
- Fallback: Use UART GPS on GPIO 0, 1 if I2C0 initialization is blocked

---

## Phase 2: GPS I2C/DDC Driver Implementation

### Phase 2 Goal

- Implement NEO-M8N I2C/DDC driver with NMEA sentence parsing and checksum validation

### Phase 2 Inputs

- Dependencies:
  - Phase 1: `I2cInterface` trait and I2C0 initialization
  - Existing code: `src/devices/gps.rs` (NMEA parser)
- Source Code to Modify:
  - `/src/devices/gps_i2c.rs` – New GPS I2C driver
  - `/src/devices/gps.rs` – Extract reusable NMEA parser functions

### Phase 2 Tasks

- [x] **NEO-M8N DDC protocol implementation**
  - [x] Create `GpsI2c` struct in `src/devices/gps_i2c.rs` with `I2cInterface` generic
  - [x] Implement `read_bytes_available()`: Read register 0xFF (2-byte big-endian)
  - [x] Implement `read_data_stream()`: Read register 0xFD up to available bytes
  - [x] Add circular buffer (256 bytes) for NMEA sentence accumulation
- [x] **NMEA sentence parsing**
  - [x] Extract `parse_nmea_sentence()` from `src/devices/gps.rs` into shared module
  - [x] Reuse existing GPGGA parser (latitude, longitude, altitude, fix quality, satellites)
  - [x] Reuse existing GPRMC parser (latitude, longitude, timestamp, status A/V)
  - [x] Implement checksum validation: XOR of characters between `$` and `*`
- [x] **GPS position extraction**
  - [x] Implement `read_nmea()` method: Poll I2C, buffer data, extract complete NMEA sentences
  - [x] Return `GpsPosition` struct with latitude, longitude, altitude, fix type, satellites
  - [x] Handle incomplete sentences across multiple I2C reads (buffer accumulation)
- [x] **Unit tests**
  - [x] Test DDC protocol: Mock I2C with pre-recorded NEO-M8N responses
  - [x] Test NMEA parsing: Valid GPGGA/GPRMC sentences
  - [x] Test checksum validation: Valid and invalid checksums
  - [x] Test sentence buffering: Partial sentences across multiple reads

### Phase 2 Deliverables

- `GpsI2c` driver with NEO-M8N I2C/DDC protocol support
- NMEA sentence parsing (GPGGA, GPRMC) with checksum validation
- Unit tests with mocked I2C transactions

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::gps_i2c
./scripts/build-rp2350.sh pico_trail_rover
```

**Results:**

- ✅ Code formatting: Clean
- ✅ Linting: No warnings
- ✅ GPS I2C unit tests: 8/8 passed
- ✅ RP2350 embedded build: Success

### Phase 2 Acceptance Criteria

- [x] `GpsI2c::read_nmea()` successfully extracts GPS position from mocked I2C data
- [x] NMEA checksum validation rejects invalid sentences
- [x] Sentence buffering handles partial reads correctly
- [x] All unit tests pass

### Phase 2 Rollback/Fallback

- Revert `src/devices/gps_i2c.rs` if GPS I2C driver causes test failures
- Fallback: Continue using UART GPS driver until I2C driver stabilizes

### Phase 2 Refactoring Note

**Date:** 2025-11-19

**Refactoring:** Replaced custom NMEA parsing with `nmea0183` crate (v0.3)

**Rationale:**

- Eliminate \~200 lines of custom NMEA parsing code
- Leverage battle-tested library with automatic checksum validation
- Improve maintainability and reduce potential bugs
- No heap allocations, no_std compatible

**Changes:**

- Added `nmea0183 = { version = "0.3", default-features = false }` to `Cargo.toml`
- Replaced custom `parse_nmea()`, `parse_gpgga()`, `parse_gprmc()` with `nmea0183::Parser`
- Updated field access to use nmea0183 API:
  - `gga.gps_quality` (GPSQuality enum)
  - `gga.altitude.meters` (Altitude struct)
  - `gga.sat_in_use` (u8)
  - `rmc.mode` (Mode enum for validation)

**Verification:**

- ✅ All 8 unit tests pass (including GPGGA, GPRMC, invalid checksum tests)
- ✅ cargo clippy: No warnings
- ✅ cargo test: 385 passed, 1 ignored
- ✅ RP2350 embedded build: Success

---

## Phase 3: GPS Operation and Data Management

### Phase 3 Goal

- Implement GPS polling task with configurable rate (1Hz/5Hz/10Hz), NMEA validation, error recovery, and shared GPS state

### Phase 3 Inputs

- Dependencies:
  - Phase 2: `GpsI2c` driver
  - `embassy-time`: `Ticker` for periodic polling
- Source Code to Modify:
  - `/src/devices/gps_operation.rs` – New GPS operation manager
  - `/src/devices/gps.rs` – Add `GpsState`, `GpsPosition`, `GpsFixType` types

### Phase 3 Tasks

- [x] **GPS state types**
  - [x] Define `GpsState` struct: `position: Option<GpsPosition>`, `last_update: Instant`, `fix_type: GpsFixType`
  - [x] Define `GpsFixType` enum: `NoFix`, `Fix2D`, `Fix3D` (defined in Phase 2)
  - [x] Define `PollingRate` enum: `Rate1Hz`, `Rate5Hz`, `Rate10Hz`
- [x] **GPS polling task**
  - [x] Create `GpsOperation` struct in `src/devices/gps_operation.rs`
  - [x] Implement async `poll_loop()`: Use `embassy_time::Ticker` for periodic polling
  - [x] Call `GpsI2c::read_nmea()` at configured interval (1Hz/5Hz/10Hz)
  - [x] Update shared `GpsState` with latest position
- [x] **NMEA validation**
  - [x] Validate NMEA checksum: Handled by `nmea0183` crate (Phase 2)
  - [x] Validate GPS fix type: Handled in `GpsI2c::read_nmea()` (Phase 2)
  - [x] Validate position ranges: Implemented in `GpsI2c::validate_position()` (Phase 2)
  - [x] Discard invalid data, log error, continue polling
- [x] **Error recovery**
  - [x] Implement I2C retry logic: 3 retries with exponential backoff (100ms, 200ms, 400ms)
  - [x] After 3 failed retries: Enter error state, log error, continue polling
  - [x] NoFix handling: Continue polling, log info when NoFix detected
  - [x] Failsafe trigger: Trigger GPS failsafe after 3 consecutive NoFix readings (\~3s)
- [x] **Logging**
  - [x] Log GPS fix acquired: Implemented in `handle_gps_fix()`
  - [x] Log GPS fix lost: Implemented in `handle_no_data()`
  - [x] Log I2C errors: Implemented in `poll_with_retry()`
  - [x] Log invalid NMEA: Handled in `GpsI2c::read_nmea()` (Phase 2)
- [x] **Unit tests**
  - [x] Test polling rate intervals
  - [x] Test `poll_with_retry()` success
  - [x] Test `handle_gps_fix()` state updates
  - [x] Test `handle_no_data()` failsafe trigger
  - [x] Test state access

### Phase 3 Deliverables

- `GpsOperation` async task with configurable polling rate
- NMEA validation (checksum, fix type, position range)
- I2C error recovery (3 retries, exponential backoff)
- Shared `GpsState` for navigation subsystem access

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::gps_operation
./scripts/build-rp2350.sh pico_trail_rover
```

**Results:**

- ✅ Code formatting: Clean
- ✅ Linting: No warnings
- ✅ GPS operation unit tests: 6/6 passed
- ✅ All library tests: 391 passed, 1 ignored
- ✅ RP2350 embedded build: Success

### Phase 3 Acceptance Criteria

- [x] GPS polling task runs at configured rate (1Hz/5Hz/10Hz)
- [x] NMEA validation rejects invalid data (checksum, fix type, position range)
- [x] I2C error recovery retries with exponential backoff
- [x] Failsafe triggers after 3 consecutive NoFix readings
- [x] All unit tests pass

### Phase 3 Rollback/Fallback

- Revert `src/devices/gps_operation.rs` if GPS operation causes test failures
- Fallback: Use simple GPS polling without error recovery until stabilized

---

## Phase 4: Testing and Verification

### Phase 4 Goal

- Create comprehensive unit tests and validate hardware integration with NEO-M8N GPS module on I2C0 bus

### Phase 4 Tasks

- [x] **Host unit tests**
  - [x] Test `I2cInterface` trait with `MockI2c` (inject I2C errors) - 4 tests passing
  - [x] Test `GpsI2c` driver with pre-recorded NEO-M8N I2C responses - 6 tests passing
  - [x] Test `GpsOperation` with mocked GPS data and ticker - 6 tests passing
  - [x] Test error scenarios: I2C NACK, timeout, invalid NMEA, NoFix status - Covered in above tests
- [ ] **Hardware validation (requires NEO-M8N GPS module)** - Deferred (hardware not available)
  - [ ] Connect NEO-M8N GPS to GPIO 0 (SDA), GPIO 1 (SCL) with 4.7kΩ pull-ups
  - [ ] Set NEO-M8N D_SEL pin HIGH or OPEN to enable I2C/DDC
  - [ ] Test GPS fix acquisition indoors (NoFix expected) and outdoors (3D fix expected)
  - [ ] Measure I2C transaction latency: GPS fix → position available in `GpsState`
  - [ ] Test GPS cold start delay (up to 30 seconds)
  - [ ] Test GPS warm/hot start recovery (<5 seconds)
  - [ ] Disconnect GPS during operation, verify automatic recovery
- [ ] **I2C bus arbitration test** - Deferred (IMU driver future task)
  - [ ] Add BNO085 IMU to I2C0 bus at address 0x4A (placeholder driver)
  - [ ] Verify concurrent GPS and IMU communication without conflicts
  - [ ] Measure I2C bus contention impact on GPS latency (<300ms target)
- [x] **Documentation**
  - [x] Update `docs/architecture.md`: Add I2C0 bus and GPS I2C driver section
  - [x] Add hardware wiring guide: I2C0 connection in `boards/freenove_standard.hwdef`
  - [x] Document GPS testing procedure: Included in architecture.md and design.md

### Phase 4 Deliverables

- Comprehensive unit tests for I2C abstraction, GPS I2C driver, GPS operation
- Hardware validation report: GPS fix acquisition, latency, error recovery
- I2C bus arbitration validation with GPS and IMU coexistence
- Updated documentation: Architecture, hardware wiring, testing procedure

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
# Full test suite
cargo test --lib --quiet
# Embedded build
./scripts/build-rp2350.sh pico_trail_rover
# Hardware validation (manual)
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/gps_test
```

### Phase 4 Acceptance Criteria

- All unit tests pass without hardware
- GPS fix acquired outdoors within 30 seconds (cold start)
- GPS fix acquired within 5 seconds after recovery (warm/hot start)
- I2C transaction latency <300ms (GPS fix → `GpsState` update)
- GPS and IMU coexist on I2C0 bus without conflicts
- Documentation updated with architecture and hardware wiring

---

## Definition of Done

- [x] `cargo fmt` - Completed, no formatting issues
- [x] `cargo clippy --all-targets -- -D warnings` - Completed, no warnings
- [x] `cargo test --lib --quiet` - Completed, 391 passed, 1 ignored
- [x] `./scripts/build-rp2350.sh pico_trail_rover` - Completed, UF2 file generated successfully
- [ ] Hardware validation completed (GPS fix acquisition, latency, error recovery) - Deferred (requires NEO-M8N GPS module)
- [x] `docs/architecture.md` updated with I2C0 bus and GPS driver - Added comprehensive I2C0 Peripheral Interfaces section
- [x] `boards/freenove_standard.hwdef` updated with GPIO 0, 1 allocation - Updated in Phase 1
- [x] Hardware wiring guide added (I2C0 connection with pull-ups) - Documented in boards/freenove_standard.hwdef and architecture.md
- [x] No regressions in existing control loop timing - All 391 tests pass, no regressions detected
- [x] No `unsafe` code and no vague naming ("manager"/"util") - Code review confirms no unsafe code, naming follows project conventions

## Open Questions

- [ ] Should GPS task run at 1Hz, 5Hz, or 10Hz? → Next step: Benchmark CPU overhead at each rate during Phase 3
- [ ] Should we implement UBX protocol for advanced GPS configuration (10Hz mode)? → Decision: Defer to future enhancement, NMEA sufficient for initial implementation
