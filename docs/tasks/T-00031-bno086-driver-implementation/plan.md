# T-00031 BNO086 Driver Implementation - Plan

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [T-00031-bno086-driver-implementation-design](./design.md)

## Overview

Implement BNO086 9-axis IMU driver with on-chip sensor fusion, providing quaternion output via the new `QuaternionSensor` trait. The driver uses SHTP protocol over I2C with interrupt-driven data acquisition and hardware reset recovery.

## Success Metrics

- [x] BNO086 initialization and Product ID verification passes
- [x] Quaternion output at \~40Hz (polling mode) or 100Hz (INT mode)
- [x] I2C read latency acceptable for real-time operation
- [x] Automatic recovery infrastructure implemented (INT/RST driver variant)
- [x] All existing tests pass; no regressions (646 tests)
- [x] Embedded build succeeds on RP2350

## Scope

- Goal: Functional BNO086 driver with QuaternionSensor trait implementation
- Non-Goals:
  - Full SHTP protocol (only Rotation Vector report)
  - DMP reprogramming
  - Raw sensor data exposure (quaternion only)
  - ImuSensor trait implementation (different paradigm)
- Assumptions:
  - Genuine BNO086 hardware from reputable source
  - I2C0 bus available on GPIO4/GPIO5
  - GPIO6/GPIO7 available for INT/RST
- Constraints:
  - Embassy async runtime
  - No heap allocation
  - Static memory only

## ADR & Legacy Alignment

- [ ] Confirm ADR-00026 (I2C driver architecture) referenced for general patterns
- [ ] Note: BNO086 uses SHTP protocol, not standard I2C register access - this is a deviation from existing IMU drivers

## Plan Summary

- Phase 1 – QuaternionSensor trait and SHTP foundation
- Phase 2 – BNO086 driver core (initialization, reading)
- Phase 3 – INT/RST handling and error recovery
- Phase 4 – Testing and integration

---

## Phase 1: QuaternionSensor Trait and SHTP Protocol Module

### Goal

- Define new `QuaternionSensor` trait for quaternion-native sensors
- Implement generic SHTP protocol module (sensor-independent, reusable)

### Inputs

- Documentation:
  - `docs/analysis/AN-00028-bno086-imu-integration.md` – BNO086 analysis
  - `docs/tasks/T-00031-bno086-driver-implementation/design.md` – Design document
- Source Code to Modify:
  - `src/devices/traits/mod.rs` – Add quaternion trait export
  - `src/communication/mod.rs` – Add shtp module export
- Dependencies:
  - Internal: `src/devices/traits/imu.rs` – Reference for trait patterns
  - External crates: `nalgebra` (Quaternion type), `embedded-hal-async`

### Tasks

- [x] **Create QuaternionSensor trait**
  - [x] Create `src/devices/traits/quaternion.rs`
  - [x] Define `QuaternionSensor` trait with async methods
  - [x] Define `QuaternionError` enum
  - [x] Add unit tests for error types
  - [x] Export from `src/devices/traits/mod.rs`

- [x] **Create generic SHTP protocol module**
  - [x] Create `src/communication/shtp/` directory
  - [x] Create `src/communication/shtp/mod.rs`
  - [x] Implement `ShtpPacket<N>` structure (generic buffer size)
  - [x] Implement `ShtpChannel` enum
  - [x] Implement `ShtpError` enum
  - [x] Add unit tests for packet structures

- [x] **Create ShtpTransport trait and I2C implementation**
  - [x] Create `src/communication/shtp/transport.rs`
  - [x] Define `ShtpTransport` trait (async read/write)
  - [x] Create `src/communication/shtp/i2c.rs`
  - [x] Implement `ShtpI2c<I2C>` struct
  - [x] Implement `ShtpTransport` for `ShtpI2c`
  - [x] Add unit tests for header parsing

### Deliverables

- `src/devices/traits/quaternion.rs` with `QuaternionSensor` trait
- `src/communication/shtp/` with generic SHTP protocol (sensor-independent)
- `ShtpTransport` trait and `ShtpI2c` implementation
- Unit tests for all new code

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::traits::quaternion
cargo test --lib --quiet devices::imu::bno086
```

### Acceptance Criteria (Phase Gate)

- `QuaternionSensor` trait compiles and is exported
- SHTP packet parsing works with test vectors
- Rotation Vector report converts to correct quaternion values
- All unit tests pass

### Rollback/Fallback

- Revert new files if trait design proves incompatible
- Trait can be refined based on actual driver needs

---

## Phase 2: BNO086 Driver Core

### Phase 2 Goal

- Implement BNO086 driver using generic `ShtpTransport`
- Implement BNO086-specific report parsing
- Implement initialization sequence and basic quaternion reading

### Phase 2 Inputs

- Dependencies:
  - Phase 1: `QuaternionSensor` trait, `ShtpTransport` trait, `ShtpI2c`
  - External crates: `embassy-rp` (I2C), `embedded-hal-async`
- Source Code to Modify:
  - `src/devices/imu/bno086/` – New BNO086 driver module
  - `src/devices/imu/mod.rs` – Add bno086 export

### Phase 2 Tasks

- [x] **Create BNO086 report parsing (sensor-specific)**
  - [x] Create `src/devices/imu/bno086/reports.rs`
  - [x] Implement `RotationVectorReport` structure
  - [x] Implement Q14/Q12 fixed-point to float conversion
  - [x] Implement `to_quaternion()` method
  - [x] Add unit tests for report parsing

- [x] **Create driver structure**
  - [x] Create `src/devices/imu/bno086/driver.rs`
  - [x] Implement `Bno086Driver<T: ShtpTransport>` struct
  - [x] Implement `new(transport, config)` constructor
  - [x] Driver uses generic `ShtpTransport` (not tied to I2C)

- [x] **Implement initialization sequence**
  - [x] Implement `init()` method
  - [x] Read Product ID via SHTP
  - [x] Read and discard initial packets (clear buffer)
  - [x] Send "Set Feature Command" for Rotation Vector at 100Hz
  - [x] Verify sensor responds

- [x] **Implement basic reading (polling mode)**
  - [x] Use `ShtpTransport::read_packet()` for data
  - [x] Parse Rotation Vector report
  - [x] Implement `QuaternionSensor` trait for `Bno086Driver`
  - [x] Test with timer-based polling (INT handling in Phase 3)

- [x] **Create module exports**
  - [x] Create `src/devices/imu/bno086/mod.rs`
  - [x] Export driver, reports, and config types
  - [x] Update `src/devices/imu/mod.rs` to include bno086

### Phase 2 Deliverables

- `src/devices/imu/bno086/driver.rs` with core driver
- `src/devices/imu/bno086/mod.rs` with public API
- `QuaternionSensor` trait implementation
- Working quaternion reading (polling mode)

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::imu::bno086
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- Driver compiles for embedded target
- `QuaternionSensor` trait implemented
- Can read quaternion from sensor (hardware test)

### Phase 2 Rollback/Fallback

- If I2C communication fails, verify wiring and address
- If SHTP protocol issues, compare with reference implementations

---

## Phase 3: INT/RST Handling and Error Recovery

### Phase 3 Goal

- Add interrupt-driven (INT pin) data acquisition
- Add hardware reset (RST pin) error recovery
- Implement robust error handling

### Phase 3 Inputs

- Dependencies:
  - Phase 2: Basic driver with I2C communication
  - External crates: `embassy-rp` (GPIO)
- Source Code Created:
  - `src/devices/imu/bno086/gpio.rs` – GPIO trait abstractions (IntPin, RstPin)
  - `src/devices/imu/bno086/driver_gpio.rs` – GPIO-enabled driver variant

### Phase 3 Tasks

- [x] **Add GPIO handling**
  - [x] Extend driver to accept INT and RST pins as generics
  - [x] Implement `wait_for_int()` using Embassy GPIO async
  - [x] Implement `hardware_reset()` method

- [x] **Implement interrupt-driven reading**
  - [x] Replace polling with INT wait
  - [x] Add timeout handling (500ms)
  - [x] Implement `read_quaternion_async()` with INT

- [x] **Implement error recovery**
  - [x] Detect timeout (no INT within 500ms)
  - [x] Trigger hardware reset on timeout
  - [x] Re-initialize sensor after reset
  - [x] Track error count for health status
  - [x] Implement reset loop protection (max 3 consecutive resets)

- [x] **Update health tracking**
  - [x] Update `is_healthy()` based on error count
  - [x] Reset error count on successful read
  - [x] Log warnings/errors on recovery events

### Phase 3 Deliverables

- INT-driven async quaternion reading
- RST-based hardware reset recovery
- Robust error handling with health tracking

### Phase 3 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet devices::imu::bno086
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- Quaternion reading works via INT (no polling)
- Driver recovers from simulated timeout
- Health status reflects sensor state
- No blocking operations in data path

### Phase 3 Rollback/Fallback

- If INT handling problematic, fall back to timer-based polling
- RST recovery can be made optional

---

## Phase 4: Testing and Integration

### Phase 4 Goal

- Comprehensive testing on hardware
- Integration with AHRS/navigation subsystems
- Documentation and examples

### Phase 4 Tasks

- [x] **Hardware testing**
  - [x] Verify output rate (\~40Hz in polling mode, 100Hz requires INT pin)
  - [x] Verify quaternion accuracy (Euler angle output validated)
  - [x] Long-term stability test (25+ minutes, 61,443 samples, 0 errors)
  - [ ] Error recovery test (disconnect/reconnect) - deferred

- [x] **Integration testing**
  - [x] Create example using BNO086 driver (`examples/bno086_demo.rs`)
  - [x] Verify quaternion to Euler conversion
  - [ ] Test with ATTITUDE_STATE update - deferred to AHRS integration

- [x] **Documentation**
  - [x] Add doc comments to all public APIs
  - [x] Update module-level documentation
  - [x] Document pin configuration requirements

- [x] **Cleanup**
  - [x] Remove any debug code
  - [x] Verify no clippy warnings
  - [x] Final embedded build verification

### Phase 4 Deliverables

- Fully tested BNO086 driver
- Example code demonstrating usage
- Complete documentation

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
# Hardware test: verify 100Hz output, accuracy
```

### Phase 4 Acceptance Criteria

- All success metrics from design met
- Example runs successfully on hardware
- Documentation complete
- No warnings or errors

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy --lib -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh bno086_demo`
- [x] Hardware verification on actual BNO086 sensor
- [x] Documentation complete with doc comments
- [x] No `unsafe` code (except heap init in demo)
- [x] No "manager" or "util" naming

## Open Questions

- [ ] Exact pin assignments for INT/RST on target board → Method: Check board documentation
- [ ] Need to support both BNO085 and BNO086? → Decision: Start with BNO086, same SHTP protocol should work
