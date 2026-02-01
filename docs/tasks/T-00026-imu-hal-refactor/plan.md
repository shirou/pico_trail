# T-00026 IMU HAL Refactor Plan

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [T-00026-imu-hal-refactor-design](design.md)

## Overview

Refactor IMU drivers to use `embedded_hal_async::i2c::I2c` trait and relocate from platform directory to devices directory.

## Success Metrics

- [x] Zero `embassy_rp::*` imports in `src/devices/imu/`
- [x] Zero `#[cfg(feature = "pico2_w")]` in IMU driver modules
- [x] All existing tests pass
- [x] Embedded build succeeds

## Scope

- Goal: Platform-agnostic IMU drivers using embedded-hal-async
- Non-Goals: New IMU support, SPI interface
- Assumptions: `embedded_hal_async` crate available in dependencies
- Constraints: Must maintain `ImuSensor` trait compatibility

## ADR & Legacy Alignment

- [x] ADR-00028-imu-embedded-hal-async referenced
- [x] Remove `src/platform/rp2350/devices/imu/` after migration

## Plan Summary

- Phase 1 - Add embedded-hal-async dependency and refactor driver generics
- Phase 2 - Move files and update imports
- Phase 3 - Cleanup and verification

---

## Phase 1: Refactor Driver Generics

### Goal

- Change driver structs to use `embedded_hal_async::i2c::I2c` trait bound

### Inputs

- Source Code to Modify:
  - `src/platform/rp2350/devices/imu/mpu9250/driver.rs`
  - `src/platform/rp2350/devices/imu/icm20948/driver.rs`
- Dependencies:
  - External crates: `embedded-hal-async` (add if not present)

### Tasks

- [x] **Dependency check**
  - [x] Verify `embedded-hal-async` is in Cargo.toml
  - [x] Add if missing with appropriate feature gates

- [x] **Refactor Mpu9250Driver**
  - [x] Change struct generic from `<'d, T: embassy_rp::i2c::Instance>` to `<I2C: I2c>`
  - [x] Change `i2c` field from `I2c<'d, T, Async>` to `I2C`
  - [x] Update all `impl` blocks with new generic bounds
  - [x] Add `where I2C: embedded_hal_async::i2c::I2c` clauses
  - [x] Replace `embassy_rp::i2c` imports with `embedded_hal_async::i2c`

- [x] **Refactor Icm20948Driver**
  - [x] Same changes as Mpu9250Driver
  - [x] Update all method signatures

- [x] **Time abstraction**
  - [x] Create `timestamp_us()` helper with `embassy` feature gate
  - [x] Update drivers to use the helper

### Deliverables

- Drivers compile with generic I2C trait bound

### Verification

```bash
cargo check --features pico2_w
cargo test --lib --quiet
```

### Acceptance Criteria (Phase Gate)

- Drivers compile with `embassy_rp::i2c::I2c` passed as generic parameter
- No `embassy_rp::i2c::Instance` trait bounds remain

---

## Phase 2: Move Files and Update Imports

### Phase 2 Goal

- Relocate driver files to `src/devices/imu/`
- Update all import paths

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Refactored drivers
- Source Code to Modify:
  - `src/platform/rp2350/devices/imu/` (source)
  - `src/devices/imu/` (destination)
  - `src/platform/rp2350/devices/mod.rs`
  - `src/platform/rp2350/mod.rs`
  - `examples/pico_trail_rover.rs` (if uses IMU)

### Phase 2 Tasks

- [x] **Move driver files**
  - [x] Move `src/platform/rp2350/devices/imu/mpu9250/` to `src/devices/imu/mpu9250/`
  - [x] Move `src/platform/rp2350/devices/imu/icm20948/` to `src/devices/imu/icm20948/`
  - [x] Update `src/devices/imu/mod.rs` to export drivers directly

- [x] **Update platform module**
  - [x] Remove `imu` module from `src/platform/rp2350/devices/mod.rs`
  - [x] Delete `src/platform/rp2350/devices/imu/` directory

- [x] **Update imports in examples**
  - [x] Search for `use crate::devices::imu::` usages
  - [x] Verify all imports resolve correctly

### Phase 2 Deliverables

- Drivers in correct location
- No `pico2_w` feature gates on IMU exports

### Phase 2 Verification

```bash
cargo check --features pico2_w
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- Files in `src/devices/imu/`
- `src/platform/rp2350/devices/imu/` deleted
- All builds pass

---

## Phase 3: Cleanup and Verification

### Phase 3 Goal

- Verify no platform-specific code remains in drivers
- Update documentation

### Phase 3 Tasks

- [x] **Verify no platform imports**
  - [x] `grep -r "embassy_rp" src/devices/imu/` returns empty
  - [x] `grep -r "pico2_w" src/devices/imu/` returns empty

- [x] **Update traceability**
  - [x] Run `bun scripts/trace-status.ts --write`

- [x] **Final verification**
  - [x] `cargo fmt`
  - [x] `cargo clippy --lib -- -D warnings`
  - [x] `cargo test --lib --quiet`
  - [x] `./scripts/build-rp2350.sh pico_trail_rover`

### Phase 3 Deliverables

- Clean, platform-agnostic IMU drivers
- Updated documentation

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- All checks pass
- No platform-specific imports in `src/devices/imu/`

---

## Definition of Done

- [x] `cargo check --features pico2_w`
- [x] `cargo fmt`
- [x] `cargo clippy --lib -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh pico_trail_rover`
- [x] Zero `embassy_rp` imports in `src/devices/imu/`
- [x] Zero `pico2_w` feature gates in `src/devices/imu/`
- [x] `src/platform/rp2350/devices/imu/` removed

## Open Questions

- [x] Use `embedded_hal_async::i2c::I2c` - confirmed
- [ ] Error type design - use generic `ImuError<E>` where `E` is I2C error type
