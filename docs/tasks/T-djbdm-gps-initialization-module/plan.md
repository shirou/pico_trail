# T-djbdm GPS Initialization Module | Plan

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [T-djbdm-gps-initialization-module-design](design.md)

## Overview

This plan describes the implementation steps for refactoring GPS initialization code into a separate module structure with feature flags, enabling multi-vendor GPS support.

## Success Metrics

- [x] All existing GPS unit tests pass
- [x] Embedded build compiles successfully
- [x] Example code updated and functional
- [x] No regressions in GPS functionality

## Scope

- Goal: Extract u-blox initialization to `src/devices/gps/init/ublox.rs` with `gps-ublox` feature flag
- Non-Goals: Implementing MTK, SiRF, or other vendors; auto-detection
- Assumptions: Existing `GpsDriver` NMEA parsing code is stable
- Constraints: Must maintain backward compatibility; no_std environment

## ADR & Legacy Alignment

- [x] Confirm [ADR-27cz1-gps-initialization-module-structure](../../adr/ADR-27cz1-gps-initialization-module-structure.md) is the governing decision
- [x] Identify legacy code: `GpsDriver::init_ublox()`, `build_cfg_msg()`, `ubx_checksum()` to be extracted

## Plan Summary

- Phase 1 - Module structure creation and code extraction
- Phase 2 - Feature flag integration and example updates
- Phase 3 - Testing and verification

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask.

---

## Phase 1: Module Structure and Code Extraction

### Goal

- Create the `src/devices/gps/init/` module directory structure
- Extract u-blox initialization functions from `GpsDriver`

### Inputs

- Documentation:
  - `docs/adr/ADR-27cz1-gps-initialization-module-structure.md` - Architecture decision
- Source Code to Modify:
  - `src/devices/gps.rs` - Current GPS driver with embedded u-blox code
- Dependencies:
  - Internal: `src/core/uart.rs` - `UartInterface` trait

### Tasks

- [x] **Create module directory structure**
  - [x] Create `src/devices/gps/init/` directory
  - [x] Create `src/devices/gps/init/mod.rs` with feature-gated exports
  - [x] Create `src/devices/gps/init/ublox.rs` stub
- [x] **Extract u-blox initialization functions**
  - [x] Move `ubx_checksum()` to `init/ublox.rs` (make public within module)
  - [x] Move `build_cfg_msg()` to `init/ublox.rs` (make public within module)
  - [x] Create `pub fn initialize<U: UartInterface>(uart: &mut U) -> Result<()>` in `init/ublox.rs`
  - [x] Port logic from `GpsDriver::init_ublox()` to new `initialize()` function
- [x] **Update GpsDriver**
  - [x] Ensure `uart_mut()` method is public (not just `#[cfg(test)]`)
  - [x] Remove `GpsDriver::init_ublox()` (replaced by `gps::init::ublox::initialize()`)
- [x] **Update module exports**
  - [x] Add `pub mod init;` to `src/devices/gps.rs` (or `mod.rs` if converted)

### Deliverables

- `src/devices/gps/init/mod.rs` - Feature-gated module exports
- `src/devices/gps/init/ublox.rs` - u-blox initialization functions
- Updated `src/devices/gps.rs` with `uart_mut()` public (old `init_ublox()` removed)

### Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet gps
```

### Acceptance Criteria (Phase Gate)

- Code compiles without errors
- Existing tests pass
- New module structure exists

### Rollback/Fallback

- Revert to original `src/devices/gps.rs` if structural issues arise

---

## Phase 2: Feature Flag Integration

### Phase 2 Goal

- Add `gps-ublox` feature flag to Cargo.toml
- Update feature dependencies and example code

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Module structure complete
- Source Code to Modify:
  - `Cargo.toml` - Add feature flag
  - `examples/` - Update GPS examples to use new initialization path

### Phase 2 Tasks

- [x] **Add feature flag to Cargo.toml**
  - [x] Add `gps-ublox = []` feature
  - [x] Add `gps-ublox` to `pico2_w` feature dependencies
  - [x] Consider adding to default features (not added, included via pico2_w)
- [x] **Update init/mod.rs for feature gating**
  - [x] Add `#[cfg(feature = "gps-ublox")]` to `pub mod ublox;`
- [x] **Update example code**
  - [x] Find examples using `GpsDriver::init_ublox()`
  - [x] Update to use `gps::init::ublox::initialize(driver.uart_mut())?`
  - [x] Add `#[cfg(feature = "gps-ublox")]` guard where needed (not needed, pico2_w includes gps-ublox)

### Phase 2 Deliverables

- Updated `Cargo.toml` with `gps-ublox` feature
- Updated example code with new initialization pattern

### Phase 2 Verification

```bash
cargo check --features gps-ublox
cargo check --no-default-features
cargo fmt
cargo clippy --all-targets -- -D warnings
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- Build succeeds with `gps-ublox` feature
- Build succeeds without `gps-ublox` feature (GPS init unavailable)
- Embedded build compiles successfully

### Phase 2 Rollback/Fallback

- Remove feature flag if integration issues arise

---

## Phase 3: Testing & Integration

### Phase 3 Goal

- Verify all tests pass
- Ensure no regressions in GPS functionality

### Phase 3 Tasks

- [x] **Unit tests**
  - [x] Verify existing NMEA parsing tests pass (26 GPS tests pass)
  - [x] Add test for `ubx_checksum()` if not already covered (added in ublox.rs)
  - [x] Add test for `build_cfg_msg()` byte output (added in ublox.rs)
- [x] **Integration verification**
  - [x] Build all examples with GPS functionality
  - [ ] Test on hardware if available (NEO-M8N on RP2350) - pending hardware test
- [x] **Documentation**
  - [x] Add inline documentation to `gps::init::ublox::initialize()`
  - [x] Update any README references to GPS initialization (none found)

### Phase 3 Deliverables

- All tests passing
- Documentation updated

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh
```

### Phase 3 Acceptance Criteria

- All unit tests pass
- All embedded builds succeed
- Documentation is complete

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] `./scripts/build-rp2350.sh` (embedded verification)
- [x] Example code updated and working
- [x] No `unsafe` code added
- [x] Feature flag `gps-ublox` properly gating u-blox code

## Open Questions

- [x] Should `GpsDriver::init_ublox()` wrapper be removed immediately or deprecated first? - Removed immediately

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
