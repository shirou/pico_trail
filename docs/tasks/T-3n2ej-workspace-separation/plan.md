# T-3n2ej Workspace Separation Plan

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [T-3n2ej-design](./design.md)

## Overview

This plan details the phased implementation of workspace separation, migrating pico_trail from a single crate to a Cargo Workspace with `crates/core` (pure `no_std`) and `crates/firmware` (Embassy binary).

## Success Metrics

- [x] Zero `#[cfg(feature` in `crates/core/src/`
- [x] `cargo test -p pico_trail_core` passes without feature flags (121 tests)
- [x] `./scripts/build-rp2350.sh` compiles successfully
- [x] All existing functionality preserved (examples build and link)
- [x] CI lint script passing (`./scripts/check-core-no-cfg.sh`)

## Scope

- Goal: Complete workspace separation with zero cfg in core
- Non-Goals: MAVLink refactoring, device driver abstraction, ESP32 support
- Assumptions: Current test suite adequately covers business logic
- Constraints: Must maintain backward compatibility during transition

## ADR & Legacy Alignment

- [ ] Confirm ADR for workspace architecture is created before Phase 2
- [ ] Note: Existing `src/core/traits/` already has partial trait abstraction (T-d9rim)

## Plan Summary

- Phase 1 – Workspace Scaffolding and CI Setup ✅
- Phase 2 – Core Crate Foundation (traits, kinematics) ✅
- Phase 3 – Pure Module Migration (parameters, arming, scheduler, navigation types) ✅
- Phase 4 – Business Logic Migration (mission, modes) ✅
- Phase 5 – Partial Extraction (rc_channel, srv_channel, motor_driver) ✅
- Phase 6 – Firmware Migration (move remaining src/ to crates/firmware/) ✅
- Phase 7 – Final Cleanup (delete src/, workspace-only Cargo.toml, documentation) ✅
- Phase 8 – Firmware Feature Simplification (non-optional deps, default features) ✅

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task.

---

## Module Migration Analysis

Analysis of all `src/` modules for core crate migration eligibility.

### Fully Movable (No Feature Gates, Pure no_std)

| Module                               | Files                             | Status     | Notes                              |
| ------------------------------------ | --------------------------------- | ---------- | ---------------------------------- |
| `src/libraries/kinematics/`          | `differential_drive.rs`, `mod.rs` | ✅ DONE    | Pure math, migrated in Phase 2     |
| `src/core/parameters/block.rs`       | Single file                       | ✅ MOVABLE | Parameter data structures          |
| `src/core/parameters/crc.rs`         | Single file                       | ✅ MOVABLE | Pure CRC32 calculation             |
| `src/core/parameters/registry.rs`    | Single file                       | ✅ MOVABLE | Parameter registry logic           |
| `src/core/parameters/storage.rs`     | Single file                       | ✅ MOVABLE | Flash storage abstraction (traits) |
| `src/core/arming/checks.rs`          | Single file                       | ✅ MOVABLE | Pre-arm check logic                |
| `src/core/arming/error.rs`           | Single file                       | ✅ MOVABLE | Error types                        |
| `src/core/arming/monitoring.rs`      | Single file                       | ✅ MOVABLE | State monitoring types             |
| `src/core/arming/disarm.rs`          | Single file                       | ✅ MOVABLE | Disarm logic                       |
| `src/core/arming/cleanup.rs`         | Single file                       | ✅ MOVABLE | Cleanup logic                      |
| `src/core/arming/initialization.rs`  | Single file                       | ✅ MOVABLE | Initialization logic               |
| `src/core/scheduler/types.rs`        | Single file                       | ✅ MOVABLE | Task metadata, stats               |
| `src/core/scheduler/stats.rs`        | Single file                       | ✅ MOVABLE | Scheduler statistics               |
| `src/core/scheduler/registry.rs`     | Single file                       | ✅ MOVABLE | Task registry                      |
| `src/core/mission/mod.rs`            | Waypoint, MissionStorage          | ✅ MOVABLE | Pure data structures               |
| `src/subsystems/navigation/types.rs` | Single file                       | ✅ MOVABLE | PositionTarget, NavigationOutput   |
| `src/subsystems/ahrs/calibration.rs` | Single file                       | ✅ MOVABLE | Pure calibration math              |

### Partially Movable (Requires Extraction)

| Module                          | Pure Logic                                                                  | Platform-Dependent                | Status                                |
| ------------------------------- | --------------------------------------------------------------------------- | --------------------------------- | ------------------------------------- |
| `src/libraries/rc_channel/`     | `normalize_channel()`, `normalize_pwm_channel()`, `check_timeout()`         | `EmbassyState<RcInput>` global    | ✅ DONE (Phase 5)                     |
| `src/libraries/srv_channel/`    | `normalized_to_pulse()`, `pulse_to_duty_cycle()`, `ActuatorInterface` trait | `Actuators` struct (PwmInterface) | ✅ DONE (Phase 5)                     |
| `src/libraries/motor_driver/`   | `Motor` trait, `HBridgeMotor`, `MotorGroup`                                 | Platform PWM impls                | ✅ DONE (Phase 5)                     |
| `src/core/mission/state.rs`     | Mission state types                                                         | Embassy mutex wrapper             | Extract types, keep mutex in platform |
| `src/core/scheduler/monitor.rs` | Monitoring logic                                                            | Embassy time trait                | Refactor time dependency              |

### Firmware-Only (Move to crates/firmware/)

These modules contain platform-specific code and will move directly to `crates/firmware/`.

| Current Location               | Target Location                           | Reason                              |
| ------------------------------ | ----------------------------------------- | ----------------------------------- |
| `src/core/parameters/saver.rs` | `crates/firmware/src/parameters/saver.rs` | Async parameter saving with Embassy |
| `src/core/arming/tasks.rs`     | `crates/firmware/src/arming/tasks.rs`     | Embassy executor specific tasks     |
| `src/core/logging.rs`          | `crates/firmware/src/logging.rs`          | defmt-specific logging              |
| `src/core/log_buffer.rs`       | `crates/firmware/src/log_buffer.rs`       | Platform-specific buffer            |
| `src/communication/`           | `crates/firmware/src/communication/`      | Heavy async + feature gates         |
| `src/devices/`                 | `crates/firmware/src/devices/`            | Hardware drivers (I2C, SPI, GPIO)   |
| `src/platform/`                | `crates/firmware/src/platform/`           | Platform abstraction layer          |
| `src/rover/`                   | `crates/firmware/src/rover/`              | Async mode implementations          |
| `src/subsystems/` (remaining)  | `crates/firmware/src/subsystems/`         | AHRS, navigation async code         |
| `src/parameters/`              | `crates/firmware/src/parameters/`         | ArduPilot parameter definitions     |
| `src/lib.rs`                   | `crates/firmware/src/lib.rs`              | Root module, merge with firmware    |

### Final Structure

After all migrations complete, `src/` directory will be **deleted entirely**.

```text
pico_trail/
├── Cargo.toml                 # Workspace-only (no [package])
├── crates/
│   ├── core/                  # Pure no_std business logic
│   │   └── src/
│   │       ├── traits/
│   │       ├── kinematics/
│   │       ├── parameters/
│   │       ├── arming/
│   │       ├── scheduler/
│   │       ├── mission/
│   │       ├── navigation/
│   │       ├── ahrs/
│   │       ├── rc/
│   │       ├── servo/
│   │       └── motor/
│   └── firmware/              # Embassy/RP2350 binary
│       └── src/
│           ├── platform/
│           ├── devices/
│           ├── communication/
│           ├── rover/
│           ├── subsystems/
│           ├── parameters/
│           ├── arming/
│           └── logging.rs
├── examples/                  # Workspace examples (unchanged)
└── scripts/
```

---

## Phase 1: Workspace Scaffolding and CI Setup

### Goal

- Create workspace structure and CI lint script without changing existing functionality

### Inputs

- Documentation:
  - `docs/analysis/AN-q7k2m-crate-workspace-separation.md`
- Source Code to Modify:
  - Root `Cargo.toml` – Convert to workspace
- Dependencies:
  - None (scaffolding only)

### Tasks

- [x] **Workspace Setup**
  - [x] Create root `Cargo.toml` with workspace definition
  - [x] Create `crates/core/Cargo.toml` with minimal dependencies
  - [x] Create `crates/core/src/lib.rs` with `#![no_std]`
  - [x] Create `crates/firmware/Cargo.toml` depending on core
  - [x] Create `crates/firmware/src/lib.rs` as placeholder

- [x] **CI Lint Script**
  - [x] Create `scripts/check-core-no-cfg.sh`
  - [x] Test script with empty core crate (should pass)
  - [ ] Add script to CI workflow (or document in AGENTS.md)

- [x] **Build Verification**
  - [x] Verify `cargo build -p pico_trail_core` works
  - [x] Verify `cargo test -p pico_trail_core` works (empty tests)
  - [x] Verify existing `./scripts/build-rp2350.sh` still works

### Deliverables

- Workspace structure created
- CI lint script operational
- Existing builds unaffected

### Verification

```bash
# Workspace structure
ls crates/core/Cargo.toml crates/firmware/Cargo.toml

# Core builds on host
cargo build -p pico_trail_core
cargo test -p pico_trail_core

# Lint script
./scripts/check-core-no-cfg.sh

# Existing embedded build still works
./scripts/build-rp2350.sh pico_trail_rover
```

### Acceptance Criteria (Phase Gate)

- Workspace compiles with empty crates
- Lint script passes on empty core
- Existing examples still build and run

### Rollback/Fallback

- Delete `crates/` directory, revert `Cargo.toml` changes

---

## Phase 2: Core Crate Foundation

### Phase 2 Goal

- Establish trait abstractions and migrate first module (kinematics)

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Workspace structure
- Source Code to Modify:
  - `src/libraries/kinematics/` – Copy to core
  - `src/core/traits/` – Reference for existing traits

### Phase 2 Tasks

- [x] **Trait Definitions**
  - [x] Create `crates/core/src/traits/mod.rs`
  - [x] Define `TimeSource` trait
  - [x] Implement `MockTime` (no feature gate)

- [x] **Kinematics Migration**
  - [x] Copy `src/libraries/kinematics/` to `crates/core/src/kinematics/`
  - [x] Remove any cfg directives (should be none)
  - [x] Add module to `crates/core/src/lib.rs`
  - [x] Create unit tests in core

- [x] **Firmware Integration**
  - [x] Add `pico_trail_core` dependency to firmware
  - [x] Create `EmbassyTime` implementing `TimeSource`
  - [x] Re-export core kinematics from firmware (via `pub use pico_trail_core as core`)

### Phase 2 Deliverables

- Trait abstractions in core
- Kinematics module migrated
- Embassy implementations in firmware

### Phase 2 Verification

```bash
cargo fmt
cargo clippy -p pico_trail_core -- -D warnings
cargo test -p pico_trail_core --lib --quiet

# Lint script
./scripts/check-core-no-cfg.sh

# Embedded still builds
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- `TimeSource` trait defined
- Kinematics tests pass on host
- Zero cfg in core crate

### Phase 2 Rollback/Fallback

- Revert to Phase 1 state if traits prove inadequate

---

## Phase 3: Pure Module Migration

### Phase 3 Goal

- Migrate fully movable modules (no feature gates, pure no_std)

### Phase 3 Inputs

- Dependencies:
  - Phase 2: Trait abstractions, kinematics migrated
- Source Code to Migrate:
  - `src/core/parameters/` (block, crc, registry, storage)
  - `src/core/arming/` (checks, error, monitoring, disarm, cleanup, initialization)
  - `src/core/scheduler/` (types, stats, registry)
  - `src/subsystems/navigation/types.rs`
  - `src/subsystems/ahrs/calibration.rs`

### Phase 3 Tasks

- [x] **Parameters Module** (partial - pure types only)
  - [x] Copy `src/core/parameters/block.rs` to `crates/core/src/parameters/`
  - [x] Copy `src/core/parameters/crc.rs` to `crates/core/src/parameters/`
  - [x] Extract pure types (ParamType, ParamValue, ParamMetadata, RegistryError) from `src/core/parameters/registry.rs` to `crates/core/src/parameters/registry.rs`
  - Note: ParameterRegistry\<F: FlashInterface> and FlashParamStorage<F> remain in firmware (depends on FlashInterface)
  - Note: `src/core/parameters/storage.rs` stays in firmware (depends on FlashInterface)
  - [x] Create `crates/core/src/parameters/mod.rs`
  - [x] Remove any cfg directives if present
  - [x] Add unit tests

- [x] **Arming Module** (partial - error types only)
  - [x] Copy `src/core/arming/error.rs` to `crates/core/src/arming/`
  - [ ] Copy `src/core/arming/checks.rs` to `crates/core/src/arming/` (depends on SystemState)
  - [ ] Copy `src/core/arming/monitoring.rs` to `crates/core/src/arming/`
  - [ ] Copy `src/core/arming/disarm.rs` to `crates/core/src/arming/`
  - [ ] Copy `src/core/arming/cleanup.rs` to `crates/core/src/arming/`
  - [ ] Copy `src/core/arming/initialization.rs` to `crates/core/src/arming/`
  - [x] Create `crates/core/src/arming/mod.rs`
  - [x] Remove any cfg directives if present
  - [x] Add unit tests

- [x] **Scheduler Module**
  - [x] Copy `src/core/scheduler/types.rs` to `crates/core/src/scheduler/`
  - [x] Copy `src/core/scheduler/stats.rs` to `crates/core/src/scheduler/`
  - [x] Copy `src/core/scheduler/registry.rs` to `crates/core/src/scheduler/`
  - [x] Create `crates/core/src/scheduler/mod.rs`
  - [x] Remove any cfg directives if present
  - [x] Add unit tests

- [x] **Navigation Types**
  - [x] Copy `src/subsystems/navigation/types.rs` to `crates/core/src/navigation/`
  - [x] Create `crates/core/src/navigation/mod.rs`
  - [x] Remove any cfg directives if present

- [x] **AHRS Calibration**
  - [x] Copy `src/subsystems/ahrs/calibration.rs` to `crates/core/src/ahrs/` (pure math only, param loading stays in firmware)
  - [x] Create `crates/core/src/ahrs/mod.rs`
  - [x] Remove any cfg directives if present
  - [x] Add unit tests

### Phase 3 Deliverables

- Parameters, arming, scheduler, navigation, ahrs modules in core
- All modules have unit tests
- Zero cfg in core crate

### Phase 3 Verification

```bash
cargo fmt
cargo clippy -p pico_trail_core -- -D warnings
cargo test -p pico_trail_core --lib --quiet

# Full lint
./scripts/check-core-no-cfg.sh

# Embedded build still works
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 3 Acceptance Criteria

- All listed modules migrated to core
- All core tests pass on host
- Zero cfg in core crate
- Embedded build unaffected

### Phase 3 Rollback/Fallback

- Keep old modules in `src/` until fully verified
- Can revert individual module migrations

---

## Phase 4: Mission and Mode Migration

### Phase 4 Goal

- Migrate mission state and mode state machines

### Phase 4 Inputs

- Dependencies:
  - Phase 3: Arming, scheduler modules in core
- Source Code to Migrate:
  - `src/core/mission/` – Waypoint, MissionStorage types
  - `src/rover/mode/` – Mode state machines (requires refactoring)

### Phase 4 Tasks

- [x] **Mission Types**
  - [x] Extract `Waypoint` and `MissionStorage` from `src/core/mission/mod.rs`
  - [x] Copy to `crates/core/src/mission/`
  - [x] Remove Embassy-specific global state
  - [x] Add unit tests

- [x] **Mode State Machines** (pure types only)
  - [x] Create `Mode` trait definition in `crates/core/src/mode/`
  - [x] Extract state types (`AutoState`, `GuidedState`, `RtlState`)
  - [x] Extract navigation utilities (`haversine_distance_bearing`, `normalize_angle`)
  - [x] Add unit tests for navigation utilities
  - Note: Mode implementations remain in `src/rover/mode/` (use ActuatorInterface, Embassy)

- [x] **Firmware Wrappers**
  - [x] Create Embassy mutex wrappers for mission state in firmware (`crates/firmware/src/core/mission/state.rs`)
  - [x] Create async task wrappers for mode execution (`navigation_task` in examples)
  - [x] Wire up to existing examples (`pico_trail_rover.rs` uses `MISSION_STORAGE` helpers)

### Phase 4 Deliverables

- Mission types in core
- Mode trait and state types in core
- Navigation calculation utilities in core

### Phase 4 Verification

```bash
cargo fmt
cargo clippy -p pico_trail_core -- -D warnings
cargo test -p pico_trail_core --lib --quiet
./scripts/check-core-no-cfg.sh
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 4 Acceptance Criteria

- Mission types in core (Waypoint, MissionStorage)
- Mode trait and pure state types in core
- Navigation utilities in core
- All tests pass on host
- Embedded build unaffected

### Phase 4 Rollback/Fallback

- Keep old modules in `src/` until verified

---

## Phase 5: Partial Extraction

### Phase 5 Goal

- Extract pure logic from mixed modules

### Phase 5 Inputs

- Dependencies:
  - Phase 4: Core module structure established
- Source Code to Refactor:
  - `src/libraries/rc_channel/` – Extract normalization functions
  - `src/libraries/srv_channel/` – Extract PWM conversion functions
  - `src/libraries/motor_driver/` – Refactor SystemState dependency

### Phase 5 Tasks

- [x] **RC Channel Extraction**
  - [x] Extract `normalize_channel()` to `crates/core/src/rc/`
  - [x] Extract `normalize_pwm_channel()` to core
  - [x] Extract `check_timeout()` to core (via `RcInput::check_timeout()`)
  - [x] Keep `EmbassyState<RcInput>` global in main crate
  - [x] Update firmware crate to re-export and use core types
  - [x] Add unit tests

- [x] **Servo Channel Extraction**
  - [x] Extract `normalized_to_pulse()` to `crates/core/src/servo/`
  - [x] Extract `pulse_to_duty_cycle()` to core
  - [x] Extract `ActuatorInterface` trait to core
  - [x] Keep `Actuators` impl in firmware crate (PwmInterface dependency)
  - [x] Update firmware crate to re-export and use core types/functions
  - [x] Add unit tests

- [x] **Motor Driver Refactoring**
  - [x] Move `Motor` trait, `MotorError`, `PwmPin` trait to core
  - [x] Move `HBridgeMotor` and `MotorGroup` to core
  - [x] Keep firmware-specific `HBridgeMotor` with logging in firmware crate
  - [x] Update firmware crate to re-export and use core traits
  - [x] Add unit tests

### Phase 5 Deliverables

- Pure RC normalization functions in core
- Pure servo conversion functions in core
- Motor driver abstractions in core

### Phase 5 Verification

```bash
cargo fmt
cargo clippy -p pico_trail_core -- -D warnings
cargo test -p pico_trail_core --lib --quiet
./scripts/check-core-no-cfg.sh
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 5 Acceptance Criteria

- [x] All pure logic extracted to core
- [x] Firmware crate uses core functions via re-exports
- [x] Zero cfg in core
- [x] No functionality regression (736 tests pass)

### Phase 5 Rollback/Fallback

- Keep original implementations until verified

---

## Phase 6: Firmware Migration

### Phase 6 Goal

- Move all remaining platform-specific code from `src/` to `crates/firmware/`

### Phase 6 Inputs

- Dependencies:
  - Phase 5: All core extractions complete
- Source Code to Migrate:
  - All remaining modules in `src/`

### Phase 6 Tasks

- [x] **Communication Module**
  - [x] Copy `src/communication/` to `crates/firmware/src/communication/`
  - [x] Imports use crate-local paths (will use pico_trail_core after src/ deletion)
  - [x] Fix any broken references

- [x] **Device Drivers**
  - [x] Copy `src/devices/` to `crates/firmware/src/devices/`
  - [x] Update imports

- [x] **Platform Layer**
  - [x] Copy `src/platform/` to `crates/firmware/src/platform/`
  - [x] Merge with existing EmbassyTime implementation
  - [x] Update imports

- [x] **Rover Module**
  - [x] Copy `src/rover/` to `crates/firmware/src/rover/`
  - [x] Mode implementations to use core traits (via re-export in core/mod.rs)
  - [x] Update imports

- [x] **Subsystems (remaining)**
  - [x] Copy remaining `src/subsystems/` to `crates/firmware/src/subsystems/`
  - [x] Update imports

- [x] **Parameters (ArduPilot definitions)**
  - [x] Copy `src/parameters/` to `crates/firmware/src/parameters/`
  - [x] Update imports

- [x] **Logging and Utilities**
  - [x] Copy `src/core/` to `crates/firmware/src/core/` (includes logging.rs, log_buffer.rs)
  - [x] Includes parameter saver and arming tasks

- [x] **Root Module**
  - [x] Create `crates/firmware/src/lib.rs` with all module declarations
  - [x] Re-export pico_trail_core modules via core/mod.rs

- [x] **Build System**
  - [x] Copy build.rs to crates/firmware/
  - [x] Update paths for workspace root

### Phase 6 Deliverables

- All platform-specific code in `crates/firmware/`
- `src/` ready for deletion
- All imports updated

### Phase 6 Verification

```bash
cargo fmt
cargo clippy -p pico_trail_firmware --all-targets -- -D warnings
cargo test -p pico_trail_core --lib --quiet
./scripts/check-core-no-cfg.sh
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 6 Acceptance Criteria

- All code moved from `src/` to `crates/firmware/`
- Embedded build works
- No duplicate code

### Phase 6 Rollback/Fallback

- Keep `src/` until firmware crate is fully verified

---

## Phase 7: Final Cleanup and Documentation

### Phase 7 Goal

- Delete `src/`, make workspace-only, update documentation

### Phase 7 Tasks

- [x] **Delete src/ Directory**
  - [x] Verify no code remains that needs migration
  - [x] Delete `src/` directory entirely
  - [x] Remove `[package]` section from root `Cargo.toml`
  - [x] Remove `[lib]`, `[[example]]`, `[[test]]` sections from root
  - [x] Move feature flags to `crates/firmware/Cargo.toml`
  - [x] Move `examples/` to `crates/firmware/examples/`

- [x] **Update Examples**
  - [x] Update imports in all examples to use `pico_trail_firmware`
  - [x] Verify examples compile with new structure
  - [x] Update build script to use `-p pico_trail_firmware`

- [x] **Documentation**
  - [x] Update CLAUDE.md with new build commands
  - [x] Update README with workspace structure
  - [x] Document module locations in architecture.md
  - [ ] Update import examples in docstrings (deferred - requires code changes)

- [x] **CI Finalization**
  - [x] Ensure lint script runs in CI (`./scripts/check-core-no-cfg.sh`)
  - [x] Update build scripts for new structure
  - [x] Verify all checks pass

- [x] **Final Verification**
  - [x] Run full test suite (`cargo test -p pico_trail_core --lib` - 121 tests pass)
  - [x] Flash to hardware and test all modes
  - [x] Verify USB serial logging works
  - [x] Confirm `src/` no longer exists

### Phase 7 Deliverables

- `src/` directory deleted
- Root `Cargo.toml` is workspace-only
- Updated documentation
- Fully integrated CI

### Phase 7 Verification

```bash
# Verify src/ is gone
[ ! -d src ] && echo "src/ deleted"

# Full suite
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/check-core-no-cfg.sh
./scripts/build-rp2350.sh pico_trail_rover

# Documentation
bun format && bun lint
```

### Phase 7 Acceptance Criteria

- `src/` directory does not exist
- Root `Cargo.toml` has no `[package]` section
- All examples work with workspace crates
- Documentation current
- CI fully operational
- Hardware verification complete

---

## Phase 8: Firmware Feature Simplification

### Phase 8 Goal

- Remove redundant feature gates from firmware crate
- Make Embassy/RP2350 dependencies non-optional

### Phase 8 Rationale

The firmware crate is Embassy/RP2350-only by design. Feature flags like `pico2_w` and `embassy` are redundant because:

1. Firmware has no host test target (business logic tests are in core)
2. All builds target the same platform (RP2350)
3. Embassy is always required for async runtime

### Phase 8 Tasks

- [x] **Make Dependencies Non-Optional**
  - [x] Remove `optional = true` from Embassy crates
  - [x] Remove `optional = true` from cortex-m, defmt, rp235x-hal
  - [x] All embedded dependencies are always enabled

- [x] **Simplify Feature Configuration**
  - [x] Keep `embassy`, `pico2_w`, `defmt`, `mock` as empty compatibility features
  - [x] Add `embassy-executor`, `embassy-time` for cfg gate compatibility
  - [x] Set `default = ["pico2_w"]` so examples build without explicit features
  - [x] Keep meaningful features: `rover`, `gps-ublox`, `usb_serial`

- [x] **Update Build Script**
  - [x] Remove `--features pico2_w` from `scripts/build-rp2350.sh`
  - [x] Verify examples build without feature flags

- [x] **Update Example Declarations**
  - [x] Remove `required-features = ["pico2_w"]` from examples
  - [x] All examples build by default

- [x] **Fix Clippy Issues**
  - [x] Fix redundant closures in i2c.rs
  - [x] Add `#[allow(clippy::too_many_arguments)]` in network.rs
  - [x] Fix redundant pattern matching in udp.rs
  - [x] Fix manual Range::contains and is_multiple_of in flash.rs
  - [x] Remove duplicated cfg attribute in saver.rs

### Phase 8 Notes

The original plan proposed completely removing `embassy` and `pico2_w` features from
Cargo.toml. However, this would require removing all `#[cfg(feature = ...)]` directives
from source code (172+ occurrences), which is a significant refactoring effort.

The pragmatic approach taken:

1. Made all dependencies non-optional (always compiled)
2. Kept feature gates as empty compatibility features enabled by default
3. Updated build script to not require explicit `--features` flag
4. This achieves the practical goal while maintaining source code stability

### Phase 8 Verification

```bash
cargo fmt
cargo clippy -p pico_trail_firmware --target thumbv8m.main-none-eabihf -- -D warnings
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 8 Acceptance Criteria

- [x] Embassy dependencies are non-optional (always compiled)
- [x] `default = ["pico2_w"]` enables all features by default
- [x] Examples build without `--features` flag
- [x] Build script simplified (no `--features pico2_w` required)
- [x] Clippy passes without errors

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy -p pico_trail_core -- -D warnings`
- [x] `cargo test -p pico_trail_core --lib --quiet` (121 tests pass)
- [x] `./scripts/check-core-no-cfg.sh` passes
- [x] `./scripts/build-rp2350.sh pico_trail_rover` succeeds
- [x] Documentation updated (CLAUDE.md, README, architecture.md)
- [x] No `unsafe` in core crate
- [x] No vague naming ("manager"/"util")
- [x] Firmware feature simplification complete (Phase 8)

## Open Questions

- [ ] Should navigation algorithms stay in core or move to a separate `crates/nav` crate? → Next step: Evaluate complexity after Phase 3
- [ ] How to handle MAVLink message types (shared between core and firmware)? → Method: Consider `crates/mavlink-types` crate if needed
