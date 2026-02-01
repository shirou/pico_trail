# AN-00142 Parameter Store Core Crate Migration

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - N/A – No prior analyses on this topic
- Related Requirements:
  - [FR-00022-configuration-persistence](../requirements/FR-00022-configuration-persistence.md)
  - [FR-00006-runtime-parameters](../requirements/FR-00006-runtime-parameters.md)
  - [FR-00142-parameter-store-host-testable](../requirements/FR-00142-parameter-store-host-testable.md)
  - [FR-00143-parameter-groups-host-testable](../requirements/FR-00143-parameter-groups-host-testable.md)
  - [NFR-00092-parameter-migration-no-regression](../requirements/NFR-00092-parameter-migration-no-regression.md)
- Related ADRs:
  - N/A – To be created if migration approach is approved
- Related Tasks:
  - [T-00040-compass-yaw-offset-persistence](../tasks/T-00040-compass-yaw-offset-persistence/README.md)
  - [T-00041-parameter-store-core-migration](../tasks/T-00041-parameter-store-core-migration/README.md)

## Executive Summary

The firmware crate's parameter subsystem (`ParameterStore`, `CompassParams`, `ArmingParams`, `BatteryParams`, etc.) contains architecture-independent business logic that cannot be unit-tested on the host because the firmware crate depends on embedded-only libraries (`embassy-rp`, `cortex-m`, `cyw43`, `rp235x-hal`). This analysis evaluates migrating the architecture-independent portions of the parameter subsystem to the `pico_trail_core` crate, enabling host-side `cargo test --lib` execution.

**Key Findings:**

1. `ParameterStore` core operations (`new`, `get`, `set`, `register`, `Default`) are architecture-independent
2. Flash persistence methods (`load_from_flash`, `save_to_flash`) use `FlashInterface` trait — architecture-dependent but already abstracted
3. All parameter group types (`CompassParams`, `ArmingParams`, etc.) depend only on `ParameterStore` and `crate::platform::Result` — no HAL dependency
4. `PlatformError` contains HAL-specific variants — needs a core-level error type for the migrated code
5. Core crate already has a separate `ParamValue`/`ParamMetadata` in `registry.rs` — needs unification or coexistence strategy
6. `BoardParams` references `BOARD_CONFIG` (hwdef-generated) — must remain in firmware

## Problem Space

### Current State

The firmware crate (`pico_trail_firmware`) contains the entire parameter subsystem:

```text
crates/firmware/src/parameters/
├── mod.rs
├── storage.rs       ← ParameterStore, ParamValue, ParamFlags
├── compass.rs       ← CompassParams (COMPASS_OFS_*, COMPASS_DEC)
├── arming.rs        ← ArmingParams (ARMING_CHECK, ARMING_OPTIONS)
├── battery.rs       ← BatteryParams (BATT_*)
├── failsafe.rs      ← FailsafeParams (FS_*)
├── fence.rs         ← FenceParams (FENCE_*)
├── loiter.rs        ← LoiterParams (LOIT_*)
├── circle.rs        ← CircleParams (CIRCLE_*)
├── wifi.rs          ← WifiParams (NET_*)
└── board.rs         ← BoardParams (PIN_*) ← architecture-dependent
```

The firmware crate cannot compile on the host (x86_64) because its `[dependencies]` include embedded-only crates that are not feature-gated:

- `embassy-rp`, `embassy-executor`, `embassy-time` (ARM Cortex-M only)
- `cortex-m`, `cortex-m-rt` (ARM only)
- `cyw43`, `cyw43-pio` (CYW43439 WiFi chip driver)
- `rp235x-hal` (RP2350-specific HAL)

The workspace `Cargo.toml` sets `default-members = ["crates/core"]`, so `cargo test --lib` only runs core crate tests. Firmware tests in `#[cfg(test)]` modules compile during `./scripts/build-rp2350.sh` but **never execute** — they serve only as compile-time checks.

### Desired State

- Parameter subsystem core logic lives in `pico_trail_core`
- `cargo test --lib` executes parameter registration, get/set, and parameter group tests
- Flash persistence remains in firmware crate (thin adapter over core types)
- New parameter group tests are immediately runnable without embedded hardware

### Gap Analysis

| Aspect            | Current                                    | Desired                         | Gap                               |
| ----------------- | ------------------------------------------ | ------------------------------- | --------------------------------- |
| Test execution    | Compile-only (RP2350 build)                | Host-runnable via `cargo test`  | Parameter types in wrong crate    |
| Error type        | `PlatformError` (HAL variants)             | Core error type for params      | No core error enum for parameters |
| `ParamValue`      | Two separate definitions (core + firmware) | Single authoritative type       | Duplication needs resolution      |
| Flash persistence | Coupled in `ParameterStore`                | Separated into firmware adapter | All in one struct currently       |
| `BoardParams`     | Uses `BOARD_CONFIG` (hwdef)                | Remains in firmware             | Already architecture-dependent    |

## Stakeholder Analysis

| Stakeholder         | Interest/Need                           | Impact | Priority |
| ------------------- | --------------------------------------- | ------ | -------- |
| Developers          | Run parameter tests without hardware    | High   | P0       |
| CI pipeline         | Catch parameter logic regressions early | High   | P0       |
| Future contributors | Clear separation of arch-dependent code | Medium | P1       |

## Research & Discovery

### Technical Investigation

#### Dependency Analysis of `ParameterStore`

**Architecture-independent methods** (can move to core):

| Method          | Dependencies                   | Notes                         |
| --------------- | ------------------------------ | ----------------------------- |
| `new()`         | `heapless::FnvIndexMap`        | Pure data structure           |
| `get()`         | `heapless::String`             | Key lookup only               |
| `set()`         | `PlatformError::InvalidConfig` | Validation + dirty flag       |
| `register()`    | `PlatformError::InvalidConfig` | Idempotent registration       |
| `Default`       | None                           | Empty store                   |
| `count()`       | None                           | Count accessor                |
| `iter_names()`  | None                           | Iterator over parameter names |
| `is_hidden()`   | None                           | Hidden flag check             |
| `is_dirty()`    | None                           | Dirty flag check              |
| `clear_dirty()` | None                           | Reset dirty flag              |

**Architecture-dependent methods** (must stay in firmware):

| Method              | Dependencies                                    | Notes                     |
| ------------------- | ----------------------------------------------- | ------------------------- |
| `load_from_flash()` | `FlashInterface`, `crc`, `PlatformError::Flash` | Reads from hardware Flash |
| `save_to_flash()`   | `FlashInterface`, `crc`, `PlatformError::Flash` | Writes to hardware Flash  |
| `load_from_block()` | `FlashInterface`                                | Internal helper           |

Flash methods also depend on `PlatformError::InvalidConfig` for deserialization errors and `crc` for checksums. The `crc` crate is `no_std` compatible and could live in core.

#### Dependency Analysis of Parameter Group Types

All parameter groups follow the same pattern:

```rust
use super::storage::{ParamFlags, ParamValue, ParameterStore};
use crate::platform::Result;
```

| Type             | Architecture-dependent? | Notes                                               |
| ---------------- | ----------------------- | --------------------------------------------------- |
| `CompassParams`  | No                      | Only uses ParameterStore + Result                   |
| `ArmingParams`   | No                      | Only uses ParameterStore + Result                   |
| `BatteryParams`  | No                      | Only uses ParameterStore + Result                   |
| `FailsafeParams` | No                      | Only uses ParameterStore + Result                   |
| `FenceParams`    | No                      | Only uses ParameterStore + Result                   |
| `LoiterParams`   | No                      | Only uses ParameterStore + Result                   |
| `CircleParams`   | No                      | Only uses ParameterStore + Result                   |
| `WifiParams`     | No                      | Only uses ParameterStore + Result                   |
| `BoardParams`    | **Yes**                 | Uses `crate::platform::traits::board::BOARD_CONFIG` |

#### Error Type Dependency

`ParameterStore` core operations only use `PlatformError::InvalidConfig`. The full `PlatformError` enum includes HAL-specific variants (`Uart`, `I2c`, `Spi`, `Pwm`, `Gpio`, `Timer`, `Flash`). Options:

1. **Create `ParameterError`** in core with only the variants needed by parameter operations
2. **Use a generic error** (e.g., `core::fmt::Error` or a simple enum)
3. **Keep `PlatformError`** — extract common variants to core, extend in firmware

#### Existing Core `ParamValue`

The core crate already has `crates/core/src/parameters/registry.rs` with:

- `ParamType` enum (Float, Uint32)
- `ParamValue` enum (Float(f32), Uint32(u32))
- `ParamMetadata` struct (name, type, value, default, min, max, modified)

The firmware `ParamValue` is richer (String, Bool, Int, Float, Ipv4). These need unification — the core `ParamValue` should become the authoritative type, extended to support all firmware variants.

## Discovered Requirements

### Functional Requirements (Potential)

- [x] **[FR-00142](../requirements/FR-00142-parameter-store-host-testable.md)**: The parameter store core operations (new, get, set, register, Default) shall be available as a host-testable library without embedded dependencies
  - Rationale: Enables `cargo test --lib` execution for parameter logic
  - Acceptance Criteria: `cargo test --lib` runs and passes parameter store and parameter group tests

- [x] **[FR-00143](../requirements/FR-00143-parameter-groups-host-testable.md)**: All architecture-independent parameter group types shall be testable on host
  - Rationale: CompassParams, ArmingParams, BatteryParams, etc. contain only business logic
  - Acceptance Criteria: Parameter registration and from_store tests execute via `cargo test --lib`

### Non-Functional Requirements (Potential)

- [x] **[NFR-00092](../requirements/NFR-00092-parameter-migration-no-regression.md)**: The migration shall not change any public API signatures or break existing embedded builds
  - Category: Reliability
  - Rationale: Firmware code must continue to work identically
  - Target: Zero regressions in RP2350 build and existing test suite

## Design Considerations

### Technical Constraints

- Core crate must remain `no_std` compatible
- `heapless` collections are already used in core crate
- `bitflags` is already a dependency of core crate
- Flash persistence must remain in firmware (uses hardware Flash)
- `BoardParams` must remain in firmware (uses `BOARD_CONFIG`)
- No circular dependencies between core and firmware

### Potential Approaches

1. **Option A**: Move `ParameterStore` + param groups to core, create `ParameterError` in core
   - Pros: Clean separation, all param logic testable on host, single source of truth
   - Cons: Requires new error type, firmware re-exports core types, migration effort
   - Effort: Medium

2. **Option B**: Feature-gate firmware embedded dependencies, enable host compilation
   - Pros: No code moves, minimal changes
   - Cons: Complex Cargo.toml, feature explosion, doesn't fix architecture, fragile
   - Effort: High (and ongoing maintenance)

3. **Option C**: Create thin parameter adapters in core, keep `ParameterStore` in firmware
   - Pros: Less code movement
   - Cons: Duplication between adapter and real store, doesn't solve the root problem
   - Effort: Medium (with technical debt)

**Recommendation**: Option A — it aligns with the existing architecture principle (core = platform-independent, firmware = platform-specific) and provides lasting benefits.

### Architecture Impact

- Will require an ADR to document the parameter subsystem migration pattern
- Core crate's existing `parameters/registry.rs` types need unification with firmware's `ParamValue`
- Firmware's `ParameterStore` flash methods become a thin wrapper around core's store

## Risk Assessment

| Risk                                          | Probability | Impact | Mitigation Strategy                                                 |
| --------------------------------------------- | ----------- | ------ | ------------------------------------------------------------------- |
| Breaking existing firmware builds             | Medium      | High   | Phased migration with RP2350 build verification per phase           |
| Circular dependency between crates            | Low         | High   | Core only exports types; firmware imports core                      |
| `ParamValue` unification breaks existing code | Medium      | Medium | Superset approach — extend core type to cover all firmware variants |
| Test behavior differs from embedded runtime   | Low         | Low    | Core operations are pure logic with no platform dependency          |

## Open Questions

- [x] Are `ParameterStore` core operations truly architecture-independent? → **Yes**, only `load_from_flash`/`save_to_flash` use hardware
- [x] Which parameter groups can migrate? → All except `BoardParams`
- [ ] Should `FlashInterface` trait also move to core? → Investigate: it's a trait with no implementation in core, moving it would allow core to define the full `ParameterStore` including flash method signatures
- [ ] How to unify core `ParamValue` with firmware `ParamValue`? → Needs design decision in ADR

## Recommendations

### Immediate Actions

1. Create formal requirements for the migration
2. Draft ADR for the parameter subsystem migration pattern

### Next Steps

1. [x] Create formal requirements: FR-00142, FR-00143, NFR-00092
2. [x] ADR deemed unnecessary — migration follows existing core/firmware separation pattern (documented in T-00041 README)
3. [x] Create task package: T-00041-parameter-store-core-migration

### Out of Scope

- `SystemState` migration (depends on firmware-specific types like `GpsPosition`, `AttitudeState`)
- `BoardParams` migration (depends on hwdef-generated `BOARD_CONFIG`)
- Removing firmware's parameter module entirely (flash persistence stays)
