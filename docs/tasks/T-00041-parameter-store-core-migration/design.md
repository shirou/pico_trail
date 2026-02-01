# T-00041 Parameter Store Core Migration

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-00041-parameter-store-core-migration-plan](plan.md)

## Overview

Migrate the architecture-independent parameter subsystem from `pico_trail_firmware` to `pico_trail_core` to enable host-side `cargo test --lib` execution. The migration follows Option A from AN-00142: move core operations and parameter group types to core, keep flash persistence in firmware.

## Success Metrics

- [x] `ParameterStore` core operations testable on host via `cargo test --lib`
- [x] All 8 architecture-independent parameter groups testable on host
- [x] RP2350 build succeeds with no regressions
- [x] Flash persistence continues to work on embedded targets
- [x] Firmware import paths unchanged (re-exports maintain compatibility)

## Background and Current State

- Context: The firmware crate contains the entire parameter subsystem. Firmware tests compile during RP2350 build but never execute on host because the crate depends on embedded-only libraries.
- Current behavior: `cargo test --lib` only runs core crate tests. Parameter logic tests are compile-checked only.
- Pain points: Parameter logic regressions are not caught until RP2350 build; no CI-runnable tests for parameter code.
- Constraints: Core crate must remain `no_std`. No circular dependencies. Flash persistence must stay in firmware.
- Related analysis: [AN-00142](../../analysis/AN-00142-parameter-store-core-migration.md)

## Proposed Design

### High-Level Architecture

```text
BEFORE:
  pico_trail_core/src/parameters/
  ├── mod.rs          (re-exports block, crc, registry)
  ├── block.rs        (BlockStorage)
  ├── crc.rs          (CRC utilities)
  └── registry.rs     (ParamValue{Float,Uint32}, ParamMetadata, ParamType)

  pico_trail_firmware/src/parameters/
  ├── mod.rs          (re-exports all submodules)
  ├── storage.rs      (ParameterStore, ParamValue{String,Bool,Int,Float,Ipv4}, ParamFlags, ParamMetadata)
  ├── compass.rs      (CompassParams)
  ├── arming.rs       (ArmingParams)
  ├── battery.rs      (BatteryParams)
  ├── failsafe.rs     (FailsafeParams)
  ├── fence.rs        (FenceParams)
  ├── loiter.rs       (LoiterParams)
  ├── circle.rs       (CircleParams)
  ├── wifi.rs         (WifiParams)
  └── board.rs        (BoardParams) ← arch-dependent

AFTER:
  pico_trail_core/src/parameters/
  ├── mod.rs          (re-exports all submodules)
  ├── block.rs        (BlockStorage — unchanged)
  ├── crc.rs          (CRC utilities — unchanged)
  ├── registry.rs     (ParamType, ParamMetadata — unchanged or unified)
  ├── error.rs        (ParameterError — NEW)
  ├── storage.rs      (ParameterStore, ParamValue, ParamFlags, ParamMetadata — MOVED from firmware)
  ├── compass.rs      (CompassParams — MOVED)
  ├── arming.rs       (ArmingParams — MOVED)
  ├── battery.rs      (BatteryParams — MOVED)
  ├── failsafe.rs     (FailsafeParams — MOVED)
  ├── fence.rs        (FenceParams — MOVED)
  ├── loiter.rs       (LoiterParams — MOVED)
  ├── circle.rs       (CircleParams — MOVED)
  └── wifi.rs         (WifiParams — MOVED)

  pico_trail_firmware/src/parameters/
  ├── mod.rs          (re-exports core types + local modules)
  ├── storage.rs      (Flash persistence: load_from_flash, save_to_flash — KEPT)
  └── board.rs        (BoardParams — KEPT, arch-dependent)
```

### Components

#### 1. ParameterError (new in core)

```rust
// crates/core/src/parameters/error.rs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterError {
    /// Invalid configuration (e.g., duplicate registration, unknown parameter)
    InvalidConfig,
    /// Store is full
    StoreFull,
    /// Read-only parameter cannot be modified
    ReadOnly,
}
```

Firmware code converts `ParameterError` to `PlatformError::InvalidConfig` at the boundary where needed.

#### 2. ParamValue Unification

The core `ParamValue` (currently `Float(f32)`, `Uint32(u32)`) is replaced by the firmware's richer type:

```rust
// crates/core/src/parameters/storage.rs (moved from firmware)
#[derive(Debug, Clone, PartialEq)]
pub enum ParamValue {
    String(String<MAX_STRING_LEN>),
    Bool(bool),
    Int(i32),
    Float(f32),
    Ipv4([u8; 4]),
}
```

The old core `ParamValue` in `registry.rs` and its `ParamMetadata` are superseded. The `registry.rs` `ParamValue` is either removed or aliased to the new unified type.

#### 3. ParameterStore Core Operations (moved to core)

All architecture-independent methods move to core:

```rust
// crates/core/src/parameters/storage.rs
impl ParameterStore {
    pub fn new() -> Self { ... }
    pub fn get(&self, name: &str) -> Option<&ParamValue> { ... }
    pub fn set(&mut self, name: &str, value: ParamValue) -> Result<(), ParameterError> { ... }
    pub fn register(&mut self, name: &str, default: ParamValue, flags: ParamFlags) -> Result<(), ParameterError> { ... }
    pub fn is_hidden(&self, name: &str) -> bool { ... }
    pub fn iter_names(&self) -> impl Iterator<Item = &str> { ... }
    pub fn count(&self) -> usize { ... }
    pub fn is_dirty(&self) -> bool { ... }
}
```

Flash methods stay in firmware as extension methods or a wrapper:

```rust
// crates/firmware/src/parameters/storage.rs
use pico_trail_core::parameters::storage::ParameterStore;

/// Extension methods for flash persistence
impl ParameterStore {
    // NOTE: This won't work directly — can't impl methods on foreign types.
    // Instead, use free functions or a wrapper type.
}
```

**Design choice**: Since Rust doesn't allow implementing methods on foreign types, the flash persistence is implemented as free functions:

```rust
// crates/firmware/src/parameters/storage.rs
pub fn load_from_flash(store: &mut ParameterStore, flash: &mut impl FlashInterface) -> Result<()> { ... }
pub fn save_to_flash(store: &mut ParameterStore, flash: &mut impl FlashInterface) -> Result<()> { ... }
```

Or alternatively, keep `ParameterStore` in core but make the flash methods available via a trait that firmware implements. The simplest approach is free functions.

#### 4. Parameter Group Types (moved to core)

Each parameter group file moves to core with minimal changes:

```rust
// BEFORE (firmware): use super::storage::{ParamFlags, ParamValue, ParameterStore};
//                    use crate::platform::Result;

// AFTER (core):      use super::storage::{ParamFlags, ParamValue, ParameterStore};
//                    use super::error::ParameterError;
```

The only change is the error type import: `crate::platform::Result` → `Result<(), ParameterError>`.

#### 5. Firmware Re-exports

```rust
// crates/firmware/src/parameters/mod.rs
pub use pico_trail_core::parameters::storage::{ParamFlags, ParamValue, ParameterStore, ParamMetadata};
pub use pico_trail_core::parameters::error::ParameterError;
pub use pico_trail_core::parameters::compass::CompassParams;
pub use pico_trail_core::parameters::arming::ArmingParams;
// ... etc for all parameter groups
pub mod board;  // stays in firmware
pub mod storage; // flash persistence functions
```

### Data Flow

1. **Boot**: Firmware calls `load_from_flash()` → populates core `ParameterStore` → `CompassParams::from_store()` (now in core) reads values → `SystemState` initialization (stays in firmware) uses values
2. **Runtime**: Parameter get/set via core `ParameterStore` methods → firmware flash persistence saves when dirty
3. **Tests**: Core `ParameterStore` and parameter groups testable without flash → `cargo test --lib` runs all parameter logic

### Error Handling

- Core operations return `Result<(), ParameterError>`
- Firmware converts `ParameterError` to `PlatformError` at the boundary using `From` impl:

```rust
impl From<ParameterError> for PlatformError {
    fn from(e: ParameterError) -> Self {
        match e {
            ParameterError::InvalidConfig => PlatformError::InvalidConfig,
            ParameterError::StoreFull => PlatformError::InvalidConfig,
            ParameterError::ReadOnly => PlatformError::InvalidConfig,
        }
    }
}
```

### Security Considerations

- No security implications — parameter types contain configuration data only
- No change to MAVLink protocol security properties

### Performance Considerations

- No runtime performance impact — same code, different crate location
- Compile time may slightly increase due to more code in core crate
- No additional allocations or indirections

### Platform Considerations

#### Cross-Platform

- Core crate remains `no_std` — `heapless` and `bitflags` already available
- `MAX_STRING_LEN`, `MAX_PARAMS`, `PARAM_NAME_LEN` constants move to core
- Host tests use in-memory `ParameterStore` (no flash)

## Alternatives Considered

1. **Feature-gate firmware dependencies for host compilation**
   - Rejected: Complex `Cargo.toml`, fragile, doesn't fix architectural separation
   - Details in AN-00142 Option B

2. **Create thin adapters in core, keep originals in firmware**
   - Rejected: Duplication, doesn't solve root problem
   - Details in AN-00142 Option C

3. **Wrapper type in firmware around core ParameterStore**
   - Considered for flash methods: `pub struct FirmwareParameterStore(ParameterStore)`
   - Rejected: Adds unnecessary indirection; free functions are simpler

## Migration and Compatibility

- All firmware import paths maintained via re-exports
- Flash storage format unchanged (serialization/deserialization code stays in firmware)
- MAVLink parameter protocol unchanged
- Existing firmware tests continue to compile-check via RP2350 build
- New core tests provide host-runnable execution coverage

## Testing Strategy

### Unit Tests (core crate — host-runnable)

- `ParameterStore`: new, get, set, register, duplicate registration, unknown parameter, dirty flag
- `ParamValue`: serialization round-trip, type_id, equality
- `CompassParams`: register_defaults, from_store defaults, from_store custom values
- `ArmingParams`: register_defaults, from_store defaults, from_store custom values
- `BatteryParams`: register_defaults, from_store defaults, from_store custom values
- `FailsafeParams`: register_defaults, from_store defaults, from_store custom values
- `FenceParams`: register_defaults, from_store defaults, from_store custom values
- `LoiterParams`: register_defaults, from_store defaults, from_store custom values
- `CircleParams`: register_defaults, from_store defaults, from_store custom values
- `WifiParams`: register_defaults, from_store defaults, from_store custom values

### Integration Tests (RP2350 build)

- Flash persistence: save/load round-trip (existing tests in firmware)
- MAVLink PARAM protocol: unchanged behavior

## Documentation Impact

- Update AN-00142 next steps as completed
- Update FR-00142 / FR-00143 / NFR-00092 acceptance criteria

## External References

- [heapless crate](https://docs.rs/heapless/) - Fixed-capacity collections for `no_std`
- [bitflags crate](https://docs.rs/bitflags/) - Type-safe bitflags

## Open Questions

- [ ] Should `ParamValue::serialize()` and `ParamValue::deserialize()` move to core or stay with flash persistence in firmware? They use `crc` which is `no_std` compatible, but are only used by flash methods.
- [ ] Should the old core `registry.rs` `ParamValue`/`ParamMetadata` be removed or kept as a lighter-weight alternative? Removing simplifies the codebase; keeping allows registry-specific uses.
