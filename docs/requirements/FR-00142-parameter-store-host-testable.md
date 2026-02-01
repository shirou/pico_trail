# FR-00142 Parameter Store Host-Testable Library

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00142-parameter-store-core-migration](../analysis/AN-00142-parameter-store-core-migration.md)
- Prerequisite Requirements:
  - [FR-00022-configuration-persistence](FR-00022-configuration-persistence.md)
  - [FR-00006-runtime-parameters](FR-00006-runtime-parameters.md)
- Dependent Requirements:
  - [FR-00143-parameter-groups-host-testable](FR-00143-parameter-groups-host-testable.md)
  - [NFR-00092-parameter-migration-no-regression](NFR-00092-parameter-migration-no-regression.md)
- Related ADRs:
  - N/A – To be created during task design
- Related Tasks:
  - [T-00041-parameter-store-core-migration](../tasks/T-00041-parameter-store-core-migration/README.md)

## Requirement Statement

The parameter store core operations (`new`, `get`, `set`, `register`, `Default`, `count`, `iter_names`, `is_hidden`, `is_dirty`, `clear_dirty`) shall be available in the `pico_trail_core` crate as a host-testable library without embedded dependencies.

## Rationale

The firmware crate's `ParameterStore` contains architecture-independent business logic (key-value storage, registration, validation, dirty tracking) that cannot be unit-tested on the host because the firmware crate depends on embedded-only libraries (`embassy-rp`, `cortex-m`, `cyw43`, `rp235x-hal`). Moving the core operations to `pico_trail_core` enables `cargo test --lib` execution for parameter logic, catching regressions early in CI without requiring embedded hardware.

AN-00142 confirmed that all core operations depend only on `heapless` collections and a simple error type — no HAL dependency. Flash persistence methods (`load_from_flash`, `save_to_flash`) remain in the firmware crate.

## User Story (if applicable)

As a developer, I want to run parameter store unit tests via `cargo test --lib` on my development machine, so that I can validate parameter logic without requiring RP2350 hardware.

## Acceptance Criteria

- [ ] `ParameterStore` type is defined in `pico_trail_core` crate
- [ ] Core operations (`new`, `get`, `set`, `register`, `Default`, `count`, `iter_names`, `is_hidden`, `is_dirty`, `clear_dirty`) compile and run on host target
- [ ] `ParamValue` type is unified: single authoritative definition in core crate covering all variants (Float, Int, Bool, String, Ipv4)
- [ ] `ParamFlags` type is defined in core crate
- [ ] A core-level error type (`ParameterError`) replaces `PlatformError::InvalidConfig` for parameter operations
- [ ] `cargo test --lib` runs and passes parameter store registration, get, set, and validation tests
- [ ] Flash persistence methods remain in firmware crate (not moved to core)
- [ ] Firmware crate re-exports or imports core parameter types seamlessly

## Technical Details (if applicable)

### Functional Requirement Details

**Current Architecture:**

```text
pico_trail_firmware/src/parameters/storage.rs
├── ParameterStore (core ops + flash ops)
├── ParamValue (String, Bool, Int, Float, Ipv4)
└── ParamFlags (bitflags)
```

**Target Architecture:**

```text
pico_trail_core/src/parameters/
├── storage.rs    ← ParameterStore (core ops only), ParamValue, ParamFlags
└── error.rs      ← ParameterError

pico_trail_firmware/src/parameters/
├── storage.rs    ← Flash persistence wrapper (load_from_flash, save_to_flash)
└── ...           ← Re-exports core types
```

**Architecture-Independent Methods (move to core):**

| Method          | Dependencies            | Notes                         |
| --------------- | ----------------------- | ----------------------------- |
| `new()`         | `heapless::FnvIndexMap` | Pure data structure           |
| `get()`         | `heapless::String`      | Key lookup only               |
| `set()`         | `ParameterError`        | Validation + dirty flag       |
| `register()`    | `ParameterError`        | Idempotent registration       |
| `Default`       | None                    | Empty store                   |
| `count()`       | None                    | Count accessor                |
| `iter_names()`  | None                    | Iterator over parameter names |
| `is_hidden()`   | None                    | Hidden flag check             |
| `is_dirty()`    | None                    | Dirty flag check              |
| `clear_dirty()` | None                    | Reset dirty flag              |

**Architecture-Dependent Methods (stay in firmware):**

| Method              | Dependencies     | Notes                     |
| ------------------- | ---------------- | ------------------------- |
| `load_from_flash()` | `FlashInterface` | Reads from hardware Flash |
| `save_to_flash()`   | `FlashInterface` | Writes to hardware Flash  |

**ParamValue Unification:**

Core crate currently has a simpler `ParamValue` in `registry.rs` (Float, Uint32). Firmware has a richer type (String, Bool, Int, Float, Ipv4). The unified core `ParamValue` must be a superset covering all variants.

## Platform Considerations

### Cross-Platform

- Core parameter types must remain `no_std` compatible
- `heapless` and `bitflags` are already dependencies of the core crate
- Host tests use `cargo test --lib` (no special flags)

## Risks & Mitigation

| Risk                                          | Impact | Likelihood | Mitigation                                                        | Validation                          |
| --------------------------------------------- | ------ | ---------- | ----------------------------------------------------------------- | ----------------------------------- |
| `ParamValue` unification breaks existing code | Medium | Medium     | Superset approach — extend core type to cover all variants        | RP2350 build verification           |
| Circular dependency between crates            | High   | Low        | Core only exports types; firmware imports core                    | `cargo check` on both crates        |
| Flash persistence methods reference core type | Medium | Low        | Firmware wraps core `ParameterStore` with flash methods           | Compile-time check via RP2350 build |
| Error type mismatch                           | Medium | Medium     | Firmware converts `ParameterError` to `PlatformError` at boundary | Unit tests for error conversion     |

## Implementation Notes

- `heapless` collections are already used in core crate — no new dependency needed
- `bitflags` is already a dependency of core crate
- Core crate must remain `no_std` compatible (no `std` features)
- The `crc` crate is `no_std` compatible and could optionally move to core if `FlashInterface` trait is also migrated

## External References

- [heapless crate](https://docs.rs/heapless/) - Fixed-capacity collections for `no_std`
- [bitflags crate](https://docs.rs/bitflags/) - Type-safe bitflags
