# FR-5f7tx Core Crate no_std Purity

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - N/A - This is a foundational requirement
- Dependent Requirements:
  - [FR-mna5g-trait-abstractions-platform-services](../requirements/FR-mna5g-trait-abstractions-platform-services.md)
  - [FR-lmy7w-external-impl-observability](../requirements/FR-lmy7w-external-impl-observability.md)
  - [NFR-3y83q-zero-cfg-core-crate](../requirements/NFR-3y83q-zero-cfg-core-crate.md)
- Related Analyses:
  - [AN-q7k2m-crate-workspace-separation](../analysis/AN-q7k2m-crate-workspace-separation.md)
- Related Tasks:
  - [T-3n2ej-workspace-separation](../tasks/T-3n2ej-workspace-separation/README.md)

## Requirement Statement

The system shall provide a `crates/core` library crate that compiles as pure `no_std` without any dependencies on Embassy async runtime, defmt logging, or hardware abstraction layer (HAL) types.

## Rationale

The current single-crate architecture mixes business logic with platform-specific code, resulting in:

1. **Testing friction**: Host tests require complex feature gates and stub implementations
2. **Architectural coupling**: Business logic cannot be reused across platforms without conditional compilation
3. **Maintenance burden**: \~150 feature gates create cognitive overhead and code duplication

A pure `no_std` core crate enables:

- Native host testing without embedded toolchain
- Clean separation between business logic and platform concerns
- Future platform support (ESP32, STM32) without modifying core algorithms

## User Story (if applicable)

As a developer, I want core business logic in a separate crate with no embedded dependencies, so that I can run unit tests natively on my development machine without feature flags.

## Acceptance Criteria

- [ ] `crates/core/Cargo.toml` exists with `#![no_std]` attribute in `lib.rs`
- [ ] `crates/core` has no dependencies on `embassy-*`, `defmt`, or `rp235x-hal` crates
- [ ] `cargo build -p pico_trail_core` succeeds on host (x86_64) without any feature flags
- [ ] `cargo test -p pico_trail_core` runs and passes on host without embedded toolchain
- [ ] Core crate contains only pure Rust logic: data structures, algorithms, state machines
- [ ] No `#[cfg(feature = ...)]` directives present in `crates/core/src/`

## Technical Details

### Functional Requirement Details

#### Workspace Structure

```toml
# Root Cargo.toml
[workspace]
members = ["crates/core", "crates/firmware"]

# crates/core/Cargo.toml
[package]
name = "pico_trail_core"

[dependencies]
# Only no_std compatible crates allowed:
heapless = "0.8"
libm = "0.2"
# NO: embassy-*, defmt, rp235x-hal, cortex-m, etc.
```

#### Allowed Dependencies in Core

| Allowed               | Forbidden                    |
| --------------------- | ---------------------------- |
| `heapless`            | `embassy-*`                  |
| `libm`                | `defmt`                      |
| `nalgebra` (no_std)   | `rp235x-hal`                 |
| `micromath`           | `cortex-m`                   |
| `embedded-hal` traits | Platform HAL implementations |

#### Module Migration Candidates

The following modules are candidates for migration to core:

- `src/libraries/kinematics/` - Pure math, no async
- `src/libraries/motor_driver/` - Algorithm only, not PWM control
- `src/core/arming/` - State machine logic (excluding async tasks)
- `src/core/mission/` - Mission state management
- `src/rover/mode/` - Mode state machines (tick-based API)
- `src/subsystems/ahrs/` - AHRS algorithms (excluding sensor drivers)

## Platform Considerations

### Embedded (RP2350, future ESP32)

- Core crate used as dependency by firmware crate
- Firmware wraps core logic with Embassy tasks
- No platform-specific code in core

### Host (x86_64 tests)

- Core crate compiles and tests natively
- No `--target` flag required for testing
- Standard Rust toolchain sufficient

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                             | Validation                    |
| ---------------------------------- | ------ | ---------- | -------------------------------------- | ----------------------------- |
| API surface changes break firmware | High   | Medium     | Design stable trait interfaces first   | All firmware examples compile |
| Missing functionality in core      | Medium | Medium     | Phased migration, one module at a time | Feature parity checklist      |
| Increased compile time             | Low    | Low        | Workspace enables parallel compilation | Measure before/after          |

## Implementation Notes

- Start with `libraries/kinematics` as proof-of-concept (zero async dependencies)
- Use tick-based synchronous API pattern for state machines
- Core defines traits, firmware implements them with Embassy
- Avoid `Box::leak()` or other patterns that would require heap

## External References

- [Rust Embedded Book - no_std](https://docs.rust-embedded.org/book/intro/no-std.html) - no_std patterns
- [Cargo Workspaces](https://doc.rust-lang.org/book/ch14-03-cargo-workspaces.html) - Workspace setup
