# T-00035 Workspace Separation

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00039-crate-workspace-separation](../../analysis/AN-00039-crate-workspace-separation.md)
  - [AN-00029-feature-gate-reduction](../../analysis/AN-00029-feature-gate-reduction.md)
- Related Requirements:
  - [FR-00130-core-crate-nostd-purity](../../requirements/FR-00130-core-crate-nostd-purity.md)
  - [FR-00132-trait-abstractions-platform-services](../../requirements/FR-00132-trait-abstractions-platform-services.md)
  - [FR-00131-external-impl-observability](../../requirements/FR-00131-external-impl-observability.md)
  - [NFR-00089-zero-cfg-core-crate](../../requirements/NFR-00089-zero-cfg-core-crate.md)
  - [NFR-00088-ci-core-purity-lint](../../requirements/NFR-00088-ci-core-purity-lint.md)
- Related ADRs:
  - (To be created)
- Associated Design Document:
  - [T-00035-design](./design.md)
- Associated Plan Document:
  - [T-00035-plan](./plan.md)

## Summary

Refactor pico_trail from a single-crate architecture into a Cargo Workspace with `crates/core` (pure `no_std` business logic) and `crates/firmware` (Embassy/RP2350 entry point), eliminating all `#[cfg(feature = ...)]` directives from core business logic.

## Scope

- In scope:
  - Create Cargo Workspace structure with root `Cargo.toml`
  - Initialize `crates/core` as pure `no_std` library
  - Initialize `crates/firmware` as Embassy binary crate
  - Define trait abstractions (`TimeSource`) in core
  - Migrate business logic modules to core (kinematics, arming, mission, navigation, modes)
  - Implement Embassy trait implementations in firmware
  - Create external `defmt::Format` impls in firmware for core types
  - Add CI lint script to enforce zero cfg in core
  - Update build scripts and documentation

- Out of scope:
  - ESP32/STM32 platform support (future work)
  - MAVLink transport refactoring (complex, separate task)
  - Device driver abstraction (separate task)
  - Performance optimization (separate task)

## Success Metrics

- **Zero cfg in core**: `grep -r "#\[cfg(feature" crates/core/src/ | wc -l` returns 0
- **Host tests pass**: `cargo test -p pico_trail_core` succeeds without feature flags
- **Embedded build**: `./scripts/build-rp2350.sh` compiles successfully
- **Feature parity**: All existing examples continue to function
- **CI enforcement**: `scripts/check-core-no-cfg.sh` integrated and passing
