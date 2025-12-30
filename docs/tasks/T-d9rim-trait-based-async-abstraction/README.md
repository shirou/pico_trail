# T-d9rim Trait-Based Async Abstraction

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-li4m8-feature-gate-reduction](../../analysis/AN-li4m8-feature-gate-reduction.md)
- Related Requirements:
  - [FR-jpmdj-trait-based-async-abstraction](../../requirements/FR-jpmdj-trait-based-async-abstraction.md)
  - [NFR-wl974-feature-gate-reduction](../../requirements/NFR-wl974-feature-gate-reduction.md)
  - [NFR-nmmu0-platform-code-isolation](../../requirements/NFR-nmmu0-platform-code-isolation.md)
- Related ADRs:
  - [ADR-3ciu6-trait-based-async-abstraction](../../adr/ADR-3ciu6-trait-based-async-abstraction.md)
- Associated Design Document:
  - [T-d9rim-design](./design.md)
- Associated Plan Document:
  - [T-d9rim-plan](./plan.md)

## Summary

Introduce trait-based abstractions for async runtime operations (time, synchronization) to decouple core logic from Embassy, reducing feature gates from \~150 to ≤60 while enabling host testing without embedded dependencies.

## Scope

- In scope:
  - Define `TimeSource` trait for time operations
  - Define `SharedState<T>` trait for synchronized state access
  - Implement Embassy and Mock versions of both traits
  - Migrate high-impact modules: `mission/state.rs`, `log_router.rs`, `rc_channel/mod.rs`, `navigation/mod.rs`
  - Remove duplicate `#[cfg(feature)]` / `#[cfg(not(feature))]` implementations
  - Reclassify `pico2_w` gates to `embassy` where appropriate
  - Add CI script to enforce feature gate count limit

- Out of scope:
  - Async trait definitions (use sync traits with async wrappers)
  - Workspace split into multiple crates
  - ESP32/STM32 platform support (future work)
  - Device driver refactoring (separate task)

## Success Metrics

- **Feature gate count**: ≤60 total (from \~150)
- **Platform isolation**: 0 `pico2_w` gates outside `src/platform/`
- **Host tests**: `cargo test --lib` passes without feature flags
- **Embedded build**: `./scripts/build-rp2350.sh` succeeds
- **Performance**: Control loop maintains 50Hz (no regression)
