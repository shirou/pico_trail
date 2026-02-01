# T-00025 Trait-Based Async Abstraction

## Metadata

- Type: Task
- Status: Completed

## Links

- Related Analyses:
  - [AN-00029-feature-gate-reduction](../../analysis/AN-00029-feature-gate-reduction.md)
- Related Requirements:
  - [FR-00106-trait-based-async-abstraction](../../requirements/FR-00106-trait-based-async-abstraction.md)
  - [NFR-00077-feature-gate-reduction](../../requirements/NFR-00077-feature-gate-reduction.md)
  - [NFR-00005-platform-code-isolation](../../requirements/NFR-00005-platform-code-isolation.md)
- Related ADRs:
  - [ADR-00027-trait-based-async-abstraction](../../adr/ADR-00027-trait-based-async-abstraction.md)
- Associated Design Document:
  - [T-00025-design](./design.md)
- Associated Plan Document:
  - [T-00025-plan](./plan.md)

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
