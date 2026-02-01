# T-00027 Hot Path Optimization

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-00031-code-optimization](../../analysis/AN-00031-code-optimization.md)
- Related Requirements:
  - [FR-00110-std-clamp-function](../../requirements/FR-00110-std-clamp-function.md)
  - [FR-00108-dcm-constant-vectors](../../requirements/FR-00108-dcm-constant-vectors.md)
  - [FR-00109-dcm-debug-determinant](../../requirements/FR-00109-dcm-debug-determinant.md)
  - [NFR-00079-hot-path-performance](../../requirements/NFR-00079-hot-path-performance.md)
- Related ADRs:
  - N/A - No architectural changes required
- Associated Design Document:
  - [T-00027-hot-path-optimization-design](design.md)
- Associated Plan Document:
  - [T-00027-hot-path-optimization-plan](plan.md)

## Summary

Optimize hot path code in the navigation controller and DCM AHRS modules to improve CPU efficiency on the RP2350 microcontroller. Changes include replacing custom functions with standard library equivalents, defining compile-time constants, and gating debug-only computations.

## Scope

- In scope:
  - Replace custom `clamp` function with `f32::clamp()` in navigation controller
  - Define `const Z_UNIT` vector in DCM module
  - Gate determinant check with `#[cfg(debug_assertions)]`
  - Update all affected call sites
  - Verify all existing tests pass
- Out of scope:
  - Algorithm changes
  - Adding performance benchmarks (future task)
  - Optimizing non-hot-path code

## Success Metrics

- All existing unit tests pass without modification
- `cargo clippy` reports no new warnings
- Embedded build (`./scripts/build-rp2350.sh`) succeeds
- No behavioral changes in navigation or AHRS output
