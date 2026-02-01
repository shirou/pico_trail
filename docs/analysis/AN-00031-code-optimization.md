# AN-00031 Code Optimization for Hot Paths

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - N/A - First optimization analysis
- Related Requirements:
  - [FR-00110-std-clamp-function](../requirements/FR-00110-std-clamp-function.md)
  - [FR-00108-dcm-constant-vectors](../requirements/FR-00108-dcm-constant-vectors.md)
  - [FR-00109-dcm-debug-determinant](../requirements/FR-00109-dcm-debug-determinant.md)
  - [NFR-00079-hot-path-performance](../requirements/NFR-00079-hot-path-performance.md)
- Related ADRs:
  - N/A - No architectural changes required
- Related Tasks:
  - [T-00027-hot-path-optimization](../tasks/T-00027-hot-path-optimization/README.md)

## Executive Summary

This analysis identifies optimization opportunities in pico_trail's hot paths, focusing on code executed at high frequency (100Hz control loops). Three primary areas were discovered: a custom `clamp` function that should use the standard library, repeated vector allocation in the DCM algorithm, and potentially unnecessary determinant calculations. These optimizations can improve CPU efficiency on the resource-constrained RP2350 microcontroller without changing external behavior.

## Problem Space

### Current State

The codebase contains several patterns that, while functionally correct, may introduce unnecessary overhead in hot paths:

1. **Custom `clamp` function** (`src/subsystems/navigation/controller.rs:168`): Hand-written clamp logic instead of using `f32::clamp()` from the standard library. LLVM can better optimize the standard function.

2. **Repeated vector allocation** (`src/subsystems/ahrs/dcm.rs:144`): `Vector3::new(0.0, 0.0, 1.0)` is created on every call to `Dcm::update()`, which runs at 100Hz. This constant vector should be defined once.

3. **Unconditional determinant calculation** (`src/subsystems/ahrs/dcm.rs:294-298`): The `orthonormalize` function computes the matrix determinant on every call (100Hz) for drift detection. This is computationally expensive (9 multiplications + 5 additions) and may not be needed in production.

### Desired State

- Use Rust standard library functions where available for better compiler optimization
- Define constant values once rather than recreating them repeatedly
- Move expensive debug-only computations behind feature gates or conditional execution
- Maintain identical external behavior with improved internal efficiency

### Gap Analysis

| Area              | Current               | Desired                 | Impact                         |
| ----------------- | --------------------- | ----------------------- | ------------------------------ |
| Clamp function    | Custom implementation | Standard `f32::clamp()` | Minor CPU, better optimization |
| Z-unit vector     | Allocated per call    | Const definition        | Eliminates 100Hz allocations   |
| Determinant check | Every call            | Debug-only or periodic  | Reduces 100Hz computation      |

## Stakeholder Analysis

| Stakeholder     | Interest/Need           | Impact | Priority |
| --------------- | ----------------------- | ------ | -------- |
| Embedded system | CPU efficiency          | Medium | P1       |
| Control loops   | Consistent 100Hz timing | High   | P0       |
| Developers      | Clean, idiomatic code   | Low    | P2       |

## Research & Discovery

### Technical Investigation

**1. Custom clamp vs std::f32::clamp**

The custom implementation:

```rust
fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min { min } else if value > max { max } else { value }
}
```

vs `f32::clamp()` which the compiler can optimize with SIMD instructions on supported targets. Godbolt analysis shows the standard library version generates more efficient assembly on ARM Cortex-M.

**2. Vector allocation analysis**

`Vector3::new(0.0, 0.0, 1.0)` creates a new stack-allocated struct each call. While cheap, this is redundant for a constant value. A `const` definition eliminates the initialization entirely.

**3. Determinant computation cost**

Matrix determinant for 3x3:

```
det = a(ei − fh) − b(di − fg) + c(dh − eg)
```

This requires 9 multiplications and 5 additions/subtractions. At 100Hz, this adds \~1400 floating-point operations per second that are only used for logging warnings.

### Data Analysis

Estimated impact per second at 100Hz update rate:

| Optimization             | Operations saved/s  | Memory saved  |
| ------------------------ | ------------------- | ------------- |
| Z-unit vector const      | 300 stores          | 12 bytes/call |
| Determinant (debug-only) | 1400 FP ops         | N/A           |
| clamp std lib            | Variable (compiler) | N/A           |

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The DCM update function shall use constant values for fixed vectors → Will become FR-xxx
  - Rationale: Eliminates redundant stack allocations in hot paths
  - Acceptance Criteria: `const Z_UNIT: Vector3<f32>` defined and used

- [ ] **FR-DRAFT-2**: Navigation controller shall use standard library clamp function → Will become FR-xxx
  - Rationale: Better compiler optimization for common operations
  - Acceptance Criteria: Custom `clamp` replaced with `f32::clamp()`

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Debug-only computations shall not impact production performance → Will become NFR-xxx
  - Category: Performance
  - Rationale: Production builds should not include expensive debug code
  - Target: Determinant check removed from release builds or executed periodically

## Design Considerations

### Technical Constraints

- Must maintain identical external behavior
- Changes must not affect test results
- RP2350 target: Cortex-M33, 150MHz, limited power budget

### Potential Approaches

1. **Option A**: Minimal changes - Only replace clamp function
   - Pros: Lowest risk, quick win
   - Cons: Misses other optimization opportunities
   - Effort: Low

2. **Option B**: Comprehensive optimization - All three areas
   - Pros: Maximum benefit, consistent codebase
   - Cons: More testing required
   - Effort: Medium

3. **Option C**: Option B + add benchmarks for future optimization
   - Pros: Establishes performance baseline
   - Cons: Additional infrastructure work
   - Effort: High

### Architecture Impact

No architectural changes required. These are internal implementation optimizations that do not affect module interfaces or data flow.

## Risk Assessment

| Risk                   | Probability | Impact | Mitigation Strategy                |
| ---------------------- | ----------- | ------ | ---------------------------------- |
| Behavioral change      | Low         | High   | Comprehensive test coverage        |
| Performance regression | Very Low    | Medium | Verify with profiling if available |
| Build breakage         | Low         | Low    | CI verification                    |

## Open Questions

- [x] Are there other hot paths with similar optimization opportunities? → Searched codebase, these are the primary candidates
- [ ] Should we add benchmarks for control loop performance? → Defer to future task

## Recommendations

### Immediate Actions

1. Replace custom `clamp` with `f32::clamp()`
2. Define `const Z_UNIT: Vector3<f32>` for DCM gravity vector
3. Gate determinant check with `#[cfg(debug_assertions)]`

### Next Steps

1. [ ] Create formal requirements: FR-xxx (code optimization), NFR-xxx (performance)
2. [ ] Create task for: Implementation of optimizations
3. [ ] Verify optimizations with embedded build

### Out of Scope

- Adding performance benchmarking infrastructure (separate future work)
- Optimizing non-hot-path code
- Algorithm changes (only implementation improvements)

## Appendix

### References

- Rust std::cmp - <https://doc.rust-lang.org/std/cmp/trait.Ord.html#method.clamp>
- nalgebra const constructors - <https://docs.rs/nalgebra/latest/nalgebra/>

### Raw Data

**Locations identified:**

```
src/subsystems/navigation/controller.rs:168 - custom clamp
src/subsystems/ahrs/dcm.rs:144 - Vector3::new(0.0, 0.0, 1.0)
src/subsystems/ahrs/dcm.rs:294-298 - determinant calculation
```
