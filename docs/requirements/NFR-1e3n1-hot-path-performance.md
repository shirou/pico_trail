# NFR-1e3n1 Hot Path Performance Optimization

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-jy1tz-std-clamp-function](../requirements/FR-jy1tz-std-clamp-function.md)
  - [FR-ja9e4-dcm-constant-vectors](../requirements/FR-ja9e4-dcm-constant-vectors.md)
  - [FR-acgr4-dcm-debug-determinant](../requirements/FR-acgr4-dcm-debug-determinant.md)
- Dependent Requirements:
  - N/A
- Related Tasks:
  - [T-lesyx-hot-path-optimization](../tasks/T-lesyx-hot-path-optimization/README.md)

## Requirement Statement

Hot path code executed at 100Hz or higher frequency shall avoid unnecessary computations, allocations, and debug-only operations in release builds to maintain consistent control loop timing.

## Rationale

The pico_trail system runs on an RP2350 microcontroller (Cortex-M33 at 150MHz) with limited CPU budget. Control loops must complete within their timing budget (10ms for 100Hz) to ensure stable vehicle operation. Unnecessary operations in hot paths reduce headroom for additional features and can cause timing violations under load.

## User Story (if applicable)

The system shall maintain consistent 100Hz control loop timing under all operating conditions to ensure stable vehicle control.

## Acceptance Criteria

- [ ] No custom implementations of standard library functions in hot paths
- [ ] Constant values are defined at compile time, not runtime
- [ ] Debug-only computations are gated with `#[cfg(debug_assertions)]`
- [ ] Release builds show measurable reduction in hot path operations
- [ ] All control loops maintain their target frequency

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Performance targets**:

- Hot path functions should complete in < 100μs (1% of 10ms budget)
- No heap allocations in control loops
- Constant vectors/matrices defined at compile time

**Affected hot paths**:

1. `Dcm::update()` - 100Hz AHRS update
2. `NavigationController::update()` - 50Hz navigation
3. `orthonormalize()` - Called from DCM update

**Measurement approach**:

Since embedded profiling is limited, verification is through:

- Code review for eliminated operations
- Embedded build size comparison (smaller = fewer operations)
- Timing tests if probe-rs timing is available

## Platform Considerations

### Unix

N/A - This requirement targets embedded RP2350

### Windows

N/A - This requirement targets embedded RP2350

### Cross-Platform

The optimizations benefit all platforms but are most critical for the resource-constrained RP2350 target.

## Risks & Mitigation

| Risk                   | Impact | Likelihood | Mitigation                       | Validation                |
| ---------------------- | ------ | ---------- | -------------------------------- | ------------------------- |
| Over-optimization      | Low    | Low        | Only optimize verified hot paths | Profile before optimizing |
| Code clarity reduction | Medium | Low        | Document why optimizations exist | Code comments             |
| Behavioral change      | High   | Very Low   | Use semantic-preserving changes  | Comprehensive tests       |

## Implementation Notes

- Focus on changes that are guaranteed to reduce work, not micro-optimizations
- Prefer readability when performance impact is negligible
- Document any non-obvious optimizations for future maintainers
- These specific optimizations were identified through code review, not profiling, so impact estimates are theoretical

## External References

- [Rust Performance Book](https://nnethercote.github.io/perf-book/)
- [Embedded Rust optimization](https://docs.rust-embedded.org/book/unsorted/speed-vs-size.html)
