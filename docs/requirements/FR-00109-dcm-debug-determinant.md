# FR-00109 DCM Debug-Only Determinant Check

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - N/A - No prerequisites
- Dependent Requirements:
  - [NFR-00079-hot-path-performance](../requirements/NFR-00079-hot-path-performance.md)
- Related Tasks:
  - [T-00027-hot-path-optimization](../tasks/T-00027-hot-path-optimization/README.md)

## Requirement Statement

The DCM orthonormalization function shall only compute matrix determinant for drift detection in debug builds, not in release builds.

## Rationale

The `orthonormalize()` function currently computes the 3x3 matrix determinant on every call (100Hz) to detect numerical drift. This computation requires 9 multiplications and 5 additions, adding approximately 1400 floating-point operations per second. Since this check only produces warning logs and does not affect the algorithm's behavior, it should be excluded from release builds to improve performance.

## User Story (if applicable)

The system shall exclude debug-only computations from release builds to maximize performance on resource-constrained embedded targets.

## Acceptance Criteria

- [ ] Determinant calculation in `orthonormalize()` is wrapped with `#[cfg(debug_assertions)]`
- [ ] Release builds (`--release`) do not execute the determinant check
- [ ] Debug builds continue to log warnings when determinant drift is detected
- [ ] All existing tests pass (tests run in debug mode by default)
- [ ] No behavioral change in the orthonormalization algorithm itself

## Technical Details

### Functional Requirement Details

**Current implementation** (`src/subsystems/ahrs/dcm.rs:294-298`):

```rust
// Determinant check (optional, for debugging)
{
    let det = dcm.determinant();
    if (det - 1.0).abs() > 0.1 {
        crate::log_warn!("DCM determinant drift: {}", det);
    }
}
```

**Required change**:

```rust
// Determinant check - debug builds only
#[cfg(debug_assertions)]
{
    let det = dcm.determinant();
    if (det - 1.0).abs() > 0.1 {
        crate::log_warn!("DCM determinant drift: {}", det);
    }
}
```

**Alternative approach** (periodic check):

If determinant monitoring is desired in release builds, implement a counter-based periodic check:

```rust
// Check every 100 calls (1 second at 100Hz) - NOT recommended for this task
static CALL_COUNT: AtomicU32 = AtomicU32::new(0);
if CALL_COUNT.fetch_add(1, Ordering::Relaxed) % 100 == 0 {
    // determinant check
}
```

This alternative is **out of scope** for this requirement but documented for future consideration.

## Platform Considerations

### Unix

N/A - Platform agnostic

### Windows

N/A - Platform agnostic

### Cross-Platform

`#[cfg(debug_assertions)]` is a standard Rust conditional compilation attribute that works identically across all platforms.

## Risks & Mitigation

| Risk                        | Impact | Likelihood | Mitigation                                    | Validation         |
| --------------------------- | ------ | ---------- | --------------------------------------------- | ------------------ |
| Missing drift in production | Medium | Low        | Drift is rare; orthonormalization corrects it | Long-running tests |
| Test coverage gap           | Low    | Low        | Tests run in debug mode                       | Verify test mode   |

## Implementation Notes

- `debug_assertions` is enabled by default in debug builds and disabled in release builds
- The determinant check does not affect algorithm correctness - it only logs warnings
- If drift detection is critical for production, consider the periodic check alternative in a future task

## External References

- [Rust conditional compilation](https://doc.rust-lang.org/reference/conditional-compilation.html#debug_assertions)
