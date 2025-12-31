# FR-jy1tz Standard Library Clamp Function

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - N/A - No prerequisites
- Dependent Requirements:
  - [NFR-1e3n1-hot-path-performance](../requirements/NFR-1e3n1-hot-path-performance.md)
- Related Tasks:
  - [T-lesyx-hot-path-optimization](../tasks/T-lesyx-hot-path-optimization/README.md)

## Requirement Statement

The navigation controller shall use the Rust standard library `f32::clamp()` function instead of custom clamp implementations for value clamping operations.

## Rationale

The Rust standard library's `f32::clamp()` function is optimized by LLVM and can leverage SIMD instructions on supported targets. Custom implementations prevent the compiler from applying these optimizations. Using standard library functions also improves code maintainability and reduces the risk of subtle bugs.

## User Story (if applicable)

The system shall use standard library functions for common operations to ensure optimal performance and maintainability.

## Acceptance Criteria

- [ ] Custom `clamp` function in `src/subsystems/navigation/controller.rs` is removed
- [ ] All usages replaced with `f32::clamp(min, max)` method call syntax
- [ ] `sanitize_output` function updated to use `f32::clamp()`
- [ ] All existing tests pass without modification
- [ ] `cargo clippy` reports no warnings related to the change

## Technical Details

### Functional Requirement Details

**Current implementation** (`src/subsystems/navigation/controller.rs:168`):

```rust
fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min { min } else if value > max { max } else { value }
}
```

**Required change**:

Replace with `value.clamp(min, max)` which is available in Rust std.

**Affected functions**:

- `clamp()` - Remove entirely
- `sanitize_output()` - Update to use `value.clamp(min, max)`
- `calculate_steering()` - Update direct usages if any
- `calculate_throttle()` - Update direct usages if any

## Platform Considerations

### Unix

N/A - Platform agnostic

### Windows

N/A - Platform agnostic

### Cross-Platform

The `f32::clamp()` method is available in Rust std on all platforms including `no_std` with `core`.

## Risks & Mitigation

| Risk                    | Impact | Likelihood | Mitigation                                       | Validation           |
| ----------------------- | ------ | ---------- | ------------------------------------------------ | -------------------- |
| Behavioral difference   | Medium | Very Low   | Standard clamp has identical semantics           | Unit tests verify    |
| NaN handling difference | Low    | Low        | Both propagate NaN; sanitize_output handles this | Test with NaN inputs |

## Implementation Notes

- The `f32::clamp()` method was stabilized in Rust 1.50.0
- Syntax is `value.clamp(min, max)` not `clamp(value, min, max)`
- In `no_std` environments, use `core::cmp::Ord::clamp` or the inherent method on `f32`

## External References

- [f32::clamp documentation](https://doc.rust-lang.org/std/primitive.f32.html#method.clamp)
