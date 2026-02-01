# FR-00108 DCM Constant Vector Definitions

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

The DCM (Direction Cosine Matrix) algorithm shall define constant vectors as compile-time constants rather than creating them at runtime on each function call.

## Rationale

The `Dcm::update()` function runs at 100Hz and currently creates `Vector3::new(0.0, 0.0, 1.0)` on every call. While stack allocation is fast, this is unnecessary work that can be eliminated by defining the vector as a `const`. This reduces CPU cycles and improves determinism in the control loop.

## User Story (if applicable)

The system shall minimize unnecessary allocations in high-frequency control loops to ensure consistent timing and efficient CPU usage.

## Acceptance Criteria

- [ ] A `const Z_UNIT: Vector3<f32>` is defined in `src/subsystems/ahrs/dcm.rs`
- [ ] `Dcm::update()` uses the constant instead of `Vector3::new(0.0, 0.0, 1.0)`
- [ ] The constant is properly documented with its purpose
- [ ] All existing DCM tests pass without modification
- [ ] No behavioral change in attitude estimation

## Technical Details

### Functional Requirement Details

**Current implementation** (`src/subsystems/ahrs/dcm.rs:144`):

```rust
let gravity_body_expected = self.state.dcm_matrix * Vector3::new(0.0, 0.0, 1.0);
```

**Required change**:

```rust
/// Z-axis unit vector representing gravity direction in NED frame
const Z_UNIT: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);

// In update():
let gravity_body_expected = self.state.dcm_matrix * Z_UNIT;
```

**Note**: nalgebra's `Vector3::new()` is a `const fn`, so this should work. If not, use:

```rust
const Z_UNIT: Vector3<f32> = Vector3::<f32>::from_array_storage(
    ArrayStorage([[0.0], [0.0], [1.0]])
);
```

Or the simpler approach using `vector!` macro if available.

## Platform Considerations

### Unix

N/A - Platform agnostic

### Windows

N/A - Platform agnostic

### Cross-Platform

nalgebra works identically across all platforms. Const evaluation is handled at compile time.

## Risks & Mitigation

| Risk                      | Impact | Likelihood | Mitigation                         | Validation                |
| ------------------------- | ------ | ---------- | ---------------------------------- | ------------------------- |
| nalgebra const limitation | Low    | Low        | Use array initialization if needed | Compile-time verification |
| Behavioral change         | High   | Very Low   | Same mathematical value            | Existing tests            |

## Implementation Notes

- Check nalgebra version for `const fn` support on `Vector3::new()`
- If const construction is not available, consider using a lazy_static or OnceCell, though this is less ideal
- The constant should be module-private unless needed elsewhere

## External References

- [nalgebra documentation](https://docs.rs/nalgebra/latest/nalgebra/)
