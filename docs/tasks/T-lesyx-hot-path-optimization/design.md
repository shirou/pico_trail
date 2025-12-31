# T-lesyx Hot Path Optimization

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-lesyx-hot-path-optimization-plan](plan.md)

## Overview

This task optimizes hot path code executed at 100Hz in the AHRS and navigation subsystems. Three specific changes eliminate unnecessary computations and allocations: replacing a custom clamp function with the standard library version, defining a constant vector for DCM gravity calculations, and gating expensive determinant checks to debug builds only.

## Success Metrics

- [ ] All existing tests pass without modification
- [ ] No behavioral change in control outputs
- [ ] Embedded build succeeds for RP2350 target

## Background and Current State

- Context: AHRS (`Dcm::update`) and navigation controller run at 50-100Hz
- Current behavior:
  - Custom `clamp()` in `src/subsystems/navigation/controller.rs:168`
  - `Vector3::new(0.0, 0.0, 1.0)` created each call in `src/subsystems/ahrs/dcm.rs:144`
  - Unconditional determinant calculation in `src/subsystems/ahrs/dcm.rs:294-298`
- Pain points: Unnecessary CPU cycles in time-critical loops
- Constraints: Must maintain identical external behavior
- Related ADRs: None required

## Proposed Design

### High-Level Architecture

```text
No architectural changes - internal implementation only

Before:                          After:
┌─────────────────────┐         ┌─────────────────────┐
│ NavigationController│         │ NavigationController│
│  clamp() [custom]   │   →     │  f32::clamp() [std] │
└─────────────────────┘         └─────────────────────┘

┌─────────────────────┐         ┌─────────────────────┐
│ Dcm::update()       │         │ Dcm::update()       │
│  Vector3::new(...)  │   →     │  Z_UNIT [const]     │
│  determinant [100Hz]│   →     │  determinant [debug]│
└─────────────────────┘         └─────────────────────┘
```

### Components

**1. Navigation Controller (`src/subsystems/navigation/controller.rs`)**

- Remove: `fn clamp(value: f32, min: f32, max: f32) -> f32`
- Update: `sanitize_output()` to use `value.clamp(min, max)`
- Update: Any direct `clamp()` calls to use method syntax

**2. DCM Module (`src/subsystems/ahrs/dcm.rs`)**

- Add: `const Z_UNIT: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);`
- Update: `Dcm::update()` to use `Z_UNIT` instead of inline construction
- Update: `orthonormalize()` to wrap determinant check in `#[cfg(debug_assertions)]`

### Data Flow

No changes to data flow - same inputs produce same outputs.

### Data Models and Types

No new types or changes to existing types.

### Error Handling

No changes to error handling.

### Security Considerations

N/A - Internal computation changes only.

### Performance Considerations

- **clamp**: Compiler can optimize `f32::clamp()` better than custom code
- **Z_UNIT const**: Eliminates 100Hz stack allocations (12 bytes × 100/s)
- **Determinant**: Saves \~1400 FP operations/second in release builds

### Platform Considerations

#### Unix

N/A - These changes are platform-agnostic.

#### Windows

N/A - These changes are platform-agnostic.

#### Filesystem

N/A - No filesystem operations.

## Alternatives Considered

1. **Periodic determinant check (every N calls)**
   - Pros: Still monitors drift in production
   - Cons: More complex, adds state
   - Decision: Debug-only is simpler; drift is corrected by orthonormalization anyway

2. **Leave custom clamp for explicit control**
   - Pros: No external dependency behavior
   - Cons: Prevents compiler optimization
   - Decision: Standard library is well-tested and optimized

## Migration and Compatibility

- Backward/forward compatibility: No API changes
- Rollout plan: Single PR, all changes together
- Deprecation plan: N/A

## Testing Strategy

### Unit Tests

- Existing tests in `controller.rs` verify navigation behavior
- Existing tests in `dcm.rs` verify AHRS behavior
- No new tests required - semantic behavior unchanged

### Integration Tests

- `cargo test --lib` verifies all unit tests
- `./scripts/build-rp2350.sh` verifies embedded compilation

### External API Parsing (if applicable)

N/A

### Performance & Benchmarks (if applicable)

Not adding benchmarks in this task (out of scope).

## Documentation Impact

- No user-facing documentation changes
- Code comments will document the `Z_UNIT` constant purpose

## Open Questions

None - design is straightforward.

## Appendix

### Examples

**Before (clamp)**:

```rust
fn clamp(value: f32, min: f32, max: f32) -> f32 {
    if value < min { min } else if value > max { max } else { value }
}
// Usage:
clamp(steering, -1.0, 1.0)
```

**After (clamp)**:

```rust
// Usage:
steering.clamp(-1.0, 1.0)
```

**Before (Z vector)**:

```rust
let gravity_body_expected = self.state.dcm_matrix * Vector3::new(0.0, 0.0, 1.0);
```

**After (Z vector)**:

```rust
const Z_UNIT: Vector3<f32> = Vector3::new(0.0, 0.0, 1.0);
// ...
let gravity_body_expected = self.state.dcm_matrix * Z_UNIT;
```

**Before (determinant)**:

```rust
{
    let det = dcm.determinant();
    if (det - 1.0).abs() > 0.1 {
        crate::log_warn!("DCM determinant drift: {}", det);
    }
}
```

**After (determinant)**:

```rust
#[cfg(debug_assertions)]
{
    let det = dcm.determinant();
    if (det - 1.0).abs() > 0.1 {
        crate::log_warn!("DCM determinant drift: {}", det);
    }
}
```
