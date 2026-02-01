# NFR-00086 SmartRTL Memory Budget

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00123-smartrtl-path-recording](FR-00123-smartrtl-path-recording.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00036-rtl-mode](../analysis/AN-00036-rtl-mode.md)
- Related Tasks:
  - [T-00030-rtl-smartrtl-implementation](../tasks/T-00030-rtl-smartrtl-implementation/README.md)

## Requirement Statement

SmartRTL path buffer shall use statically allocated memory with configurable capacity (SRTL_POINTS), defaulting to 300 points (\~9KB) with a maximum of 500 points (\~15KB).

## Rationale

Embedded targets have limited RAM (RP2040: 264KB, RP2350: 520KB). SmartRTL path storage must:

- Fit within available RAM alongside other subsystems
- Use static allocation (no heap) for predictability
- Scale with SRTL_POINTS parameter for user control
- Balance path length vs memory usage

ArduPilot uses \~3KB per 100 points; this implementation targets \~30 bytes/point for similar efficiency.

## User Story

The system shall provide configurable path storage within memory constraints to balance mission length capability with embedded resource limits.

## Acceptance Criteria

- [ ] Path buffer uses static allocation (no heap/alloc)
- [ ] Default capacity: 300 points (\~9KB)
- [ ] Maximum capacity: 500 points (\~15KB)
- [ ] SRTL_POINTS parameter controls buffer size
- [ ] Memory usage verified via compile-time size checks
- [ ] Buffer size documented for operators

## Technical Details

### Non-Functional Requirement Details

**Memory Breakdown:**

| Component       | Size per Point | 300 Points | 500 Points  |
| --------------- | -------------- | ---------- | ----------- |
| latitude (f64)  | 8 bytes        | 2,400 B    | 4,000 B     |
| longitude (f64) | 8 bytes        | 2,400 B    | 4,000 B     |
| timestamp (u32) | 4 bytes        | 1,200 B    | 2,000 B     |
| index/metadata  | \~10 bytes     | 3,000 B    | 5,000 B     |
| **Total**       | **\~30 bytes** | **\~9 KB** | **\~15 KB** |

**Static Allocation:**

```rust
// Compile-time buffer sizing
const MAX_SRTL_POINTS: usize = 500;

pub struct PathBuffer {
    points: [PathPoint; MAX_SRTL_POINTS],
    head: usize,
    tail: usize,
    count: usize,
}
```

**SRTL_POINTS Parameter:**

- Range: 50 to 500
- Default: 300
- Applied as runtime limit within static buffer

**Verification:**

```rust
#[test]
fn test_path_buffer_size() {
    assert!(core::mem::size_of::<PathBuffer>() <= 15360); // 15KB
}
```

## Platform Considerations

### RP2040 (264KB RAM)

- 15KB for SmartRTL is \~6% of total RAM
- Reasonable allocation leaving room for other subsystems

### RP2350 (520KB RAM)

- 15KB for SmartRTL is \~3% of total RAM
- Generous headroom for future expansion

## Risks & Mitigation

| Risk                          | Impact | Likelihood | Mitigation                     | Validation              |
| ----------------------------- | ------ | ---------- | ------------------------------ | ----------------------- |
| Memory pressure from buffer   | Medium | Low        | 15KB max is conservative       | Profile total RAM usage |
| Compile-time size too large   | Low    | Low        | Configurable via feature flag  | Build verification      |
| Points too few for long paths | Medium | Medium     | Path simplification (FR-00124) | Test with long missions |

## Implementation Notes

- Use `static` allocation in path_recorder.rs
- Consider `MaybeUninit` for efficient initialization
- SRTL_POINTS runtime check limits used capacity
- Future: Consider compressed storage (f32 instead of f64)

## External References

- [ArduPilot SRTL_POINTS](https://ardupilot.org/rover/docs/parameters.html#srtl-points) - Points parameter reference
