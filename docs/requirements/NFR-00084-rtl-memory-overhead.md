# NFR-00084 RTL Mode Memory Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00122-rtl-navigate-home](FR-00122-rtl-navigate-home.md)
- Dependent Requirements: None
- Related Analyses:
  - [AN-00036-rtl-mode](../analysis/AN-00036-rtl-mode.md)
- Related Tasks:
  - [T-00030-rtl-smartrtl-implementation](../tasks/T-00030-rtl-smartrtl-implementation/README.md)

## Requirement Statement

RTL mode implementation shall add no more than 100 bytes of RAM overhead, ensuring the mode fits within the memory constraints of RP2040/RP2350 embedded targets.

## Rationale

Embedded targets have limited RAM:

- RP2040: 264 KB RAM
- RP2350: 520 KB RAM

Each mode must be memory-efficient to allow multiple modes and subsystems to coexist. 100 bytes is a conservative budget aligned with other mode implementations.

## User Story

The system shall implement RTL mode within 100 bytes RAM to ensure memory efficiency on embedded targets.

## Acceptance Criteria

- [ ] RtlMode struct size <= 100 bytes
- [ ] No dynamic memory allocation during RTL operation
- [ ] Memory usage verified via `core::mem::size_of::<RtlMode>()`
- [ ] Memory overhead documented in code comments

## Technical Details

### Non-Functional Requirement Details

**Memory Budget Breakdown:**

| Component                  | Estimated Size | Notes                  |
| -------------------------- | -------------- | ---------------------- |
| SimpleNavigationController | \~16 bytes     | Config and state       |
| PositionTarget             | \~12 bytes     | lat/lon/alt            |
| arrived flag               | 1 byte         | bool                   |
| Padding/alignment          | \~3 bytes      | Struct alignment       |
| **Total RtlMode**          | **\~32 bytes** | Well under 100B budget |

**Verification Method:**

```rust
#[test]
fn test_rtl_mode_size() {
    assert!(core::mem::size_of::<RtlMode>() <= 100);
}
```

**No Dynamic Allocation:**

- All state is stack-allocated in RtlMode struct
- No Vec, String, or Box usage
- No heap allocation during update()

## Platform Considerations

### RP2040/RP2350

Memory constraints are primary driver for this requirement. The 100-byte budget ensures RTL fits comfortably alongside other subsystems.

## Risks & Mitigation

| Risk                          | Impact | Likelihood | Mitigation                  | Validation          |
| ----------------------------- | ------ | ---------- | --------------------------- | ------------------- |
| Future features exceed budget | Low    | Low        | Review before adding state  | size_of test        |
| Struct alignment waste        | Low    | Low        | Field ordering optimization | Verify with size_of |

## Implementation Notes

- Use `#[repr(C)]` if precise layout needed
- Avoid storing redundant data (e.g., don't cache home position)
- Consider: Compile-time size assertion

## External References

N/A - No external references
