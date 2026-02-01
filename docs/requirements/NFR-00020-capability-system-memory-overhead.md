# NFR-00020 Capability System Memory Overhead Under 10 Bytes Per Mode Instance

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P2
- Category: Resource Constraints

## Links

- Parent Analysis: [AN-00012-mode-capability-system](../analysis/AN-00012-mode-capability-system.md)
- Related Requirements:
  - [FR-00045-mode-capability-queries](FR-00045-mode-capability-queries.md)
  - [NFR-00056-validation-memory-overhead](NFR-00056-validation-memory-overhead.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Capability system shall add no more than 10 bytes RAM per mode instance because limited RAM on RP2040/RP2350 (264 KB) requires efficient design. Trait object vtable is only overhead (\~8 bytes pointer + vtable reference).

## Rationale

Capability queries implemented as trait methods compile to code, not data. No per-mode capability storage needed. Only overhead is trait object vtable for dynamic dispatch. Minimal memory impact enables scalability.

## Measurement Criteria

- Per-mode overhead: < 10 bytes
- Components measured:
  - Trait object pointer: 8 bytes (fat pointer)
  - Capability data: 0 bytes (code only, no data)
  - Total: \~8 bytes per mode instance
- Measurement method: Size profiling, linker map analysis

## Acceptance Criteria

1. Memory overhead analysis:
   - Profile mode struct sizes with capability trait
   - Verify no additional data fields for capabilities
   - Confirm vtable overhead only
2. Zero capability data storage:
   - Capability queries return computed values
   - No bitmasks or capability flags stored
   - All capability logic in code, not data
3. Size targets:
   - Trait object overhead: \~8 bytes
   - No per-capability data: 0 bytes
   - Total overhead: < 10 bytes
4. Memory regression testing:
   - CI monitors mode struct sizes
   - Fail if capability overhead exceeds 10 bytes

## Success Metrics

- Capability overhead < 10 bytes per mode
- No per-capability data storage detected
- Memory usage stable over time
- Size profiling confirms expectations

## Verification Methods

- Size profiling: cargo bloat analysis
- Linker map analysis: Verify capability data absence
- HITL testing: Monitor RAM usage on hardware
- Static analysis: Confirm code-only implementation

## ArduPilot Comparison

ArduPilot capability system memory overhead:

- Virtual method table pointer: 4-8 bytes (architecture dependent)
- No per-capability data storage
- Capability queries are virtual methods (code only)

## Notes

- Capability queries compile to code (trait methods)
- No runtime capability data structures
- Trait object overhead unavoidable for dynamic dispatch
- Consider static dispatch for performance-critical paths (Phase 2)
