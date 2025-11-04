# NFR-cy29p Capability Queries Zero Runtime Allocation Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P1
- Category: Performance

## Links

- Parent Analysis: [AN-g5w99-mode-capability-system](../analysis/AN-g5w99-mode-capability-system.md)
- Prerequisite Requirements:
  - [FR-qj0d1-mode-capability-declaration](FR-qj0d1-mode-capability-declaration.md)
- Related Requirements: [FR-sk7ty-mode-capability-queries](FR-sk7ty-mode-capability-queries.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  - [NFR-pvpky-...](NFR-pvpky-capability-query-performance.md)

## Requirement Statement

Capability queries shall have zero runtime allocation overhead because queries are called frequently (mode validation, pre-arm checks, failsafe) and must be efficient with inline functions or vtable calls with no heap allocation.

## Rationale

Capability queries called at high frequency (every mode change, arm attempt, failsafe check). Heap allocations would introduce performance overhead and memory fragmentation. Zero-allocation design ensures queries are lightweight and fast.

## Measurement Criteria

- Heap allocations: 0 during capability queries
- Implementation: Inline functions or trait methods (vtable)
- Measurement method: Memory allocation profiling
- Target: No allocations detected during query execution

## Acceptance Criteria

1. Capability query implementation:
   - Trait methods returning primitive types (bool)
   - Inline or vtable dispatch only
   - No String allocations, no Vec allocations, no heap usage
2. Query execution profile:
   - Run capability query tests under allocator profiling
   - Verify zero heap allocations
   - Check for hidden allocations (e.g., in panic paths)
3. Performance testing:
   - Micro-benchmark: 1M+ capability queries
   - Verify no allocation overhead
   - Memory usage stable over time
4. Code review:
   - Verify query methods return primitives only
   - Check for allocation-free implementation

## Success Metrics

- Zero heap allocations during capability queries
- Query performance: < 1µs per query
- No memory growth over extended operation
- Allocation profiler confirms zero allocations

## Verification Methods

- Allocation profiling: Monitor heap allocations during queries
- Micro-benchmarks: Measure query performance and allocations
- Code review: Verify allocation-free implementation
- HITL testing: Long-duration operation, monitor memory

## ArduPilot Comparison

ArduPilot capability queries are zero-allocation:

- Virtual methods returning primitive bool
- No dynamic allocation in query paths
- Inline or vtable dispatch only

## Notes

- Target: Trait method overhead only (vtable lookup \~1-5 CPU cycles)
- Consider using `#[inline]` attribute for hot paths
- Queries should be pure functions (no side effects)
- Phase 1: Basic profiling; Phase 2: Optimization if needed

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
