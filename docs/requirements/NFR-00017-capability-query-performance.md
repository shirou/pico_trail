# NFR-00017 Capability Query Execution Under 1 Microsecond Per Query

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P2
- Category: Performance

## Links

- Parent Analysis: [AN-00012-mode-capability-system](../analysis/AN-00012-mode-capability-system.md)
- Prerequisite Requirements:
  - [FR-00044-mode-capability-declaration](FR-00044-mode-capability-declaration.md)
- Related Requirements: [NFR-00018-capability-query-zero-allocation](NFR-00018-capability-query-zero-allocation.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Capability query execution shall complete within 1 µs per query because multiple queries are called per mode operation (validation, arming, failsafe) and must be fast to avoid cumulative overhead.

## Rationale

Mode validation calls 5-6 capability queries, taking 5-6µs total if each query is 1µs. This is acceptable overhead for validation timing budget (< 1ms). Slower queries would accumulate and impact validation performance.

## Measurement Criteria

- Per-query execution time: < 1µs
- Full validation (6 queries): < 6µs
- Measurement method: Micro-benchmarks using system timer
- Target hardware: RP2350 @ 150 MHz

## Acceptance Criteria

1. Query timing micro-benchmarks:
   - Measure each capability query execution time
   - Run 10,000+ iterations for statistical significance
   - Calculate mean, max, p99 timing
2. Performance targets:
   - Mean query time: < 1µs
   - 99th percentile: < 2µs
   - Maximum: < 5µs
3. Full validation timing:
   - 6 queries (full validation): < 10µs total
   - Acceptable overhead in 1ms validation budget
4. Performance optimization:
   - Consider `#[inline]` for hot paths
   - Optimize trait method dispatch if needed

## Success Metrics

- Mean query time < 1µs
- Full validation < 10µs
- No performance degradation over time
- Query overhead negligible in validation timing

## Verification Methods

- Micro-benchmarks: Measure individual query timing
- Statistical analysis: Mean, p99, max from 10K+ iterations
- Integration timing: Measure full validation with queries
- HITL testing: Profile query timing on hardware

## ArduPilot Comparison

ArduPilot capability queries execute in:

- Virtual method call: 1-10 CPU cycles (\~0.01-0.1µs @ 100 MHz)
- Inline method: < 1 CPU cycle
- Total negligible compared to validation overhead

## Notes

- Target: < 150 CPU cycles @ 150 MHz = 1µs
- Trait vtable dispatch: \~5-10 cycles
- Simple bool return: negligible
- Consider inlining for hot paths
