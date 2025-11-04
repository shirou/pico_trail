# NFR-u91xr Mode Exit Completion Within 5 Milliseconds

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P2
- Category: Performance

## Links

- Parent Analysis: [AN-9rbvh-mode-lifecycle-management](../analysis/AN-9rbvh-mode-lifecycle-management.md)
- Related Requirements: [FR-7e0cr-mode-lifecycle-management](FR-7e0cr-mode-lifecycle-management.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Mode exit shall complete within 5ms from exit start to completion to enable fast cleanup and responsive mode transitions.

## Rationale

Exit timing contributes to total mode transition time. Fast cleanup ensures responsive transitions. Simple cleanup logic should execute quickly without complex operations.

## Measurement Criteria

- Exit timing: Measure from exit() call start to return
- Target: < 5ms mean, < 10ms 99th percentile
- Measurement method: Timestamp before/after exit() call
- Test across all modes

## Acceptance Criteria

1. Exit timing measurement:
   - Instrument exit() with timestamps
   - Calculate duration = end_time - start_time
   - Log slow exits (> 5ms) with warning
2. Performance targets:
   - Mean exit time: < 5ms
   - 99th percentile: < 10ms
   - Typical: < 1ms (most modes have minimal cleanup)
3. Exit optimization:
   - Keep cleanup logic simple
   - Avoid expensive operations during exit
   - Defer complex cleanup if possible
4. Performance monitoring:
   - Log exit timing per mode
   - Track maximum exit time

## Success Metrics

- Mean exit time < 5ms across all modes
- Total transition time (exit + entry) < 15ms
- No operator-reported delays
- Consistent exit timing

## Verification Methods

- Micro-benchmarks: Measure exit() per mode
- HITL testing: Profile on hardware
- Integration testing: Measure full transition timing
- Code review: Verify simple exit logic

## ArduPilot Comparison

ArduPilot mode exit typically completes in:

- Most modes: < 1ms (minimal cleanup)
- Complex modes: < 5ms
- Exit is typically faster than entry

## Notes

- Exit should be simple (best-effort cleanup)
- Avoid blocking operations during exit
- Most modes have minimal cleanup needs
- Target hardware: RP2350 @ 150 MHz

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
