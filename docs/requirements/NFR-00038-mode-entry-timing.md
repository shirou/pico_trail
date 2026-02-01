# NFR-00038 Mode Entry Completion Within 10 Milliseconds

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P1
- Category: Performance

## Links

- Parent Analysis: [AN-00014-mode-lifecycle-management](../analysis/AN-00014-mode-lifecycle-management.md)
- Related Requirements: [FR-00048-mode-lifecycle-management](FR-00048-mode-lifecycle-management.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Mode entry shall complete within 10ms from entry start to completion (including validation) to ensure fast mode changes responsive to operator commands.

## Rationale

Operators expect immediate response to mode change commands. Delays make vehicle feel sluggish and reduce operator confidence. Emergency mode changes (e.g., failsafe) require rapid execution.

## Measurement Criteria

- Entry timing: Measure from enter() call start to return
- Target: < 10ms mean, < 20ms 99th percentile
- Measurement method: Timestamp before/after enter() call
- Test across all modes and all entry scenarios

## Acceptance Criteria

1. Entry timing measurement:
   - Instrument enter() with timestamps
   - Calculate duration = end_time - start_time
   - Log entry timing for analysis
2. Performance targets:
   - Mean entry time: < 10ms
   - 99th percentile: < 20ms
   - Maximum: < 50ms (pathological cases)
3. Optimization if needed:
   - Optimize validation checks (< 1ms)
   - Minimize state initialization overhead
   - Cache sensor health status
4. Performance regression testing:
   - CI measures entry timing
   - Fail build if entry exceeds 10ms mean

## Success Metrics

- Mean entry time < 10ms across all modes
- No operator-reported delays
- Performance stable over time
- Emergency mode changes complete quickly

## Verification Methods

- Micro-benchmarks: Measure enter() per mode
- HITL testing: Profile on target hardware
- Statistical analysis: 1000+ entry measurements
- Stress testing: Entry timing under load

## ArduPilot Comparison

ArduPilot mode entry typically completes in:

- Validation: < 1ms
- Mode-specific init: < 5ms
- Total: < 10ms for most modes

## Notes

- Target hardware: RP2350 @ 150 MHz
- Budget: < 1.5M CPU cycles @ 150 MHz
- Most time in validation and sensor queries
- Phase 1: Measurement; Phase 2: Optimization if needed
