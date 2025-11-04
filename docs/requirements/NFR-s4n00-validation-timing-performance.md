# NFR-s4n00 Mode Entry Validation Performance Under 1 Millisecond

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P1
- Category: Performance

## Links

- Parent Analysis: [AN-5aniu-mode-entry-validation](../analysis/AN-5aniu-mode-entry-validation.md)
- Related Requirements: [FR-a9rc3-mode-entry-sensor-validation](FR-a9rc3-mode-entry-sensor-validation.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Mode entry validation shall complete within 1ms of mode change command to ensure validation does not introduce perceptible delay in mode transitions.

## Rationale

Mode changes must be responsive to operator commands. Validation delays would make the vehicle feel sluggish and could interfere with emergency mode changes (e.g., switching to Manual during failsafe). Keeping validation under 1ms ensures imperceptible latency.

## Measurement Criteria

- Validation execution time measured from mode change command to validation result
- Target: < 1ms (mean), < 2ms (99th percentile)
- Measurement method: Execution time profiling using system timer
- Test conditions: All validation paths exercised (pass, fail for each sensor)

## Acceptance Criteria

1. Validation timing profiling infrastructure:
   - Timestamp before validation start
   - Timestamp after validation complete
   - Calculate duration = end - start
   - Log validation timing for analysis
2. Validation optimizations if needed:
   - Inline critical path functions
   - Cache sensor health status (refresh at 10 Hz)
   - Minimize branching in validation logic
   - Optimize capability query overhead
3. Performance regression testing:
   - Automated tests measure validation timing
   - CI fails if validation exceeds 1ms
   - Regular profiling on target hardware (RP2350)

## Success Metrics

- Mean validation time: < 1ms
- 99th percentile: < 2ms
- No operator-reported delays during mode changes
- Validation overhead unnoticeable in operation

## Verification Methods

- Micro-benchmarks: Measure validation function execution time
- HITL testing: Profile validation timing on hardware
- Statistical analysis: Collect timing data over 1000+ mode changes
- Stress testing: Validation timing under sensor failure conditions

## ArduPilot Comparison

ArduPilot Rover's mode entry validation completes in approximately:

- Position check: < 10µs
- Velocity check: < 10µs
- EKF filter status query: < 50µs
- Total validation: < 100µs (well under 1ms target)

## Notes

- Target hardware: RP2350 @ 150 MHz
- Budget: < 150,000 CPU cycles @ 150 MHz = 1ms
- Validation is synchronous (blocks mode change until complete)
- Phase 1: Basic timing measurement; Phase 2: Optimization if needed

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
