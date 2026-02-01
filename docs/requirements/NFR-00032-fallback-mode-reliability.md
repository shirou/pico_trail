# NFR-00032 Fallback Mode Selection Guaranteed Success for Vehicle Safety

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P0
- Category: Safety / Reliability

## Links

- Parent Analysis: [AN-00013-mode-entry-validation](../analysis/AN-00013-mode-entry-validation.md)
- Related Requirements: [FR-00039-fallback-mode-selection](FR-00039-fallback-mode-selection.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Fallback mode selection shall always succeed to ensure vehicle has control mode even with sensor failures. Manual mode requires no sensors and is always available as last fallback, preventing loss of control during degraded sensor conditions.

## Rationale

Vehicle safety depends on having a functional control mode at all times. If fallback mode selection fails, vehicle could become uncontrollable during sensor failures. Manual mode serves as guaranteed fallback because it requires no sensors (direct RC control).

## Measurement Criteria

- Fallback success rate: 100% (measured across all failure scenarios)
- Fallback selection time: < 200ms (measured from sensor loss to mode entry)
- Test coverage: All sensor failure combinations exercised
- Target: Zero fallback selection failures in 10,000+ tests

## Acceptance Criteria

1. Fallback hierarchy design:
   - Each level requires fewer sensors than previous
   - Final fallback (Manual) requires no sensors
   - Hierarchy example: Current → Stabilize → Manual
   - Manual mode fallback cannot be skipped
2. Fallback selection algorithm guarantees:
   - Try modes in priority order
   - If all modes fail validation, force Manual mode
   - Manual mode entry cannot fail (no prerequisites)
   - No panic/crash if fallback selection called
3. Fallback reliability testing:
   - Test all single-sensor failures
   - Test all multi-sensor failure combinations
   - Test fallback during high-load conditions
   - Verify 100% success rate across all scenarios
4. Fallback failure handling:
   - If intermediate fallback fails, try next in hierarchy
   - Log fallback attempts and outcomes
   - Always terminate with successful mode entry

## Success Metrics

- Fallback success rate: 100% across all test scenarios
- Zero loss-of-control events due to fallback failure
- Manual mode entry always succeeds (verified in testing)
- Fallback selection completes within 200ms

## Verification Methods

- Unit tests: Mock all sensor failure combinations
- Fault injection testing: Induce sensor failures, verify fallback
- HITL testing: Real sensor failures on hardware
- Stress testing: Fallback during system overload

## ArduPilot Comparison

ArduPilot guarantees fallback success through:

- Hierarchical mode fallback (RTL → Hold → Manual)
- Manual mode as ultimate fallback (requires nothing)
- Forced mode selection as last resort
- No failure path in fallback selection code

## Notes

- Fallback reliability is safety-critical requirement
- Manual mode must be bulletproof (minimal dependencies)
- Consider adding telemetry alert if forced to Manual fallback
- Phase 1: Basic hierarchy (Stabilize → Manual); Phase 2: Add RTL
