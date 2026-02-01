# NFR-00016 Capability Declarations Self-Documenting and Verifiable

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P1
- Category: Usability / Safety

## Links

- Parent Analysis: [AN-00012-mode-capability-system](../analysis/AN-00012-mode-capability-system.md)
- Related Requirements: [FR-00044-mode-capability-declaration](FR-00044-mode-capability-declaration.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Capability declarations shall be self-documenting and verifiable so safety reviewers can understand mode requirements. Capability queries must have clear names, documented meanings, and unit tests to verify correctness.

## Rationale

Safety reviewers need to understand what each mode requires without reading implementation code. Self-documenting capability names and comprehensive tests provide verifiable evidence that mode requirements are correctly declared.

## Measurement Criteria

- Documentation completeness: All capability queries documented
- Test coverage: Each mode's capabilities tested
- Clarity: Capability names self-explanatory
- Target: 100% capability queries documented and tested

## Acceptance Criteria

1. Capability query documentation:
   - Each trait method has doc comment explaining purpose
   - Examples of when query returns true/false
   - How validation/arming/failsafe uses each capability
2. Mode capability matrix:
   - Table documenting all modes vs all capabilities
   - Easy reference for reviewers
   - Included in architecture documentation
3. Unit test coverage:
   - Each mode's capability queries tested
   - Test verifies expected return values
   - Test coverage: 100% of capability queries
4. Naming clarity:
   - Capability names self-explanatory (e.g., `requires_position`, not `needs_sensor_1`)
   - Boolean queries follow convention: `requires_*`, `allows_*`, `is_*`, `has_*`

## Success Metrics

- 100% capability queries documented
- Mode capability matrix complete and accurate
- Unit test coverage: 100% of capabilities
- Safety review: Reviewers can understand requirements from docs alone

## Verification Methods

- Documentation review: Verify all queries documented
- Test coverage analysis: Verify all capabilities tested
- Safety review: Reviewers assess documentation clarity
- Code review: Verify naming conventions followed

## ArduPilot Comparison

ArduPilot capability system is self-documenting:

- Clear method names (`requires_position`, `allows_arming`)
- Inline documentation in mode.h
- Comments explain purpose of each capability

## Notes

- Documentation is safety-critical (enables review)
- Consider generating capability matrix from code (Phase 2)
- Unit tests serve as executable documentation
- Capability names should be intuitive to domain experts
