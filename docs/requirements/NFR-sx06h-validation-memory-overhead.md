# NFR-sx06h Mode Entry Validation Minimal RAM Overhead Under 10 Bytes

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P2
- Category: Resource Constraints

## Links

- Parent Analysis: [AN-5aniu-mode-entry-validation](../analysis/AN-5aniu-mode-entry-validation.md)
- Related Requirements:
  - [FR-sk7ty-mode-capability-queries](FR-sk7ty-mode-capability-queries.md)
  - [FR-a9rc3-mode-entry-sensor-validation](FR-a9rc3-mode-entry-sensor-validation.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  - [NFR-sz1ul-...](NFR-sz1ul-capability-system-memory-overhead.md)

## Requirement Statement

Mode entry validation shall add negligible RAM overhead (< 10 bytes) because capability queries are code-only (trait methods) with no runtime allocation, and validation logic operates on existing sensor state.

## Rationale

Limited RAM on RP2040/RP2350 (264 KB) requires efficient memory usage. Mode validation should not introduce significant memory overhead. Capability queries implemented as trait methods compile to code, not data, eliminating runtime allocation. Validation logic queries existing sensor state without duplication.

## Measurement Criteria

- RAM usage measured via linker map analysis and size profiling
- Target: < 10 bytes runtime overhead for validation system
- Measurement method: Compare binary size with/without validation
- Components measured:
  - ModeCapability trait vtable: \~8 bytes
  - Validation state: 0 bytes (operates on existing SystemState)
  - Error result enum: 1 byte (stack allocated, temporary)

## Acceptance Criteria

1. Memory usage analysis:
   - Profile RAM usage with validation enabled vs disabled
   - Identify any unexpected allocations
   - Document memory layout of validation components
2. Capability queries zero-allocation:
   - Trait methods inline or vtable dispatch only
   - No heap allocation during queries
   - No additional per-mode data structures
3. Validation logic stack-only:
   - Validation result enum stack-allocated (1 byte)
   - No dynamic allocation during validation
   - Temporary variables minimal and stack-allocated
4. Memory regression testing:
   - CI checks binary size for unexpected growth
   - Size profiling on target hardware

## Success Metrics

- Total validation overhead: < 10 bytes RAM
- No heap allocations during validation
- Binary size increase: < 2 KB code (for validation logic)
- Memory usage meets budget on RP2350

## Verification Methods

- Linker map analysis: Identify validation-related allocations
- Size profiling: cargo bloat analysis
- HITL testing: Verify memory usage on hardware
- Static analysis: Ensure no hidden allocations

## ArduPilot Comparison

ArduPilot's mode validation has minimal memory overhead:

- Mode capability queries: Virtual methods (vtable only)
- Validation logic: Stack-allocated locals
- No per-mode validation state required

## Notes

- Focus on keeping validation logic simple and allocation-free
- Capability queries should compile to inline or vtable calls
- Validation state temporary, not persistent
- Phase 1: Basic measurement; Phase 2: Optimization if needed

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
