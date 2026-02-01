# NFR-00019 Capability System Extensible for New Queries Without Framework Changes

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P1
- Category: Maintainability / Extensibility

## Links

- Parent Analysis: [AN-00012-mode-capability-system](../analysis/AN-00012-mode-capability-system.md)
- Related Requirements: [FR-00045-mode-capability-queries](FR-00045-mode-capability-queries.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Capability system shall be extensible for new queries without framework changes by allowing new capability query methods to be added to trait with default implementations, avoiding modification of validation/arming/failsafe frameworks.

## Rationale

Future sensor types (rangefinder, vision, lidar) may require new capability queries. Extensible design prevents framework churn when adding capabilities. Trait-based design with default implementations enables backward compatibility.

## Measurement Criteria

- Extensibility test: Add new capability query without modifying framework code
- Target: New query requires only trait method addition + per-mode implementations
- Framework stability: Validation/arming/failsafe unchanged when adding capability
- Measurement: Count lines of code changed to add new capability

## Acceptance Criteria

1. Extensibility mechanism:
   - Add new method to ModeCapability trait
   - Provide default implementation in trait
   - Override in modes needing different behavior
   - No changes to validation/arming/failsafe frameworks
2. Example: Add `requires_rangefinder()`:
   - Add trait method: `fn requires_rangefinder(&self) -> bool { false }`
   - Override in modes needing rangefinder
   - Validation calls `mode.requires_rangefinder()` if checking rangefinder
   - No framework restructuring needed
3. Backward compatibility:
   - Default implementations prevent breaking existing modes
   - Optional capabilities don't affect modes that don't need them
4. Documentation:
   - Guidelines for adding new capability queries
   - Examples of extending capability system

## Success Metrics

- New capability added without framework changes
- < 100 lines of code to add new capability (trait + mode impls)
- Existing modes unaffected by new capabilities
- Documentation clear for extension process

## Verification Methods

- Extensibility test: Add mock new capability, measure code changes
- Code review: Verify framework not modified for new capability
- Documentation review: Verify extension process documented
- Integration testing: Verify existing functionality unchanged

## ArduPilot Comparison

ArduPilot capability system is extensible:

- New virtual methods added to Mode base class
- Default implementations provided
- Modes override as needed
- Framework uses new capabilities automatically

## Notes

- Trait default implementations key to extensibility
- Consider using marker traits for optional capabilities (Phase 2)
- Extensibility enables future sensor support (vision, lidar, etc.)
