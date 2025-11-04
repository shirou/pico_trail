# FR-sk7ty Mode Capability Query System for Mode-Specific Requirements

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P0
- Category: Mode Management / Validation

## Links

- Parent Analysis: [AN-5aniu-mode-entry-validation](../analysis/AN-5aniu-mode-entry-validation.md)
- Related Analysis: [AN-g5w99-mode-capability-system](../analysis/AN-g5w99-mode-capability-system.md)
- Related Requirements: TBD
- Dependent Requirements:
  - [FR-a9rc3-mode-entry-sensor-validation](FR-a9rc3-mode-entry-sensor-validation.md)
  - [FR-dch39-fallback-mode-selection](FR-dch39-fallback-mode-selection.md)
  - [FR-7e0cr-mode-lifecycle-management](FR-7e0cr-mode-lifecycle-management.md)
  - [FR-qj0d1-mode-capability-declaration](FR-qj0d1-mode-capability-declaration.md)
  - [NFR-cy29p-capability-query-zero-allocation](NFR-cy29p-capability-query-zero-allocation.md)
  - [NFR-mhhin-capability-system-extensibility](NFR-mhhin-capability-system-extensibility.md)
  - [NFR-sx06h-validation-memory-overhead](NFR-sx06h-validation-memory-overhead.md)
  - [NFR-sz1ul-capability-system-memory-overhead](NFR-sz1ul-capability-system-memory-overhead.md)
- Related ADRs: TBD
- Related Tasks: TBD

## Requirement Statement

Each flight mode shall declare sensor and estimate requirements via capability queries (requires_position(), requires_velocity()) to enable validation framework to check prerequisites before mode entry.

## Rationale

Declarative capability queries enable the validation framework to automatically enforce mode-specific sensor requirements without hardcoding checks for each mode. This follows ArduPilot's proven pattern where each mode implements virtual methods declaring its needs, allowing validation logic to remain generic and extensible.

## Acceptance Criteria

1. Implement ModeCapability trait with the following query methods:
   - `requires_position() -> bool`: Returns true if mode needs position estimate
   - `requires_velocity() -> bool`: Returns true if mode needs velocity estimate
   - `is_autopilot_mode() -> bool`: Returns true if mode is autonomous
   - `allows_arming() -> bool`: Returns true if arming allowed in mode
   - `allows_arming_from_transmitter() -> bool`: Returns true if RC arming allowed
   - `has_manual_input() -> bool`: Returns true if mode uses RC input
2. Each mode (Manual, Stabilize, Loiter, Auto, RTL) implements trait with correct requirements
3. Validation framework uses capability queries to check prerequisites
4. Query results documented in mode capability matrix

## Success Metrics

- All modes implement ModeCapability trait
- Validation framework calls capability queries before mode entry
- Unit tests verify each mode returns correct capability values
- Mode capability matrix documented in code comments or docs

## Dependencies

- Mode enum defined in `src/communication/mavlink/state.rs`
- Mode lifecycle system (FR-ojilc)
- Mode capability system (AN-g5w99)

## ArduPilot References

This requirement is based on ArduPilot Rover's mode capability pattern:

**File**: `Rover/mode.h` (lines 50-70)

```cpp
class Mode {
public:
    virtual bool requires_position() const { return true; }
    virtual bool requires_velocity() const { return true; }
    virtual bool is_autopilot_mode() const { return false; }
    virtual bool allows_arming() const { return true; }
    virtual bool allows_arming_from_transmitter() const;
    virtual bool has_manual_input() const { return false; }
};
```

**Mode-Specific Examples**:

- **Manual**: requires_position()=false, requires_velocity()=false, has_manual_input()=true
- **Auto**: requires_position()=true, requires_velocity()=true, is_autopilot_mode()=true
- **RTL**: requires_position()=true, requires_velocity()=true, is_autopilot_mode()=true

**Related ArduPilot Parameters**: None - Capability queries are hardcoded per mode class

## Verification Methods

- Unit tests: Each mode's capability queries return expected values
- Integration tests: Validation framework calls capability queries and enforces requirements
- Code review: Safety team verifies capability declarations match mode behavior
- Documentation review: Mode capability matrix complete and accurate

## Notes

- Capability queries should have zero runtime allocation overhead (inline or trait methods)
- Default implementations in trait reduce boilerplate for common patterns
- Extensible design allows adding new capability queries without modifying all modes
- See AN-g5w99 for full capability system analysis including additional query types

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
