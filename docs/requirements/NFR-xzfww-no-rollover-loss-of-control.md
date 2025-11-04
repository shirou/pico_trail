# NFR-xzfww No Vehicle Rollover or Loss of Control During Stop

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-cxts2-controlled-emergency-stop](FR-cxts2-controlled-emergency-stop.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Emergency stop shall not cause vehicle rollover or loss of heading control on typical surfaces (pavement, grass, gravel), maintaining vehicle stability and predictability during deceleration.

## Rationale

Safety requirement: vehicle must remain upright and controllable during emergency stop. Rollover or loss of control defeats purpose of safety mechanism.

## Acceptance Criteria

- [ ] No rollover observed during field testing at velocities up to 2 m/s
- [ ] Heading deviation < 15 degrees during controlled stop
- [ ] Test on multiple surfaces: pavement, grass, gravel
- [ ] Test with various vehicle masses and centers of gravity
- [ ] Document P gain tuning procedure per vehicle type

## Technical Details (if applicable)

**Test Conditions**:

- Velocities: 0.5, 1.0, 1.5, 2.0 m/s
- Surfaces: Dry pavement, wet pavement, grass, gravel
- Vehicle configurations: Various masses, COG heights

**Acceptance Criteria**:

- No rollover in any test
- Heading deviation < 15° (maintains general direction)

## External References

- Analysis: [AN-4ez27-emergency-stop](../analysis/AN-4ez27-emergency-stop.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
