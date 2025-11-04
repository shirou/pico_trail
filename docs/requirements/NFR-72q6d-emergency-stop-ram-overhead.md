# NFR-72q6d Emergency Stop RAM Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-cxts2-controlled-emergency-stop](FR-cxts2-controlled-emergency-stop.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Emergency stop system shall add no more than 50 bytes RAM overhead on RP2040/RP2350 platforms, maintaining memory budget for other subsystems.

## Rationale

RP2040/RP2350 have limited RAM (264 KB). Emergency stop system must be lightweight to preserve memory for vehicle control, communication, and navigation subsystems.

## Acceptance Criteria

- [ ] Total RAM usage < 50 bytes (measured via runtime profiling)
- [ ] Includes stop state, tier, trigger, and timing variables
- [ ] No dynamic allocation during stop execution
- [ ] Memory usage validated via size_of() checks and profiling

## Technical Details (if applicable)

**Memory Budget**: < 50 bytes RAM

**Components**:

- Stop state enum: 1 byte
- Stop tier enum: 1 byte
- Trigger source enum: 1 byte
- Timestamps: 12 bytes (3 × u32)
- P gain parameter: 4 bytes (f32)
- Velocity cache: 4 bytes (f32)

**Total**: \~23 bytes (well under 50 byte target)

## External References

- Analysis: [AN-4ez27-emergency-stop](../analysis/AN-4ez27-emergency-stop.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
