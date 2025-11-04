# FR-9qug9 Mode Validation Enforces Sensor Requirements Using Capability Queries

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P0
- Category: Mode Management / Safety

## Links

- Parent Analysis: [AN-g5w99-mode-capability-system](../analysis/AN-g5w99-mode-capability-system.md)
- Related Requirements:
  - [FR-a9rc3-mode-entry-sensor-validation](FR-a9rc3-mode-entry-sensor-validation.md)
  - [FR-qj0d1-mode-capability-declaration](FR-qj0d1-mode-capability-declaration.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Mode validation shall enforce sensor requirements using capability queries by calling capability queries to check requirements, denying mode entry if `requires_position()` but no position available, denying if `requires_gps()` but no GPS fix, and returning specific error for each missing requirement.

## Rationale

Automated enforcement based on declared capabilities ensures validation logic stays synchronized with mode requirements. Adding new modes doesn't require modifying validation framework. Capability-based validation is self-documenting and verifiable.

## Acceptance Criteria

1. Validation algorithm uses capability queries:
   - If `mode.requires_position()` and `!has_position_estimate()` → Deny (DeniedNoPosition)
   - If `mode.requires_velocity()` and `!has_velocity_estimate()` → Deny (DeniedNoVelocity)
   - If `mode.requires_gps()` and `!has_gps_fix()` → Deny (DeniedNoGPS)
   - If `mode.requires_imu()` and `!has_imu()` → Deny (DeniedNoIMU)
   - If `mode.requires_compass()` and `!has_compass()` → Deny (DeniedNoCompass)
2. Validation framework is generic:
   - No mode-specific conditionals in validation logic
   - Works for all modes via capability queries
   - Adding new mode doesn't require validation changes
3. Error messages include mode and sensor:
   - "Cannot enter AUTO: GPS not available"
   - "Cannot enter LOITER: No position estimate"
4. Validation called during mode entry:
   - Integrated with enter() method
   - See FR-a9rc3 for validation enforcement

## Success Metrics

- Validation uses capability queries (no hardcoded mode checks)
- All sensor requirements enforced via capabilities
- Adding new mode requires no validation code changes
- Error messages specify mode and missing sensor

## Dependencies

- Mode capability queries (FR-qj0d1)
- Mode validation framework (FR-a9rc3)
- Sensor health monitoring

## ArduPilot References

**File**: `Rover/mode.cpp` (lines 21-54)

```cpp
bool Mode::enter()
{
    // Generic validation using capability queries
    if (requires_position() && !position_ok) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires position");
        return false;
    }

    if (requires_velocity() && !velocity_ok) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires velocity");
        return false;
    }

    return _enter();
}
```

**Pattern**: Validation logic queries capabilities, not hardcoded per mode.

**Related ArduPilot Parameters**: None - Validation is capability-driven

## Verification Methods

- Unit tests: Mock sensor availability, verify validation uses capabilities
- Integration tests: Test validation for each sensor failure type
- Code review: Verify no hardcoded mode checks in validation
- Extensibility test: Add new mode, verify validation works without changes

## Notes

- Capability-based validation is self-documenting
- Extensible for new sensor types
- Clear separation: modes declare, validation enforces

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
