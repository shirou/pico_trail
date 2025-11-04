# FR-upqdp Disarmed Mode Change Validation Exception for Pre-Flight Configuration

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P0
- Category: Mode Management / Usability

## Links

- Parent Analysis: [AN-5aniu-mode-entry-validation](../analysis/AN-5aniu-mode-entry-validation.md)
- Related Requirements: [FR-a9rc3-mode-entry-sensor-validation](FR-a9rc3-mode-entry-sensor-validation.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

The system shall allow all mode changes when disarmed (pre-flight configuration exception) by skipping sensor validation checks, logging mode changes with "disarmed" flag, and applying normal validation only when armed.

## Rationale

Pre-flight configuration requires ability to select modes without full sensor suite operational. Operators need to configure desired flight mode (e.g., Auto for mission) before GPS acquires fix or before arming. This matches ArduPilot's pattern of relaxed validation when disarmed.

## Acceptance Criteria

1. Mode change function checks armed state before validation:
   - If `is_armed() == false`, skip all sensor validation checks
   - If `is_armed() == true`, perform full validation (FR-a9rc3)
2. Disarmed mode changes logged with flag indicating validation skipped:
   - Log entry includes: timestamp, old mode, new mode, armed=false
   - Log message: "Mode change to {:?} (disarmed, validation skipped)"
3. Validation applied immediately upon arming:
   - At arm time, verify current mode's requirements are met
   - If current mode requirements not met, deny arming or force mode change
4. Documentation explains pre-flight configuration workflow

## Success Metrics

- All mode changes allowed when disarmed (100% success rate)
- Sensor validation enforced when armed (0% unsafe transitions)
- Operators can configure flight mode before sensor initialization
- Log distinguishes disarmed vs armed mode changes

## Dependencies

- Armed state tracking in `SystemState`
- Mode validation function (FR-a9rc3)
- Pre-arm checks (AN-r2fps) verify mode compatibility

## ArduPilot References

This requirement is based on ArduPilot Rover's disarmed exception pattern:

**File**: `Rover/mode.cpp` (lines 21-54)

```cpp
bool Mode::enter()
{
    // Get filter status for validation
    nav_filter_status filt_status;
    rover.ahrs.get_filter_status(filt_status);

    const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;

    if (requires_position() && !position_ok) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires position");
        return false;
    }

    // Note: ArduPilot allows mode changes when disarmed via armed state check
    // in set_mode() before calling enter()

    return _enter();
}
```

**ArduPilot Pattern**: Mode validation checks are bypassed when disarmed, allowing operators to select modes for pre-flight configuration.

**Related ArduPilot Parameters**: None - Disarmed exception is architectural, not configurable

## Verification Methods

- Unit tests: Verify validation skipped when disarmed, enforced when armed
- Integration tests: Test mode change sequences (disarmed → mode change → arm → validation)
- HITL testing: Verify pre-flight configuration workflow on hardware
- User testing: Operators confirm ability to configure modes before sensor init

## Notes

- Disarmed exception applies to mode validation only, not pre-arm checks
- Pre-arm checks (ARMING_CHECK) still verify current mode allows arming
- Disarmed mode changes should still log for debugging/analysis
- Safety rationale: When disarmed, actuators are disabled, so mode requirements less critical

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
