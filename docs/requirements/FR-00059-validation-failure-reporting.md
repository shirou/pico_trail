# FR-00059 Clear Validation Failure Reporting for Mode Change Denials

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P1
- Category: Mode Management / Usability

## Links

- Parent Analysis: [AN-00013-mode-entry-validation](../analysis/AN-00013-mode-entry-validation.md)
- Related Requirements:
  - [FR-00046-mode-entry-sensor-validation](FR-00046-mode-entry-sensor-validation.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  - [NFR-00055-...](NFR-00055-validation-error-message-usability.md)
  - [NFR-00054-...](NFR-00054-validation-attempt-logging.md)

## Requirement Statement

The system shall provide clear feedback when mode change denied by returning specific error message for each validation failure, sending STATUSTEXT to GCS with failure reason, logging validation failure with details, and including sensor name in error message (GPS, position, velocity, IMU).

## Rationale

Operators need to understand why mode change rejected to take corrective action (e.g., wait for GPS fix, check sensor health). Generic error messages like "Mode change failed" don't provide actionable information. Specific error messages enable rapid troubleshooting.

## Acceptance Criteria

1. Validation failure results have specific error messages:
   - `DeniedNoPosition`: "Cannot enter mode: No position estimate"
   - `DeniedNoVelocity`: "Cannot enter mode: No velocity estimate"
   - `DeniedNoGPS`: "Cannot enter mode: GPS not available"
   - `DeniedPoorEKF`: "Cannot enter mode: Poor navigation quality"
   - `DeniedNoIMU`: "Cannot enter mode: IMU not available"
   - `DeniedNoCompass`: "Cannot enter mode: Compass not available"
2. GCS notification via STATUSTEXT:
   - Send MAV_SEVERITY_WARNING with error message
   - Include mode name in message: "Cannot enter AUTO: GPS not available"
3. Validation failure logging:
   - Log entry includes: timestamp, attempted mode, denial reason, current sensor status
   - Log format: "MODE_CHANGE_DENIED,{timestamp},{mode},{reason}"
4. Error messages human-readable and actionable

## Success Metrics

- 100% of validation failures include specific reason
- Operators can identify missing sensor from error message
- Error messages tested for clarity in user testing
- GCS displays error messages within 100ms of denial

## Dependencies

- Mode validation framework (FR-00046)
- MAVLink STATUSTEXT message handling
- Logging system
- GCS communication

## ArduPilot References

This requirement is based on ArduPilot's validation error reporting:

**File**: `Rover/mode.cpp` (lines 36-48)

```cpp
bool Mode::enter()
{
    const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;

    if (requires_position() && !position_ok) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires position");
        return false;
    }

    if (requires_velocity() && !position_ok && !filt_status.flags.horiz_vel) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires velocity");
        return false;
    }

    return _enter();
}
```

**ArduPilot Pattern**: Send specific error message via `gcs().send_text()` immediately when validation fails.

**Related ArduPilot Parameters**: None - Error messaging is hardcoded

## Verification Methods

- Unit tests: Verify each validation failure returns correct error message
- Integration tests: Test GCS receives STATUSTEXT messages
- User testing: Operators confirm error messages are clear and actionable
- Log analysis: Verify all denials logged with specific reason

## Notes

- Error messages should be concise (< 50 characters) for GCS display
- Include mode name and sensor name for context
- Consider adding remediation hints in Phase 2 (e.g., "Wait for GPS fix")
- Log all validation attempts (success and failure) for debugging
