# NFR-52km9 Validation Error Messages Human-Readable and Specific

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P1
- Category: Usability

## Links

- Parent Analysis: [AN-5aniu-mode-entry-validation](../analysis/AN-5aniu-mode-entry-validation.md)
- Related Requirements: [FR-bemq9-validation-failure-reporting](FR-bemq9-validation-failure-reporting.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Validation failure messages shall be human-readable and specific to enable operators to quickly understand and resolve mode change failures. Error messages must specify sensor/estimate missing, not provide generic failure messages.

## Rationale

Operators need clear diagnosis of mode change failure to take corrective action. Generic messages like "Validation failed" or error codes don't provide actionable information. Specific messages like "Cannot enter AUTO: GPS not available" enable rapid troubleshooting.

## Measurement Criteria

- Error message clarity assessed through user testing
- Target: 100% of operators can identify missing sensor from error message
- Error message specificity: Each failure type has unique message
- Message format: "{Mode name}: {Specific reason}"

## Acceptance Criteria

1. Error message format standards:
   - Include mode name: "Cannot enter AUTO"
   - Include specific sensor/requirement: "GPS not available"
   - Avoid error codes or technical jargon
   - Keep messages concise (< 50 characters for GCS display)
2. Error message coverage:
   - Position estimate missing: Specific message
   - Velocity estimate missing: Specific message
   - GPS not available: Specific message
   - IMU not available: Specific message
   - EKF unhealthy: Specific message
   - Compass not available: Specific message
3. User testing validation:
   - Present error messages to operators
   - Operators identify missing sensor/requirement
   - Target: 100% correct identification
4. Documentation of error messages:
   - User manual lists all validation errors
   - Remediation steps for each error type

## Success Metrics

- 100% of validation errors include specific reason
- User testing: 100% operator identification of missing sensor
- No user reports of unclear error messages
- Error messages comply with MAVLink STATUSTEXT character limit

## Verification Methods

- User testing: Operators interpret error messages
- Code review: Verify all validation paths return specific errors
- Documentation review: Error messages documented in user manual
- GCS testing: Verify messages display correctly

## ArduPilot Comparison

ArduPilot uses clear, specific error messages:

- "Mode requires position" (not "Validation failed")
- "Mode requires velocity" (not "Sensor error")
- Pattern: Always specify what is missing

## Notes

- Error messages should be actionable (tell operator what's wrong)
- Consider adding remediation hints in Phase 2
- Translation support may be needed for international operators (Phase 2)
- Error messages logged for post-flight analysis

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
