# NFR-zxxit All Mode Transition Attempts Logged for Audit Trail

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P1
- Category: Reliability / Auditability

## Links

- Related Analyses:
  - [AN-5aniu-mode-entry-validation](../analysis/AN-5aniu-mode-entry-validation.md)
- Prerequisite Requirements:
  - [FR-a9rc3-mode-entry-sensor-validation](FR-a9rc3-mode-entry-sensor-validation.md)
  - [FR-bemq9-validation-failure-reporting](FR-bemq9-validation-failure-reporting.md)
- Dependent Requirements:
  - [NFR-v1gvk-lifecycle-transition-logging](NFR-v1gvk-lifecycle-transition-logging.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

All mode transition attempts shall be logged regardless of outcome to support post-flight analysis and debugging. Log entries must include timestamp, old mode, new mode, outcome (success/denied), and validation result.

## Rationale

Comprehensive logging enables post-flight analysis of mode changes, troubleshooting validation issues, and safety investigations. Logging both successful and denied transitions provides complete audit trail of vehicle behavior.

## Measurement Criteria

- Logging coverage: 100% of mode transition attempts logged
- Log completeness: All required fields present in every log entry
- Log format consistency: Standard format across all transitions
- Target: Zero missing mode transitions in log analysis

## Acceptance Criteria

1. Mode transition logging format:
   - Successful transitions: `"MODE_CHANGE,{timestamp},{old_mode},{new_mode},SUCCESS"`
   - Denied transitions: `"MODE_CHANGE_DENIED,{timestamp},{attempted_mode},{reason}"`
   - Include current armed state: `"MODE_CHANGE,{timestamp},{old},{new},SUCCESS,armed={bool}"`
2. Logging trigger points:
   - Log before validation (attempt initiated)
   - Log after validation (success or denied)
   - Log fallback mode selections (sensor-triggered)
3. Log entry completeness:
   - Timestamp (milliseconds since boot)
   - Old mode name (for successful changes)
   - New/attempted mode name
   - Outcome (SUCCESS, DENIED, FALLBACK)
   - Validation result (for denials)
   - Armed state
4. Log analysis tools:
   - Log parser extracts mode transitions
   - Mode timeline visualization
   - Validation failure statistics

## Success Metrics

- 100% of mode transitions logged (verified via testing)
- Log format parseable by automated tools
- Post-flight analysis can reconstruct complete mode history
- No gaps in mode transition timeline

## Verification Methods

- Unit tests: Verify logging called for all transition outcomes
- Integration tests: Parse logs, verify all transitions present
- HITL testing: Review logs for completeness
- Log analysis: Statistical validation of logging coverage

## ArduPilot Comparison

ArduPilot logs mode changes via dataflash logging:

- MODE log message: Includes mode number, reason, timestamp
- Logged for all mode changes (success and failure)
- Supports post-flight analysis and troubleshooting

## Notes

- Logging must be non-blocking (don't delay mode changes)
- Consider log rate limiting if mode oscillation occurs
- Log storage management: Prevent log overflow on long flights
- Phase 1: Basic logging; Phase 2: Add structured logging with levels

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
