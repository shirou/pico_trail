# FR-ap18p EKF Health Validation for Autonomous Mode Entry

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P1
- Category: Mode Management / Safety

## Links

- Parent Analysis: [AN-5aniu-mode-entry-validation](../analysis/AN-5aniu-mode-entry-validation.md)
- Prerequisite Requirements:
  - [FR-a9rc3-mode-entry-sensor-validation](FR-a9rc3-mode-entry-sensor-validation.md)
  - [FR-eyuh8-ahrs-attitude-estimation](FR-eyuh8-ahrs-attitude-estimation.md)
- Dependent Requirements: N/A – No dependent requirements
- Related ADRs: TBD
- Related Tasks: TBD

## Requirement Statement

The system shall check EKF health before allowing autonomous mode entry by querying EKF/AHRS filter status, validating position estimate quality, denying autonomous modes if EKF unhealthy, and returning "Poor navigation quality" error.

## Rationale

Autonomous modes (Auto, Loiter, RTL) require high-quality navigation estimates from the Extended Kalman Filter. Operating autonomous modes with poor EKF health leads to unstable navigation, position drift, and mission failures. EKF health checking prevents reliance on degraded navigation estimates.

## Acceptance Criteria

1. EKF health check function queries navigation filter status:
   - Check position estimate variance within acceptable bounds
   - Check velocity estimate variance within acceptable bounds
   - Check innovation consistency (measurement vs prediction)
   - Verify filter convergence state
2. Autonomous mode validation includes EKF health check:
   - If `is_autopilot_mode()` returns true, call `ekf_healthy()`
   - If `ekf_healthy()` returns false, deny mode entry
   - Return error: `DeniedPoorEKF` ("Cannot enter mode: Poor navigation quality")
3. EKF health thresholds configurable (Phase 2):
   - Position variance threshold: < 5m
   - Velocity variance threshold: < 2m/s
   - Innovation threshold: < 3 sigma
4. EKF health logged when denying mode entry

## Success Metrics

- Autonomous modes denied when EKF unhealthy
- EKF health validation prevents navigation failures
- Operators notified of poor navigation quality
- EKF health status visible in logs

## Dependencies

- EKF/AHRS system implementation (NFR-eyuh8)
- Filter status query interface
- Mode capability queries (FR-sk7ty)
- Navigation quality monitoring

## ArduPilot References

This requirement is based on ArduPilot's EKF health checking:

**File**: `Rover/mode.cpp` (lines 21-40)

```cpp
bool Mode::enter()
{
    nav_filter_status filt_status;
    rover.ahrs.get_filter_status(filt_status);

    const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;

    if (requires_position() && !position_ok) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires position");
        return false;
    }

    return _enter();
}
```

**ArduPilot EKF Health Checks**:

- `ekf_position_ok()`: Checks position estimate variance and innovation
- `rover.failsafe.ekf`: EKF failsafe flag set when filter unhealthy

**Related ArduPilot Parameters**:

- **FS_EKF_ACTION** (u8) - Action when EKF fails during flight
  - 0 = Disabled (no action)
  - 1 = Hold (stop vehicle)
  - 2 = Disarm
  - Used indirectly by mode validation to check EKF failsafe state

- **FS_EKF_THRESH** (float) - EKF failsafe variance threshold (meters)
  - Default: 0.8m
  - Triggers EKF failsafe when position variance exceeds threshold

**Note**: For Phase 1, implement basic EKF health check (stub returning true). Phase 2 integrates full AHRS/EKF health validation with configurable thresholds.

## Verification Methods

- Unit tests: Mock EKF health status, verify validation enforces requirements
- Integration tests: Test mode entry with varying EKF health states
- Simulation testing: Induce EKF degradation, verify mode transitions denied
- HITL testing: Verify EKF health checking on hardware with real navigation

## Notes

- Phase 1: Stub implementation returns `ekf_healthy() = true`
- Phase 2: Full EKF health validation with variance/innovation checks
- EKF health check should be fast (< 50µs) to avoid validation delays
- Consider hysteresis to prevent mode oscillation during transient EKF issues

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
