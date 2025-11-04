# FR-dch39 Automatic Fallback Mode Selection When Sensor Requirements Lost

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P0
- Category: Mode Management / Safety / Failsafe

## Links

- Parent Analysis: [AN-5aniu-mode-entry-validation](../analysis/AN-5aniu-mode-entry-validation.md)
- Related Analysis: [AN-dgpck-armed-state-monitoring](../analysis/AN-dgpck-armed-state-monitoring.md)
- Prerequisite Requirements:
  - [FR-7e0cr-mode-lifecycle-management](FR-7e0cr-mode-lifecycle-management.md)
- Related Requirements:
  - [FR-a9rc3-mode-entry-sensor-validation](FR-a9rc3-mode-entry-sensor-validation.md)
  - [FR-sk7ty-mode-capability-queries](FR-sk7ty-mode-capability-queries.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  - [FR-xex86-...](FR-xex86-failsafe-capability-based-selection.md)
  - [NFR-p44ci-...](NFR-p44ci-fallback-mode-reliability.md)

## Requirement Statement

The system shall automatically select fallback mode when sensor requirements lost by monitoring sensor health during flight (via AN-dgpck monitoring), detecting when current mode requirements no longer met, selecting fallback mode hierarchy (Stabilize → Manual), and logging fallback mode transition with reason.

## Rationale

Graceful degradation preserves vehicle control when sensors fail during flight. If GPS is lost while in Auto mode, remaining in Auto would cause mission failure or loss of control. Automatic fallback to modes requiring fewer sensors (Stabilize, then Manual) maintains operator control during sensor degradation.

## Acceptance Criteria

1. Sensor monitoring detects requirement violations:
   - Continuous monitoring of GPS fix status
   - Continuous monitoring of position/velocity estimate availability
   - Continuous monitoring of IMU health
   - Detect when current mode's `requires_position()` / `requires_velocity()` no longer satisfied
2. Fallback mode selection algorithm:
   - Try modes in priority order: Current → Stabilize → Manual
   - For each candidate mode, check `validate_mode_entry()`
   - Select first mode with satisfied requirements
   - Manual mode always succeeds (requires no sensors)
3. Fallback transition executed automatically:
   - Call `set_mode(fallback_mode, ModeReason::Failsafe)`
   - Log transition with reason (e.g., "GPS lost", "Position estimate lost")
   - Notify GCS via STATUSTEXT: "Failsafe: Mode changed to STABILIZE (GPS lost)"
4. Fallback transitions logged with full details:
   - Timestamp, old mode, new mode, sensor lost, reason

## Success Metrics

- Sensor loss triggers fallback within 200ms
- Fallback mode always successfully entered (100% reliability)
- No loss of vehicle control during sensor failures
- All fallback transitions logged for post-flight analysis

## Dependencies

- Armed state monitoring system (AN-dgpck)
- Sensor health monitoring
- Mode validation framework (FR-a9rc3)
- Mode capability queries (FR-sk7ty)

## ArduPilot References

This requirement is based on ArduPilot's failsafe mode selection:

**File**: `Rover/failsafe.cpp` (lines 90-120)

```cpp
void Rover::select_failsafe_mode()
{
    Mode *fallback_modes[] = {
        &mode_rtl,        // RTL requires position
        &mode_hold,       // Hold requires nothing
        &mode_manual,     // Manual requires nothing
    };

    for (Mode *mode : fallback_modes) {
        if (mode->requires_position() && !ekf_position_ok()) {
            continue;  // Cannot use this mode
        }
        if (mode->requires_velocity() && !ekf_velocity_ok()) {
            continue;
        }
        set_mode(*mode, ModeReason::FAILSAFE);
        return;
    }

    // Force Manual as last resort
    set_mode(mode_manual, ModeReason::FAILSAFE);
}
```

**Related ArduPilot Parameters**:

- **FS_ACTION** (u8) - Action when failsafe triggers
  - 0 = None (no action)
  - 1 = Hold (stop vehicle)
  - 2 = RTL (return to launch)
  - 3 = SmartRTL (intelligent return)
  - 4 = SmartRTL + Hold (SmartRTL with Hold fallback)
  - 5 = Terminate (emergency motor stop)
  - Determines which mode to select during failsafe

- **FS_EKF_ACTION** (u8) - Action when EKF fails
  - 0 = Disabled
  - 1 = Hold
  - 2 = Disarm
  - Specific action for navigation failure

**Note**: For Phase 1, implement basic fallback hierarchy (Stabilize → Manual). Phase 2 can add RTL and configurable fallback modes via FS_ACTION parameter.

## Verification Methods

- Unit tests: Mock sensor loss scenarios, verify fallback selection
- Integration tests: Test each sensor failure type (GPS, position, IMU)
- HITL testing: Induce sensor failures, verify fallback transitions
- Log analysis: Confirm all sensor-triggered fallbacks logged

## Notes

- Fallback selection must be reliable (cannot fail, Manual always available)
- Add hysteresis to prevent mode oscillation during transient sensor glitches
- Fallback hierarchy should prioritize preserving operator control
- Consider rate limiting fallback attempts (max 1 per second)

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
