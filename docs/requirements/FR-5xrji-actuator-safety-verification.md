# FR-5xrji Actuator Safety Verification

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-081u0-post-disarm-cleanup](../analysis/AN-081u0-post-disarm-cleanup.md)
  - [AN-081u0-post-disarm-cleanup](../analysis/AN-081u0-post-disarm-cleanup.md)
- Prerequisite Requirements:
  - [FR-0xjwy-post-disarm-event-recording](FR-0xjwy-post-disarm-event-recording.md)
- Dependent Requirements: N/A – No dependent requirements
- Related Tasks: N/A – Tasks will be created after ADRs

## Requirement Statement

The system shall verify that actuators return to safe neutral state after disarming by reading back PWM output values and confirming throttle is at minimum (1000µs or off) and steering is at center (1500µs) within 50ms.

## Rationale

Actuator safety verification is a critical safety check that ensures the vehicle is safe for operator handling after disarm. Without verification:

- Motors may remain in an active state, posing injury risk to operators
- Actuators may have residual commands that could activate unexpectedly
- No confirmation that the disarm command was successfully executed by hardware
- Operators cannot safely approach or handle the vehicle

Verification provides confidence that the vehicle is truly in a safe state before personnel interaction, which is essential for operational safety and regulatory compliance.

## User Story

As a maintenance operator or pilot, I want the system to verify actuators are in safe neutral state after disarm, so that I can safely handle the vehicle without risk of unexpected motor activation.

## Acceptance Criteria

- [ ] Read back throttle PWM output after disarm command
- [ ] Verify throttle PWM is 1000µs (minimum) or off (no pulse)
- [ ] Read back steering PWM output after disarm command
- [ ] Verify steering PWM is 1500µs (center/neutral position)
- [ ] Verification completes within 50ms of disarm command
- [ ] Log warning if verification fails (PWM values incorrect)
- [ ] Verification timeout does not prevent disarm from completing
- [ ] Verification failure logs error but allows disarm to proceed (safety-first)

## Technical Details

### Functional Requirement Details

**Input:**

- Actuator PWM output interface (read capability)
- Expected safe values: throttle 1000µs, steering 1500µs

**Output:**

- Verification result (pass/fail)
- Warning log if verification fails
- PostDisarmState.actuators_safe flag set appropriately

**Behavior:**

1. After setting disarmed state, send neutral PWM commands to actuators
2. Wait for actuators to settle (typically 5-10ms)
3. Read back throttle PWM value from hardware
4. Compare throttle PWM to expected safe value (1000µs ± tolerance)
5. Read back steering PWM value from hardware
6. Compare steering PWM to expected safe value (1500µs ± tolerance)
7. If all values correct, set actuators_safe = true
8. If any value incorrect, log warning and set actuators_safe = false
9. Timeout after 50ms to prevent blocking disarm operation
10. Complete disarm regardless of verification result (log error if failed)

**Error Conditions:**

- Hardware read failure: Log error, assume unsafe, warn operator
- PWM values out of range: Log warning with actual values, complete disarm
- Verification timeout (>50ms): Log timeout warning, complete disarm
- No actuator feedback available: Skip verification, log "unverified" status

**Tolerance:**

- Throttle: 1000µs ± 20µs (accounting for hardware precision)
- Steering: 1500µs ± 20µs (accounting for hardware precision)

## Platform Considerations

### Unix

N/A – Platform agnostic

### Windows

N/A – Platform agnostic

### Cross-Platform

N/A – Platform agnostic

## Risks & Mitigation

| Risk                                                   | Impact   | Likelihood | Mitigation                                                            | Validation                                 |
| ------------------------------------------------------ | -------- | ---------- | --------------------------------------------------------------------- | ------------------------------------------ |
| Hardware read capability not available                 | High     | Medium     | Graceful degradation: skip verification, log unverified status        | Test on hardware without PWM readback      |
| Verification blocks disarm (operator stuck armed)      | Critical | Low        | Timeout-based (50ms), disarm proceeds even if verification fails      | Test with slow/unresponsive hardware       |
| False negative (actuators safe but verification fails) | Medium   | Low        | Allow configurable tolerance (±20µs), log actual PWM values for debug | Test with various hardware/actuator types  |
| PWM settle time longer than expected                   | Medium   | Medium     | Configurable settle delay, retry logic if first read shows transient  | Profile PWM settle time on target hardware |

## Implementation Notes

**Preferred approaches:**

- Use ArduPilot's `MOT_SAFE_DISARM` parameter approach (PWM output behavior while disarmed)
  - 0 = Output trim values (default)
  - 1 = No PWM pulses (safer, ensures neutral)
- Implement timeout-based verification to prevent blocking
- Log actual PWM values on verification failure for debugging
- Use hardware PWM readback if available, otherwise skip verification

**Known pitfalls:**

- Do not block disarm() waiting indefinitely for actuator settle
- Avoid failing disarm operation if verification cannot be performed
- Do not assume all hardware supports PWM readback
- Account for hardware-specific PWM precision and noise

**Related code areas:**

- `src/communication/mavlink/state.rs` - SystemState::disarm()
- `src/vehicle/arming/cleanup.rs` - Post-disarm cleanup logic
- `src/platform/actuators/` - Actuator abstraction layer (PWM interface)

**Suggested libraries:**

- rp2040-hal or embassy-rp for RP2040/RP2350 PWM control
- embedded-hal traits for hardware abstraction

## External References

- [ArduPilot MOT_SAFE_DISARM Parameter](https://ardupilot.org/rover/docs/parameters.html#mot-safe-disarm) - PWM output behavior while disarmed (0=trim, 1=no pulses)
- [ArduPilot AP_Arming Disarm Sequence](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Arming/AP_Arming.cpp) - Reference implementation

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
