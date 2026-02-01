# FR-00018 Armed Operation State Reset

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
  - [AN-00016-post-disarm-cleanup](../analysis/AN-00016-post-disarm-cleanup.md)
- Prerequisite Requirements:
  - [FR-00027-disarm-subsystem-notification](FR-00027-disarm-subsystem-notification.md)
- Dependent Requirements: N/A – No dependent requirements
- Related Tasks:
  - [T-00008-arming-system-implementation](../tasks/T-00008-arming-system-implementation/README.md)

## Requirement Statement

The system shall reset all armed-operation-specific state variables after disarming to ensure the vehicle starts with a clean state on the next arm cycle, preventing state accumulation and false alarms from previous armed operations.

## Rationale

State reset is essential for maintaining repeatable, predictable arm/disarm cycles. Without proper state reset:

- Failsafe trigger flags from previous armed operation persist, causing false alarms on next arm
- Monitoring baselines accumulate stale data, leading to incorrect health assessments
- Mode-specific state from previous flight affects next flight behavior
- Emergency stop or crash flags remain active, blocking subsequent arm attempts
- Accumulated state makes debugging difficult (cannot distinguish current vs. previous operation)

Proper state reset ensures each arm cycle starts from a known-good baseline, preventing cross-contamination between flights and enabling repeatable system behavior for testing and operations.

## User Story

As the vehicle system, I want all armed-operation state cleared after disarm, so that the next arm cycle starts with clean state and previous operation does not affect current behavior.

## Acceptance Criteria

- [ ] Clear all failsafe trigger flags (RC loss, battery low, GCS loss, etc.)
- [ ] Reset monitoring baselines (sensor health, signal strength, battery voltage)
- [ ] Clear mode-specific armed operation state
- [ ] Reset emergency stop state if triggered during previous armed operation
- [ ] Clear crash detection flags
- [ ] Reset armed duration counters and timestamps
- [ ] Vehicle starts clean on next arm (no residual state from previous operation)
- [ ] State reset completes within 1ms
- [ ] State reset failures are logged but do not prevent disarm completion

## Technical Details

### Functional Requirement Details

**Input:**

- Current armed-operation state variables across subsystems

**Output:**

- All armed-operation state variables cleared/reset
- Clean baseline for next arm cycle
- Log entry confirming state reset

**State Variables to Reset:**

1. **Failsafe System:**
   - Clear RC loss trigger flag
   - Clear battery low/critical trigger flags
   - Clear GCS loss trigger flag
   - Clear terrain failsafe trigger flag (if applicable)
   - Reset failsafe activation count
   - Clear failsafe action history

2. **Monitoring System:**
   - Reset RC signal strength baseline
   - Reset battery voltage baseline
   - Clear sensor health warning flags
   - Reset timeout counters
   - Clear accumulated error counts

3. **Mode System:**
   - Clear armed-specific mode state
   - Reset mode transition history
   - Clear mode-specific timers

4. **Emergency State:**
   - Clear emergency stop flag
   - Reset crash detection state
   - Clear forced disarm history

5. **Timing State:**
   - Clear arm timestamp
   - Reset armed duration counter
   - Clear timing-based triggers

**Behavior:**

1. After subsystem notification, begin state reset
2. Clear failsafe system state variables
3. Reset monitoring system baselines and counters
4. Clear mode system armed-specific state
5. Reset emergency stop and crash detection flags
6. Clear timing state (arm timestamp, duration)
7. Verify all state variables reset (sanity check)
8. Log state reset completion
9. If any reset fails, log error but continue with remaining resets
10. Complete disarm even if some resets fail

**Error Conditions:**

- State variable inaccessible: Log error, skip that variable
- State reset verification failure: Log warning with details
- State inconsistency detected: Force reset, log warning

## Platform Considerations

### Unix

N/A – Platform agnostic

### Windows

N/A – Platform agnostic

### Cross-Platform

N/A – Platform agnostic

## Risks & Mitigation

| Risk                                                     | Impact | Likelihood | Mitigation                                                            | Validation                                             |
| -------------------------------------------------------- | ------ | ---------- | --------------------------------------------------------------------- | ------------------------------------------------------ |
| Incomplete state reset (residual state affects next arm) | High   | Medium     | Comprehensive checklist, verification checks, log all state resets    | Test multiple arm/disarm cycles for state accumulation |
| State reset takes too long (delays disarm)               | Medium | Low        | Profile state reset, optimize slow operations, target <1ms            | Profile state reset duration on target hardware        |
| Hidden state variables not identified                    | Medium | Medium     | Code review, testing with edge cases, monitor for unexpected behavior | Thorough testing with various failure scenarios        |
| State reset breaks other subsystem assumptions           | High   | Low        | Document state lifecycle, coordinate with subsystem owners            | Integration testing with all subsystems                |

## Implementation Notes

**Preferred approaches:**

- Create explicit reset functions for each subsystem (e.g., `failsafe.reset_armed_state()`)
- Use a checklist or state reset registry to ensure all state variables are reset
- Log each state reset step for debugging and verification
- Implement verification checks to detect incomplete resets

**Known pitfalls:**

- Do not assume state variables are automatically cleared
- Avoid hidden state in static variables or globals
- Do not forget timing-based state (timestamps, counters)
- Ensure state reset order matches dependency graph (reset dependents before dependencies)

**Related code areas:**

- `src/communication/mavlink/state.rs` - SystemState::disarm()
- `src/vehicle/arming/cleanup.rs` - Post-disarm cleanup logic
- `src/vehicle/monitoring/` - Monitoring system state
- `src/vehicle/failsafe/` - Failsafe system state
- `src/vehicle/modes/` - Mode system state

**Suggested patterns:**

- Builder pattern for state initialization
- Trait-based reset interface (e.g., `ArmedStateReset` trait)
- State verification functions to detect incomplete resets

## External References

- Analysis: [AN-00015-post-arm-initialization](../analysis/AN-00015-post-arm-initialization.md)
- [ArduPilot AP_Arming Disarm Sequence](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Arming/AP_Arming.cpp) - Reference implementation
