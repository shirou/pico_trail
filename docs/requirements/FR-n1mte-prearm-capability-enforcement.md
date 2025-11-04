# FR-n1mte Pre-Arm Checks Enforce Arming Permissions Using Capability Queries

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P0
- Category: Arming / Safety

## Links

- Parent Analysis: [AN-g5w99-mode-capability-system](../analysis/AN-g5w99-mode-capability-system.md)
- Related Analysis: [AN-r2fps-pre-arm-checks](../analysis/AN-r2fps-pre-arm-checks.md)
- Related Requirements:
  - [FR-qj0d1-mode-capability-declaration](FR-qj0d1-mode-capability-declaration.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Pre-arm checks shall enforce arming permissions using capability queries by calling `allows_arming()` before allowing arm, checking `allows_arming_from_transmitter()` for RC arming, denying arming if capability query returns false, and logging arming denial with mode name and reason.

## Rationale

Prevents arming in modes that don't allow it (e.g., theoretically possible modes that require post-arm setup). Prevents RC arming in autonomous modes (safety - avoid accidental mission start). Automated enforcement via capability queries ensures consistency.

## Acceptance Criteria

1. Pre-arm check algorithm:
   - Call `current_mode.allows_arming()`
   - If false, deny arming: "Mode {mode} does not allow arming"
   - For RC arming: Call `allows_arming_from_transmitter()`
   - If false, deny RC arming: "Mode {mode} does not allow RC arming"
2. Arming method detection:
   - GCS arming: COMMAND_LONG with MAV_CMD_COMPONENT_ARM_DISARM
   - RC arming: Rudder stick or RC switch
   - Apply appropriate capability check per method
3. Arming denial logging:
   - Log: `"ARMING_DENIED,{ts},{mode},{reason}"`
   - Send STATUSTEXT to GCS: "{reason}"
4. Integration with pre-arm checks:
   - Mode capability check part of pre-arm sequence
   - See AN-r2fps for full pre-arm check system

## Success Metrics

- 100% of arming attempts check mode capabilities
- RC arming denied in autonomous modes
- Arming denials logged with clear reason
- GCS displays arming denial message

## Dependencies

- Mode capability queries (FR-qj0d1)
- Pre-arm checks system (AN-r2fps)
- Arming command handlers (MAVLink)

## ArduPilot References

**File**: `Rover/AP_Arming.cpp` (lines 150-180)

```cpp
bool AP_Arming_Rover::arm_checks(AP_Arming::Method method)
{
    // Check if mode allows arming
    if (!rover.control_mode->allows_arming()) {
        check_failed(ARMING_CHECK_NONE, true,
                     "Mode %s does not allow arming",
                     rover.control_mode->name());
        return false;
    }

    // Check if RC arming allowed
    if (method == Method::RUDDER || method == Method::SWITCH) {
        if (!rover.control_mode->allows_arming_from_transmitter()) {
            check_failed(ARMING_CHECK_NONE, true,
                         "Mode %s does not allow RC arming",
                         rover.control_mode->name());
            return false;
        }
    }

    return true;
}
```

**Related ArduPilot Parameters**:

- **ARMING_CHECK** (bitmask) - Controls which pre-arm checks are performed
  - While not directly related to mode capabilities, it's used in the same pre-arm check framework
  - Mode capability checks are always performed (not optional)

## Verification Methods

- Unit tests: Mock mode capabilities, verify pre-arm checks enforce
- Integration tests: Attempt RC arming in Auto mode (should fail)
- HITL testing: Test arming sequences on hardware
- Safety review: Verify autonomous modes prevent RC arming

## Notes

- Mode capability checks always performed (not optional like ARMING_CHECK)
- GCS arming typically allowed in all modes
- RC arming restricted for safety
- Consider audible alert when RC arming denied

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
