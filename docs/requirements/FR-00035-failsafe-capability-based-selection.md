# FR-00035 Failsafe System Uses Capability Queries for Fallback Mode Selection

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P0
- Category: Failsafe / Safety

## Links

- Parent Analysis: [AN-00012-mode-capability-system](../analysis/AN-00012-mode-capability-system.md)
- Related Analysis: [AN-00011-failsafe-system](../analysis/AN-00011-failsafe-system.md)
- Related Requirements:
  - [FR-00039-fallback-mode-selection](FR-00039-fallback-mode-selection.md)
  - [FR-00044-mode-capability-declaration](FR-00044-mode-capability-declaration.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Failsafe system shall use capability queries for fallback mode selection by checking mode capabilities, skipping modes with unmet requirements, selecting first mode with met requirements from priority list, and always having fallback (Manual requires no sensors).

## Rationale

Capability-based fallback selection automatically adapts to available sensors. If GPS lost, skip modes requiring position. Select appropriate mode based on actual sensor availability, not hardcoded assumptions.

## Acceptance Criteria

1. Fallback mode selection algorithm:
   - Define priority list: RTL → Hold → Stabilize → Manual
   - For each mode in priority order:
     - If `mode.requires_position()` and !has_position → skip
     - If `mode.requires_velocity()` and !has_velocity → skip
     - If `mode.requires_gps()` and !has_gps → skip
     - If requirements met → select this mode
   - Manual always succeeds (requires no sensors)
2. Capability-driven logic:
   - No hardcoded mode names in fallback algorithm
   - Works generically via capability queries
   - Adding new mode updates priority list only
3. Fallback logging:
   - Log attempted modes and skip reasons
   - Log selected fallback mode and reason
   - Format: `"FAILSAFE_FALLBACK,{ts},{selected_mode},{reason}"`
4. Integration with failsafe system:
   - Called when sensor loss detected
   - See FR-00039 for fallback selection details
   - See AN-00011 for full failsafe system

## Success Metrics

- Fallback selection uses capability queries
- Appropriate mode selected based on sensors
- Manual always succeeds as last fallback
- Fallback selection logged with reasoning

## Dependencies

- Mode capability queries (FR-00044)
- Failsafe system (AN-00011)
- Sensor health monitoring
- Fallback mode selection (FR-00039)

## ArduPilot References

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
        // Check capabilities vs available sensors
        if (mode->requires_position() && !ekf_position_ok()) {
            continue;  // Skip this mode
        }
        if (mode->requires_velocity() && !ekf_velocity_ok()) {
            continue;
        }

        // Requirements met - select mode
        set_mode(*mode, ModeReason::FAILSAFE);
        return;
    }

    // Force Manual as last resort
    set_mode(mode_manual, ModeReason::FAILSAFE);
}
```

**Pattern**: Iterate fallback modes, use capability queries to check requirements, select first viable mode.

**Related ArduPilot Parameters**:

- **FS_ACTION** (u8) - Failsafe action determines fallback mode priority
  - 0 = None
  - 1 = Hold
  - 2 = RTL
  - 3 = SmartRTL
  - Used to configure fallback mode preference

**Note**: Phase 1 implements basic fallback (Hold → Manual). Phase 2 can add RTL and make fallback configurable via FS_ACTION parameter.

## Verification Methods

- Unit tests: Mock sensor failures, verify capability-based selection
- Integration tests: Test fallback under various sensor failure scenarios
- HITL testing: Induce sensor failures, verify appropriate mode selected
- Log analysis: Verify fallback reasoning logged

## Notes

- Capability-based selection is extensible for new modes
- Manual always available (no sensor requirements)
- Fallback priorities may be configurable (Phase 2)
- Consider hysteresis to prevent mode oscillation
