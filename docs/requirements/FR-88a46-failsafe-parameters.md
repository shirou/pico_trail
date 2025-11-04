# FR-88a46 Configurable Failsafe Parameters

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-a1cuu-runtime-parameters](FR-a1cuu-runtime-parameters.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall provide configurable failsafe parameters for timeouts, thresholds, and actions, following ArduPilot parameter naming conventions and persisting to storage across reboots.

## Rationale

Customization per vehicle type, mission profile, and operator preference is essential. ArduPilot-compatible parameter names ease migration and operator familiarity.

## Acceptance Criteria

- [ ] `FS_TIMEOUT`: RC timeout (float, 0.0-10.0s, default 1.5s)
- [ ] `FS_THR_ENABLE`: Enable RC failsafe (bool, default true)
- [ ] `FS_ACTION`: RC action (u8, 0-5, default 1=Hold)
- [ ] `FS_GCS_TIMEOUT`: GCS timeout (float, 0.0-30.0s, default 5.0s)
- [ ] `FS_GCS_ENABLE`: Enable GCS failsafe (bool, default false)
- [ ] `BATT_LOW_VOLT`: Low voltage (float, 0.0-30.0V, default 0.0=disabled)
- [ ] `BATT_CRT_VOLT`: Critical voltage (float, 0.0-30.0V, default 0.0=disabled)
- [ ] `BATT_FS_LOW_ACT`: Low battery action (u8, 0-5, default 0=None)
- [ ] `BATT_FS_CRT_ACT`: Critical battery action (u8, 0-5, default 0=None)
- [ ] Parameters persist to storage, survive reboot
- [ ] Parameter changes take effect immediately (no reboot required)

## Technical Details (if applicable)

**ArduPilot Compatibility**:

All parameters match ArduPilot Rover naming and value ranges for operator familiarity.

**Action Values**:

- 0 = None
- 1 = Hold
- 2 = RTL
- 3 = SmartRTL
- 4 = SmartRTL_Hold
- 5 = Disarm/Terminate

## External References

- ArduPilot Parameters: <https://ardupilot.org/rover/docs/parameters.html>
- Analysis: [AN-kajh6-failsafe-system](../analysis/AN-kajh6-failsafe-system.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
