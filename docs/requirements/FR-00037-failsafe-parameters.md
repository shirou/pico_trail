# FR-00037 Configurable Failsafe Parameters

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00006-runtime-parameters](FR-00006-runtime-parameters.md)
- Dependent Requirements:
  - [FR-00041-gcs-loss-failsafe](FR-00041-gcs-loss-failsafe.md)
- Related Tasks:
  - [T-00167-gcs-failsafe-implementation](../tasks/T-00167-gcs-failsafe-implementation/README.md)

## Requirement Statement

The system shall provide configurable failsafe parameters for timeouts, thresholds, and actions, following ArduPilot parameter naming conventions and persisting to storage across reboots.

## Rationale

Customization per vehicle type, mission profile, and operator preference is essential. ArduPilot-compatible parameter names ease migration and operator familiarity.

## Acceptance Criteria

- [ ] `FS_TIMEOUT`: Failsafe condition persistence timeout (float, 1.0-100.0s, default 1.5s) -- shared between RC and GCS failsafes
- [ ] `FS_THR_ENABLE`: Enable RC failsafe (bool, default true)
- [ ] `FS_ACTION`: Failsafe action (u8, 0-5, default 2=Hold, ArduPilot numbering: 0=None, 1=RTL, 2=Hold)
- [ ] `FS_GCS_TIMEOUT`: GCS detection timeout (float, 2.0-120.0s, default 5.0s)
- [ ] `FS_GCS_ENABLE`: Enable GCS failsafe (u8, 0-2, default 1=Enabled)
- [ ] `BATT_LOW_VOLT`: Low voltage (float, 0.0-30.0V, default 0.0=disabled)
- [ ] `BATT_CRT_VOLT`: Critical voltage (float, 0.0-30.0V, default 0.0=disabled)
- [ ] `BATT_FS_LOW_ACT`: Low battery action (u8, 0-5, default 0=None)
- [ ] `BATT_FS_CRT_ACT`: Critical battery action (u8, 0-5, default 0=None)
- [ ] Parameters persist to storage, survive reboot
- [ ] Parameter changes take effect immediately (no reboot required)

## Technical Details (if applicable)

### ArduPilot Compatibility

All parameters match ArduPilot Rover naming and value ranges for operator familiarity.

### FS_ACTION Values (ArduPilot Rover Numbering)

| Value | Action           | Behavior                         |
| ----- | ---------------- | -------------------------------- |
| 0     | None             | No mode change, warning only     |
| 1     | RTL              | Return to launch                 |
| 2     | Hold             | Stop and hold position (default) |
| 3     | SmartRTL or RTL  | Try SmartRTL, fallback to RTL    |
| 4     | SmartRTL or Hold | Try SmartRTL, fallback to Hold   |
| 5     | Terminate        | Immediate disarm                 |

**Note**: Current code has Hold=1, RTL=2 (swapped from ArduPilot). Must be corrected to match ArduPilot numbering during GCS failsafe implementation per [AN-00046](../analysis/AN-00046-communication-lost-action.md).

**Note**: `FS_GCS_ENABLE` defaults to 1 (Enabled), which differs from ArduPilot's default of 0 (Disabled). This is intentional because pico_trail operates primarily over WiFi where GCS failsafe protection is important.

### Two-Timeout System

- **`FS_GCS_TIMEOUT`** (default 5.0s): Time since last GCS message before GCS loss condition is detected
- **`FS_TIMEOUT`** (default 1.5s): Time the failsafe condition must persist before action executes (shared between RC and GCS failsafes)

## External References

- ArduPilot Parameters: <https://ardupilot.org/rover/docs/parameters.html>
- Analysis: [AN-00011-failsafe-system](../analysis/AN-00011-failsafe-system.md)
- Analysis: [AN-00046-communication-lost-action](../analysis/AN-00046-communication-lost-action.md)
