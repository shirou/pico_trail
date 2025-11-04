# FR-nl9o0 Battery Voltage and Capacity Failsafe

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-7nn0e-battery-voltage-monitoring](FR-7nn0e-battery-voltage-monitoring.md)
  - [FR-jiotb-configuration-persistence](FR-jiotb-configuration-persistence.md)

- Dependent Requirements:
  - [FR-0wy2c-failsafe-action-priority](FR-0wy2c-failsafe-action-priority.md)

- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall monitor battery voltage continuously and trigger failsafe actions at LOW and CRITICAL thresholds, using a two-stage system with less aggressive action at LOW threshold and more aggressive action at CRITICAL threshold.

## Rationale

Battery failsafe prevents over-discharge damage and ensures sufficient power for safe landing/RTL. Two-stage system provides warning (LOW) before emergency (CRITICAL), allowing operator intervention or automated RTL before battery depletion.

## User Story (if applicable)

As a safety officer, I want the vehicle to automatically return to launch when battery voltage drops to LOW threshold (e.g., 10.5V for 3S LiPo) and land immediately at CRITICAL threshold (e.g., 10.0V), so that battery damage and vehicle stranding are prevented.

## Acceptance Criteria

- [ ] Check battery voltage every 100ms (10 Hz)
- [ ] Trigger LOW failsafe when voltage < `BATT_LOW_VOLT` for 10+ seconds continuous
- [ ] Trigger CRITICAL failsafe when voltage < `BATT_CRT_VOLT` immediately (no delay)
- [ ] Execute `BATT_FS_LOW_ACT` action on LOW threshold
- [ ] Execute `BATT_FS_CRT_ACT` action on CRITICAL threshold
- [ ] Send STATUSTEXT: "Failsafe: Battery Low" / "Battery Critical" with severity WARNING/CRITICAL
- [ ] Log battery voltage, capacity (if available), and action at failsafe trigger
- [ ] Battery failsafe does not auto-clear (voltage recovery too slow, action must complete)
- [ ] Default: Battery thresholds disabled (`BATT_LOW_VOLT=0.0`, `BATT_CRT_VOLT=0.0`)

## Technical Details (if applicable)

### Functional Requirement Details

**ArduPilot Parameters**:

- `BATT_LOW_VOLT`: Low voltage threshold (float, default 0.0=disabled, range 0.0-30.0V)
- `BATT_CRT_VOLT`: Critical voltage threshold (float, default 0.0=disabled, range 0.0-30.0V)
- `BATT_FS_LOW_ACT`: Low battery action (u8, default 0=None, 0-5)
- `BATT_FS_CRT_ACT`: Critical battery action (u8, default 0=None, 0-5)

**Example Configuration** (3S LiPo, 3300mAh):

- `BATT_LOW_VOLT=10.5` (3.5V per cell)
- `BATT_CRT_VOLT=10.0` (3.33V per cell minimum safe)
- `BATT_FS_LOW_ACT=2` (RTL on low battery)
- `BATT_FS_CRT_ACT=1` (Hold on critical battery)

**Two-Stage System**:

1. LOW threshold: Warning, less aggressive action (RTL recommended)
2. CRITICAL threshold: Emergency, more aggressive action (Hold/Land/Disarm)

## Platform Considerations

### Cross-Platform

Battery monitoring uses platform abstraction for voltage reading.

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                                  |
| ----------------------------------------- | ------ | ---------- | ----------------------------------------------------------- |
| False trigger from voltage sag under load | Medium | High       | 10 second delay on LOW, use moving average, tune thresholds |
| Threshold misconfiguration (too high/low) | High   | High       | Default disabled, document per-battery configuration        |
| Vehicle stranded (LOW threshold too low)  | High   | Medium     | Conservative defaults, test before missions                 |

## Implementation Notes

- Default thresholds to 0.0V (disabled), require explicit configuration
- Document threshold selection per battery type (3S, 4S, LiPo, Li-ion)
- Use moving average filter to reduce voltage noise
- 10 second delay on LOW prevents false triggers from transient voltage sag

Related code areas:

- `src/vehicle/failsafe/checkers/battery.rs`
- `src/devices/battery/` - Battery monitor

## External References

- ArduPilot Battery Failsafe: <https://ardupilot.org/rover/docs/rover-failsafes.html>
- ArduPilot Battery Monitor: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_BattMonitor/>
- Analysis: [AN-kajh6-failsafe-system](../analysis/AN-kajh6-failsafe-system.md)

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
