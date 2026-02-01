# FR-00068 | BATTERY_STATUS Telemetry

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-00019-battery-telemetry](../analysis/AN-00019-battery-telemetry.md)
- Prerequisite Requirements:
  - [FR-00069-voltage-conversion-calculation](../requirements/FR-00069-voltage-conversion-calculation.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00013-battery-telemetry](../tasks/T-00013-battery-telemetry/README.md)

## Requirement Statement

The system shall transmit MAVLink BATTERY_STATUS (#147) messages containing battery voltage, current estimate, and remaining capacity at a configurable rate for ground control station monitoring.

## Rationale

Ground control software (Mission Planner, QGroundControl) expects BATTERY_STATUS messages for detailed battery monitoring. While the existing SYS_STATUS message includes basic battery voltage, BATTERY_STATUS provides comprehensive battery information including per-cell voltages (future), temperature, and energy consumption. This follows ArduPilot conventions and enables full-featured battery displays in ground control stations.

## User Story (if applicable)

As a rover operator using Mission Planner, I want to see detailed battery status in the battery monitor panel, so that I can track voltage trends and remaining capacity during missions.

## Acceptance Criteria

- [ ] Transmits MAVLink BATTERY_STATUS message (ID #147) via telemetry stream
- [ ] Message streams at 2 Hz by default (controlled by `SR0_EXTRA1` parameter per ArduPilot convention)
- [ ] Message includes battery voltage in millivolts (converted from BatteryState.voltage)
- [ ] Message includes current estimate in centiamps (10mA units) or -1 if unknown
- [ ] Message includes remaining battery percentage (0-100) or -1 if unknown
- [ ] Message compatible with Mission Planner and QGroundControl battery displays
- [ ] Existing SYS_STATUS (#1) message continues to include battery voltage for backward compatibility
- [ ] Message handler integrated with telemetry scheduler in main loop

## Technical Details

### Functional Requirement Details

**MAVLink BATTERY_STATUS Message Specification:**

```c
// MAVLink message #147
uint8_t id;                    // Battery instance (0 for first battery)
uint8_t battery_function;      // Function (0=unknown, 1=all flight systems)
uint8_t type;                  // Battery chemistry (0=unknown, 1=LiPo)
int16_t temperature;           // Temperature in cdegC (INT16_MAX if unknown)
uint16_t voltages[10];         // Cell voltages in mV (UINT16_MAX if unknown)
int16_t current_battery;       // Current in cA (10mA units), -1 if unknown
int32_t current_consumed;      // Consumed charge in mAh, -1 if unknown
int32_t energy_consumed;       // Consumed energy in hJ (hectojoules), -1 if unknown
int8_t battery_remaining;      // Remaining percentage (0-100), -1 if unknown
```

**Field Population Strategy:**

| Field             | Value Source                          | Notes                                     |
| ----------------- | ------------------------------------- | ----------------------------------------- |
| id                | 0 (first battery)                     | Single battery system                     |
| battery_function  | 1 (MAV_BATTERY_FUNCTION_ALL)          | Powers all flight systems                 |
| type              | 1 (MAV_BATTERY_TYPE_LIPO)             | 3S LiPo battery per Freenove spec         |
| temperature       | INT16_MAX                             | No temperature sensor; mark as unknown    |
| voltages\[0]      | (BatteryState.voltage \* 1000) as u16 | Total pack voltage in mV                  |
| voltages\[1..9]   | UINT16_MAX                            | No per-cell monitoring; mark as unknown   |
| current_battery   | -1 or estimate                        | No current sensor; placeholder (see note) |
| current_consumed  | -1                                    | No current integration; mark as unknown   |
| energy_consumed   | -1                                    | No energy tracking; mark as unknown       |
| battery_remaining | Voltage-based estimate or -1          | Optional: LiPo voltage curve estimation   |

**Current Estimation Note:** The Freenove platform lacks a current sensor. Options:

1. Return -1 (unknown) - simplest, honest approach
2. Estimate based on motor commands - complex, requires testing
3. Use fixed conservative value - may be misleading

**Recommendation**: Start with -1 (unknown); defer estimation to future work.

**Stream Rate Control:**

ArduPilot convention uses `SR0_EXTRA1` parameter to control BATTERY_STATUS message rate:

- Parameter: `SR0_EXTRA1` (stream rate for extra sensors)
- Default: 2 Hz (2 messages per second)
- Range: 0 (disabled) to 50 Hz
- Applied in telemetry scheduler alongside other EXTRA1 messages

**Implementation Location:**

- Message builder: `src/communication/mavlink/handlers/telemetry.rs` (add new function)
- Scheduler integration: Main loop at `examples/mavlink_rc_control.rs` (add to telemetry stream)

**Existing Code Pattern (from SYS_STATUS):**

```rust
// src/communication/mavlink/handlers/telemetry.rs:235
let voltage_battery = (state.battery.voltage * 1000.0) as u16;  // V to mV
let current_battery = (state.battery.current * 100.0) as i16;   // A to cA
```

BATTERY_STATUS should follow the same unit conversion pattern.

**Backward Compatibility:**

SYS_STATUS (#1) message already includes battery voltage at 1 Hz. Both messages should coexist:

- SYS_STATUS: Low-frequency (1 Hz), basic voltage for legacy GCS
- BATTERY_STATUS: Medium-frequency (2 Hz), detailed monitoring for modern GCS

## Platform Considerations

### Unix

N/A – Platform agnostic (embedded-only feature)

### Windows

N/A – Platform agnostic (embedded-only feature)

### Cross-Platform

MAVLink message encoding is platform-agnostic. Host tests shall verify message structure and field values using mock BatteryState.

## Risks & Mitigation

| Risk                                                       | Impact | Likelihood | Mitigation                                                  | Validation                                        |
| ---------------------------------------------------------- | ------ | ---------- | ----------------------------------------------------------- | ------------------------------------------------- |
| Ground control station does not display BATTERY_STATUS     | Medium | Low        | Test with Mission Planner and QGroundControl                | Verify battery panel shows voltage and percentage |
| Current sensor absence reduces battery monitoring utility  | Low    | Certain    | Document limitation; use voltage-based estimation           | User feedback and documentation clarity           |
| Stream rate parameter (`SR0_EXTRA1`) not implemented       | Medium | Medium     | Add parameter to telemetry configuration (future work)      | Hardcode 2 Hz initially, add parameter later      |
| BATTERY_STATUS conflicts with SYS_STATUS causing confusion | Low    | Low        | Both messages required by ArduPilot; no conflict expected   | Test with GCS to confirm both messages handled    |
| Message size increases telemetry bandwidth usage           | Low    | Medium     | BATTERY_STATUS is 41 bytes at 2 Hz = 82 bytes/sec (minimal) | Measure actual telemetry bandwidth with defmt     |

## Implementation Notes

**Preferred Approaches:**

- Create `build_battery_status()` function in `src/communication/mavlink/handlers/telemetry.rs`
- Mirror existing `build_sys_status()` pattern for consistency
- Use mavlink-rust crate message builders
- Example structure:

  ```rust
  pub fn build_battery_status(state: &SystemState) -> mavlink::common::MavMessage {
      let voltage_mv = (state.battery.voltage * 1000.0) as u16;
      let mut voltages = [u16::MAX; 10];
      voltages[0] = voltage_mv;  // Total pack voltage

      mavlink::common::MavMessage::BATTERY_STATUS(
          mavlink::common::BATTERY_STATUS_DATA {
              id: 0,
              battery_function: mavlink::common::MavBatteryFunction::MAV_BATTERY_FUNCTION_ALL as u8,
              type_: mavlink::common::MavBatteryType::MAV_BATTERY_TYPE_LIPO as u8,
              temperature: i16::MAX,
              voltages,
              current_battery: -1,
              current_consumed: -1,
              energy_consumed: -1,
              battery_remaining: -1,
          }
      )
  }
  ```

**Known Pitfalls:**

- MAVLink uses different unknown/invalid sentinels for different types (INT16_MAX, UINT16_MAX, -1)
- Cell voltage array size is fixed at 10; must fill unused slots with UINT16_MAX
- Battery remaining percentage is int8_t, not uint8_t; -1 means unknown
- `SR0_EXTRA1` parameter may not exist yet; hardcode 2 Hz initially

**Related Code Areas:**

- `src/communication/mavlink/handlers/telemetry.rs` - Telemetry message builders
- `src/communication/mavlink/state.rs:97` - BatteryState source data
- Main scheduler loop - Telemetry streaming at 2 Hz

**Integration with Scheduler:**

- Add 2 Hz timer for BATTERY_STATUS in main loop
- Reuse existing BatteryState updates (10 Hz from FR-00067)
- Message generation is fast (<100µs); no blocking concerns

## External References

- [MAVLink BATTERY_STATUS Message Specification](https://mavlink.io/en/messages/common.html#BATTERY_STATUS) - Official message definition
- [MAVLink Battery Protocol](https://mavlink.io/en/services/battery.html) - Protocol usage guidelines
- [ArduPilot Stream Rate Parameters](https://ardupilot.org/copter/docs/parameters.html#sr0-extra1-extra-data-type-1-stream-rate) - SR0_EXTRA1 definition
- Mission Planner Battery Monitor Documentation - GCS display requirements | N/A
