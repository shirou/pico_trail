# FR-00069 | Voltage Conversion Calculation

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-00019-battery-telemetry](../analysis/AN-00019-battery-telemetry.md)
- Prerequisite Requirements:
  - [FR-00066-adc-battery-voltage-reading](../requirements/FR-00066-adc-battery-voltage-reading.md)
- Dependent Requirements:
  - [FR-00068-battery-status-telemetry](../requirements/FR-00068-battery-status-telemetry.md)
  - [FR-00067-battery-state-scheduler-update](../requirements/FR-00067-battery-state-scheduler-update.md)
  - [NFR-00061-calibration-persistence](../requirements/NFR-00061-calibration-persistence.md)
  - [NFR-00062-host-test-support](../requirements/NFR-00062-host-test-support.md)
- Related Tasks:
  - [T-00013-battery-telemetry](../tasks/T-00013-battery-telemetry/README.md)

## Requirement Statement

The system shall convert raw ADC counts to battery voltage in volts using a configurable voltage divider coefficient that accounts for hardware scaling.

## Rationale

Raw ADC values (0-4095) represent scaled voltage after passing through the Freenove hardware voltage divider. Converting to actual battery voltage enables meaningful telemetry reporting, threshold comparisons for failsafes, and pre-arm checks. A configurable coefficient accommodates component tolerance variations between hardware units.

## User Story (if applicable)

As a rover operator, I want the system to display actual battery voltage in volts, so that I can compare readings with my multimeter and know when the battery needs charging.

## Acceptance Criteria

- [ ] Conversion formula: `voltage = (adc_value / 4095.0 * 3.3) * coefficient`
- [ ] Voltage divider coefficient loaded from ArduPilot-compatible parameter (see Technical Details)
- [ ] Default coefficient value is 3.95 (per Freenove reference implementation)
- [ ] Voltage accuracy within ±0.2V compared to multimeter reading at 12V nominal
- [ ] Conversion result stored in `BatteryState.voltage` field as f32 in volts
- [ ] Supports voltage range 9.0V to 12.6V (3S LiPo operating range)

## Technical Details

### Functional Requirement Details

**Voltage Divider Calculation:**

The Freenove hardware uses a resistor divider to scale 12V battery voltage to the RP2350's 3.3V ADC range.

1. **ADC to voltage at pin**: `voltage_at_pin = (adc_value / 4095.0) * 3.3V`
2. **Pin voltage to battery voltage**: `battery_voltage = voltage_at_pin * coefficient`
3. **Combined formula**: `battery_voltage = (adc_value / 4095.0 * 3.3) * coefficient`

**Coefficient Determination:**

- Freenove reference: 3.95 (for 3S LiPo with 12.6V max)
- Theoretical divider ratio: 12.6V / 3.3V ≈ 3.82x
- Difference accounts for component tolerances

**Parameter Configuration:**

According to CLAUDE.md parameter standards, this project must use ArduPilot-compatible parameters. After reviewing ArduPilot documentation, the appropriate parameter for voltage divider scaling is:

- **Parameter Name**: `BATT_VOLT_MULT` (ArduPilot standard)
- **Description**: Voltage multiplier to convert ADC reading to battery voltage
- **Type**: Float
- **Default**: 3.95 (Freenove hardware calibration)
- **Range**: 1.0 to 10.0
- **ArduPilot Reference**: Used in AP_BattMonitor for analog voltage sensing

**Note**: This differs from the analysis document's suggestion of `BATT_VOLT_MULT` being custom. Upon verification, `BATT_VOLT_MULT` is a standard ArduPilot parameter used in the AP_BattMonitor library for analog voltage sensor scaling.

**Integration Points:**

- Parameter loaded from `src/parameters/battery.rs` (requires parameter addition)
- Conversion performed in battery update function
- Result updates `BatteryState.voltage` in `src/communication/mavlink/state.rs:97`

**Calibration Process (for users):**

1. Measure actual battery voltage with multimeter (e.g., 12.1V)
2. Read system-reported voltage via MAVLink telemetry
3. Calculate correction: `new_coefficient = old_coefficient * (multimeter_reading / system_reading)`
4. Set `BATT_VOLT_MULT` parameter via Mission Planner or QGroundControl
5. Reboot system to apply new coefficient

**Example:**

- ADC reading: 3000 counts
- Conversion: `(3000 / 4095.0 * 3.3) * 3.95 = 9.52V`

## Platform Considerations

### Unix

N/A – Platform agnostic (embedded-only feature)

### Windows

N/A – Platform agnostic (embedded-only feature)

### Cross-Platform

Voltage conversion logic is platform-agnostic. Host tests shall use hardcoded coefficient or parameter mock.

## Risks & Mitigation

| Risk                                              | Impact | Likelihood | Mitigation                                          | Validation                               |
| ------------------------------------------------- | ------ | ---------- | --------------------------------------------------- | ---------------------------------------- |
| Coefficient varies between hardware units         | Medium | High       | Make `BATT_VOLT_MULT` user-configurable parameter   | Test with multiple hardware units        |
| Component aging changes voltage divider ratio     | Low    | Medium     | Document calibration procedure in user manual       | Periodic calibration check               |
| Incorrect parameter name conflicts with ArduPilot | High   | Low        | Use exact ArduPilot parameter naming convention     | Cross-reference with ArduPilot docs      |
| Voltage reading exceeds expected range (>12.6V)   | Medium | Low        | Add sanity check and log warning if voltage > 13.0V | Test with power supply at voltage limits |

## Implementation Notes

**Preferred Approaches:**

- Add `BATT_VOLT_MULT` parameter to `src/parameters/battery.rs` following existing `BatteryParams` pattern
- Create conversion method in `BatteryState` struct: `fn from_adc(adc_value: u16, coefficient: f32) -> f32`
- Use floating-point arithmetic (f32) for precision; RP2350 has hardware FPU
- Example implementation:
  ```rust
  impl BatteryState {
      pub fn voltage_from_adc(adc_value: u16, volt_mult: f32) -> f32 {
          const ADC_MAX: f32 = 4095.0;
          const ADC_VREF: f32 = 3.3;
          (adc_value as f32 / ADC_MAX * ADC_VREF) * volt_mult
      }
  }
  ```

**Known Pitfalls:**

- Do not use integer division; ensure floating-point calculation
- RP2040/RP2350 ADC max is 4095 (12-bit), not 1023 (10-bit Arduino)
- Parameter must persist to flash (use ParameterStore); see NFR-00061

**Related Code Areas:**

- `src/parameters/battery.rs` - Battery parameter definitions
- `src/communication/mavlink/state.rs:97` - BatteryState struct
- `src/core/arming/checks.rs:179` - Pre-arm battery voltage check

**Suggested Validation:**

- Unit test with known ADC values (e.g., 3000 counts → \~9.5V with 3.95 coefficient)
- Hardware test comparing system voltage with multimeter reading

## External References

- [ArduPilot Battery Parameters - BATT_VOLT_MULT](https://ardupilot.org/copter/docs/parameters.html#batt-volt-mult-voltage-multiplier) - Standard parameter definition
- [Freenove 4WD Car Battery Level Example](https://github.com/Freenove/Freenove_4WD_Car_Kit_for_Raspberry_Pi) - Reference coefficient value
- Voltage Divider Calculator - Theory and examples | N/A
