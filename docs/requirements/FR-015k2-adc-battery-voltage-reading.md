# FR-015k2 | ADC Battery Voltage Reading

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-n8yb7-battery-telemetry](../analysis/AN-n8yb7-battery-telemetry.md)
- Prerequisite Requirements: None
- Dependent Requirements:
  - [FR-uq6as-voltage-conversion-calculation](../requirements/FR-uq6as-voltage-conversion-calculation.md)
  - [FR-ygjkj-battery-state-scheduler-update](../requirements/FR-ygjkj-battery-state-scheduler-update.md)
  - [NFR-r1h41-adc-performance-constraint](../requirements/NFR-r1h41-adc-performance-constraint.md)
  - [NFR-tximn-host-test-support](../requirements/NFR-tximn-host-test-support.md)
- Related Tasks:
  - [T-q3psc-battery-telemetry](../tasks/T-q3psc-battery-telemetry/README.md)

## Requirement Statement

The system shall read battery voltage from the ADC channel connected to GPIO 26 with noise reduction through multi-sample averaging.

## Rationale

The Freenove 4WD car kit includes battery voltage sensing hardware on GPIO 26. Reading this ADC channel is the foundation for battery monitoring, enabling voltage-based failsafes and telemetry reporting. Multi-sample averaging reduces ADC noise that could trigger false failsafes.

## User Story (if applicable)

As a rover operator, I want the system to accurately read battery voltage from hardware, so that I can monitor battery status and avoid power loss during missions.

## Acceptance Criteria

- [ ] ADC reads 12-bit value (0-4095) from GPIO 26 (ADC0 on RP2040/RP2350)
- [ ] Implements 5-sample averaging to reduce noise (per Freenove reference implementation)
- [ ] ADC reading updates at minimum 10 Hz frequency
- [ ] ADC driver uses embassy-rp HAL for RP2350 compatibility
- [ ] Reading completes within allocated time budget (see NFR-r1h41)

## Technical Details

### Functional Requirement Details

**Hardware Configuration:**

- Pin: GPIO 26 (ADC0 channel)
- ADC Resolution: 12-bit (0-4095 counts)
- ADC Reference Voltage: 3.3V
- Target Platform: RP2350A (Pico 2 W), compatible with RP2040

**ADC Reading Process:**

1. Configure GPIO 26 as ADC input using embassy-rp `Adc` driver
2. Read ADC value 5 times consecutively
3. Calculate average of 5 samples
4. Return averaged ADC count (0-4095)

**Integration Points:**

- ADC pin configuration defined in `src/platform/traits/board.rs:320` (`battery_adc`)
- Raw ADC value feeds into voltage conversion (FR-uq6as)
- Converted voltage updates `BatteryState` struct in `src/communication/mavlink/state.rs:97`

**Error Handling:**

- ADC read failure shall use last known good value
- Log warning if ADC driver initialization fails
- System shall not crash if ADC hardware is unavailable (for host testing)

## Platform Considerations

### Unix

N/A – Platform agnostic (embedded-only feature)

### Windows

N/A – Platform agnostic (embedded-only feature)

### Cross-Platform

This requirement applies only to embedded targets (pico2_w feature). Host tests shall use mock ADC values (see NFR-tximn).

## Risks & Mitigation

| Risk                                         | Impact | Likelihood | Mitigation                                     | Validation                              |
| -------------------------------------------- | ------ | ---------- | ---------------------------------------------- | --------------------------------------- |
| ADC noise causes voltage reading fluctuation | Medium | High       | 5-sample averaging per Freenove reference      | Test with oscilloscope on hardware      |
| embassy-rp ADC driver incompatible with code | High   | Low        | Verify embassy-rp version supports RP2350      | Compile test before implementation      |
| GPIO 26 already used by another peripheral   | High   | Low        | Document pin allocation in board configuration | Review board.rs pin assignments         |
| ADC sampling blocks scheduler interrupt      | High   | Low        | Ensure ADC read completes within time budget   | Measure actual ADC read time with defmt |

## Implementation Notes

**Preferred Approaches:**

- Use embassy-rp `embassy_rp::adc::Adc` and `embassy_rp::adc::Channel` for ADC operations
- Initialize ADC once at startup, store handle in board-specific state
- Example pattern from embassy-rp:
  ```rust
  let mut adc = Adc::new(p.ADC, Irqs, Config::default());
  let mut adc_pin = Channel::new_pin(p.PIN_26, Pull::None);
  let adc_value: u16 = adc.read(&mut adc_pin).await;
  ```

**Known Pitfalls:**

- RP2040/RP2350 ADC is 12-bit (0-4095), not Arduino's 10-bit (0-1023)
- Freenove reference code uses Arduino normalization (1023); we must use 4095
- ADC read is async in embassy-rp; ensure proper async context in scheduler

**Related Code Areas:**

- `src/platform/traits/board.rs` - Board pin definitions
- `src/communication/mavlink/state.rs:97` - BatteryState struct
- Future: `src/drivers/adc.rs` or similar module for ADC abstraction

## External References

- [RP2040 Datasheet - ADC Section](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) - ADC specifications
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - ADC compatibility with RP2040
- [embassy-rp ADC Documentation](https://docs.embassy.dev/embassy-rp/) - HAL driver usage
- Freenove 4WD Car Battery Level Example - Reference implementation

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
