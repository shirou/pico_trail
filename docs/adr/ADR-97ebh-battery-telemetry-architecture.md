# ADR-97ebh Battery Telemetry Architecture

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-015k2-adc-battery-voltage-reading](../requirements/FR-015k2-adc-battery-voltage-reading.md)
  - [FR-uq6as-voltage-conversion-calculation](../requirements/FR-uq6as-voltage-conversion-calculation.md)
  - [FR-zxxlp-battery-status-telemetry](../requirements/FR-zxxlp-battery-status-telemetry.md)
  - [FR-ygjkj-battery-state-scheduler-update](../requirements/FR-ygjkj-battery-state-scheduler-update.md)
  - [NFR-r1h41-adc-performance-constraint](../requirements/NFR-r1h41-adc-performance-constraint.md)
  - [NFR-tximn-host-test-support](../requirements/NFR-tximn-host-test-support.md)
- Supersedes ADRs: None
- Related Tasks:
  - [T-q3psc-battery-telemetry](../tasks/T-q3psc-battery-telemetry/README.md)

## Context

The pico_trail rover platform requires battery voltage monitoring for safety-critical operations including pre-arm checks, failsafe triggering, and ground control station telemetry. The Freenove 4WD car kit provides battery voltage sensing hardware on GPIO 26, but the current implementation only uses placeholder values (12.0V).

**Key constraints:**

- RP2350 (Pico 2 W) platform with 12-bit ADC (0-4095 counts)
- Voltage divider circuit scales 12V battery to 3.3V ADC range (coefficient \~3.95)
- No current sensor available on Freenove hardware
- Must integrate with existing 400 Hz control loop without timing disruption
- Must follow ArduPilot parameter conventions per CLAUDE.md standards
- Ground control stations (Mission Planner, QGroundControl) expect MAVLink BATTERY_STATUS (#147) messages

**Forces in tension:**

1. **Safety vs. Complexity**: Battery monitoring is safety-critical but adds ADC driver integration, scheduler changes, and telemetry handlers
2. **Compatibility vs. Simplicity**: Full BATTERY_STATUS message (41 bytes) provides ArduPilot compatibility but requires more implementation than basic SYS_STATUS voltage field
3. **Hardware Limitations**: No per-cell voltage or current sensing available, requiring placeholder/unknown values in some message fields
4. **Host Testing**: Embedded-only ADC driver conflicts with cross-platform testing requirements

**Prior art:**

- Freenove reference implementation uses 5-sample averaging with coefficient 3.95
- ArduPilot uses `BATT_VOLT_MULT` parameter for analog voltage sensor scaling
- Existing BatteryState struct and SYS_STATUS message provide foundation

## Success Metrics

- Metric 1: Battery voltage accuracy within ±0.2V compared to multimeter reading
- Metric 2: ADC read completes within 2ms worst-case (meets NFR-r1h41)
- Metric 3: Ground control station displays battery status correctly (Mission Planner verification)
- Metric 4: Host unit tests pass without hardware (`cargo test --lib`)

## Decision

We will implement battery voltage monitoring with MAVLink BATTERY_STATUS (#147) telemetry using the following architecture:

1. **ADC Driver**: Use embassy-rp HAL for RP2350 ADC access
2. **Voltage Conversion**: Use ArduPilot standard `BATT_VOLT_MULT` parameter (default 3.95)
3. **Telemetry**: Implement BATTERY_STATUS message streaming at 2 Hz
4. **Scheduler Integration**: Update BatteryState at 10 Hz in main loop
5. **Host Testing**: Provide mock ADC via Board trait abstraction

### Decision Drivers

- **ArduPilot Compatibility**: Ground control stations expect standard MAVLink messages and parameters
- **Safety Requirements**: Pre-arm checks and failsafes depend on accurate voltage readings
- **Performance Constraints**: Must not disrupt 400 Hz control loop timing
- **Developer Experience**: Host tests enable fast iteration without hardware

### Considered Options

- Option A: Minimal implementation (SYS_STATUS only, no BATTERY_STATUS)
- Option B: Full BATTERY_STATUS implementation (selected)
- Option C: Custom parameter naming (BATT_ADC_COEFF vs BATT_VOLT_MULT)

### Option Analysis

- Option A — Pros: Smallest code change (50-100 LOC), SYS_STATUS already exists | Cons: Not ArduPilot standard, limited GCS display
- Option B — Pros: Full compatibility, future-proof, professional GCS display | Cons: More implementation effort (150-200 LOC)
- Option C — Pros: Custom naming clarity | Cons: Violates CLAUDE.md ArduPilot parameter standards, confuses users familiar with ArduPilot

## Rationale

**Why Option B (Full BATTERY_STATUS):**

- Ground control software expects BATTERY_STATUS for detailed battery monitoring
- ArduPilot compatibility enables community reuse and familiar user experience
- One-time implementation cost provides long-term maintainability
- Future current sensor integration requires BATTERY_STATUS fields already

**Why BATT_VOLT_MULT parameter:**

- Standard ArduPilot parameter from AP_BattMonitor library
- Documented in ArduPilot parameter reference
- Follows CLAUDE.md requirement to use ArduPilot parameters, not custom ones
- Users familiar with ArduPilot understand this parameter name

**Why embassy-rp ADC:**

- Official HAL for RP2350/RP2040 with active maintenance
- Async API integrates with embassy executor used throughout project
- Proven in Raspberry Pi ecosystem
- No need for direct register manipulation

**Why 10 Hz battery update rate:**

- Matches ArduPilot medium-frequency monitoring convention
- Provides 100ms response time for failsafe detection
- Low overhead (2ms ADC read budget per NFR-r1h41)
- Fast enough for battery voltage dynamics (seconds to minutes)

## Consequences

### Positive

- Ground control stations display rich battery information (voltage, capacity, chemistry)
- Pre-arm checks use real voltage instead of placeholder
- Voltage-based failsafes work correctly (BATT_CRT_VOLT triggers)
- User-calibratable voltage divider coefficient accommodates hardware variations
- Host unit tests enable fast iteration without hardware
- ArduPilot parameter compatibility reduces learning curve for users

### Negative

- Implementation complexity increases (ADC driver, telemetry handler, scheduler integration)
- No current sensor means current-related fields use placeholder (-1/unknown)
- No per-cell voltage monitoring (single pack voltage only)
- embassy-rp dependency increases binary size (\~5-10KB estimated)
- Host tests require mock Board implementations

### Neutral

- Existing SYS_STATUS message continues to include battery voltage (backward compatibility maintained)
- Freenove voltage divider coefficient (3.95) may vary between hardware units (addressed by calibratable parameter)

## Implementation Notes

**ADC Integration:**

- Initialize embassy-rp `Adc` at board startup
- Configure GPIO 26 as ADC0 channel with `Channel::new_pin`
- Implement 5-sample averaging per Freenove reference
- Add `read_battery_adc() -> u16` method to Board trait

**Voltage Conversion:**

- Add `BATT_VOLT_MULT` to BatteryParams in `src/parameters/battery.rs`
- Formula: `voltage = (adc_value / 4095.0 * 3.3) * BATT_VOLT_MULT`
- Create `BatteryState::voltage_from_adc(adc: u16, mult: f32) -> f32` helper

**BATTERY_STATUS Message:**

- Create `build_battery_status(state: &SystemState) -> MavMessage` in telemetry handler
- Field population:
  - `id`: 0 (first battery)
  - `battery_function`: MAV_BATTERY_FUNCTION_ALL (1)
  - `type`: MAV_BATTERY_TYPE_LIPO (1)
  - `temperature`: INT16_MAX (unknown)
  - `voltages[0]`: pack voltage in mV
  - `voltages[1..9]`: UINT16_MAX (unknown)
  - `current_battery`: -1 (no current sensor)
  - `current_consumed`: -1 (unknown)
  - `energy_consumed`: -1 (unknown)
  - `battery_remaining`: -1 or voltage-based estimate (future)

**Scheduler Integration:**

- Add 10 Hz battery update timer in main loop
- Call `system_state.update_battery(&board).await` every 100ms
- Ensure update runs before pre-arm checks in scheduler order

**Host Testing:**

- Create `MockBoard` with `#[cfg(test)]` for unit tests
- Implement configurable ADC value via `set_adc_value(u16)`
- Use tokio::test for async test runtime
- Validate voltage conversion, pre-arm logic, failsafe triggering

**Error Handling:**

- ADC read timeout (>2ms) logs error and uses last known value
- ADC driver initialization failure logs warning but does not crash
- Invalid voltage range (>13.0V) logs warning

## Examples

**Battery voltage conversion:**

```rust
// ADC reading: 3000 counts
// Conversion: (3000 / 4095.0 * 3.3) * 3.95 = 9.52V
let adc_value = 3000_u16;
let volt_mult = 3.95_f32;
let voltage = BatteryState::voltage_from_adc(adc_value, volt_mult);
assert!((voltage - 9.52).abs() < 0.01);
```

**Scheduler integration:**

```rust
use embassy_time::{Duration, Ticker};

let mut battery_ticker = Ticker::every(Duration::from_hz(10));
loop {
    select! {
        _ = battery_ticker.next() => {
            system_state.update_battery(&board).await;
        }
        // ... other scheduler events ...
    }
}
```

**BATTERY_STATUS message:**

```rust
pub fn build_battery_status(state: &SystemState) -> MavMessage {
    let voltage_mv = (state.battery.voltage * 1000.0) as u16;
    let mut voltages = [u16::MAX; 10];
    voltages[0] = voltage_mv;

    MavMessage::BATTERY_STATUS(BATTERY_STATUS_DATA {
        id: 0,
        battery_function: MavBatteryFunction::MAV_BATTERY_FUNCTION_ALL as u8,
        type_: MavBatteryType::MAV_BATTERY_TYPE_LIPO as u8,
        temperature: i16::MAX,
        voltages,
        current_battery: -1,
        current_consumed: -1,
        energy_consumed: -1,
        battery_remaining: -1,
    })
}
```

## Platform Considerations

- **RP2350/RP2040**: 12-bit ADC resolution (0-4095), GPIO 26-29 support ADC, 500 kSPS max rate
- **Host Tests**: Mock ADC provides simulated values, no hardware access required
- **Cross-Platform**: Battery monitoring logic platform-agnostic, only ADC driver is embedded-specific

## Monitoring & Logging

- Log ADC read timing with defmt timestamps for performance validation
- Log warning if ADC read exceeds 2ms timeout
- Log warning if voltage exceeds expected range (>13.0V)
- Log battery voltage at startup for hardware verification

## Open Questions

- [ ] Should we implement voltage-based battery remaining percentage estimation? → Next step: Research ArduPilot's LiPo voltage curve algorithm in separate analysis
- [ ] Should BATTERY_STATUS be configurable via SR0_EXTRA1 parameter or hardcoded at 2 Hz? → Next step: Add SR0_EXTRA1 parameter in future task, hardcode 2 Hz initially

## External References

- [ArduPilot BATT_VOLT_MULT Parameter](https://ardupilot.org/copter/docs/parameters.html#batt-volt-mult-voltage-multiplier) - Standard parameter definition
- [MAVLink BATTERY_STATUS Message](https://mavlink.io/en/messages/common.html#BATTERY_STATUS) - Message specification
- [RP2040 Datasheet - ADC Section](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) - ADC specifications
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - ADC compatibility
- [Embassy-RP ADC Documentation](https://docs.embassy.dev/embassy-rp/) - HAL driver usage

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
