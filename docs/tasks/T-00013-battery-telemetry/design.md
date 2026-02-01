# T-00013 Battery Telemetry Implementation

## Metadata

- Type: Design
- Status: Completed

## Links

- Associated Plan Document:
  - [T-00013-battery-telemetry-plan](./plan.md)
- Architecture Decision Record:
  - [ADR-00017-battery-telemetry-architecture](../../adr/ADR-00017-battery-telemetry-architecture.md)
- Impacted Requirements:
  - [FR-00066-adc-battery-voltage-reading](../../requirements/FR-00066-adc-battery-voltage-reading.md)
  - [FR-00069-voltage-conversion-calculation](../../requirements/FR-00069-voltage-conversion-calculation.md)
  - [FR-00068-battery-status-telemetry](../../requirements/FR-00068-battery-status-telemetry.md)
  - [FR-00067-battery-state-scheduler-update](../../requirements/FR-00067-battery-state-scheduler-update.md)
  - [NFR-00060-adc-performance-constraint](../../requirements/NFR-00060-adc-performance-constraint.md)
  - [NFR-00062-host-test-support](../../requirements/NFR-00062-host-test-support.md)

## Overview

Implement battery voltage monitoring for the pico_trail rover platform using the Freenove 4WD car kit's GPIO 26 voltage sensing hardware. Replace placeholder battery voltage values (12.0V) with real ADC readings converted via ArduPilot-standard `BATT_VOLT_MULT` parameter. Stream MAVLink BATTERY_STATUS (#147) messages at 2 Hz for ground control station compatibility, enabling safety-critical pre-arm checks and voltage-based failsafes.

## Success Metrics

- [ ] Battery voltage accuracy within ±0.2V compared to multimeter reading
- [ ] ADC read completes within 2ms worst-case (NFR-00060)
- [ ] Ground control station displays battery status correctly (Mission Planner verification)
- [ ] Host unit tests pass without hardware (`cargo test --lib`)
- [ ] No timing disruption to 400 Hz control loop

## Background and Current State

- Context: Freenove 4WD car kit provides battery voltage sensing on GPIO 26 with voltage divider circuit (12V → 3.3V range, coefficient \~3.95)
- Current behavior: `src/state/battery.rs` BatteryState struct uses hardcoded 12.0V placeholder; SYS_STATUS message sends placeholder voltage
- Pain points: Pre-arm checks cannot validate battery health; failsafes cannot trigger on low voltage; ground control stations show incorrect battery status
- Constraints:
  - RP2350 12-bit ADC (0-4095 counts)
  - No current sensor or per-cell voltage monitoring
  - Must integrate with 400 Hz control loop without timing impact
  - Host testing requires platform abstraction
- Related ADRs: [ADR-00017-battery-telemetry-architecture](../../adr/ADR-00017-battery-telemetry-architecture.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                        Main Loop (400 Hz)                        │
└─────────────────────────────────────────────────────────────────┘
                               │
                               │ 10 Hz Timer
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    SystemState::update_battery()                 │
└─────────────────────────────────────────────────────────────────┘
                               │
                               │ Board::read_battery_adc()
                               ▼
┌──────────────────────┐  ┌────────────────────────────────────┐
│  Pico2WBoard         │  │  MockBoard (host tests)            │
│  - embassy-rp Adc    │  │  - Configurable test values        │
│  - GPIO 26 (ADC0)    │  │                                    │
│  - 5-sample avg      │  │                                    │
└──────────────────────┘  └────────────────────────────────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │  BatteryState        │
                    │  - voltage_from_adc()│
                    │  - BATT_VOLT_MULT    │
                    └──────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│              Telemetry Handler (2 Hz streaming)                  │
│  - build_battery_status()                                        │
│  - MAVLink BATTERY_STATUS (#147)                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Components

1. **Board Trait ADC Method** (`src/platform/board.rs`)
   - Add `read_battery_adc() -> u16` to Board trait
   - Pico2WBoard implements via embassy-rp Adc driver
   - MockBoard provides configurable test values

2. **Battery Parameter** (`src/parameters/battery.rs`)
   - Add `BATT_VOLT_MULT: f32` (default 3.95)
   - ArduPilot-standard parameter for voltage divider coefficient

3. **Voltage Conversion** (`src/state/battery.rs`)
   - Add `BatteryState::voltage_from_adc(adc: u16, mult: f32) -> f32`
   - Formula: `(adc / 4095.0 * 3.3) * mult`
   - Update `SystemState::update_battery()` to call Board trait

4. **BATTERY_STATUS Message** (`src/communication/mavlink/handlers/telemetry.rs`)
   - Create `build_battery_status(state: &SystemState) -> MavMessage`
   - Populate fields per ADR implementation notes
   - Stream at 2 Hz in telemetry handler

5. **Scheduler Integration** (example files like `examples/scheduler_demo.rs`)
   - Add 10 Hz battery update ticker
   - Call `system_state.update_battery(&board).await`
   - Ensure update runs before pre-arm checks

### Data Flow

1. Main loop 10 Hz timer triggers
2. `SystemState::update_battery(&board)` calls `board.read_battery_adc()`
3. Pico2WBoard reads GPIO 26 via embassy-rp Adc (5-sample average)
4. `BatteryState::voltage_from_adc()` converts ADC value to voltage using `BATT_VOLT_MULT`
5. Voltage stored in `system_state.battery.voltage`
6. Telemetry handler reads `system_state.battery.voltage` at 2 Hz
7. `build_battery_status()` creates MAVLink BATTERY_STATUS message
8. Message sent to ground control station via MAVLink connection

### Data Models and Types

```rust
// src/platform/board.rs
pub trait Board {
    // ... existing methods ...
    fn read_battery_adc(&mut self) -> u16;
}

// src/parameters/battery.rs
pub struct BatteryParams {
    // ... existing fields ...
    pub volt_mult: f32, // BATT_VOLT_MULT
}

// src/state/battery.rs
impl BatteryState {
    pub fn voltage_from_adc(adc: u16, mult: f32) -> f32 {
        (adc as f32 / 4095.0 * 3.3) * mult
    }
}

// src/communication/mavlink/handlers/telemetry.rs
pub fn build_battery_status(state: &SystemState) -> MavMessage {
    // MAVLink BATTERY_STATUS message construction
}
```

### Error Handling

- ADC read timeout (>2ms): Log error and use last known value
- ADC driver initialization failure: Log warning but do not crash
- Invalid voltage range (>13.0V): Log warning for hardware verification
- English error messages using `log_error!()` macro
- Graceful degradation: Missing ADC hardware continues with last value

### Security Considerations

Not applicable - battery monitoring is a read-only sensor operation with no network/filesystem access.

### Performance Considerations

- **ADC Sampling**: 5-sample averaging per Freenove reference (\~500 µs estimated)
- **Update Rate**: 10 Hz battery state update (100ms period, 2ms budget per NFR-00060)
- **Telemetry Rate**: 2 Hz BATTERY_STATUS streaming (\~41 bytes/message)
- **Async Design**: embassy-rp Adc uses async API, integrates with main loop executor
- **No Blocking**: ADC reads are non-blocking, will not stall control loop
- **Memory**: BatteryState size unchanged, BATTERY_STATUS allocated on telemetry stack

### Platform Considerations

#### RP2350/RP2040 (Pico 2 W)

- 12-bit ADC resolution (0-4095 counts)
- GPIO 26-29 support ADC channels
- ADC0 channel (GPIO 26) used for battery voltage
- embassy-rp HAL provides async Adc driver
- Voltage divider circuit: 12V battery → 3.3V ADC range

#### Host Tests

- MockBoard with configurable ADC values via `set_adc_value(u16)`
- `#[cfg(test)]` conditional compilation
- tokio::test for async runtime
- No hardware access required for unit tests

#### Cross-Platform

- Battery monitoring logic platform-agnostic
- Only ADC driver implementation is embedded-specific
- Board trait abstracts hardware access

## Alternatives Considered

1. **SYS_STATUS Only (No BATTERY_STATUS)**
   - Pros: Minimal code change (50-100 LOC), existing SYS_STATUS message
   - Cons: Not ArduPilot standard, limited GCS display features
   - Decision: Rejected - BATTERY_STATUS provides full ArduPilot compatibility

2. **Custom Parameter Naming (BATT_ADC_COEFF vs BATT_VOLT_MULT)**
   - Pros: Clearer custom naming for project-specific hardware
   - Cons: Violates CLAUDE.md ArduPilot parameter standards, user confusion
   - Decision: Rejected - Use ArduPilot standard `BATT_VOLT_MULT`

3. **Direct Register Manipulation for ADC**
   - Pros: Smaller binary size, no HAL dependency
   - Cons: Error-prone, maintenance burden, no async support
   - Decision: Rejected - embassy-rp HAL is proven and maintained

## Migration and Compatibility

- Backward compatibility: SYS_STATUS message continues to include battery voltage
- New parameter: `BATT_VOLT_MULT` defaults to 3.95 (Freenove reference value)
- User calibration: Users can adjust `BATT_VOLT_MULT` via Mission Planner for hardware variations
- No API breakage: Board trait extends with new method, existing code unaffected

## Testing Strategy

### Unit Tests

- `BatteryState::voltage_from_adc()` conversion accuracy
- MockBoard ADC value configuration
- Edge cases: ADC 0, ADC max (4095), boundary voltages

### Integration Tests

Not applicable - embedded target does not support full integration test suite. Host unit tests validate logic; hardware verification via Mission Planner.

### External API Parsing (if applicable)

Not applicable - MAVLink message structure is defined by mavlink crate bindings.

### Performance & Benchmarks (if applicable)

- Manual timing verification: Log ADC read duration with defmt timestamps
- Verify <2ms worst-case per NFR-00060
- Monitor main loop timing to ensure no disruption to 400 Hz control

## Documentation Impact

- Update example files (`examples/scheduler_demo.rs`, `examples/pico_trail_rover.rs`) with battery update integration
- Add hardware setup notes to project README (GPIO 26 connection, voltage divider)
- Document `BATT_VOLT_MULT` calibration procedure for users

## External References

- [ArduPilot BATT_VOLT_MULT Parameter](https://ardupilot.org/copter/docs/parameters.html#batt-volt-mult-voltage-multiplier)
- [MAVLink BATTERY_STATUS Message](https://mavlink.io/en/messages/common.html#BATTERY_STATUS)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [Embassy-RP ADC Documentation](https://docs.embassy.dev/embassy-rp/)

## Open Questions

- [ ] Should BATTERY_STATUS be configurable via SR0_EXTRA1 parameter or hardcoded at 2 Hz? → Next step: Hardcode 2 Hz initially, add SR0_EXTRA1 in future task
- [ ] Should we implement voltage-based battery remaining percentage estimation? → Next step: Research ArduPilot's LiPo voltage curve algorithm in separate analysis

## Appendix

### Examples

```rust
// Voltage conversion example
let adc_value = 3000_u16;
let volt_mult = 3.95_f32;
let voltage = BatteryState::voltage_from_adc(adc_value, volt_mult);
assert!((voltage - 9.52).abs() < 0.01);
```

```rust
// Scheduler integration
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
