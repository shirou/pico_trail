# AN-n8yb7 Battery Telemetry | MAVLink Battery Status Reporting

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses: None
- Related Requirements:
  - [FR-015k2-adc-battery-voltage-reading](../requirements/FR-015k2-adc-battery-voltage-reading.md)
  - [FR-uq6as-voltage-conversion-calculation](../requirements/FR-uq6as-voltage-conversion-calculation.md)
  - [FR-zxxlp-battery-status-telemetry](../requirements/FR-zxxlp-battery-status-telemetry.md)
  - [FR-ygjkj-battery-state-scheduler-update](../requirements/FR-ygjkj-battery-state-scheduler-update.md)
  - [NFR-r1h41-adc-performance-constraint](../requirements/NFR-r1h41-adc-performance-constraint.md)
  - [NFR-j17oa-calibration-persistence](../requirements/NFR-j17oa-calibration-persistence.md)
  - [NFR-tximn-host-test-support](../requirements/NFR-tximn-host-test-support.md)
- Related ADRs:
  - [ADR-97ebh-battery-telemetry-architecture](../adr/ADR-97ebh-battery-telemetry-architecture.md)
- Related Tasks: (To be created)

## Executive Summary

This analysis investigates implementing battery voltage telemetry for the pico_trail rover platform. The Freenove 4WD car kit includes battery voltage sensing hardware (ADC on GPIO 26), but the current implementation only partially utilizes this capability. This document examines how to integrate hardware-level battery monitoring with MAVLink telemetry streams following ArduPilot standards, enabling ground control stations to display real-time battery status and trigger voltage-based failsafes.

Key findings: The project already has foundational battery infrastructure (BatteryState, BatteryParams, SYS_STATUS telemetry) but lacks ADC reading implementation and BATTERY_STATUS message support. Implementation requires ADC driver integration, voltage calibration, and periodic telemetry streaming.

## Problem Space

### Current State

**Existing Infrastructure:**

- `BatteryState` struct exists in `src/communication/mavlink/state.rs:97` with voltage, current, and remaining_percent fields
- `BatteryParams` in `src/parameters/battery.rs` defines ArduPilot-compatible parameters (BATT_ARM_VOLT, BATT_CRT_VOLT, BATT_FS_CRT_ACT)
- Battery voltage is included in SYS_STATUS (#1) telemetry at `src/communication/mavlink/handlers/telemetry.rs:237`
- Board configuration includes `battery_adc` pin definition (GPIO 26) at `src/platform/traits/board.rs:320`
- Pre-arm checks verify battery voltage threshold at `src/core/arming/checks.rs:179`

**Pain Points:**

1. **No ADC Reading**: BatteryState.voltage remains at placeholder value (12.0V) because no code reads ADC hardware
2. **Limited Telemetry**: Only SYS_STATUS message provides battery info; no BATTERY_STATUS (#147) message for detailed monitoring
3. **Missing Calibration**: No voltage divider calibration coefficient stored/retrieved from parameters
4. **No Periodic Updates**: Battery voltage not updated in scheduler loop

### Desired State

**Hardware Integration:**

- ADC reads GPIO 26 at regular intervals (10 Hz recommended per ArduPilot's medium-frequency monitoring)
- Voltage divider calibration factor loaded from parameters or hardcoded constant
- Raw ADC values (0-4095 for RP2040 12-bit ADC) converted to actual battery voltage

**MAVLink Telemetry:**

- BATTERY_STATUS (#147) message streamed at 2 Hz with voltage, current estimate, remaining capacity
- Existing SYS_STATUS message continues to include battery voltage for backward compatibility
- Ground control stations display battery status and trigger low-voltage warnings

**Safety Integration:**

- Battery voltage feeds into ArmedStateMonitor at `src/core/arming/monitoring.rs:201` for failsafe detection
- Pre-arm checks at `src/core/arming/checks.rs:183` use real voltage instead of placeholder
- BATT_CRT_VOLT parameter triggers emergency actions when voltage drops below threshold

### Gap Analysis

| Component          | Current State    | Target State             | Gap                                             |
| ------------------ | ---------------- | ------------------------ | ----------------------------------------------- |
| ADC Driver         | Not implemented  | Reads GPIO 26 at 10 Hz   | Need ADC HAL integration for RP2040             |
| Voltage Conversion | No calibration   | BATT_VOLT_MULT parameter | Need to add parameter and conversion formula    |
| Battery Update     | Placeholder only | Periodic scheduler task  | Need to add update_battery() calls in main loop |
| Telemetry          | SYS_STATUS only  | + BATTERY_STATUS (#147)  | Need to implement message handler               |
| Current Sensing    | Not available    | Estimated/placeholder    | Hardware limitation (no current sensor)         |

## Stakeholder Analysis

| Stakeholder                                    | Interest/Need                                                | Impact | Priority |
| ---------------------------------------------- | ------------------------------------------------------------ | ------ | -------- |
| Rover operators                                | Real-time battery status to avoid power loss during missions | High   | P0       |
| Ground control software (Mission Planner, QGC) | Standard MAVLink battery messages for display                | High   | P0       |
| Autonomous missions                            | Voltage-based failsafe to RTL before battery depletion       | High   | P0       |
| Developers                                     | Testable battery monitoring without physical hardware        | Medium | P1       |

## Research & Discovery

### User Feedback

From project context, this is a personal rover platform but follows ArduPilot conventions for potential community reuse. Battery monitoring is a fundamental safety feature in all ArduPilot vehicles.

### Competitive Analysis

**ArduPilot Rover Battery Monitoring:**

- Uses analog voltage sensing with configurable voltage divider ratios
- Supports multiple batteries with BATT, BATT2-BATTC parameter families
- Streams BATTERY_STATUS at 2 Hz by default (SR0_EXTRA1 parameter)
- Triggers failsafe actions (RTL, Land, Disarm) on low/critical voltage

**Reference Implementation (Freenove Example):**

Located in the reference repository under Freenove's battery level example:

```cpp
// Hardware configuration
#define PIN_BATTERY 26
#define LOW_VOLTAGE_VALUE 525  // ADC threshold
float batteryCoefficient = 3.95;

// ADC reading with averaging
int Get_Battery_Voltage_ADC(void) {
  pinMode(PIN_BATTERY, INPUT);
  int batteryADC = 0;
  for (int i = 0; i < 5; i++)
    batteryADC += analogRead(PIN_BATTERY);
  return batteryADC / 5;
}

// Voltage conversion
float Get_Battery_Voltage(void) {
  int batteryADC = Get_Battery_Voltage_ADC();
  batteryVoltage = (batteryADC / 1023.0 * 3.3) * batteryCoefficient;
  return batteryVoltage;
}
```

**Key findings:**

- 5-sample averaging reduces ADC noise
- Arduino `analogRead()` returns 0-1023 (10-bit), but RP2040/RP2350 ADC is 12-bit (0-4095)
- Voltage divider coefficient of 3.95 suggests monitoring 3S LiPo (11.1V nominal, 12.6V max)
- Freenove uses 1.023 as ADC reference normalization (Arduino-specific)

### Technical Investigation

**RP2040/RP2350 ADC Characteristics:**

- 12-bit resolution (0-4095 counts)
- 500 kSPS maximum sampling rate
- 0-3.3V input range (VREF)
- SAR (Successive Approximation Register) ADC type
- GPIO 26-29 support ADC (ADC0-ADC3) on both RP2040 and RP2350A (Pico 2 W)
  - Note: RP2350B (QFN-80 package) adds 4 more ADC channels on GPIO 40-43
- Requires `embassy-rp` ADC driver

**Project Hardware:**
This project targets RP2350 (Pico 2 W), which uses the RP2350A chip with QFN-60 package. The ADC specifications are identical to RP2040 for GPIO 26-29, so the Freenove reference code and voltage divider calculations apply directly.

**Voltage Divider Calculation:**
For a 3S LiPo (12.6V max) with 3.3V ADC range, voltage divider ratio = 12.6V / 3.3V ≈ 3.82x
Freenove's coefficient of 3.95 is reasonable given component tolerances.

**MAVLink BATTERY_STATUS (#147) Specification:**

```c
uint8_t id;                    // Battery instance (0 for first battery)
uint8_t battery_function;      // Function (0=unknown, 1=all flight systems, etc.)
uint8_t type;                  // Battery chemistry (0=unknown, 1=LiPo, etc.)
int16_t temperature;           // cdegC (INT16_MAX if unknown)
uint16_t voltages[10];         // Cell voltages in mV (UINT16_MAX if unknown)
int16_t current_battery;       // Current in cA (10mA units), -1 if unknown
int32_t current_consumed;      // Consumed charge in mAh, -1 if unknown
int32_t energy_consumed;       // Consumed energy in hJ, -1 if unknown
int8_t battery_remaining;      // Remaining percentage (0-100), -1 if unknown
```

**Existing Code Patterns:**
At `src/communication/mavlink/handlers/telemetry.rs:235`, SYS_STATUS conversion:

```rust
let voltage_battery = (state.battery.voltage * 1000.0) as u16;  // V to mV
let current_battery = (state.battery.current * 100.0) as i16;   // A to cA
```

### Data Analysis

**Battery Voltage Ranges (3S LiPo):**

- Fully charged: 12.6V (4.2V/cell)
- Nominal: 11.1V (3.7V/cell)
- Low warning: 10.5V (3.5V/cell) - BATT_ARM_VOLT default
- Critical: 10.0V (3.33V/cell) - BATT_CRT_VOLT default
- Absolute minimum: 9.0V (3.0V/cell) - damage threshold

**Recommended Update Rates:**

- ADC sampling: 10 Hz (matches ArduPilot medium-frequency monitoring)
- SYS_STATUS: 1 Hz (low-frequency telemetry)
- BATTERY_STATUS: 2 Hz (SR0_EXTRA1 default in ArduPilot)

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: System shall read battery voltage from ADC GPIO 26
  - Rationale: Hardware provides voltage sensing capability on Freenove platform
  - Acceptance Criteria:
    - ADC reads 12-bit value (0-4095) from GPIO 26
    - Implements 5-sample averaging to reduce noise per Freenove reference
    - Updates at 10 Hz minimum

- [ ] **FR-DRAFT-2**: System shall convert ADC value to battery voltage in volts
  - Rationale: Raw ADC counts must be scaled to engineering units for telemetry and safety checks
  - Acceptance Criteria:
    - Formula: `voltage = (adc_value / 4095.0 * 3.3) * coefficient`
    - Coefficient loaded from BATT_VOLT_MULT parameter (default 3.95)
    - Voltage accurate within ±0.2V compared to multimeter reading

- [ ] **FR-DRAFT-3**: System shall transmit BATTERY_STATUS (#147) MAVLink message
  - Rationale: Provides detailed battery telemetry for ground control station monitoring
  - Acceptance Criteria:
    - Streams at 2 Hz (controlled by SR0_EXTRA1 parameter)
    - Includes voltage, current estimate, remaining percentage
    - Compatible with Mission Planner and QGroundControl

- [ ] **FR-DRAFT-4**: System shall update BatteryState in main scheduler loop
  - Rationale: Arming checks and failsafes depend on current voltage reading
  - Acceptance Criteria:
    - `SystemState.update_battery()` called at 10 Hz
    - Voltage propagates to ArmedStateMonitor for failsafe evaluation
    - Pre-arm check uses actual voltage instead of placeholder

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: ADC reading shall complete within 2ms worst-case
  - Category: Performance
  - Rationale: Must not block 400 Hz scheduler interrupt
  - Target: 5 samples @ 500 kSPS = 10µs nominal, 2ms timeout

- [ ] **NFR-DRAFT-2**: Voltage calibration shall persist across reboots
  - Category: Reliability
  - Rationale: Users should not recalibrate after power cycle
  - Target: BATT_VOLT_MULT parameter stored in flash via ParameterStore

- [ ] **NFR-DRAFT-3**: Battery monitoring shall work in host tests without hardware
  - Category: Usability
  - Rationale: Developers need to test battery logic on development machines
  - Target: Mock ADC driver provides simulated voltage for unit tests

## Design Considerations

### Technical Constraints

1. **Hardware Limitations:**
   - No current sensor on Freenove platform (current must be estimated or placeholder)
   - Single ADC channel (GPIO 26) monitors total pack voltage, not individual cells
   - Voltage divider ratio fixed in hardware (cannot change calibration coefficient for different battery chemistries)

2. **Platform Constraints:**
   - Embassy-rp ADC driver required for RP2350 (same API as RP2040)
   - Must integrate with existing 400 Hz scheduler at `examples/mavlink_rc_control.rs`
   - Parameter storage limited to 256 entries in flash

3. **Safety Constraints:**
   - Battery voltage must update before pre-arm checks run
   - Failsafe actions must trigger within 100ms of critical voltage detection
   - ADC failure should not crash system (use last known good value)

### Potential Approaches

**Option A: Minimal Implementation (SYS_STATUS only)**

- Description: Add ADC reading and update BatteryState.voltage, keep SYS_STATUS telemetry
- Pros:
  - Smallest code change (50-100 LOC)
  - SYS_STATUS already supported by all GCS software
  - Meets minimum safety requirement for pre-arm checks
- Cons:
  - No per-cell voltage monitoring
  - Less detailed than ArduPilot standard
  - Mission Planner battery view shows limited info
- Effort: Low (1-2 days)

**Option B: Full BATTERY_STATUS Implementation**

- Description: Add ADC reading + implement BATTERY_STATUS message handler
- Pros:
  - Full ArduPilot compatibility
  - Ground stations display detailed battery info
  - Future-proof for current sensor addition
  - Matches competitive feature parity
- Cons:
  - Requires new message handler (150-200 LOC)
  - Must add BATTERY_STATUS to telemetry streamer
  - More testing required
- Effort: Medium (3-5 days)

**Option C: Smart Battery Protocol (Future)**

- Description: Implement SMBus communication with smart LiPo packs
- Pros:
  - Real per-cell voltages and current sensing
  - Temperature monitoring
  - Accurate state-of-charge estimation
- Cons:
  - Requires different hardware (smart battery pack)
  - SMBus driver implementation complex
  - Not compatible with Freenove platform
- Effort: High (2-3 weeks)

**Recommendation: Option B (Full BATTERY_STATUS Implementation)**

- Provides complete ArduPilot compatibility
- Effort is reasonable (one sprint)
- Enables future current sensor integration without rework
- Ground station users expect BATTERY_STATUS message

### Architecture Impact

**New ADRs Required:**

1. ADR for ADC driver selection (embassy-rp vs. direct register access)
2. ADR for battery parameter naming convention (BATT vs. BATT_VOLT_MULT vs. custom)

**Modified Components:**

- `src/platform/traits/board.rs`: Add ADC pin configuration API
- `src/communication/mavlink/state.rs`: Enhance BatteryState with conversion methods
- `src/communication/mavlink/handlers/telemetry.rs`: Add BATTERY_STATUS message builder
- `src/parameters/battery.rs`: Add BATT_VOLT_MULT parameter
- Main scheduler example: Add 10 Hz battery update task

## Risk Assessment

| Risk                                                       | Probability | Impact | Mitigation Strategy                                |
| ---------------------------------------------------------- | ----------- | ------ | -------------------------------------------------- |
| ADC noise causes voltage spikes triggering false failsafes | Medium      | High   | Implement 5-sample averaging + low-pass filter     |
| Voltage divider coefficient varies between hardware units  | High        | Medium | Make BATT_VOLT_MULT user-calibratable parameter    |
| No current sensor limits state-of-charge accuracy          | Certain     | Low    | Document limitation, use voltage-based estimation  |
| embassy-rp ADC driver not available in no_std              | Low         | High   | Verify embassy-rp ADC compiles for RP2350 thumbv8m |
| Battery parameter namespace conflicts with ArduPilot       | Medium      | Medium | Follow exact ArduPilot parameter names from docs   |

## Open Questions

- [x] What is the exact voltage divider ratio on Freenove hardware? → 3.95 coefficient from reference code
- [ ] Should we implement BATTERY2 support for dual-battery configurations? → Next step: Defer to future analysis (out of scope)
- [ ] What current estimation algorithm should we use without a sensor? → Method: Research ArduPilot's voltage-based current estimation or use fixed conservative value
- [ ] Should BATTERY_STATUS be in EXTRA1 or EXTRA2 stream rate group? → Next step: Review ArduPilot SR parameter defaults (EXTRA1 is standard)

## Recommendations

### Immediate Actions

1. **Verify embassy-rp ADC Support**: Confirm `embassy-rp::adc::Adc` driver compiles for RP2350 and supports GPIO 26 (ADC API identical to RP2040)
2. **Add BATT_VOLT_MULT Parameter**: Extend `src/parameters/battery.rs` with voltage multiplier coefficient (default 3.95)
3. **Prototype ADC Reading**: Create simple test that reads GPIO 26 and prints voltage to defmt log

### Next Steps

1. [ ] Create formal requirements: FR-<id> for ADC reading, voltage conversion, BATTERY_STATUS telemetry
2. [ ] Draft ADR for: ADC driver architecture (embassy-rp vs. alternatives)
3. [ ] Draft ADR for: Battery parameter standard (confirm BATT_VOLT_MULT matches ArduPilot)
4. [ ] Create task for: Implement battery voltage monitoring with BATTERY_STATUS message
5. [ ] Further investigation: Current estimation algorithms (voltage-based SOC, Peukert's equation)

### Out of Scope

- **Individual Cell Voltage Monitoring**: Requires hardware redesign with cell-balancing harness
- **Current Sensing**: No current sensor on Freenove platform; placeholder values acceptable
- **Smart Battery Protocol**: Different hardware platform needed
- **Dual Battery Support**: Single battery sufficient for initial implementation
- **Fuel Gauge IC Integration**: Freenove uses simple voltage divider, not dedicated battery IC

## Appendix

### References

- [ArduPilot Rover Battery Parameters](https://ardupilot.org/rover/docs/parameters.html#batt-arm-volt)
- [MAVLink Battery Protocol](https://mavlink.io/en/services/battery.html)
- [MAVLink BATTERY_STATUS Message Specification](https://mavlink.io/en/messages/common.html#BATTERY_STATUS)
- Freenove 4WD Car Battery Level Example (reference repository)
- [RP2040 Datasheet ADC Section](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- Project existing battery code:
  - `src/communication/mavlink/state.rs:97` - BatteryState struct
  - `src/parameters/battery.rs` - Battery parameters
  - `src/core/arming/checks.rs:179` - Battery voltage pre-arm check

### Raw Data

**Freenove Hardware Configuration:**

```
Battery Sensor: GPIO 26 (ADC0)
Voltage Divider: R1=10kΩ, R2=3.3kΩ (estimated from 3.95 coefficient)
ADC Reference: 3.3V (VREF)
Max Input Voltage: 12.6V (3S LiPo fully charged)
Low Voltage Threshold: ADC 525 ≈ 10.5V
```

**Existing Parameter Defaults:**

```rust
BATT_ARM_VOLT: 10.5V  // Minimum arming voltage
BATT_CRT_VOLT: 10.0V  // Critical failsafe voltage
BATT_FS_CRT_ACT: Land (2)  // Failsafe action
```

**RP2040/RP2350 ADC Timing:**

```
Conversion time: ~2µs per sample
5-sample average: 10µs
DMA transfer: +5µs
Total latency: <20µs (well within 2ms budget)
Note: RP2350 uses identical ADC hardware to RP2040
```
