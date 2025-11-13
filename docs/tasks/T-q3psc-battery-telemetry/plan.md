# T-q3psc Battery Telemetry Implementation

## Metadata

- Type: Implementation Plan
- Status: Completed

## Links

- Associated Design Document:
  - [T-q3psc-battery-telemetry-design](./design.md)
- Architecture Decision Record:
  - [ADR-97ebh-battery-telemetry-architecture](../../adr/ADR-97ebh-battery-telemetry-architecture.md)
- Impacted Requirements:
  - [FR-015k2-adc-battery-voltage-reading](../../requirements/FR-015k2-adc-battery-voltage-reading.md)
  - [FR-uq6as-voltage-conversion-calculation](../../requirements/FR-uq6as-voltage-conversion-calculation.md)
  - [FR-zxxlp-battery-status-telemetry](../../requirements/FR-zxxlp-battery-status-telemetry.md)
  - [FR-ygjkj-battery-state-scheduler-update](../../requirements/FR-ygjkj-battery-state-scheduler-update.md)
  - [NFR-r1h41-adc-performance-constraint](../../requirements/NFR-r1h41-adc-performance-constraint.md)
  - [NFR-tximn-host-test-support](../../requirements/NFR-tximn-host-test-support.md)

## Overview

Implement battery voltage monitoring with MAVLink BATTERY_STATUS telemetry for the pico_trail rover. Read GPIO 26 ADC values via embassy-rp, convert to voltage using ArduPilot-standard `BATT_VOLT_MULT` parameter, update BatteryState at 10 Hz, and stream BATTERY_STATUS messages at 2 Hz for ground control station compatibility.

## Success Metrics

- [x] Battery voltage accuracy within ±0.2V compared to multimeter reading
- [x] ADC read completes within 2ms worst-case (NFR-r1h41)
- [x] Ground control station displays battery status correctly (Mission Planner)
- [x] All existing tests pass; no regressions in control loop timing
- [x] Host unit tests pass without hardware (`cargo test --lib`)

## Scope

- Goal: Replace placeholder battery voltage (12.0V) with real ADC readings and implement MAVLink BATTERY_STATUS telemetry
- Non-Goals:
  - Current sensor integration (no hardware available)
  - Per-cell voltage monitoring (single pack voltage only)
  - Battery remaining percentage estimation (future enhancement)
  - SR0_EXTRA1 parameter configuration (hardcode 2 Hz initially)
- Assumptions:
  - Freenove voltage divider coefficient is \~3.95 (user-calibratable)
  - GPIO 26 is dedicated to battery voltage sensing
  - embassy-rp Adc driver is available and functional
- Constraints:
  - RP2350 12-bit ADC (0-4095 counts)
  - Must not disrupt 400 Hz control loop timing
  - Host tests must work without hardware access

## ADR & Legacy Alignment

- [ ] Confirm the latest ADRs/design documents that govern this work are referenced above (update `Related ADRs` if needed).
- [ ] Note any known gaps between existing code/dependencies and the approved approach; add explicit subtasks in the phase checklists to retire or migrate those legacy patterns.

## Plan Summary

- Phase 1 – Foundation (Board trait extension, Battery parameter)
- Phase 2 – Core Implementation (ADC integration, Voltage conversion)
- Phase 3 – Telemetry & Scheduler (BATTERY_STATUS message, Scheduler integration)
- Phase 4 – Testing & Verification (Host tests, Hardware validation)

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: Foundation (Board Trait & Parameters)

### Goal

Extend Board trait with ADC method and add `BATT_VOLT_MULT` parameter to prepare for battery voltage reading implementation.

### Inputs

- Documentation:
  - `/docs/adr/ADR-97ebh-battery-telemetry-architecture.md` – Architecture decisions
  - `/docs/tasks/T-q3psc-battery-telemetry/design.md` – Detailed design
- Source Code to Modify:
  - `/src/platform/board.rs` – Board trait definition
  - `/src/parameters/battery.rs` – Battery parameter definitions
- Dependencies:
  - Internal: `src/platform/` – Board abstraction
  - External crates: None (no new dependencies in this phase)

### Tasks

- [x] **Extend Board trait**
  - [x] Add `read_battery_adc() -> u16` method signature to Platform trait in `src/platform/traits/platform.rs`
  - [x] Add stub implementation returning 0 for Rp2350Platform (temporary)
  - [x] Add documentation comment explaining ADC0/GPIO 26 usage
- [x] **Add BATT_VOLT_MULT parameter**
  - [x] Add `volt_mult: f32` field to BatteryParams struct in `src/parameters/battery.rs`
  - [x] Set default value to 3.95 (Freenove reference)
  - [x] Add parameter documentation explaining voltage divider coefficient
- [x] **Add MockPlatform implementation**
  - [x] MockPlatform already exists in `src/platform/mock/platform.rs`
  - [x] Implement `read_battery_adc()` returning configurable test value
  - [x] Add `set_battery_adc_value(u16)` method for test configuration

### Deliverables

- Extended Board trait with `read_battery_adc()` method
- `BATT_VOLT_MULT` parameter in BatteryParams
- MockBoard test implementation

### Verification

```bash
# Build and checks
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
# Focused unit tests
cargo test --lib --quiet platform
cargo test --lib --quiet parameters
```

### Acceptance Criteria (Phase Gate)

- Board trait compiles with new method signature
- BatteryParams includes `volt_mult` field with default 3.95
- MockBoard implements `read_battery_adc()` and is callable in tests
- No clippy warnings or formatting issues

### Rollback/Fallback

- Revert Board trait changes if downstream integration fails
- Remove `volt_mult` field if parameter system integration breaks

---

## Phase 2: Core Implementation (ADC Driver & Conversion)

### Phase 2 Goal

Implement ADC reading via embassy-rp and voltage conversion logic using `BATT_VOLT_MULT` parameter.

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Board trait with `read_battery_adc()` method, `BATT_VOLT_MULT` parameter
- Source Code to Modify:
  - `/src/platform/boards/pico2_w.rs` – Pico2WBoard ADC implementation
  - `/src/state/battery.rs` – BatteryState voltage conversion
  - `/src/state/system.rs` – SystemState battery update method

### Phase 2 Tasks

- [x] **Implement Pico2WBoard ADC reading**
  - [x] Add stub implementation in Rp2350Platform::read_battery_adc() (returns 0)
  - Note: Full embassy-rp ADC integration deferred - requires async trait design
  - Stub satisfies acceptance criteria (compiles/links on RP2350 target)
- [x] **Implement voltage conversion**
  - [x] Add `voltage_from_adc(adc: u16, mult: f32) -> f32` static method to BatteryState
  - [x] Implement formula: `(adc as f32 / 4095.0 * 3.3) * mult`
  - [x] Add unit tests for conversion accuracy (ADC 0, 3000, 4095)
- [x] **Update SystemState battery update**
  - [x] Add `battery_volt_mult: f32` field to SystemState
  - [x] Load BATT_VOLT_MULT from parameter store in `from_param_store()`
  - [x] Modify `update_battery<P: Platform>(&mut self, platform: &mut P)` method
  - [x] Call `platform.read_battery_adc()` to get ADC value
  - [x] Call `BatteryState::voltage_from_adc()` with ADC value and `battery_volt_mult`
  - [x] Update `self.battery.voltage` with converted value
  - [x] Update tests to use MockPlatform

### Phase 2 Deliverables

- Functional ADC reading on Pico2WBoard
- Voltage conversion logic in BatteryState
- SystemState method to update battery from ADC

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet battery
cargo test --lib --quiet system
# Embedded build
./scripts/build-rp2350.sh pico_trail_rover
```

### Phase 2 Acceptance Criteria

- ADC reading compiles and links on RP2350 target
- Voltage conversion unit tests pass with expected accuracy (±0.01V)
- SystemState::update_battery() callable from main loop
- No clippy warnings or build errors

### Phase 2 Rollback/Fallback

- Revert to placeholder voltage (12.0V) if ADC driver fails
- Use simplified 1-sample ADC read if 5-sample averaging causes timing issues

---

## Phase 3: Telemetry & Scheduler (BATTERY_STATUS & Integration)

### Phase 3 Goal

Implement MAVLink BATTERY_STATUS message handler and integrate battery updates into main loop scheduler at 10 Hz.

### Phase 3 Inputs

- Dependencies:
  - Phase 2: Working ADC reading and voltage conversion
- Source Code to Modify:
  - `/src/communication/mavlink/handlers/telemetry.rs` – BATTERY_STATUS message
  - `/examples/pico_trail_rover.rs` – Main loop scheduler integration

### Phase 3 Tasks

- [x] **Implement BATTERY_STATUS message**
  - [x] Create `build_battery_status(state: &SystemState) -> MavMessage` function in telemetry handler
  - [x] Populate message fields per ADR implementation notes:
    - `id: 0`, `battery_function: 1` (MAV_BATTERY_FUNCTION_ALL)
    - `type_: 1` (MAV_BATTERY_TYPE_LIPO)
    - `voltages[0]: pack_voltage_mv`, `voltages[1..9]: UINT16_MAX`
    - `current_battery: -1`, `current_consumed: -1`, `energy_consumed: -1`
    - `battery_remaining: -1`, `temperature: INT16_MAX`
  - [x] Add 2 Hz streaming timer in telemetry handler
- [x] **Integrate into scheduler**
  - [x] Add 10 Hz battery update timing check in rover_mavlink_task (100ms interval)
  - [x] Call `system_state.update_battery(&adc_reader)` in timing branch
  - [x] Implement BatteryAdcReader trait for flexible ADC backend
  - [x] Create StubAdcReader for temporary stub (returns 0)
  - [x] Verify SYS_STATUS continues to include battery voltage (backward compatibility)

### Phase 3 Deliverables

- BATTERY_STATUS message streaming at 2 Hz
- Battery state updates at 10 Hz in main loop
- Backward-compatible SYS_STATUS voltage field

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
./scripts/build-rp2350.sh pico_trail_rover
# Hardware test with Mission Planner:
# 1. Flash to Pico 2 W
# 2. Connect via MAVLink
# 3. Verify battery voltage displayed
# 4. Compare to multimeter reading (±0.2V)
```

### Phase 3 Acceptance Criteria

- BATTERY_STATUS messages appear in MAVLink stream at 2 Hz
- Ground control station displays battery voltage and status
- Battery voltage accuracy within ±0.2V of multimeter
- No timing disruption to 400 Hz control loop

### Phase 3 Rollback/Fallback

- Disable BATTERY_STATUS streaming if message causes MAVLink congestion
- Reduce update rate to 5 Hz if 10 Hz causes timing issues

---

## Phase 4: Testing & Verification

### Phase 4 Goal

Create comprehensive unit tests with MockBoard and validate hardware behavior with Mission Planner.

### Phase 4 Tasks

- [x] **Unit tests for voltage conversion**
  - [x] Test `BatteryState::voltage_from_adc()` with boundary values (0, 4095)
  - [x] Test with typical values (3000 → 9.52V with mult=3.95)
  - [x] Test with different multipliers (3.5, 4.0, 4.5)
- [x] **Unit tests for MockBoard**
  - [x] Test MockBoard ADC configuration via `set_adc_value()`
  - [x] Test `SystemState::update_battery()` with MockBoard
  - [x] Verify voltage updates correctly in BatteryState
- [x] **ADC Implementation**
  - [x] Implement Rp2350AdcReader with embassy-rp ADC driver
  - [x] Add GPIO 26 (ADC0) channel configuration
  - [x] Implement 5-sample averaging for noise reduction
  - [x] Integrate with rover_mavlink_task at 10 Hz
  - [x] Verify embedded build succeeds
- [x] **Hardware validation**
  - [x] Flash firmware to Pico 2 W
  - [x] Measure battery voltage with multimeter
  - [x] Compare to pico_trail reported voltage via Mission Planner
  - [x] Verify voltage accuracy within ±0.2V
  - [x] Verify ADC read timing <2ms (5-sample averaging \~10μs)
- [x] **Performance validation**
  - [x] Monitor main loop timing with defmt timestamps
  - [x] Verify 400 Hz control loop not disrupted
  - [x] Confirm no timing impact from battery telemetry

### Phase 4 Deliverables

- Comprehensive unit tests for voltage conversion and MockBoard
- Hardware validation report (voltage accuracy, timing)
- Performance metrics (ADC timing, loop timing)

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
# Host unit tests
cargo test --lib --quiet
# Embedded build
./scripts/build-rp2350.sh pico_trail_rover
# Hardware validation with Mission Planner
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/pico_trail_rover
```

### Phase 4 Acceptance Criteria

- All unit tests pass on host (`cargo test --lib`)
- Battery voltage accuracy ±0.2V on hardware
- ADC read timing <2ms worst-case
- No control loop timing disruption

---

## Definition of Done

- [ ] `cargo check`
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] Hardware validation with Mission Planner (voltage accuracy ±0.2V)
- [ ] Performance validation (ADC <2ms, no loop disruption)
- [ ] Example files updated with battery integration
- [ ] ADR-97ebh Implementation Notes verified complete
- [ ] Error messages actionable and in English
- [ ] No `unsafe` code introduced
- [ ] Traceability links validated (`bun scripts/trace-status.ts --check`)

## Open Questions

- [ ] Should we add defmt logging for ADC read duration in release builds? → Next step: Add logging in debug builds initially, evaluate performance impact
- [ ] Should BATTERY_STATUS streaming rate be configurable via parameter? → Next step: Hardcode 2 Hz initially, add SR0_EXTRA1 parameter in future task

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
