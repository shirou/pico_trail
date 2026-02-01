# FR-00067 | Scheduler BatteryState Update

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-00019-battery-telemetry](../analysis/AN-00019-battery-telemetry.md)
- Prerequisite Requirements:
  - [FR-00066-adc-battery-voltage-reading](../requirements/FR-00066-adc-battery-voltage-reading.md)
  - [FR-00069-voltage-conversion-calculation](../requirements/FR-00069-voltage-conversion-calculation.md)
- Dependent Requirements:
  - [NFR-00062-host-test-support](../requirements/NFR-00062-host-test-support.md)
- Related Tasks:
  - [T-00013-battery-telemetry](../tasks/T-00013-battery-telemetry/README.md)

## Requirement Statement

The system shall periodically update BatteryState with current voltage readings in the main scheduler loop at 10 Hz frequency, ensuring battery data is fresh for arming checks and failsafe monitoring.

## Rationale

Battery voltage monitoring is a safety-critical function. Pre-arm checks (`src/core/arming/checks.rs:179`) and armed state monitoring (`src/core/arming/monitoring.rs:201`) depend on current voltage readings from `BatteryState`. Without periodic updates in the scheduler loop, these safety systems would operate on stale placeholder values (12.0V), defeating the purpose of battery monitoring. The 10 Hz update rate matches ArduPilot's medium-frequency monitoring convention.

## User Story (if applicable)

As a rover control system, I want battery voltage to update continuously in the scheduler loop, so that arming checks and failsafes respond to actual battery conditions within 100ms.

## Acceptance Criteria

- [ ] `SystemState.update_battery()` or equivalent function called in main scheduler loop
- [ ] Battery update executes at 10 Hz frequency (every 100ms)
- [ ] Update reads ADC via FR-00066 and converts via FR-00069
- [ ] Updated voltage propagates to `BatteryState.voltage` field
- [ ] ArmedStateMonitor receives updated voltage for failsafe evaluation
- [ ] Pre-arm battery check at `src/core/arming/checks.rs:179` uses current voltage, not placeholder
- [ ] Battery update completes within time budget (see NFR-00060)
- [ ] Scheduler integration does not disrupt existing 400 Hz control loop timing

## Technical Details

### Functional Requirement Details

**Scheduler Integration:**

The main scheduler loop (currently at `examples/mavlink_rc_control.rs`) operates at 400 Hz (2.5ms period). Battery updates at 10 Hz represent every 40th iteration.

**Update Flow:**

1. Main scheduler loop detects 100ms elapsed (10 Hz timer)
2. Calls battery update function (e.g., `system_state.update_battery()`)
3. Update function:
   - Reads ADC via board-specific driver (FR-00066)
   - Converts ADC to voltage using `BATT_VOLT_MULT` parameter (FR-00069)
   - Updates `BatteryState.voltage` field
4. Updated `BatteryState` propagates to:
   - Pre-arm checks (if vehicle disarmed)
   - ArmedStateMonitor (if vehicle armed)
   - Telemetry handlers (SYS_STATUS, BATTERY_STATUS)

**Example Scheduler Pattern:**

```rust
// In main scheduler loop (simplified)
let mut last_battery_update = Instant::now();
const BATTERY_UPDATE_INTERVAL: Duration = Duration::from_millis(100); // 10 Hz

loop {
    // 400 Hz control loop
    // ... RC processing, motor control, etc. ...

    // 10 Hz battery update
    if last_battery_update.elapsed() >= BATTERY_UPDATE_INTERVAL {
        system_state.update_battery(&board).await;
        last_battery_update = Instant::now();
    }

    // ... telemetry, arming checks, etc. ...
}
```

**SystemState.update_battery() Implementation:**

This method should be added to `SystemState` in `src/communication/mavlink/state.rs`:

```rust
impl SystemState {
    pub async fn update_battery(&mut self, board: &impl Board) {
        let adc_value = board.read_battery_adc().await;
        let volt_mult = self.parameters.battery.batt_volt_mult; // From parameters
        self.battery.voltage = BatteryState::voltage_from_adc(adc_value, volt_mult);
        // Future: Update battery.current, battery.remaining_percent
    }
}
```

**Integration Points:**

- Scheduler loop: `examples/mavlink_rc_control.rs` (requires modification)
- SystemState: `src/communication/mavlink/state.rs` (add update_battery method)
- Board trait: `src/platform/traits/board.rs` (add read_battery_adc method)
- Pre-arm check: `src/core/arming/checks.rs:179` (already reads BatteryState.voltage)
- Armed monitor: `src/core/arming/monitoring.rs:201` (already monitors BatteryState)

**Timing Requirements:**

- Battery update must complete within 2ms (NFR-00060)
- Update must not delay 400 Hz control loop (2.5ms deadline)
- ADC read (10µs) + conversion (<1µs) + state update (<1µs) << 2ms ✓

**Error Handling:**

- If ADC read fails, use last known good voltage value
- Log warning but do not crash scheduler
- Mark battery status as invalid in telemetry if repeated failures

## Platform Considerations

### Unix

N/A – Platform agnostic (embedded-only feature)

### Windows

N/A – Platform agnostic (embedded-only feature)

### Cross-Platform

Scheduler integration logic is platform-agnostic. Host tests shall mock battery updates with simulated voltage values (see NFR-00062).

## Risks & Mitigation

| Risk                                                  | Impact | Likelihood | Mitigation                                                 | Validation                                        |
| ----------------------------------------------------- | ------ | ---------- | ---------------------------------------------------------- | ------------------------------------------------- |
| Battery update blocks control loop causing jitter     | High   | Low        | Ensure ADC read is async and completes within 2ms budget   | Measure loop timing with defmt timestamps         |
| Scheduler integration breaks existing functionality   | High   | Medium     | Add battery update to separate scheduler task/timer        | Test all existing functionality after integration |
| Battery voltage not updated before pre-arm check runs | High   | Low        | Ensure update runs before arming check in scheduler order  | Unit test scheduler execution order               |
| 10 Hz update rate too slow for fast voltage drops     | Medium | Low        | 10 Hz = 100ms response time; adequate for battery dynamics | Test with voltage drop scenarios                  |
| SystemState.update_battery() not implemented yet      | High   | Low        | Create method as part of FR implementation                 | Code review confirms method exists                |

## Implementation Notes

**Preferred Approaches:**

- Add `update_battery()` method to `SystemState` in `src/communication/mavlink/state.rs`
- Add `read_battery_adc() -> u16` method to `Board` trait in `src/platform/traits/board.rs`
- Use embassy-time `Instant` and `Duration` for 10 Hz timing
- Integrate into existing scheduler loop with minimal disruption
- Example integration:
  ```rust
  // In main loop
  let mut battery_ticker = Ticker::every(Duration::from_hz(10));
  loop {
      select {
          _ = battery_ticker.next() => {
              system_state.update_battery(&board).await;
          }
          // ... other scheduler events ...
      }
  }
  ```

**Known Pitfalls:**

- Do not block 400 Hz loop waiting for 10 Hz battery update; use async select or polling
- Battery update must run before arming checks, not after
- Do not create separate task for battery; keep in main loop for timing predictability
- Embassy async requires proper executor context; ensure scheduler supports async calls

**Related Code Areas:**

- `examples/mavlink_rc_control.rs` - Main scheduler loop (requires modification)
- `src/communication/mavlink/state.rs` - SystemState and BatteryState structs
- `src/platform/traits/board.rs` - Board trait for hardware abstraction
- `src/core/arming/checks.rs:179` - Pre-arm battery voltage check
- `src/core/arming/monitoring.rs:201` - Armed state battery monitoring

**Testing Strategy:**

- Unit test: Verify `update_battery()` correctly reads ADC and converts voltage
- Integration test: Confirm battery updates propagate to arming checks
- Hardware test: Verify 10 Hz update rate using defmt timing logs
- Stress test: Ensure battery update does not disrupt 400 Hz control loop timing

## External References

- [ArduPilot Scheduler Documentation](https://ardupilot.org/dev/docs/scheduling-your-new-code-to-run-intermittently.html) - Scheduler best practices
- Embassy Executor Documentation - Async task scheduling patterns | N/A
