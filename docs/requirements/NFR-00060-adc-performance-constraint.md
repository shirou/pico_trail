# NFR-00060 | ADC Performance Constraint

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-00019-battery-telemetry](../analysis/AN-00019-battery-telemetry.md)
- Prerequisite Requirements:
  - [FR-00066-adc-battery-voltage-reading](../requirements/FR-00066-adc-battery-voltage-reading.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00013-battery-telemetry](../tasks/T-00013-battery-telemetry/README.md)

## Requirement Statement

ADC battery voltage reading shall complete within 2ms worst-case execution time to ensure it does not block the 400 Hz scheduler interrupt.

## Rationale

The main scheduler loop operates at 400 Hz (2.5ms period) for motor control and RC input processing. Battery voltage updates at 10 Hz run within this loop. If ADC reading takes too long, it will delay motor commands and introduce control jitter, degrading vehicle responsiveness and stability. A 2ms timeout provides ample margin: the RP2040/RP2350 ADC can sample 5 readings in \~10µs, leaving 2ms budget sufficient even with interrupt latency and task switching overhead.

## User Story (if applicable)

The system shall complete ADC battery voltage reading within 2ms to ensure motor control timing is not disrupted by battery monitoring operations.

## Acceptance Criteria

- [ ] ADC read operation (5 samples + averaging) completes within 2ms in all cases
- [ ] Timing verified using defmt timestamps in hardware tests
- [ ] Battery voltage update does not cause 400 Hz control loop to miss deadlines
- [ ] Timeout mechanism implemented: if ADC read exceeds 2ms, log error and use last known value
- [ ] Performance validated under worst-case conditions (maximum interrupt load, cache misses)

## Technical Details

### Non-Functional Requirement Details

- **Performance**: ADC read latency target
  - **Target**: 2ms worst-case execution time
  - **Nominal**: \~10µs (5 samples @ 2µs per sample)
  - **Margin**: 200x safety factor to account for system overhead
  - **Measurement method**: defmt::timestamp() before/after ADC read

- **Reliability**: Fallback behavior
  - If ADC read fails or times out, use last known good voltage value
  - Log warning via defmt for debugging
  - Do not crash scheduler or halt system

- **Compatibility**: Platform-specific constraints
  - RP2040/RP2350 ADC specifications:
    - Conversion time: \~2µs per sample (500 kSPS rate)
    - 5-sample average: 10µs nominal
    - DMA transfer overhead: +5µs
    - Total nominal: <20µs
  - Embassy-rp async ADC API: `adc.read(&mut channel).await`
    - Async overhead: task switch + executor latency (\~1-10µs)
    - Total with async: <100µs expected

**Why 2ms Budget?**

The 400 Hz control loop has a 2.5ms period. Battery updates at 10 Hz occur once every 40 control iterations. To avoid disrupting control timing:

- Battery update must not delay control loop beyond 2.5ms deadline
- Allocate 2ms for ADC read (generous margin for 10µs operation)
- Reserve 0.5ms for motor control, RC processing, and telemetry
- If ADC read exceeds 2ms, something is seriously wrong (hardware failure, runaway loop)

**Performance Testing:**

Measure actual timing on hardware using defmt:

```rust
let start = embassy_time::Instant::now();
let adc_value = board.read_battery_adc().await;
let elapsed = start.elapsed();
defmt::debug!("Battery ADC read: {}µs", elapsed.as_micros());
if elapsed.as_millis() > 2 {
    defmt::error!("ADC read timeout: {}ms", elapsed.as_millis());
}
```

Expected results:

- Nominal: 10-50µs
- Worst-case: <100µs
- Timeout threshold: 2000µs (2ms)

## Platform Considerations

### Unix

N/A – Platform agnostic (embedded-only feature)

### Windows

N/A – Platform agnostic (embedded-only feature)

### Cross-Platform

This requirement applies only to embedded targets (RP2040/RP2350). Host tests do not require real-time constraints.

## Risks & Mitigation

| Risk                                                     | Impact | Likelihood | Mitigation                                             | Validation                           |
| -------------------------------------------------------- | ------ | ---------- | ------------------------------------------------------ | ------------------------------------ |
| ADC read blocks longer than expected due to DMA conflict | Medium | Low        | Use embassy-rp async API; measure actual timing        | Hardware test with defmt timing logs |
| Embassy async task switch introduces unpredictable delay | Medium | Low        | 2ms budget provides 200x margin over nominal 10µs      | Stress test with high interrupt load |
| ADC hardware fault causes infinite loop                  | High   | Very Low   | Implement 2ms timeout; use last known value on failure | Unit test timeout handling           |
| Compiler optimization changes timing characteristics     | Low    | Low        | Release build testing required; do not rely on debug   | Test both debug and release builds   |

## Implementation Notes

**Preferred Approaches:**

- Use embassy-rp async ADC API for non-blocking reads
- Measure timing with `embassy_time::Instant` and log via defmt
- Implement timeout using `embassy_futures::select` with timer:

  ```rust
  use embassy_futures::select;
  use embassy_time::{Duration, Timer};

  pub async fn read_battery_adc_with_timeout(&mut self) -> Result<u16, AdcError> {
      let adc_future = self.read_battery_adc();
      let timeout_future = Timer::after(Duration::from_millis(2));

      match select(adc_future, timeout_future).await {
          Either::First(adc_value) => Ok(adc_value),
          Either::Second(_) => {
              defmt::error!("ADC read timeout (>2ms)");
              Err(AdcError::Timeout)
          }
      }
  }
  ```

**Known Pitfalls:**

- Do not use blocking ADC reads; embassy-rp provides async API
- Debug builds may have different timing than release; test both
- defmt logging itself adds overhead; ensure timing measurement excludes log statements
- ADC channel must be properly initialized before first read; verify in board initialization

**Validation Strategy:**

- Unit test: Mock ADC with known delays; verify timeout handling
- Hardware test: Measure actual ADC read timing on Pico 2 W using defmt
- Stress test: Run with maximum telemetry rate and motor commands to measure worst-case
- Release build test: Verify optimized build meets timing constraint

## External References

- [RP2040 Datasheet - ADC Timing](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) - ADC conversion time specifications
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - ADC performance (identical to RP2040)
- [Embassy-RP ADC Documentation](https://docs.embassy.dev/embassy-rp/) - Async ADC API usage
