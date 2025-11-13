# NFR-tximn | Host Test Support

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-n8yb7-battery-telemetry](../analysis/AN-n8yb7-battery-telemetry.md)
- Prerequisite Requirements:
  - [FR-015k2-adc-battery-voltage-reading](../requirements/FR-015k2-adc-battery-voltage-reading.md)
  - [FR-uq6as-voltage-conversion-calculation](../requirements/FR-uq6as-voltage-conversion-calculation.md)
  - [FR-ygjkj-battery-state-scheduler-update](../requirements/FR-ygjkj-battery-state-scheduler-update.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-q3psc-battery-telemetry](../tasks/T-q3psc-battery-telemetry/README.md)

## Requirement Statement

Battery monitoring functionality shall support host-based unit tests without requiring physical hardware by providing mock ADC drivers that simulate realistic voltage values and state transitions.

## Rationale

Developers need to test battery monitoring logic (voltage conversion, failsafe triggering, pre-arm checks) on their development machines without physical Pico 2 W hardware. Running tests in CI/CD pipelines requires host-compatible code. Mock ADC drivers enable fast iteration, automated testing, and validation of battery logic before hardware integration. This follows the project's existing pattern of platform abstraction (see `src/platform/` structure).

## User Story (if applicable)

The system shall provide mock ADC implementations for host tests to ensure developers can validate battery monitoring logic on development machines without requiring physical hardware.

## Acceptance Criteria

- [ ] Battery monitoring code compiles and runs in `cargo test --lib` (host target)
- [ ] Mock ADC driver provides configurable simulated voltage values
- [ ] Mock ADC integrated with `Board` trait abstraction (`src/platform/traits/board.rs`)
- [ ] Unit tests can simulate voltage scenarios: nominal, low, critical, rising, falling
- [ ] Mock ADC does not require real hardware or GPIO access
- [ ] Host tests validate voltage conversion formula (FR-uq6as)
- [ ] Host tests validate pre-arm check battery logic (`src/core/arming/checks.rs:179`)
- [ ] Host tests validate failsafe triggering on low/critical voltage
- [ ] Mock ADC implementation separate from production code (conditional compilation or trait impl)

## Technical Details

### Non-Functional Requirement Details

- **Usability**: Developer experience
  - Run tests with `cargo test --lib` (no special flags or setup)
  - Tests execute in <5 seconds on typical development machine
  - Clear test output shows battery voltage scenarios and results
  - No hardware required for CI/CD integration

- **Compatibility**: Platform abstraction
  - Production code uses `Board` trait for hardware access
  - Host tests use mock `Board` implementation
  - Same battery monitoring code runs on both embedded and host targets
  - Conditional compilation (`#[cfg(test)]` or feature gates) separates implementations

- **Reliability**: Test coverage
  - Unit tests cover voltage conversion edge cases (0V, 12.6V, overflow)
  - Unit tests cover pre-arm check threshold logic
  - Unit tests cover failsafe triggering at `BATT_CRT_VOLT`
  - Integration tests simulate battery drain over time

**Mock ADC Architecture:**

The project uses platform abstraction via the `Board` trait. Mock ADC should integrate naturally:

1. **Board Trait** (`src/platform/traits/board.rs`):

   ```rust
   pub trait Board {
       async fn read_battery_adc(&self) -> u16;
       // ... other methods
   }
   ```

2. **Production Implementation** (e.g., `src/platform/pico2_w/board.rs`):

   ```rust
   impl Board for Pico2WBoard {
       async fn read_battery_adc(&self) -> u16 {
           // Real embassy-rp ADC read
       }
   }
   ```

3. **Mock Implementation** (test-only):

   ```rust
   #[cfg(test)]
   pub struct MockBoard {
       battery_adc_value: u16,
   }

   #[cfg(test)]
   impl Board for MockBoard {
       async fn read_battery_adc(&self) -> u16 {
           self.battery_adc_value
       }
   }
   ```

**Test Scenarios:**

Host tests should validate battery monitoring in realistic scenarios:

1. **Nominal Operation**:
   - ADC: 3000 counts → 9.52V (healthy 3S LiPo)
   - Pre-arm: Pass (above BATT_ARM_VOLT = 10.5V? May need adjustment)
   - Failsafe: Not triggered

2. **Low Voltage Warning**:
   - ADC: 2600 counts → 8.23V (below BATT_ARM_VOLT)
   - Pre-arm: Fail
   - Failsafe: Not triggered yet

3. **Critical Voltage Failsafe**:
   - ADC: 2500 counts → 7.91V (below BATT_CRT_VOLT = 10.0V? May need adjustment)
   - Pre-arm: Fail
   - Failsafe: Triggered (BATT_FS_CRT_ACT action)

4. **Voltage Conversion Accuracy**:
   - Test known ADC values against expected voltages
   - Example: ADC 4095 → 13.04V (max ADC, coefficient 3.95)

**Example Test:**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_battery_voltage_conversion() {
        let mock_board = MockBoard { battery_adc_value: 3000 };
        let volt_mult = 3.95;
        let voltage = BatteryState::voltage_from_adc(3000, volt_mult);

        // Expected: (3000 / 4095.0 * 3.3) * 3.95 = 9.52V
        assert!((voltage - 9.52).abs() < 0.01, "Voltage conversion incorrect: {}", voltage);
    }

    #[tokio::test]
    async fn test_battery_pre_arm_check_low_voltage() {
        let mock_board = MockBoard { battery_adc_value: 2600 }; // ~8.2V
        let mut system_state = SystemState::default();
        system_state.update_battery(&mock_board).await;

        let pre_arm_result = check_battery_voltage(&system_state);
        assert!(pre_arm_result.is_err(), "Pre-arm should fail with low voltage");
    }
}
```

## Platform Considerations

### Unix

Host tests run on Unix development machines (Linux, macOS). Mock ADC uses standard library features (no hardware access).

### Windows

Host tests run on Windows development machines. Mock ADC uses standard library features (no hardware access).

### Cross-Platform

Mock ADC must compile and run on all host platforms (Linux, macOS, Windows). Use `#[cfg(test)]` or feature gates to separate from embedded code.

## Risks & Mitigation

| Risk                                                | Impact | Likelihood | Mitigation                                              | Validation                                         |
| --------------------------------------------------- | ------ | ---------- | ------------------------------------------------------- | -------------------------------------------------- |
| Mock ADC behavior diverges from real hardware       | Medium | Medium     | Validate mock values against real hardware measurements | Cross-check test voltages with actual ADC readings |
| Tests pass on host but fail on hardware             | High   | Medium     | Integration tests on hardware; CI runs embedded builds  | Hardware-in-the-loop testing before releases       |
| Async runtime differences between host and embedded | Medium | Low        | Use tokio for host tests, embassy for embedded          | Test async behavior in both environments           |
| Mock ADC not updated when real ADC API changes      | Medium | Medium     | Code review ensures mock and production stay in sync    | Trait abstraction enforces API consistency         |

## Implementation Notes

**Preferred Approaches:**

- Use `#[cfg(test)]` for mock implementations in test modules
- Mock ADC implements `Board` trait for seamless integration
- Use `tokio::test` macro for async tests on host
- Example mock board:

  ```rust
  #[cfg(test)]
  pub struct MockBoard {
      battery_adc_value: Arc<Mutex<u16>>,
  }

  #[cfg(test)]
  impl MockBoard {
      pub fn new(initial_adc: u16) -> Self {
          Self {
              battery_adc_value: Arc::new(Mutex::new(initial_adc)),
          }
      }

      pub fn set_adc_value(&self, value: u16) {
          *self.battery_adc_value.lock().unwrap() = value;
      }
  }
  ```

**Known Pitfalls:**

- Do not use `#[cfg(target_os = "...")]` for mock; use `#[cfg(test)]` or feature gates
- Ensure mock ADC is realistic (0-4095 range, simulates averaging behavior)
- Do not couple mock to specific test values; make it configurable
- Mock flash storage (NFR-j17oa) also needed for parameter persistence tests

**Related Code Areas:**

- `src/platform/traits/board.rs` - Board trait definition
- `src/communication/mavlink/state.rs` - BatteryState struct
- `src/core/arming/checks.rs:179` - Pre-arm battery check (test target)
- Test modules: Add `tests/battery_monitoring.rs` or similar

**Validation Strategy:**

- Unit tests: Test voltage conversion with known ADC values
- Unit tests: Test pre-arm checks with low/critical voltage scenarios
- Unit tests: Test failsafe triggering logic
- Integration tests: Simulate battery drain over time
- CI/CD: Run `cargo test --lib` in GitHub Actions or similar

## External References

- [Rust Testing Documentation](https://doc.rust-lang.org/book/ch11-00-testing.html) - Unit testing best practices
- [Tokio Testing Guide](https://tokio.rs/tokio/topics/testing) - Async test setup for host tests
- Embassy Testing Patterns - Embedded testing strategies | N/A

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
