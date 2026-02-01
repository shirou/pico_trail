# NFR-444kl Pin Configuration Build-time Safety

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-h47nw-pin-configuration-management](FR-h47nw-pin-configuration-management.md)
- Dependent Requirements: N/A - Not yet defined
- Related Tasks:
  - [T-po5ns-pin-configuration](../tasks/T-po5ns-pin-configuration/README.md)

## Requirement Statement

The GPIO pin configuration system shall detect and reject invalid configurations at build time, including pin conflicts, out-of-range GPIO numbers, and platform constraint violations, preventing invalid firmware from being compiled or flashed to hardware.

## Rationale

GPIO pin configuration errors can cause:

1. **Hardware Damage**: Pin conflicts driving outputs against each other can damage GPIO pins or connected peripherals
2. **Unpredictable Behavior**: Invalid GPIO numbers access undefined hardware registers
3. **Late Detection**: Runtime errors during field deployment are costly and dangerous
4. **Development Friction**: Trial-and-error pin assignment wastes development time

Build-time validation shifts error detection left in the development cycle, preventing invalid configurations from ever reaching hardware. This follows the Rust philosophy of "compile-time guarantees over runtime checks" and reduces the attack surface for configuration-related failures.

## User Story (if applicable)

The system shall validate GPIO pin configurations at build time to ensure that invalid configurations (conflicts, out-of-range pins, platform violations) are rejected during compilation rather than discovered during runtime or hardware testing.

## Acceptance Criteria

- [ ] Duplicate pin assignments in hwdef cause build errors with clear conflict report (e.g., "GPIO 18 assigned to both M1_IN1 and M2_IN1")
- [ ] GPIO numbers outside platform valid range rejected at build time (e.g., RP2350: GPIO 30+ invalid)
- [ ] Reserved platform pins (UART0: GPIO 0-1, QSPI: GPIO 47-53) flagged as warnings if assigned to actuators
- [ ] Reserved pin warnings do not fail the build (warnings only, compilation succeeds)
- [ ] Build errors include file name, line number, and specific validation failure message
- [ ] Build warnings include helpful suggestions (e.g., "Consider using GPIO 2-29 for motor control")
- [ ] Successful build guarantees no pin conflicts in generated default configuration
- [ ] Invalid hwdef syntax (missing pin number, invalid format) causes build error with helpful message
- [ ] Platform-specific constraints validated (e.g., RP2350 PWM slice limitations documented)
- [ ] Include directive recursion detected and rejected with clear error message
- [ ] Invalid pin modifiers generate build warnings (not errors) if not supported on platform
- [ ] Undef of non-existent pins generates warning (not error)

## Technical Details

### Non-Functional Requirement Details

**Security:**

- Invalid configurations rejected before firmware creation (cannot flash bad config)
- Type safety: Generated Rust code uses u8 for GPIO numbers (no negative or out-of-type values)
- Const definitions prevent runtime modification of default board config

**Reliability:**

- Build-time validation: Zero runtime overhead for default configurations
- Deterministic: Same hwdef always produces same generated code and validation results
- Fail-fast: Build stops immediately on first validation error (no partial generation)

**Usability:**

- Clear error messages:

```
error: GPIO 18 used multiple times
  --> boards/freenove_standard.hwdef:8:8
   |
 8 | M2_IN1 18
   |        ^^ GPIO 18 already assigned to M1_IN1 (line 6)
   |
   = help: Each GPIO pin can only be assigned to one function

error: Include recursion detected
  --> boards/circular.hwdef:3:1
   |
 3 | include boards/base.hwdef
   |         ^^^^^^^^^^^^^^^^^^^ Include chain: circular.hwdef -> base.hwdef -> circular.hwdef
   |
   = help: Remove circular include references

error: Include file not found
  --> boards/custom.hwdef:2:9
   |
 2 | include boards/missing.hwdef
   |         ^^^^^^^^^^^^^^^^^^^^^ File does not exist
   |
   = help: Check the file path relative to project root
```

- Warnings for potentially problematic configurations:

```
warning: GPIO 0 is reserved for UART0_TX on RP2350
  --> boards/custom_rover.hwdef:10:8
   |
10 | M1_IN1 0
   |        ^ Consider using GPIO 2-29 for motor control
   |
   = note: This may conflict with console output or debugging

warning: Pin modifier SPEED_VERY_HIGH not supported on RP2350
  --> boards/custom.hwdef:15:20
   |
15 | SERVO1_PWM 10 OUTPUT SPEED_VERY_HIGH
   |                      ^^^^^^^^^^^^^^^ Using default speed instead
   |
   = note: RP2350 supports SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH

warning: Undef of non-existent pin M5_IN1
  --> boards/override.hwdef:8:7
   |
 8 | undef M5_IN1
   |       ^^^^^^ Pin was not previously defined
   |
   = note: This undef has no effect
```

**Compatibility:**

- Platform-specific validation rules (RP2350, ESP32, STM32)
- Extensible validation framework for new platforms
- Integration with Cargo build system (errors shown in IDE)

## Platform Considerations

### RP2350

**Valid GPIO Range:**

- GPIO 0-29: User-accessible pins (30 total)
- GPIO 30-46: Internal/special functions (not user-accessible)
- GPIO 47-53: QSPI flash (reserved, internal)

**Reserved Pins (generate warnings):**

- GPIO 0-1: UART0 (TX/RX) - Warn if assigned to actuators (console/debug interface)
- GPIO 47-53: QSPI flash - Error if assigned (internal, not user-accessible)

**Optional Peripheral Conflicts (no warnings by default):**

- GPIO 2-3: I2C0 (SDA/SCL) - Users may intentionally reassign
- GPIO 16-19: SPI0 - Users may intentionally reassign

**Supported Pin Modifiers:**

- Pin types: INPUT, OUTPUT, ADC
- Pull modes: PULLUP, PULLDOWN, NOPULL
- Output modes: PUSHPULL, OPENDRAIN
- Speeds: SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH

**Unsupported Modifiers (generate warnings):**

- SPEED_VERY_HIGH (not available on RP2350)

**PWM Constraints:**

- 12 PWM slices (24 channels total)
- Document recommended pin assignments for optimal PWM distribution

### Cross-Platform

- Validation rules defined per platform in build.rs
- Generic validation: duplicates, syntax errors
- Platform-specific validation: GPIO ranges, reserved pins, hardware constraints

## Risks & Mitigation

| Risk                                           | Impact | Likelihood | Mitigation                                                         | Validation                               |
| ---------------------------------------------- | ------ | ---------- | ------------------------------------------------------------------ | ---------------------------------------- |
| False positives (valid configs rejected)       | Medium | Low        | Use warnings for ambiguous cases, errors only for definite invalid | Manual testing with edge cases           |
| Incomplete validation (errors slip through)    | High   | Medium     | Comprehensive test suite with invalid hwdef files                  | Integration tests with known-bad configs |
| Poor error messages (users confused)           | Medium | Medium     | Test error messages with users, iterate on clarity                 | User testing with intentional mistakes   |
| Platform constraints missed (new platform)     | Medium | Low        | Document validation extension process                              | Code review checklist for new platforms  |
| Include recursion causes infinite loop         | High   | Low        | Track include chain and detect cycles                              | Unit test with circular includes         |
| Reserved pin warnings too noisy (user fatigue) | Low    | Medium     | Limit warnings to critical reserved pins only (UART0, QSPI)        | User feedback on warning frequency       |

## Implementation Notes

**Preferred Approach:**

```rust
// build.rs validation functions
fn validate_pins(config: &HwDefConfig) -> Result<(), BuildError> {
    validate_no_duplicates(config)?;
    validate_gpio_range(config)?;
    validate_reserved_pins(config)?;
    Ok(())
}

fn validate_no_duplicates(config: &HwDefConfig) -> Result<(), BuildError> {
    let mut seen = HashMap::new();
    for (pin_name, gpio_num, line_num) in &config.pins {
        if let Some((prev_name, prev_line)) = seen.get(gpio_num) {
            return Err(BuildError::DuplicatePin {
                gpio: *gpio_num,
                first_use: (*prev_name, *prev_line),
                second_use: (*pin_name, *line_num),
            });
        }
        seen.insert(gpio_num, (pin_name, line_num));
    }
    Ok(())
}
```

**Error Message Format:**

```rust
impl fmt::Display for BuildError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            BuildError::DuplicatePin { gpio, first_use, second_use } => {
                write!(f, "Pin conflict: GPIO {} assigned to both {} and {}",
                       gpio, first_use.0, second_use.0)
            }
            BuildError::InvalidGpio { gpio, platform, valid_range } => {
                write!(f, "Invalid GPIO {}: {} supports only GPIO {}",
                       gpio, platform, valid_range)
            }
        }
    }
}
```

**Verification Commands:**

```bash
# Test with valid hwdef (should succeed)
BOARD=freenove_standard cargo build

# Test with invalid hwdef (should fail with clear error)
BOARD=test_duplicate_pins cargo build

# Test with reserved pins (should succeed with warnings)
BOARD=test_reserved_uart cargo build 2>&1 | grep "warning:"

# Test with circular includes (should fail)
BOARD=test_circular_include cargo build 2>&1 | grep "Include recursion"

# Test with missing include file (should fail)
BOARD=test_missing_include cargo build 2>&1 | grep "Include file not found"

# Test with unsupported modifiers (should succeed with warnings)
BOARD=test_unsupported_modifier cargo build 2>&1 | grep "warning:"

# Check error message clarity
BOARD=test_invalid_gpio cargo build 2>&1 | grep "error:"
```

**Known Pitfalls:**

- Validation must occur before code generation (fail fast)
- Error messages must include hwdef file path and line numbers
- Reserved pin warnings should not fail build (warn only)
- Include recursion must be detected before attempting to parse (prevent stack overflow)
- Validation should be fast (< 100ms) to avoid slowing builds
- Warning messages should be actionable (suggest alternatives)
- Unsupported modifiers should fall back to safe defaults (documented in warning)

**Related Code Areas:**

- `build.rs` - Validation logic implementation
- `boards/*.hwdef` - Input files to validate
- CI configuration - Build tests with invalid hwdef files

**Suggested Testing:**

- Unit tests for validation functions (duplicates, ranges, reserved pins)
- Integration tests with intentionally broken hwdef files
- Verify error messages are actionable and clear

## External References

- [Rust Error Handling Best Practices](https://nick.groenen.me/posts/rust-error-handling/) - Guidelines for clear error messages
- [RP2350 GPIO Specifications](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - Valid GPIO ranges and constraints (Section 3.1)
- [Cargo Build Script Error Handling](https://doc.rust-lang.org/cargo/reference/build-script-examples.html#case-study-code-generation) - Error reporting in build.rs

---

## Template Usage

This requirement follows the structure defined in [docs/templates/requirements.md](../templates/requirements.md).
