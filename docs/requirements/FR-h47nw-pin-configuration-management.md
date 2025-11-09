# FR-h47nw GPIO Pin Configuration Management

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-higbc-hbridge-motor-control](../requirements/FR-higbc-hbridge-motor-control.md)
- Dependent Requirements:
  - [NFR-444kl-pin-config-build-safety](../requirements/NFR-444kl-pin-config-build-safety.md)
- Related Tasks:
  - [T-po5ns-pin-configuration](../tasks/T-po5ns-pin-configuration/README.md)
  - [T-vf57h-hbridge-motor-control](../tasks/T-vf57h-hbridge-motor-control/README.md)

## Requirement Statement

The system shall support declarative GPIO pin configuration through board definition files (hwdef.dat format) that are parsed at build time to generate type-safe Rust code, providing validated default configurations for known hardware platforms while allowing runtime parameter overrides for development and testing.

## Rationale

Different hardware platforms require different GPIO pin assignments for motors, sensors, and peripherals. Hardcoding pin assignments in source code requires recompilation for each hardware variant and makes pin conflicts difficult to detect. Following ArduPilot's proven hwdef.dat pattern, declarative text-based board definitions provide:

1. **Out-of-Box Experience**: Default configurations for Freenove and custom boards work without user configuration
2. **Readability**: Text files easier to review and diff than Rust const definitions
3. **Build-Time Safety**: Pin conflicts detected during compilation, not at runtime
4. **Multi-Board Support**: One codebase supports multiple hardware variants via build-time selection
5. **Development Flexibility**: Parameter store overrides enable testing without recompilation

This approach balances compile-time safety with runtime flexibility while maintaining consistency with ArduPilot's architecture.

## User Story (if applicable)

As a hardware integrator, I want to define GPIO pin assignments in declarative text files that are validated at build time, so that I can support multiple hardware configurations without modifying Rust code and detect pin conflicts before flashing firmware.

## Acceptance Criteria

- [ ] System supports hwdef.dat format for board definitions (similar to ArduPilot)
- [ ] Build script (`build.rs`) parses hwdef files and generates type-safe Rust const definitions
- [ ] Pin conflict detection occurs at build time (duplicate pin assignments cause compile errors)
- [ ] Reserved platform pins (UART, SPI, QSPI) generate build warnings when assigned to actuators
- [ ] Default board configuration (Freenove Standard Wheel) works without user configuration
- [ ] Generated code provides const `BoardPinConfig` struct with motor pins, buzzer, LED, battery ADC
- [ ] Invalid GPIO numbers (outside platform valid range) rejected during build
- [ ] Multiple board definitions supported via build-time selection (`BOARD=freenove_standard cargo build`)
- [ ] Runtime parameter overrides allowed for development/testing (e.g., `PIN_M1_IN1=18`)
- [ ] Validation at initialization detects pin conflicts from parameter overrides
- [ ] Include directive supported for sharing common configurations (`include boards/common/rp2350.hwdef`)
- [ ] Undef directive supported for overriding included pin definitions
- [ ] Pin modifiers supported (PULLUP, PULLDOWN, OUTPUT, INPUT, ADC, OPENDRAIN, SPEED_HIGH, etc.)
- [ ] Pin type specification supported (INPUT, OUTPUT, ADC) for explicit hardware configuration

## Technical Details

### Functional Requirement Details

**hwdef.dat Format:**

```
# boards/freenove_standard.hwdef
# Freenove 4WD Car Standard Wheel Configuration

# Include common platform settings
include boards/common/rp2350.hwdef

# Actuator counts
MOTOR_COUNT 4

# Motor pins (H-bridge DRV8837)
M1_IN1 18 OUTPUT  # Left front motor IN1
M1_IN2 19 OUTPUT  # Left front motor IN2
M2_IN1 20 OUTPUT  # Left rear motor IN1
M2_IN2 21 OUTPUT  # Left rear motor IN2
M3_IN1 6 OUTPUT   # Right front motor IN1
M3_IN2 7 OUTPUT   # Right front motor IN2
M4_IN1 8 OUTPUT   # Right rear motor IN1
M4_IN2 9 OUTPUT   # Right rear motor IN2

# Optional peripherals
BUZZER 2 OUTPUT PULLDOWN
LED_WS2812 16 OUTPUT
BATTERY_ADC 26 ADC
```

**Common platform file** (`boards/common/rp2350.hwdef`):

```
# Common RP2350 platform settings
PLATFORM rp2350
```

**Override example** (using `undef`):

```
# Custom board based on common config
include boards/freenove_standard.hwdef
undef M1_IN1
M1_IN1 22 OUTPUT  # Override motor 1 IN1 to different GPIO
```

**Build-Time Code Generation (build.rs):**

```rust
fn main() {
    let board = env::var("BOARD").unwrap_or_else(|_| "freenove_standard".to_string());
    let hwdef_path = format!("boards/{}.hwdef", board);

    // Parse hwdef.dat
    let config = parse_hwdef(&hwdef_path)?;

    // Validate pin assignments
    validate_pins(&config)?;

    // Generate Rust code
    let generated = generate_board_config(&config);
    fs::write(out_dir.join("board_config.rs"), generated)?;
}
```

**Generated Code:**

```rust
// Auto-generated from boards/freenove_standard.hwdef
pub const BOARD_CONFIG: BoardPinConfig = BoardPinConfig {
    motors: [
        MotorPins { in1: 18, in2: 19 },
        MotorPins { in1: 20, in2: 21 },
        MotorPins { in1: 6, in2: 7 },
        MotorPins { in1: 8, in2: 9 },
    ],
    buzzer: Some(2),
    led: Some(16),
    battery_adc: Some(26),
};
```

**Pin Configuration API:**

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinType {
    Input,
    Output,
    Adc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PullMode {
    None,
    PullUp,
    PullDown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputMode {
    PushPull,
    OpenDrain,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    Low,
    Medium,
    High,
    VeryHigh,
}

#[derive(Debug, Clone, Copy)]
pub struct PinConfig {
    pub gpio: u8,
    pub pin_type: PinType,
    pub pull: PullMode,
    pub output_mode: OutputMode,
    pub speed: Speed,
}

#[derive(Debug, Clone)]
pub struct MotorPins {
    pub in1: PinConfig,
    pub in2: PinConfig,
}

#[derive(Debug, Clone)]
pub struct BoardPinConfig {
    pub motors: [MotorPins; 4],
    pub buzzer: Option<PinConfig>,
    pub led: Option<PinConfig>,
    pub battery_adc: Option<PinConfig>,
}

impl BoardPinConfig {
    /// Validate pin configuration (no duplicates, valid GPIO numbers)
    pub fn validate(&self) -> Result<(), PinError>;

    /// Load from parameter store with board defaults as fallback
    pub fn load(params: &ParameterStore, board_default: &Self) -> Result<Self, PinError>;
}
```

**Error Conditions:**

- Duplicate pin assignments in hwdef: Build error with conflict report
- Invalid GPIO numbers: Build error with valid range message
- Reserved pins used: Build warning (not error) with suggestion to use different GPIO
- Duplicate parameter overrides: Runtime validation error at initialization
- Pin conflicts from parameters: Runtime validation error before hardware use
- Invalid pin modifiers: Build warning if modifier not supported on platform
- Include file not found: Build error with file path
- Undef on non-existent pin: Build warning (non-fatal)

## Platform Considerations

### RP2350

- Valid GPIO range: 0-29 (30 GPIO pins)
- Reserved pins: UART0 (GPIO0/1 by default), QSPI flash (GPIO47-53)
- PWM constraints: 12 PWM slices available (sufficient for 8 motor pins)
- Build-time validation enforces RP2350 GPIO range

### Cross-Platform

- hwdef format extensible to ESP32, STM32 platforms
- Platform-specific validation rules (GPIO ranges, reserved pins) in build script
- Same BoardPinConfig API across platforms

## Risks & Mitigation

| Risk                                                        | Impact | Likelihood | Mitigation                                        | Validation                               |
| ----------------------------------------------------------- | ------ | ---------- | ------------------------------------------------- | ---------------------------------------- |
| Pin conflicts undetected until hardware testing             | High   | Low        | Build-time validation in build.rs                 | Unit tests for conflict detection        |
| Parameter overrides bypass validation                       | High   | Medium     | Runtime validation before hardware initialization | Integration tests with invalid overrides |
| Invalid hwdef syntax causes build failures                  | Medium | Medium     | Clear error messages, hwdef syntax validation     | Test with malformed hwdef files          |
| Users modify source instead of hwdef (bypassing validation) | Medium | Low        | Documentation emphasizes hwdef as primary method  | Code review checks for hardcoded pins    |

## Implementation Notes

**Preferred Approach:**

- Create `boards/` directory at project root with hwdef files
- Implement build.rs hwdef parser with clear error messages
- Generate minimal Rust code (const definitions only)
- Keep runtime validation simple (duplicate detection, range checks)

**Build Commands:**

```bash
# Default board (Freenove Standard)
cargo build --release

# Custom board
BOARD=custom_rover cargo build --release

# Inspect generated code
cat target/OUT_DIR/board_config.rs
```

**Known Pitfalls:**

- hwdef parser must handle comments (#) and blank lines
- GPIO number validation must be platform-specific (RP2350: 0-29, ESP32: different ranges)
- Reserved pin warnings must not fail the build (warnings only)
- Parameter overrides must not bypass safety validation
- Build script must detect missing hwdef files with helpful error messages
- Include directive must prevent infinite recursion (track include chain)
- Pin modifiers must be optional and have sensible defaults
- Undef must be idempotent (undefining non-existent pin is non-fatal)

**Related Code Areas:**

- `boards/*.hwdef` - Board definition files (new)
- `boards/common/*.hwdef` - Common platform definition files (new)
- `build.rs` - hwdef parser and code generator (new)
- `src/platform/traits/board.rs` - BoardPinConfig types (new)
- `src/platform/rp2350/boards/generated.rs` - Generated const (auto-generated)
- `docs/hwdef-specification.md` - Complete hwdef format specification

**Suggested Libraries:**

- Standard library only for build.rs (file parsing, code generation)
- No runtime dependencies for generated code

## External References

- [ArduPilot hwdef.dat Format](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef) - Reference board definition files
- [ArduPilot Hardware Definition Documentation](https://ardupilot.org/dev/docs/building-the-code.html#board-specific-features) - hwdef.dat usage guide
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - GPIO specifications (Section 3)
- [Cargo Build Scripts](https://doc.rust-lang.org/cargo/reference/build-scripts.html) - build.rs documentation

---

## Template Usage

This requirement follows the structure defined in [docs/templates/requirements.md](../templates/requirements.md).
