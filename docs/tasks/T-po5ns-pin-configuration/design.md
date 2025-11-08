# T-po5ns Pin Configuration Management

## Metadata

- Type: Design
- Status: Completed

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design implements GPIO pin configuration management using hwdef.dat-style board definition files with build-time code generation. The system parses declarative text files at build time to generate type-safe Rust const definitions, providing validated default configurations for known hardware platforms (Freenove 4WD Car) while allowing runtime parameter overrides for development and testing. This approach follows ArduPilot's proven architecture pattern for board definitions.

## Success Metrics

- [ ] Freenove Standard Wheel works with zero manual configuration
- [ ] Pin conflicts detected at build time (duplicate pin assignments cause compile errors)
- [ ] Reserved pins (UART0 GPIO 0-1, QSPI GPIO 47-53) generate build warnings when assigned to actuators
- [ ] Include directive enables sharing common platform settings across board definitions
- [ ] Undef directive allows fine-grained overrides of included pin definitions
- [ ] Pin modifiers (PULLUP, PULLDOWN, OUTPUT, INPUT, ADC, OPENDRAIN, SPEED_HIGH, etc.) supported with platform-specific validation
- [ ] Include recursion detected and rejected with clear error message
- [ ] Generated code provides type-safe BoardPinConfig struct with full pin configuration details
- [ ] Zero runtime overhead (all parsing done at build time)
- [ ] Parameter store can override pins for development without recompilation

## Background and Current State

- Context: Freenove 4WD Car requires specific GPIO pin assignments for 4 motors (8 pins total) with H-bridge drivers, plus optional peripherals (buzzer, LED, battery ADC)
- Current behavior: Pin assignments scattered across modules, hardcoded in source code, requires recompilation for changes
- Pain points:
  - No central registry of pin assignments
  - Pin conflicts discovered at runtime or during hardware testing
  - Difficult to support multiple hardware configurations
  - Source code changes required for custom pin layouts
- Constraints:
  - RP2350 platform: GPIO 0-29 valid, some pins reserved (UART, SPI, etc.)
  - Limited flash memory (avoid large parameter tables)
  - No_std embedded environment (no file I/O at runtime)
- Related ADRs: [ADR-mcg03-pin-configuration-management](../../adr/ADR-mcg03-pin-configuration-management.md)

## Proposed Design

### High-Level Architecture

```text
Build Time:
  boards/freenove_standard.hwdef
    ↓ (include directive)
  boards/common/rp2350.hwdef
    ↓ (parsed by)
  build.rs (with include resolution, undef support, pin modifiers)
    ↓ (validates: GPIO ranges, duplicates, reserved pins warnings)
    ↓ (generates)
  OUT_DIR/board_config.rs (with PinConfig structs)
    ↓ (included in)
  src/main.rs

Runtime:
  BOARD_CONFIG (const) → BoardPinConfig::load() → validated config → MotorGroup
                              ↑
                       ParameterStore (optional overrides)
```

### Components

- **hwdef Parser** (`build.rs`):
  - Reads `boards/*.hwdef` files
  - **NEW**: Processes `include` directives to merge common platform configurations
  - **NEW**: Handles `undef` directives to override included pin definitions
  - Parses pin assignments (M1_IN1, M1_IN2, etc.) with optional modifiers
  - **NEW**: Parses actuator count directives (MOTOR_COUNT, SERVO_COUNT, ESC_COUNT, STEPPER_COUNT)
  - **NEW**: Parses pin modifiers (PULLUP, PULLDOWN, OUTPUT, INPUT, ADC, OPENDRAIN, SPEED_HIGH, etc.)
  - **NEW**: Detects include recursion and rejects circular dependencies
  - Validates GPIO numbers (platform-specific ranges)
  - Detects duplicate pin assignments
  - **NEW**: Generates warnings for reserved pins (UART0, QSPI) without failing the build
  - Generates Rust const definitions with full pin configuration

- **Code Generator** (`build.rs`):
  - **NEW**: Produces `BoardPinConfig` const with `PinConfig` structs containing GPIO number and modifiers
  - Supports motor pins, servo pins, ESC pins, stepper pins, buzzer, LED, battery ADC
  - Writes to `OUT_DIR/board_config.rs`
  - Includes platform metadata and validation markers
  - **NEW**: Generates enums for pin types, pull modes, output modes, and speeds

- **Board Types** (`src/platform/traits/board.rs`):
  - **NEW**: `PinType` enum (Input, Output, Adc)
  - **NEW**: `PullMode` enum (None, PullUp, PullDown)
  - **NEW**: `OutputMode` enum (PushPull, OpenDrain)
  - **NEW**: `Speed` enum (Low, Medium, High, VeryHigh)
  - **NEW**: `PinConfig` struct (gpio, pin_type, pull, output_mode, speed)
  - `MotorPins` struct (in1, in2 as `PinConfig`)
  - `BoardPinConfig` struct (motors array, optional peripherals as `PinConfig`)
  - `validate()` method (runtime conflict detection)
  - `load()` method (parameter store integration)

- **Generated Code** (`OUT_DIR/board_config.rs`):
  - Auto-generated from hwdef.dat
  - Const `BOARD_CONFIG: BoardPinConfig`
  - Included via `include!(concat!(env!("OUT_DIR"), "/board_config.rs"))`

### Data Flow

**Build-time flow:**

1. User selects board via `BOARD=freenove_standard` environment variable (default: freenove_standard)
2. `build.rs` reads `boards/${BOARD}.hwdef`
3. **NEW**: Parser resolves `include` directives recursively, detecting cycles
4. **NEW**: Parser processes `undef` directives to remove included pin definitions
5. Parser extracts pin assignments with modifiers (M1_IN1=18 OUTPUT, M1_IN2=19 OUTPUT, etc.)
6. **NEW**: Parser extracts actuator counts (MOTOR_COUNT=4)
7. **NEW**: Validator checks GPIO ranges (RP2350: 0-29)
8. **NEW**: Validator detects duplicate pins across all actuators and peripherals
9. **NEW**: Validator checks reserved pins (GPIO 0-1 for UART0, GPIO 47-53 for QSPI) and emits warnings
10. **NEW**: Validator checks pin modifier support for platform (e.g., SPEED_VERY_HIGH not supported on RP2350)
11. Generator writes Rust const with full `PinConfig` structs to `OUT_DIR/board_config.rs`
12. Compiler includes generated code in binary

**Runtime flow:**

1. Application calls `BoardPinConfig::load(&param_store, &BOARD_CONFIG)`
2. Load checks parameter store for overrides (e.g., `PIN_M1_IN1`)
3. Falls back to BOARD_CONFIG defaults if no override
4. Validate checks for duplicate pins and invalid GPIO numbers
5. Returns validated config or error
6. MotorGroup uses validated config to initialize hardware

### Data Models and Types

```rust
// src/platform/traits/board.rs

// NEW: Pin configuration enums
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinType {
    Input,   // Digital input
    Output,  // Digital output (default for actuators)
    Adc,     // Analog input
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PullMode {
    None,     // No pull resistors (default)
    PullUp,   // Enable internal pull-up
    PullDown, // Enable internal pull-down
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputMode {
    PushPull,  // Push-pull output (default)
    OpenDrain, // Open-drain output
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    Low,      // Low slew rate (reduced EMI)
    Medium,   // Medium slew rate (default)
    High,     // High slew rate (fast switching)
    VeryHigh, // Very high slew rate (maximum speed, not all platforms)
}

// NEW: Pin configuration with modifiers
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
    pub in1: PinConfig,  // Motor IN1 with full configuration
    pub in2: PinConfig,  // Motor IN2 with full configuration
}

#[derive(Debug, Clone)]
pub struct BoardPinConfig {
    pub motors: [MotorPins; 4],           // 4 motors for Freenove
    pub buzzer: Option<PinConfig>,        // Optional buzzer with configuration
    pub led: Option<PinConfig>,           // Optional WS2812 LED with configuration
    pub battery_adc: Option<PinConfig>,   // Optional battery ADC with configuration
}

#[derive(Debug)]
pub enum PinError {
    DuplicatePin(u8),                // Pin used multiple times
    InvalidGpio(u8),                 // GPIO outside valid range
    ParameterParseError(String),     // Invalid parameter value
    ReservedPinUsed(u8, String),     // Reserved pin (GPIO, purpose)
    UnsupportedModifier(String),     // Pin modifier not supported on platform
}

impl BoardPinConfig {
    /// Validate pin configuration (no duplicates, valid GPIO numbers)
    pub fn validate(&self) -> Result<(), PinError>;

    /// Load from parameter store with board defaults as fallback
    pub fn load(params: &ParameterStore, board_default: &Self) -> Result<Self, PinError>;
}
```

**hwdef.dat format with new features:**

```
# boards/freenove_standard.hwdef
# Freenove 4WD Car Standard Wheel Configuration

# NEW: Include common platform settings
include boards/common/rp2350.hwdef

# NEW: Actuator count directives
MOTOR_COUNT 4

# Motor pins with modifiers (NEW: pin modifiers)
M1_IN1 18 OUTPUT           # Left front motor IN1
M1_IN2 19 OUTPUT           # Left front motor IN2
M2_IN1 20 OUTPUT           # Left rear motor IN1
M2_IN2 21 OUTPUT           # Left rear motor IN2
M3_IN1 6 OUTPUT            # Right front motor IN1
M3_IN2 7 OUTPUT            # Right front motor IN2
M4_IN1 8 OUTPUT            # Right rear motor IN1
M4_IN2 9 OUTPUT            # Right rear motor IN2

# Optional peripherals with modifiers (NEW: pin modifiers)
BUZZER 2 OUTPUT PULLDOWN   # Buzzer with pull-down resistor
LED_WS2812 16 OUTPUT       # WS2812 LED data line
BATTERY_ADC 26 ADC         # Battery voltage monitoring
```

**Common platform file** (`boards/common/rp2350.hwdef`):

```
# Common RP2350 platform settings
# Included by all RP2350-based boards
PLATFORM rp2350
```

**Override example using undef** (NEW: undef directive):

```
# Custom board based on Freenove configuration
include boards/freenove_standard.hwdef

# Override motor 1 pins for different hardware layout
undef M1_IN1
undef M1_IN2
M1_IN1 22 OUTPUT    # Different H-bridge wiring
M1_IN2 23 OUTPUT
```

### Error Handling

**Build-time errors (build.rs):**

- Invalid hwdef syntax → Compile error with line number and expected format
- Duplicate pin assignments → Compile error listing conflicts (e.g., "GPIO 18 used for M1_IN1 and BUZZER")
- Invalid GPIO numbers → Compile error with valid range (e.g., "GPIO 35 invalid for RP2350, valid range: 0-29")
- Missing required pins → Compile error listing missing motor pins
- **NEW**: Include recursion detected → Compile error with include chain (e.g., "Include recursion: circular.hwdef -> base.hwdef -> circular.hwdef")
- **NEW**: Include file not found → Compile error with file path (e.g., "Include file not found: boards/missing.hwdef")
- **NEW**: Invalid actuator count → Compile error (e.g., "MOTOR_COUNT must be 0-8, got 10")

**Build-time warnings (non-fatal):**

- **NEW**: Reserved pins assigned → Warning with suggestion (e.g., "GPIO 0 is reserved for UART0_TX on RP2350. Consider using GPIO 2-29 for motor control")
- **NEW**: Unsupported pin modifiers → Warning with fallback (e.g., "Pin modifier SPEED_VERY_HIGH not supported on RP2350. Using default speed instead")
- **NEW**: Undef on non-existent pin → Warning (e.g., "Undef of non-existent pin M5_IN1. This undef has no effect")

**Runtime errors:**

- Parameter override creates conflict → `PinError::DuplicatePin(gpio)` at initialization
- Invalid GPIO from parameter → `PinError::InvalidGpio(gpio)` at validation
- Parameter parse failure → `PinError::ParameterParseError(msg)`
- **NEW**: Reserved pin used after parameter override → `PinError::ReservedPinUsed(gpio, purpose)` at validation
- **NEW**: Unsupported modifier in parameter → `PinError::UnsupportedModifier(modifier_name)` at parsing

All error messages in English following project standards.

### Security Considerations

- Pin validation prevents hardware damage from invalid GPIO assignments
- Build-time validation reduces attack surface (no runtime parsing)
- Parameter overrides must pass same validation as defaults
- Future: Consider requiring unarmed state for pin configuration changes

### Performance Considerations

- Zero runtime overhead for default configuration (const definitions)
- Minimal runtime cost for parameter override (one-time lookup at initialization)
- No heap allocations (embedded no_std environment)
- Build time increases negligibly (simple text parsing)

### Platform Considerations

#### RP2350

- Valid GPIO range: 0-29 (30 GPIO pins)
- **NEW**: Reserved pins (generate warnings, not errors):
  - GPIO 0-1: UART0 (TX/RX) - Console/debug interface
  - GPIO 47-53: QSPI flash (internal, not user-accessible) - Bootloader and firmware storage
- **NEW**: Supported pin modifiers:
  - Pin types: INPUT, OUTPUT, ADC
  - Pull modes: PULLUP, PULLDOWN, NOPULL
  - Output modes: PUSHPULL, OPENDRAIN
  - Speeds: SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH (SPEED_VERY_HIGH not supported, generates warning)
- PWM constraints: 12 PWM slices (24 channels), sufficient for 8 motor pins
- Build-time validation enforces GPIO 0-29 range

#### Cross-Platform (Future)

- hwdef format extensible to ESP32, STM32 platforms
- Platform-specific validation rules in build.rs
- Same BoardPinConfig API across platforms

#### Filesystem

- Build script reads from `boards/` directory
- Generated code written to `OUT_DIR` (cargo-managed)
- No runtime file I/O (embedded no_std)

## Alternatives Considered

1. **Hardcoded Rust Consts**
   - Pros: Simplest implementation, type-safe, compile-time validated
   - Cons: Requires source code changes for pin customization, less readable than declarative format
2. **Full Parameter Store Configuration**
   - Pros: Maximum runtime flexibility, one binary for all boards
   - Cons: Flash overhead, runtime validation required, easy to misconfigure
3. **JSON/TOML Board Files**
   - Pros: Structured format, familiar to developers
   - Cons: Requires parser dependencies, more complex than hwdef.dat, less ArduPilot-aligned

### Decision Rationale

Selected hwdef.dat approach because:

- Exact alignment with ArduPilot's proven architecture
- Declarative text format more readable than Rust code
- Build-time validation catches errors before flashing
- Zero runtime overhead while maintaining flexibility via parameter overrides
- Easy to review and diff in version control

## Migration and Compatibility

- Backward compatibility: N/A (new feature, no existing pin configuration system)
- Forward compatibility: hwdef format designed to be extensible (additional pin types, platforms)
- Rollout plan: Implement for RP2350/Freenove first, extend to other platforms as needed
- Deprecation plan: N/A

## Testing Strategy

### Unit Tests

- `build.rs` parser tests (in `build.rs` with `#[cfg(test)]`):
  - Parse valid hwdef.dat
  - Reject invalid syntax
  - Detect duplicate pins
  - Validate GPIO ranges
  - Handle comments and blank lines
- `BoardPinConfig` validation tests (in `src/platform/traits/board.rs`):
  - Detect duplicate motor pins
  - Detect motor + peripheral conflicts
  - Accept valid configurations
  - Reject invalid GPIO numbers

### Integration Tests

- Build test with valid hwdef → verify generated code compiles
- Build test with invalid hwdef → verify compile error
- Runtime test: Load config from parameter store
- Runtime test: Validate detects override conflicts

### External API Parsing (if applicable)

- N/A (no external APIs)

### Performance & Benchmarks (if applicable)

- Benchmark build time with hwdef parsing (expect <100ms overhead)
- N/A for runtime (const definitions have zero cost)

## Documentation Impact

- Add `docs/hwdef-specification.md` defining hwdef.dat syntax and validation rules
- Update architecture documentation with pin configuration module
- Add examples to Freenove board documentation
- Document parameter override pattern (though note: ArduPilot uses board definitions architecturally, not parameters)

## External References

- [ArduPilot Hardware Definition Files](https://ardupilot.org/dev/docs/building-the-code.html#board-specific-features)
- [ArduPilot hwdef.dat Examples](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef)
- [RP2350 Datasheet GPIO Specifications](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [Cargo Build Scripts Documentation](https://doc.rust-lang.org/cargo/reference/build-scripts.html)

## Open Questions

- [x] Should build.rs validation enforce reserved pins (UART, SPI) or just document them? → **DECIDED**: Generate warnings only, allowing advanced users to override if needed (e.g., disabling UART console to free GPIO 0-1)
- [x] Do we need per-motor enable/disable in hwdef.dat? → **DECIDED**: Use MOTOR_COUNT directive to specify number of active motors (implementation detail for motor abstraction layer)
- [x] Should parameter override prefix be `PIN_` or follow different pattern? → **DECIDED**: Use `PIN_` prefix (though note: GPIO is typically architectural via hwdef, not parameter-based in ArduPilot)
- [x] How should include recursion be handled? → **DECIDED**: Track include chain with HashSet, detect cycles, fail with clear error message showing include chain
- [x] What pin modifiers should be supported? → **DECIDED**: PULLUP, PULLDOWN, INPUT, OUTPUT, ADC, OPENDRAIN, SPEED_LOW/MEDIUM/HIGH (platform-specific, with warnings for unsupported modifiers)
- [x] Should undef of non-existent pins be an error or warning? → **DECIDED**: Warning only (non-fatal), as it may occur due to conditional includes

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
