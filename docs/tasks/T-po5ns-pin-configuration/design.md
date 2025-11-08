# T-po5ns Pin Configuration Management

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design implements GPIO pin configuration management using hwdef.dat-style board definition files with build-time code generation. The system parses declarative text files at build time to generate type-safe Rust const definitions, providing validated default configurations for known hardware platforms (Freenove 4WD Car) while allowing runtime parameter overrides for development and testing. This approach follows ArduPilot's proven architecture pattern for board definitions.

## Success Metrics

- [ ] Freenove Standard Wheel works with zero manual configuration
- [ ] Pin conflicts detected at build time (duplicate pin assignments cause compile errors)
- [ ] Generated code provides type-safe BoardPinConfig struct
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
    ↓ (parsed by)
  build.rs
    ↓ (generates)
  OUT_DIR/board_config.rs
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
  - Parses pin assignments (M1_IN1, M1_IN2, etc.)
  - Validates GPIO numbers (platform-specific ranges)
  - Detects duplicate pin assignments
  - Generates Rust const definitions

- **Code Generator** (`build.rs`):
  - Produces `BoardPinConfig` const with motor pins, buzzer, LED, battery ADC
  - Writes to `OUT_DIR/board_config.rs`
  - Includes platform metadata and validation markers

- **Board Types** (`src/platform/traits/board.rs`):
  - `MotorPins` struct (in1, in2 GPIO numbers)
  - `BoardPinConfig` struct (motors array, optional peripherals)
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
3. Parser extracts pin assignments (M1_IN1=18, M1_IN2=19, etc.)
4. Validator checks GPIO ranges and detects conflicts
5. Generator writes Rust const to `OUT_DIR/board_config.rs`
6. Compiler includes generated code in binary

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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorPins {
    pub in1: u8,  // GPIO number for IN1
    pub in2: u8,  // GPIO number for IN2
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoardPinConfig {
    pub motors: [MotorPins; 4],   // 4 motors for Freenove
    pub buzzer: Option<u8>,        // Optional buzzer GPIO
    pub led: Option<u8>,           // Optional WS2812 LED GPIO
    pub battery_adc: Option<u8>,  // Optional battery ADC channel
}

#[derive(Debug)]
pub enum PinError {
    DuplicatePin(u8),           // Pin used multiple times
    InvalidGpio(u8),            // GPIO outside valid range
    ParameterParseError(String), // Invalid parameter value
}

impl BoardPinConfig {
    /// Validate pin configuration (no duplicates, valid GPIO numbers)
    pub fn validate(&self) -> Result<(), PinError>;

    /// Load from parameter store with board defaults as fallback
    pub fn load(params: &ParameterStore, board_default: &Self) -> Result<Self, PinError>;
}
```

**hwdef.dat format:**

```
# boards/freenove_standard.hwdef
# Comments start with #

# Platform specification
PLATFORM rp2350

# Motor pins (required, 4 motors)
M1_IN1 18
M1_IN2 19
M2_IN1 20
M2_IN2 21
M3_IN1 6
M3_IN2 7
M4_IN1 8
M4_IN2 9

# Optional peripherals
BUZZER 2
LED_WS2812 16
BATTERY_ADC 26
```

### Error Handling

**Build-time errors (build.rs):**

- Invalid hwdef syntax → Compile error with line number and expected format
- Duplicate pin assignments → Compile error listing conflicts (e.g., "GPIO 18 used for M1_IN1 and BUZZER")
- Invalid GPIO numbers → Compile error with valid range (e.g., "GPIO 35 invalid for RP2350, valid range: 0-29")
- Missing required pins → Compile error listing missing motor pins

**Runtime errors:**

- Parameter override creates conflict → `PinError::DuplicatePin(gpio)` at initialization
- Invalid GPIO from parameter → `PinError::InvalidGpio(gpio)` at validation
- Parameter parse failure → `PinError::ParameterParseError(msg)`

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
- Reserved pins (documented, not enforced in initial version):
  - GPIO 0-1: UART0 (default console)
  - GPIO 47-53: QSPI flash (internal, not user-accessible)
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

- [ ] Should build.rs validation enforce reserved pins (UART, SPI) or just document them? → Next step: Start with documentation only, add enforcement if needed based on user feedback
- [ ] Do we need per-motor enable/disable in hwdef.dat? → Next step: Defer to motor abstraction layer (out of scope for pin configuration)
- [ ] Should parameter override prefix be `PIN_` or follow different pattern? → Next step: Review ArduPilot parameter conventions (though note: GPIO typically architectural, not parameter-based)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
