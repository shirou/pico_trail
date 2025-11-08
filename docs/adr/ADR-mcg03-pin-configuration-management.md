# ADR-mcg03 Pin Configuration Management

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-h47nw-pin-configuration-management](../requirements/FR-h47nw-pin-configuration-management.md)
  - [NFR-444kl-pin-config-build-safety](../requirements/NFR-444kl-pin-config-build-safety.md)
- Related Analyses:
  - [AN-qlnt3-freenove-hardware-support](../analysis/AN-qlnt3-freenove-hardware-support.md)
- Related ADRs:
  - [ADR-4hx3d-motor-driver-abstraction](../adr/ADR-4hx3d-motor-driver-abstraction.md)
- Related Tasks:
  - [T-po5ns-pin-configuration](../tasks/T-po5ns-pin-configuration/README.md)

## Context

The Freenove 4WD Car requires specific GPIO pin assignments for motor control (8 pins for 4 motors with H-bridge drivers). Different hardware platforms may use different pin configurations, and users may need to adapt pin assignments for custom hardware or avoid conflicts with other peripherals.

**Current State:**

- Platform traits define some pin assignments (e.g., SPI0 on GPIO16/18/19 in `src/platform/rp2350/platform.rs`)
- Pin assignments are hardcoded at compile time
- No mechanism to change pin assignments without modifying source code
- Pin conflicts discovered at runtime or during hardware testing

**New Requirements:**

- Support Freenove Standard Wheel pin configuration (8 motor pins + buzzer + LED + ADC)
- Allow custom pin configurations for different hardware variants
- Detect and prevent pin conflicts at initialization
- Provide validated default configurations for known platforms
- Enable runtime configuration for development/testing

**Forces in Tension:**

- **Compile-Time Safety**: Static pin assignments catch conflicts at compile time
- **Runtime Flexibility**: Users need to change pins without recompiling firmware
- **Type Safety**: Pin configuration should prevent invalid GPIO assignments
- **Flash Usage**: Large parameter tables consume limited flash memory
- **Usability**: Default configurations should work out-of-box for known hardware

**Pain Points:**

- Hardcoded pins require source code changes for hardware variants
- No central registry of pin assignments (scattered across modules)
- Pin conflicts discovered late (at runtime or during testing)
- Difficult to support multiple hardware configurations in one binary

## Success Metrics

- **Default Usability**: Freenove platform works out-of-box with zero configuration changes
- **Flexibility**: Custom pin assignments possible via parameter store without recompiling, include/undef directives enable board variants
- **Conflict Detection**: Pin conflicts detected at build time (duplicate pins), reserved pins generate warnings
- **Type Safety**: Invalid GPIO pin numbers rejected at compile time (for static configs) or initialization (for parameter-based configs)
- **Reusability**: Common platform settings shared via include directive, reducing duplication across board definitions

## Decision

We will implement pin configuration using hwdef.dat-style board definition files with build-time code generation, following ArduPilot's exact architecture. Pin assignments are declared in declarative text files (`boards/*.hwdef`) and parsed at build time by `build.rs` to generate type-safe Rust const definitions.

### Decision Drivers

- **ArduPilot Consistency**: Match ArduPilot's board definition (hwdef.dat) + parameter override pattern
- **Out-of-Box Experience**: Default configurations for known platforms (Freenove, custom boards)
- **Development Flexibility**: Override pins via parameter store for testing/prototyping
- **Safety**: Validate pin assignments at initialization before hardware operations

### Considered Options

- **Option A**: Hardcoded Const Definitions
- **Option B**: Full Parameter Store Configuration
- **Option C**: Hybrid Rust Const + Parameter Override
- **Option D**: hwdef.dat-style Build-time Code Generation (Selected)

### Option Analysis

- **Option A** — Pros: Type-safe, zero runtime cost, compile-time conflict detection | Cons: No flexibility, requires recompilation for pin changes, difficult to support multiple hardware variants
- **Option B** — Pros: Maximum flexibility, one binary for all configurations | Cons: Flash overhead, runtime validation required, easy to misconfigure, no type safety
- **Option C** — Pros: Safe defaults with flexibility, matches ArduPilot pattern | Cons: Pin definitions in Rust code (less readable than declarative format), moderate complexity
- **Option D** — Pros: Declarative text format (hwdef.dat), build-time code generation (zero runtime cost), type-safe, matches ArduPilot exactly, easy to review/diff | Cons: Requires build.rs implementation, slightly more complex build process

## Rationale

Option D provides the best alignment with ArduPilot architecture and embedded systems best practices:

1. **ArduPilot Exact Match**: ArduPilot uses hwdef.dat files that are parsed at build time to generate C code. This is a proven, battle-tested pattern for autopilot board definitions.

2. **Declarative Configuration**: Text-based hwdef format is more readable and easier to review than Rust code:

   ```
   # boards/freenove_standard.hwdef
   # Freenove 4WD Car Standard Wheel Configuration

   # Motor pins (H-bridge DRV8837)
   M1_IN1 18  # Left front motor IN1
   M1_IN2 19  # Left front motor IN2
   M2_IN1 20  # Left rear motor IN1
   M2_IN2 21  # Left rear motor IN2
   M3_IN1 6   # Right front motor IN1
   M3_IN2 7   # Right front motor IN2
   M4_IN1 8   # Right rear motor IN1
   M4_IN2 9   # Right rear motor IN2

   # Optional peripherals
   BUZZER 2
   LED_WS2812 16
   BATTERY_ADC 26
   ```

3. **Build-time Code Generation**: `build.rs` parses hwdef.dat and generates type-safe Rust code:
   - Zero runtime overhead (all parsing happens at build time)
   - Compile-time type safety (generated const definitions with pin modifiers)
   - Pin conflict detection during build (not at runtime)
   - Reserved pin warnings (UART0, QSPI) prevent accidental conflicts
   - Include directive recursion detection prevents infinite loops

4. **Multi-Board Support**: Easy to add new boards by creating new hwdef files with shared configurations:

   ```
   boards/
   ├── common/
   │   └── rp2350.hwdef         # Shared platform settings
   ├── freenove_standard.hwdef  # includes common/rp2350.hwdef
   ├── freenove_mecanum.hwdef   # includes common/rp2350.hwdef
   └── custom_rover.hwdef       # includes freenove_standard.hwdef + overrides
   ```

   Select board at build time via environment variable:

   ```bash
   BOARD=freenove_standard cargo build --release
   ```

5. **Development Flexibility**: Parameter store still available for runtime overrides (testing/debugging), but defaults come from type-safe generated code.

6. **Diff-Friendly**: Text files are easier to review in version control than Rust const definitions.

Option A was rejected because pin definitions in Rust are less readable than declarative format. Option B was rejected for lack of type safety. Option C was rejected because while functional, it doesn't fully embrace ArduPilot's proven hwdef.dat pattern.

## Consequences

### Positive

- Freenove platform works out-of-box with zero configuration
- Declarative hwdef.dat format easier to read and maintain than Rust code
- Build-time code generation provides zero runtime overhead
- Type-safe generated code with pin modifiers prevents invalid configurations
- Pin conflicts detected at build time (compile errors)
- Reserved pin warnings prevent accidental conflicts with UART/SPI/QSPI
- Include directive reduces duplication across board variants
- Undef directive allows fine-grained overrides of included configurations
- Pin modifiers (PULLUP, OUTPUT, SPEED_HIGH, etc.) enable hardware-specific tuning
- Easy to add new boards (create new hwdef file, no Rust code needed)
- Diff-friendly text format for version control
- Exact alignment with ArduPilot's proven architecture

### Negative

- Requires build.rs implementation for hwdef parser with include/undef support (initial development cost)
- Slightly more complex build process (parse → resolve includes → generate → compile)
- Developers must understand hwdef.dat format and include/undef semantics (though simpler than Rust)
- Build-time selection means one binary per board (unless runtime override via parameters)
- Include directive adds complexity (recursion detection, file path resolution)
- Pin modifiers add parser complexity (optional syntax, platform-specific validation)

### Neutral

- hwdef files live in `boards/` directory with `common/` subdirectory for shared configs
- Parameter store still available for runtime overrides (testing/debugging)
- Future platforms (ESP32, STM32) follow same hwdef.dat pattern with platform-specific reserved pins
- Pin modifiers generate warnings if unsupported on platform (graceful degradation)
- Reserved pin warnings can be ignored by advanced users (intentional override)

## Implementation Notes

### Module Structure

```
project_root/
├── boards/                     # NEW: Board definition files
│   ├── common/                 # Shared platform configurations
│   │   └── rp2350.hwdef
│   ├── freenove_standard.hwdef
│   ├── freenove_mecanum.hwdef
│   └── custom_rover.hwdef
├── build.rs                    # NEW: hwdef parser with include/undef support
└── src/
    ├── platform/
    │   ├── traits/
    │   │   └── board.rs        # BoardPinConfig trait and types with modifiers
    │   └── rp2350/
    │       └── boards/
    │           └── generated.rs # AUTO-GENERATED from hwdef.dat
```

### Core Types

```rust
// src/platform/traits/board.rs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinType { Input, Output, Adc }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PullMode { None, PullUp, PullDown }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputMode { PushPull, OpenDrain }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed { Low, Medium, High, VeryHigh }

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
    pub in1: PinConfig,  // Motor IN1 with modifiers
    pub in2: PinConfig,  // Motor IN2 with modifiers
}

#[derive(Debug, Clone)]
pub struct BoardPinConfig {
    pub motors: [MotorPins; 4],        // 4 motors for Freenove
    pub buzzer: Option<PinConfig>,     // Optional buzzer with modifiers
    pub led: Option<PinConfig>,        // Optional LED with modifiers
    pub battery_adc: Option<PinConfig>,// Optional battery ADC with modifiers
}

impl BoardPinConfig {
    /// Validate pin configuration (no duplicates, valid GPIO numbers)
    pub fn validate(&self) -> Result<(), PinError> {
        // Implementation
    }

    /// Load from parameter store with board defaults as fallback
    pub fn load(params: &ParameterStore, board_default: &Self) -> Result<Self, PinError> {
        // Try parameter overrides first, fall back to board defaults
    }
}
```

### hwdef.dat Format

```
# boards/freenove_standard.hwdef
# Freenove 4WD Car Standard Wheel Configuration

# Include common platform settings
include boards/common/rp2350.hwdef

# Actuator counts
MOTOR_COUNT 4

# Motor pins (H-bridge DRV8837) with modifiers
M1_IN1 18 OUTPUT  # Left front motor IN1
M1_IN2 19 OUTPUT  # Left front motor IN2
M2_IN1 20 OUTPUT  # Left rear motor IN1
M2_IN2 21 OUTPUT  # Left rear motor IN2
M3_IN1 6 OUTPUT   # Right front motor IN1
M3_IN2 7 OUTPUT   # Right front motor IN2
M4_IN1 8 OUTPUT   # Right rear motor IN1
M4_IN2 9 OUTPUT   # Right rear motor IN2

# Optional peripherals with modifiers
BUZZER 2 OUTPUT PULLDOWN
LED_WS2812 16 OUTPUT
BATTERY_ADC 26 ADC

# Note: Reserved pins (GPIO 0-1 for UART0, GPIO 47-53 for QSPI)
# will generate warnings if assigned to actuators
```

**Common platform file** (`boards/common/rp2350.hwdef`):

```
# Common RP2350 platform settings
PLATFORM rp2350
```

### Build-time Code Generation

```rust
// build.rs
use std::env;
use std::fs;
use std::path::Path;
use std::collections::HashSet;

fn main() {
    let board = env::var("BOARD").unwrap_or_else(|_| "freenove_standard".to_string());
    let hwdef_path = format!("boards/{}.hwdef", board);

    // Parse hwdef.dat with include resolution
    let config = parse_hwdef_with_includes(&hwdef_path, &mut HashSet::new())?;

    // Validate (duplicates, GPIO ranges, reserved pins)
    validate_hwdef(&config)?;

    // Generate Rust code with pin modifiers
    let generated_code = generate_board_config(&config);

    // Write to OUT_DIR
    let out_dir = env::var("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("board_config.rs");
    fs::write(&dest_path, generated_code).unwrap();
}

fn parse_hwdef_with_includes(path: &str, include_chain: &mut HashSet<String>) -> Result<HwDefConfig, BuildError> {
    // Detect include recursion
    if include_chain.contains(path) {
        return Err(BuildError::IncludeRecursion(include_chain.clone()));
    }
    include_chain.insert(path.to_string());

    // Read and parse file
    let content = fs::read_to_string(path)?;
    let mut config = HwDefConfig::new();

    for (line_num, line) in content.lines().enumerate() {
        if line.trim().starts_with("include ") {
            let include_path = line.trim_start_matches("include ").trim();
            let included = parse_hwdef_with_includes(include_path, include_chain)?;
            config.merge(included);
        } else if line.trim().starts_with("undef ") {
            let pin_name = line.trim_start_matches("undef ").trim();
            config.remove_pin(pin_name);
        } else {
            // Parse regular directives (PLATFORM, M1_IN1, etc.)
            config.parse_line(line, line_num)?;
        }
    }

    include_chain.remove(path);
    Ok(config)
}

fn validate_hwdef(config: &HwDefConfig) -> Result<(), BuildError> {
    // Check duplicates, GPIO ranges, reserved pins (warnings)
    validate_duplicates(config)?;
    validate_gpio_ranges(config)?;
    validate_reserved_pins(config)?; // Emits warnings, not errors
    validate_pin_modifiers(config)?; // Check platform support
    Ok(())
}

fn generate_board_config(config: &HwDefConfig) -> String {
    // Generate const definitions with pin modifiers:
    // pub const BOARD_CONFIG: BoardPinConfig = BoardPinConfig {
    //     motors: [
    //         MotorPins {
    //             in1: PinConfig { gpio: 18, pin_type: PinType::Output, ... },
    //             in2: PinConfig { gpio: 19, pin_type: PinType::Output, ... },
    //         },
    //         ...
    //     ],
    //     ...
    // };
}
```

### Generated Code Example

```rust
// target/OUT_DIR/board_config.rs (auto-generated)
use crate::platform::traits::board::{BoardPinConfig, MotorPins, PinConfig, PinType, PullMode, OutputMode, Speed};

pub const BOARD_CONFIG: BoardPinConfig = BoardPinConfig {
    motors: [
        MotorPins {
            in1: PinConfig { gpio: 18, pin_type: PinType::Output, pull: PullMode::None, output_mode: OutputMode::PushPull, speed: Speed::Medium },
            in2: PinConfig { gpio: 19, pin_type: PinType::Output, pull: PullMode::None, output_mode: OutputMode::PushPull, speed: Speed::Medium },
        }, // M1 (left-front)
        MotorPins {
            in1: PinConfig { gpio: 20, pin_type: PinType::Output, pull: PullMode::None, output_mode: OutputMode::PushPull, speed: Speed::Medium },
            in2: PinConfig { gpio: 21, pin_type: PinType::Output, pull: PullMode::None, output_mode: OutputMode::PushPull, speed: Speed::Medium },
        }, // M2 (left-rear)
        MotorPins {
            in1: PinConfig { gpio: 6, pin_type: PinType::Output, pull: PullMode::None, output_mode: OutputMode::PushPull, speed: Speed::Medium },
            in2: PinConfig { gpio: 7, pin_type: PinType::Output, pull: PullMode::None, output_mode: OutputMode::PushPull, speed: Speed::Medium },
        }, // M3 (right-front)
        MotorPins {
            in1: PinConfig { gpio: 8, pin_type: PinType::Output, pull: PullMode::None, output_mode: OutputMode::PushPull, speed: Speed::Medium },
            in2: PinConfig { gpio: 9, pin_type: PinType::Output, pull: PullMode::None, output_mode: OutputMode::PushPull, speed: Speed::Medium },
        }, // M4 (right-rear)
    ],
    buzzer: Some(PinConfig { gpio: 2, pin_type: PinType::Output, pull: PullMode::PullDown, output_mode: OutputMode::PushPull, speed: Speed::Medium }),
    led: Some(PinConfig { gpio: 16, pin_type: PinType::Output, pull: PullMode::None, output_mode: OutputMode::PushPull, speed: Speed::Medium }),
    battery_adc: Some(PinConfig { gpio: 26, pin_type: PinType::Adc, pull: PullMode::None, output_mode: OutputMode::PushPull, speed: Speed::Low }),
};
```

### Usage Pattern

```rust
// In main.rs or platform initialization
// Include generated board config
include!(concat!(env!("OUT_DIR"), "/board_config.rs"));

// Use generated config as default
let pin_config = BoardPinConfig::load(&param_store, &BOARD_CONFIG)?;

// Validate before use
pin_config.validate()?;

// Initialize motors with validated config
let motors = MotorGroup::new(&pin_config, &platform)?;
```

### Build Commands

```bash
# Build for Freenove Standard Wheel (default)
cargo build --release

# Build for custom board
BOARD=custom_rover cargo build --release

# Build for Freenove Mecanum Wheel
BOARD=freenove_mecanum cargo build --release
```

### Parameter Store Integration

Following ArduPilot conventions, pin overrides would use a naming pattern (though note: ArduPilot typically uses board definitions, not parameters for GPIO pins):

```
# Example parameter override (for development/testing)
# Note: This is an extension pattern, not standard ArduPilot parameters
PIN_M1_IN1 = 18  # Override motor 1 IN1 pin
PIN_M1_IN2 = 19  # Override motor 1 IN2 pin
```

**IMPORTANT**: GPIO pin assignment is typically handled architecturally (board definitions), not via parameters, in ArduPilot. The parameter store provides flexibility for development/testing, not as the primary configuration method.

## Platform Considerations

- **RP2350**: GPIO0-29 valid (30 GPIO pins total)
- **Reserved Pins**: UART (GPIO0/1 by default), I2C (configurable), SPI (if enabled)
- **PWM Constraints**: RP2350 has 12 PWM slices (can support 24 PWM outputs, sufficient for 8 motor pins)
- **Pin Validation**: Check against platform-specific reserved pins

## Security & Privacy

- **Validation Required**: Invalid pin configurations must be rejected at initialization to prevent hardware damage
- **Parameter Tampering**: Pin configuration changes should require armed=false state (safety check)

## Monitoring & Logging

- Log active pin configuration at initialization for debugging
- Warn if parameter overrides differ from board defaults
- Error clearly when pin conflicts detected

## Open Questions

- [x] Should hwdef parser be implemented in build.rs or as a separate crate? → **Decision**: Implement in build.rs for simplicity, extract to crate if reused by other projects
- [x] Do we need per-motor enable/disable flags (skip unused motors) in hwdef.dat? → **Decision**: Use MOTOR_COUNT directive to specify number of motors
- [x] Should pin validation happen at build time (in build.rs) or at initialization? → **Decision**: Build-time validation for hwdef (duplicates, invalid GPIO), runtime for parameter overrides
- [x] What hwdef.dat syntax should we support (subset of ArduPilot's format or full compatibility)? → **Decision**: Subset with key features (include, undef, pin modifiers), excluding MCU-specific settings
- [x] hwdef.dat specification needs to be documented → **Completed**: `docs/hwdef-specification.md` defines complete syntax and validation rules
- [x] Should reserved pins fail the build or generate warnings? → **Decision**: Warnings only, allowing advanced users to override if needed
- [x] How should include recursion be handled? → **Decision**: Track include chain, detect cycles, fail with clear error message
- [x] What pin modifiers should be supported? → **Decision**: PULLUP, PULLDOWN, INPUT, OUTPUT, ADC, OPENDRAIN, SPEED_LOW/MEDIUM/HIGH (platform-specific)

## External References

- [ArduPilot Hardware Definition Files](https://ardupilot.org/dev/docs/building-the-code.html#board-specific-features) - Board definition pattern reference
- [ArduPilot hwdef.dat Format](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef) - Example hwdef.dat files
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - GPIO capabilities and constraints
- [Freenove Documentation](https://docs.freenove.com/projects/fnk0089/en/latest/fnk0089/codes/Standard/2_Module_test_.html) - Freenove pin assignments
- [Cargo Build Scripts](https://doc.rust-lang.org/cargo/reference/build-scripts.html) - build.rs documentation

---

## Template Usage

This ADR follows the structure defined in [docs/templates/adr.md](../templates/adr.md).
