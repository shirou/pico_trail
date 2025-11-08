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
- **Flexibility**: Custom pin assignments possible via parameter store without recompiling
- **Conflict Detection**: Pin conflicts detected and reported at initialization (before motor movement)
- **Type Safety**: Invalid GPIO pin numbers rejected at compile time (for static configs) or initialization (for parameter-based configs)

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
   - Compile-time type safety (generated const definitions)
   - Pin conflict detection during build (not at runtime)

4. **Multi-Board Support**: Easy to add new boards by creating new hwdef files:

   ```
   boards/
   ├── freenove_standard.hwdef
   ├── freenove_mecanum.hwdef
   └── custom_rover.hwdef
   ```

   Select board at build time via feature flag or environment variable:

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
- Type-safe generated code prevents invalid configurations
- Pin conflicts detected at build time (compile errors)
- Easy to add new boards (create new hwdef file, no Rust code needed)
- Diff-friendly text format for version control
- Exact alignment with ArduPilot's proven architecture

### Negative

- Requires build.rs implementation for hwdef parser (initial development cost)
- Slightly more complex build process (parse → generate → compile)
- Developers must understand hwdef.dat format (though simpler than Rust)
- Build-time selection means one binary per board (unless runtime override via parameters)

### Neutral

- hwdef files live in `boards/` directory at project root
- Parameter store still available for runtime overrides (testing/debugging)
- Future platforms (ESP32, STM32) follow same hwdef.dat pattern

## Implementation Notes

### Module Structure

```
project_root/
├── boards/                     # NEW: Board definition files
│   ├── freenove_standard.hwdef
│   └── custom_rover.hwdef
├── build.rs                    # NEW: hwdef parser and code generator
└── src/
    ├── platform/
    │   ├── traits/
    │   │   └── board.rs        # BoardPinConfig trait and types
    │   └── rp2350/
    │       └── boards/
    │           └── generated.rs # AUTO-GENERATED from hwdef.dat
```

### Core Types

```rust
// src/platform/traits/board.rs
#[derive(Debug, Clone)]
pub struct MotorPins {
    pub in1: u8,  // GPIO number for IN1
    pub in2: u8,  // GPIO number for IN2
}

#[derive(Debug, Clone)]
pub struct BoardPinConfig {
    pub motors: [MotorPins; 4],        // 4 motors for Freenove
    pub buzzer: Option<u8>,            // Optional buzzer GPIO
    pub led: Option<u8>,               // Optional LED GPIO (WS2812)
    pub battery_adc: Option<u8>,       // Optional battery ADC channel
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

# Platform
PLATFORM rp2350

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

# Reserved pins (documentation, validation)
# UART0_TX 0
# UART0_RX 1
```

### Build-time Code Generation

```rust
// build.rs
use std::env;
use std::fs;
use std::path::Path;

fn main() {
    let board = env::var("BOARD").unwrap_or_else(|_| "freenove_standard".to_string());
    let hwdef_path = format!("boards/{}.hwdef", board);

    println!("cargo:rerun-if-changed={}", hwdef_path);

    // Parse hwdef.dat
    let hwdef_content = fs::read_to_string(&hwdef_path)
        .expect("Failed to read hwdef file");
    let config = parse_hwdef(&hwdef_content);

    // Generate Rust code
    let generated_code = generate_board_config(&config);

    // Write to OUT_DIR
    let out_dir = env::var("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("board_config.rs");
    fs::write(&dest_path, generated_code).unwrap();
}

fn parse_hwdef(content: &str) -> HwDefConfig {
    // Parse hwdef.dat format
    // Extract pin assignments, validate GPIO numbers
    // Detect conflicts
}

fn generate_board_config(config: &HwDefConfig) -> String {
    // Generate const definitions like:
    // pub const BOARD_CONFIG: BoardPinConfig = BoardPinConfig { ... };
}
```

### Generated Code Example

```rust
// target/OUT_DIR/board_config.rs (auto-generated)
use crate::platform::traits::board::{BoardPinConfig, MotorPins};

pub const BOARD_CONFIG: BoardPinConfig = BoardPinConfig {
    motors: [
        MotorPins { in1: 18, in2: 19 }, // M1 (left-front)
        MotorPins { in1: 20, in2: 21 }, // M2 (left-rear)
        MotorPins { in1: 6, in2: 7 },   // M3 (right-front)
        MotorPins { in1: 8, in2: 9 },   // M4 (right-rear)
    ],
    buzzer: Some(2),
    led: Some(16),
    battery_adc: Some(26),
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

- [ ] Should hwdef parser be implemented in build.rs or as a separate crate? → Next step: Determine during implementation (consider reusability across platforms)
- [ ] Do we need per-motor enable/disable flags (skip unused motors) in hwdef.dat? → Next step: Defer to motor group abstraction requirements (FR-DRAFT-3)
- [ ] Should pin validation happen at build time (in build.rs) or at initialization? → Method: Implement basic validation in build.rs (duplicates, invalid GPIO), runtime for parameter overrides
- [ ] What hwdef.dat syntax should we support (subset of ArduPilot's format or full compatibility)? → Next step: Start with minimal subset (PIN definitions), expand as needed
- [ ] hwdef.dat specification needs to be documented → Next step: Create formal specification document (docs/hwdef-specification.md) defining syntax, supported directives, validation rules, and examples

## External References

- [ArduPilot Hardware Definition Files](https://ardupilot.org/dev/docs/building-the-code.html#board-specific-features) - Board definition pattern reference
- [ArduPilot hwdef.dat Format](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef) - Example hwdef.dat files
- [RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) - GPIO capabilities and constraints
- [Freenove Documentation](https://docs.freenove.com/projects/fnk0089/en/latest/fnk0089/codes/Standard/2_Module_test_.html) - Freenove pin assignments
- [Cargo Build Scripts](https://doc.rust-lang.org/cargo/reference/build-scripts.html) - build.rs documentation

---

## Template Usage

This ADR follows the structure defined in [docs/templates/adr.md](../templates/adr.md).
