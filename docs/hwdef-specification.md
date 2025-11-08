# Hardware Definition (hwdef.dat) Specification

## Overview

Hardware definition files (`hwdef.dat`) are text-based configuration files that define GPIO pin assignments and platform-specific settings for board configurations. These files are parsed at build time to generate type-safe Rust const definitions, following ArduPilot's proven architecture pattern.

## File Format

### Basic Syntax

- **Format**: `KEY VALUE [MODIFIERS...]` (whitespace-separated)
- **Comments**: Lines starting with `#` are ignored
- **Blank lines**: Ignored
- **Case sensitivity**: Keys are case-sensitive
- **Location**: `boards/<board_name>.hwdef`
- **Include files**: Use `include` directive to include other hwdef files
- **Override definitions**: Use `undef` directive to undefine previously defined pins

### Example

```hwdef
# Freenove 4WD Standard Wheel configuration
include boards/common/rp2350.hwdef

# H-bridge motors (TB6612FNG)
MOTOR_COUNT 4
M1_IN1 18 OUTPUT
M1_IN2 19 OUTPUT
M2_IN1 20 OUTPUT
M2_IN2 21 OUTPUT
M3_IN1 6 OUTPUT
M3_IN2 7 OUTPUT
M4_IN1 8 OUTPUT
M4_IN2 9 OUTPUT

# Servos (optional, for steering or control surfaces)
SERVO_COUNT 0

# ESCs (optional, for brushless motors)
ESC_COUNT 0

# Stepper motors (optional)
STEPPER_COUNT 0

# Optional peripherals
BUZZER 2 OUTPUT PULLDOWN
LED_WS2812 16 OUTPUT
BATTERY_ADC 26 ADC
```

## File Composition Directives

### include

Includes another hwdef file, useful for sharing common platform definitions across multiple board configurations.

- **Format**: `include <relative_path>`
- **Required**: No
- **Example**: `include boards/common/rp2350.hwdef`
- **Behavior**: File contents are inserted at the location of the include directive
- **Nesting**: Includes can be nested (include files can include other files)
- **Path resolution**: Relative to project root

**Use cases**:

- Share common platform settings (GPIO ranges, reserved pins) across board variants
- Define standard peripheral configurations (UART, SPI) in one place
- Reduce duplication between similar board configurations

**Example common file** (`boards/common/rp2350.hwdef`):

```hwdef
# Common RP2350 platform settings
PLATFORM rp2350
```

### undef

Undefines a previously defined pin, allowing includes to be overridden with board-specific configurations.

- **Format**: `undef <PIN_NAME>`
- **Required**: No
- **Example**: `undef M1_IN1`
- **Behavior**: Removes the definition so it can be redefined
- **Use case**: Override included common definitions with board-specific pins

**Example**:

```hwdef
include boards/common/motors.hwdef
undef M1_IN1
M1_IN1 22  # Override with different GPIO
```

## Required Directives

### PLATFORM

Specifies the target hardware platform.

- **Format**: `PLATFORM <platform_name>`
- **Required**: Yes
- **Valid values**: `rp2350` (RP2350/Pico 2 W)
- **Example**: `PLATFORM rp2350`

**Platform-specific constraints:**

| Platform | GPIO Range | Reserved Pins                   | Notes                                             |
| -------- | ---------- | ------------------------------- | ------------------------------------------------- |
| rp2350   | 0-29       | 0-1 (UART0), 47-53 (QSPI flash) | 30 GPIO pins, QSPI flash pins (47-53) not exposed |

**Reserved pins warning**: The build system will issue warnings (not errors) if reserved pins are assigned to actuators or peripherals, as they may conflict with system functions like UART console or SPI flash.

## Actuator Count Directives

At least one actuator type must be configured (count > 0). You can mix multiple types in a single configuration.

### MOTOR_COUNT

Specifies the number of H-bridge motors (or thrusters) for this board configuration.

- **Format**: `MOTOR_COUNT <number>`
- **Required**: No (but at least one actuator type must be specified)
- **Valid range**: 0-8
- **Default**: 0
- **Example**: `MOTOR_COUNT 4`

This directive determines which motor pin definitions are required. For example, `MOTOR_COUNT 4` requires pins for motors M1, M2, M3, and M4 (8 pins total).

**Use cases**: Wheeled rovers, tracked vehicles, boats with thrusters, any H-bridge controlled actuator.

### SERVO_COUNT

Specifies the number of hobby servos for this board configuration.

- **Format**: `SERVO_COUNT <number>`
- **Required**: No
- **Valid range**: 0-8
- **Default**: 0
- **Example**: `SERVO_COUNT 2`

Each servo requires one PWM pin for control signal.

**Use cases**: Steering servos, gimbal control, control surfaces (planes), robotic arms.

### ESC_COUNT

Specifies the number of Electronic Speed Controllers for brushless motors.

- **Format**: `ESC_COUNT <number>`
- **Required**: No
- **Valid range**: 0-8
- **Default**: 0
- **Example**: `ESC_COUNT 4`

Each ESC requires one PWM pin for throttle control.

**Use cases**: Quadcopters, multi-rotors, brushless motor propulsion systems.

### STEPPER_COUNT

Specifies the number of stepper motors for this board configuration.

- **Format**: `STEPPER_COUNT <number>`
- **Required**: No
- **Valid range**: 0-8
- **Default**: 0
- **Example**: `STEPPER_COUNT 2`

Each stepper requires 3-4 pins (STEP, DIR, EN, optional MS1 for microstepping).

**Use cases**: Precision positioning, CNC, 3D printers, camera sliders.

## Pin Modifiers

Pin definitions can include optional modifiers to specify hardware configuration:

### Pin Type Modifiers

- `INPUT` - Configure pin as digital input
- `OUTPUT` - Configure pin as digital output (default for motor/servo/ESC pins)
- `ADC` - Configure pin as analog input (for ADC-capable pins)

### Pull Resistor Modifiers

- `PULLUP` - Enable internal pull-up resistor
- `PULLDOWN` - Enable internal pull-down resistor
- `NOPULL` - Disable pull resistors (default)

### Output Type Modifiers

- `OPENDRAIN` - Configure as open-drain output
- `PUSHPULL` - Configure as push-pull output (default)

### Speed Modifiers

- `SPEED_LOW` - Low slew rate (reduced EMI)
- `SPEED_MEDIUM` - Medium slew rate
- `SPEED_HIGH` - High slew rate (fast switching)
- `SPEED_VERY_HIGH` - Very high slew rate (maximum speed)

**Modifier format**: `PIN_NAME GPIO [TYPE] [PULL] [OUTPUT_TYPE] [SPEED]`

**Examples**:

```hwdef
M1_IN1 18 OUTPUT           # Motor pin, output (default settings)
BUZZER 2 OUTPUT PULLDOWN   # Buzzer with pull-down resistor
BUTTON 10 INPUT PULLUP     # Button input with pull-up
LED 16 OUTPUT OPENDRAIN    # Open-drain LED output
SERVO1_PWM 12 OUTPUT SPEED_HIGH  # Fast PWM output
```

**Platform support**: Not all platforms support all modifiers. Unsupported modifiers will generate build warnings.

## Actuator Pin Definitions

### Motor Pins (H-bridge)

Motor pins define the H-bridge control signals for each motor or thruster. Each motor requires two pins (IN1 and IN2) for direction control.

**Format**: `M<N>_IN<X> <gpio> [modifiers...]`

- `<N>`: Motor number (1-based, up to `MOTOR_COUNT`)
- `<X>`: Pin number (1 or 2)
- `<gpio>`: GPIO number
- `[modifiers...]`: Optional pin modifiers (OUTPUT, PULLUP, etc.)
- **Required**: Yes, for all motors 1 through `MOTOR_COUNT`

**Examples**:

- `M1_IN1 18` - Motor 1, IN1 signal on GPIO 18 (default: OUTPUT)
- `M1_IN2 19 OUTPUT` - Motor 1, IN2 signal on GPIO 19, explicit OUTPUT
- `M4_IN1 8 OUTPUT PULLDOWN` - Motor 4, IN1 with pull-down resistor

**Validation**: All motors from M1 to M\<MOTOR_COUNT> must have both IN1 and IN2 defined.

### Servo Pins (PWM)

Servo pins define PWM control signals for hobby servos. Each servo requires one PWM pin.

**Format**: `SERVO<N>_PWM <gpio> [modifiers...]`

- `<N>`: Servo number (1-based, up to `SERVO_COUNT`)
- `<gpio>`: GPIO number
- `[modifiers...]`: Optional pin modifiers (OUTPUT, SPEED_HIGH, etc.)
- **Required**: Yes, for all servos 1 through `SERVO_COUNT`

**Examples**:

- `SERVO1_PWM 10` - Servo 1 control signal on GPIO 10
- `SERVO2_PWM 11 OUTPUT SPEED_HIGH` - Servo 2 with high-speed PWM

**Validation**: All servos from SERVO1 to SERVO\<SERVO_COUNT> must have PWM defined.

### ESC Pins (PWM)

ESC pins define PWM throttle signals for electronic speed controllers. Each ESC requires one PWM pin.

**Format**: `ESC<N>_PWM <gpio> [modifiers...]`

- `<N>`: ESC number (1-based, up to `ESC_COUNT`)
- `<gpio>`: GPIO number
- `[modifiers...]`: Optional pin modifiers (OUTPUT, SPEED_HIGH, etc.)
- **Required**: Yes, for all ESCs 1 through `ESC_COUNT`

**Examples**:

- `ESC1_PWM 12` - ESC 1 throttle signal on GPIO 12
- `ESC2_PWM 13 OUTPUT SPEED_HIGH` - ESC 2 with high-speed PWM

**Validation**: All ESCs from ESC1 to ESC\<ESC_COUNT> must have PWM defined.

### Stepper Pins

Stepper pins define control signals for stepper motor drivers. Each stepper requires 3-4 pins.

**Format**:

- `STEPPER<N>_STEP <gpio> [modifiers...]` - Step pulse (required)
- `STEPPER<N>_DIR <gpio> [modifiers...]` - Direction control (required)
- `STEPPER<N>_EN <gpio> [modifiers...]` - Enable signal (required)
- `STEPPER<N>_MS1 <gpio> [modifiers...]` - Microstepping pin 1 (optional)

**Examples**:

```hwdef
STEPPER1_STEP 14 OUTPUT
STEPPER1_DIR 15 OUTPUT
STEPPER1_EN 16 OUTPUT PULLDOWN
STEPPER1_MS1 17 OUTPUT
```

**Validation**: All steppers from STEPPER1 to STEPPER\<STEPPER_COUNT> must have STEP, DIR, and EN defined. MS1 is optional.

### Pin Conflict Validation

All actuator pin assignments are checked for:

1. GPIO numbers within platform-specific valid range
2. No duplicate GPIO assignments across all actuator types and peripherals (detected at build time)

## Optional Peripheral Pins

### BUZZER

GPIO pin for piezo buzzer output.

- **Format**: `BUZZER <gpio> [modifiers...]`
- **Required**: No
- **Example**: `BUZZER 2 OUTPUT PULLDOWN`

### LED_WS2812

GPIO pin for WS2812/NeoPixel addressable LED data line.

- **Format**: `LED_WS2812 <gpio> [modifiers...]`
- **Required**: No
- **Example**: `LED_WS2812 16 OUTPUT`

### BATTERY_ADC

ADC channel number for battery voltage monitoring.

- **Format**: `BATTERY_ADC <channel> [modifiers...]`
- **Required**: No
- **Valid range**: Platform-dependent (RP2350: 0-4, where channel 4 is internal temperature sensor)
- **Example**: `BATTERY_ADC 26 ADC`

**Note**: On RP2350, ADC-capable GPIOs are GPIO 26-29 (ADC channels 0-3).

## Build-Time Validation

The hwdef parser performs the following checks at build time:

### Syntax Validation

- All lines must follow `KEY VALUE` format
- Unknown keys cause compilation error
- Invalid GPIO numbers (non-numeric) cause compilation error

### Required Fields

- `PLATFORM` must be specified
- At least one actuator type must be configured (MOTOR_COUNT, SERVO_COUNT, ESC_COUNT, or STEPPER_COUNT > 0)
- All pins for configured actuators must be defined:
  - Motors: M1_IN1 through M\<MOTOR_COUNT>\_IN2
  - Servos: SERVO1_PWM through SERVO\<SERVO_COUNT>\_PWM
  - ESCs: ESC1_PWM through ESC\<ESC_COUNT>\_PWM
  - Steppers: STEPPER1_STEP/DIR/EN through STEPPER\<STEPPER_COUNT>\_STEP/DIR/EN (MS1 optional)

### GPIO Validation

- GPIO numbers must be within platform-specific valid range
- Duplicate GPIO assignments cause compilation error with list of conflicts
- Reserved platform pins (UART, SPI, QSPI flash) generate build warnings (not errors) when assigned to actuators or peripherals

### Reserved Pins Warning

The build system checks for assignments to platform-reserved pins and issues warnings:

**RP2350 reserved pins**:

- GPIO 0-1: UART0 (TX/RX) - Console/debug interface
- GPIO 47-53: QSPI flash - Bootloader and firmware storage (internal, not user-accessible)

**Warning format**:

```
warning: GPIO 0 is reserved for UART0_TX on RP2350
  --> boards/custom_board.hwdef:10:8
   |
10 | M1_IN1 0
   |        ^ Consider using a different GPIO for motor control
   |
   = note: This may conflict with console output or debugging
```

**Rationale**: Reserved pin warnings allow advanced users to override defaults if needed (e.g., disabling UART console to free GPIO 0-1), while protecting typical users from accidental conflicts.

### Error Message Format

Build errors and warnings include file name, line number, and clear description:

**Errors** (compilation fails):

```
error: GPIO 18 used multiple times
  --> boards/freenove_standard.hwdef:15:8
   |
15 | SERVO1_PWM 18
   |            ^^ GPIO 18 already assigned to M1_IN1 (line 6)

error: GPIO 35 invalid for M1_IN1 (platform rp2350, valid range: 0-29)
  --> boards/custom_board.hwdef:12:12
   |
12 | M1_IN1 35
   |        ^^ GPIO number out of range

error: Missing required pin M2_IN2 for motor 2
  --> boards/minimal.hwdef
   |
   = note: MOTOR_COUNT is 2, but M2_IN2 is not defined

error: At least one actuator type must be configured
  --> boards/empty.hwdef
   |
   = help: Set MOTOR_COUNT, SERVO_COUNT, ESC_COUNT, or STEPPER_COUNT > 0
```

**Warnings** (compilation succeeds):

```
warning: GPIO 0 is reserved for UART0_TX on RP2350
  --> boards/custom_board.hwdef:10:8
   |
10 | M1_IN1 0
   |        ^ Consider using a different GPIO for motor control
```

## Extending the Format

### Adding New Pin Types

To add support for new peripheral pins:

1. Update `build.rs` parser to recognize the new key
2. Add validation in `validate_hwdef()`
3. Update this specification document
4. Update code generator to include the new pin in generated output

### Adding New Platforms

To add a new platform:

1. Add platform name to `PLATFORM` validation in `parse_hwdef()`
2. Add GPIO range and reserved pins validation in `validate_hwdef()`
3. Update the platform constraints table in this document
4. Create common hwdef file in `boards/common/<platform>.hwdef`
5. Test with board-specific hwdef files

### Adding New Pin Modifiers

To add a new pin modifier:

1. Update `build.rs` parser to recognize the modifier keyword
2. Add modifier to `PinModifiers` struct
3. Update code generator to emit modifier in generated code
4. Update this specification document
5. Add tests for the new modifier

## Design Rationale

### Why hwdef.dat?

- **Declarative**: Pin assignments are data, not code
- **Version control friendly**: Text format is easy to diff and review
- **Build-time validation**: Errors caught before flashing to hardware
- **ArduPilot alignment**: Follows proven pattern from mature autopilot project
- **Type safety**: Generated Rust code provides compile-time guarantees

### Why Board-Specific Files?

- Each hardware platform has unique pin requirements
- Single binary per board (vs. runtime board detection) reduces complexity
- Clear separation between board definitions and application logic
- Easy to add new boards without modifying source code

## ArduPilot Compatibility Notes

This hwdef format is inspired by ArduPilot's hardware definition system but simplified for embedded Rust targets:

**Features from ArduPilot**:

- Text-based declarative configuration
- Build-time parsing and code generation
- Pin conflict detection
- Include directive for shared configurations
- Undef directive for overrides
- Pin modifiers (PULLUP, PULLDOWN, etc.)
- Platform-specific validation

**Intentionally excluded** (out of scope for this project):

- MCU-specific settings (OSCILLATOR_HZ, FLASH_SIZE_KB)
- Complex sensor configurations (SPIDEV, IMU, BARO, COMPASS)
- DMA configuration (handled by HAL)
- Serial/I2C ordering (handled architecturally)
- ROMFS embedded filesystem

**Differences from ArduPilot**:

- Simplified syntax (no complex pin function syntax like `PA0 TIMx_CHy`)
- Explicit actuator count directives (MOTOR_COUNT, SERVO_COUNT)
- Focus on GPIO pin assignments rather than full peripheral configuration
- Rust code generation instead of C/C++

## Related Documentation

- [ADR-mcg03: Pin Configuration Management](adr/ADR-mcg03-pin-configuration-management.md)
- [FR-h47nw: Pin Configuration Management](requirements/FR-h47nw-pin-configuration-management.md)
- [Task T-po5ns: Pin Configuration Implementation](tasks/T-po5ns-pin-configuration/)
- [ArduPilot hwdef Reference](https://ardupilot.org/dev/docs/building-the-code.html#board-specific-features)
- [ArduPilot hwdef Examples](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef)

## Example Board Configurations

### Freenove 4WD Car (4 motors)

```hwdef
# Freenove 4WD Car Standard Wheel
include boards/common/rp2350.hwdef

MOTOR_COUNT 4

M1_IN1 18 OUTPUT
M1_IN2 19 OUTPUT
M2_IN1 20 OUTPUT
M2_IN2 21 OUTPUT
M3_IN1 6 OUTPUT
M3_IN2 7 OUTPUT
M4_IN1 8 OUTPUT
M4_IN2 9 OUTPUT

BUZZER 2 OUTPUT PULLDOWN
LED_WS2812 16 OUTPUT
BATTERY_ADC 26 ADC
```

### Minimal 2-Wheel Rover

```hwdef
PLATFORM rp2350
MOTOR_COUNT 2

M1_IN1 18
M1_IN2 19
M2_IN1 20
M2_IN2 21
```

### Custom 6-Wheel Platform

```hwdef
PLATFORM rp2350
MOTOR_COUNT 6

M1_IN1 10
M1_IN2 11
M2_IN1 12
M2_IN2 13
M3_IN1 14
M3_IN2 15
M4_IN1 16
M4_IN2 17
M5_IN1 18
M5_IN2 19
M6_IN1 20
M6_IN2 21

BATTERY_ADC 26
```

### Boat with Thrusters (4 thrusters)

```hwdef
PLATFORM rp2350
MOTOR_COUNT 4  # Thrusters use H-bridge control

# Thrusters (same interface as motors)
M1_IN1 18
M1_IN2 19
M2_IN1 20
M2_IN2 21
M3_IN1 6
M3_IN2 7
M4_IN1 8
M4_IN2 9

BATTERY_ADC 26
```

### Rover with Steering Servo

```hwdef
include boards/common/rp2350.hwdef

# Drive motors
MOTOR_COUNT 2
M1_IN1 18 OUTPUT
M1_IN2 19 OUTPUT
M2_IN1 20 OUTPUT
M2_IN2 21 OUTPUT

# Steering servo
SERVO_COUNT 1
SERVO1_PWM 10 OUTPUT SPEED_HIGH

BATTERY_ADC 26 ADC
```

### Quadcopter (4 ESCs)

```hwdef
PLATFORM rp2350

# Brushless motor ESCs
ESC_COUNT 4
ESC1_PWM 6
ESC2_PWM 7
ESC3_PWM 8
ESC4_PWM 9

BATTERY_ADC 26
```

### Mixed Actuator Platform

```hwdef
include boards/common/rp2350.hwdef

# Drive motors (differential drive)
MOTOR_COUNT 2
M1_IN1 18 OUTPUT
M1_IN2 19 OUTPUT
M2_IN1 20 OUTPUT
M2_IN2 21 OUTPUT

# Camera gimbal servos
SERVO_COUNT 2
SERVO1_PWM 10 OUTPUT SPEED_HIGH  # Pan
SERVO2_PWM 11 OUTPUT SPEED_HIGH  # Tilt

# Stepper for precision mechanism
STEPPER_COUNT 1
STEPPER1_STEP 14 OUTPUT
STEPPER1_DIR 15 OUTPUT
STEPPER1_EN 16 OUTPUT PULLDOWN

BATTERY_ADC 26 ADC
```
