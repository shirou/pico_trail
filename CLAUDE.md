# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

@AGENTS.md

## Mandatory Pre-Work Verification Process

**⚠️ For ALL work, the following MUST be completed before starting any task:**

### Required Checklist

- [ ] CLAUDE.md/AGENTS.md have been re-read
- [ ] Relevant template documents have been reviewed (docs/templates/)
- [ ] Existing similar documents have been referenced to understand format
- [ ] Project-specific standards > general knowledge priority has been confirmed

### Additional Verification for Documentation Tasks

- [ ] Applicable template has been read and loaded first
- [ ] Template structure is being followed completely
- [ ] Completed work has been re-verified against template compliance

**If this verification process is skipped, immediately STOP work and report to the user.**
**When uncertain or unclear about anything, do NOT interpret independently - always confirm with the user.**

### Rationale

Ignoring project-established documentation standards invalidates the team's entire documentation effort and destroys quality consistency. Prioritizing project-specific standards over general knowledge is essential for collaborative development.

## Parameter Standards

**⚠️ MANDATORY: All parameters in documentation MUST follow ArduPilot standards.**

### Core Principle

**Do NOT create custom parameters.** This project follows ArduPilot's parameter conventions strictly. All parameters referenced in documentation (Analysis, Requirements, ADRs, Tasks) must be standard ArduPilot parameters that exist in the official ArduPilot Rover/Copter/Plane parameter lists.

### Parameter Verification Process

Before documenting any parameter:

1. **Search ArduPilot documentation**: Verify the parameter exists at <https://ardupilot.org/rover/docs/parameters.html> (or Copter/Plane)
2. **Check parameter prefixes**: Ensure the prefix follows ArduPilot conventions (e.g., `FS_*`, `ARMING_*`, `BATT*_*`, `MODE*`, `ATC_*`, `RCx_OPTION`)
3. **Use exact names**: Parameter names must match ArduPilot exactly (case-sensitive)
4. **Document standard behavior**: Describe how ArduPilot uses the parameter, not how you think it should work

### Approved ArduPilot Parameter Families

Use these standard ArduPilot parameter families:

- **Failsafe**: `FS_*` (e.g., `FS_ACTION`, `FS_TIMEOUT`, `FS_GCS_ENABLE`)
- **Arming**: `ARMING_*` (e.g., `ARMING_CHECK`, `ARMING_REQUIRE`, `ARMING_RUDDER`, `ARMING_OPTIONS`)
- **Battery**: `BATT*_*` (e.g., `BATT_LOW_VOLT`, `BATT_CRT_VOLT`, `BATT_FS_LOW_ACT`)
- **Mode**: `MODE_CH`, `MODE1-6`, `INITIAL_MODE`, `FLTMODE_*`
- **Attitude Control**: `ATC_*` (e.g., `ATC_STR_RAT_P`, `ATC_SPEED_P`)
- **RC Options**: `RCx_OPTION` (e.g., `RC7_OPTION=31` for Motor Emergency Stop)
- **Motor**: `MOT_*` (e.g., `MOT_SAFE_DISARM`)
- **Logging**: `LOG_*` (e.g., `LOG_DISARMED`, `LOG_BITMASK`)

### When ArduPilot Lacks a Parameter

If ArduPilot does not have a parameter for a specific feature:

1. **DO NOT invent one** - Document the feature without parameter configuration
2. **Use architectural approach** - Implement as hardcoded behavior or trait-based design
3. **Note the gap** - Document that ArduPilot handles this architecturally, not via parameters
4. **Example**: Emergency stop is handled via `RCx_OPTION=31` and `stop_vehicle()` function, not via `ESTOP_*` parameters

### Approved Custom Parameters (Exceptions)

**⚠️ These are intentional exceptions to the "no custom parameters" policy:**

#### PIN\_\* - Board Pin Configuration Parameters

ArduPilot uses `hwdef.dat` files for compile-time pin configuration. This project follows the same pattern with `boards/*.hwdef` files. However, for development and testing flexibility, we provide runtime parameter overrides:

- **Parameters**: `PIN_M1_IN1`, `PIN_M1_IN2`, `PIN_M2_IN1`, `PIN_M2_IN2`, `PIN_M3_IN1`, `PIN_M3_IN2`, `PIN_M4_IN1`, `PIN_M4_IN2`, `PIN_BUZZER`, `PIN_LED`, `PIN_BATTERY_ADC`
- **Default values**: Loaded from `boards/*.hwdef` at build time
- **Usage**: Production uses hwdef files, development/testing can override via Mission Planner
- **Safety**: Pin changes require reboot and validation before hardware use
- **Documentation**: See `src/parameters/board.rs` for implementation details

### Verification Commands

```bash
# Search ArduPilot documentation for parameters
# Rover: https://ardupilot.org/rover/docs/parameters.html
# Copter: https://ardupilot.org/copter/docs/parameters.html
# Plane: https://ardupilot.org/plane/docs/parameters.html
```

**If uncertain, ask the user to verify against ArduPilot documentation before proceeding.**
