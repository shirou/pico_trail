# T-po5ns Pin Configuration Management

## Metadata

- Type: Implementation Plan
- Status: Complete

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement GPIO pin configuration management using hwdef.dat-style board definition files with build-time code generation. This task creates the infrastructure to parse declarative text files at build time, generate type-safe Rust const definitions, and integrate with the parameter store for runtime overrides.

## Success Metrics

- [ ] Freenove Standard Wheel hwdef.dat file created and working
- [ ] Build-time pin conflict detection functional (duplicate pins cause compile errors)
- [ ] Include directive with recursion detection functional
- [ ] Undef directive allows overriding included definitions
- [ ] Pin modifiers (PinType, PullMode, OutputMode, Speed) supported
- [ ] Reserved pins warnings (non-fatal) for RP2350 UART0/QSPI pins
- [ ] BoardPinConfig types provide type-safe pin access
- [ ] Parameter store can override pins for development
- [ ] Common platform hwdef (boards/common/rp2350.hwdef) created and reusable
- [ ] All existing tests pass; no regressions in platform initialization

## Scope

- Goal: Enable declarative GPIO pin configuration with build-time validation and type-safe runtime access, supporting include directives, pin modifiers, and reserved pins warnings
- Non-Goals:
  - Multiple board support in single binary (one board per build initially)
  - Advanced ArduPilot hwdef features (MCU settings, DMA, complex sensor configs)
  - Hot-swapping pins at runtime (parameter overrides require initialization restart)
- Assumptions:
  - RP2350 platform only (initial version)
  - Parameter store infrastructure exists
  - Motor driver abstraction will consume BoardPinConfig
- Constraints:
  - No_std embedded environment (no runtime file I/O)
  - Limited flash memory (keep generated code minimal)

## ADR & Legacy Alignment

- [ ] Confirm ADR-mcg03-pin-configuration-management is referenced above
- [ ] No legacy pin configuration system exists (new feature)
- [ ] Motor driver code will be updated in separate task to consume BoardPinConfig

## Plan Summary

- Phase 1 – Core Types and hwdef Parser (foundation scaffolding)
- Phase 2 – Code Generation and Build Integration (code generation)
- Phase 3 – Parameter Store Integration and Testing (runtime integration and validation)

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it (e.g., strike-through with a brief note) instead of leaving it unchecked.

---

## Phase 1: Core Types and hwdef Parser

### Goal

- Create BoardPinConfig types and implement hwdef.dat parser with validation

### Inputs

- Documentation:
  - `docs/adr/ADR-mcg03-pin-configuration-management.md` – Architecture decision and format specification
  - `docs/requirements/FR-h47nw-pin-configuration-management.md` – Functional requirements
- Source Code to Modify:
  - N/A (new files)
- Dependencies:
  - Internal: None (foundation layer)
  - External crates: None (standard library for build.rs)

### Tasks

- [x] **Create board types module**
  - [x] Create `src/platform/traits/board.rs`
  - [x] Define `MotorPins` struct (in1, in2 fields)
  - [x] Define `BoardPinConfig` struct (motors array, optional peripherals)
  - [x] Define `PinError` enum (DuplicatePin, InvalidGpio, ParameterParseError)
  - [x] Add module to `src/platform/traits/mod.rs`
- [x] **Add pin modifier types**
  - [x] Define `PinType` enum (Input, Output, Adc)
  - [x] Define `PullMode` enum (None, PullUp, PullDown)
  - [x] Define `OutputMode` enum (PushPull, OpenDrain)
  - [x] Define `Speed` enum (Low, Medium, High, VeryHigh)
  - [x] Define `PinConfig` struct with gpio + all modifiers
  - [x] Update `MotorPins` to use `PinConfig` instead of `u8`
  - [x] Update `BoardPinConfig` to use `PinConfig` for peripherals
- [x] **Implement hwdef parser in build.rs**
  - [x] Create or modify `build.rs` in project root
  - [x] Implement hwdef.dat text parser (handle comments, blank lines)
  - [x] Parse PLATFORM directive
  - [x] Parse motor pin assignments (M1_IN1, M1_IN2, etc.)
  - [x] Parse optional peripheral pins (BUZZER, LED_WS2812, BATTERY_ADC)
  - [x] Create HwDefConfig struct to hold parsed data
- [x] **Add pin modifier parsing**
  - [x] Parse pin modifiers from hwdef lines (INPUT, OUTPUT, ADC, PULLUP, PULLDOWN, etc.)
  - [x] Build PinConfig from parsed modifiers
  - [x] Set sensible defaults (Output for motors, Input for sensors, etc.)
  - [x] Store PinConfig in HwDefConfig instead of raw u8
- [x] **Implement include directive**
  - [x] Parse `include <path>` directive
  - [x] Resolve relative paths from current hwdef location
  - [x] Recursively parse included hwdef files
  - [x] Detect include cycles using HashSet<PathBuf>
  - [x] Report clear error on circular includes with chain
- [x] **Implement undef directive**
  - [x] Parse `undef <name>` directive
  - [x] Remove previously defined pin from HwDefConfig
  - [x] Allow undef to override included definitions
  - [x] Warn if undef targets non-existent pin (optional)
- [x] **Implement build-time validation**
  - [x] Validate GPIO numbers against platform range (RP2350: 0-29)
  - [x] Detect duplicate pin assignments
  - [x] Report clear error messages with line numbers
  - [x] Add unit tests in build.rs for parser and validator
- [x] **Add reserved pins warnings**
  - [x] Detect RP2350 reserved pins (GPIO 0-1 for UART0, GPIO 47-53 for QSPI)
  - [x] Emit non-fatal warnings (not errors) for reserved pins
  - [x] Format warnings clearly with pin name and reason
  - [x] Allow builds to continue with warnings
- [x] **Add pin modifier validation**
  - [x] Warn about unsupported modifiers on RP2350 (OPENDRAIN, SPEED\_\*)
  - [x] Ensure ADC pins use valid ADC channels (GPIO 26-29 on RP2350)
  - [x] Validate modifier combinations (e.g., PULLUP + PULLDOWN conflict)

### Deliverables

- `src/platform/traits/board.rs` with type definitions
- `build.rs` with hwdef parser and validation logic
- Parser unit tests in build.rs

### Verification

```bash
# Build and checks (will fail until code generation added in Phase 2)
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
# Run build script tests
cargo test --package pico_trail --lib --quiet build
```

### Acceptance Criteria (Phase Gate)

- BoardPinConfig types compile without errors
- hwdef parser can read valid hwdef.dat files
- Validation detects duplicate pins and invalid GPIO numbers
- Parser unit tests pass

### Rollback/Fallback

- Delete `src/platform/traits/board.rs` if types are incorrect
- Revert build.rs changes if parser introduces build issues

---

## Phase 2: Code Generation and Build Integration

### Phase 2 Goal

- Generate Rust const definitions from hwdef.dat and integrate into build system

### Phase 2 Inputs

- Dependencies:
  - Phase 1: hwdef parser and BoardPinConfig types
  - `boards/` directory (to be created)
- Source Code to Modify:
  - `build.rs` – Add code generator
  - Create `boards/freenove_standard.hwdef`

### Phase 2 Tasks

- [x] **Implement code generator**
  - [x] Add generator function in build.rs
  - [x] Generate const `BOARD_CONFIG: BoardPinConfig` from HwDefConfig
  - [x] Generate PinConfig constants with all modifiers (pin_type, pull, output_mode, speed)
  - [x] Handle include resolution before code generation
  - [x] Write generated code to `OUT_DIR/board_config.rs`
  - [x] Add `cargo:rerun-if-changed` for all hwdef files (including included files)
- [x] **Create common platform hwdef**
  - [x] Create `boards/common/` directory
  - [x] Create `boards/common/rp2350.hwdef` with platform-wide defaults
  - [x] Add PLATFORM RP2350 directive
  - [x] Add MOTOR_COUNT and common settings
  - [x] Document intended for inclusion by board-specific files
- [x] **Create Freenove hwdef file**
  - [x] Create `boards/` directory
  - [x] Create `boards/freenove_standard.hwdef` using include directive
  - [x] Include `boards/common/rp2350.hwdef`
  - [x] Define motor pins with modifiers (M1_IN1 18 OUTPUT, M1_IN2 19 OUTPUT PULLDOWN, etc.)
  - [x] Add optional peripherals with modifiers (BUZZER 2 OUTPUT PULLDOWN, LED_WS2812 16 OUTPUT, BATTERY_ADC 26 ADC)
  - [x] Add comments documenting pin assignments
- [x] **Integrate generated code**
  - [x] Add build.rs BOARD environment variable handling (default: freenove_standard)
  - [x] Include generated code in application (main.rs or platform module)
  - [x] Verify BOARD_CONFIG const is accessible
- [x] **Test build system**
  - [x] Build with default board (freenove_standard)
  - [x] Build with BOARD=freenove_standard explicitly
  - [x] Verify generated code in OUT_DIR includes PinConfig with modifiers
  - [x] Test include directive works (freenove includes common/rp2350)
  - [x] Test undef directive (create test hwdef that undefines and redefines a pin)

### Phase 2 Deliverables

- `build.rs` with code generator supporting include/undef/modifiers
- `boards/common/rp2350.hwdef` common platform file
- `boards/freenove_standard.hwdef` with include and modifiers
- Generated `board_config.rs` in OUT_DIR with PinConfig structs
- Application includes and uses BOARD_CONFIG

### Phase 2 Verification

```bash
cargo clean
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo build --release
# Verify generated code includes PinConfig with modifiers
cat target/debug/build/pico_trail-*/out/board_config.rs
# Test reserved pins warning (should see warning for GPIO 0-1)
# Create test hwdef using UART pins and verify warning appears in build output
# Test include recursion detection
# Create circular include and verify error with clear chain
```

### Phase 2 Acceptance Criteria

- Build succeeds with freenove_standard.hwdef
- Generated code contains PinConfig with all modifiers (pin_type, pull, output_mode, speed)
- Include directive works (freenove includes common/rp2350)
- Undef directive works (can override included definitions)
- Reserved pins warnings appear (non-fatal) for GPIO 0-1, 47-53 on RP2350
- Include recursion detected with clear error message
- BOARD_CONFIG const accessible in application
- Invalid hwdef causes compile error with clear message

### Phase 2 Rollback/Fallback

- Revert build.rs generator if code generation fails
- Delete boards/ directory if not needed

---

## Phase 3: Parameter Store Integration and Testing

### Phase 3 Goal

- Implement runtime configuration loading and comprehensive testing

### Phase 3 Tasks

- [x] **Implement BoardPinConfig methods**
  - [x] Implement `validate()` method (detect duplicates, check GPIO ranges)
  - [x] Add validation for pin modifier compatibility
  - [x] Implement `load()` method (parameter store integration)
  - [x] Handle parameter overrides (e.g., PIN_M1_IN1)
  - [x] Fall back to board defaults if no override
- [x] **Add runtime validation**
  - [x] Validate after load() before hardware initialization
  - [x] Check for unsupported modifiers at runtime (if needed)
  - [x] Validate ADC pins use valid channels (GPIO 26-29)
  - [x] Return clear PinError variants
  - [x] Test error paths (duplicate from override, invalid GPIO)
- [x] **Testing**
  - [x] Unit tests for validate() method
  - [x] Unit tests for load() method
  - [x] Integration test: Build with valid hwdef (verified via cargo build)
  - [x] Integration test: Build with invalid hwdef (covered by build.rs tests in Phase 1-2)
  - [x] Integration test: Build with reserved pins (covered by build.rs tests in Phase 1-2)
  - [x] Integration test: Build with circular include (covered by build.rs tests in Phase 1-2)
  - [x] Integration test: Build with undef directive (covered by build.rs tests in Phase 1-2)
  - [x] Integration test: Build with pin modifiers (verified via cargo build)
  - [x] Integration test: Load with parameter overrides (tested via unit tests)
  - [x] Integration test: Validate detects override conflicts (tested via unit tests)

### Phase 3 Deliverables

- BoardPinConfig::validate() and ::load() implementation
- Comprehensive unit and integration tests
- Documentation updates (if needed)

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
# Unit tests
cargo test --lib --quiet platform::traits::board
# Integration tests (if created)
cargo test --quiet
# Full build verification
cargo build --release
```

### Phase 3 Acceptance Criteria

- validate() detects all conflict scenarios including modifier conflicts
- validate() checks ADC pins use valid channels
- load() correctly applies parameter overrides
- Tests cover happy path and error cases
- Tests verify reserved pins warnings (non-fatal)
- Tests verify include/undef functionality
- Tests verify pin modifier parsing and code generation
- No regressions in existing tests

---

## Definition of Done

- [x] `cargo check`
- [x] `cargo fmt`
- [x] `cargo clippy --all-targets -- -D warnings`
- [x] `cargo test --lib --quiet`
- [x] Integration tests pass (if created)
- [x] Freenove Standard Wheel hwdef file created and working
- [x] Common platform hwdef (boards/common/rp2350.hwdef) created
- [x] Include directive with recursion detection functional
- [x] Undef directive allows overriding included definitions
- [x] Pin modifiers (PinType, PullMode, OutputMode, Speed) supported
- [x] Reserved pins warnings (non-fatal) for RP2350 UART0/QSPI pins
- [x] Generated code compiles without errors
- [x] Generated code includes PinConfig with all modifiers
- [x] Parameter overrides functional
- [x] Build-time validation detects conflicts
- [x] Documentation updated (hwdef format documented in specification and ADR)
- [x] No `unsafe` and no vague naming (no "manager"/"util")

## Open Questions

- [x] ~~Should we create `docs/hwdef-specification.md` or document format in ADR only?~~ → **DECIDED**: Created `docs/hwdef-specification.md` with comprehensive format documentation
- [x] ~~Do we need examples of parameter overrides in documentation?~~ → **DECIDED**: Examples added to FR-h47nw and hwdef specification

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
