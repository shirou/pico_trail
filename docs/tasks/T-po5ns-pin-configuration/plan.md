# T-po5ns Pin Configuration Management

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [design.md](design.md)

## Overview

Implement GPIO pin configuration management using hwdef.dat-style board definition files with build-time code generation. This task creates the infrastructure to parse declarative text files at build time, generate type-safe Rust const definitions, and integrate with the parameter store for runtime overrides.

## Success Metrics

- [ ] Freenove Standard Wheel hwdef.dat file created and working
- [ ] Build-time pin conflict detection functional (duplicate pins cause compile errors)
- [ ] BoardPinConfig types provide type-safe pin access
- [ ] Parameter store can override pins for development
- [ ] All existing tests pass; no regressions in platform initialization

## Scope

- Goal: Enable declarative GPIO pin configuration with build-time validation and type-safe runtime access
- Non-Goals:
  - Multiple board support in single binary (one board per build initially)
  - Advanced hwdef features beyond basic pin assignments
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

- [ ] **Create board types module**
  - [ ] Create `src/platform/traits/board.rs`
  - [ ] Define `MotorPins` struct (in1, in2 fields)
  - [ ] Define `BoardPinConfig` struct (motors array, optional peripherals)
  - [ ] Define `PinError` enum (DuplicatePin, InvalidGpio, ParameterParseError)
  - [ ] Add module to `src/platform/traits/mod.rs`
- [ ] **Implement hwdef parser in build.rs**
  - [ ] Create or modify `build.rs` in project root
  - [ ] Implement hwdef.dat text parser (handle comments, blank lines)
  - [ ] Parse PLATFORM directive
  - [ ] Parse motor pin assignments (M1_IN1, M1_IN2, etc.)
  - [ ] Parse optional peripheral pins (BUZZER, LED_WS2812, BATTERY_ADC)
  - [ ] Create HwDefConfig struct to hold parsed data
- [ ] **Implement build-time validation**
  - [ ] Validate GPIO numbers against platform range (RP2350: 0-29)
  - [ ] Detect duplicate pin assignments
  - [ ] Report clear error messages with line numbers
  - [ ] Add unit tests in build.rs for parser and validator

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

- [ ] **Implement code generator**
  - [ ] Add generator function in build.rs
  - [ ] Generate const `BOARD_CONFIG: BoardPinConfig` from HwDefConfig
  - [ ] Write generated code to `OUT_DIR/board_config.rs`
  - [ ] Add `cargo:rerun-if-changed` for hwdef files
- [ ] **Create Freenove hwdef file**
  - [ ] Create `boards/` directory
  - [ ] Create `boards/freenove_standard.hwdef` with motor pins (18,19,20,21,6,7,8,9)
  - [ ] Add optional peripherals (BUZZER=2, LED_WS2812=16, BATTERY_ADC=26)
  - [ ] Add comments documenting pin assignments
- [ ] **Integrate generated code**
  - [ ] Add build.rs BOARD environment variable handling (default: freenove_standard)
  - [ ] Include generated code in application (main.rs or platform module)
  - [ ] Verify BOARD_CONFIG const is accessible
- [ ] **Test build system**
  - [ ] Build with default board (freenove_standard)
  - [ ] Build with BOARD=freenove_standard explicitly
  - [ ] Verify generated code in OUT_DIR

### Phase 2 Deliverables

- `build.rs` with code generator
- `boards/freenove_standard.hwdef`
- Generated `board_config.rs` in OUT_DIR
- Application includes and uses BOARD_CONFIG

### Phase 2 Verification

```bash
cargo clean
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo build --release
# Verify generated code
cat target/debug/build/pico_trail-*/out/board_config.rs
```

### Phase 2 Acceptance Criteria

- Build succeeds with freenove_standard.hwdef
- Generated code contains correct pin assignments
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

- [ ] **Implement BoardPinConfig methods**
  - [ ] Implement `validate()` method (detect duplicates, check GPIO ranges)
  - [ ] Implement `load()` method (parameter store integration)
  - [ ] Handle parameter overrides (e.g., PIN_M1_IN1)
  - [ ] Fall back to board defaults if no override
- [ ] **Add runtime validation**
  - [ ] Validate after load() before hardware initialization
  - [ ] Return clear PinError variants
  - [ ] Test error paths (duplicate from override, invalid GPIO)
- [ ] **Testing**
  - [ ] Unit tests for validate() method
  - [ ] Unit tests for load() method with mock parameter store
  - [ ] Integration test: Build with valid hwdef
  - [ ] Integration test: Build with invalid hwdef (expect error)
  - [ ] Integration test: Load with parameter overrides
  - [ ] Integration test: Validate detects override conflicts

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

- validate() detects all conflict scenarios
- load() correctly applies parameter overrides
- Tests cover happy path and error cases
- No regressions in existing tests

---

## Definition of Done

- [ ] `cargo check`
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] Integration tests pass (if created)
- [ ] Freenove Standard Wheel hwdef file created and working
- [ ] Generated code compiles without errors
- [ ] Parameter overrides functional
- [ ] Build-time validation detects conflicts
- [ ] Documentation updated (hwdef format documented in ADR)
- [ ] No `unsafe` and no vague naming (no "manager"/"util")

## Open Questions

- [ ] Should we create `docs/hwdef-specification.md` or document format in ADR only? → Next step: Coordinate with documentation task, use ADR for initial version
- [ ] Do we need examples of parameter overrides in documentation? → Method: Add examples to FR-h47nw or create separate usage guide

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
