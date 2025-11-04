# FR-qj0d1 Mode Capability Declaration System

## Metadata

- Type: Functional Requirement
- Status: Draft
- Priority: P0
- Category: Mode Management / Architecture

## Links

- Parent Analysis: [AN-g5w99-mode-capability-system](../analysis/AN-g5w99-mode-capability-system.md)
- Related Analysis: [AN-r2fps-pre-arm-checks](../analysis/AN-r2fps-pre-arm-checks.md)
- Related Requirements:
  - [FR-sk7ty-mode-capability-queries](FR-sk7ty-mode-capability-queries.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  - [FR-a9rc3-mode-entry-sensor-validation](FR-a9rc3-mode-entry-sensor-validation.md)
  - [FR-9qug9-validation-capability-enforcement](FR-9qug9-validation-capability-enforcement.md)
  - [FR-n1mte-prearm-capability-enforcement](FR-n1mte-prearm-capability-enforcement.md)
  - [FR-xex86-failsafe-capability-based-selection](FR-xex86-failsafe-capability-based-selection.md)
  - [NFR-hmxli-capability-declaration-self-documenting](NFR-hmxli-capability-declaration-self-documenting.md)
  - [NFR-cy29p-capability-query-zero-allocation](NFR-cy29p-capability-query-zero-allocation.md)
  - [NFR-pvpky-capability-query-performance](NFR-pvpky-capability-query-performance.md)

## Requirement Statement

Each flight mode shall declare capabilities via query methods (sensor requirements, arming permissions, control type) to enable automated validation framework, enforce mode-specific safety requirements, and provide self-documenting mode characteristics, with all modes implementing capability trait methods that return correct requirements.

## Rationale

Declarative capability system is essential for:

- **Automated validation**: Validation framework enforces prerequisites without per-mode conditional logic
- **Safety enforcement**: Prevents unsafe scenarios (RC arming in Auto, entering Auto without GPS)
- **Self-documenting**: Mode capabilities explicit and verifiable by safety reviewers
- **Failsafe integration**: Failsafe system uses capabilities to select appropriate fallback modes
- **Compile-time safety**: Trait-based design ensures all modes declare capabilities
- **Maintainability**: Adding new capability type requires single trait extension

ArduPilot's capability query pattern provides proven approach for mode requirements declaration.

## User Story (if applicable)

As a validation framework developer, I want modes to declare their requirements via capability queries, so that I can automatically enforce prerequisites without hardcoding per-mode logic.

As a safety reviewer, I want mode capabilities clearly documented and verifiable, so that I can assess system safety requirements and validate compliance.

## Acceptance Criteria

### Sensor Requirement Capabilities

- [ ] Sensor requirement capability methods:
  - `requires_position() -> bool`: True if mode needs position estimate (e.g., Auto, RTL, Loiter)
  - `requires_velocity() -> bool`: True if mode needs velocity estimate
  - `requires_gps() -> bool`: True if mode needs GPS fix
  - `requires_imu() -> bool`: True if mode needs IMU data
  - `requires_compass() -> bool`: True if mode needs compass/heading

- [ ] Per-mode sensor implementations:
  - Manual: All sensors false (direct RC control, no sensor requirements)
  - Stabilize: IMU + compass true (heading stabilization)
  - Hold: IMU + compass true (attitude control)
  - Loiter: All sensors true (position hold requires full sensor suite)
  - Auto: All sensors true (waypoint navigation)
  - RTL: All sensors true (return home navigation)

- [ ] Validation integration:
  - Mode validation uses capability queries to check sensor availability
  - Mode entry denied if required sensors unavailable
  - Log denial: `"Cannot enter {mode}: {sensor} unavailable"`

### Arming Permission Capabilities

- [ ] Arming permission capability methods:
  - `allows_arming() -> bool`: True if vehicle can arm while in this mode
  - `allows_arming_from_transmitter() -> bool`: True if RC/transmitter can initiate arming

- [ ] Per-mode arming implementations:
  - Manual: allows_arming=true, allows_arming_from_transmitter=true (safe for RC arming)
  - Stabilize: allows_arming=true, allows_arming_from_transmitter=true
  - Hold: allows_arming=true, allows_arming_from_transmitter=true
  - Loiter: allows_arming=true, allows_arming_from_transmitter=false (prevent accidental mission start)
  - Auto: allows_arming=true, allows_arming_from_transmitter=false (safety: prevent accidental mission start)
  - RTL: allows_arming=true, allows_arming_from_transmitter=false

- [ ] Pre-arm checks integration:
  - Pre-arm checks call `allows_arming()` before allowing arm
  - RC arming checks call `allows_arming_from_transmitter()`
  - Deny arming if capability query returns false
  - Log arming denial: `"Cannot arm in {mode}"`
  - Send STATUSTEXT: "Arming not allowed in {mode}"

- [ ] Safety rationale:
  - Autonomous modes disallow RC arming to prevent accidental mission start
  - GCS arming in Auto requires deliberate action (not accidental stick pattern)

### Control Type Capabilities

- [ ] Control type capability methods:
  - `has_manual_input() -> bool`: True if mode uses pilot RC input
  - `is_autopilot_mode() -> bool`: True if mode is autonomous

- [ ] Per-mode control type implementations:
  - Manual: has_manual_input=true, is_autopilot_mode=false (direct RC control)
  - Stabilize: has_manual_input=true, is_autopilot_mode=false (RC with stabilization)
  - Hold: has_manual_input=false, is_autopilot_mode=false (stop mode, no input)
  - Loiter: has_manual_input=false, is_autopilot_mode=true (position hold)
  - Auto: has_manual_input=false, is_autopilot_mode=true (waypoint navigation)
  - RTL: has_manual_input=false, is_autopilot_mode=true (return home)

- [ ] Logical consistency:
  - Manual input modes are not autopilot modes
  - Autopilot modes do not use manual input
  - Default relationship: `is_autopilot_mode() = !has_manual_input()` (with exceptions like Hold)

- [ ] Usage integration:
  - Failsafe system uses control type for mode selection (prefer manual modes for RC loss)
  - Validation applies stricter checks for autopilot modes
  - GCS displays mode type to operator (Manual/Auto indicator)

### General Capability System

- [ ] Capability trait extension point:
  - Easy to add new capability types (extend trait, implement in all modes)
  - Compile-time verification (trait methods must be implemented)

- [ ] Documentation requirements:
  - Each capability method documented with purpose
  - Mode capability matrix in documentation (table showing all modes × capabilities)
  - Examples of validation usage

- [ ] Performance requirements:
  - Zero allocation (capability queries return primitives)
  - Fast execution (< 1 μs per query, suitable for hot path)
  - No side effects (pure functions)

## Technical Details (if applicable)

### Functional Requirement Details

**Mode Capability Trait:**

```rust
/// Mode capability declaration trait
pub trait ModeCapability {
    // ===== Sensor Requirements =====

    /// Does this mode require position estimate?
    /// Position from GPS or other localization system
    fn requires_position(&self) -> bool;

    /// Does this mode require velocity estimate?
    /// Velocity from GPS or IMU integration
    fn requires_velocity(&self) -> bool;

    /// Does this mode require GPS fix?
    /// Direct GPS satellite fix (not just position estimate)
    fn requires_gps(&self) -> bool;

    /// Does this mode require IMU data?
    /// Accelerometer and gyroscope data
    fn requires_imu(&self) -> bool;

    /// Does this mode require compass/heading?
    /// Magnetometer or heading estimate
    fn requires_compass(&self) -> bool;

    // ===== Arming Permissions =====

    /// Can vehicle arm while in this mode?
    /// False = must change mode before arming
    fn allows_arming(&self) -> bool;

    /// Can RC/transmitter initiate arming in this mode?
    /// False = only GCS can arm (prevent accidental mission start)
    fn allows_arming_from_transmitter(&self) -> bool;

    // ===== Control Type =====

    /// Does this mode use pilot RC input?
    /// True = manual/assisted control, False = autonomous
    fn has_manual_input(&self) -> bool;

    /// Is this an autopilot/autonomous mode?
    /// True = mode navigates without pilot input
    fn is_autopilot_mode(&self) -> bool;
}
```

**Mode Capability Matrix:**

| Mode      | Position | Velocity | GPS | IMU | Compass | Allows Arm | RC Arm | Manual Input | Autopilot |
| --------- | -------- | -------- | --- | --- | ------- | ---------- | ------ | ------------ | --------- |
| Manual    | ❌       | ❌       | ❌  | ❌  | ❌      | ✅         | ✅     | ✅           | ❌        |
| Stabilize | ❌       | ❌       | ❌  | ✅  | ✅      | ✅         | ✅     | ✅           | ❌        |
| Hold      | ❌       | ❌       | ❌  | ✅  | ✅      | ✅         | ✅     | ❌           | ❌        |
| Loiter    | ✅       | ✅       | ✅  | ✅  | ✅      | ✅         | ❌     | ❌           | ✅        |
| Auto      | ✅       | ✅       | ✅  | ✅  | ✅      | ✅         | ❌     | ❌           | ✅        |
| RTL       | ✅       | ✅       | ✅  | ✅  | ✅      | ✅         | ❌     | ❌           | ✅        |

**Example Mode Implementation (Manual):**

```rust
pub struct ManualMode {
    // ... mode state ...
}

impl ModeCapability for ManualMode {
    // ===== Sensor Requirements =====
    // Manual mode has no sensor requirements (direct RC → actuators)

    fn requires_position(&self) -> bool { false }
    fn requires_velocity(&self) -> bool { false }
    fn requires_gps(&self) -> bool { false }
    fn requires_imu(&self) -> bool { false }
    fn requires_compass(&self) -> bool { false }

    // ===== Arming Permissions =====
    // Manual mode allows arming from any source

    fn allows_arming(&self) -> bool { true }
    fn allows_arming_from_transmitter(&self) -> bool { true }

    // ===== Control Type =====
    // Manual mode uses RC input, not autonomous

    fn has_manual_input(&self) -> bool { true }
    fn is_autopilot_mode(&self) -> bool { false }
}
```

**Example Mode Implementation (Auto):**

```rust
pub struct AutoMode {
    // ... mode state ...
}

impl ModeCapability for AutoMode {
    // ===== Sensor Requirements =====
    // Auto mode requires full sensor suite for waypoint navigation

    fn requires_position(&self) -> bool { true }
    fn requires_velocity(&self) -> bool { true }
    fn requires_gps(&self) -> bool { true }
    fn requires_imu(&self) -> bool { true }
    fn requires_compass(&self) -> bool { true }

    // ===== Arming Permissions =====
    // Auto mode allows GCS arming only (prevent accidental mission start)

    fn allows_arming(&self) -> bool { true }
    fn allows_arming_from_transmitter(&self) -> bool {
        false  // Safety: prevent RC stick pattern from starting mission
    }

    // ===== Control Type =====
    // Auto mode is autonomous, no manual input

    fn has_manual_input(&self) -> bool { false }
    fn is_autopilot_mode(&self) -> bool { true }
}
```

**Validation Usage:**

```rust
/// Validate mode entry prerequisites
fn validate_mode_entry(mode: &dyn ModeCapability, state: &SystemState)
                       -> Result<(), &'static str> {
    // Check sensor requirements
    if mode.requires_position() && !state.has_position_estimate() {
        return Err("Mode requires position estimate");
    }

    if mode.requires_velocity() && !state.has_velocity_estimate() {
        return Err("Mode requires velocity estimate");
    }

    if mode.requires_gps() && !state.has_gps_fix() {
        return Err("Mode requires GPS fix");
    }

    if mode.requires_imu() && !state.has_imu_data() {
        return Err("Mode requires IMU data");
    }

    if mode.requires_compass() && !state.has_compass_data() {
        return Err("Mode requires compass/heading");
    }

    Ok(())
}

/// Validate arming request
fn validate_arming(mode: &dyn ModeCapability, method: ArmMethod)
                   -> Result<(), &'static str> {
    // Check if mode allows arming
    if !mode.allows_arming() {
        return Err("Cannot arm in this mode");
    }

    // Check if RC arming allowed
    if method == ArmMethod::RcRudder || method == ArmMethod::RcSwitch {
        if !mode.allows_arming_from_transmitter() {
            return Err("RC arming not allowed in this mode");
        }
    }

    Ok(())
}

/// Select fallback mode based on capabilities
fn select_fallback_mode(modes: &[Box<dyn Mode>], state: &SystemState)
                        -> Option<&Box<dyn Mode>> {
    // Prefer manual input modes for RC loss
    for mode in modes {
        if mode.has_manual_input() && validate_mode_entry(mode.as_ref(), state).is_ok() {
            return Some(mode);
        }
    }

    // Fallback to any valid mode
    for mode in modes {
        if validate_mode_entry(mode.as_ref(), state).is_ok() {
            return Some(mode);
        }
    }

    None
}
```

**Log Entry Examples:**

```
# Mode entry denied (sensor requirement)
MODE_ENTRY_FAILED,123456,Auto,Position estimate unavailable

# Arming denied (RC arming in Auto)
ARM_DENIED,123500,RcRudder,RC arming not allowed in Auto mode

# Fallback mode selection
FAILSAFE,123600,RcLoss,Auto->Manual (manual input mode selected)
```

## Platform Considerations

### Pico W (RP2040) / Pico 2 W (RP2350)

- Zero allocation: Capability queries return primitives (bool)
- Fast execution: Simple boolean returns, no computation
- Embedded-friendly: No dynamic dispatch in hot path (trait methods can be inlined)

### Cross-Platform

- Capability trait is platform-agnostic
- Query results independent of platform
- Performance requirements apply to all platforms

## Risks & Mitigation

| Risk                                                            | Impact | Likelihood | Mitigation                                                                | Validation                                                           |
| --------------------------------------------------------------- | ------ | ---------- | ------------------------------------------------------------------------- | -------------------------------------------------------------------- |
| Mode capability declaration incorrect (false positive/negative) | High   | Medium     | Code review, test each capability, cross-check with ArduPilot             | Test: verify each mode capability matches expected behavior          |
| Validation too strict (prevents legitimate use)                 | Medium | Medium     | Allow override for test/debug (ARMING_CHECK parameter), review thresholds | Operational review: monitor mode entry denial rate                   |
| Validation too loose (allows unsafe operation)                  | High   | Low        | Conservative defaults, safety review, test edge cases                     | Safety audit: verify all unsafe scenarios caught                     |
| Capability taxonomy incomplete (missing sensor types)           | Medium | Low        | Review ArduPilot taxonomy, extend as needed, maintain compatibility       | Compare with ArduPilot capabilities, add missing types               |
| Performance regression (capability queries too slow)            | Low    | Very Low   | Profile hot path, inline capability queries, avoid dynamic dispatch       | Performance profiling: verify < 1 μs per query                       |
| New mode added without capability implementation                | Medium | Low        | Compile-time enforcement via trait, code review checklist                 | Compilation check: all modes implement ModeCapability trait          |
| Capability inconsistencies (logical contradictions)             | Medium | Medium     | Document capability relationships, validation checks for consistency      | Unit tests: verify logical relationships (e.g., autopilot → !manual) |
| Arming denied in unexpected scenarios                           | Medium | Medium     | Comprehensive test coverage, operational testing, clear error messages    | Test: verify arming allowed/denied in all mode/method combinations   |

## Implementation Notes

**Preferred approaches:**

- **Trait-based design**: Compile-time enforcement of capability declaration
- **Zero allocation**: Return primitives (bool), no heap allocation
- **Conservative defaults**: When uncertain, declare requirement (safe to over-specify)
- **Self-documenting**: Capability names and documentation clearly explain purpose
- **Validation reuse**: Single validation framework uses capability queries consistently

**Known pitfalls:**

- **False capabilities**: Don't declare capabilities mode doesn't actually use
- **Inconsistent logic**: Ensure capability relationships make sense (autopilot → !manual)
- **Over-restriction**: Don't prevent legitimate use cases (allow test/debug overrides)
- **Under-restriction**: Don't allow unsafe scenarios (conservative safety requirements)
- **Performance assumptions**: Don't assume capability queries cached (keep fast)

**Related code areas:**

- `src/vehicle/mode/capability.rs` - ModeCapability trait definition
- `src/vehicle/mode/manual.rs` - Manual mode capability implementation
- `src/vehicle/mode/auto.rs` - Auto mode capability implementation
- `src/vehicle/validation/` - Validation framework using capability queries
- `src/vehicle/arming/checks.rs` - Pre-arm checks using capability queries
- `src/vehicle/failsafe/actions.rs` - Failsafe mode selection using capabilities

**Suggested patterns:**

- Trait composition for capability groups (sensor requirements, arming permissions)
- Builder pattern for complex capability validation
- Capability matrix documentation (table format)

## External References

- Analysis: [AN-g5w99-mode-capability-system](../analysis/AN-g5w99-mode-capability-system.md)
- ArduPilot Mode Capability Queries: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.h>
- ArduPilot Pre-Arm Checks: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Arming/AP_Arming.cpp>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
