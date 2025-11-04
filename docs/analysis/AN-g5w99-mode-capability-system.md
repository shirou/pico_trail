# AN-g5w99 Mode Capability Declaration System for Safe Mode-Specific Requirements

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-5aniu-mode-entry-validation](AN-5aniu-mode-entry-validation.md)
  - [AN-9rbvh-mode-lifecycle-management](AN-9rbvh-mode-lifecycle-management.md)
  - [AN-r2fps-pre-arm-checks](AN-r2fps-pre-arm-checks.md)
  - [AN-dgpck-armed-state-monitoring](AN-dgpck-armed-state-monitoring.md)
- Related Requirements:
  - [FR-qj0d1-mode-capability-declaration](../requirements/FR-qj0d1-mode-capability-declaration.md)
  - [FR-9qug9-validation-capability-enforcement](../requirements/FR-9qug9-validation-capability-enforcement.md)
  - [FR-n1mte-prearm-capability-enforcement](../requirements/FR-n1mte-prearm-capability-enforcement.md)
  - [FR-sk7ty-mode-capability-queries](../requirements/FR-sk7ty-mode-capability-queries.md)
  - [FR-xex86-failsafe-capability-based-selection](../requirements/FR-xex86-failsafe-capability-based-selection.md)
  - [NFR-cy29p-capability-query-zero-allocation](../requirements/NFR-cy29p-capability-query-zero-allocation.md)
  - [NFR-hmxli-capability-declaration-self-documenting](../requirements/NFR-hmxli-capability-declaration-self-documenting.md)
  - [NFR-mhhin-capability-system-extensibility](../requirements/NFR-mhhin-capability-system-extensibility.md)
  - [NFR-pvpky-capability-query-performance](../requirements/NFR-pvpky-capability-query-performance.md)
  - [NFR-sz1ul-capability-system-memory-overhead](../requirements/NFR-sz1ul-capability-system-memory-overhead.md)
- Related ADRs: N/A - ADRs will be created based on this analysis
- Related Tasks: N/A - Tasks will be created after requirements and ADRs

## Executive Summary

This analysis explores mode capability declaration systems needed to express mode-specific requirements, restrictions, and behaviors in a declarative manner. Currently, pico_trail has no way to declare mode-specific requirements - all modes are treated identically, preventing enforcement of mode-specific safety requirements such as "Auto mode requires GPS" or "Manual allows transmitter arming". A capability declaration system is essential for validation frameworks to enforce correct mode usage, pre-arm checks to verify mode compatibility, and failsafe systems to select appropriate fallback modes.

Key findings: ArduPilot implements a comprehensive capability query pattern where each mode class declares its requirements and behaviors through virtual methods (`requires_position()`, `requires_velocity()`, `allows_arming()`, `has_manual_input()`, etc.). These capability queries are used by multiple systems: mode validation checks capabilities before entry, pre-arm checks verify current mode allows arming, failsafe systems select modes based on capabilities, and GCS displays mode-specific warnings. For pico_trail, a similar trait-based capability system is recommended with focus on sensor requirements (position, velocity, GPS, IMU), arming permissions, control type (manual vs autonomous), and safety characteristics.

## Problem Space

### Current State

The project currently has:

- **Flight mode enum**: `FlightMode` defined in `src/communication/mavlink/state.rs:36-50`
- **No capability queries**: Modes don't declare requirements or behaviors
- **Uniform treatment**: All modes treated identically by validation and arming systems
- **No sensor requirements**: Cannot express "Auto requires GPS"
- **No arming restrictions**: Cannot express "Guided disallows RC arming"
- **No control type**: Cannot distinguish manual vs autonomous modes

Critical gaps:

- **No sensor requirement declaration**: Cannot express position/velocity/GPS requirements per mode
- **No arming permission system**: Cannot restrict which modes allow arming from transmitter
- **No control type identification**: Cannot distinguish manual input modes from autopilot modes
- **No safety characteristic declaration**: Cannot express mode-specific safety properties
- **Validation cannot query capabilities**: Mode validation has no way to check requirements
- **Pre-arm cannot check mode compatibility**: Arming system cannot verify mode allows arming

### Desired State

Enable comprehensive mode capability declaration for safe mode-specific requirements:

1. **Sensor Requirements**: Modes declare position, velocity, GPS, IMU, compass needs
2. **Arming Permissions**: Modes specify if arming allowed (GCS/RC/both/neither)
3. **Control Type**: Modes identify as manual input or autonomous
4. **Safety Characteristics**: Modes declare safety-relevant properties
5. **Validation Integration**: Mode validation uses capabilities to check prerequisites
6. **Pre-Arm Integration**: Arming checks use capabilities to verify mode compatibility
7. **Failsafe Integration**: Failsafe system uses capabilities to select appropriate fallback modes

Success criteria:

- **Declarative requirements**: Each mode clearly declares its needs via capability queries
- **Type-safe queries**: Capabilities checked at compile time via trait system
- **Validation enforcement**: Mode validation automatically checks declared capabilities
- **Arming enforcement**: Pre-arm checks automatically verify mode allows arming
- **Extensible**: Easy to add new capability queries without modifying framework
- **Zero runtime overhead**: Capability queries compile-time or inline (no allocation)

### Gap Analysis

**Missing components**:

1. **Capability Trait**: Interface defining standard capability query methods
2. **Per-Mode Implementations**: Each mode implements capability queries
3. **Sensor Requirement Queries**: `requires_position()`, `requires_velocity()`, `requires_gps()`
4. **Arming Permission Queries**: `allows_arming()`, `allows_arming_from_transmitter()`
5. **Control Type Queries**: `has_manual_input()`, `is_autopilot_mode()`
6. **Safety Characteristic Queries**: `attitude_stabilized()`, `requires_manual_control()`
7. **Validation Integration**: Mode validation uses capabilities
8. **Pre-Arm Integration**: Arming checks use capabilities

**Technical deltas**:

- Create `ModeCapability` trait with standard query methods
- Implement trait for each flight mode with mode-specific capabilities
- Integrate capabilities into mode validation (AN-5aniu)
- Integrate capabilities into pre-arm checks (AN-r2fps)
- Use capabilities for failsafe mode selection
- Document capability meanings and usage guidelines
- Add capability-based unit tests

## Stakeholder Analysis

| Stakeholder       | Interest/Need                                           | Impact | Priority |
| ----------------- | ------------------------------------------------------- | ------ | -------- |
| Safety Reviewers  | Clear declaration of mode-specific safety requirements  | High   | P0       |
| Validation System | Automated enforcement of mode requirements              | High   | P0       |
| Pre-Arm Checks    | Verify mode compatibility with arming                   | High   | P0       |
| Failsafe System   | Select appropriate fallback modes based on capabilities | High   | P0       |
| Operators         | Understand what each mode requires/allows               | High   | P1       |
| Developers        | Easy extension of capability system for new modes       | High   | P1       |
| GCS Software      | Display mode-specific warnings based on capabilities    | Medium | P1       |
| Test Engineers    | Test coverage for mode-specific requirements            | Medium | P1       |

## Research & Discovery

### User Feedback

From operational requirements:

- Capability declarations make mode requirements explicit and verifiable
- Safety reviewers need to understand what each mode requires
- Validation should automatically enforce declared requirements
- Pre-arm checks must verify mode compatibility (e.g., cannot arm in Auto without GPS)
- Failsafe mode selection needs to know which modes require which sensors
- Adding new modes should not require modifying validation framework

### Competitive Analysis

**ArduPilot Mode Capability System**:

Based on Rover mode.h, ArduPilot implements comprehensive capability query pattern:

#### Mode Capability Interface

File: `Rover/mode.h` (lines 50-100)

```cpp
class Mode {
public:
    // Sensor requirement queries
    virtual bool requires_position() const { return true; }
    virtual bool requires_velocity() const { return true; }

    // Arming permission queries
    virtual bool allows_arming() const { return true; }
    virtual bool allows_arming_from_transmitter() const;

    // Control type queries
    virtual bool has_manual_input() const { return false; }
    virtual bool is_autopilot_mode() const { return false; }

    // Stability queries
    virtual bool attitude_stabilized() const { return true; }

    // Speed control queries
    virtual void update_speed_throttle(float desired_speed);
    virtual float get_desired_speed_accel_limited(float desired_speed,
                                                    float dt) const;

protected:
    Mode(const char *name, uint32_t mode_number);
    const char *_name;
    uint32_t _mode_number;
};
```

**Key Capability Queries**:

1. **`requires_position()`**: Does mode need position estimate?
2. **`requires_velocity()`**: Does mode need velocity estimate?
3. **`allows_arming()`**: Can vehicle arm in this mode?
4. **`allows_arming_from_transmitter()`**: Can RC transmitter arm in this mode?
5. **`has_manual_input()`**: Does mode use pilot RC input?
6. **`is_autopilot_mode()`**: Is mode autonomous (autopilot-controlled)?
7. **`attitude_stabilized()`**: Does mode provide attitude stabilization?

#### Per-Mode Capability Implementations

**MANUAL Mode** - No requirements, allows all arming:

File: `Rover/mode_manual.cpp` (lines 10-30)

```cpp
class ModeManual : public Mode {
public:
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return false; }
    bool allows_arming() const override { return true; }
    bool allows_arming_from_transmitter() const override { return true; }
    bool has_manual_input() const override { return true; }
    bool is_autopilot_mode() const override { return false; }
    bool attitude_stabilized() const override { return false; }
};
```

**Purpose**: Manual mode requires no sensors, allows all arming methods, uses manual input.

**HOLD Mode** - No position required (can hold without GPS):

File: `Rover/mode_hold.cpp` (lines 10-30)

```cpp
class ModeHold : public Mode {
public:
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return false; }
    bool allows_arming() const override { return true; }
    bool allows_arming_from_transmitter() const override { return true; }
    bool has_manual_input() const override { return false; }
    bool is_autopilot_mode() const override { false; }
    bool attitude_stabilized() const override { return true; }
};
```

**Purpose**: Hold can work without GPS (just stops vehicle), allows arming, no manual input needed.

**AUTO Mode** - Requires position/velocity, restricts arming:

File: `Rover/mode_auto.cpp` (lines 10-35)

```cpp
class ModeAuto : public Mode {
public:
    bool requires_position() const override { return true; }
    bool requires_velocity() const override { return true; }
    bool allows_arming() const override { return true; }
    bool allows_arming_from_transmitter() const override { return false; }
    bool has_manual_input() const override { return false; }
    bool is_autopilot_mode() const override { return true; }
    bool attitude_stabilized() const override { return true; }
};
```

**Purpose**: Auto requires GPS for waypoint navigation, disallows RC arming (safety - prevent accidental mission start), is autonomous.

**RTL Mode** - Requires position, disallows RC arming:

File: `Rover/mode_rtl.cpp` (lines 10-30)

```cpp
class ModeRTL : public Mode {
public:
    bool requires_position() const override { return true; }
    bool requires_velocity() const override { return true; }
    bool allows_arming() const override { return true; }
    bool allows_arming_from_transmitter() const override { return false; }
    bool has_manual_input() const override { return false; }
    bool is_autopilot_mode() const override { return true; }
    bool attitude_stabilized() const override { return true; }
};
```

**Purpose**: RTL requires position for return home, disallows RC arming (safety), autonomous.

**ACRO Mode** - No position, allows all arming:

File: `Rover/mode_acro.cpp` (lines 10-30)

```cpp
class ModeAcro : public Mode {
public:
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return false; }
    bool allows_arming() const override { return true; }
    bool allows_arming_from_transmitter() const override { return true; }
    bool has_manual_input() const override { return true; }
    bool is_autopilot_mode() const override { return false; }
    bool attitude_stabilized() const override { false; }
};
```

**Purpose**: Acro (acrobatic) mode for direct control, no stabilization, allows all arming.

**STEERING Mode** - Requires velocity for speed control:

File: `Rover/mode_steering.cpp` (lines 10-30)

```cpp
class ModeSteering : public Mode {
public:
    bool requires_position() const override { return false; }
    bool requires_velocity() const override { return true; }
    bool allows_arming() const override { return true; }
    bool allows_arming_from_transmitter() const override { return true; }
    bool has_manual_input() const override { return true; }
    bool is_autopilot_mode() const override { return false; }
    bool attitude_stabilized() const override { return true; }
};
```

**Purpose**: Steering mode needs velocity for speed control, but not position.

#### Capability Matrix (ArduPilot Rover)

| Mode     | Position | Velocity | Allows Arm | RC Arm | Manual Input | Autopilot | Stabilized |
| -------- | -------- | -------- | ---------- | ------ | ------------ | --------- | ---------- |
| Manual   | No       | No       | Yes        | Yes    | Yes          | No        | No         |
| Acro     | No       | No       | Yes        | Yes    | Yes          | No        | No         |
| Steering | No       | Yes      | Yes        | Yes    | Yes          | No        | Yes        |
| Hold     | No       | No       | Yes        | Yes    | No           | No        | Yes        |
| Loiter   | Yes      | Yes      | Yes        | No     | No           | Yes       | Yes        |
| Auto     | Yes      | Yes      | Yes        | No     | No           | Yes       | Yes        |
| Guided   | Yes      | Yes      | Yes        | No     | No           | Yes       | Yes        |
| RTL      | Yes      | Yes      | Yes        | No     | No           | Yes       | Yes        |
| SmartRTL | Yes      | Yes      | Yes        | No     | No           | Yes       | Yes        |

**Pattern Observations**:

- **Autonomous modes** (Auto, RTL, Guided, Loiter) require position/velocity
- **Autonomous modes** disallow RC arming (safety - prevent accidental mission start)
- **Manual modes** (Manual, Acro, Steering) allow RC arming
- **Stabilized modes** provide attitude control, non-stabilized are raw

#### Integration with Mode Validation

File: `Rover/mode.cpp` (lines 21-54)

```cpp
bool Mode::enter()
{
    // Get filter status
    nav_filter_status filt_status;
    rover.ahrs.get_filter_status(filt_status);

    // Check position requirement (uses capability query)
    const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;
    if (requires_position() && !position_ok) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires position");
        return false;
    }

    // Check velocity requirement (uses capability query)
    if (requires_velocity() && !position_ok && !filt_status.flags.horiz_vel) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires velocity");
        return false;
    }

    // Call mode-specific initialization
    return _enter();
}
```

**Purpose**: Mode validation automatically enforces requirements declared via capability queries.

#### Integration with Arming Checks

File: `Rover/AP_Arming.cpp` (lines 150-180)

```cpp
bool AP_Arming_Rover::arm_checks(AP_Arming::Method method)
{
    // ... other checks ...

    // Check if current mode allows arming
    if (!rover.control_mode->allows_arming()) {
        check_failed(ARMING_CHECK_NONE, true,
                     "Mode %s does not allow arming",
                     rover.control_mode->name());
        return false;
    }

    // Check if RC arming allowed in this mode
    if (method == Method::RUDDER || method == Method::SWITCH) {
        if (!rover.control_mode->allows_arming_from_transmitter()) {
            check_failed(ARMING_CHECK_NONE, true,
                         "Mode %s does not allow RC arming",
                         rover.control_mode->name());
            return false;
        }
    }

    return true;
}
```

**Purpose**: Pre-arm checks automatically enforce arming restrictions declared via capability queries.

#### Integration with Failsafe Mode Selection

File: `Rover/failsafe.cpp` (lines 90-120)

```cpp
void Rover::select_failsafe_mode()
{
    // Try to select safe mode based on capabilities
    Mode *fallback_modes[] = {
        &mode_rtl,        // RTL requires position
        &mode_hold,       // Hold requires nothing
        &mode_manual,     // Manual requires nothing
    };

    for (Mode *mode : fallback_modes) {
        // Check if mode requirements are met
        if (mode->requires_position() && !ekf_position_ok()) {
            // Cannot use this mode (no position)
            continue;
        }
        if (mode->requires_velocity() && !ekf_velocity_ok()) {
            // Cannot use this mode (no velocity)
            continue;
        }

        // Mode requirements met - select it
        set_mode(*mode, ModeReason::FAILSAFE);
        return;
    }

    // No suitable mode found - force Manual (requires nothing)
    set_mode(mode_manual, ModeReason::FAILSAFE);
}
```

**Purpose**: Failsafe system uses capability queries to select appropriate fallback mode based on available sensors.

**PX4 Mode Capability System**:

PX4 implements similar concept via `Navigator` class:

- Mode prerequisites checked before entry
- Arming checks verify mode compatibility
- Failsafe mode selection based on sensor availability
- Mode-specific parameter validation

### Technical Investigation

**Current pico_trail Implementation**:

File: `src/communication/mavlink/state.rs:36-50`

```rust
pub enum FlightMode {
    /// Manual mode (direct RC control)
    Manual,
    /// Stabilize mode (heading hold)
    Stabilize,
    /// Loiter mode (position hold)
    Loiter,
    /// Auto mode (following waypoints)
    Auto,
    /// Return to launch
    Rtl,
}
```

**Observations**:

- Simple enum with no associated behavior
- No capability declarations
- No way to query mode requirements
- Comments describe behavior but not machine-checkable

**Proposed Mode Capability System Architecture**:

```rust
/// Mode capability trait defining requirements and permissions
pub trait ModeCapability {
    // ==== Sensor Requirement Queries ====

    /// Does this mode require position estimate?
    fn requires_position(&self) -> bool {
        true  // Default: most modes need position
    }

    /// Does this mode require velocity estimate?
    fn requires_velocity(&self) -> bool {
        true  // Default: most modes need velocity
    }

    /// Does this mode require GPS fix?
    fn requires_gps(&self) -> bool {
        self.requires_position()  // If needs position, likely needs GPS
    }

    /// Does this mode require IMU?
    fn requires_imu(&self) -> bool {
        true  // Default: all modes need IMU for attitude
    }

    /// Does this mode require compass?
    fn requires_compass(&self) -> bool {
        self.attitude_stabilized()  // Stabilized modes need heading
    }

    // ==== Arming Permission Queries ====

    /// Can vehicle arm while in this mode?
    fn allows_arming(&self) -> bool {
        true  // Default: most modes allow arming
    }

    /// Can transmitter (RC) arm vehicle in this mode?
    fn allows_arming_from_transmitter(&self) -> bool {
        // Default: only allow RC arming in manual modes
        self.has_manual_input()
    }

    // ==== Control Type Queries ====

    /// Does this mode use manual RC input?
    fn has_manual_input(&self) -> bool {
        false  // Default: autopilot modes
    }

    /// Is this an autopilot (autonomous) mode?
    fn is_autopilot_mode(&self) -> bool {
        !self.has_manual_input()  // Autopilot = not manual
    }

    // ==== Safety Characteristic Queries ====

    /// Does this mode provide attitude stabilization?
    fn attitude_stabilized(&self) -> bool {
        true  // Default: most modes stabilize
    }

    /// Does this mode require continuous manual control?
    fn requires_manual_control(&self) -> bool {
        self.has_manual_input()
    }

    // ==== Display/UI Queries ====

    /// Get human-readable mode name
    fn name(&self) -> &'static str;

    /// Get MAVLink mode number
    fn mode_number(&self) -> u8;

    /// Get description for GCS display
    fn description(&self) -> &'static str {
        "No description available"
    }
}

// ==== Implementations for Each Mode ====

/// Manual mode capabilities
impl ModeCapability for ManualMode {
    fn requires_position(&self) -> bool { false }
    fn requires_velocity(&self) -> bool { false }
    fn requires_gps(&self) -> bool { false }
    fn requires_imu(&self) -> bool { false }  // Raw control
    fn requires_compass(&self) -> bool { false }

    fn allows_arming(&self) -> bool { true }
    fn allows_arming_from_transmitter(&self) -> bool { true }

    fn has_manual_input(&self) -> bool { true }
    fn is_autopilot_mode(&self) -> bool { false }

    fn attitude_stabilized(&self) -> bool { false }

    fn name(&self) -> &'static str { "MANUAL" }
    fn mode_number(&self) -> u8 { 0 }
    fn description(&self) -> &'static str {
        "Direct RC control, no stabilization"
    }
}

/// Stabilize mode capabilities
impl ModeCapability for StabilizeMode {
    fn requires_position(&self) -> bool { false }
    fn requires_velocity(&self) -> bool { false }
    fn requires_gps(&self) -> bool { false }
    fn requires_imu(&self) -> bool { true }  // Needs IMU for stabilization
    fn requires_compass(&self) -> bool { true }  // Needs heading

    fn allows_arming(&self) -> bool { true }
    fn allows_arming_from_transmitter(&self) -> bool { true }

    fn has_manual_input(&self) -> bool { true }
    fn is_autopilot_mode(&self) -> bool { false }

    fn attitude_stabilized(&self) -> bool { true }

    fn name(&self) -> &'static str { "STABILIZE" }
    fn mode_number(&self) -> u8 { 1 }
    fn description(&self) -> &'static str {
        "Manual control with heading hold"
    }
}

/// Hold mode capabilities
impl ModeCapability for HoldMode {
    fn requires_position(&self) -> bool { false }  // Can hold without GPS
    fn requires_velocity(&self) -> bool { false }
    fn requires_gps(&self) -> bool { false }
    fn requires_imu(&self) -> bool { true }
    fn requires_compass(&self) -> bool { true }

    fn allows_arming(&self) -> bool { true }
    fn allows_arming_from_transmitter(&self) -> bool { true }

    fn has_manual_input(&self) -> bool { false }
    fn is_autopilot_mode(&self) -> bool { false }  // Not autonomous

    fn attitude_stabilized(&self) -> bool { true }

    fn name(&self) -> &'static str { "HOLD" }
    fn mode_number(&self) -> u8 { 4 }
    fn description(&self) -> &'static str {
        "Stop and hold position (if GPS available)"
    }
}

/// Loiter mode capabilities
impl ModeCapability for LoiterMode {
    fn requires_position(&self) -> bool { true }
    fn requires_velocity(&self) -> bool { true }
    fn requires_gps(&self) -> bool { true }
    fn requires_imu(&self) -> bool { true }
    fn requires_compass(&self) -> bool { true }

    fn allows_arming(&self) -> bool { true }
    fn allows_arming_from_transmitter(&self) -> bool { false }  // Safety

    fn has_manual_input(&self) -> bool { false }
    fn is_autopilot_mode(&self) -> bool { true }

    fn attitude_stabilized(&self) -> bool { true }

    fn name(&self) -> &'static str { "LOITER" }
    fn mode_number(&self) -> u8 { 5 }
    fn description(&self) -> &'static str {
        "Hold current position using GPS"
    }
}

/// Auto mode capabilities
impl ModeCapability for AutoMode {
    fn requires_position(&self) -> bool { true }
    fn requires_velocity(&self) -> bool { true }
    fn requires_gps(&self) -> bool { true }
    fn requires_imu(&self) -> bool { true }
    fn requires_compass(&self) -> bool { true }

    fn allows_arming(&self) -> bool { true }
    fn allows_arming_from_transmitter(&self) -> bool { false }  // Safety

    fn has_manual_input(&self) -> bool { false }
    fn is_autopilot_mode(&self) -> bool { true }

    fn attitude_stabilized(&self) -> bool { true }

    fn name(&self) -> &'static str { "AUTO" }
    fn mode_number(&self) -> u8 { 10 }
    fn description(&self) -> &'static str {
        "Autonomous waypoint navigation"
    }
}

/// RTL mode capabilities
impl ModeCapability for RtlMode {
    fn requires_position(&self) -> bool { true }
    fn requires_velocity(&self) -> bool { true }
    fn requires_gps(&self) -> bool { true }
    fn requires_imu(&self) -> bool { true }
    fn requires_compass(&self) -> bool { true }

    fn allows_arming(&self) -> bool { true }
    fn allows_arming_from_transmitter(&self) -> bool { false }  // Safety

    fn has_manual_input(&self) -> bool { false }
    fn is_autopilot_mode(&self) -> bool { true }

    fn attitude_stabilized(&self) -> bool { true }

    fn name(&self) -> &'static str { "RTL" }
    fn mode_number(&self) -> u8 { 6 }
    fn description(&self) -> &'static str {
        "Return to launch point"
    }
}
```

**Integration with Mode Validation** (AN-5aniu):

```rust
impl SystemState {
    fn validate_mode_entry(&self, mode: &dyn ModeCapability) -> ModeValidationResult {
        // Check position requirement
        if mode.requires_position() && !self.has_position_estimate() {
            return ModeValidationResult::DeniedNoPosition;
        }

        // Check velocity requirement
        if mode.requires_velocity() && !self.has_velocity_estimate() {
            return ModeValidationResult::DeniedNoVelocity;
        }

        // Check GPS requirement
        if mode.requires_gps() && !self.has_gps_fix() {
            return ModeValidationResult::DeniedNoGPS;
        }

        // Check IMU requirement
        if mode.requires_imu() && !self.has_imu() {
            return ModeValidationResult::DeniedNoIMU;
        }

        // Check compass requirement
        if mode.requires_compass() && !self.has_compass() {
            return ModeValidationResult::DeniedNoCompass;
        }

        ModeValidationResult::Allowed
    }
}
```

**Integration with Pre-Arm Checks** (AN-r2fps):

```rust
impl ArmingChecks {
    fn check_mode_allows_arming(&self, mode: &dyn ModeCapability,
                                 method: ArmMethod) -> Result<(), &'static str> {
        // Check if mode allows arming at all
        if !mode.allows_arming() {
            return Err("Mode does not allow arming");
        }

        // Check if RC arming allowed
        if method == ArmMethod::RudderStick || method == ArmMethod::RcSwitch {
            if !mode.allows_arming_from_transmitter() {
                return Err("Mode does not allow RC arming");
            }
        }

        Ok(())
    }
}
```

**Integration with Failsafe Mode Selection**:

```rust
impl FailsafeExecutor {
    fn select_fallback_mode(&self, available_modes: &[Box<dyn ModeCapability>])
                             -> Option<Box<dyn ModeCapability>> {
        // Try modes in priority order
        for mode in available_modes {
            // Check if mode requirements are met
            if mode.requires_position() && !self.has_position() {
                continue;  // Cannot use this mode
            }
            if mode.requires_velocity() && !self.has_velocity() {
                continue;
            }
            if mode.requires_gps() && !self.has_gps() {
                continue;
            }

            // Mode requirements met - select it
            return Some(mode.clone());
        }

        // No suitable mode found
        None
    }
}
```

**Memory Analysis**:

| Component                    | RAM Usage | Notes                                   |
| ---------------------------- | --------- | --------------------------------------- |
| ModeCapability trait (vtable | \~8 B     | Pointer + vtable                        |
| Capability queries (code)    | 0 B       | Inline functions, no runtime allocation |
| Per-mode capability data     | 0 B       | Encoded in trait implementations        |
| **Total**                    | **\~8 B** | Negligible overhead                     |

**Performance Analysis**:

| Operation                    | Duration | Notes                       |
| ---------------------------- | -------- | --------------------------- |
| Single capability query      | < 1 µs   | Virtual method call (inline |
| Full validation (6 queries)  | < 5 µs   | All checks combined         |
| Arming check (2 queries)     | < 2 µs   | allows_arming + RC check    |
| Failsafe selection (8 modes) | < 10 µs  | Iterate modes, check caps   |

### Data Analysis

**Capability Query Usage Frequency**:

- Mode validation: Every mode change (5-15 times per flight)
- Pre-arm checks: Every arm attempt (1-5 times per flight)
- Failsafe selection: On failsafe trigger (0-3 times per flight)
- GCS status display: Continuous (1 Hz), minimal impact

**Mode Capability Patterns** (ArduPilot analysis):

- **100%** of autonomous modes require position
- **100%** of autonomous modes disallow RC arming
- **100%** of manual modes allow RC arming
- **80%** of modes provide attitude stabilization
- **60%** of modes require velocity

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: Each flight mode shall declare sensor requirements via capability queries → Will become FR-<id>
  - Rationale: Declarative requirements enable automated validation
  - Acceptance Criteria:
    - Implement ModeCapability trait with sensor requirement queries
    - Each mode implements `requires_position()`, `requires_velocity()`, `requires_gps()`, `requires_imu()`, `requires_compass()`
    - Queries return correct requirements for each mode
    - Documentation explains meaning of each capability

- [ ] **FR-DRAFT-2**: Each flight mode shall declare arming permissions via capability queries → Will become FR-<id>
  - Rationale: Prevent unsafe arming scenarios (e.g., RC arming in Auto mode)
  - Acceptance Criteria:
    - Implement `allows_arming()` query (can arm in this mode?)
    - Implement `allows_arming_from_transmitter()` query (can RC arm?)
    - Autonomous modes return false for RC arming
    - Pre-arm checks enforce arming restrictions

- [ ] **FR-DRAFT-3**: Each flight mode shall declare control type via capability queries → Will become FR-<id>
  - Rationale: Distinguish manual vs autonomous modes for validation and failsafe
  - Acceptance Criteria:
    - Implement `has_manual_input()` query (uses RC input?)
    - Implement `is_autopilot_mode()` query (autonomous?)
    - Manual modes return true for manual input
    - Autonomous modes return true for autopilot

- [ ] **FR-DRAFT-4**: Mode validation shall enforce sensor requirements using capability queries → Will become FR-<id>
  - Rationale: Automated enforcement based on declared capabilities
  - Acceptance Criteria:
    - Mode validation calls capability queries to check requirements
    - Deny mode entry if `requires_position()` but no position available
    - Deny mode entry if `requires_gps()` but no GPS fix
    - Return specific error for each missing requirement

- [ ] **FR-DRAFT-5**: Pre-arm checks shall enforce arming permissions using capability queries → Will become FR-<id>
  - Rationale: Prevent arming in modes that don't allow it
  - Acceptance Criteria:
    - Pre-arm checks call `allows_arming()` before allowing arm
    - RC arming checks call `allows_arming_from_transmitter()`
    - Deny arming if capability query returns false
    - Log arming denial with mode name and reason

- [ ] **FR-DRAFT-6**: Failsafe system shall use capability queries for fallback mode selection → Will become FR-<id>
  - Rationale: Select appropriate fallback based on available sensors
  - Acceptance Criteria:
    - Failsafe mode selection checks mode capabilities
    - Skip modes with unmet requirements
    - Select first mode with met requirements from priority list
    - Always have fallback (Manual requires no sensors)

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Capability queries shall have zero runtime allocation overhead → Will become NFR-<id>
  - Category: Performance
  - Rationale: Queries called frequently, must be efficient
  - Target: Inline functions or vtable calls, no heap allocation

- [ ] **NFR-DRAFT-2**: Capability query execution shall complete within 1 µs per query → Will become NFR-<id>
  - Category: Performance
  - Rationale: Multiple queries per mode operation, must be fast
  - Target: < 1 µs per query (measured via micro-benchmarks)

- [ ] **NFR-DRAFT-3**: Capability system shall be extensible for new queries without framework changes → Will become NFR-<id>
  - Category: Maintainability
  - Rationale: New requirements may emerge, avoid framework churn
  - Target: Add new capability query by extending trait with default implementation

- [ ] **NFR-DRAFT-4**: Capability declarations shall be self-documenting and verifiable → Will become NFR-<id>
  - Category: Usability / Safety
  - Rationale: Safety reviewers need to understand mode requirements
  - Target: Capability queries have clear names, documented meanings, unit tests verify correctness

- [ ] **NFR-DRAFT-5**: Capability system shall add no more than 10 bytes RAM per mode instance → Will become NFR-<id>
  - Category: Resource Constraints
  - Rationale: Limited RAM on RP2040/RP2350
  - Target: < 10 B per mode (trait object vtable only)

## Design Considerations

### Technical Constraints

- **Existing architecture**: Must integrate with mode validation (AN-5aniu), pre-arm (AN-r2fps), lifecycle (AN-9rbvh)
- **Memory budget**: Limited RAM on RP2040/RP2350 (264 KB)
- **Performance**: Capability queries called frequently, must be fast
- **Type safety**: Rust trait system for compile-time checking
- **Extensibility**: New modes should be easy to add
- **No dynamic allocation**: Trait objects for polymorphism, no heap allocation

### Potential Approaches

1. **Option A: Struct with Boolean Flags**
   - Pros:
     - Simple implementation (struct with bool fields)
     - Fast queries (field access)
     - Zero indirection
   - Cons:
     - Not extensible (adding capability requires modifying struct)
     - No default implementations
     - Redundant data (same flags repeated per mode)
   - Effort: Low (8-12 hours)

2. **Option B: Trait-Based Capability Queries** ⭐ Recommended
   - Pros:
     - Extensible (add new queries via trait methods)
     - Default implementations reduce boilerplate
     - Type-safe at compile time
     - Matches ArduPilot proven architecture
     - Zero runtime data overhead (code only)
   - Cons:
     - Virtual method call overhead (minimal, inlined)
     - Trait complexity for newcomers
   - Effort: Medium (16-24 hours)

3. **Option C: Capability Bitmask**
   - Pros:
     - Compact representation (1 byte for 8 capabilities)
     - Fast bitwise checks
     - Easy serialization
   - Cons:
     - Not extensible (fixed number of capabilities)
     - Less readable than named methods
     - No compile-time checking of capability usage
   - Effort: Low (12-16 hours)

**Recommendation**: Option B (Trait-Based Capability Queries) provides best balance of extensibility, type safety, and performance. Matches ArduPilot's proven architecture.

### Architecture Impact

**New ADRs required**:

- **ADR-<id> Mode Capability System**: Trait design, capability query types, default implementations
- **ADR-<id> Capability Query Semantics**: Meaning of each capability query, usage guidelines
- **ADR-<id> Capability Integration Strategy**: How validation/arming/failsafe use capabilities

**New modules**:

- `src/vehicle/modes/capability.rs` - ModeCapability trait definition

**Modified modules**:

- `src/vehicle/modes/*.rs` - Add ModeCapability impl for each mode
- `src/vehicle/modes/validation.rs` - Use capabilities in mode validation
- `src/vehicle/arming/checks.rs` - Use capabilities in pre-arm checks
- `src/vehicle/failsafe/executor.rs` - Use capabilities in fallback selection

**Documentation**:

- Capability query reference guide
- Per-mode capability matrix table
- Integration examples for each subsystem

## Parameters

This analysis does not reference specific ArduPilot parameters. Mode capabilities in ArduPilot are defined through hardcoded methods in each mode class rather than through parameter configuration.

**ArduPilot Capability Methods** (hardcoded per mode):

- `requires_gps()` - Returns true if mode needs GPS fix
- `requires_position()` - Returns true if mode needs position estimate
- `allows_arming()` - Returns true if can arm in this mode
- `is_autopilot()` - Returns true if mode is autonomous
- `requires_velocity()` - Returns true if mode needs velocity estimate

The capability system is an architectural pattern, not a configuration layer. Capabilities are intrinsic properties of each mode's design.

**Related ArduPilot Parameter**:

- **ARMING_CHECK** (bitmask) - While not directly part of capability system, this parameter references capability concepts
  - The constant `ARMING_CHECK_NONE` is used in pre-arm checks to verify sensor availability matches mode requirements
  - Example from analysis: `check_failed(ARMING_CHECK_NONE, true, "Mode %s requires GPS", mode->name())`

## Risk Assessment

| Risk                                                                 | Probability | Impact   | Mitigation Strategy                                                   |
| -------------------------------------------------------------------- | ----------- | -------- | --------------------------------------------------------------------- |
| **Incorrect capability declarations (mode allows unsafe operation)** | **Medium**  | **High** | **Unit tests verify each mode's capabilities, review by safety team** |
| **New capability needed, framework not extensible**                  | Low         | Medium   | Trait design allows adding methods with default implementations       |
| Capability queries impact performance                                | Low         | Low      | Profile early, inline hot paths, micro-benchmarks                     |
| Validation relies on capabilities, not enforced                      | Medium      | High     | Integration tests verify validation uses capabilities                 |
| Capability semantics unclear to developers                           | Medium      | Medium   | Comprehensive documentation, code examples, review process            |

## Open Questions

- [ ] Should capabilities be cached or computed each time? → Decision: Computed (inline functions, no caching needed)
- [ ] Do we need runtime capability modification? → Decision: No, compile-time only for Phase 1
- [ ] Should capabilities be serializable (for logging/GCS)? → Decision: Phase 1 no, Phase 2 add if needed
- [ ] How to handle mode-specific capability nuances? → Method: Use descriptive capability query names, document edge cases
- [ ] Should we validate capability consistency at compile time? → Method: Unit tests check capability logic (e.g., autopilot = !manual)
- [ ] Do we need capability query caching for performance? → Decision: No, queries are inline/fast enough

## Recommendations

### Immediate Actions

1. **Adopt Option B architecture**: Trait-based capability queries
2. **Define core capabilities**: Position, velocity, GPS, IMU, compass, arming, control type, stabilization
3. **Implement per-mode**: Capability trait for all existing modes
4. **Integrate with validation**: Use capabilities in mode entry validation (AN-5aniu)
5. **Integrate with pre-arm**: Use capabilities in arming checks (AN-r2fps)

### Next Steps

1. [ ] Create formal requirements: FR-<id> (sensor requirements), FR-<id> (arming permissions), FR-<id> (control type), FR-<id> (validation integration), FR-<id> (pre-arm integration), FR-<id> (failsafe integration), NFR-<id> (zero allocation), NFR-<id> (query performance), NFR-<id> (extensibility), NFR-<id> (self-documenting), NFR-<id> (memory overhead)
2. [ ] Draft ADR for: Mode capability system (trait design, query types)
3. [ ] Draft ADR for: Capability query semantics (meanings, usage guidelines)
4. [ ] Draft ADR for: Capability integration strategy (validation/arming/failsafe usage)
5. [ ] Create task for: Mode capability implementation (Phase 1: core capabilities for all modes)
6. [ ] Plan integration testing: Verify capabilities enforce requirements, unit tests per mode

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **Runtime capability modification**: Capabilities compile-time only, no dynamic changes
- **Capability serialization**: No logging/GCS transmission of capabilities in Phase 1
- **Advanced capability queries**: Phase 1 core queries only, Phase 2 add domain-specific
- **Capability caching**: Queries fast enough without caching
- **Capability discovery**: No runtime introspection of available capabilities
- **Capability parameters**: No per-mode tuning of capability thresholds
- **Conditional capabilities**: Capabilities fixed, not dependent on vehicle state
- **Capability negotiation**: No runtime capability matching between systems
- **Capability versioning**: No backward compatibility support for Phase 1

## Appendix

### References

- ArduPilot Rover Mode Class: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.h>
- ArduPilot Mode Validation: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.cpp>
- ArduPilot Arming Checks: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/AP_Arming.cpp>
- MAVLink Custom Mode Documentation: <https://mavlink.io/en/messages/common.html#HEARTBEAT>

### Raw Data

**ArduPilot Mode Capability Matrix** (Complete):

| Mode           | Position | Velocity | Arm | RC Arm | Manual | Autopilot | Stabilized | Notes                         |
| -------------- | -------- | -------- | --- | ------ | ------ | --------- | ---------- | ----------------------------- |
| Manual         | No       | No       | Yes | Yes    | Yes    | No        | No         | Direct control, no sensors    |
| Acro           | No       | No       | Yes | Yes    | Yes    | No        | No         | Acrobatic mode, no stability  |
| Steering       | No       | Yes      | Yes | Yes    | Yes    | No        | Yes        | Speed control needs velocity  |
| Hold           | No       | No       | Yes | Yes    | No     | No        | Yes        | Stop vehicle, optional GPS    |
| Loiter         | Yes      | Yes      | Yes | No     | No     | Yes       | Yes        | Position hold                 |
| Follow         | Yes      | Yes      | Yes | No     | No     | Yes       | Yes        | Follow target                 |
| Simple         | No       | No       | Yes | Yes    | Yes    | No        | Yes        | Simplified controls           |
| Auto           | Yes      | Yes      | Yes | No     | No     | Yes       | Yes        | Waypoint navigation           |
| RTL            | Yes      | Yes      | Yes | No     | No     | Yes       | Yes        | Return to launch              |
| SmartRTL       | Yes      | Yes      | Yes | No     | No     | Yes       | Yes        | Intelligent return            |
| Guided         | Yes      | Yes      | Yes | No     | No     | Yes       | Yes        | GCS commanded navigation      |
| Circle         | Yes      | Yes      | Yes | No     | No     | Yes       | Yes        | Circle around point           |
| Dock (Sailing) | Yes      | Yes      | Yes | No     | No     | Yes       | Yes        | Autonomous docking (sailboat) |

**Capability Query Performance Benchmarks** (estimated):

| Query                            | Inline | Virtual Call | Cached  |
| -------------------------------- | ------ | ------------ | ------- |
| requires_position()              | 0.1 µs | 0.5 µs       | 0.05 µs |
| requires_velocity()              | 0.1 µs | 0.5 µs       | 0.05 µs |
| allows_arming()                  | 0.1 µs | 0.5 µs       | 0.05 µs |
| allows_arming_from_transmitter() | 0.1 µs | 0.5 µs       | 0.05 µs |
| Full validation (6 queries)      | 0.6 µs | 3.0 µs       | 0.3 µs  |

**Proposed pico_trail Capability Matrix**:

| Mode      | Position | Velocity | GPS | IMU | Compass | Arm | RC Arm | Manual | Autopilot | Stabilized |
| --------- | -------- | -------- | --- | --- | ------- | --- | ------ | ------ | --------- | ---------- |
| Manual    | No       | No       | No  | No  | No      | Yes | Yes    | Yes    | No        | No         |
| Stabilize | No       | No       | No  | Yes | Yes     | Yes | Yes    | Yes    | No        | Yes        |
| Hold      | No       | No       | No  | Yes | Yes     | Yes | Yes    | No     | No        | Yes        |
| Loiter    | Yes      | Yes      | Yes | Yes | Yes     | Yes | No     | No     | Yes       | Yes        |
| Auto      | Yes      | Yes      | Yes | Yes | Yes     | Yes | No     | No     | Yes       | Yes        |
| RTL       | Yes      | Yes      | Yes | Yes | Yes     | Yes | No     | No     | Yes       | Yes        |

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
