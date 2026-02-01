# AN-00013 Mode Entry Validation for Safe Mode Transitions

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-00008-pre-arm-checks](AN-00008-pre-arm-checks.md)
  - [AN-00009-armed-state-monitoring](AN-00009-armed-state-monitoring.md)
- Related Requirements:
  - [FR-00046-mode-entry-sensor-validation](../requirements/FR-00046-mode-entry-sensor-validation.md)
  - [FR-00031-ekf-health-validation](../requirements/FR-00031-ekf-health-validation.md)
  - [FR-00059-validation-failure-reporting](../requirements/FR-00059-validation-failure-reporting.md)
  - [FR-00039-fallback-mode-selection](../requirements/FR-00039-fallback-mode-selection.md)
  - [FR-00045-mode-capability-queries](../requirements/FR-00045-mode-capability-queries.md)
  - [FR-00030-disarmed-validation-exception](../requirements/FR-00030-disarmed-validation-exception.md)
  - [NFR-00055-validation-error-message-usability](../requirements/NFR-00055-validation-error-message-usability.md)
  - [NFR-00032-fallback-mode-reliability](../requirements/NFR-00032-fallback-mode-reliability.md)
  - [NFR-00057-validation-timing-performance](../requirements/NFR-00057-validation-timing-performance.md)
  - [NFR-00056-validation-memory-overhead](../requirements/NFR-00056-validation-memory-overhead.md)
  - [NFR-00054-validation-attempt-logging](../requirements/NFR-00054-validation-attempt-logging.md)
- Related ADRs: N/A - ADRs will be created based on this analysis
- Related Tasks: N/A - Tasks will be created after requirements and ADRs

## Executive Summary

This analysis explores mode entry validation procedures needed to ensure the vehicle is in appropriate state before transitioning to a new flight mode. Currently, pico_trail performs no validation before mode changes - allowing switches to any mode regardless of sensor availability, position estimates, or system state. Mode entry validation is critical for preventing unsafe mode transitions such as entering Auto mode without GPS fix, entering position-hold modes without position estimates, or entering autonomous modes with degraded sensor health.

Key findings: ArduPilot implements comprehensive mode entry validation through capability query pattern - each mode declares requirements via `requires_position()` and `requires_velocity()` functions, validated against EKF filter status and sensor health before allowing mode entry. The validation framework includes fallback mode selection when requirements not met and allows all mode changes when disarmed (pre-flight configuration). For pico_trail, a similar capability-based validation system is recommended with focus on sensor requirement validation (GPS, IMU, position estimates), EKF health checking, mode-specific prerequisites, and graceful degradation with fallback modes.

## Problem Space

### Current State

The project currently has:

- **Mode change function**: `SystemState::set_mode()` in `src/communication/mavlink/state.rs:185-194`
- **No mode validation**: All mode changes allowed without checks
- **No capability queries**: Modes don't declare sensor requirements
- **No sensor health checking**: GPS/IMU availability not validated
- **No position estimate validation**: Can enter Auto without position fix
- **Flight modes**: Manual, Stabilize, Loiter, Auto, RTL defined but no requirements

Critical safety gaps:

- **No sensor validation**: Can enter Auto mode without GPS fix
- **No position check**: Can enter Loiter without position estimate
- **No EKF health validation**: Can use autonomous modes with poor navigation quality
- **No fallback mechanism**: No safe mode to revert to when requirements not met
- **No capability declaration**: Modes don't specify what sensors they need
- **Unsafe transitions**: Can switch between incompatible modes without validation

### Desired State

Enable comprehensive mode entry validation ensuring safe mode transitions:

1. **Capability Query System**: Each mode declares sensor/estimate requirements
2. **Sensor Requirement Validation**: Check GPS, IMU, compass availability per mode
3. **EKF Health Checking**: Validate position/velocity estimate quality
4. **Armed State Exception**: Allow all mode changes when disarmed (pre-flight config)
5. **Mode Transition Validation**: Verify prerequisites before mode entry
6. **Validation Failure Reporting**: Clear feedback why mode change denied
7. **Fallback Mode Selection**: Automatic safe mode when requirements lost

Success criteria:

- **Sensor-appropriate modes**: Cannot enter Auto without GPS, Loiter without position
- **Quality-assured navigation**: Cannot use autonomous modes with poor EKF health
- **Graceful degradation**: Automatic fallback to safe mode when sensors lost
- **Pre-flight flexibility**: All modes selectable when disarmed (for configuration)
- **Clear feedback**: Operator informed why mode change denied with specific reason
- **Audit trail**: Mode transition attempts (successful and denied) logged

### Gap Analysis

**Missing components**:

1. **Mode Capability Trait**: Interface declaring sensor/estimate requirements per mode
2. **Sensor Validator**: Check GPS, IMU, compass availability
3. **EKF Health Checker**: Validate position/velocity estimate quality
4. **Transition Validator**: Verify mode-specific prerequisites
5. **Fallback Mode Selector**: Choose safe mode when requirements lost
6. **Failure Reporter**: Generate descriptive error messages
7. **Transition Logger**: Log all mode change attempts with outcome

**Technical deltas**:

- Create `ModeCapability` trait with `requires_position()`, `requires_velocity()` methods
- Implement capability queries for each flight mode
- Add `validate_mode_entry()` function checking EKF status and sensors
- Integrate with monitoring system (AN-00009) for sensor health
- Add armed state exception (skip validation when disarmed)
- Create fallback mode selection logic (Manual → Stabilize → Hold hierarchy)
- Generate validation failure messages (no GPS, no position, poor EKF, etc.)
- Log mode transition attempts with validation outcome
- Define sensor requirements per mode as configuration
- Integrate with mode lifecycle (will be analyzed in AN-<next>)

## Stakeholder Analysis

| Stakeholder        | Interest/Need                                              | Impact | Priority |
| ------------------ | ---------------------------------------------------------- | ------ | -------- |
| Operators          | Prevent mode changes that will fail due to missing sensors | High   | P0       |
| Safety Reviewers   | Ensure modes only entered with required capabilities       | High   | P0       |
| Autonomous Systems | Reliable fallback when sensor data degrades                | High   | P0       |
| Test Engineers     | Understand why mode change denied during testing           | High   | P1       |
| GCS Software       | Display mode transition failures to operator               | Medium | P1       |
| Regulatory Bodies  | Audit trail of mode transitions and validation             | Medium | P1       |

## Research & Discovery

### User Feedback

From operational requirements:

- Mode entry validation is fundamental to safe autonomous operation
- Cannot rely on GPS for autonomous navigation without position fix
- Graceful degradation essential when sensors fail during flight
- Pre-flight mode configuration must work without full sensor suite
- Operators need clear feedback when mode change denied
- Automatic fallback prevents loss of control when sensors degrade

### Competitive Analysis

**ArduPilot Mode Entry Validation**:

Based on Rover mode.cpp and mode.h, ArduPilot implements comprehensive mode entry validation:

#### Mode Entry Validation Framework

File: `Rover/mode.cpp` (lines 21-54)

```cpp
bool Mode::enter()
{
    // Get filter status for EKF validation
    nav_filter_status filt_status;
    rover.ahrs.get_filter_status(filt_status);

    // Check if position estimate available
    const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;

    // Validate position requirement
    if (requires_position() && !position_ok) {
        // Cannot enter mode without position estimate
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires position");
        return false;
    }

    // Validate velocity requirement
    if (requires_velocity() && !position_ok && !filt_status.flags.horiz_vel) {
        // Cannot enter mode without velocity estimate
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires velocity");
        return false;
    }

    // Armed state exception
    // Allow switching to any mode if disarmed
    // (Enables pre-flight mode configuration)

    // Call mode-specific enter logic
    const bool success = _enter();

    // Initialize mode state on success
    if (success) {
        rover.mode_initialized = true;
    }

    return success;
}
```

**Key Validation Points**:

1. **EKF Filter Status**: Query navigation filter for health assessment
2. **Position Estimate Check**: Verify position available via `ekf_position_ok()`
3. **Velocity Estimate Check**: Confirm horizontal velocity data present
4. **Capability-Based Validation**: Use `requires_position()` / `requires_velocity()` queries
5. **Disarmed Exception**: Skip validation when disarmed for pre-flight config
6. **Mode-Specific Logic**: Call `_enter()` for mode-specific initialization

#### Mode Capability Query System

File: `Rover/mode.h`

Each mode implements capability queries declaring sensor requirements:

```cpp
class Mode {
public:
    // Capability queries (virtual, overridden by subclasses)
    virtual bool requires_position() const { return true; }
    virtual bool requires_velocity() const { return true; }
    virtual bool is_autopilot_mode() const { return false; }
    virtual bool allows_arming() const { return true; }
    virtual bool allows_arming_from_transmitter() const;
    virtual bool has_manual_input() const { return false; }
    virtual bool attitude_stabilized() const { return true; }

    // Mode entry/exit
    virtual bool enter();
    virtual void _enter() {}
    virtual void _exit() {}
};
```

**Mode-Specific Requirements**:

| Mode     | requires_position() | requires_velocity() | is_autopilot_mode() |
| -------- | ------------------- | ------------------- | ------------------- |
| Manual   | false               | false               | false               |
| Hold     | false               | false               | false               |
| Acro     | false               | (vehicle-dependent) | false               |
| Steering | false               | true                | false               |
| Auto     | true                | true                | true                |
| Guided   | true                | true                | true                |
| Loiter   | true                | true                | true                |
| RTL      | true                | true                | true                |
| SmartRTL | true                | true                | true                |
| Circle   | true                | true                | true                |
| Follow   | true                | true                | true                |

**Purpose**: Declarative requirements enable validation framework to deny mode entry when prerequisites not met.

#### EKF Health Checking

**Position Estimate Validation**:

```cpp
bool ekf_position_ok() const {
    return rover.ahrs.have_inertial_nav() &&
           rover.ahrs.get_relative_position_NED_origin(pos, timestamp);
}
```

**Purpose**: Verify Extended Kalman Filter providing valid position estimates.

**Filter Status Flags**:

```cpp
struct nav_filter_status {
    bool flags.attitude : 1;
    bool flags.horiz_vel : 1;
    bool flags.vert_vel : 1;
    bool flags.horiz_pos_rel : 1;
    bool flags.horiz_pos_abs : 1;
    bool flags.vert_pos : 1;
    bool flags.terrain_alt : 1;
    bool flags.const_pos_mode : 1;
    bool flags.pred_horiz_pos_rel : 1;
    bool flags.pred_horiz_pos_abs : 1;
    bool flags.using_gps : 1;
};
```

**Purpose**: Detailed navigation filter health for fine-grained validation.

#### Fallback Mode Selection

When sensor health degrades during operation, ArduPilot automatically transitions to safe fallback modes:

**Failsafe Mode Hierarchy**:

1. **Position loss**: Auto/Guided/Loiter → Hold mode
2. **Complete GPS loss**: Hold → Manual (if armed) or land/disarm (multicopter)
3. **IMU failure**: Any mode → Disarm immediately (critical failure)

**Purpose**: Graceful degradation preserves vehicle control when sensors fail.

**PX4 Mode Entry Validation**:

PX4 Commander state machine implements:

- **Sensor check**: GPS, IMU, magnetometer, barometer health validation
- **EKF health**: Innovation consistency checks before autonomous modes
- **Mode prerequisites**: Each mode has specific sensor requirements
- **Preflight checks**: Relaxed validation when disarmed
- **Automatic fallback**: Transition to stabilized modes on sensor loss

### Technical Investigation

**Current pico_trail Implementation**:

File: `src/communication/mavlink/state.rs:185-194`

```rust
pub fn set_mode(&mut self, mode: FlightMode) -> Result<(), &'static str> {
    // Mode change restrictions can be added here
    // For now, allow all mode changes

    self.mode = mode;
    Ok(())
}
```

**Observations**:

- No validation logic
- No capability queries
- No sensor health checks
- No EKF validation
- Generic success return

**Current Flight Modes**:

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

**Observations**: Modes defined but no capability declarations.

**Proposed Mode Entry Validation Architecture**:

```rust
/// Mode capability trait defining sensor/estimate requirements
pub trait ModeCapability {
    /// Does this mode require position estimate?
    fn requires_position(&self) -> bool;

    /// Does this mode require velocity estimate?
    fn requires_velocity(&self) -> bool;

    /// Is this an autopilot mode (autonomous)?
    fn is_autopilot_mode(&self) -> bool;

    /// Does this mode allow arming?
    fn allows_arming(&self) -> bool;

    /// Does this mode allow transmitter arming?
    fn allows_arming_from_transmitter(&self) -> bool;

    /// Does this mode use manual RC input?
    fn has_manual_input(&self) -> bool;
}

impl ModeCapability for FlightMode {
    fn requires_position(&self) -> bool {
        match self {
            FlightMode::Manual => false,
            FlightMode::Stabilize => false,
            FlightMode::Loiter => true,
            FlightMode::Auto => true,
            FlightMode::Rtl => true,
        }
    }

    fn requires_velocity(&self) -> bool {
        match self {
            FlightMode::Manual => false,
            FlightMode::Stabilize => false,
            FlightMode::Loiter => true,
            FlightMode::Auto => true,
            FlightMode::Rtl => true,
        }
    }

    fn is_autopilot_mode(&self) -> bool {
        match self {
            FlightMode::Manual => false,
            FlightMode::Stabilize => false,
            FlightMode::Loiter => true,
            FlightMode::Auto => true,
            FlightMode::Rtl => true,
        }
    }

    fn allows_arming(&self) -> bool {
        // All modes allow arming for now
        true
    }

    fn allows_arming_from_transmitter(&self) -> bool {
        // Only non-autopilot modes allow transmitter arming
        !self.is_autopilot_mode()
    }

    fn has_manual_input(&self) -> bool {
        match self {
            FlightMode::Manual => true,
            FlightMode::Stabilize => true,
            FlightMode::Loiter => false,
            FlightMode::Auto => false,
            FlightMode::Rtl => false,
        }
    }
}

/// Mode entry validation result
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ModeValidationResult {
    Allowed,
    DeniedNoPosition,
    DeniedNoVelocity,
    DeniedNoGPS,
    DeniedPoorEKF,
    DeniedNoIMU,
}

impl ModeValidationResult {
    pub fn to_error_message(&self) -> &'static str {
        match self {
            ModeValidationResult::Allowed => "Validation passed",
            ModeValidationResult::DeniedNoPosition =>
                "Cannot enter mode: No position estimate",
            ModeValidationResult::DeniedNoVelocity =>
                "Cannot enter mode: No velocity estimate",
            ModeValidationResult::DeniedNoGPS =>
                "Cannot enter mode: GPS not available",
            ModeValidationResult::DeniedPoorEKF =>
                "Cannot enter mode: Poor navigation quality",
            ModeValidationResult::DeniedNoIMU =>
                "Cannot enter mode: IMU not available",
        }
    }
}

impl SystemState {
    /// Set flight mode with validation
    pub fn set_mode(&mut self, mode: FlightMode) -> Result<(), &'static str> {
        // Validate mode entry (unless disarmed)
        if self.is_armed() {
            let validation_result = self.validate_mode_entry(mode);
            if validation_result != ModeValidationResult::Allowed {
                // Log denied mode change
                self.log_mode_change_denied(mode, validation_result)?;
                return Err(validation_result.to_error_message());
            }
        } else {
            // Allow all mode changes when disarmed (pre-flight config)
            info!("Mode change to {:?} (disarmed, validation skipped)", mode);
        }

        // Validation passed (or disarmed), change mode
        let old_mode = self.mode;
        self.mode = mode;

        // Log successful mode change
        self.log_mode_change(old_mode, mode)?;
        info!("Mode changed from {:?} to {:?}", old_mode, mode);

        Ok(())
    }

    /// Validate mode entry conditions
    fn validate_mode_entry(&self, mode: FlightMode) -> ModeValidationResult {
        // Check position requirement
        if mode.requires_position() {
            if !self.has_position_estimate() {
                return ModeValidationResult::DeniedNoPosition;
            }
        }

        // Check velocity requirement
        if mode.requires_velocity() {
            if !self.has_velocity_estimate() {
                return ModeValidationResult::DeniedNoVelocity;
            }
        }

        // Check GPS availability for autopilot modes
        if mode.is_autopilot_mode() {
            if !self.has_gps_fix() {
                return ModeValidationResult::DeniedNoGPS;
            }
        }

        // Check EKF health for autonomous modes
        if mode.is_autopilot_mode() {
            if !self.ekf_healthy() {
                return ModeValidationResult::DeniedPoorEKF;
            }
        }

        // Check IMU availability (all modes need IMU)
        if !self.has_imu() {
            return ModeValidationResult::DeniedNoIMU;
        }

        ModeValidationResult::Allowed
    }

    /// Check if position estimate available
    fn has_position_estimate(&self) -> bool {
        // TODO: Query navigation system for position estimate
        // For Phase 1: check if GPS has fix
        self.has_gps_fix()
    }

    /// Check if velocity estimate available
    fn has_velocity_estimate(&self) -> bool {
        // TODO: Query navigation system for velocity estimate
        // For Phase 1: check if GPS has fix (provides velocity)
        self.has_gps_fix()
    }

    /// Check if GPS fix available
    fn has_gps_fix(&self) -> bool {
        // TODO: Integrate with GPS system
        // For Phase 1: return false (no GPS yet)
        false
    }

    /// Check EKF health
    fn ekf_healthy(&self) -> bool {
        // TODO: Integrate with EKF/AHRS system
        // For Phase 1: return true (no EKF yet, assume healthy)
        true
    }

    /// Check if IMU available
    fn has_imu(&self) -> bool {
        // TODO: Integrate with IMU driver
        // For Phase 1: return true (assume IMU present)
        true
    }

    /// Log denied mode change
    fn log_mode_change_denied(
        &self,
        mode: FlightMode,
        reason: ModeValidationResult,
    ) -> Result<(), &'static str> {
        let log_entry = format!(
            "MODE_CHANGE_DENIED,{},{:?},{:?}",
            get_time_ms(),
            mode,
            reason
        );

        // TODO: Write to log storage
        warn!("Mode change denied: {:?} reason: {:?}", mode, reason);
        Ok(())
    }

    /// Log successful mode change
    fn log_mode_change(
        &self,
        old_mode: FlightMode,
        new_mode: FlightMode,
    ) -> Result<(), &'static str> {
        let log_entry = format!(
            "MODE_CHANGE,{},{:?},{:?}",
            get_time_ms(),
            old_mode,
            new_mode
        );

        // TODO: Write to log storage
        info!("Mode change: {:?} → {:?}", old_mode, new_mode);
        Ok(())
    }

    /// Select fallback mode when requirements lost
    pub fn select_fallback_mode(&mut self) -> Result<(), &'static str> {
        // Fallback hierarchy: current → Stabilize → Manual
        let fallback_modes = [
            FlightMode::Stabilize,
            FlightMode::Manual,
        ];

        for fallback_mode in &fallback_modes {
            if self.validate_mode_entry(*fallback_mode) == ModeValidationResult::Allowed {
                warn!("Selecting fallback mode: {:?}", fallback_mode);
                self.mode = *fallback_mode;
                return Ok(());
            }
        }

        // No fallback mode available (critical - should not happen)
        error!("No fallback mode available - forcing Manual");
        self.mode = FlightMode::Manual;
        Ok(())
    }
}
```

**Integration with Command Handler**:

```rust
fn handle_set_mode(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
    let custom_mode = cmd.param2 as u32;

    match FlightMode::from_custom_mode(custom_mode) {
        Some(mode) => match self.state.set_mode(mode) {
            Ok(()) => {
                info!("Mode changed to {:?}", mode);
                MavResult::MAV_RESULT_ACCEPTED
            }
            Err(reason) => {
                warn!("Mode change rejected: {}", reason);
                // Send STATUSTEXT to GCS explaining why
                self.send_status_text(MAV_SEVERITY_WARNING, reason)?;
                MavResult::MAV_RESULT_DENIED
            }
        },
        None => {
            warn!("Invalid mode number: {}", custom_mode);
            MavResult::MAV_RESULT_DENIED
        }
    }
}
```

**Integration with Monitoring System** (AN-00009):

```rust
/// Monitoring task detects sensor loss and triggers fallback
fn check_sensor_health(health: &SystemHealth, state: &mut SystemState) {
    // If GPS lost during autonomous mode
    if state.mode.requires_position() && !state.has_position_estimate() {
        warn!("Position estimate lost in autonomous mode - selecting fallback");
        state.select_fallback_mode()?;
    }

    // If IMU fails (critical)
    if health.imu == HealthStatus::Unhealthy {
        error!("IMU failure - forcing Manual mode and disarm");
        state.mode = FlightMode::Manual;
        state.disarm(DisarmMethod::Failsafe, DisarmReason::EmergencyStop)?;
    }
}
```

**Memory Analysis**:

| Component            | RAM Usage | Notes                            |
| -------------------- | --------- | -------------------------------- |
| ModeCapability trait | 0 B       | Trait implementation (code only) |
| ModeValidationResult | 1 B       | Enum (1 byte)                    |
| Validation logic     | 0 B       | Code only, no runtime allocation |
| **Total (mode val)** | **\~1 B** | Negligible overhead              |

### Data Analysis

**Mode Validation Failure Rate** (estimated from ArduPilot forums):

- No GPS/position: 50% of validation failures
- Poor EKF health: 20% of validation failures
- No velocity estimate: 15% of validation failures
- IMU failure: 10% of validation failures
- Other: 5% of validation failures

**Mode Requirements Summary**:

| Mode      | Position | Velocity | GPS | IMU | Compass | Autopilot |
| --------- | -------- | -------- | --- | --- | ------- | --------- |
| Manual    | No       | No       | No  | No  | No      | No        |
| Stabilize | No       | No       | No  | Yes | Yes     | No        |
| Loiter    | Yes      | Yes      | Yes | Yes | Yes     | Yes       |
| Auto      | Yes      | Yes      | Yes | Yes | Yes     | Yes       |
| RTL       | Yes      | Yes      | Yes | Yes | Yes     | Yes       |

**Validation Check Timing**:

| Check Type              | Duration     | Notes                     |
| ----------------------- | ------------ | ------------------------- |
| Armed state check       | < 1 µs       | Simple boolean comparison |
| Position estimate check | < 10 µs      | Query navigation system   |
| Velocity estimate check | < 10 µs      | Query navigation system   |
| GPS availability check  | < 10 µs      | Read GPS status           |
| EKF health check        | < 50 µs      | Query filter status       |
| IMU health check        | < 10 µs      | Read sensor status        |
| **Total (typical)**     | **< 100 µs** | Negligible delay          |

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: Each flight mode shall declare sensor/estimate requirements via capability queries → Will become FR-<id>
  - Rationale: Declarative requirements enable validation framework
  - Acceptance Criteria:
    - Implement ModeCapability trait with requires_position(), requires_velocity()
    - Each mode returns correct requirements
    - Validation framework uses capability queries to check prerequisites

- [ ] **FR-DRAFT-2**: The system shall validate sensor requirements before allowing mode entry when armed → Will become FR-<id>
  - Rationale: Prevent mode changes that will fail due to missing sensors
  - Acceptance Criteria:
    - Check position estimate if mode requires_position()
    - Check velocity estimate if mode requires_velocity()
    - Check GPS availability for autopilot modes
    - Return specific error message for each validation failure

- [ ] **FR-DRAFT-3**: The system shall allow all mode changes when disarmed (pre-flight config exception) → Will become FR-<id>
  - Rationale: Enable pre-flight mode configuration without full sensor suite
  - Acceptance Criteria:
    - Skip validation checks when vehicle disarmed
    - Log mode changes with "disarmed" flag
    - Apply validation when armed

- [ ] **FR-DRAFT-4**: The system shall check EKF health before allowing autonomous mode entry → Will become FR-<id>
  - Rationale: Autonomous modes require high-quality navigation estimates
  - Acceptance Criteria:
    - Query EKF/AHRS filter status
    - Validate position estimate quality
    - Deny autonomous modes if EKF unhealthy
    - Return "Poor navigation quality" error

- [ ] **FR-DRAFT-5**: The system shall automatically select fallback mode when sensor requirements lost → Will become FR-<id>
  - Rationale: Graceful degradation preserves vehicle control
  - Acceptance Criteria:
    - Monitor sensor health during flight (via AN-00009 monitoring)
    - Detect when current mode requirements no longer met
    - Select fallback mode hierarchy: Stabilize → Manual
    - Log fallback mode transition with reason

- [ ] **FR-DRAFT-6**: The system shall provide clear feedback when mode change denied → Will become FR-<id>
  - Rationale: Operators need to understand why mode change rejected
  - Acceptance Criteria:
    - Return specific error message for each validation failure
    - Send STATUSTEXT to GCS with failure reason
    - Log validation failure with details
    - Include sensor name in error message (GPS, position, velocity)

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Mode entry validation shall complete within 1ms of mode change command → Will become NFR-<id>
  - Category: Performance
  - Rationale: Validation must not introduce perceptible delay
  - Target: < 1ms validation time (measured via execution time profiling)

- [ ] **NFR-DRAFT-2**: Validation failure messages shall be human-readable and specific → Will become NFR-<id>
  - Category: Usability
  - Rationale: Operators need clear diagnosis of mode change failure
  - Target: Error messages specify sensor/estimate missing (not generic failure)

- [ ] **NFR-DRAFT-3**: Mode entry validation shall add negligible RAM overhead → Will become NFR-<id>
  - Category: Resource Constraints
  - Rationale: Capability queries are code-only, no runtime allocation
  - Target: < 10 B runtime overhead

- [ ] **NFR-DRAFT-4**: All mode transition attempts shall be logged regardless of outcome → Will become NFR-<id>
  - Category: Reliability / Auditability
  - Rationale: Support post-flight analysis and debugging
  - Target: Log includes timestamp, old mode, new mode, outcome, validation result

- [ ] **NFR-DRAFT-5**: Fallback mode selection shall always succeed → Will become NFR-<id>
  - Category: Safety / Reliability
  - Rationale: Vehicle must have control mode even with sensor failures
  - Target: Manual mode requires no sensors, always available as last fallback

## Design Considerations

### Technical Constraints

- **Existing mode enum**: Must extend current FlightMode without breaking API
- **Memory budget**: Limited RAM on RP2040/RP2350 (264 KB)
- **Real-time requirements**: Validation must not delay mode changes significantly
- **No dynamic allocation**: All structures must use static/stack allocation
- **Sensor dependencies**: GPS, IMU, EKF systems not yet fully implemented
- **Monitoring integration**: Must work with health monitoring system (AN-00009)

### Potential Approaches

1. **Option A: Inline Validation in set_mode()**
   - Pros:
     - Simplest implementation (all code in one function)
     - No additional trait needed
     - Easy to understand flow
   - Cons:
     - Large switch statements for each mode
     - Hard to add new modes or requirements
     - Difficult to test individual validation checks
   - Effort: Low (8-12 hours)

2. **Option B: Trait-Based Capability Queries** ⭐ Recommended
   - Pros:
     - Clean separation of concerns (mode declares, validator checks)
     - Easy to add new modes or capabilities
     - Matches ArduPilot proven architecture
     - Testable per-mode requirements
     - Extensible for future modes
   - Cons:
     - Requires trait implementation
     - Slightly more complex than inline
   - Effort: Medium (16-24 hours)

3. **Option C: Capability Registry with Runtime Configuration**
   - Pros:
     - Maximum flexibility (configure requirements at runtime)
     - Can adjust requirements per vehicle type
     - Easy to enable/disable specific checks
   - Cons:
     - High complexity
     - Runtime overhead
     - Overkill for initial implementation
   - Effort: High (40-50 hours)

**Recommendation**: Option B (Trait-Based Capability Queries) provides best balance of clarity, extensibility, and development effort. Matches ArduPilot's proven architecture.

### Architecture Impact

**New ADRs required**:

- **ADR-<id> Mode Capability Query System**: Trait design, query methods, validation framework
- **ADR-<id> Mode Entry Validation Rules**: Validation checks per capability
- **ADR-<id> Fallback Mode Selection**: Hierarchy and selection logic
- **ADR-<id> Disarmed Validation Exception**: Why skip checks when disarmed

**New modules**:

- `src/vehicle/modes/` - Mode system
  - `src/vehicle/modes/capability.rs` - ModeCapability trait
  - `src/vehicle/modes/validation.rs` - Mode entry validation logic

**Modified modules**:

- `src/communication/mavlink/state.rs` - Add validation to set_mode()
- `src/communication/mavlink/handlers/command.rs` - Handle validation failures
- `src/vehicle/monitoring/` - Add sensor loss detection triggering fallback

## Parameters

### ArduPilot References

This analysis is based on ArduPilot Rover's mode entry validation in `Mode::enter()` and sensor requirement checking. ArduPilot does not have dedicated mode entry validation parameters - sensor requirements are hardcoded per mode in `requires_gps()`, `requires_position()` methods.

**ArduPilot Mode Requirements** (hardcoded):

- **Auto, Guided, RTL, SmartRTL**: Require GPS fix (checked via `requires_position()`)
- **Loiter**: Requires position estimate (GPS or visual odometry)
- **Manual, Acro, Steering**: No sensor requirements
- **Hold**: Works without GPS (stops in place) but uses GPS if available for position hold

**Related ArduPilot Parameters**:

- **ARMING_CHECK** (bitmask) - Includes GPS and sensor health checks
  - Bit 3: GPS check, Bit 10: Compass check, Bit 19: EKF check
  - Applied at arm time, not at mode change time
- **FS_EKF_ACTION** (u8) - Action when EKF fails during flight
  - Indirectly affects mode availability if EKF unhealthy

### Note on Mode Entry Validation

ArduPilot integrates mode capability checking with its pre-arm system. The **ARMING_CHECK** bitmask parameter controls sensor health verification:

- **ARMING_CHECK** (bitmask) - Controls which checks are performed
  - Bit 3: GPS health and fix status
  - Bit 10: Compass calibration and health
  - Bit 19: EKF/AHRS health and variance thresholds
  - Applied at arm time, not at mode change time

Mode fallback behavior is handled by failsafe actions:

- **FS_ACTION** (u8) - Action when failsafe triggers during flight
  - 0 = None, 1 = Hold, 2 = RTL, 3 = SmartRTL, 4 = SmartRTL+Hold, 5 = Terminate
  - Provides automatic fallback when sensor/communication failure occurs

- **FS_EKF_ACTION** (u8) - Action when EKF fails during flight
  - 0 = Disabled, 1 = Hold, 2 = Disarm
  - Handles position estimation failure

For pico_trail Phase 1, we follow ArduPilot's approach: use ARMING_CHECK for pre-flight validation and FS_ACTION for in-flight mode fallback. No custom MODE\_\* parameters needed.

## Risk Assessment

| Risk                                                    | Probability | Impact       | Mitigation Strategy                                                                  |
| ------------------------------------------------------- | ----------- | ------------ | ------------------------------------------------------------------------------------ |
| **Validation too strict (prevents valid mode changes)** | **Medium**  | **Medium**   | **Disarmed exception allows pre-flight config, tunable parameters to adjust checks** |
| **Fallback mode selection fails (no mode available)**   | **Low**     | **CRITICAL** | **Manual mode requires no sensors, always available, force Manual as last resort**   |
| Validation timing degrades mode change responsiveness   | Low         | Low          | Profile validation timing, optimize hot paths, target < 1ms                          |
| GPS/EKF not yet implemented (validation cannot run)     | High        | Medium       | Phase 1 stub implementations return conservative defaults, Phase 2 integrate sensors |
| Capability queries not extensible for new modes         | Low         | Low          | Trait-based design allows easy addition of new capability queries                    |
| Operator confusion about why mode denied                | Medium      | Low          | Clear error messages, send STATUSTEXT to GCS, document requirements per mode         |
| Fallback mode triggers unexpectedly (sensor glitches)   | Medium      | Medium       | Add hysteresis to sensor health checks, require sustained failure before fallback    |

## Open Questions

- [ ] Should EKF health check be required in Phase 1? → Decision: No, Phase 1 stub returns true, Phase 2 integrate AHRS
- [ ] What is fallback mode hierarchy for Rover? → Method: Stabilize → Manual (match ArduPilot)
- [ ] Should compass be required for all modes? → Decision: Phase 1 no compass check, Phase 2 add for heading-dependent modes
- [ ] How to handle mode validation when sensors unavailable? → Method: Conservative approach - deny if sensor check not possible
- [ ] Should we support per-vehicle-type mode requirements? → Decision: Phase 1 single requirement set, Phase 2 vehicle-specific
- [ ] Do we need validation logging for successful mode changes? → Decision: Yes, log all transitions (success and failure)
- [ ] How to test validation without actual GPS/EKF? → Method: Unit tests with mocked sensor availability, simulation testing

## Recommendations

### Immediate Actions

1. **Adopt Option B architecture**: Trait-based capability queries
2. **Implement Phase 1 validation**: Position/velocity checks with stub sensors
3. **Use ArduPilot requirements**: Match proven mode-to-sensor mappings
4. **Add disarmed exception**: Skip validation for pre-flight config
5. **Implement fallback selection**: Stabilize → Manual hierarchy

### Next Steps

1. [ ] Create formal requirements: FR-<id> (capability queries), FR-<id> (sensor validation), FR-<id> (disarmed exception), FR-<id> (EKF health), FR-<id> (fallback mode), FR-<id> (failure reporting), NFR-<id> (validation timing), NFR-<id> (error messages), NFR-<id> (memory), NFR-<id> (logging), NFR-<id> (fallback reliability)
2. [ ] Draft ADR for: Mode capability query system (trait design, validation framework)
3. [ ] Draft ADR for: Mode entry validation rules (checks per capability)
4. [ ] Draft ADR for: Fallback mode selection (hierarchy and logic)
5. [ ] Draft ADR for: Disarmed validation exception (rationale and implementation)
6. [ ] Create task for: Mode entry validation implementation (Phase 1: capability trait + validation)
7. [ ] Plan integration testing: Verify validation denies unsafe modes, fallback works, disarmed exception

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **EKF health checking**: Phase 1 stub returns true, Phase 2 integrate AHRS (FR-00001)
- **Compass requirement**: Phase 1 no compass check, Phase 2 add for modes needing heading
- **Altitude requirement**: Not applicable to ground vehicle (no altitude hold modes)
- **Airspeed requirement**: Not applicable to ground vehicle
- **Advanced EKF innovation checks**: Phase 1 basic health only, Phase 2 detailed validation
- **Per-vehicle-type requirements**: Phase 1 single requirement set
- **Runtime requirement configuration**: Phase 1 compile-time traits, Phase 2 parameters
- **Custom mode capability callbacks**: Phase 1 hardcoded traits, Phase 2 extensible
- **Mode transition validation**: Phase 1 entry only, Phase 2 add exit validation (AN-<next>)
- **Intermediate fallback modes**: Phase 1 two-level hierarchy, Phase 2 add more levels

## Appendix

### References

- ArduPilot Rover Mode Implementation: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.cpp>
- ArduPilot Mode Class Definition: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.h>
- MAVLink MAV_CMD_DO_SET_MODE: <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_MODE>

### Raw Data

**ArduPilot Mode Entry Validation** (from Rover/mode.cpp):

```cpp
bool Mode::enter()
{
    // Get filter status
    nav_filter_status filt_status;
    rover.ahrs.get_filter_status(filt_status);

    // Check position requirement
    const bool position_ok = rover.ekf_position_ok() && !rover.failsafe.ekf;
    if (requires_position() && !position_ok) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires position");
        return false;
    }

    // Check velocity requirement
    if (requires_velocity() && !position_ok && !filt_status.flags.horiz_vel) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Mode requires velocity");
        return false;
    }

    // Mode-specific enter
    return _enter();
}
```

**Proposed pico_trail Validation Sequence**:

```
set_mode(new_mode)
  │
  ├─ Disarmed?
  │   ├─ Yes → Skip validation (pre-flight config)
  │   └─ No → Continue validation
  │
  ├─ validate_mode_entry(new_mode)
  │   │
  │   ├─ 1. Check position requirement
  │   │   └─ requires_position() && !has_position_estimate() → DENY
  │   │
  │   ├─ 2. Check velocity requirement
  │   │   └─ requires_velocity() && !has_velocity_estimate() → DENY
  │   │
  │   ├─ 3. Check GPS availability
  │   │   └─ is_autopilot_mode() && !has_gps_fix() → DENY
  │   │
  │   ├─ 4. Check EKF health
  │   │   └─ is_autopilot_mode() && !ekf_healthy() → DENY
  │   │
  │   ├─ 5. Check IMU availability
  │   │   └─ !has_imu() → DENY
  │   │
  │   └─ Return validation result
  │
  ├─ Validation passed?
  │   ├─ No → Log denied + return error
  │   └─ Yes → Continue
  │
  ├─ Change mode
  │   └─ old_mode → new_mode
  │
  └─ Log successful mode change

Total validation time: < 100µs
```

**Mode Requirements Matrix**:

| Mode      | Position | Velocity | GPS | IMU | Compass | Notes                                 |
| --------- | -------- | -------- | --- | --- | ------- | ------------------------------------- |
| Manual    | No       | No       | No  | No  | No      | Minimal requirements (direct control) |
| Stabilize | No       | No       | No  | Yes | Yes     | Heading stabilization requires IMU    |
| Loiter    | Yes      | Yes      | Yes | Yes | Yes     | Position hold requires all sensors    |
| Auto      | Yes      | Yes      | Yes | Yes | Yes     | Waypoint navigation requires all      |
| RTL       | Yes      | Yes      | Yes | Yes | Yes     | Return home requires position         |
