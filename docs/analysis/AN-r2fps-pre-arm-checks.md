# AN-r2fps Pre-Arm Safety Checks and Validation Framework

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-yqeju-manual-control-implementation](AN-yqeju-manual-control-implementation.md)
  - [AN-cp76d-ardupilot-analysis](AN-cp76d-ardupilot-analysis.md)
- Related Requirements:
  - [FR-n1mte-prearm-capability-enforcement](../requirements/FR-n1mte-prearm-capability-enforcement.md)
  - [FR-qj0d1-mode-capability-declaration](../requirements/FR-qj0d1-mode-capability-declaration.md)
  - [FR-exazo-force-arm-override](../requirements/FR-exazo-force-arm-override.md)
- Related ADRs:
  - [ADR-w8d02-arming-system-architecture](../adr/ADR-w8d02-arming-system-architecture.md)
- Related Tasks:
  - [T-zmv9u-arming-system-implementation](../tasks/T-zmv9u-arming-system-implementation/README.md)

## Executive Summary

This analysis explores the pre-arm validation framework needed before transitioning from DISARMED to ARMED state. Currently, the project implements basic arming/disarming via MAVLink COMPONENT_ARM_DISARM command, but lacks safety validation to prevent arming when the vehicle is not flight-ready. Pre-arm checks are critical safety mechanisms that validate sensor health, calibration status, configuration validity, and system readiness before allowing motor/actuator activation.

Key findings: ArduPilot implements a comprehensive pre-arm check framework through the AP_Arming library, executing 30+ sequential validation checks across sensors, configuration, hardware, and mission planning. The framework uses a fail-fast pattern where any check failure prevents arming and reports specific failure reasons to the operator via GCS. For pico_trail, a minimal viable subset focusing on critical safety checks (RC input, system state, actuator readiness) is recommended for initial implementation, with expansion to sensor validation (IMU, GPS, battery) in subsequent phases.

## Problem Space

### Current State

The project currently has:

- **Basic arming logic**: `SystemState::arm()` and `SystemState::disarm()` methods in `src/communication/mavlink/state.rs:160-182`
- **Arming command handler**: MAV_CMD_COMPONENT_ARM_DISARM implemented in `src/communication/mavlink/handlers/command.rs:84-113`
- **Simple validation**: Only checks if already armed/disarmed (duplicate state check)
- **No pre-arm checks**: Arming succeeds regardless of sensor health, calibration status, or configuration validity
- **No failure reporting**: No mechanism to communicate why arming was denied

Critical safety gaps:

- **RC input not validated**: Can arm without RC connection or with invalid RC calibration
- **Sensor health ignored**: Can arm with failed IMU, compass, or GPS
- **Configuration unchecked**: No validation of PWM output configuration, parameter sanity, or mode requirements
- **Battery state unknown**: No check for low battery voltage or battery health
- **Actuator state not verified**: No confirmation that servos/motors are ready
- **System state incomplete**: No check for active errors, storage issues, or subsystem failures

### Desired State

Enable safe arming workflow with comprehensive pre-arm validation:

1. **Operator requests arming** via Mission Planner (MAV_CMD_COMPONENT_ARM_DISARM)
2. **System executes pre-arm checks** in priority order (critical to advisory)
3. **If all checks pass**: Vehicle arms, sends ACK, logs event, enables actuators
4. **If any check fails**: Vehicle remains disarmed, sends NACK with specific failure reason, displays error in GCS
5. **Operator resolves issue** (e.g., calibrate compass, connect RC, fix configuration)
6. **Operator retries arming** until all checks pass

Success criteria:

- **Safety-first**: Vehicle cannot arm if critical systems are not ready
- **Clear feedback**: Operator knows exactly which check failed and how to fix it
- **Configurable**: Checks can be selectively disabled for bench testing (via ARMING_CHECK parameter)
- **Extensible**: New checks can be added without modifying core arming logic
- **Consistent**: Same check framework used across all vehicle types (rover, plane)
- **Auditable**: All arming attempts and check failures logged for post-flight analysis

### Gap Analysis

**Missing components**:

1. **Pre-arm Check Framework**: Infrastructure to register, execute, and report validation checks
2. **Check Implementations**: Individual check functions for RC, sensors, configuration, etc.
3. **Failure Reporting**: Mechanism to send failure reasons to GCS and log locally
4. **ARMING_CHECK Parameter**: Configuration to enable/disable specific check categories
5. **Check State Tracking**: Remember which checks failed to avoid repeated reporting
6. **Integration with Arming Logic**: Hook pre-arm checks into SystemState::arm() method

**Technical deltas**:

- Add `src/core/arming/` module with check framework
- Implement check trait: `PreArmCheck` with `check()` method returning Result
- Create check registry: `PreArmChecker` that runs all registered checks
- Modify `SystemState::arm()` to call pre-arm checks before arming
- Add parameter: `ARMING_CHECK` (bitmask for selective disabling)
- Implement initial checks: RC input, system state, actuator readiness
- Add GCS messaging for check failures (STATUSTEXT messages)
- Log arming attempts and failures to storage

## Stakeholder Analysis

| Stakeholder         | Interest/Need                                             | Impact | Priority |
| ------------------- | --------------------------------------------------------- | ------ | -------- |
| Operators           | Prevent unsafe arming, clear feedback on failures         | High   | P0       |
| Developers          | Catch configuration errors early, debug arming issues     | High   | P0       |
| System Integrators  | Validate sensor integration before flight                 | High   | P0       |
| Safety Reviewers    | Ensure vehicle cannot arm in unsafe conditions            | High   | P0       |
| Test Engineers      | Ability to bypass checks for bench testing                | Medium | P1       |
| Autonomous Features | Validate all subsystems ready before autonomous operation | High   | P0       |

## Research & Discovery

### User Feedback

From operational safety requirements:

- Pre-arm checks are mandatory safety feature in all autopilot systems
- Clear, actionable failure messages are critical for field operations
- Ability to disable specific checks needed for hardware-in-loop testing
- Arming without validation caused multiple test failures (motors spinning unexpectedly)
- Need to prevent arming when RC connection lost or RC input invalid

### Competitive Analysis

**ArduPilot AP_Arming Framework**:

ArduPilot implements pre-arm checks in `libraries/AP_Arming/AP_Arming.cpp`:

```cpp
bool AP_Arming::pre_arm_checks(bool report)
{
    // Execute checks in priority order (fail-fast pattern)
    return hardware_safety_check(report)
        & barometer_checks(report)
        & ins_checks(report)              // IMU (gyro/accel)
        & compass_checks(report)
        & gps_checks(report)
        & battery_checks(report)
        & logging_checks(report)
        & rc_calibration_checks(report)
        & mission_checks(report)
        & rangefinder_checks(report)
        & servo_checks(report)
        & board_voltage_checks(report)
        & system_checks(report);
}
```

Key architectural patterns:

- **Sequential execution**: Checks run in priority order (critical first)
- **Fail-fast**: First failure stops execution, reports immediately
- **Conditional compilation**: Checks included/excluded based on feature flags (`#if AP_GPS_ENABLED`)
- **Severity levels**: "PreArm: " prefix for blocking failures, "Arm: " for warnings
- **Check bitmask**: `ARMING_CHECK` parameter (32-bit) enables/disables categories
- **Periodic reporting**: Failures reported every 4 seconds until resolved
- **State tracking**: `last_prearm_checks_result` detects state changes

**ArduPilot Check Categories** (ARMING_CHECK bitmask):

| Bit | Category    | Description                               |
| --- | ----------- | ----------------------------------------- |
| 0   | ALL         | Enable all checks (default)               |
| 1   | BAROMETER   | Barometric pressure sensor                |
| 2   | COMPASS     | Magnetic compass calibration and health   |
| 3   | GPS         | GPS lock quality and consistency          |
| 4   | INS         | Inertial sensors (gyro/accel) calibration |
| 5   | PARAMETERS  | Parameter validity and sanity             |
| 6   | RC          | RC calibration and connection             |
| 7   | VOLTAGE     | Battery and board voltage                 |
| 8   | BATTERY     | Battery capacity and health               |
| 9   | AIRSPEED    | Airspeed sensor (plane only)              |
| 10  | LOGGING     | SD card and logging availability          |
| 11  | MISSION     | Mission item validity                     |
| 12  | RANGEFINDER | Rangefinder health                        |
| 13  | SYSTEM      | System health (storage, scripting, etc.)  |
| 14  | CAMERA      | Camera/gimbal readiness                   |

**PX4 Preflight Checks**:

Similar architecture but different implementation:

- **Commander Module**: Centralized state machine manages arming
- **Health Flags**: Each subsystem reports health via uORB messages
- **Event Logging**: All check failures logged to flight log (ulog)
- **Safety Button**: Physical button required for arming (optional)
- **Calibration Status**: Persistent storage of calibration validity

### Technical Investigation

**Current Arming Implementation**:

File: `src/communication/mavlink/state.rs:160-171`

```rust
pub fn arm(&mut self) -> Result<(), &'static str> {
    if self.is_armed() {
        return Err("Already armed");  // Only check: duplicate state
    }

    // TODO: Add pre-arm checks here
    // - RC input available
    // - Sensors healthy
    // - Configuration valid

    self.armed = ArmedState::Armed;
    Ok(())
}
```

Current flow:

1. Check if already armed → Err("Already armed")
2. If not armed → Set armed state to Armed
3. Return Ok(())

**No validation of**:

- RC connection status
- Sensor initialization
- Actuator readiness
- Battery health
- Configuration validity

**Proposed Pre-Arm Check Framework**:

Architecture:

```rust
/// Pre-arm check trait - all checks implement this interface
pub trait PreArmCheck {
    /// Execute the check
    /// Returns Ok(()) if check passes, Err(reason) if check fails
    fn check(&self) -> Result<(), &'static str>;

    /// Get check name for logging/reporting
    fn name(&self) -> &'static str;

    /// Get check category (for ARMING_CHECK bitmask)
    fn category(&self) -> ArmingCheckCategory;

    /// Check priority (0 = highest, run first)
    fn priority(&self) -> u8 {
        10  // Default priority
    }
}

/// Check categories (matching ARMING_CHECK parameter bits)
#[derive(Clone, Copy, PartialEq)]
pub enum ArmingCheckCategory {
    All = 0,
    RC = 6,
    Voltage = 7,
    Battery = 8,
    System = 13,
    Actuators = 15,  // Custom category for pico_trail
}

/// Pre-arm checker - manages all registered checks
pub struct PreArmChecker {
    checks: Vec<Box<dyn PreArmCheck>>,
    arming_check_mask: u32,  // ARMING_CHECK parameter
}

impl PreArmChecker {
    /// Register a new check
    pub fn register(&mut self, check: Box<dyn PreArmCheck>) {
        self.checks.push(check);
        // Sort by priority after registration
        self.checks.sort_by_key(|c| c.priority());
    }

    /// Execute all enabled checks
    /// Returns Ok(()) if all pass, Err(reason) with first failure
    pub fn check_all(&self) -> Result<(), &'static str> {
        for check in &self.checks {
            // Skip if disabled via ARMING_CHECK parameter
            if !self.is_check_enabled(check.category()) {
                continue;
            }

            // Execute check (fail-fast)
            if let Err(reason) = check.check() {
                return Err(reason);
            }
        }
        Ok(())
    }

    /// Check if specific category enabled via ARMING_CHECK bitmask
    fn is_check_enabled(&self, category: ArmingCheckCategory) -> bool {
        if self.arming_check_mask == 0 {
            return false;  // All checks disabled
        }
        if category == ArmingCheckCategory::All {
            return true;   // Always enabled
        }
        (self.arming_check_mask & (1 << category as u32)) != 0
    }
}
```

**Initial Check Implementations**:

1. **RC Input Check**:

```rust
pub struct RcInputCheck {
    rc_state: &RcInput,
}

impl PreArmCheck for RcInputCheck {
    fn check(&self) -> Result<(), &'static str> {
        // Check RC connection (timeout detection)
        if !self.rc_state.is_connected() {
            return Err("RC not connected");
        }

        // Check RC calibration (neutral values valid)
        if !self.rc_state.is_calibrated() {
            return Err("RC not calibrated");
        }

        // Check failsafe not active
        if self.rc_state.is_failsafe() {
            return Err("RC failsafe active");
        }

        Ok(())
    }

    fn name(&self) -> &'static str {
        "RC Input"
    }

    fn category(&self) -> ArmingCheckCategory {
        ArmingCheckCategory::RC
    }

    fn priority(&self) -> u8 {
        5  // High priority (critical for manual control)
    }
}
```

2. **System State Check**:

```rust
pub struct SystemStateCheck {
    system_state: &SystemState,
}

impl PreArmCheck for SystemStateCheck {
    fn check(&self) -> Result<(), &'static str> {
        // Check valid flight mode selected
        if self.system_state.mode == FlightMode::Unknown {
            return Err("Invalid flight mode");
        }

        // Check no critical errors active
        if self.system_state.has_critical_error() {
            return Err("System error active");
        }

        Ok(())
    }

    fn name(&self) -> &'static str {
        "System State"
    }

    fn category(&self) -> ArmingCheckCategory {
        ArmingCheckCategory::System
    }

    fn priority(&self) -> u8 {
        8  // Medium priority
    }
}
```

3. **Actuator Readiness Check**:

```rust
pub struct ActuatorReadinessCheck {
    actuators: &Actuators,
}

impl PreArmCheck for ActuatorReadinessCheck {
    fn check(&self) -> Result<(), &'static str> {
        // Check PWM outputs initialized
        if !self.actuators.is_initialized() {
            return Err("Actuators not initialized");
        }

        // Check outputs are neutral (safety)
        if !self.actuators.are_outputs_neutral() {
            return Err("Actuators not neutral");
        }

        Ok(())
    }

    fn name(&self) -> &'static str {
        "Actuators"
    }

    fn category(&self) -> ArmingCheckCategory {
        ArmingCheckCategory::Actuators
    }

    fn priority(&self) -> u8 {
        3  // Very high priority (safety critical)
    }
}
```

**Integration with Arming Logic**:

Modified `SystemState::arm()`:

```rust
pub fn arm(&mut self, pre_arm_checker: &PreArmChecker) -> Result<(), &'static str> {
    // Check if already armed
    if self.is_armed() {
        return Err("Already armed");
    }

    // Execute all pre-arm checks
    if let Err(reason) = pre_arm_checker.check_all() {
        // Log failure
        error!("Pre-arm check failed: {}", reason);
        return Err(reason);
    }

    // All checks passed - arm vehicle
    self.armed = ArmedState::Armed;
    info!("Vehicle armed");
    Ok(())
}
```

**Failure Reporting to GCS**:

ArduPilot sends STATUSTEXT messages (MAVLink message ID 253) for pre-arm failures:

```rust
/// Send pre-arm failure to GCS
fn report_prearm_failure(failure_reason: &str, router: &mut MavlinkRouter) {
    let msg = mavlink::common::STATUSTEXT_DATA {
        severity: mavlink::common::MavSeverity::MAV_SEVERITY_CRITICAL,
        text: format!("PreArm: {}", failure_reason),
    };
    router.send_message(MavMessage::STATUSTEXT(msg));
}
```

Mission Planner displays these messages in red on the HUD when arming is attempted.

**Memory Analysis**:

| Component               | RAM Usage    | Notes                                      |
| ----------------------- | ------------ | ------------------------------------------ |
| PreArmChecker           | \~200 B      | Check registry + state                     |
| Check implementations   | \~50 B each  | 5 initial checks = 250 B                   |
| ARMING_CHECK parameter  | 4 B          | u32 bitmask                                |
| Check state tracking    | \~100 B      | Last result, failure count                 |
| **Total (initial)**     | **\~550 B**  | Minimal overhead for safety                |
| **Total (full checks)** | **\~1.5 KB** | With all 15 check types (future expansion) |

**ArduPilot Parameters**:

ArduPilot defines multiple parameters for controlling pre-arm check behavior:

1. **ARMING_CHECK** (u32 bitmask, default: 1 = All enabled):

| Bit | Value   | Check Type  | Description                                    |
| --- | ------- | ----------- | ---------------------------------------------- |
| 0   | 1       | ALL         | Enable all checks (default)                    |
| 1   | 2       | BARO        | Barometer health and calibration               |
| 2   | 4       | COMPASS     | Magnetometer calibration and interference      |
| 3   | 8       | GPS         | GPS lock quality and consistency               |
| 4   | 16      | INS         | Inertial sensors (gyro/accel) calibration      |
| 5   | 32      | PARAMETERS  | Parameter validity (deprecated/unused)         |
| 6   | 64      | RC          | RC connection and calibration                  |
| 7   | 128     | VOLTAGE     | Board voltage within limits                    |
| 8   | 256     | BATTERY     | Battery level and health                       |
| 9   | 512     | AIRSPEED    | Airspeed sensor (Plane only)                   |
| 10  | 1024    | LOGGING     | SD card available for logging                  |
| 11  | 2048    | SWITCH      | Hardware safety switch engaged                 |
| 12  | 4096    | GPS_CONFIG  | GPS configuration valid                        |
| 13  | 8192    | SYSTEM      | System health (storage, scripting, etc.)       |
| 14  | 16384   | MISSION     | Mission items valid                            |
| 15  | 32768   | RANGEFINDER | Rangefinder health                             |
| 16  | 65536   | CAMERA      | Camera system ready                            |
| 17  | 131072  | AUX_AUTH    | Auxiliary authorization (external safety auth) |
| 18  | 262144  | VISION      | Vision system health (visual odometry)         |
| 19  | 524288  | FFT         | FFT analysis health (vibration monitoring)     |
| 20  | 1048576 | OSD         | On-screen display configuration                |

Examples:

- `ARMING_CHECK = 0`: All checks disabled (UNSAFE - bench testing only)
- `ARMING_CHECK = 1`: All checks enabled (recommended)
- `ARMING_CHECK = 64`: Only RC check (useful for RC calibration testing)
- `ARMING_CHECK = 72`: RC + GPS checks only (64 + 8)
- `ARMING_CHECK = -1`: All checks enabled (alternative to 1)

2. **ARMING_OPTIONS** (u32 bitmask, default: 0):

| Bit | Value | Option                       | Description                                     |
| --- | ----- | ---------------------------- | ----------------------------------------------- |
| 0   | 1     | Suppress prearm display      | Don't send prearm failure messages every 30s    |
| 1   | 2     | Skip arming text messages    | Don't send "Armed"/"Disarmed" status text       |
| 2   | 4     | Skip IMU check when ICE runs | Allow arming with inconsistent IMU if engine on |

3. **ARMING_REQUIRE** (float, default: 1 = YES_MIN_PWM):

| Value | Meaning      | Description                                 |
| ----- | ------------ | ------------------------------------------- |
| 0     | Disabled     | Arming not required (motors always enabled) |
| 1     | YES_MIN_PWM  | Arming required, min PWM when disarmed      |
| 2     | YES_ZERO_PWM | Arming required, zero PWM when disarmed     |

4. **ARMING_ACCTHRESH** (float, default: 0.75, range: 0.25-3.0 m/s²):
   - Accelerometer error threshold for INS consistency check
   - Higher values = more tolerant to accelerometer variance

5. **ARMING_MAGTHRESH** (int, default: 100, range: 0-500 mGauss):
   - Compass magnetic field strength error threshold vs Earth model
   - Higher values = more tolerant to magnetic interference

6. **ARMING_RUDDER** (u8, default: 2):

| Value | Meaning     | Description                              |
| ----- | ----------- | ---------------------------------------- |
| 0     | Disabled    | No rudder arming/disarming               |
| 1     | ArmingOnly  | Rudder-stick can arm only                |
| 2     | ArmOrDisarm | Rudder-stick can arm or disarm (default) |

7. **ARMING_MIS_ITEMS** (bitmask, default: 0):
   - Required mission items before arming (Auto mode)
   - Bits: Land, VTOL_Land, DO_LAND_START, Takeoff, VTOL_Takeoff, Rallypoint, RTL

8. **ARMING_CRSDP_IGN** (u8, default: 0):
   - Ignore crash dump check (allow arming with unacknowledged crash dump)
   - 0 = Crash dump blocks arming, 1 = Ignore crash dump

9. **ARMING_NEED_LOC** (float, default: varies by vehicle):
   - Require vehicle location (GPS lock or other position source) before arming
   - Vehicle-specific defaults (Copter requires, Rover may not)

**Force Arm via MAVLink**:

ArduPilot provides an alternative mechanism to bypass pre-arm checks without disabling ARMING_CHECK parameter:

- **MAV_CMD_COMPONENT_ARM_DISARM** with param2 = 21196 (magic number)
  - param1 = 1.0 (arm request) + param2 = 21196.0 → Force arm (bypass all pre-arm checks)
  - param1 = 0.0 (disarm request) + param2 = 21196.0 → Force disarm (bypass all pre-disarm checks)
  - Purpose: Temporary bypass for testing/emergency without permanently disabling ARMING_CHECK
  - Use case: Bench testing, SITL environment, emergency recovery
  - Safety: Logged as warning with audit trail, operator explicitly acknowledges risk

Advantages of force-arm over ARMING_CHECK=0:

- One-time bypass (doesn't persist across reboots)
- Explicit operator action required each time
- Clear audit trail (force arm logged separately)
- Doesn't mask check configuration issues (checks remain enabled for normal arming)

Usage example (MAVProxy):

```
arm throttle force
```

This is safer than setting ARMING_CHECK=0 because it requires explicit confirmation for each arming attempt and doesn't hide underlying check configuration problems.

### Data Analysis

**Check Execution Performance**:

- **Target latency**: < 10ms for all checks (operator expects immediate feedback)
- **Per-check budget**: < 1ms each (simple boolean/threshold checks)
- **Retry rate**: Operator retries arming every 2-5 seconds until successful
- **Reporting frequency**: Failed checks reported at 4-second intervals (avoid spam)

**Failure Rates** (estimated from ArduPilot community feedback):

- **RC calibration**: 30% of new users encounter this on first arming attempt
- **Compass calibration**: 20% fail compass checks (magnetic interference)
- **GPS lock**: 15% fail GPS checks (indoor testing, poor sky view)
- **Battery voltage**: 10% fail voltage checks (incorrect parameter settings)
- **Configuration errors**: 5% fail parameter/configuration checks

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The system shall execute pre-arm safety checks before allowing transition to ARMED state → Will become FR-<id>
  - Rationale: Prevent unsafe arming when vehicle is not flight-ready (core safety requirement)
  - Acceptance Criteria:
    - Pre-arm checks execute before arming when MAV_CMD_COMPONENT_ARM_DISARM(1.0) received
    - If any check fails, vehicle remains disarmed
    - If all checks pass, vehicle transitions to ARMED state
    - Check execution latency < 10ms total
    - Check results logged to storage

- [ ] **FR-DRAFT-2**: The system shall report specific pre-arm check failures to ground control station → Will become FR-<id>
  - Rationale: Operator needs actionable feedback to resolve arming failures
  - Acceptance Criteria:
    - Send STATUSTEXT message with failure reason when check fails
    - Message format: "PreArm: \<specific failure reason>"
    - Severity level: CRITICAL for blocking failures
    - Command ACK includes failure reason in result_param2
    - Failures logged locally with timestamp

- [ ] **FR-DRAFT-3**: The system shall validate RC input connection and calibration before arming → Will become FR-<id>
  - Rationale: Prevent arming without operator control (critical for Manual mode)
  - Acceptance Criteria:
    - Check RC connection: RC_CHANNELS message received within 1 second
    - Check RC calibration: Neutral values (channels 1,3) within 1400-1600 μs range
    - Check RC failsafe not active (rssi != 255 or equivalent flag)
    - Failure messages: "RC not connected", "RC not calibrated", "RC failsafe active"

- [ ] **FR-DRAFT-4**: The system shall validate actuator readiness before arming → Will become FR-<id>
  - Rationale: Ensure motors/servos safe to activate (prevent unexpected motion)
  - Acceptance Criteria:
    - Check PWM outputs initialized (not null/uninitialized)
    - Check outputs at neutral position (steering centered, throttle zero)
    - Check no actuator errors active (hardware fault, calibration failure)
    - Failure messages: "Actuators not initialized", "Actuators not neutral"

- [ ] **FR-DRAFT-5**: The system shall provide configurable pre-arm check enable/disable via ARMING_CHECK parameter → Will become FR-<id>
  - Rationale: Allow selective disabling for bench testing and hardware-in-loop simulation
  - Acceptance Criteria:
    - ARMING_CHECK parameter (u32 bitmask) controls which check categories run
    - Value 0: All checks disabled (bench testing only)
    - Value 1: All checks enabled (default, recommended for flight)
    - Individual bits disable specific categories (bit 6 = RC, bit 13 = System, etc.)
    - Parameter change takes effect immediately (no reboot required)

- [ ] **FR-DRAFT-6**: The system shall implement extensible pre-arm check framework supporting future checks → Will become FR-<id>
  - Rationale: Enable adding sensor checks (IMU, GPS, battery) without modifying core arming logic
  - Acceptance Criteria:
    - PreArmCheck trait defines standard check interface
    - PreArmChecker registry manages all checks
    - Checks executed in priority order (critical first)
    - New checks added via register() method (no core code changes)
    - Checks can be unit tested in isolation

- [ ] **FR-DRAFT-7**: The system shall support force-arm override via MAVLink command parameter to bypass pre-arm checks → Will become FR-<id>
  - Rationale: Enable temporary bypass of pre-arm checks for bench testing, SITL, or emergency recovery without permanently disabling ARMING_CHECK
  - Acceptance Criteria:
    - MAV_CMD_COMPONENT_ARM_DISARM with param1=1.0 and param2=21196.0 bypasses all pre-arm checks
    - Force arm attempt logged with WARNING severity and audit trail
    - STATUSTEXT message sent to GCS: "Armed (FORCED)"
    - Normal arming (param2=0.0) still executes pre-arm checks
    - Force arm succeeds even when pre-arm checks would fail
    - Force arm only bypasses checks, still validates already-armed state
    - ARMING_CHECK parameter remains unchanged (not disabled by force-arm)

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Pre-arm check execution shall not exceed 10ms total latency → Will become NFR-<id>
  - Category: Performance
  - Rationale: Operator expects immediate arming response (< 100ms including command processing)
  - Target: < 10ms for all checks combined, < 1ms per individual check

- [ ] **NFR-DRAFT-2**: Pre-arm check failures shall be reported to GCS within 100ms of arming attempt → Will become NFR-<id>
  - Category: Usability
  - Rationale: Immediate feedback critical for operator situation awareness
  - Target: STATUSTEXT message sent within 100ms of arm command rejection

- [ ] **NFR-DRAFT-3**: Pre-arm check framework shall consume no more than 2 KB RAM → Will become NFR-<id>
  - Category: Resource Constraints
  - Rationale: Maintain memory budget for other subsystems on RP2040/RP2350
  - Target: < 2 KB for check framework + all registered checks (measured via runtime profiling)

- [ ] **NFR-DRAFT-4**: Pre-arm check failures shall be logged to persistent storage for post-flight analysis → Will become NFR-<id>
  - Category: Reliability / Auditability
  - Rationale: Support debugging and safety investigations
  - Target: All arming attempts (success/failure) logged with timestamp, check name, result

## Design Considerations

### Technical Constraints

- **Existing architecture**: Must integrate with current SystemState, MAVLink handlers, and parameter system
- **Memory budget**: Limited RAM on RP2040/RP2350 (264 KB) - checks must be lightweight
- **Real-time constraints**: Arming response must be immediate (< 100ms total)
- **Safety critical**: False negatives acceptable (reject valid arm), false positives UNACCEPTABLE (allow unsafe arm)
- **Platform abstraction**: Checks must work on both RP2040 and RP2350
- **No dynamic allocation**: Check framework must use static/stack allocation only

### Potential Approaches

1. **Option A: Inline Check Functions (No Framework)**
   - Pros:
     - Simplest implementation (no abstraction)
     - Zero overhead (direct function calls)
     - Easy to understand and debug
     - Minimal memory footprint (\~100 B)
   - Cons:
     - Hard to extend (adding checks requires modifying arm() method)
     - No check prioritization
     - Difficult to selectively disable checks
     - Code duplication across check implementations
     - No unit testing of individual checks
   - Effort: Low (1-2 hours)

2. **Option B: Trait-Based Check Framework** ⭐ Recommended
   - Pros:
     - Extensible: New checks added via register() without core changes
     - Testable: Each check isolated, can unit test independently
     - Prioritizable: Checks run in priority order (critical first)
     - Configurable: ARMING_CHECK bitmask enables/disables categories
     - Maintainable: Clear separation of concerns
     - Matches ArduPilot architecture (proven, familiar to community)
   - Cons:
     - More upfront design effort
     - Slightly higher memory (\~1-2 KB for framework)
     - Trait object overhead (mitigated with static dispatch)
   - Effort: Medium (4-8 hours)

3. **Option C: Full ArduPilot-Style Framework with Periodic Reporting**
   - Pros:
     - Maximum feature parity with ArduPilot
     - Sophisticated failure reporting (4-second periodic updates)
     - State tracking for check transitions
     - Advanced diagnostics and logging
   - Cons:
     - High complexity (check state machine, timing, periodic tasks)
     - Higher memory overhead (\~3-5 KB)
     - Overkill for initial implementation
     - Long development time
   - Effort: High (16-24 hours)

**Recommendation**: Option B (Trait-Based Framework) provides best balance of extensibility, testability, and simplicity for initial implementation. Option C features (periodic reporting, state tracking) can be added incrementally if needed.

### Check Implementation Strategy

**Phase 1: Minimal Viable Checks** (Initial Implementation)

- RC Input Check (connection, calibration, failsafe)
- System State Check (valid mode, no errors)
- Actuator Readiness Check (initialized, neutral position)

**Phase 2: Sensor Health Checks** (Future Expansion)

- IMU Check (gyro/accel initialized, calibration valid)
- GPS Check (fix quality, HDOP threshold) - if GPS module added
- Compass Check (calibration valid, interference check) - if compass added

**Phase 3: Advanced Validation** (Future Expansion)

- Battery Check (voltage threshold, capacity remaining)
- Parameter Sanity Check (critical parameters in valid ranges)
- Mission Check (valid waypoints if in Auto mode)
- Logging Check (SD card available, space remaining)

### Architecture Impact

**New ADRs required**:

- **ADR-<id> Pre-Arm Check Framework Architecture**: Trait design, check registry, ARMING_CHECK parameter
- **ADR-<id> Pre-Arm Check Categories**: Which checks to implement, priority order, failure messages

**New modules**:

- `src/core/arming/` - Pre-arm check framework
  - `src/core/arming/check.rs` - PreArmCheck trait, ArmingCheckCategory enum
  - `src/core/arming/checker.rs` - PreArmChecker registry and execution logic
  - `src/core/arming/checks/` - Individual check implementations
    - `src/core/arming/checks/rc_input.rs` - RC connection and calibration check
    - `src/core/arming/checks/system_state.rs` - System health and mode check
    - `src/core/arming/checks/actuators.rs` - Actuator readiness check

**Modified modules**:

- `src/communication/mavlink/state.rs` - Modify SystemState::arm() to call pre-arm checks
- `src/communication/mavlink/handlers/command.rs` - Report check failures via STATUSTEXT
- `src/core/parameters/` - Add ARMING_CHECK parameter (u32, default 1 = all enabled)

**Parameters**:

- **ARMING_CHECK** (u32, default 1):
  - 0 = All checks disabled (bench testing only)
  - 1 = All checks enabled (recommended)
  - Bit 6 = RC checks
  - Bit 13 = System checks
  - Bit 15 = Actuator checks

## Risk Assessment

| Risk                                                       | Probability | Impact       | Mitigation Strategy                                                                                 |
| ---------------------------------------------------------- | ----------- | ------------ | --------------------------------------------------------------------------------------------------- |
| **False positive check (allows unsafe arm)**               | **Low**     | **CRITICAL** | **Extensive testing, fail-safe defaults, conservative thresholds, mandatory code review**           |
| Excessive false negatives (rejects valid arm)              | Medium      | Medium       | Tunable thresholds, operator can disable specific checks via ARMING_CHECK                           |
| Check execution timeout degrades arming response           | Low         | Low          | Per-check timeout budget (1ms), total latency monitoring                                            |
| ARMING_CHECK parameter misused (all checks disabled)       | Medium      | High         | Log warning when checks disabled, require parameter file edit (not GCS parameter set)               |
| Check framework memory overhead exceeds budget             | Low         | Medium       | Profile early, use static dispatch, limit initial check count to 5                                  |
| Integration breaks existing arming workflow                | Low         | Medium       | Comprehensive integration tests, gradual rollout (checks disabled by default initially)             |
| Operator confusion from unclear failure messages           | Medium      | Medium       | User testing, clear actionable messages ("RC not calibrated" vs "Check 6 failed")                   |
| Check state becomes stale (e.g., RC disconnects after arm) | Low         | High         | Checks only validate at arming time, post-arm monitoring separate (failsafe system handles runtime) |

## Open Questions

- [ ] Should ARMING_CHECK default to 1 (all enabled) or 0 (all disabled) for initial rollout? → Next step: User testing to validate check reliability before enabling by default
- [ ] Do we need periodic re-checking after arm (continuous validation)? → Decision: No, out of scope - post-arm monitoring handled by failsafe system
- [ ] Should RC calibration check use fixed thresholds (1400-1600 μs) or configurable parameters? → Method: Start with fixed, add RCx_MIN/MAX/TRIM parameters if needed
- [ ] Do we report all check failures or only the first failure? → Decision: First failure only (fail-fast), matches ArduPilot behavior
- [ ] Should we implement check state tracking (last result, transition detection)? → Decision: Not in initial version, add if periodic reporting needed later
- [ ] How to handle check failures during autonomous operation (mode requires arming)? → Method: Prevent mode changes requiring arming if checks would fail

## Recommendations

### Immediate Actions

1. **Adopt Option B architecture**: Trait-based pre-arm check framework with ARMING_CHECK parameter
2. **Implement Phase 1 checks only**: RC input, system state, actuator readiness (minimal viable safety)
3. **Default ARMING_CHECK to 1**: All checks enabled (conservative, safe default)
4. **Follow ArduPilot patterns**: Check categories, failure message format, bitmask parameter
5. **Implement force-arm support**: MAVLink param2=21196 for temporary check bypass (testing/emergency)

### Next Steps

1. [ ] Create formal requirements: FR-<id> (check framework), FR-<id> (RC check), FR-<id> (system check), FR-<id> (actuator check), FR-<id> (ARMING_CHECK param), FR-<id> (force-arm override), NFR-<id> (latency), NFR-<id> (reporting), NFR-<id> (memory), NFR-<id> (logging)
2. [ ] Draft ADR for: Pre-arm check framework architecture (trait design, registry pattern, execution flow)
3. [ ] Draft ADR for: Pre-arm check categories (which checks, priority order, failure messages)
4. [ ] Create task for: Implementation (check framework + initial 3 checks)
5. [ ] Plan integration testing: Verify check failures prevent arming, success allows arming

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **Periodic check reporting**: No 4-second re-reporting of failures (ArduPilot feature)
- **Check state tracking**: No detection of check state transitions (last result comparison)
- **Advanced sensor checks**: No IMU, GPS, compass, battery checks (Phase 2/3 features)
- **Mission validation**: No waypoint or mission item checks (no mission planning yet)
- **External safety switch**: No physical safety button/switch requirement (optional future feature)
- **Visual/audible alerts**: No LED patterns or buzzer feedback for check failures (GCS messages only)
- **Auto-retry mechanism**: No automatic re-checking on failure (operator manual retry)
- **Check dependency graph**: No inter-check dependencies (e.g., GPS requires IMU)
- **Runtime continuous validation**: Checks only at arming time, not during flight (failsafe handles post-arm)

## Appendix

### References

- ArduPilot AP_Arming: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Arming/AP_Arming.cpp>
- ArduPilot Pre-Arm Checks Documentation: <https://ardupilot.org/copter/docs/common-prearm-safety-checks.html>
- ArduPilot ARMING_CHECK Parameter: <https://ardupilot.org/copter/docs/parameters.html#arming-check>
- MAVLink STATUSTEXT: <https://mavlink.io/en/messages/common.html#STATUSTEXT>
- MAVLink COMMAND_ACK: <https://mavlink.io/en/messages/common.html#COMMAND_ACK>
- PX4 Commander Module: <https://docs.px4.io/main/en/concept/architecture.html>

### Raw Data

**ArduPilot Check Execution Flow**:

```cpp
// From AP_Arming.cpp
bool AP_Arming::pre_arm_checks(bool report)
{
    // All checks use bitwise AND (&) not logical AND (&&)
    // This ensures ALL checks run even if early ones fail (collect all failures)
    return hardware_safety_check(report)
        & barometer_checks(report)
        & ins_checks(report)
        & compass_checks(report)
        & gps_checks(report)
        & battery_checks(report)
        & logging_checks(report)
        & rc_calibration_checks(report)
        & mission_checks(report)
        & rangefinder_checks(report)
        & servo_checks(report)
        & board_voltage_checks(report)
        & system_checks(report);
}

// Individual check example
bool AP_Arming::rc_calibration_checks(bool report)
{
    // Get RC input instance
    const RC_Channel *c = rc().channel(rcmap.throttle()-1);

    // Check throttle at neutral
    if (c->get_radio_in() > 1300) {
        check_failed(report, "Throttle not down");
        return false;
    }

    // Check calibration ranges valid
    if (c->get_radio_min() >= 1300) {
        check_failed(report, "RC min too high");
        return false;
    }

    return true;
}
```

**Proposed Check Framework Pseudocode**:

```rust
// Check registration (at system initialization)
fn setup_prearm_checks() -> PreArmChecker {
    let mut checker = PreArmChecker::new();

    // Register checks in any order (framework sorts by priority)
    checker.register(Box::new(RcInputCheck::new()));
    checker.register(Box::new(SystemStateCheck::new()));
    checker.register(Box::new(ActuatorReadinessCheck::new()));

    // Load ARMING_CHECK parameter from storage
    checker.set_check_mask(params.get_u32("ARMING_CHECK").unwrap_or(1));

    checker
}

// Modified arming flow with force-arm support
fn handle_arm_command(cmd: &COMMAND_LONG_DATA) -> MavResult {
    if cmd.param1 == 1.0 {
        // Arm request
        let force_arm = cmd.param2 == 21196.0;  // ArduPilot magic number for force arm

        if force_arm {
            // Force arm bypasses all pre-arm checks (emergency/testing only)
            log::warn!("FORCE ARM requested - bypassing all pre-arm checks");
            match system_state.arm_forced() {
                Ok(()) => {
                    send_statustext("Armed (FORCED)", MAV_SEVERITY_WARNING);
                    MavResult::MAV_RESULT_ACCEPTED
                }
                Err(reason) => {
                    send_statustext(&format!("Force arm failed: {}", reason), MAV_SEVERITY_CRITICAL);
                    MavResult::MAV_RESULT_FAILED
                }
            }
        } else {
            // Normal arm with pre-arm checks
            match system_state.arm(&prearm_checker) {
                Ok(()) => {
                    // Arming successful
                    send_statustext("Armed", MAV_SEVERITY_INFO);
                    MavResult::MAV_RESULT_ACCEPTED
                }
                Err(reason) => {
                    // Pre-arm check failed
                    send_statustext(&format!("PreArm: {}", reason), MAV_SEVERITY_CRITICAL);
                    MavResult::MAV_RESULT_FAILED
                }
            }
        }
    }
}
```

**Example Check Failure Output** (Mission Planner HUD):

```
PreArm: RC not connected
PreArm: RC not calibrated
PreArm: Actuators not neutral
PreArm: Invalid flight mode
PreArm: System error active
```

**ARMING_CHECK Bitmask Examples**:

```
ARMING_CHECK = 0    # All checks disabled (UNSAFE - bench testing only)
ARMING_CHECK = 1    # All checks enabled (RECOMMENDED - safe default)
ARMING_CHECK = 64   # Only RC check enabled (bit 6)
ARMING_CHECK = 8256 # Only System (bit 13) + Actuator (bit 15) checks
ARMING_CHECK = -1   # All checks enabled (all bits set)
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
