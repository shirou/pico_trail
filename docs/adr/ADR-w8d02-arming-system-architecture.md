# ADR-w8d02 Arming System Architecture

## Metadata

- Type: ADR
- Status: Approved

## Links

- Impacted Requirements:
  - [FR-n1mte-prearm-capability-enforcement](../requirements/FR-n1mte-prearm-capability-enforcement.md)
  - [FR-b22fh-armed-state-reset](../requirements/FR-b22fh-armed-state-reset.md)
  - [FR-c6ej0-arm-subsystem-notification](../requirements/FR-c6ej0-arm-subsystem-notification.md)
  - [FR-qyrn3-system-health-status-tracking](../requirements/FR-qyrn3-system-health-status-tracking.md)
  - [FR-jvydv-disarm-armed-state-check](../requirements/FR-jvydv-disarm-armed-state-check.md)
  - [FR-jynu9-forced-disarm-override](../requirements/FR-jynu9-forced-disarm-override.md)
  - [FR-exazo-force-arm-override](../requirements/FR-exazo-force-arm-override.md)
  - [NFR-9s8i3-post-arm-safety](../requirements/NFR-9s8i3-post-arm-safety.md)
  - [NFR-z9g39-high-frequency-monitor-detection-time](../requirements/NFR-z9g39-high-frequency-monitor-detection-time.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-zmv9u-arming-system-implementation](../tasks/T-zmv9u-arming-system-implementation/README.md)

## Context

This ADR addresses findings from five upstream analyses: [AN-r2fps-pre-arm-checks](../analysis/AN-r2fps-pre-arm-checks.md), [AN-m4dpl-post-arm-initialization](../analysis/AN-m4dpl-post-arm-initialization.md), [AN-dgpck-armed-state-monitoring](../analysis/AN-dgpck-armed-state-monitoring.md), [AN-dqzc6-pre-disarm-validation](../analysis/AN-dqzc6-pre-disarm-validation.md), and [AN-081u0-post-disarm-cleanup](../analysis/AN-081u0-post-disarm-cleanup.md).

Currently, pico_trail implements basic arming/disarming via MAVLink COMPONENT_ARM_DISARM command (`src/communication/mavlink/handlers/command.rs:84-113`) with minimal validation. The system only checks if already armed/disarmed but performs no safety validation before state transitions. This creates critical safety gaps:

**Pre-Arm Issues:**

- Can arm without RC connection or valid RC calibration
- Can arm with failed sensors (IMU, compass, GPS)
- No configuration sanity checks (PWM outputs, parameters)
- No battery health or voltage validation
- No actuator readiness verification

**Post-Arm Issues:**

- No initialization sequence after arming (timestamps, logging, subsystem sync)
- No continuous monitoring of system health during armed operation
- RC signal age, battery voltage, sensor status not tracked after arming

**Pre-Disarm Issues:**

- Can disarm with motors running at high power
- Can disarm while moving at unsafe speed
- No method-specific validation (GCS vs RC vs failsafe)
- No emergency override for forced disarm

**Post-Disarm Issues:**

- No cleanup sequence after disarming
- Actuators not verified to return to neutral state
- No subsystem shutdown or state reset

ArduPilot implements comprehensive arming safety through the AP_Arming library with 30+ pre-arm checks, post-arm initialization, multi-rate monitoring (50-400Hz), and structured disarm validation. We need a similar safety framework adapted to pico_trail's resource constraints and embedded environment.

**Constraints:**

- Memory: Arming system must fit within 5 KB RAM budget
- CPU: Pre-arm checks must complete within 500ms, monitoring overhead < 10%
- Platform: Must work on RP2040 (Pico W, no FPU) and RP2350 (Pico 2 W, with FPU)
- Compatibility: Must support standard ArduPilot parameters (ARMING_CHECK, etc.)

**Forces in tension:**

- Comprehensive validation vs fast arming response time
- Safety enforcement vs operator flexibility (bench testing, troubleshooting)
- Continuous monitoring overhead vs control loop performance
- Extensibility (adding new checks) vs code complexity

## Success Metrics

- **Safety**: Zero instances of arming with failed critical systems (RC, battery, sensors) in testing
- **Latency**: Pre-arm checks complete < 500ms, monitoring detection time < 200ms
- **Memory**: Arming system RAM usage < 5 KB total
- **CPU**: Monitoring overhead < 10% measured on RP2040 @ 133 MHz
- **Extensibility**: Adding new pre-arm check requires < 50 lines of code
- **Compatibility**: Works with QGroundControl and Mission Planner without GCS changes

## Decision

We will implement a trait-based arming system architecture with five integrated subsystems: Pre-Arm Checks, Post-Arm Initialization, Armed State Monitoring, Pre-Disarm Validation, and Post-Disarm Cleanup.

### Core Architecture

**1. Pre-Arm Check Framework**

Implement trait-based check system for validation before arming:

```rust
// src/core/arming/checks.rs
pub trait PreArmCheck {
    fn check(&self, context: &SystemContext) -> Result<(), ArmingError>;
    fn name(&self) -> &'static str;
    fn category(&self) -> CheckCategory; // RC, Sensors, Configuration, etc.
}

pub struct ArmingChecker {
    checks: Vec<Box<dyn PreArmCheck>>,
    enabled_categories: CheckCategory, // Configurable via ARMING_CHECK parameter
}

impl ArmingChecker {
    pub fn run_checks(&self, context: &SystemContext) -> ArmingResult {
        for check in &self.checks {
            if self.enabled_categories.contains(check.category()) {
                if let Err(e) = check.check(context) {
                    return ArmingResult::Denied {
                        reason: format!("{}: {}", check.name(), e),
                        category: check.category(),
                    };
                }
            }
        }
        ArmingResult::Allowed
    }
}
```

**Check registration at initialization:**

```rust
// src/core/arming/mod.rs
pub fn initialize_arming_system() -> ArmingChecker {
    let mut checker = ArmingChecker::new();

    // Register checks in priority order (critical first)
    checker.register(RcInputCheck::new());      // RC signal present and valid
    checker.register(BatteryVoltageCheck::new()); // Battery above minimum
    checker.register(SystemStateCheck::new());    // No active errors
    checker.register(ImuHealthCheck::new());      // IMU calibrated and healthy
    checker.register(ActuatorReadyCheck::new());  // Actuators initialized

    checker
}
```

**2. Post-Arm Initialization Sequence**

Establish operational baseline after successful arming:

```rust
// src/core/arming/initialization.rs
pub struct PostArmInitializer;

impl PostArmInitializer {
    pub fn execute(&self, context: &mut SystemContext) -> Result<(), ArmingError> {
        // 1. Record arm timestamp (for duration tracking, timeouts)
        context.state.set_arm_time(embassy_time::Instant::now());

        // 2. Log arm event (audit trail)
        log::info!("Vehicle armed via {:?}", context.arm_method);

        // 3. Initialize actuators (transition from neutral to ready)
        context.actuators.enter_armed_state()?;

        // 4. Notify subsystems of arm event
        context.monitoring.on_armed();
        context.failsafe.on_armed();
        context.mode_manager.on_armed();

        // 5. Enable safety boundaries if configured
        if context.params.get_bool("FENCE_AUTOENABLE") {
            context.fence.enable();
        }

        // 6. Warn if checks were disabled
        if !context.arming_check.all_enabled() {
            log::warn!("Armed with some checks disabled: {:?}",
                       context.arming_check.disabled_categories());
        }

        Ok(())
    }
}
```

**3. Armed State Monitoring (Multi-Rate)**

Continuous health monitoring during armed operation:

```rust
// src/core/arming/monitoring.rs
pub struct ArmedStateMonitor {
    // High-frequency state (updated at 50-400 Hz)
    rc_last_received: Option<Instant>,
    sensor_health: SensorHealthFlags,

    // Medium-frequency state (updated at 10 Hz)
    battery_voltage: f32,
    ekf_status: EkfStatus,

    // Low-frequency state (updated at 1 Hz)
    fence_status: FenceStatus,
}

impl ArmedStateMonitor {
    // Called from high-frequency task (400 Hz)
    pub fn update_fast(&mut self, context: &SystemContext) {
        // Track RC signal freshness
        if context.rc.has_new_data() {
            self.rc_last_received = Some(Instant::now());
        }

        // Check for RC timeout (critical - triggers failsafe)
        if let Some(last_rc) = self.rc_last_received {
            if last_rc.elapsed() > Duration::from_secs(1) {
                context.failsafe.trigger(FailsafeReason::RcLoss);
            }
        }

        // Update sensor health flags
        self.sensor_health.update(context.sensors);
    }

    // Called from medium-frequency task (10 Hz)
    pub fn update_medium(&mut self, context: &SystemContext) {
        // Monitor battery voltage
        self.battery_voltage = context.battery.voltage();
        if self.battery_voltage < context.params.get_float("BATT_CRT_VOLT") {
            context.failsafe.trigger(FailsafeReason::BatteryCritical);
        }

        // Validate EKF health
        self.ekf_status = context.ahrs.ekf_status();
        if !self.ekf_status.is_healthy() {
            log::warn!("EKF unhealthy: {:?}", self.ekf_status);
        }
    }

    // Called from low-frequency task (1 Hz)
    pub fn update_slow(&mut self, context: &SystemContext) {
        // Check geofence violations
        if context.fence.is_enabled() {
            self.fence_status = context.fence.check_boundaries();
            if self.fence_status.violated() {
                context.failsafe.trigger(FailsafeReason::FenceViolation);
            }
        }

        // Send health status to GCS
        context.mavlink.send_sys_status(
            self.sensor_health,
            self.battery_voltage,
            self.ekf_status,
        );
    }
}
```

**4. Pre-Disarm Validation**

Safety checks before allowing disarm:

```rust
// src/core/arming/disarm.rs
pub struct DisarmValidator;

impl DisarmValidator {
    pub fn validate(
        &self,
        context: &SystemContext,
        method: DisarmMethod,
        forced: bool,
    ) -> Result<(), DisarmError> {
        // Forced disarm bypasses all validation (emergency only)
        if forced {
            log::warn!("FORCED DISARM - bypassing validation");
            return Ok(());
        }

        // Check if currently armed
        if !context.state.is_armed() {
            return Err(DisarmError::NotArmed);
        }

        // Method-specific validation
        match method {
            DisarmMethod::RcStick => {
                // RC disarm requires stricter checks
                self.validate_throttle_low(context)?;
                self.validate_velocity_safe(context)?;
            }
            DisarmMethod::GcsCommand => {
                // GCS command has more lenient checks
                self.validate_velocity_safe(context)?;
            }
            DisarmMethod::Failsafe => {
                // Failsafe disarm has no additional checks
            }
        }

        Ok(())
    }

    fn validate_throttle_low(&self, context: &SystemContext) -> Result<(), DisarmError> {
        let throttle = context.rc.get_channel(3).normalized_value();
        if throttle.abs() > 0.1 {
            return Err(DisarmError::ThrottleNotLow {
                current: throttle
            });
        }
        Ok(())
    }

    fn validate_velocity_safe(&self, context: &SystemContext) -> Result<(), DisarmError> {
        let speed = context.ahrs.ground_speed();
        const MAX_DISARM_SPEED: f32 = 0.5; // m/s
        if speed > MAX_DISARM_SPEED {
            return Err(DisarmError::VelocityTooHigh {
                current: speed,
                max: MAX_DISARM_SPEED
            });
        }
        Ok(())
    }
}
```

**5. Post-Disarm Cleanup**

Shutdown sequence after disarming:

```rust
// src/core/arming/cleanup.rs
pub struct PostDisarmCleanup;

impl PostDisarmCleanup {
    pub fn execute(&self, context: &mut SystemContext, reason: DisarmReason) -> Result<(), ArmingError> {
        // 1. Log disarm event (audit trail)
        log::info!("Vehicle disarmed: method={:?}, reason={:?}",
                   context.disarm_method, reason);

        // 2. Verify actuators returned to neutral
        context.actuators.verify_neutral_state()?;

        // 3. Notify subsystems of disarm
        context.monitoring.on_disarmed();
        context.failsafe.on_disarmed();
        context.mode_manager.on_disarmed();

        // 4. Disable safety boundaries
        if context.fence.is_enabled() && context.params.get_bool("FENCE_AUTOENABLE") {
            context.fence.disable();
        }

        // 5. Persist configuration changes made during flight
        if context.params.has_unsaved_changes() {
            context.params.save_to_flash()?;
        }

        // 6. Clear armed-state-specific data
        context.state.clear_arm_time();
        context.monitoring.reset();

        Ok(())
    }
}
```

### Integration with SystemState

Update `SystemState` to integrate all five subsystems:

```rust
// src/communication/mavlink/state.rs (updated)
impl SystemState {
    pub fn arm(&mut self, context: &SystemContext, method: ArmMethod) -> Result<(), ArmingError> {
        // 1. Run pre-arm checks
        let check_result = context.arming_checker.run_checks(context);
        if let ArmingResult::Denied { reason, category } = check_result {
            log::warn!("Arming denied: {} (category: {:?})", reason, category);
            return Err(ArmingError::CheckFailed { reason, category });
        }

        // 2. Transition to armed state
        self.armed = true;

        // 3. Execute post-arm initialization
        context.post_arm_init.execute(context)?;

        log::info!("Vehicle armed successfully via {:?}", method);
        Ok(())
    }

    pub fn arm_forced(&mut self, context: &SystemContext, method: ArmMethod) -> Result<(), ArmingError> {
        // Force arm bypasses all pre-arm checks (emergency/testing only)
        log::warn!("FORCE ARM requested via {:?} - bypassing all pre-arm checks", method);

        // 1. Transition to armed state (skip checks)
        self.armed = true;

        // 2. Execute post-arm initialization (still required for safety)
        context.post_arm_init.execute(context)?;

        log::warn!("Vehicle FORCE armed - operator accepted risk");
        Ok(())
    }

    pub fn disarm(
        &mut self,
        context: &SystemContext,
        method: DisarmMethod,
        reason: DisarmReason,
        forced: bool,
    ) -> Result<(), DisarmError> {
        // 1. Run pre-disarm validation
        context.disarm_validator.validate(context, method, forced)?;

        // 2. Transition to disarmed state
        self.armed = false;

        // 3. Execute post-disarm cleanup
        context.post_disarm_cleanup.execute(context, reason)?;

        log::info!("Vehicle disarmed successfully");
        Ok(())
    }
}
```

### Decision Drivers

- **Safety-first design**: Comprehensive validation prevents unsafe operations
- **ArduPilot compatibility**: Uses standard parameters (ARMING_CHECK, FENCE_AUTOENABLE) and patterns
- **Resource efficiency**: Trait-based design minimizes memory, multi-rate monitoring optimizes CPU
- **Extensibility**: New checks/monitors added without modifying core logic
- **Clear failure reporting**: Specific error types and messages aid troubleshooting

### Considered Options

**Option A: Function-based checks**

- Simple check functions called sequentially
- Pros: Simpler, less code
- Cons: Hard to extend, no category grouping, difficult to disable selectively

**Option B: Trait-based checks (selected)**

- `PreArmCheck` trait with check registration
- Pros: Extensible, supports selective enabling via ARMING_CHECK, clear interface
- Cons: Slightly more code, requires Box/dynamic dispatch

**Option C: Macro-based checks**

- Use macros to generate check functions
- Pros: Less boilerplate
- Cons: Harder to debug, less flexible, obscures control flow

**Decision: Option B (Trait-based)** - Extensibility and ArduPilot ARMING_CHECK compatibility outweigh small code overhead.

## Rationale

**Why trait-based pre-arm checks?**
ArduPilot's AP_Arming library uses a similar pattern where checks are registered and executed in sequence. The trait approach allows new checks to be added without modifying core arming logic, and supports ArduPilot's ARMING_CHECK parameter for selective enabling. This aligns with the extensibility and compatibility requirements.

**Why multi-rate monitoring?**
Different health checks have different performance requirements. RC signal loss must be detected within 200ms (requires high-frequency checking), while geofence violations can be checked at 1 Hz. Using three monitoring rates (fast/medium/slow) balances safety responsiveness with CPU efficiency, meeting the < 10% overhead requirement on RP2040.

**Why separate pre-disarm validation?**
Disarming with motors running or while moving at high speed creates safety hazards. ArduPilot implements throttle and velocity checks before allowing disarm. Separate validation logic allows method-specific rules (RC disarm stricter than GCS command) and forced override for emergencies.

**Why force-arm override (param2=21196)?**
Provides a safer alternative to ARMING_CHECK=0 for bypassing pre-arm checks during bench testing, SITL simulations, or emergency recovery. Unlike ARMING_CHECK=0 (permanent configuration), force-arm is a one-time bypass requiring explicit GCS action. Each force-arm attempt is logged with warning severity, creating an audit trail without masking underlying check failures. This matches ArduPilot's approach and provides ArduPilot GCS (Mission Planner, QGroundControl) compatibility.

**Why post-arm initialization and post-disarm cleanup?**
These establish operational baselines and audit trails. Recording arm timestamps enables timeout logic (e.g., auto-disarm after inactivity). Logging arm/disarm events provides post-flight analysis. Actuator safety verification ensures motors actually stopped after disarm. These mirror ArduPilot's initialization and cleanup sequences.

**Trade-offs accepted:**

- Trait objects require dynamic dispatch (small performance cost) for extensibility
- Comprehensive checking adds 500ms to arm latency vs instant arming (acceptable for safety)
- Continuous monitoring uses \~10% CPU on RP2040 (acceptable given safety benefits)

## Consequences

### Positive

- **Safety baseline**: Prevents arming with failed systems, detects failures during operation
- **Clear failure feedback**: Operators receive specific reasons for denied arming/disarming
- **Audit trail**: All arm/disarm events logged with timestamps and reasons
- **ArduPilot compatibility**: Uses standard parameters and patterns familiar to operators
- **Extensibility**: New checks/monitors added without core logic changes
- **Resource efficient**: Meets 5 KB RAM budget and < 10% CPU overhead target

### Negative

- **Arm latency**: Pre-arm checks add \~500ms delay vs instant arming
- **Code complexity**: Five integrated subsystems increase codebase size (\~2000 LOC)
- **Testing burden**: Each check/monitor requires unit tests and hardware validation
- **Operator learning curve**: Must understand ARMING_CHECK parameter and failure messages

### Neutral

- Monitoring framework usable for other systems beyond arming (e.g., general health reporting)
- Error types and context structures reusable across subsystems

## Implementation Notes

### Module Structure

```
src/core/arming/
├── mod.rs              # Public API, ArmingChecker, monitoring rates
├── checks.rs           # PreArmCheck trait, built-in checks
├── initialization.rs   # PostArmInitializer
├── monitoring.rs       # ArmedStateMonitor (multi-rate)
├── disarm.rs           # DisarmValidator
├── cleanup.rs          # PostDisarmCleanup
└── error.rs            # ArmingError, DisarmError types
```

### Implementation Phases

**Phase 1: Pre-Arm Checks + Post-Arm Initialization**

- Implement trait-based check framework
- Add critical checks: RC input, battery, system state
- Implement post-arm initialization sequence
- Integrate with SystemState::arm()

**Phase 2: Armed State Monitoring**

- Implement multi-rate monitoring architecture
- Add high-frequency RC signal age tracking
- Add medium-frequency battery/sensor monitoring
- Add low-frequency status reporting

**Phase 3: Disarm Validation + Post-Disarm Cleanup**

- Implement disarm validator with method-specific rules
- Add forced disarm override mechanism
- Implement post-disarm cleanup sequence
- Verify actuator safety in cleanup

### MAVLink Command Integration

Force-arm and force-disarm support via MAV_CMD_COMPONENT_ARM_DISARM:

```rust
// src/communication/mavlink/handlers/command.rs
pub fn handle_component_arm_disarm(
    cmd: &COMMAND_LONG_DATA,
    state: &mut SystemState,
    context: &SystemContext,
) -> MavResult {
    const FORCE_MAGIC_NUMBER: f32 = 21196.0;  // ArduPilot standard magic number

    let arm_request = cmd.param1 == 1.0;
    let force = cmd.param2 == FORCE_MAGIC_NUMBER;

    if arm_request {
        // Arm vehicle
        let result = if force {
            state.arm_forced(context, ArmMethod::GcsCommand)
        } else {
            state.arm(context, ArmMethod::GcsCommand)
        };

        match result {
            Ok(()) => {
                let msg = if force { "Armed (FORCED)" } else { "Armed" };
                context.mavlink.send_statustext(msg,
                    if force { MavSeverity::MAV_SEVERITY_WARNING }
                    else { MavSeverity::MAV_SEVERITY_INFO });
                MavResult::MAV_RESULT_ACCEPTED
            }
            Err(e) => {
                context.mavlink.send_statustext(
                    &format!("Arm failed: {}", e),
                    MavSeverity::MAV_SEVERITY_ERROR
                );
                MavResult::MAV_RESULT_FAILED
            }
        }
    } else {
        // Disarm vehicle
        let result = state.disarm(
            context,
            DisarmMethod::GcsCommand,
            DisarmReason::Operator,
            force,  // Pass force flag to disarm validator
        );

        match result {
            Ok(()) => {
                context.mavlink.send_statustext("Disarmed", MavSeverity::MAV_SEVERITY_INFO);
                MavResult::MAV_RESULT_ACCEPTED
            }
            Err(e) => {
                context.mavlink.send_statustext(
                    &format!("Disarm failed: {}", e),
                    MavSeverity::MAV_SEVERITY_ERROR
                );
                MavResult::MAV_RESULT_FAILED
            }
        }
    }
}
```

**Magic Number (21196) Rationale**:

- ArduPilot standard value for force operations
- Same value used for both force-arm and force-disarm
- Unlikely to be set accidentally (requires explicit GCS support)
- Documented in ArduPilot MAVLink developer documentation

### ArduPilot Parameter Mapping

**Arming Configuration (`ARMING_*`)**

- `ARMING_CHECK`: Bitmask to enable/disable pre-arm check categories (all, barometer, compass, GPS, etc.)
- `ARMING_REQUIRE`: Require arming before motors can be enabled (0=disabled, 1=enabled)
- `ARMING_ACCTHRESH`: Maximum accelerometer error allowed for arming (m/s²)
- `ARMING_RUDDER`: Enable/disable rudder stick arming method (0=disabled, 1=ArmOnly, 2=ArmOrDisarm)
- `ARMING_MIS_ITEMS`: Require mission with minimum waypoints loaded before arming
- `ARMING_OPTIONS`: Arming behavior flags (bit 0: ignore throttle for arming, bit 1: ignore safety switch)
- `ARMING_MAGTHRESH`: Maximum compass field strength error allowed for arming (%)

**Battery Monitoring (`BATT*_*`)**

- `BATT_ARM_VOLT`: Minimum battery voltage required to arm (V)
- `BATT_ARM_MAH`: Minimum battery capacity remaining to arm (mAh)
- `BATT_LOW_VOLT`: Low battery voltage threshold for warnings (V)
- `BATT_LOW_MAH`: Low battery capacity threshold for warnings (mAh)
- `BATT_CRT_VOLT`: Critical battery voltage triggering failsafe (V)
- `BATT_CRT_MAH`: Critical battery capacity triggering failsafe (mAh)
- `BATT_FS_LOW_ACT`: Action when battery low threshold reached
- `BATT_FS_CRT_ACT`: Action when battery critical threshold reached

**Failsafe Configuration (`FS_*`)**

- `FS_ACTION`: Default failsafe action (0=None, 1=RTL, 2=Hold, 3=SmartRTL, 4=Disarm)
- `FS_TIMEOUT`: Failsafe timeout for GCS heartbeat loss (seconds)
- `FS_GCS_ENABLE`: Enable GCS heartbeat loss failsafe (0=disabled, 1=enabled)
- `FS_THR_ENABLE`: Enable throttle failsafe (0=disabled, 1=enabled)
- `FS_THR_VALUE`: PWM value below which throttle failsafe triggers
- `FS_EKF_ACTION`: Action when EKF/DCM reports unhealthy
- `FS_CRASH_CHECK`: Enable crash detection (0=disabled, 1=hold, 2=disarm)

**Fence Configuration (`FENCE_*`)**

- `FENCE_AUTOENABLE`: Auto-enable fence on arm, auto-disable on disarm (0=disabled, 1=enabled)
- `FENCE_ACTION`: Action when fence is breached (0=Report, 1=RTL, 2=Hold, 3=SmartRTL, 4=Disarm)

**Motor Safety (`MOT_*`)**

- `MOT_SAFE_DISARM`: Behavior when disarmed (0=PWM disabled, 1=zero throttle)
- `MOT_SAFE_TIME`: Time delay for motors to spin up after arming (seconds)

### Error Handling

All arming operations return `Result<(), ArmingError>` or `Result<(), DisarmError>` with specific error variants:

```rust
pub enum ArmingError {
    CheckFailed { reason: String, category: CheckCategory },
    InitializationFailed { subsystem: &'static str },
    AlreadyArmed,
}

pub enum DisarmError {
    ValidationFailed { reason: String },
    NotArmed,
    ThrottleNotLow { current: f32 },
    VelocityTooHigh { current: f32, max: f32 },
}
```

Errors are logged locally and reported to GCS via MAVLink COMMAND_ACK with failure reason in result_param2.

## Platform Considerations

- **RP2040 (Pico W)**: No hardware FPU, software float operations slower
  - Use integer arithmetic where possible in high-frequency monitors
  - Battery voltage checks use fixed-point math
- **RP2350 (Pico 2 W)**: Hardware FPU available
  - Floating-point calculations faster, can use more sophisticated checks (EKF validation)
- **Embassy async**: Monitoring tasks run concurrently via Embassy executor
  - Use `embassy_time::Timer` for periodic checks
  - Share state via `Mutex<RefCell<T>>` or channels

## Security & Privacy

- **ARMING_CHECK=0**: Disabling all checks creates safety risk
  - Log warning when armed with checks disabled
  - Document that ARMING_CHECK=0 only for bench testing
- **Force-arm**: Emergency override bypasses all pre-arm checks
  - Requires explicit `param2=21196` in MAV_CMD_COMPONENT_ARM_DISARM
  - Logged with WARNING severity and "FORCE ARM" audit marker
  - STATUSTEXT message sent to GCS: "Armed (FORCED)"
  - Safer than ARMING_CHECK=0 (one-time bypass, doesn't persist)
  - All force-arm attempts logged for post-flight analysis
- **Forced disarm**: Emergency override bypasses validation
  - Requires explicit `param2=21196` in MAVLink command (same magic number as force-arm)
  - Log all forced disarms for audit
- **Audit trail**: Arm/disarm events logged with timestamps
  - Stored in non-volatile log for post-flight analysis
  - Includes method (RC/GCS/failsafe) and failure reasons
  - Force operations marked with "FORCED" flag for easy identification

## Open Questions

- **Should we implement auto-disarm after inactivity?** (e.g., ArduPilot's LAND_DISARMDELAY)
  - Leaning yes: Safety feature to prevent leaving vehicle armed unintentionally
  - Can be configurable parameter (0 = disabled)

- **Should EKF health check be mandatory pre-arm or advisory?**
  - Leaning advisory: Allows operation with GPS-denied navigation (mag/accel-only AHRS)
  - Can be enforced via ARMING_CHECK bitmask if desired

- **Should we implement safety switch support?** (physical switch to enable arming)
  - Leaning no for Phase 1: Adds hardware dependency, not common on small rovers
  - Can be added later if needed via GPIO

## References

- ArduPilot AP_Arming library: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Arming>
- ArduPilot pre-arm check documentation: <https://ardupilot.org/copter/docs/common-prearm-safety-checks.html>
- ArduPilot ARMING_CHECK parameter: <https://ardupilot.org/rover/docs/parameters.html#arming-check-bitmask>
