# T-zmv9u Arming System Implementation

## Metadata

- Type: Design
- Status: Complete

## Links

- Associated Plan Document:
  - [T-zmv9u-arming-system-implementation-plan](./plan.md)

## Overview

This design implements a comprehensive arming safety system for pico_trail, preventing unsafe operations through validation at arm/disarm transitions and continuous monitoring during armed operation. The system addresses critical safety gaps in the current minimal arming implementation by introducing five integrated subsystems: trait-based pre-arm checks, post-arm initialization, multi-rate armed state monitoring, pre-disarm validation, and post-disarm cleanup.

## Success Metrics

- [ ] Zero instances of arming with failed critical systems (RC, battery, sensors) in testing
- [ ] Pre-arm checks complete < 500ms, monitoring detection time < 200ms
- [ ] Arming system RAM usage < 5 KB total measured on RP2040
- [ ] Monitoring overhead < 10% CPU on RP2040 @ 133 MHz
- [ ] Adding new pre-arm check requires < 50 lines of code
- [ ] Compatible with QGroundControl and Mission Planner without GCS changes

## Background and Current State

- Context: Currently pico_trail implements basic arming/disarming via MAVLink COMPONENT_ARM_DISARM command (`src/communication/mavlink/handlers/command.rs:84-113`) with only armed state checking
- Current behavior: System can arm with failed sensors, no RC connection, low battery, and lacks continuous health monitoring during armed operation
- Pain points: Critical safety gaps allow unsafe arming conditions; no validation prevents dangerous disarm scenarios (motors running, vehicle moving)
- Constraints: Memory budget 5 KB, pre-arm latency < 500ms, monitoring overhead < 10% CPU, must work on RP2040 (no FPU) and RP2350 (with FPU)
- Related ADRs: [ADR-w8d02-arming-system-architecture](../../adr/ADR-w8d02-arming-system-architecture.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                         SystemState                             │
│                    (arm/disarm coordination)                    │
└────────┬────────────────────────────────────────────┬───────────┘
         │                                            │
    ┌────▼─────┐                                ┌────▼──────┐
    │   ARM    │                                │  DISARM   │
    │  Request │                                │  Request  │
    └────┬─────┘                                └────┬──────┘
         │                                            │
    ┌────▼─────────────┐                       ┌────▼────────────────┐
    │  ArmingChecker   │                       │ DisarmValidator     │
    │ (Pre-Arm Checks) │                       │ (Pre-Disarm Checks) │
    └────┬─────────────┘                       └────┬────────────────┘
         │                                            │
         │ Allowed                                    │ Allowed
         ▼                                            ▼
    ┌─────────────────┐                       ┌─────────────────┐
    │ PostArmInit     │                       │ PostDisarmClean │
    │ (Initialization)│                       │ (Cleanup)       │
    └─────────────────┘                       └─────────────────┘
         │
         │ Armed state
         ▼
    ┌──────────────────────────────────────┐
    │     ArmedStateMonitor                │
    │  (Continuous Health Monitoring)      │
    │                                      │
    │  ┌─────────┐  ┌────────┐  ┌──────┐ │
    │  │400 Hz   │  │ 10 Hz  │  │ 1 Hz │ │
    │  │RC signal│  │Battery │  │Fence │ │
    │  └─────────┘  └────────┘  └──────┘ │
    └──────────────────────────────────────┘
```

### Components

**1. Pre-Arm Check Framework (`src/core/arming/checks.rs`)**

- `PreArmCheck` trait: Interface for extensible validation checks
  - `check(&self, context: &SystemContext) -> Result<(), ArmingError>`
  - `name(&self) -> &'static str`
  - `category(&self) -> CheckCategory`
- `ArmingChecker`: Orchestrates check execution with category filtering
- Built-in checks: `RcInputCheck`, `BatteryVoltageCheck`, `SystemStateCheck`, `ImuHealthCheck`, `ActuatorReadyCheck`
- Category-based enabling via ARMING_CHECK parameter (bitmask)

**2. Post-Arm Initialization (`src/core/arming/initialization.rs`)**

- `PostArmInitializer`: Executes initialization sequence after successful arming
- Responsibilities: Record arm timestamp, log arm event, initialize actuators, notify subsystems, enable geofence if configured, warn if checks disabled

**3. Armed State Monitor (`src/core/arming/monitoring.rs`)**

- `ArmedStateMonitor`: Multi-rate health monitoring during armed operation
- High-frequency (400 Hz): RC signal age tracking, triggers failsafe on timeout
- Medium-frequency (10 Hz): Battery voltage monitoring, EKF health validation
- Low-frequency (1 Hz): Geofence violation checking, GCS status reporting

**4. Pre-Disarm Validation (`src/core/arming/disarm.rs`)**

- `DisarmValidator`: Safety checks before allowing disarm
- Method-specific rules: RC stick (strictest), GCS command (lenient), failsafe (no checks)
- Validations: Throttle low, velocity safe, armed state verification
- Forced disarm override for emergencies (bypasses all validation)

**5. Post-Disarm Cleanup (`src/core/arming/cleanup.rs`)**

- `PostDisarmCleanup`: Shutdown sequence after disarming
- Responsibilities: Log disarm event, verify actuators neutral, notify subsystems, disable geofence, persist configuration changes, clear armed-state data

### Data Flow

**Arming Sequence:**

1. MAVLink COMPONENT_ARM_DISARM command received → `SystemState::arm()`
2. `ArmingChecker::run_checks()` validates all enabled pre-arm checks
3. If denied: Return error with reason and category to GCS
4. If allowed: Set `self.armed = true`
5. `PostArmInitializer::execute()` performs initialization sequence
6. `ArmedStateMonitor` begins continuous monitoring at multiple rates
7. Send COMMAND_ACK success to GCS

**Disarming Sequence:**

1. Disarm request received → `SystemState::disarm()`
2. `DisarmValidator::validate()` checks method-specific rules (unless forced)
3. If validation fails: Return error with reason to GCS
4. If allowed: Set `self.armed = false`
5. `PostDisarmCleanup::execute()` performs shutdown sequence
6. Send COMMAND_ACK success to GCS

**Monitoring Loop (during armed operation):**

1. High-frequency task (400 Hz): Update RC signal age, trigger RC loss failsafe if timeout
2. Medium-frequency task (10 Hz): Update battery voltage, trigger critical battery failsafe if needed, validate EKF health
3. Low-frequency task (1 Hz): Check geofence violations, send SYS_STATUS to GCS

### Data Models and Types

**Core Types:**

```rust
pub trait PreArmCheck {
    fn check(&self, context: &SystemContext) -> Result<(), ArmingError>;
    fn name(&self) -> &'static str;
    fn category(&self) -> CheckCategory;
}

pub struct ArmingChecker {
    checks: Vec<Box<dyn PreArmCheck>>,
    enabled_categories: CheckCategory, // Bitmask from ARMING_CHECK parameter
}

pub enum ArmingResult {
    Allowed,
    Denied { reason: String, category: CheckCategory },
}

pub enum CheckCategory {
    All       = 0xFFFF,
    Barometer = 0x0001,
    Compass   = 0x0002,
    Gps       = 0x0004,
    Ins       = 0x0008,  // IMU/INS
    Parameters= 0x0010,
    RcChannels= 0x0020,
    Board     = 0x0040,
    Battery   = 0x0080,
    Airspeed  = 0x0100,
    Logging   = 0x0200,
    Switch    = 0x0400,  // Safety switch
    GpsCfg    = 0x0800,  // GPS configuration
    System    = 0x1000,
}

pub enum ArmMethod {
    RcStick,
    GcsCommand,
}

pub enum DisarmMethod {
    RcStick,
    GcsCommand,
    Failsafe,
}

pub enum DisarmReason {
    UserRequest,
    Failsafe,
    AutoLand,
}
```

**Error Types:**

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

**Monitoring State:**

```rust
pub struct ArmedStateMonitor {
    // High-frequency state (400 Hz)
    rc_last_received: Option<Instant>,
    sensor_health: SensorHealthFlags,

    // Medium-frequency state (10 Hz)
    battery_voltage: f32,
    ekf_status: EkfStatus,

    // Low-frequency state (1 Hz)
    fence_status: FenceStatus,
}
```

### Error Handling

All arming operations return `Result<(), ArmingError>` or `Result<(), DisarmError>` with specific error variants for troubleshooting.

**Error Reporting:**

- Log errors locally via `log::warn!()` or `log::error!()`
- Report to GCS via MAVLink COMMAND_ACK with failure reason
- Use `result_param2` field for additional context (category bitmask, current values)

**Error Messages:**

- English only, actionable descriptions
- Examples:
  - "RC input check failed: No RC signal received"
  - "Battery voltage check failed: 10.5V < 11.0V minimum"
  - "Cannot disarm: Throttle at 45% (must be < 10%)"
  - "Cannot disarm: Ground speed 1.2 m/s exceeds 0.5 m/s maximum"

### Security Considerations

**ARMING_CHECK=0 Warning:**

- Disabling all checks creates safety risk (bench testing only)
- Log prominent warning when armed with checks disabled
- Include disabled categories in arm event log

**Forced Disarm Audit:**

- Emergency override bypasses all validation
- Requires explicit `force=1` parameter in MAVLink command
- Log all forced disarms with timestamp and method for audit trail

**Audit Trail:**

- Record all arm/disarm events with timestamps in non-volatile log
- Include method (RC/GCS/failsafe), success/failure, and denial reasons
- Enables post-flight safety analysis

### Performance Considerations

**Hot Paths:**

- Pre-arm checks run sequentially but short-circuit on first failure (< 500ms total)
- High-frequency monitoring (400 Hz) limited to RC signal age check and sensor flags (integer operations)
- Use fixed-point arithmetic on RP2040 for battery voltage comparisons (no FPU)

**Memory Optimization:**

- Trait objects use dynamic dispatch (small vtable overhead)
- Share `SystemContext` reference across all checks/monitors (no cloning)
- Monitor state fits in \~1 KB (timestamps, flags, cached values)
- Total arming system budget: \~5 KB RAM

**Concurrency:**

- Monitoring tasks run concurrently via Embassy async executor
- Use `embassy_time::Timer` for periodic checks at multiple rates
- Share state via `Mutex<RefCell<ArmedStateMonitor>>` or channels

### Platform Considerations

#### RP2040 (Pico W)

- No hardware FPU: Use integer arithmetic in high-frequency monitors
- Battery voltage checks use fixed-point math (millivolts as u32)
- Float operations acceptable in medium/low-frequency tasks (< 10% overhead target)

#### RP2350 (Pico 2 W)

- Hardware FPU available: Float calculations faster
- Can use more sophisticated checks (EKF covariance analysis)
- Same code path as RP2040 but better performance

#### Embassy Async

- Monitoring tasks spawned as separate async tasks
- Use `Timer::after()` for periodic execution
- Failsafe actions triggered via channels to main control task

## Alternatives Considered

1. **Function-based checks**
   - Pros: Simpler implementation, less code, no trait objects
   - Cons: Hard to extend, no category grouping, difficult to disable selectively, doesn't support ARMING_CHECK parameter

2. **Trait-based checks (selected)**
   - Pros: Extensible, supports selective enabling via ARMING_CHECK, clear interface, mirrors ArduPilot design
   - Cons: Slightly more code, requires Box/dynamic dispatch (minimal overhead)

3. **Macro-based checks**
   - Pros: Less boilerplate, generates check functions automatically
   - Cons: Harder to debug, less flexible, obscures control flow, poor IDE support

**Decision Rationale:** Trait-based approach selected for extensibility and ArduPilot ARMING_CHECK compatibility. Small dynamic dispatch overhead acceptable for safety-critical functionality.

## Migration and Compatibility

- Backward compatibility: Existing MAVLink COMPONENT_ARM_DISARM command interface unchanged
- New behavior: Arming now validates system health before allowing transition (breaking behavior change for unsafe conditions)
- Parameter compatibility: Uses standard ArduPilot parameters (ARMING_CHECK, BATT_CRT_VOLT, FENCE_AUTOENABLE, etc.)
- GCS compatibility: Works with QGroundControl and Mission Planner without changes (standard COMMAND_ACK response)
- Rollout plan: Single release (Phase 1-3 implemented together for safety completeness)

## Testing Strategy

### Unit Tests

```rust
// src/core/arming/checks.rs
#[cfg(test)]
mod tests {
    #[test]
    fn test_rc_input_check_passes_with_valid_signal() { ... }

    #[test]
    fn test_rc_input_check_fails_without_signal() { ... }

    #[test]
    fn test_battery_check_fails_below_minimum() { ... }
}

// src/core/arming/disarm.rs
#[cfg(test)]
mod tests {
    #[test]
    fn test_disarm_validation_fails_high_throttle() { ... }

    #[test]
    fn test_forced_disarm_bypasses_validation() { ... }
}
```

### Integration Tests

Integration tests under `tests/` directory would require hardware abstraction for RC input, sensors, battery monitoring. Defer to hardware-in-the-loop testing.

**Hardware Verification:**

- RP2040 Pico W: Verify pre-arm checks execute < 500ms, monitoring overhead < 10% CPU
- RP2350 Pico 2 W: Verify same functionality with hardware FPU
- Failsafe scenarios: Disconnect RC during armed operation, verify RC loss failsafe triggers < 200ms

## Documentation Impact

- Add `docs/reference/arming-system.md` describing check categories, parameters, and troubleshooting
- Update `docs/architecture.md` with arming system module structure
- Document all ARMING_CHECK categories and their checks in user documentation

## External References

- ArduPilot AP_Arming library: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Arming>
- ArduPilot pre-arm check documentation: <https://ardupilot.org/copter/docs/common-prearm-safety-checks.html>
- ArduPilot ARMING_CHECK parameter: <https://ardupilot.org/rover/docs/parameters.html#arming-check-bitmask>
- MAVLink COMMAND_LONG specification: <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>

## Open Questions

- [ ] Should we implement auto-disarm after inactivity? (ArduPilot's LAND_DISARMDELAY) → Next step: Defer to future enhancement (requires LAND_DISARMDELAY parameter support)
- [ ] Should EKF health check be mandatory pre-arm or advisory? → Next step: Start advisory, make configurable via ARMING_CHECK bitmask
- [ ] Should we implement safety switch support? → Next step: Defer to future enhancement (hardware dependency)

## Appendix

### Module Structure

```
src/core/arming/
├── mod.rs              # Public API, ArmingChecker, initialization
├── checks.rs           # PreArmCheck trait, built-in checks
├── initialization.rs   # PostArmInitializer
├── monitoring.rs       # ArmedStateMonitor (multi-rate)
├── disarm.rs           # DisarmValidator
├── cleanup.rs          # PostDisarmCleanup
└── error.rs            # ArmingError, DisarmError types
```

### ArduPilot Parameter Mapping

**Arming Configuration:**

- `ARMING_CHECK`: Bitmask to enable/disable pre-arm check categories
- `ARMING_REQUIRE`: Require arming before motors enabled
- `ARMING_ACCTHRESH`: Maximum accelerometer error (m/s²)
- `ARMING_RUDDER`: Rudder stick arming method
- `ARMING_OPTIONS`: Behavior flags

**Battery Monitoring:**

- `BATT_ARM_VOLT`: Minimum voltage to arm (V)
- `BATT_CRT_VOLT`: Critical voltage for failsafe (V)
- `BATT_FS_CRT_ACT`: Action when critical threshold reached

**Failsafe Configuration:**

- `FS_ACTION`: Default failsafe action
- `FS_TIMEOUT`: GCS heartbeat loss timeout (seconds)
- `FS_GCS_ENABLE`: Enable GCS heartbeat loss failsafe

**Fence Configuration:**

- `FENCE_AUTOENABLE`: Auto-enable fence on arm
- `FENCE_ACTION`: Action when fence breached

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
