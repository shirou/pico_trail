# T-zmv9u Arming System Implementation

## Metadata

- Type: Implementation Plan
- Status: Draft

## Links

- Associated Design Document:
  - [T-zmv9u-arming-system-implementation-design](./design.md)

## Overview

This plan implements a comprehensive trait-based arming system with five integrated subsystems: Pre-Arm Checks, Post-Arm Initialization, Armed State Monitoring, Pre-Disarm Validation, and Post-Disarm Cleanup. The implementation follows a three-phase approach that builds upon each subsystem progressively, ensuring safety validation throughout the arm/disarm lifecycle.

## Success Metrics

- [ ] Zero instances of arming with failed critical systems (RC, battery, sensors) in testing
- [ ] Pre-arm checks complete < 500ms, monitoring detection time < 200ms
- [ ] Arming system RAM usage < 5 KB total measured on RP2040
- [ ] Monitoring overhead < 10% CPU on RP2040 @ 133 MHz
- [ ] Adding new pre-arm check requires < 50 lines of code
- [ ] Compatible with QGroundControl and Mission Planner without GCS changes
- [ ] All existing tests pass; no regressions in MAVLink command handling

## Scope

- Goal: Implement comprehensive arming safety system preventing unsafe operations through validation and continuous monitoring
- Non-Goals: Auto-disarm after inactivity, safety switch support, advanced geofence shapes, EKF mandatory pre-arm checks
- Assumptions: SystemContext provides access to RC, battery, sensors, actuators, and parameters; Embassy async executor available for concurrent monitoring tasks
- Constraints: Memory budget 5 KB, pre-arm latency < 500ms, monitoring overhead < 10% CPU, must work on RP2040 (no FPU) and RP2350 (with FPU)

## ADR & Legacy Alignment

- [ ] Confirm [ADR-w8d02-arming-system-architecture](../../adr/ADR-w8d02-arming-system-architecture.md) governs this work
- [ ] Current implementation in `src/communication/mavlink/handlers/command.rs:84-113` performs only armed state checking - will be replaced with comprehensive validation

## Plan Summary

- Phase 1 – Pre-Arm Check Framework + Post-Arm Initialization
- Phase 2 – Armed State Monitoring (Multi-Rate)
- Phase 3 – Pre-Disarm Validation + Post-Disarm Cleanup

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task or subtask. If an item is intentionally skipped or deferred, annotate it with a brief note instead of leaving it unchecked.

---

## Phase 1: Pre-Arm Check Framework + Post-Arm Initialization

### Goal

- Establish trait-based pre-arm check framework with category filtering
- Implement post-arm initialization sequence
- Integrate with SystemState::arm() method

### Inputs

- Documentation:
  - [ADR-w8d02-arming-system-architecture](../../adr/ADR-w8d02-arming-system-architecture.md) - Architecture decision and design patterns
  - [design.md](./design.md) - Detailed component specifications
- Source Code to Modify:
  - `src/communication/mavlink/state.rs` - SystemState::arm() integration
  - `src/communication/mavlink/handlers/command.rs` - MAVLink COMPONENT_ARM_DISARM handler
- Dependencies:
  - Internal: `src/communication/mavlink/state.rs` - SystemState for armed state management
  - External crates: `embassy_time` - Timestamp recording

### Tasks

- [ ] **Create arming module structure**
  - [ ] Create `src/core/arming/` directory
  - [ ] Create `src/core/arming/mod.rs` with public API
  - [ ] Create `src/core/arming/error.rs` with ArmingError/DisarmError types
  - [ ] Register arming module in `src/core/mod.rs`

- [ ] **Implement PreArmCheck trait and framework**
  - [ ] Define `PreArmCheck` trait in `src/core/arming/checks.rs`
  - [ ] Define `CheckCategory` enum with ArduPilot bitmask values
  - [ ] Define `ArmingResult` enum (Allowed/Denied)
  - [ ] Implement `ArmingChecker` with check registration and execution
  - [ ] Implement category filtering based on ARMING_CHECK parameter

- [ ] **Implement built-in pre-arm checks**
  - [ ] `RcInputCheck` - Verify RC signal present and valid
  - [ ] `BatteryVoltageCheck` - Verify battery above BATT_ARM_VOLT
  - [ ] `SystemStateCheck` - Verify no active errors in system state
  - [ ] `ImuHealthCheck` - Verify IMU calibrated and healthy
  - [ ] `ActuatorReadyCheck` - Verify actuators initialized

- [ ] **Implement post-arm initialization**
  - [ ] Create `src/core/arming/initialization.rs`
  - [ ] Implement `PostArmInitializer::execute()`
  - [ ] Record arm timestamp using `embassy_time::Instant::now()`
  - [ ] Log arm event with method (RC/GCS)
  - [ ] Initialize actuators via `context.actuators.enter_armed_state()`
  - [ ] Notify subsystems (monitoring, failsafe, mode_manager) of arm event
  - [ ] Enable geofence if FENCE_AUTOENABLE parameter set
  - [ ] Warn if checks disabled via ARMING_CHECK

- [ ] **Integrate with SystemState**
  - [ ] Update `SystemState::arm()` to call `ArmingChecker::run_checks()`
  - [ ] Return ArmingError with reason and category on check failure
  - [ ] Call `PostArmInitializer::execute()` after setting armed=true
  - [ ] Implement `SystemState::arm_forced()` to bypass pre-arm checks (FR-exazo)
  - [ ] Update MAVLink COMPONENT_ARM_DISARM handler to detect param2=21196 (force-arm magic number)
  - [ ] Update MAVLink handler to call `arm_forced()` when param2=21196 detected
  - [ ] Log force-arm with WARNING severity and "FORCE ARM" audit marker
  - [ ] Send STATUSTEXT "Armed (FORCED)" with MAV_SEVERITY_WARNING on force-arm
  - [ ] Update MAVLink COMPONENT_ARM_DISARM handler to report check failures in COMMAND_ACK

- [ ] **Add unit tests for Phase 1**
  - [ ] Test PreArmCheck trait with mock checks
  - [ ] Test ArmingChecker category filtering with ARMING_CHECK parameter
  - [ ] Test built-in checks (RC, battery, system state, IMU, actuators)
  - [ ] Test PostArmInitializer sequence
  - [ ] Test SystemState::arm() integration with check failures
  - [ ] Test SystemState::arm_forced() bypasses all pre-arm checks
  - [ ] Test force-arm succeeds even when pre-arm checks would fail
  - [ ] Test force-arm logs WARNING and sends "Armed (FORCED)" STATUSTEXT
  - [ ] Test MAVLink handler detects param2=21196 and calls arm_forced()

### Deliverables

- `src/core/arming/` module with trait-based pre-arm check framework
- Five built-in pre-arm checks (RC, battery, system state, IMU, actuators)
- Post-arm initialization sequence
- Integration with SystemState::arm() method
- Force-arm support (SystemState::arm_forced() and MAVLink param2=21196 handling)
- Unit tests for all checks, initialization logic, and force-arm override

### Verification

```bash
# Build and checks
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
# Focused unit tests
cargo test --lib --quiet arming
```

### Acceptance Criteria (Phase Gate)

- [ ] `ArmingChecker::run_checks()` executes all enabled checks in < 500ms
- [ ] Check failures return specific ArmingError with reason and category
- [ ] Post-arm initialization logs arm event and records timestamp
- [ ] Force-arm (param2=21196) bypasses all pre-arm checks and succeeds
- [ ] Force-arm logs WARNING and sends "Armed (FORCED)" to GCS
- [ ] All unit tests pass for pre-arm checks, initialization, and force-arm
- [ ] No clippy warnings in arming module

### Rollback/Fallback

- Revert to minimal arming logic in `src/communication/mavlink/handlers/command.rs:84-113`
- Alternative: Implement function-based checks instead of trait-based (simpler but less extensible)

---

## Phase 2: Armed State Monitoring (Multi-Rate)

### Phase 2 Goal

- Implement multi-rate continuous health monitoring during armed operation
- High-frequency (400 Hz): RC signal age, sensor health
- Medium-frequency (10 Hz): Battery voltage, EKF status
- Low-frequency (1 Hz): Geofence violations, GCS status reporting

### Phase 2 Inputs

- Dependencies:
  - Phase 1: Pre-arm checks and post-arm initialization must be complete
  - `embassy_time::Timer` - Periodic execution at multiple rates
  - SystemContext with RC, battery, sensors, EKF, fence, failsafe access
- Source Code to Modify:
  - `src/core/arming/monitoring.rs` - ArmedStateMonitor implementation

### Phase 2 Tasks

- [ ] **Create ArmedStateMonitor structure**
  - [ ] Create `src/core/arming/monitoring.rs`
  - [ ] Define `ArmedStateMonitor` struct with high/medium/low-frequency state
  - [ ] Define `SensorHealthFlags`, `EkfStatus`, `FenceStatus` types (or import from existing modules)

- [ ] **Implement high-frequency monitoring (400 Hz)**
  - [ ] Implement `ArmedStateMonitor::update_fast()`
  - [ ] Track RC signal freshness via `rc_last_received: Option<Instant>`
  - [ ] Check RC timeout (> 1 second) and trigger `FailsafeReason::RcLoss`
  - [ ] Update sensor health flags from `context.sensors`

- [ ] **Implement medium-frequency monitoring (10 Hz)**
  - [ ] Implement `ArmedStateMonitor::update_medium()`
  - [ ] Monitor battery voltage via `context.battery.voltage()`
  - [ ] Trigger `FailsafeReason::BatteryCritical` if below BATT_CRT_VOLT
  - [ ] Validate EKF health via `context.ahrs.ekf_status()`
  - [ ] Log EKF unhealthy warnings

- [ ] **Implement low-frequency monitoring (1 Hz)**
  - [ ] Implement `ArmedStateMonitor::update_slow()`
  - [ ] Check geofence violations via `context.fence.check_boundaries()`
  - [ ] Trigger `FailsafeReason::FenceViolation` if fence violated
  - [ ] Send SYS_STATUS to GCS with sensor health, battery, EKF status

- [ ] **Integrate with Embassy async executor**
  - [ ] Create async tasks for fast/medium/slow monitoring loops
  - [ ] Use `Timer::after()` for periodic execution (2.5ms / 100ms / 1000ms)
  - [ ] Share ArmedStateMonitor via `Mutex<RefCell<...>>` or channels
  - [ ] Spawn monitoring tasks in post-arm initialization
  - [ ] Stop monitoring tasks in post-disarm cleanup

- [ ] **Add unit tests for Phase 2**
  - [ ] Test RC timeout detection triggers failsafe < 200ms
  - [ ] Test battery critical threshold triggers failsafe
  - [ ] Test geofence violation triggers failsafe
  - [ ] Test monitoring state updates at correct rates

### Phase 2 Deliverables

- `ArmedStateMonitor` with multi-rate monitoring (400 Hz / 10 Hz / 1 Hz)
- RC signal age tracking with timeout failsafe
- Battery voltage monitoring with critical failsafe
- Geofence violation checking
- GCS status reporting
- Embassy async task integration

### Phase 2 Verification

```bash
cargo check
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet monitoring
```

### Phase 2 Acceptance Criteria

- [ ] RC timeout detected within 200ms of signal loss
- [ ] Battery critical failsafe triggered when voltage drops below threshold
- [ ] Monitoring overhead < 10% CPU on RP2040 @ 133 MHz
- [ ] All unit tests pass for monitoring logic
- [ ] No clippy warnings in monitoring module

### Phase 2 Rollback/Fallback

- Disable continuous monitoring (armed state without health checks)
- Alternative: Single-rate monitoring (e.g., 10 Hz for all checks) to reduce complexity

---

## Phase 3: Pre-Disarm Validation + Post-Disarm Cleanup

### Phase 3 Goal

- Implement pre-disarm validation with method-specific rules
- Implement post-disarm cleanup sequence
- Integrate with SystemState::disarm() method

### Phase 3 Tasks

- [ ] **Implement DisarmValidator**
  - [ ] Create `src/core/arming/disarm.rs`
  - [ ] Implement `DisarmValidator::validate()` with method and forced parameters
  - [ ] Implement forced disarm override (bypasses all validation)
  - [ ] Implement method-specific validation logic (RC stick, GCS command, failsafe)
  - [ ] Implement `validate_throttle_low()` - Throttle < 10%
  - [ ] Implement `validate_velocity_safe()` - Ground speed < 0.5 m/s
  - [ ] Return DisarmError with specific reasons (ThrottleNotLow, VelocityTooHigh)

- [ ] **Implement post-disarm cleanup**
  - [ ] Create `src/core/arming/cleanup.rs`
  - [ ] Implement `PostDisarmCleanup::execute()`
  - [ ] Log disarm event with method and reason
  - [ ] Verify actuators returned to neutral via `context.actuators.verify_neutral_state()`
  - [ ] Notify subsystems (monitoring, failsafe, mode_manager) of disarm event
  - [ ] Disable geofence if FENCE_AUTOENABLE enabled
  - [ ] Persist configuration changes if `context.params.has_unsaved_changes()`
  - [ ] Clear arm timestamp and reset monitoring state

- [ ] **Integrate with SystemState**
  - [ ] Update `SystemState::disarm()` to call `DisarmValidator::validate()`
  - [ ] Return DisarmError with reason on validation failure
  - [ ] Call `PostDisarmCleanup::execute()` after setting armed=false
  - [ ] Update MAVLink COMPONENT_ARM_DISARM handler to report validation failures in COMMAND_ACK
  - [ ] Support forced disarm via MAVLink command parameter

- [ ] **Add unit tests for Phase 3**
  - [ ] Test disarm validation fails with high throttle
  - [ ] Test disarm validation fails with high velocity
  - [ ] Test forced disarm bypasses all validation
  - [ ] Test method-specific validation rules (RC vs GCS vs failsafe)
  - [ ] Test PostDisarmCleanup sequence
  - [ ] Test SystemState::disarm() integration with validation failures

### Phase 3 Deliverables

- `DisarmValidator` with method-specific pre-disarm checks
- `PostDisarmCleanup` with shutdown sequence
- Integration with SystemState::disarm() method
- Forced disarm override support
- Unit tests for disarm validation and cleanup

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet arming
# Full test suite
cargo test --quiet
```

### Phase 3 Acceptance Criteria

- [ ] Disarm validation prevents unsafe disarms (throttle high, moving)
- [ ] Forced disarm bypasses validation with audit log warning
- [ ] Post-disarm cleanup verifies actuators neutral
- [ ] All unit tests pass for disarm validation and cleanup
- [ ] No clippy warnings in disarm/cleanup modules

---

## Definition of Done

- [ ] `cargo check`
- [ ] `cargo fmt`
- [ ] `cargo clippy --all-targets -- -D warnings`
- [ ] `cargo test --lib --quiet`
- [ ] Memory usage < 5 KB measured on RP2040 hardware
- [ ] Pre-arm checks complete < 500ms, monitoring detection < 200ms measured on RP2040 hardware
- [ ] CPU overhead < 10% measured on RP2040 @ 133 MHz with monitoring active
- [ ] User documentation updated with arming system parameters and troubleshooting
- [ ] No `unsafe` code and no vague naming (no "manager"/"util")

## Open Questions

- [ ] Should we implement auto-disarm after inactivity? → Next step: Defer to future enhancement (requires LAND_DISARMDELAY parameter)
- [ ] Should EKF health check be mandatory pre-arm or advisory? → Next step: Implement as advisory, make configurable via ARMING_CHECK bitmask
- [ ] Should we implement safety switch support? → Next step: Defer to future enhancement (hardware dependency)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
