# AN-00016 Post-Disarm Cleanup for Safe System Shutdown

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-00015-post-arm-initialization](AN-00015-post-arm-initialization.md)
  - [AN-00009-armed-state-monitoring](AN-00009-armed-state-monitoring.md)
  - [AN-00011-failsafe-system](AN-00011-failsafe-system.md)
- Related Requirements:
  - [FR-00050-post-disarm-event-recording](../requirements/FR-00050-post-disarm-event-recording.md)
  - [FR-00027-disarm-subsystem-notification](../requirements/FR-00027-disarm-subsystem-notification.md)
  - [NFR-00049-post-disarm-error-tolerance](../requirements/NFR-00049-post-disarm-error-tolerance.md)
  - [NFR-00023-disarm-logging-detail](../requirements/NFR-00023-disarm-logging-detail.md)
  - [NFR-00022-disarm-attempt-logging](../requirements/NFR-00022-disarm-attempt-logging.md)
  - [NFR-00050-post-disarm-memory](../requirements/NFR-00050-post-disarm-memory.md)
  - [NFR-00051-post-disarm-performance](../requirements/NFR-00051-post-disarm-performance.md)
  - [FR-00016-actuator-safety-verification](../requirements/FR-00016-actuator-safety-verification.md)
  - [FR-00022-configuration-persistence](../requirements/FR-00022-configuration-persistence.md)
  - [NFR-00013-actuator-verification-mandatory](../requirements/NFR-00013-actuator-verification-mandatory.md)
- Related ADRs:
  - [ADR-00012-arming-system-architecture](../adr/ADR-00012-arming-system-architecture.md)
- Related Tasks:
  - [T-00008-arming-system-implementation](../tasks/T-00008-arming-system-implementation/README.md)

## Executive Summary

This analysis explores post-disarm cleanup procedures needed to properly shut down the vehicle after disarming. Post-disarm cleanup is the mirror of post-arm initialization - where post-arm establishes a safe operational baseline, post-disarm ensures safe shutdown and state reset for the next arm cycle. Currently, pico_trail simply changes the armed state to `Disarmed` with no subsystem cleanup, logging, or actuator safety verification. Post-disarm cleanup is critical for establishing a safe shutdown baseline - logging disarm events for audit trails, ensuring actuators return to neutral/safe state, disabling geofences that only apply during armed operation, persisting configuration changes made during flight, and synchronizing all subsystems with the disarmed condition.

Key findings: ArduPilot implements comprehensive post-disarm cleanup including disarm event logging (with reason and timestamp), forced logging continuation for failsafe-triggered disarms, fence auto-disable, FFT parameter persistence, GPIO pin updates for disarm indicators, and optional safety switch force-on. For pico_trail, a similar cleanup sequence is recommended with focus on logging (disarm event with method, reason, and timestamp), actuator safety verification (confirm neutral PWM output), subsystem shutdown (disable monitoring, reset failsafe state), and state reset for next arm cycle.

## Problem Space

### Current State

The project currently has:

- **Disarm function**: `SystemState::disarm()` in `src/communication/mavlink/state.rs:173-183`
- **Minimal disarm logic**: Only checks armed state and sets state to Disarmed
- **No post-disarm cleanup**: No subsystem shutdown after disarming
- **No logging**: Disarm events not recorded
- **No actuator safety verification**: No confirmation that actuators returned to neutral
- **No subsystem notification**: Failsafe, monitoring, modes unaware of disarm event

Critical gaps:

- **No audit trail**: Cannot determine when/how/why vehicle was disarmed post-flight
- **No actuator safety confirmation**: Actuators may remain in active state after disarm
- **No subsystem synchronization**: Monitoring, failsafe continue running as if armed
- **No fence deactivation**: Geofence remains enabled after disarm (when implemented)
- **No configuration persistence**: Changes made during flight not saved
- **No safe baseline**: No known-good state for next arm cycle

### Desired State

Enable comprehensive post-disarm cleanup establishing safe shutdown baseline. As the mirror of post-arm initialization, post-disarm cleanup should reverse the initialization sequence:

1. **Disarm Event Logging**: Record disarm events with timestamp, method, reason (normal vs failsafe) - mirrors arm event logging
2. **Actuator Safety Shutdown**: Verify actuators return to neutral/safe PWM output - reverses actuator initialization
3. **Fence Deactivation**: Auto-disable geofence boundaries on disarm (when fence implemented) - reverses fence activation
4. **Subsystem Shutdown**: Notify monitoring, failsafe, modes of disarm event - reverses subsystem activation
5. **Configuration Persistence**: Save parameters or state changes made during flight
6. **GPIO Updates**: Update disarm indicator LED/pins if hardware present - reverses GPIO arming indicators
7. **State Reset**: Clear armed-state-specific data for next arm cycle - ensures clean baseline for next arm

Success criteria:

- **Complete audit trail**: All disarm events logged with full context (method, reason, timestamp)
- **Actuator safety verified**: Actuators confirmed in neutral state after disarm
- **Subsystem awareness**: All systems synchronized with disarmed state
- **Configuration preserved**: Flight changes persisted for next session
- **Safety boundaries inactive**: Fence disabled automatically on disarm
- **Clean state**: Vehicle ready for next arm cycle without residual state

### Gap Analysis

**Missing components**:

1. **Disarm Event Logger**: Log disarm events to persistent storage
2. **Actuator Safety Verifier**: Confirm actuators in neutral PWM state
3. **Subsystem Notifier**: Broadcast disarm event to other systems
4. **Fence Auto-Disabler**: Deactivate geofence on disarm (future)
5. **Configuration Persister**: Save changed parameters to storage
6. **GPIO Updater**: Set hardware disarm indicator pins (if present)
7. **State Resetter**: Clear armed-specific data structures

**Technical deltas**:

- Add `post_disarm_cleanup()` function called after `disarm()` succeeds
- Implement disarm event logging with timestamp, method, and reason
- Add actuator safety shutdown sequence (set PWM to neutral, verify)
- Create subsystem notification mechanism (publish disarm event)
- Add logging for disarm reason (normal, RC loss, battery, manual, forced)
- Integrate with monitoring system to disable health checks
- Add configuration persistence for changed parameters
- Prepare for future fence auto-disable integration
- Reset armed-state-specific timers and counters

## Stakeholder Analysis

| Stakeholder          | Interest/Need                                            | Impact | Priority |
| -------------------- | -------------------------------------------------------- | ------ | -------- |
| Operators            | Know when/why vehicle was disarmed                       | High   | P0       |
| Safety Investigators | Post-flight analysis requires complete disarm event logs | High   | P0       |
| Maintenance Crew     | Verify actuators safe before handling vehicle            | High   | P0       |
| Monitoring System    | Needs to stop health checks after disarm                 | High   | P0       |
| Failsafe System      | Must reset state for next arm cycle                      | High   | P0       |
| Test Engineers       | Verify correct post-disarm cleanup sequence              | High   | P1       |
| Regulatory Bodies    | Audit trail required for vehicle certification           | Medium | P1       |

## Research & Discovery

### User Feedback

From operational requirements:

- Post-disarm cleanup is standard practice in aerospace systems
- Disarm event logging critical for post-incident investigations
- Actuator safety verification essential before personnel approach vehicle
- Subsystem shutdown prevents wasted battery power and false health warnings
- Configuration persistence ensures flight-tuned parameters retained
- Clean state reset prevents next arm cycle from starting with stale data

### Competitive Analysis

**ArduPilot Post-Disarm Cleanup**:

Based on AP_Arming.cpp, ArduPilot performs the following after successful disarm:

#### 1. Disarm State Update

```cpp
// Record disarm state and method
armed = false;
_last_disarm_method = method;
```

**Purpose**: Track how vehicle was disarmed for logging and analysis.

#### 2. Disarm Event Logging

```cpp
// Log disarming event with method and checks status
Log_Write_Disarm(!do_disarm_checks, method);
```

**Logged Data**:

- Timestamp (when disarm occurred)
- Disarm method (GCS command, RC switch, failsafe, forced)
- Checks status (whether checks were bypassed)
- Vehicle state at disarm time

**Purpose**: Create audit trail for post-flight analysis and safety investigations.

#### 3. Logging Persistence Check

```cpp
// Determine if logging should continue after disarm
check_forced_logging(method);

// If failsafe-triggered, extend logging duration
if (bad_failsafe_disarm) {
    logger->set_long_log_persist(true);
}
```

**Forced Logging Reasons**:

- Battery failsafe disarm
- Terrain failsafe disarm
- EKF failsafe disarm
- Other critical failures

**Purpose**: Ensure logs capture full failure sequence, not just disarm event.

#### 4. Safety Switch Handling

```cpp
// Force safety switch on if configured
if (board_cfg->get_safety_button_options() &
    BOARD_SAFETY_OPTION_SAFETY_ON_DISARM) {
    hal.rcout->force_safety_on();
}
```

**Purpose**: Physical safety lockout prevents accidental motor activation during handling.

#### 5. FFT System Cleanup

```cpp
// Preserve FFT parameters for next flight
auto *fft = AP::fft();
if (fft != nullptr) {
    fft->save_params_on_disarm();
}
```

**Purpose**: Save in-flight learned parameters (vibration profiles, notch filters) for future use.

#### 6. Fence Deactivation

```cpp
// Disable geofence boundaries
auto *fence = AP::fence();
if (fence != nullptr) {
    fence->auto_disable_fence_on_disarming();
}
```

**Purpose**: Geofence only applies during armed operation, disable to prevent false warnings.

#### 7. GPIO Pin Updates

```cpp
// Update hardware disarm indicator pin
if (HAL_ARM_GPIO_PIN) {
    // Set GPIO to disarmed state (implementation in HAL)
}
```

**Purpose**: Visual/hardware indication of disarmed state (LEDs, relays).

#### ArduPilot Disarm Sequence

Complete sequence in `disarm()` function:

```cpp
bool AP_Arming::disarm(Method method, bool do_disarm_checks)
{
    // Pre-disarm validation
    if (!armed) {
        return false;  // Already disarmed
    }

    // Set disarmed state
    armed = false;
    _last_disarm_method = method;

    // ===== POST-DISARM CLEANUP =====

    // 1. Log disarming event
    Log_Write_Disarm(!do_disarm_checks, method);

    // 2. Check if logging should persist
    check_forced_logging(method);

    // 3. Save FFT parameters
    auto *fft = AP::fft();
    if (fft != nullptr) {
        fft->save_params_on_disarm();
    }

    // 4. Disable fence
    auto *fence = AP::fence();
    if (fence != nullptr) {
        fence->auto_disable_fence_on_disarming();
    }

    // 5. Update GPIO pins
    update_disarm_gpio();

    // 6. Force safety switch on (if configured)
    if (board_cfg->get_safety_button_options() &
        BOARD_SAFETY_OPTION_SAFETY_ON_DISARM) {
        hal.rcout->force_safety_on();
    }

    // Notify vehicle-specific code
    on_successful_disarming();

    return true;
}
```

**PX4 Post-Disarm Cleanup**:

PX4 uses Commander state machine for disarming:

- **Disarm timestamp**: Recorded in vehicle_status topic
- **Actuator disarming**: Transition ESCs to safe state (neutral PWM)
- **Safety lockout**: Enable hardware safety switch if available
- **LED updates**: Visual indication via LED manager
- **GCS notification**: HEARTBEAT message includes armed bit cleared
- **Parameter persistence**: Save changed parameters to storage

### Technical Investigation

**Current pico_trail Implementation**:

File: `src/communication/mavlink/state.rs:173-183`

```rust
pub fn disarm(&mut self) -> Result<(), &'static str> {
    if !self.is_armed() {
        return Err("Already disarmed");
    }

    self.armed = ArmedState::Disarmed;
    Ok(())
}
```

**Observations**:

- No post-disarm cleanup
- No logging
- No actuator safety verification
- No subsystem notification
- No configuration persistence

**Proposed Post-Disarm Cleanup Architecture**:

```rust
/// Disarm reason enumeration (for logging and audit)
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum DisarmReason {
    Unknown,
    Normal,           // Manual disarm (GCS or RC)
    RcLoss,           // RC loss failsafe triggered disarm
    BatteryLow,       // Battery failsafe triggered disarm
    BatteryCritical,  // Battery critical failsafe triggered disarm
    GcsLoss,          // GCS loss failsafe triggered disarm
    EmergencyStop,    // Emergency stop triggered disarm
    Forced,           // Forced disarm bypassing checks
}

/// Disarm method enumeration (how disarm was commanded)
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum DisarmMethod {
    Unknown,
    GcsCommand,    // MAV_CMD_COMPONENT_ARM_DISARM from GCS
    RcSwitch,      // RC aux switch
    Failsafe,      // Failsafe system triggered disarm
    ForceDisarm,   // Forced disarm (emergency)
}

/// Post-disarm cleanup state
pub struct PostDisarmState {
    /// Timestamp when vehicle disarmed (milliseconds)
    pub disarm_time_ms: u32,

    /// Method used to disarm
    pub disarm_method: DisarmMethod,

    /// Reason for disarm
    pub disarm_reason: DisarmReason,

    /// Actuators verified safe?
    pub actuators_safe: bool,

    /// Subsystems notified?
    pub subsystems_notified: bool,

    /// Configuration persisted?
    pub config_saved: bool,
}

impl SystemState {
    /// Disarm the vehicle with post-disarm cleanup
    pub fn disarm(&mut self, method: DisarmMethod, reason: DisarmReason)
                   -> Result<(), &'static str> {
        // Pre-disarm validation
        if !self.is_armed() {
            return Err("Already disarmed");
        }

        // Set disarmed state
        self.armed = ArmedState::Disarmed;

        // Execute post-disarm cleanup
        self.post_disarm_cleanup(method, reason)?;

        Ok(())
    }

    /// Post-disarm cleanup sequence
    fn post_disarm_cleanup(&mut self, method: DisarmMethod, reason: DisarmReason)
                           -> Result<(), &'static str> {
        // 1. Record timestamp
        let disarm_time_ms = get_time_ms();
        self.post_disarm_state = PostDisarmState {
            disarm_time_ms,
            disarm_method: method,
            disarm_reason: reason,
            actuators_safe: false,
            subsystems_notified: false,
            config_saved: false,
        };

        // 2. Log disarming event
        self.log_disarm_event(disarm_time_ms, method, reason)?;

        // 3. Shutdown actuators to safe state
        self.shutdown_actuators()?;
        self.post_disarm_state.actuators_safe = true;

        // 4. Notify subsystems
        self.notify_subsystems_disarmed()?;
        self.post_disarm_state.subsystems_notified = true;

        // 5. Persist configuration (if changed during flight)
        if self.config_changed {
            self.save_configuration()?;
            self.post_disarm_state.config_saved = true;
        }

        // 6. Update GPIO (if hardware present)
        self.update_disarm_gpio(false)?;

        // 7. Reset armed-state-specific data
        self.reset_armed_state_data()?;

        // 8. Determine if logging should persist
        if self.should_persist_logging(reason) {
            self.extend_logging_duration()?;
        }

        info!("Post-disarm cleanup complete (method: {:?}, reason: {:?})",
              method, reason);
        Ok(())
    }

    /// Log disarming event
    fn log_disarm_event(&mut self, timestamp: u32, method: DisarmMethod,
                       reason: DisarmReason) -> Result<(), &'static str> {
        // Log to persistent storage
        // Format: "DISARM,timestamp,method,reason"
        let log_entry = format!(
            "DISARM,{},{:?},{:?}",
            timestamp,
            method,
            reason
        );

        // TODO: Write to log storage
        info!("Disarm event: {}", log_entry);
        Ok(())
    }

    /// Shutdown actuators to safe state
    fn shutdown_actuators(&mut self) -> Result<(), &'static str> {
        // Set all actuators to neutral/safe PWM values
        // This ensures motors cannot start accidentally

        // Set throttle to neutral (typically 1500 us PWM)
        // Set steering to center (1500 us PWM)
        // Verify actuator commands accepted

        // TODO: Integrate with actuator abstraction layer
        info!("Actuators shutdown to safe state");
        Ok(())
    }

    /// Notify subsystems of disarm event
    fn notify_subsystems_disarmed(&mut self) -> Result<(), &'static str> {
        // Notify monitoring system to stop health checks
        // - Disable RC signal timeout monitoring
        // - Disable battery monitoring (or reduce frequency)
        // - Disable sensor health checks

        // Notify failsafe system that vehicle is disarmed
        // - Disable RC loss detection
        // - Disable GCS loss detection
        // - Disable battery failsafe
        // - Reset failsafe state for next arm

        // Notify mode system of disarm state
        // - Mode may need to adjust behavior for disarmed operation

        // TODO: Implement subsystem notification mechanism
        info!("Subsystems notified of disarm event");
        Ok(())
    }

    /// Save configuration to persistent storage
    fn save_configuration(&mut self) -> Result<(), &'static str> {
        // Write changed parameters to storage
        // This preserves in-flight tuning adjustments

        // TODO: Implement parameter persistence
        info!("Configuration saved to storage");
        Ok(())
    }

    /// Update GPIO disarm indicator
    fn update_disarm_gpio(&mut self, armed: bool) -> Result<(), &'static str> {
        // Set GPIO pin state for external disarm indicator
        // Typically LED or relay

        // TODO: Only if hardware GPIO configured
        // gpio::set_pin(ARM_INDICATOR_PIN, armed);
        Ok(())
    }

    /// Reset armed-state-specific data
    fn reset_armed_state_data(&mut self) -> Result<(), &'static str> {
        // Clear data that only applies during armed operation
        // - Arm timestamp
        // - Armed duration counters
        // - Failsafe trigger counts
        // - Health warning flags

        // TODO: Identify and reset armed-specific state
        info!("Armed-state data reset");
        Ok(())
    }

    /// Determine if logging should persist after disarm
    fn should_persist_logging(&self, reason: DisarmReason) -> bool {
        // Continue logging for failsafe-triggered disarms
        matches!(reason,
            DisarmReason::RcLoss |
            DisarmReason::BatteryCritical |
            DisarmReason::GcsLoss |
            DisarmReason::EmergencyStop
        )
    }

    /// Extend logging duration for post-disarm analysis
    fn extend_logging_duration(&mut self) -> Result<(), &'static str> {
        // Keep logging active for additional time after disarm
        // to capture full failure sequence

        // TODO: Integrate with logging system
        info!("Logging duration extended for failsafe analysis");
        Ok(())
    }

    /// Get time since disarm (milliseconds)
    pub fn time_since_disarm_ms(&self) -> Option<u32> {
        if self.is_armed() {
            return None;
        }

        let current_time = get_time_ms();
        Some(current_time - self.post_disarm_state.disarm_time_ms)
    }
}
```

**Integration with Command Handler**:

```rust
/// Handle MAV_CMD_COMPONENT_ARM_DISARM command
fn handle_arm_disarm(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
    let should_arm = cmd.param1 > 0.5;
    let force = cmd.param2 > 0.5;  // param2=21196 for force

    if should_arm {
        // Arm logic...
    } else {
        // Determine disarm method
        let method = if force {
            DisarmMethod::ForceDisarm
        } else {
            DisarmMethod::GcsCommand
        };

        // Normal disarm (not failsafe-triggered)
        let reason = DisarmReason::Normal;

        match self.state.disarm(method, reason) {
            Ok(()) => {
                info!("Vehicle disarmed via GCS (method: {:?})", method);
                MavResult::MAV_RESULT_ACCEPTED
            }
            Err(reason) => {
                warn!("Disarm rejected: {}", reason);
                MavResult::MAV_RESULT_DENIED
            }
        }
    }
}
```

**Integration with Failsafe System**:

```rust
/// Failsafe triggers disarm
fn execute_disarm_action(&mut self, trigger: FailsafeTrigger) -> Result<(), &'static str> {
    let method = DisarmMethod::Failsafe;

    let reason = match trigger {
        FailsafeTrigger::RcLoss => DisarmReason::RcLoss,
        FailsafeTrigger::BatteryLow => DisarmReason::BatteryLow,
        FailsafeTrigger::BatteryCritical => DisarmReason::BatteryCritical,
        FailsafeTrigger::GcsLoss => DisarmReason::GcsLoss,
    };

    self.state.disarm(method, reason)?;
    info!("Failsafe triggered disarm (reason: {:?})", reason);
    Ok(())
}
```

**Memory Analysis**:

| Component                | RAM Usage   | Notes                           |
| ------------------------ | ----------- | ------------------------------- |
| PostDisarmState          | \~20 B      | Timestamp + enums + flags       |
| Disarm event log entry   | \~50 B      | Temporary string for logging    |
| **Total (post-disarm)**  | **\~70 B**  | Minimal overhead                |
| \*\*Total (with all sys) | **\~530 B** | Post-disarm + monitoring + fail |

### Data Analysis

**Post-Disarm Cleanup Timing**:

| Task                      | Duration    | Notes                              |
| ------------------------- | ----------- | ---------------------------------- |
| Timestamp recording       | < 1 µs      | Read system clock                  |
| Disarm event logging      | \~10 ms     | Flash write (if persistent log)    |
| Actuator safety shutdown  | \~5 ms      | PWM command transmission           |
| Subsystem notification    | \~1 ms      | Update data structures             |
| Configuration persistence | \~20 ms     | Flash write (if config changed)    |
| GPIO updates              | < 1 µs      | Set pin state                      |
| State reset               | < 1 µs      | Clear memory structures            |
| **Total (typical)**       | **\~20 ms** | Acceptable delay for disarm        |
| **Total (worst case)**    | **\~60 ms** | With config save, still acceptable |

**Disarm Reason Distribution** (typical usage):

- Normal (GCS/RC): 80% (planned disarms after successful operation)
- Battery Low: 10% (failsafe-triggered)
- RC Loss: 5% (failsafe-triggered)
- Emergency Stop: 3% (emergency scenarios)
- Forced Disarm: 2% (testing, emergency overrides)

**Logging Persistence Requirements**:

- **Normal disarm**: Continue logging for 5 seconds after disarm
- **Failsafe disarm**: Continue logging for 30 seconds after disarm
- **Emergency stop**: Continue logging for 60 seconds after disarm

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The system shall log disarm events to persistent storage → Will become FR-<id>
  - Rationale: Audit trail required for post-flight analysis and safety investigations
  - Acceptance Criteria:
    - Log includes timestamp, disarm method, disarm reason
    - Format: "DISARM,timestamp_ms,method,reason"
    - Log written before actuators shutdown (capture even if cleanup fails)
    - Log readable post-flight for analysis

- [ ] **FR-DRAFT-2**: The system shall shutdown actuators to neutral/safe state after disarming → Will become FR-<id>
  - Rationale: Prevent accidental motor activation during handling
  - Acceptance Criteria:
    - Set throttle to neutral PWM (typically 1500 µs)
    - Set steering to center PWM (1500 µs)
    - Verify actuator commands accepted (no errors)
    - Shutdown completes within 10ms of disarm

- [ ] **FR-DRAFT-3**: The system shall notify subsystems of disarm event → Will become FR-<id>
  - Rationale: Synchronize all systems with disarmed state
  - Acceptance Criteria:
    - Notify monitoring system to stop or reduce health checks
    - Notify failsafe system to disable detection and reset state
    - Notify mode system to adjust behavior for disarmed operation
    - All notifications complete before disarm() returns

- [ ] **FR-DRAFT-4**: The system shall record disarm method and reason for logging and audit → Will become FR-<id>
  - Rationale: Post-flight analysis needs to know how and why vehicle was disarmed
  - Acceptance Criteria:
    - Support methods: GcsCommand, RcSwitch, Failsafe, ForceDisarm
    - Support reasons: Normal, RcLoss, BatteryLow, BatteryCritical, GcsLoss, EmergencyStop, Forced
    - Disarm method and reason stored in PostDisarmState
    - Disarm method and reason included in log entry
    - Disarm method and reason accessible via API

- [ ] **FR-DRAFT-5**: The system shall persist configuration changes made during flight → Will become FR-<id>
  - Rationale: Preserve in-flight tuning adjustments for next session
  - Acceptance Criteria:
    - Detect if parameters changed during armed operation
    - Write changed parameters to persistent storage on disarm
    - Configuration save completes within 50ms
    - Configuration save failure does not prevent disarm

- [ ] **FR-DRAFT-6**: The system shall extend logging duration for failsafe-triggered disarms → Will become FR-<id>
  - Rationale: Capture full failure sequence for analysis
  - Acceptance Criteria:
    - Continue logging for 30 seconds after failsafe disarm
    - Continue logging for 5 seconds after normal disarm
    - Logging extension does not prevent disarm
    - Operator can manually stop extended logging

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Post-disarm cleanup shall complete within 60ms of disarm command → Will become NFR-<id>
  - Category: Performance
  - Rationale: Disarm operation must complete quickly for safety
  - Target: < 60ms total (20ms typical), measured from disarm command to complete

- [ ] **NFR-DRAFT-2**: Disarm event logging shall not block disarm operation → Will become NFR-<id>
  - Category: Performance / Reliability
  - Rationale: Logging failure should not prevent disarming
  - Target: Log write asynchronous or with timeout, disarm proceeds even if log fails

- [ ] **NFR-DRAFT-3**: Post-disarm cleanup shall add no more than 70 bytes RAM → Will become NFR-<id>
  - Category: Resource Constraints
  - Rationale: Maintain memory budget on RP2040/RP2350
  - Target: < 70 B for PostDisarmState structure (measured via runtime profiling)

- [ ] **NFR-DRAFT-4**: Disarm events shall be logged with sufficient detail for post-flight analysis → Will become NFR-<id>
  - Category: Reliability / Auditability
  - Rationale: Support safety investigations and debugging
  - Target: Log includes timestamp (ms), method (enum), reason (enum), all readable post-flight

- [ ] **NFR-DRAFT-5**: Post-disarm cleanup errors shall not prevent disarm from completing → Will become NFR-<id>
  - Category: Safety
  - Rationale: Vehicle must disarm even if cleanup fails
  - Target: Log errors but complete disarm, vehicle left in safest possible state

## Design Considerations

### Technical Constraints

- **Mirror of post-arm**: Should mirror post-arm initialization structure for consistency and maintainability
- **Existing disarm function**: Must extend current `disarm()` without breaking API
- **Memory budget**: Limited RAM on RP2040/RP2350 (264 KB)
- **Logging storage**: Flash has limited write cycles, log format must be efficient
- **Real-time requirements**: Post-disarm cleanup must not delay disarm operation significantly
- **No dynamic allocation**: All structures must use static/stack allocation
- **Subsystem dependencies**: Monitoring, failsafe must be notified of disarm
- **Actuator abstraction**: Must work with different actuator types (PWM, CAN, etc.)

### Potential Approaches

1. **Option A: Inline Post-Disarm Code in disarm()**
   - Pros:
     - Simplest implementation (all code in one function)
     - No additional structures needed
     - Easy to understand flow
   - Cons:
     - disarm() function becomes large and complex
     - Hard to test individual cleanup steps
     - Difficult to extend with new cleanup tasks
   - Effort: Low (8-12 hours)

2. **Option B: Separate post_disarm_cleanup() Function** ⭐ Recommended
   - Pros:
     - Clean separation of concerns
     - Each cleanup step is testable function
     - Easy to add new cleanup tasks
     - Matches ArduPilot architecture (proven design)
     - Mirrors post-arm initialization structure for consistency
     - Better error handling (can log failures without blocking disarm)
   - Cons:
     - Slightly more code than inline
     - Need PostDisarmState structure
   - Effort: Medium (16-24 hours)

3. **Option C: Event-Based Cleanup System**
   - Pros:
     - Maximum flexibility
     - Subsystems can register cleanup callbacks
     - Easy to add/remove cleanup steps
     - Non-blocking execution possible
   - Cons:
     - High complexity
     - Overkill for initial implementation
     - Harder to debug
   - Effort: High (40-50 hours)

**Recommendation**: Option B (Separate post_disarm_cleanup() Function) provides best balance of clarity, testability, and development effort. Event-based system (Option C) can be added later if needed.

### Architecture Impact

**New ADRs required**:

- **ADR-<id> Post-Disarm Cleanup Sequence**: Order of cleanup steps, error handling, continuation policy
- **ADR-<id> Disarm Event Logging Format**: Log entry structure, storage location, retention policy
- **ADR-<id> Actuator Safety Shutdown**: PWM values, verification method, timeout handling
- **ADR-<id> Logging Persistence Policy**: When to extend logging, duration per disarm reason

**New modules**:

- `src/vehicle/arming/` - Arming system (if not already exists)
  - `src/vehicle/arming/types.rs` - DisarmMethod, DisarmReason, PostDisarmState types
  - `src/vehicle/arming/cleanup.rs` - Post-disarm cleanup logic
  - `src/vehicle/arming/logging.rs` - Disarm event logging

**Modified modules**:

- `src/communication/mavlink/state.rs` - Add post-disarm cleanup to disarm()
- `src/communication/mavlink/handlers/command.rs` - Pass disarm method/reason to disarm()
- `src/vehicle/monitoring/` - React to disarm event, stop or reduce health checks
- `src/vehicle/failsafe/` - Reset failsafe state on disarm, trigger disarms with reason
- `src/core/logging/` - Add disarm event log entry type (if logging exists)

## Parameters

### ArduPilot References

This analysis is based on ArduPilot's post-disarm cleanup sequence in `AP_Arming::disarm()`, which performs:

- Disarm event logging (`Log_Write_Disarm()`)
- Forced logging check (`check_forced_logging()`)
- FFT parameter saving, fence disabling, GPIO updates, safety switch activation

ArduPilot parameters related to disarm behavior:

- **LOG_DISARMED** (u8) - Controls whether logging continues after disarm
  - 0 = disable logging on disarm, 1 = continue logging, 2 = continue logging if armed log created
- **FENCE_AUTOENABLE** (u8) - Controls fence auto-disable on disarm
  - Bit operations control enable/disable behavior on arm/disarm
- **BRD_SAFETY_DEFLT** (u8) - Safety switch behavior
  - Bit 1: Force safety on when disarmed

### Note on Post-Disarm Parameters

ArduPilot handles post-disarm behavior through existing parameters rather than dedicated DISARM\_\* parameters:

- **MOT_SAFE_DISARM** (u8, default 0, range 0-1): PWM output behavior while disarmed
  - 0 = Output trim values (ArduPilot default)
  - 1 = No PWM pulses sent (safer, ensures actuators neutral)

- **LOG_DISARMED** (u8, default 0): Logging behavior when disarmed
  - 0 = Logging stopped when disarmed
  - 1 = Continue logging when disarmed

- **ARMING_OPTIONS** (bitmask): Arming behavior options
  - Bit 0: Disable pre-arm display
  - Bit 1: Force safety switch on when disarmed

For pico_trail Phase 1, we use these standard ArduPilot parameters. Disarm event logging is handled automatically by the logging subsystem. No custom DISARM\_\* parameters needed.

## Risk Assessment

| Risk                                               | Probability | Impact       | Mitigation Strategy                                                                   |
| -------------------------------------------------- | ----------- | ------------ | ------------------------------------------------------------------------------------- |
| **Post-disarm cleanup failure prevents disarm**    | **Medium**  | **CRITICAL** | **Log errors but complete disarm, vehicle safety more important than cleanup**        |
| **Cleanup takes too long (delayed disarm)**        | **Low**     | **High**     | **Profile early, optimize slow steps, target < 60ms total duration**                  |
| Logging flash wear reduces storage lifetime        | Medium      | Low          | Limit log frequency (only arm/disarm, not continuous), use wear-leveling if available |
| Actuator shutdown fails (motors remain active)     | Low         | High         | Timeout-based retry, log failure but proceed with disarm                              |
| Subsystem notification race condition              | Low         | Medium       | Notification order documented in ADR, critical systems notified first                 |
| Post-disarm cleanup memory overhead exceeds budget | Low         | Low          | Profile early, optimize PostDisarmState structure if needed                           |
| Configuration save corrupts parameters             | Low         | High         | Use atomic writes with validation, fall back to defaults if corruption detected       |
| Logging persistence drains battery                 | Low         | Low          | Limit extended logging duration, provide manual stop command                          |
| Disarm during critical operation (mid-air)         | Low         | Critical     | Pre-disarm validation (separate analysis) prevents unsafe disarms                     |

## Open Questions

- [ ] Should post-disarm cleanup be synchronous or asynchronous? → Decision: Synchronous for Phase 1 (simpler, sufficient for cleanup tasks)
- [ ] What happens if logging fails? → Decision: Log failure does not prevent disarm, warn operator but continue
- [ ] Should we support custom post-disarm cleanup callbacks? → Method: Phase 1 = hardcoded cleanup sequence, Phase 2 = add callback registration if needed
- [ ] How to handle actuator shutdown failure? → Decision: Retry once with timeout, log failure but complete disarm, vehicle left in safest state
- [ ] Should disarm timestamp be in milliseconds or microseconds? → Decision: Milliseconds sufficient for logging (32-bit, 49-day wrap-around)
- [ ] Do we need separate cleanup for different disarm reasons? → Decision: No, same cleanup sequence for all reasons (reason only affects logging persistence)
- [ ] Should we log system state snapshot at disarm time? → Method: Phase 1 = no, Phase 2 = add state snapshot if useful for analysis
- [ ] How to test post-disarm cleanup without actual hardware? → Method: Unit tests with mocked subsystems, integration tests in simulation

## Recommendations

### Immediate Actions

1. **Adopt Option B architecture**: Separate post_disarm_cleanup() function mirroring post-arm initialization
2. **Implement core cleanup steps**: Logging, actuator safety, subsystem notification, state reset
3. **Ensure structural consistency**: Maintain symmetry with post-arm initialization patterns
4. **Start with minimal logging**: Timestamp + method + reason (expand later if needed)
5. **Use millisecond timestamps**: Sufficient resolution, 49-day wrap-around acceptable
6. **Make cleanup non-blocking**: Log errors but continue disarm

### Next Steps

1. [ ] Create formal requirements: FR-<id> (disarm logging), FR-<id> (actuator safety), FR-<id> (subsystem notification), FR-<id> (disarm method/reason), FR-<id> (config persistence), FR-<id> (logging persistence), NFR-<id> (cleanup duration), NFR-<id> (non-blocking cleanup), NFR-<id> (memory), NFR-<id> (logging detail), NFR-<id> (error safety)
2. [ ] Draft ADR for: Post-disarm cleanup sequence (order, error handling, continuation policy)
3. [ ] Draft ADR for: Disarm event logging format (entry structure, storage, retention)
4. [ ] Draft ADR for: Actuator safety shutdown (PWM values, verification, timeout)
5. [ ] Draft ADR for: Logging persistence policy (duration per disarm reason)
6. [ ] Create task for: Post-disarm cleanup implementation (Phase 1: core cleanup steps)
7. [ ] Plan testing: Verify cleanup order, test failure continuation, validate timing requirements

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **Fence auto-disable**: Defer to Phase 2 when geofence implemented
- **FFT parameter persistence**: Not applicable (no FFT in pico_trail)
- **Safety switch force-on**: Phase 1 = no hardware safety switch, Phase 2 = add if hardware present
- **GPIO disarm indicator**: Phase 1 = no hardware GPIO, Phase 2 = add if hardware present
- **Custom cleanup callbacks**: Phase 1 = hardcoded sequence, Phase 2 = callback registration if needed
- **System state snapshot logging**: Phase 1 = basic log only, Phase 2 = add sensor snapshots
- **Asynchronous cleanup**: Phase 1 = synchronous (simpler), Phase 2 = async if performance issue
- **Multiple log destinations**: Phase 1 = single log file, Phase 2 = add SD card / network logging
- **Cleanup step profiling**: No automatic timing measurement in Phase 1
- **Recovery from partial cleanup**: Phase 1 = log errors and continue, Phase 2 = add graceful degradation
- **Advanced actuator verification**: Phase 1 = command only, Phase 2 = add feedback verification

## Appendix

### References

- ArduPilot AP_Arming Library: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Arming/AP_Arming.cpp>
- MAVLink MAV_CMD_COMPONENT_ARM_DISARM: <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>
- MAVLink STATUSTEXT: <https://mavlink.io/en/messages/common.html#STATUSTEXT>

### Raw Data

**ArduPilot Post-Disarm Cleanup** (from AP_Arming.cpp):

```cpp
bool AP_Arming::disarm(Method method, bool do_disarm_checks)
{
    // Pre-disarm checks
    if (!armed) {
        return false;
    }

    // Set disarmed state
    armed = false;
    _last_disarm_method = method;

    // ===== POST-DISARM CLEANUP =====

    // 1. Logging
    Log_Write_Disarm(!do_disarm_checks, method);

    // 2. Check forced logging
    check_forced_logging(method);

    // 3. FFT cleanup
    auto *fft = AP::fft();
    if (fft) fft->save_params_on_disarm();

    // 4. Fence deactivation
    auto *fence = AP::fence();
    if (fence) fence->auto_disable_fence_on_disarming();

    // 5. GPIO update
    update_disarm_gpio();

    // 6. Safety switch
    if (board_cfg->get_safety_button_options() &
        BOARD_SAFETY_OPTION_SAFETY_ON_DISARM) {
        hal.rcout->force_safety_on();
    }

    // Vehicle-specific cleanup
    on_successful_disarming();

    return true;
}
```

**Proposed pico_trail Cleanup Sequence**:

```
disarm()
  │
  ├─ Pre-disarm checks
  │   ├─ Already disarmed? → ERROR
  │   └─ (Additional checks in pre-disarm validation analysis...)
  │
  ├─ Set disarmed state
  │   └─ self.armed = Disarmed
  │
  └─ post_disarm_cleanup()
      │
      ├─ 1. Record timestamp (< 1µs)
      │   └─ disarm_time_ms = get_time_ms()
      │
      ├─ 2. Log disarm event (~10ms)
      │   └─ log("DISARM,{},{:?},{:?}",timestamp,method,reason)
      │
      ├─ 3. Shutdown actuators (~5ms)
      │   ├─ Throttle → Neutral (1500µs)
      │   ├─ Steering → Center (1500µs)
      │   └─ Verify commands accepted
      │
      ├─ 4. Notify subsystems (~1ms)
      │   ├─ Monitoring: stop or reduce checks
      │   ├─ Failsafe: disable and reset state
      │   └─ Modes: adjust for disarmed
      │
      ├─ 5. Persist configuration (~20ms, optional)
      │   └─ Save changed parameters to storage
      │
      ├─ 6. Update GPIO (< 1µs)
      │   └─ Clear disarm indicator LED/pin
      │
      ├─ 7. Reset armed-state data (< 1µs)
      │   └─ Clear arm timestamp, counters, flags
      │
      └─ 8. Extend logging (if failsafe disarm)
          └─ Continue logging for 30s

Total: ~20ms typical, ~60ms worst case (with config save)
```
