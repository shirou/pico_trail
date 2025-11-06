# AN-m4dpl Post-Arm Initialization for Armed State Preparation

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-r2fps-pre-arm-checks](AN-r2fps-pre-arm-checks.md)
  - [AN-kajh6-failsafe-system](AN-kajh6-failsafe-system.md)
  - [AN-dgpck-armed-state-monitoring](AN-dgpck-armed-state-monitoring.md)
- Related Requirements:
  - [FR-b22fh-armed-state-reset](../requirements/FR-b22fh-armed-state-reset.md)
  - [FR-c6ej0-arm-subsystem-notification](../requirements/FR-c6ej0-arm-subsystem-notification.md)
  - [FR-fusl9-arming-checks-warning](../requirements/FR-fusl9-arming-checks-warning.md)
  - [FR-96ok4-post-arm-event-recording](../requirements/FR-96ok4-post-arm-event-recording.md)
  - [FR-xeo4y-actuator-armed-initialization](../requirements/FR-xeo4y-actuator-armed-initialization.md)
  - [NFR-25i4f-arm-logging-detail](../requirements/NFR-25i4f-arm-logging-detail.md)
  - [NFR-35u09-post-arm-memory](../requirements/NFR-35u09-post-arm-memory.md)
  - [NFR-9ko79-post-arm-performance](../requirements/NFR-9ko79-post-arm-performance.md)
  - [NFR-9s8i3-post-arm-safety](../requirements/NFR-9s8i3-post-arm-safety.md)
  - [NFR-aa1z4-arm-logging-non-blocking](../requirements/NFR-aa1z4-arm-logging-non-blocking.md)
- Related ADRs:
  - [ADR-w8d02-arming-system-architecture](../adr/ADR-w8d02-arming-system-architecture.md)
- Related Tasks:
  - [T-zmv9u-arming-system-implementation](../tasks/T-zmv9u-arming-system-implementation/README.md)

## Executive Summary

This analysis explores post-arm initialization procedures needed to properly prepare the vehicle for armed operation. Currently, pico_trail simply changes the armed state to `Armed` with no subsystem initialization, logging, or state synchronization. Post-arm initialization is critical for establishing a proper operational baseline - logging arm events for audit trails, recording timestamps for time-based logic, enabling geofences for safety boundaries, initializing actuators for ready state, and synchronizing all subsystems with the armed condition.

Key findings: ArduPilot implements comprehensive post-arm initialization including arm event logging, timestamp recording (`last_arm_time_us`), fence auto-enable, GPIO pin updates, FFT subsystem preparation, terrain reference setup, and warning notifications if checks disabled. For pico_trail, a similar initialization sequence is recommended with focus on logging (arm event with method and timestamp), timestamp recording (for timeout tracking and duration calculations), actuator initialization (transition from neutral to ready state), and subsystem synchronization (notify all systems of armed state).

## Problem Space

### Current State

The project currently has:

- **Arm function**: `SystemState::arm()` in `src/communication/mavlink/state.rs:157-171`
- **Minimal arm logic**: Only checks battery and sets armed state
- **No post-arm initialization**: No subsystem preparation after arming
- **No logging**: Arm events not recorded
- **No timestamp recording**: No record of when vehicle armed
- **No actuator initialization**: Actuators not transitioned to ready state

Critical gaps:

- **No audit trail**: Cannot determine when/how vehicle was armed post-flight
- **No time-based logic**: Cannot calculate armed duration, time since arm
- **No subsystem synchronization**: Failsafe, monitoring, modes unaware of arm event
- **No actuator preparation**: Actuators may not be in correct initial state
- **No fence activation**: Geofence not enabled on arm (when implemented)
- **No operational baseline**: No known-good starting state for armed operation

### Desired State

Enable comprehensive post-arm initialization establishing operational baseline:

1. **Arm Event Logging**: Record arm events with timestamp, method, checks status
2. **Timestamp Recording**: Store arm time for duration tracking and time-based logic
3. **Actuator Initialization**: Transition actuators from neutral to ready/armed state
4. **Subsystem Notification**: Notify failsafe, monitoring, modes of arm event
5. **Fence Activation**: Auto-enable geofence boundaries on arm (when fence implemented)
6. **GPIO Updates**: Update arm indicator LED/pins if hardware present
7. **Warning Notifications**: Alert operator if arming checks were disabled

Success criteria:

- **Complete audit trail**: All arm events logged with full context
- **Timestamp available**: Arm time accessible for timeout and duration calculations
- **Subsystem awareness**: All systems synchronized with armed state
- **Actuator readiness**: Actuators in correct initial state for armed operation
- **Safety boundaries active**: Fence enabled automatically on arm
- **Operator awareness**: Warnings issued for degraded arming conditions

### Gap Analysis

**Missing components**:

1. **Arm Event Logger**: Log arm events to persistent storage
2. **Timestamp Tracker**: Record and expose arm timestamp
3. **Actuator Initializer**: Set actuators to armed state
4. **Subsystem Notifier**: Broadcast arm event to other systems
5. **Fence Auto-Enabler**: Activate geofence on arm (future)
6. **GPIO Updater**: Set hardware arm indicator pins (if present)
7. **Warning System**: Issue notifications for degraded arm conditions

**Technical deltas**:

- Add `post_arm_init()` function called after `arm()` succeeds
- Implement arm event logging with timestamp and method
- Store `last_arm_time_ms` in `SystemState`
- Add actuator initialization sequence (set PWM to armed state)
- Create subsystem notification mechanism (publish arm event)
- Add logging for arm method (RC rudder, GCS command, force arm)
- Integrate with monitoring system to reset health baselines
- Add warning generation if ARMING_CHECK disabled
- Prepare for future fence auto-enable integration

## Stakeholder Analysis

| Stakeholder          | Interest/Need                                         | Impact | Priority |
| -------------------- | ----------------------------------------------------- | ------ | -------- |
| Operators            | Know when vehicle was armed, how it was armed         | High   | P0       |
| Safety Investigators | Post-flight analysis requires complete arm event logs | High   | P0       |
| Monitoring System    | Needs arm timestamp for timeout tracking              | High   | P0       |
| Failsafe System      | Depends on arm time for RC/GCS loss detection         | High   | P0       |
| Test Engineers       | Verify correct post-arm initialization sequence       | High   | P1       |
| Regulatory Bodies    | Audit trail required for vehicle certification        | Medium | P1       |

## Research & Discovery

### User Feedback

From operational requirements:

- Post-arm initialization is standard practice in aerospace systems
- Arm event logging critical for post-incident investigations
- Timestamp recording enables accurate timeout detection
- Subsystem synchronization prevents race conditions and state inconsistencies
- Actuator initialization ensures known starting point for control loops
- Warning notifications help operators identify degraded safety conditions

### Competitive Analysis

**ArduPilot Post-Arm Initialization**:

Based on AP_Arming.cpp, ArduPilot performs the following after successful arm:

#### 1. Timestamp Recording

```cpp
// Record arm time for duration tracking
last_arm_time_us = AP_HAL::micros64();

// Store arm method for logging
_last_arm_method = method;
```

**Purpose**: Enables calculation of:

- Armed duration (for logging, limits)
- Time since arm (for timeout detection)
- Flight time tracking

#### 2. Arm Event Logging

```cpp
// Log arming event with method and checks status
Log_Write_Arm(!do_arming_checks, method);

// Transition logger to active state
logger->transition_to_active_logging();
```

**Logged Data**:

- Timestamp (when arm occurred)
- Arming method (rudder, GCS, switch, forced)
- Checks status (enabled/disabled)
- Vehicle state at arm time

#### 3. FFT Subsystem Preparation

```cpp
// Prepare frequency analysis system
auto *fft = AP::fft();
if (fft != nullptr) {
    fft->prepare_for_arming();
}
```

**Purpose**: Initializes gyro FFT analysis for in-flight vibration/harmonic monitoring.

#### 4. Terrain Reference Setup

```cpp
// Establish terrain reference location
auto *terrain = AP::terrain();
if (terrain != nullptr) {
    terrain->set_reference_location();
}
```

**Purpose**: Enables terrain-relative altitude calculations during mission.

#### 5. Fence Auto-Enable

```cpp
// Activate geofence boundaries
auto *fence = AP::fence();
if (fence != nullptr) {
    fence->auto_enable_fence_on_arming();
}
```

**Purpose**: Ensures safety boundaries active before vehicle moves.

#### 6. GPIO Pin Updates

```cpp
// Update hardware arm indicator pins
update_arm_gpio();
```

**Purpose**: Visual/hardware indication of armed state (LEDs, relays).

#### 7. Warning Notifications

```cpp
// Alert operator if checks were disabled
if (!do_arming_checks) {
    gcs().send_text(MAV_SEVERITY_WARNING,
                    "Warning: Arming Checks Disabled");
}
```

**Purpose**: Operator awareness of reduced safety verification.

#### ArduPilot Arm Sequence

Complete sequence in `arm()` function:

```cpp
bool AP_Arming::arm(Method method, bool do_arming_checks)
{
    // Pre-arm checks
    if (do_arming_checks) {
        if (!all_checks_passing()) {
            return false;  // Checks failed
        }
    }

    // Set armed state
    hal.util->set_soft_armed(true);

    // ===== POST-ARM INITIALIZATION =====

    // 1. Record timestamp
    last_arm_time_us = AP_HAL::micros64();
    _last_arm_method = method;

    // 2. Log arming event
    Log_Write_Arm(!do_arming_checks, method);
    AP::logger()->transition_to_active_logging();

    // 3. Prepare FFT
    auto *fft = AP::fft();
    if (fft != nullptr) {
        fft->prepare_for_arming();
    }

    // 4. Setup terrain reference
    auto *terrain = AP::terrain();
    if (terrain != nullptr) {
        terrain->set_reference_location();
    }

    // 5. Enable fence
    auto *fence = AP::fence();
    if (fence != nullptr) {
        fence->auto_enable_fence_on_arming();
    }

    // 6. Update GPIO pins
    update_arm_gpio();

    // 7. Send warnings
    if (!do_arming_checks) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "Warning: Arming Checks Disabled");
    }

    // Notify vehicle-specific code
    on_successful_arming();

    return true;
}
```

**PX4 Post-Arm Initialization**:

PX4 uses Commander state machine for arming:

- **Arming timestamp**: Recorded in vehicle_status topic
- **Safety checks logging**: All checks logged before arm
- **Actuator arming**: Transition ESCs to armed state
- **Failsafe reset**: Clear previous failsafe states on arm
- **LED updates**: Visual indication via LED manager
- **GCS notification**: HEARTBEAT message includes armed bit

### Technical Investigation

**Current pico_trail Implementation**:

File: `src/communication/mavlink/state.rs:157-171`

```rust
pub fn arm(&mut self) -> Result<(), &'static str> {
    if self.is_armed() {
        return Err("Already armed");
    }

    if self.battery.is_critical() {
        return Err("Battery voltage too low");
    }

    self.armed = ArmedState::Armed;  // Only state change
    Ok(())
}
```

**Observations**:

- No post-arm initialization
- No logging
- No timestamp recording
- No subsystem notification
- No actuator preparation

**Proposed Post-Arm Initialization Architecture**:

```rust
/// Arm method enumeration (for logging and audit)
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum ArmMethod {
    Unknown,
    RcRudder,      // RC stick pattern (throttle down + rudder right)
    GcsCommand,    // MAV_CMD_COMPONENT_ARM_DISARM from GCS
    RcSwitch,      // RC aux switch
    ForceArm,      // Forced arm bypassing checks
}

/// Post-arm initialization data
pub struct PostArmState {
    /// Timestamp when vehicle armed (milliseconds)
    pub arm_time_ms: u32,

    /// Method used to arm
    pub arm_method: ArmMethod,

    /// Were arming checks performed?
    pub checks_performed: bool,

    /// Actuators initialized?
    pub actuators_initialized: bool,

    /// Subsystems notified?
    pub subsystems_notified: bool,
}

impl SystemState {
    /// Arm the vehicle with post-arm initialization
    pub fn arm(&mut self, method: ArmMethod, checks_performed: bool)
               -> Result<(), &'static str> {
        // Pre-arm validation
        if self.is_armed() {
            return Err("Already armed");
        }

        if self.battery.is_critical() {
            return Err("Battery voltage too low");
        }

        // Set armed state
        self.armed = ArmedState::Armed;

        // Execute post-arm initialization
        self.post_arm_init(method, checks_performed)?;

        Ok(())
    }

    /// Post-arm initialization sequence
    fn post_arm_init(&mut self, method: ArmMethod, checks_performed: bool)
                     -> Result<(), &'static str> {
        // 1. Record timestamp
        let arm_time_ms = get_time_ms();
        self.post_arm_state = PostArmState {
            arm_time_ms,
            arm_method: method,
            checks_performed,
            actuators_initialized: false,
            subsystems_notified: false,
        };

        // 2. Log arming event
        self.log_arm_event(arm_time_ms, method, checks_performed)?;

        // 3. Initialize actuators
        self.initialize_actuators()?;
        self.post_arm_state.actuators_initialized = true;

        // 4. Notify subsystems
        self.notify_subsystems_armed()?;
        self.post_arm_state.subsystems_notified = true;

        // 5. Update GPIO (if hardware present)
        self.update_arm_gpio(true)?;

        // 6. Send warnings if checks disabled
        if !checks_performed {
            self.send_warning("Arming checks disabled")?;
        }

        info!("Post-arm initialization complete (method: {:?})", method);
        Ok(())
    }

    /// Log arming event
    fn log_arm_event(&mut self, timestamp: u32, method: ArmMethod,
                     checks_performed: bool) -> Result<(), &'static str> {
        // Log to persistent storage
        // Format: "ARM,timestamp,method,checks"
        let log_entry = format!(
            "ARM,{},{:?},{}",
            timestamp,
            method,
            if checks_performed { "1" } else { "0" }
        );

        // TODO: Write to log storage
        info!("Arm event: {}", log_entry);
        Ok(())
    }

    /// Initialize actuators to armed state
    fn initialize_actuators(&mut self) -> Result<(), &'static str> {
        // Transition actuators from neutral to armed/ready state
        // This ensures known starting point for control loops

        // Set throttle to armed idle (typically 1100 us PWM for ESCs)
        // Set steering to center (1500 us PWM)
        // Verify actuator commands accepted

        // TODO: Integrate with actuator abstraction layer
        info!("Actuators initialized to armed state");
        Ok(())
    }

    /// Notify subsystems of arm event
    fn notify_subsystems_armed(&mut self) -> Result<(), &'static str> {
        // Notify monitoring system to reset baselines
        // - Reset RC signal timeout baseline
        // - Reset battery voltage baseline
        // - Reset sensor health baselines

        // Notify failsafe system that vehicle is armed
        // - Enable RC loss detection
        // - Enable GCS loss detection (if configured)
        // - Enable battery failsafe

        // Notify mode system of arm state
        // - Mode may need to adjust behavior for armed operation

        // TODO: Implement subsystem notification mechanism
        info!("Subsystems notified of arm event");
        Ok(())
    }

    /// Update GPIO arm indicator
    fn update_arm_gpio(&mut self, armed: bool) -> Result<(), &'static str> {
        // Set GPIO pin state for external arm indicator
        // Typically LED or relay

        // TODO: Only if hardware GPIO configured
        // gpio::set_pin(ARM_INDICATOR_PIN, armed);
        Ok(())
    }

    /// Send warning message to GCS
    fn send_warning(&self, message: &str) -> Result<(), &'static str> {
        // Send STATUSTEXT message with warning severity
        // TODO: Integrate with MAVLink router
        warn!("{}", message);
        Ok(())
    }

    /// Get time since arm (milliseconds)
    pub fn time_since_arm_ms(&self) -> Option<u32> {
        if !self.is_armed() {
            return None;
        }

        let current_time = get_time_ms();
        Some(current_time - self.post_arm_state.arm_time_ms)
    }

    /// Get armed duration (seconds)
    pub fn armed_duration_s(&self) -> Option<f32> {
        self.time_since_arm_ms().map(|ms| ms as f32 / 1000.0)
    }
}
```

**Integration with Command Handler**:

```rust
/// Handle MAV_CMD_COMPONENT_ARM_DISARM command
fn handle_arm_disarm(&mut self, cmd: &COMMAND_LONG_DATA) -> MavResult {
    let should_arm = cmd.param1 > 0.5;
    let force_arm = cmd.param2 > 0.5;  // param2=21196 for force arm

    if should_arm {
        // Determine arm method
        let method = if force_arm {
            ArmMethod::ForceArm
        } else {
            ArmMethod::GcsCommand
        };

        // Arm with checks (unless force arm)
        let checks_performed = !force_arm;

        match self.state.arm(method, checks_performed) {
            Ok(()) => {
                info!("Vehicle armed via GCS (method: {:?})", method);
                MavResult::MAV_RESULT_ACCEPTED
            }
            Err(reason) => {
                warn!("Arm rejected: {}", reason);
                MavResult::MAV_RESULT_DENIED
            }
        }
    } else {
        // Disarm logic...
    }
}
```

**Memory Analysis**:

| Component                | RAM Usage   | Notes                            |
| ------------------------ | ----------- | -------------------------------- |
| PostArmState             | \~16 B      | Timestamp + enums + flags        |
| Arm event log entry      | \~40 B      | Temporary string for logging     |
| **Total (post-arm)**     | **\~60 B**  | Minimal overhead                 |
| \*\*Total (with all sys) | **\~460 B** | Post-arm + monitoring + failsafe |

### Data Analysis

**Post-Arm Initialization Timing**:

| Task                    | Duration    | Notes                              |
| ----------------------- | ----------- | ---------------------------------- |
| Timestamp recording     | < 1 µs      | Read system clock                  |
| Arm event logging       | \~10 ms     | Flash write (if persistent log)    |
| Actuator initialization | \~5 ms      | PWM command transmission           |
| Subsystem notification  | \~1 ms      | Update data structures             |
| GPIO updates            | < 1 µs      | Set pin state                      |
| **Total (typical)**     | **\~20 ms** | Acceptable delay for arm operation |
| **Total (worst case)**  | **\~50 ms** | If logging slow, still acceptable  |

**Arm Method Distribution** (typical usage):

- GCS Command: 70% (most common in autonomous operations)
- RC Rudder: 20% (manual test flights)
- RC Switch: 5% (convenience feature)
- Force Arm: 5% (testing, emergency scenarios)

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The system shall record arm timestamp when vehicle arms → Will become FR-<id>
  - Rationale: Enable time-based logic, duration tracking, timeout detection
  - Acceptance Criteria:
    - Store arm time in milliseconds (u32)
    - Timestamp accessible via `time_since_arm_ms()` API
    - Timestamp persists until disarm
    - Timestamp used by monitoring system for RC/GCS loss detection

- [ ] **FR-DRAFT-2**: The system shall log arm events to persistent storage → Will become FR-<id>
  - Rationale: Audit trail required for post-flight analysis and safety investigations
  - Acceptance Criteria:
    - Log includes timestamp, arm method, checks status
    - Format: "ARM,timestamp_ms,method,checks_enabled"
    - Log written before actuators initialized (capture even if init fails)
    - Log readable post-flight for analysis

- [ ] **FR-DRAFT-3**: The system shall initialize actuators to armed state after arming → Will become FR-<id>
  - Rationale: Establish known starting point for control loops
  - Acceptance Criteria:
    - Set throttle to armed idle PWM (typically 1100 µs for ESCs)
    - Set steering to center PWM (1500 µs)
    - Verify actuator commands accepted (no errors)
    - Initialization completes within 10ms of arm

- [ ] **FR-DRAFT-4**: The system shall notify subsystems of arm event → Will become FR-<id>
  - Rationale: Synchronize all systems with armed state
  - Acceptance Criteria:
    - Notify monitoring system to reset health baselines
    - Notify failsafe system to enable RC/GCS loss detection
    - Notify mode system to adjust behavior for armed operation
    - All notifications complete before arm() returns

- [ ] **FR-DRAFT-5**: The system shall record arm method for logging and audit → Will become FR-<id>
  - Rationale: Post-flight analysis needs to know how vehicle was armed
  - Acceptance Criteria:
    - Support methods: GcsCommand, RcRudder, RcSwitch, ForceArm
    - Arm method stored in PostArmState
    - Arm method included in log entry
    - Arm method accessible via API

- [ ] **FR-DRAFT-6**: The system shall warn operator if arming checks disabled → Will become FR-<id>
  - Rationale: Operator awareness of reduced safety verification
  - Acceptance Criteria:
    - Send STATUSTEXT warning if checks bypassed
    - Warning severity: MAV_SEVERITY_WARNING
    - Message: "Arming checks disabled"
    - Warning sent immediately after arm

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Post-arm initialization shall complete within 50ms of arm command → Will become NFR-<id>
  - Category: Performance
  - Rationale: Arm operation must feel responsive to operator
  - Target: < 50ms total (20ms typical), measured from arm command to ready state

- [ ] **NFR-DRAFT-2**: Arm event logging shall not block arm operation → Will become NFR-<id>
  - Category: Performance / Reliability
  - Rationale: Logging failure should not prevent arming
  - Target: Log write asynchronous or with timeout, arm proceeds even if log fails

- [ ] **NFR-DRAFT-3**: Post-arm initialization shall add no more than 60 bytes RAM → Will become NFR-<id>
  - Category: Resource Constraints
  - Rationale: Maintain memory budget on RP2040/RP2350
  - Target: < 60 B for PostArmState structure (measured via runtime profiling)

- [ ] **NFR-DRAFT-4**: Arm events shall be logged with sufficient detail for post-flight analysis → Will become NFR-<id>
  - Category: Reliability / Auditability
  - Rationale: Support safety investigations and debugging
  - Target: Log includes timestamp (ms), method (enum), checks status (bool), all readable post-flight

- [ ] **NFR-DRAFT-5**: Post-arm initialization errors shall not leave vehicle in unsafe state → Will become NFR-<id>
  - Category: Safety
  - Rationale: Partial initialization worse than no initialization
  - Target: If any init step fails, disarm vehicle and report error to operator

## Design Considerations

### Technical Constraints

- **Existing arm function**: Must extend current `arm()` without breaking API
- **Memory budget**: Limited RAM on RP2040/RP2350 (264 KB)
- **Logging storage**: Flash has limited write cycles, log format must be efficient
- **Real-time requirements**: Post-arm init must not delay arm operation significantly
- **No dynamic allocation**: All structures must use static/stack allocation
- **Subsystem dependencies**: Monitoring, failsafe must be notified of arm
- **Actuator abstraction**: Must work with different actuator types (PWM, CAN, etc.)

### Potential Approaches

1. **Option A: Inline Post-Arm Code in arm()**
   - Pros:
     - Simplest implementation (all code in one function)
     - No additional structures needed
     - Easy to understand flow
   - Cons:
     - arm() function becomes large and complex
     - Hard to test individual init steps
     - Difficult to extend with new init tasks
   - Effort: Low (8-12 hours)

2. **Option B: Separate post_arm_init() Function** ⭐ Recommended
   - Pros:
     - Clean separation of concerns
     - Each init step is testable function
     - Easy to add new init tasks
     - Matches ArduPilot architecture
     - Better error handling (can rollback on failure)
   - Cons:
     - Slightly more code than inline
     - Need PostArmState structure
   - Effort: Medium (16-24 hours)

3. **Option C: Event-Based Init System**
   - Pros:
     - Maximum flexibility
     - Subsystems can register init callbacks
     - Easy to add/remove init steps
     - Non-blocking execution possible
   - Cons:
     - High complexity
     - Overkill for initial implementation
     - Harder to debug
   - Effort: High (40-50 hours)

**Recommendation**: Option B (Separate post_arm_init() Function) provides best balance of clarity, testability, and development effort. Event-based system (Option C) can be added later if needed.

### Architecture Impact

**New ADRs required**:

- **ADR-<id> Post-Arm Initialization Sequence**: Order of init steps, error handling, rollback policy
- **ADR-<id> Arm Event Logging Format**: Log entry structure, storage location, retention policy
- **ADR-<id> Subsystem Notification Mechanism**: How subsystems register for arm events

**New modules**:

- `src/vehicle/arming/` - Arming system (if not already exists)
  - `src/vehicle/arming/types.rs` - ArmMethod, PostArmState types
  - `src/vehicle/arming/init.rs` - Post-arm initialization logic
  - `src/vehicle/arming/logging.rs` - Arm event logging

**Modified modules**:

- `src/communication/mavlink/state.rs` - Add post-arm init to arm()
- `src/communication/mavlink/handlers/command.rs` - Pass arm method to arm()
- `src/vehicle/monitoring/` - React to arm event, reset baselines
- `src/vehicle/failsafe/` - Enable failsafe detection on arm
- `src/core/logging/` - Add arm event log entry type (if logging exists)

## Parameters

### ArduPilot References

This analysis is based on ArduPilot's post-arm initialization sequence in `AP_Arming::arm()`, which performs:

- Timestamp recording (`last_arm_time_us`)
- Arm event logging (`Log_Write_Arm()`)
- FFT preparation, terrain setup, fence enabling, GPIO updates

ArduPilot does not have specific parameters controlling post-arm initialization - these actions are hardcoded in the arm sequence. However, related parameters exist:

- **ARMING_CHECK** (bitmask) - Controls which pre-arm checks are performed, logged during post-arm init
- **LOG_BITMASK** (bitmask) - Controls logging behavior, affects arm event logging
- Various fence parameters (**FENCE\_**\*) - Control auto-enable behavior on arm

### Note on Post-Arm Parameters

ArduPilot handles post-arm behavior through existing parameters rather than dedicated ARM\_\* parameters:

- **LOG_DISARMED** (u8, default 0): Logging behavior when disarmed
  - 0 = Logging stopped when disarmed
  - 1 = Continue logging when disarmed
  - 2 = Continue if armed log was created
  - Arming automatically starts logging; no separate ARM_LOG_ENABLE needed

- **MOT_SAFE_DISARM** (u8, default 0, range 0-1): PWM output behavior while disarmed
  - 0 = Output trim values (ArduPilot default, typically 1500 µs)
  - 1 = No PWM pulses sent
  - When arming, actuators transition from disarmed state to armed idle automatically

- **ARMING_OPTIONS** (bitmask): Arming behavior options
  - Bit 0: Disable pre-arm display
  - Bit 1: Force safety switch on when disarmed
  - No separate GPIO pin configuration; GPIO control typically handled by BRD\_\* parameters

For pico_trail Phase 1, we follow ArduPilot's approach: arm event logging is handled automatically by the logging subsystem, actuator initialization uses MOT_SAFE_DISARM behavior, and no custom ARM\_\* parameters needed.

## Risk Assessment

| Risk                                                     | Probability | Impact       | Mitigation Strategy                                                                   |
| -------------------------------------------------------- | ----------- | ------------ | ------------------------------------------------------------------------------------- |
| **Post-arm init failure leaves vehicle in unsafe state** | **Medium**  | **CRITICAL** | **Rollback on failure: disarm if any init step fails, report error to operator**      |
| **Init takes too long (operator perceives lag)**         | **Low**     | **Medium**   | **Profile early, optimize slow steps, target < 50ms total duration**                  |
| Logging flash wear reduces storage lifetime              | Medium      | Low          | Limit log frequency (only arm/disarm, not continuous), use wear-leveling if available |
| Subsystem notification race condition                    | Low         | Medium       | Notification order documented in ADR, critical systems notified first                 |
| Actuator initialization fails (hardware not responding)  | Low         | High         | Timeout-based retry, disarm if initialization fails after retry                       |
| Post-arm init memory overhead exceeds budget             | Low         | Low          | Profile early, optimize PostArmState structure if needed                              |
| Timestamp overflow (u32 wraps after 49 days)             | Low         | Low          | Document overflow behavior, use delta calculations that handle wrap-around            |

## Open Questions

- [ ] Should post-arm init be synchronous or asynchronous? → Decision: Synchronous for Phase 1 (simpler, sufficient for init tasks)
- [ ] What happens if logging fails? → Decision: Log failure does not prevent arming, warn operator but continue
- [ ] Should we support custom post-arm init callbacks? → Method: Phase 1 = hardcoded init sequence, Phase 2 = add callback registration if needed
- [ ] How to handle actuator init failure? → Decision: Retry once with timeout, disarm if still fails, report error to GCS
- [ ] Should arm timestamp be in milliseconds or microseconds? → Decision: Milliseconds sufficient for timeout tracking (32-bit, 49-day wrap-around)
- [ ] Do we need separate init for different arm methods? → Decision: No, same init sequence for all methods (method only affects logging)
- [ ] Should we log battery voltage at arm time? → Method: Phase 1 = no, Phase 2 = add battery snapshot to log entry if useful
- [ ] How to test post-arm init without actual hardware? → Method: Unit tests with mocked subsystems, integration tests in SITL/simulation

## Recommendations

### Immediate Actions

1. **Adopt Option B architecture**: Separate post_arm_init() function
2. **Implement core init steps**: Timestamp, logging, subsystem notification
3. **Start with minimal logging**: Timestamp + method + checks (expand later if needed)
4. **Use millisecond timestamps**: Sufficient resolution, 49-day wrap-around acceptable
5. **Make logging non-blocking**: Warn if log fails but continue arming

### Next Steps

1. [ ] Create formal requirements: FR-<id> (timestamp recording), FR-<id> (arm logging), FR-<id> (actuator init), FR-<id> (subsystem notification), FR-<id> (arm method), FR-<id> (warnings), NFR-<id> (init duration), NFR-<id> (non-blocking log), NFR-<id> (memory), NFR-<id> (logging detail), NFR-<id> (error safety)
2. [ ] Draft ADR for: Post-arm initialization sequence (order, error handling, rollback)
3. [ ] Draft ADR for: Arm event logging format (entry structure, storage, retention)
4. [ ] Draft ADR for: Subsystem notification mechanism (how subsystems learn of arm)
5. [ ] Create task for: Post-arm initialization implementation (Phase 1: core init steps)
6. [ ] Plan testing: Verify init order, test failure rollback, validate timing requirements

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **Fence auto-enable**: Defer to Phase 2 when geofence implemented
- **FFT subsystem preparation**: Not applicable (no FFT in pico_trail)
- **Terrain reference setup**: Not applicable (no terrain database)
- **GPIO arm indicator**: Phase 1 = no hardware GPIO, Phase 2 = add if hardware present
- **Custom init callbacks**: Phase 1 = hardcoded sequence, Phase 2 = callback registration if needed
- **Battery snapshot logging**: Phase 1 = basic log only, Phase 2 = add sensor snapshots
- **Asynchronous logging**: Phase 1 = synchronous (simpler), Phase 2 = async if performance issue
- **Multiple log destinations**: Phase 1 = single log file, Phase 2 = add SD card / network logging
- **Init step profiling**: No automatic timing measurement in Phase 1
- **Recovery from partial init**: Phase 1 = disarm on failure, Phase 2 = add graceful degradation

## Appendix

### References

- ArduPilot AP_Arming Library: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Arming/AP_Arming.cpp>
- MAVLink MAV_CMD_COMPONENT_ARM_DISARM: <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>
- MAVLink STATUSTEXT: <https://mavlink.io/en/messages/common.html#STATUSTEXT>

### Raw Data

**ArduPilot Post-Arm Initialization** (conceptual, from AP_Arming.cpp):

```cpp
bool AP_Arming::arm(Method method, bool do_arming_checks)
{
    // Pre-arm checks
    if (do_arming_checks && !all_checks_passing()) {
        return false;
    }

    // Set armed state
    hal.util->set_soft_armed(true);

    // ===== POST-ARM INITIALIZATION =====

    // 1. Timestamp
    last_arm_time_us = AP_HAL::micros64();
    _last_arm_method = method;

    // 2. Logging
    Log_Write_Arm(!do_arming_checks, method);
    AP::logger()->transition_to_active_logging();

    // 3. FFT
    auto *fft = AP::fft();
    if (fft) fft->prepare_for_arming();

    // 4. Terrain
    auto *terrain = AP::terrain();
    if (terrain) terrain->set_reference_location();

    // 5. Fence
    auto *fence = AP::fence();
    if (fence) fence->auto_enable_fence_on_arming();

    // 6. GPIO
    update_arm_gpio();

    // 7. Warnings
    if (!do_arming_checks) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "Warning: Arming Checks Disabled");
    }

    // Vehicle-specific init
    on_successful_arming();

    return true;
}
```

**Proposed pico_trail Init Sequence**:

```
arm()
  │
  ├─ Pre-arm checks
  │   ├─ Already armed? → ERROR
  │   ├─ Battery critical? → ERROR
  │   └─ (Additional checks...)
  │
  ├─ Set armed state
  │   └─ self.armed = Armed
  │
  └─ post_arm_init()
      │
      ├─ 1. Record timestamp (< 1µs)
      │   └─ arm_time_ms = get_time_ms()
      │
      ├─ 2. Log arm event (~10ms)
      │   └─ log("ARM,{},{:?},{}",timestamp,method,checks)
      │
      ├─ 3. Initialize actuators (~5ms)
      │   ├─ Throttle → Armed idle (1100µs)
      │   ├─ Steering → Center (1500µs)
      │   └─ Verify commands accepted
      │
      ├─ 4. Notify subsystems (~1ms)
      │   ├─ Monitoring: reset baselines
      │   ├─ Failsafe: enable detection
      │   └─ Modes: adjust for armed
      │
      ├─ 5. Update GPIO (< 1µs)
      │   └─ Set arm indicator LED/pin
      │
      └─ 6. Send warnings (if needed)
          └─ "Arming checks disabled"

Total: ~20ms typical, ~50ms worst case
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
