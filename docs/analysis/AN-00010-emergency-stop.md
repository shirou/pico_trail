# AN-00010 Emergency Stop Mechanism for Safe Vehicle Shutdown

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-00011-failsafe-system](AN-00011-failsafe-system.md)
  - [AN-00009-armed-state-monitoring](AN-00009-armed-state-monitoring.md)
  - [AN-00008-pre-arm-checks](AN-00008-pre-arm-checks.md)
- Related Requirements:
  - [FR-00023-controlled-emergency-stop](../requirements/FR-00023-controlled-emergency-stop.md)
  - [FR-00056-stop-tier-system](../requirements/FR-00056-stop-tier-system.md)
  - [FR-00032-emergency-stop-recovery](../requirements/FR-00032-emergency-stop-recovery.md)
  - [FR-00036-failsafe-integration](../requirements/FR-00036-failsafe-integration.md)
  - [FR-00033-emergency-stop-triggers](../requirements/FR-00033-emergency-stop-triggers.md)
  - [FR-00055-stop-state-tracking](../requirements/FR-00055-stop-state-tracking.md)
  - [NFR-00027-emergency-stop-ram-overhead](../requirements/NFR-00027-emergency-stop-ram-overhead.md)
  - [NFR-00026-emergency-stop-initiation-latency](../requirements/NFR-00026-emergency-stop-initiation-latency.md)
  - [NFR-00025-emergency-stop-event-logging](../requirements/NFR-00025-emergency-stop-event-logging.md)
  - [NFR-00021-controlled-stop-completion-time](../requirements/NFR-00021-controlled-stop-completion-time.md)
  - [NFR-00045-no-rollover-loss-of-control](../requirements/NFR-00045-no-rollover-loss-of-control.md)
- Related ADRs: N/A - ADRs will be created based on this analysis
- Related Tasks: N/A - Tasks will be created after requirements and ADRs

## Executive Summary

This analysis explores emergency stop mechanisms needed to safely halt vehicle operation in critical situations. Currently, pico_trail has no emergency stop capability - the only way to stop the vehicle is full disarm, which provides no controlled deceleration or safety measures. Emergency stop is distinct from disarm: it performs controlled deceleration while maintaining vehicle stability, prevents rollover through steering management, and can be triggered by multiple sources (GCS command, RC switch, failsafe conditions).

Key findings: ArduPilot implements a sophisticated `stop_vehicle()` function that uses attitude controllers for gradual deceleration rather than immediate motor cutoff, maintains heading stability during stop, tracks stopped state completion, and integrates with failsafe systems. For pico_trail, a similar graduated approach is recommended with three tiers: controlled stop (normal), aggressive stop (critical situations), and emergency disarm (absolute last resort). The controlled stop mechanism should be the default response to most failsafe conditions.

## Problem Space

### Current State

The project currently has:

- **Arming/disarming**: `SystemState::arm()` and `SystemState::disarm()` in `src/communication/mavlink/state.rs:157-183`
- **No emergency stop**: No mechanism between "normal operation" and "full disarm"
- **No controlled deceleration**: Disarm immediately cuts power to motors
- **No stop state**: No concept of "stopped but still armed"
- **No trigger mechanisms**: No RC switch or GCS command to trigger emergency stop

Critical safety gaps:

- **Abrupt stop risk**: Immediate motor cutoff can cause rollover on ground vehicles
- **No failsafe integration**: Failsafe system (AN-00011) has no controlled stop option
- **No operator control**: No way for operator to trigger emergency stop via RC or GCS
- **Uncontrolled deceleration**: No steering correction during stop leads to unpredictable paths
- **No stop confirmation**: System doesn't know when vehicle has fully stopped
- **Recovery unclear**: No defined path from stopped state back to normal operation

### Desired State

Enable comprehensive emergency stop system with graduated response levels:

1. **Controlled Stop (Tier 1)**: Gradual deceleration with steering management, maintained heading stability
2. **Aggressive Stop (Tier 2)**: Rapid deceleration for critical situations, still controlled but faster
3. **Emergency Disarm (Tier 3)**: Immediate motor cutoff as absolute last resort
4. **Multiple Triggers**: RC switch, GCS command, MAV_CMD_DO_MOTOR_TEST with 0 throttle, failsafe conditions
5. **Stop State Management**: Track stopped state, know when deceleration complete
6. **Integration with Modes**: Hold mode uses controlled stop, failsafes trigger appropriate tier
7. **Recovery Procedures**: Clear path from stopped state to normal operation

Success criteria:

- **Safe deceleration**: Vehicle stops without rollover or loss of control
- **Predictable behavior**: Operators know exactly what will happen on emergency stop
- **Tiered response**: Appropriate stop method selected based on severity
- **Failsafe integration**: Emergency stop available as failsafe action
- **Stop confirmation**: System reports when vehicle has fully stopped
- **Recoverable**: Vehicle can resume operation after emergency stop clears

### Gap Analysis

**Missing components**:

1. **Controlled Stop Function**: Gradual deceleration algorithm with throttle ramping
2. **Steering Management**: Heading hold or steering lockout during stop
3. **Stop State Tracking**: Boolean flag or enum tracking stopped status
4. **Attitude Controller Integration**: Use existing control loops for smooth deceleration
5. **Emergency Stop Triggers**: RC switch mapping, GCS command handlers, failsafe integration
6. **Stop Tier Selection**: Logic to choose controlled/aggressive/disarm based on situation
7. **Stop Confirmation**: Velocity monitoring to detect when vehicle fully stopped
8. **Recovery Logic**: Transition from stopped state back to normal operation

**Technical deltas**:

- Add `src/vehicle/emergency_stop/` module with stop mechanisms
- Implement `controlled_stop()` function for gradual deceleration
- Add `aggressive_stop()` for rapid but controlled halt
- Create stop state enum: `StopState::Operating | Stopping | Stopped`
- Integrate with attitude controller for throttle ramping
- Add steering management during stop (heading hold or lockout)
- Implement velocity monitoring for stop confirmation
- Add MAV_CMD_DO_MOTOR_TEST handler for emergency stop
- Create RC channel mapping for emergency stop switch
- Integrate with failsafe system (use controlled stop as failsafe action)
- Add recovery procedures (operator must acknowledge stop before resume)

## Stakeholder Analysis

| Stakeholder        | Interest/Need                                       | Impact | Priority |
| ------------------ | --------------------------------------------------- | ------ | -------- |
| Operators          | Safe emergency stop without vehicle rollover        | High   | P0       |
| Safety Reviewers   | Graduated response to emergencies                   | High   | P0       |
| Failsafe System    | Controlled stop as failsafe action option           | High   | P0       |
| Test Engineers     | Predictable stop behavior for testing               | High   | P0       |
| Autonomous Systems | Safe stop mechanism for autonomous operation        | High   | P1       |
| Regulatory Bodies  | Emergency stop compliance for vehicle certification | Medium | P1       |

## Research & Discovery

### User Feedback

From operational requirements:

- Emergency stop is fundamental safety feature for any vehicle
- Immediate motor cutoff causes rollovers in high-speed ground vehicles
- Operators need multiple ways to trigger emergency stop (RC, GCS, autonomous)
- Stop behavior must be predictable and testable
- Recovery from emergency stop should require explicit operator acknowledgment
- Emergency stop should be available in all flight modes

### Competitive Analysis

**ArduPilot Rover Emergency Stop System**:

ArduPilot implements emergency stop through multiple mechanisms working together.

#### stop_vehicle() Function

**Implementation** (from mode.cpp:332-361):

```cpp
bool Mode::stop_vehicle()
{
    bool stopped = false;

    // For balance bots, use speed controller to maintain stability
    if (rover.is_balancebot()) {
        // Call velocity controller requesting zero speed
        float throttle_out = attitude_control.get_throttle_out_speed(
            /* target_speed */ 0.0f,
            /* limit_output */ true,
            /* stopped */ stopped
        );
        g2.motors.set_throttle(throttle_out);
    } else {
        // For normal rovers, use stop controller for gradual deceleration
        float throttle_out = attitude_control.get_throttle_out_stop(
            /* limit_output */ true,
            /* stopped */ stopped
        );
        g2.motors.set_throttle(throttle_out);
    }

    // Steering management during stop
    float steering_out = 0.0f;
    if (!stopped) {
        // Maintain heading while decelerating (prevent turns)
        steering_out = attitude_control.get_steering_out_rate(
            /* desired_rate */ 0.0f,
            /* limit_output */ true
        );
    } else {
        // Once stopped, lock steering to zero
        steering_out = 0.0f;
    }
    g2.motors.set_steering(steering_out);

    // Relax sails if present (sailboat variants)
    g2.sailboat.relax(true);

    return stopped;
}
```

**Key Design Principles**:

1. **Controlled Deceleration**: Uses attitude controller's stop throttle function, not immediate cutoff
2. **Heading Stability**: Maintains heading via rate controller during deceleration
3. **Steering Lockout**: Only after stopped, steering goes to zero
4. **Stopped Confirmation**: Returns boolean indicating when vehicle fully stopped
5. **Vehicle Type Aware**: Different logic for balance bots vs. normal rovers
6. **Auxiliary Systems**: Gracefully handles sailboat sails and other systems

#### Throttle Controller Stop Function

**get_throttle_out_stop() Implementation**:

The stop controller gradually reduces throttle output based on current velocity:

```cpp
float AR_AttitudeControl::get_throttle_out_stop(bool limit_output, bool& stopped)
{
    // Get current speed from velocity controller
    float speed_curr;
    if (!get_forward_speed(speed_curr)) {
        stopped = false;
        return 0.0f;
    }

    // Check if already stopped (velocity near zero)
    if (fabsf(speed_curr) <= SPEED_STOP_THRESHOLD) {
        stopped = true;
        return 0.0f;
    }

    // Calculate deceleration throttle
    // Uses negative throttle proportional to current speed
    const float brake_throttle = -constrain_float(
        speed_curr * _stop_speed_P,  // P gain on speed error
        0.0f,
        100.0f
    );

    stopped = false;
    return brake_throttle;
}
```

**Design Notes**:

- **Velocity-based**: Stop throttle proportional to current speed
- **Threshold detection**: Considers vehicle stopped at `SPEED_STOP_THRESHOLD` (typically 0.1 m/s)
- **Gradual ramping**: As speed decreases, brake force decreases
- **Never abrupt**: Always returns smooth throttle commands

#### Emergency Stop vs. Disarm

ArduPilot distinguishes between three stopping mechanisms:

**1. Controlled Stop** (`stop_vehicle()`):

- Gradual deceleration using controllers
- Vehicle remains armed
- Can resume operation without re-arming
- Used by: Hold mode, failsafe actions, waypoint arrival

**2. Emergency Stop (E-Stop)**:

- Immediate motor output to zero via `SRV_Channels::set_emergency_stop(true)`
- Vehicle remains armed but motors disabled
- Triggered by: RC switch, GCS command
- Requires explicit clear before resuming

**3. Disarm**:

- Full system shutdown
- Vehicle disarmed, requires full arm sequence to resume
- Triggered by: Pilot command, critical failsafes, landing complete
- Most aggressive option

#### Emergency Stop Triggers

**RC Switch Trigger**:

```cpp
// Check RC emergency stop switch
void Rover::read_aux_switches()
{
    if (rc().find_channel_for_option(RC_Channel::AUX_FUNC::EMERGENCY_STOP)) {
        if (rc().get_aux_cached(RC_Channel::AUX_FUNC::EMERGENCY_STOP) ==
            RC_Channel::AuxSwitchPos::HIGH) {
            // Trigger emergency stop
            SRV_Channels::set_emergency_stop(true);
        } else {
            // Clear emergency stop
            SRV_Channels::set_emergency_stop(false);
        }
    }
}
```

**GCS Command Trigger**:

```cpp
MAV_RESULT Rover::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    switch (packet.command) {
        case MAV_CMD_DO_MOTOR_TEST:
            // param1: motor instance
            // param2: throttle type
            // param3: throttle value
            // param4: timeout
            if (packet.param3 == 0.0f) {
                // Zero throttle = emergency stop
                motor_test_stop();
                return MAV_RESULT_ACCEPTED;
            }
            return mavlink_motor_test_start(packet);

        case MAV_CMD_COMPONENT_ARM_DISARM:
            // param1: 0=disarm, 1=arm, 2=emergency_stop
            if (packet.param1 == 2.0f) {
                SRV_Channels::set_emergency_stop(true);
                return MAV_RESULT_ACCEPTED;
            }
            // ... normal arm/disarm logic
    }
}
```

#### Failsafe Integration

Emergency stop is available as failsafe action in ArduPilot:

```cpp
// Failsafe action parameter values
enum Failsafe_Action {
    Failsafe_Action_None        = 0,
    Failsafe_Action_RTL         = 1,
    Failsafe_Action_Hold        = 2,  // Uses stop_vehicle()
    Failsafe_Action_SmartRTL    = 3,
    Failsafe_Action_SmartRTL_Hold = 4,
    Failsafe_Action_Terminate   = 5   // Emergency stop + disarm
};
```

Hold mode calls `stop_vehicle()` continuously, providing controlled stop as failsafe response.

**PX4 Emergency Stop System**:

PX4 uses a different approach:

- **Kill Switch**: Immediate motor cutoff via `VEHICLE_CMD_DO_SET_ACTUATOR` with zero
- **Land Mode**: Controlled descent/stop for multicopters, gradual stop for rovers
- **Commander State Machine**: Centralized emergency handling

### Technical Investigation

**Current pico_trail Architecture**:

File: `src/communication/mavlink/state.rs:173-183` (disarm implementation)

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

- Only disarm available, no emergency stop
- No controlled deceleration
- Immediate state change, no gradual process
- No stop state tracking

**Proposed Emergency Stop Architecture**:

```rust
/// Stop tier enumeration (severity levels)
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum StopTier {
    None,            // Normal operation
    Controlled,      // Gradual deceleration (2-3 seconds)
    Aggressive,      // Rapid deceleration (0.5-1 second)
    EmergencyDisarm, // Immediate motor cutoff
}

/// Stop state enumeration
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum StopState {
    Operating,   // Normal operation
    Stopping,    // Deceleration in progress
    Stopped,     // Vehicle fully stopped
}

/// Emergency stop manager
pub struct EmergencyStop {
    /// Current stop state
    state: StopState,

    /// Active stop tier
    tier: StopTier,

    /// Stop start timestamp
    stop_start_time_ms: u32,

    /// Target velocity (always 0 for stop)
    target_velocity: f32,

    /// Stop trigger source (for logging)
    trigger_source: StopTrigger,
}

/// Stop trigger source
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum StopTrigger {
    None,
    RcSwitch,
    GcsCommand,
    Failsafe,
    ModeChange,
}

impl EmergencyStop {
    /// Initiate controlled stop (Tier 1)
    pub fn start_controlled_stop(&mut self, trigger: StopTrigger) {
        if self.state == StopState::Operating {
            info!("Starting controlled stop (trigger: {:?})", trigger);
            self.state = StopState::Stopping;
            self.tier = StopTier::Controlled;
            self.trigger_source = trigger;
            self.stop_start_time_ms = get_time_ms();
            self.target_velocity = 0.0;
        }
    }

    /// Initiate aggressive stop (Tier 2)
    pub fn start_aggressive_stop(&mut self, trigger: StopTrigger) {
        info!("Starting aggressive stop (trigger: {:?})", trigger);
        self.state = StopState::Stopping;
        self.tier = StopTier::Aggressive;
        self.trigger_source = trigger;
        self.stop_start_time_ms = get_time_ms();
        self.target_velocity = 0.0;
    }

    /// Initiate emergency disarm (Tier 3)
    pub fn start_emergency_disarm(&mut self, trigger: StopTrigger) {
        warn!("Emergency disarm triggered (trigger: {:?})", trigger);
        self.state = StopState::Stopped;  // Immediate
        self.tier = StopTier::EmergencyDisarm;
        self.trigger_source = trigger;
        self.stop_start_time_ms = get_time_ms();
        self.target_velocity = 0.0;
    }

    /// Update stop state (called from vehicle control loop)
    pub fn update(
        &mut self,
        current_velocity: f32,
        dt: f32,
    ) -> Result<StopOutput, &'static str> {
        match self.state {
            StopState::Operating => {
                // Not stopping, return normal operation
                Ok(StopOutput {
                    throttle: None,
                    steering: None,
                    stopped: false,
                })
            }
            StopState::Stopping => {
                // Check if stopped (velocity near zero)
                if current_velocity.abs() < STOP_VELOCITY_THRESHOLD {
                    info!("Vehicle stopped after {:.1}s",
                          (get_time_ms() - self.stop_start_time_ms) as f32 / 1000.0);
                    self.state = StopState::Stopped;
                    return Ok(StopOutput {
                        throttle: Some(0.0),
                        steering: Some(0.0),
                        stopped: true,
                    });
                }

                // Calculate stop throttle based on tier
                let stop_throttle = match self.tier {
                    StopTier::Controlled => {
                        // Gradual deceleration (proportional to velocity)
                        -current_velocity * CONTROLLED_STOP_P_GAIN
                    }
                    StopTier::Aggressive => {
                        // Rapid deceleration (higher gain)
                        -current_velocity * AGGRESSIVE_STOP_P_GAIN
                    }
                    StopTier::EmergencyDisarm => 0.0,  // Should not reach here
                    StopTier::None => 0.0,
                };

                // Clamp throttle to safe limits
                let throttle = stop_throttle.clamp(-100.0, 0.0);

                // Steering during stop: maintain heading (rate = 0)
                // TODO: Integrate with attitude controller
                let steering = 0.0;

                Ok(StopOutput {
                    throttle: Some(throttle),
                    steering: Some(steering),
                    stopped: false,
                })
            }
            StopState::Stopped => {
                // Already stopped, maintain zero throttle
                Ok(StopOutput {
                    throttle: Some(0.0),
                    steering: Some(0.0),
                    stopped: true,
                })
            }
        }
    }

    /// Clear emergency stop (resume operation)
    pub fn clear_stop(&mut self) -> Result<(), &'static str> {
        if self.state == StopState::Operating {
            return Err("Not stopped");
        }

        info!("Clearing emergency stop");
        self.state = StopState::Operating;
        self.tier = StopTier::None;
        self.trigger_source = StopTrigger::None;
        Ok(())
    }

    /// Check if vehicle is stopped
    pub fn is_stopped(&self) -> bool {
        self.state == StopState::Stopped
    }

    /// Check if vehicle is stopping
    pub fn is_stopping(&self) -> bool {
        self.state == StopState::Stopping
    }
}

/// Stop output (throttle and steering commands)
pub struct StopOutput {
    pub throttle: Option<f32>,  // -100 to 100, or None if not stopping
    pub steering: Option<f32>,  // -100 to 100, or None if not stopping
    pub stopped: bool,          // True if vehicle fully stopped
}

// Stop configuration constants
const STOP_VELOCITY_THRESHOLD: f32 = 0.1;  // m/s (consider stopped below this)
const CONTROLLED_STOP_P_GAIN: f32 = 10.0;  // Gradual deceleration gain
const AGGRESSIVE_STOP_P_GAIN: f32 = 30.0;  // Rapid deceleration gain
```

**Integration with Vehicle Control Loop**:

```rust
/// Vehicle control task
pub fn vehicle_control_task() {
    let mut emergency_stop = EmergencyStop::new();
    let mut velocity_estimator = VelocityEstimator::new();

    loop {
        // Update velocity estimate
        let current_velocity = velocity_estimator.update();

        // Update emergency stop
        let stop_output = emergency_stop.update(current_velocity, dt)?;

        if stop_output.stopped {
            // Vehicle fully stopped
            if let Some(throttle) = stop_output.throttle {
                set_throttle(throttle);
            }
            if let Some(steering) = stop_output.steering {
                set_steering(steering);
            }
        } else if stop_output.throttle.is_some() {
            // Stopping in progress
            set_throttle(stop_output.throttle.unwrap());
            set_steering(stop_output.steering.unwrap());
        } else {
            // Normal operation, run mode logic
            mode.update();
        }

        delay_ms(20);  // 50 Hz control loop
    }
}
```

**RC Switch Trigger**:

```rust
/// Handle RC emergency stop switch (mapped to channel 7, for example)
pub fn handle_rc_emergency_stop(rc_channels: &RcChannels, emergency_stop: &mut EmergencyStop) {
    // Check channel 7 (emergency stop switch)
    if let Some(ch7_value) = rc_channels.get_channel(7) {
        // High position (> 1700) = emergency stop active
        if ch7_value > 1700 {
            if emergency_stop.state == StopState::Operating {
                emergency_stop.start_controlled_stop(StopTrigger::RcSwitch);
            }
        } else {
            // Low position = clear stop
            if emergency_stop.is_stopped() || emergency_stop.is_stopping() {
                let _ = emergency_stop.clear_stop();
            }
        }
    }
}
```

**GCS Command Handler**:

```rust
/// Handle MAV_CMD_DO_MOTOR_TEST for emergency stop
fn handle_motor_test_command(
    cmd: &COMMAND_LONG_DATA,
    emergency_stop: &mut EmergencyStop,
) -> MavResult {
    let motor_instance = cmd.param1 as u8;
    let throttle_type = cmd.param2 as u8;
    let throttle_value = cmd.param3;

    // Zero throttle = emergency stop
    if throttle_value == 0.0 {
        emergency_stop.start_controlled_stop(StopTrigger::GcsCommand);
        return MavResult::MAV_RESULT_ACCEPTED;
    }

    // Normal motor test logic...
    MavResult::MAV_RESULT_UNSUPPORTED
}

/// Handle MAV_CMD_COMPONENT_ARM_DISARM with emergency stop extension
fn handle_arm_disarm_command(
    cmd: &COMMAND_LONG_DATA,
    state: &mut SystemState,
    emergency_stop: &mut EmergencyStop,
) -> MavResult {
    let arm_value = cmd.param1;

    if arm_value == 2.0 {
        // param1 = 2.0 = emergency stop (ArduPilot extension)
        emergency_stop.start_aggressive_stop(StopTrigger::GcsCommand);
        MavResult::MAV_RESULT_ACCEPTED
    } else if arm_value > 0.5 {
        // Arm
        state.arm().map_or(MavResult::MAV_RESULT_DENIED, |_| {
            MavResult::MAV_RESULT_ACCEPTED
        })
    } else {
        // Disarm
        state.disarm().map_or(MavResult::MAV_RESULT_DENIED, |_| {
            MavResult::MAV_RESULT_ACCEPTED
        })
    }
}
```

**Failsafe Integration**:

```rust
/// Failsafe executor using emergency stop
impl FailsafeExecutor {
    fn execute_action(
        &mut self,
        failsafe_type: FailsafeType,
        action: FailsafeAction,
        emergency_stop: &mut EmergencyStop,
    ) -> Result<(), &'static str> {
        match action {
            FailsafeAction::Hold => {
                // Hold mode = controlled stop
                emergency_stop.start_controlled_stop(StopTrigger::Failsafe);
                Ok(())
            }
            FailsafeAction::Terminate => {
                // Terminate = aggressive stop + disarm
                emergency_stop.start_aggressive_stop(StopTrigger::Failsafe);
                // Will disarm after stopped
                Ok(())
            }
            // ... other actions
        }
    }
}
```

**Memory Analysis**:

| Component              | RAM Usage   | Notes                                  |
| ---------------------- | ----------- | -------------------------------------- |
| EmergencyStop          | \~40 B      | State + timestamps + flags             |
| StopOutput             | \~12 B      | Throttle + steering + stopped flag     |
| **Total (emergency)**  | **\~50 B**  | Minimal overhead for safety            |
| \*\*Total (full system | **\~400 B** | Emergency stop + monitoring + failsafe |

### Data Analysis

**Stop Tier Selection Criteria**:

| Situation                  | Stop Tier       | Rationale                                         |
| -------------------------- | --------------- | ------------------------------------------------- |
| Hold mode entry            | Controlled      | Normal operation, no emergency                    |
| RC loss failsafe           | Controlled      | Loss of manual control, but not critical          |
| Battery low failsafe       | Controlled      | Warning level, time for safe stop                 |
| Battery critical failsafe  | Aggressive      | Emergency level, rapid stop needed                |
| GCS emergency stop command | Aggressive      | Operator-initiated emergency                      |
| RC emergency stop switch   | Controlled      | Operator-initiated, but allow smooth deceleration |
| Obstacle detection         | Aggressive      | Collision imminent, rapid stop required           |
| Motor/ESC failure          | EmergencyDisarm | Hardware failure, cannot control deceleration     |

**Stop Performance Requirements**:

| Stop Tier       | Deceleration Time   | Max Deceleration | Use Cases                    |
| --------------- | ------------------- | ---------------- | ---------------------------- |
| Controlled      | 2-3 seconds         | \~0.5 m/s²       | Normal failsafes, Hold mode  |
| Aggressive      | 0.5-1 second        | \~2.0 m/s²       | Critical failsafes, GCS stop |
| EmergencyDisarm | Immediate (< 100ms) | N/A (cutoff)     | Hardware failure only        |

**Velocity Threshold Analysis**:

- **Too high** (> 0.2 m/s): Vehicle still moving when declared stopped
- **Optimal** (0.1 m/s): Good balance, typical ArduPilot value
- **Too low** (< 0.05 m/s): Takes too long to detect stop, false negatives

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The system shall implement controlled emergency stop with gradual deceleration → Will become FR-<id>
  - Rationale: Prevent vehicle rollover and loss of control during emergency stops
  - Acceptance Criteria:
    - Calculate stop throttle proportional to current velocity (velocity-based P controller)
    - Deceleration time: 2-3 seconds for controlled stop
    - Maintain heading stability during deceleration (steering rate = 0)
    - Monitor velocity to detect when vehicle fully stopped (< 0.1 m/s)
    - Return stopped confirmation flag when stop complete

- [ ] **FR-DRAFT-2**: The system shall support three stop tiers with different severity levels → Will become FR-<id>
  - Rationale: Different situations require different stop responses
  - Acceptance Criteria:
    - Tier 1 (Controlled): Gradual deceleration, 2-3 seconds, P gain = 10.0
    - Tier 2 (Aggressive): Rapid deceleration, 0.5-1 second, P gain = 30.0
    - Tier 3 (EmergencyDisarm): Immediate motor cutoff, < 100ms
    - Tier selection based on trigger source and situation severity

- [ ] **FR-DRAFT-3**: The system shall track stop state through Operating/Stopping/Stopped states → Will become FR-<id>
  - Rationale: System and operators need to know stop progress
  - Acceptance Criteria:
    - Operating: Normal operation, no stop active
    - Stopping: Deceleration in progress, stop throttle applied
    - Stopped: Vehicle velocity below threshold, stop complete
    - State transitions logged with timestamps
    - State exposed via telemetry (SYS_STATUS or custom message)

- [ ] **FR-DRAFT-4**: The system shall support multiple emergency stop triggers → Will become FR-<id>
  - Rationale: Operators need multiple ways to trigger emergency stop
  - Acceptance Criteria:
    - RC switch trigger (configurable channel, high position = stop)
    - GCS command trigger (MAV_CMD_DO_MOTOR_TEST with throttle = 0)
    - Failsafe trigger (Hold mode, battery critical, etc.)
    - Mode change trigger (switching to Hold mode)
    - All triggers logged with source identification

- [ ] **FR-DRAFT-5**: The system shall require explicit operator acknowledgment to clear emergency stop → Will become FR-<id>
  - Rationale: Prevent accidental resume after emergency stop
  - Acceptance Criteria:
    - Emergency stop does not auto-clear when trigger removed
    - Operator must explicitly clear via RC switch or GCS command
    - Clearing stop logged with timestamp and duration
    - Vehicle resumes normal operation only after clear confirmed

- [ ] **FR-DRAFT-6**: The system shall integrate emergency stop with failsafe system → Will become FR-<id>
  - Rationale: Failsafes need controlled stop as action option
  - Acceptance Criteria:
    - Hold failsafe action triggers controlled stop
    - Terminate failsafe action triggers aggressive stop + disarm
    - Emergency stop state influences failsafe decisions
    - Failsafe priority overrides emergency stop clear (cannot clear during active failsafe)

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Emergency stop shall initiate within 100ms of trigger → Will become NFR-<id>
  - Category: Performance / Safety
  - Rationale: Rapid response critical for emergency situations
  - Target: < 100ms from trigger detection to first stop throttle command

- [ ] **NFR-DRAFT-2**: Controlled stop shall complete within 3 seconds from 1 m/s velocity → Will become NFR-<id>
  - Category: Performance
  - Rationale: Predictable stop behavior for operator planning
  - Target: 2-3 seconds deceleration time at typical velocities (measured via testing)

- [ ] **NFR-DRAFT-3**: Emergency stop shall add no more than 50 bytes RAM overhead → Will become NFR-<id>
  - Category: Resource Constraints
  - Rationale: Maintain memory budget for other subsystems
  - Target: < 50 B for emergency stop state (measured via runtime profiling)

- [ ] **NFR-DRAFT-4**: Emergency stop events shall be logged with full context → Will become NFR-<id>
  - Category: Reliability / Auditability
  - Rationale: Support post-incident analysis and debugging
  - Target: Log trigger source, tier, duration, velocity at stop for all emergency stops

- [ ] **NFR-DRAFT-5**: Emergency stop shall not cause vehicle rollover or loss of control → Will become NFR-<id>
  - Category: Safety
  - Rationale: Emergency stop must be safer than continued operation
  - Target: No rollover events during emergency stop testing (validated via field tests)

## Design Considerations

### Technical Constraints

- **Velocity estimation required**: Need reliable velocity measurement for stop detection
- **Attitude controller dependency**: Controlled stop requires attitude control loops
- **Real-time requirements**: Stop must execute in control loop (50 Hz minimum)
- **Memory budget**: Limited RAM on RP2040/RP2350 (264 KB)
- **No dynamic allocation**: Emergency stop must use static/stack allocation only
- **Integration with modes**: Hold mode and others must use emergency stop
- **Failsafe integration**: Emergency stop must work with failsafe system (AN-00011)

### Potential Approaches

1. **Option A: Simple Immediate Cutoff**
   - Pros:
     - Simplest implementation (set throttle = 0)
     - No velocity estimation required
     - Fastest stop possible
     - Minimal memory (\~10 B)
   - Cons:
     - High rollover risk on ground vehicles
     - Uncontrolled deceleration path
     - No stop confirmation
     - Poor integration with modes
   - Effort: Low (4-8 hours)

2. **Option B: Controlled Stop with P Controller** ⭐ Recommended
   - Pros:
     - Safe deceleration prevents rollover
     - Velocity-based control provides smooth stop
     - Matches ArduPilot architecture (proven, familiar)
     - Integrates well with modes and failsafes
     - Moderate memory (\~50 B)
   - Cons:
     - Requires velocity estimation
     - More complex than immediate cutoff
     - Longer stop time than immediate cutoff
   - Effort: Medium (16-24 hours)

3. **Option C: Full ArduPilot-Style Stop with PID**
   - Pros:
     - Maximum stop performance
     - PID controller handles all vehicle types
     - Sophisticated steering management
     - Best integration with attitude controllers
   - Cons:
     - High complexity
     - Requires full attitude controller implementation
     - Higher memory (\~100 B)
     - Overkill for initial implementation
   - Effort: High (40-60 hours)

**Recommendation**: Option B (Controlled Stop with P Controller) provides best balance of safety, performance, and development effort. P control sufficient for initial implementation; can upgrade to PID later if needed.

### Architecture Impact

**New ADRs required**:

- **ADR-<id> Emergency Stop Architecture**: Stop tiers, state machine, trigger mechanisms
- **ADR-<id> Stop Tier Selection Policy**: Criteria for choosing controlled/aggressive/disarm
- **ADR-<id> Emergency Stop Recovery**: Operator acknowledgment requirements, auto-clear policy

**New modules**:

- `src/vehicle/emergency_stop/` - Emergency stop system
  - `src/vehicle/emergency_stop/types.rs` - StopTier, StopState, StopTrigger enums
  - `src/vehicle/emergency_stop/manager.rs` - EmergencyStop manager
  - `src/vehicle/emergency_stop/controller.rs` - Stop throttle calculation (P controller)
- `src/vehicle/velocity/` - Velocity estimation (if not already exists)
  - `src/vehicle/velocity/estimator.rs` - Velocity estimation from wheel encoders/GPS

**Modified modules**:

- `src/vehicle/modes/hold.rs` - Use controlled stop mechanism
- `src/vehicle/failsafe/executor.rs` - Integrate emergency stop with failsafe actions
- `src/communication/mavlink/handlers/command.rs` - Add emergency stop command handlers
- `src/communication/mavlink/handlers/telemetry.rs` - Handle RC emergency stop switch
- `src/vehicle/control_task.rs` - Integrate emergency stop into control loop

## Parameters

### ArduPilot Standard Implementation

ArduPilot implements emergency stop functionality through existing systems rather than dedicated emergency stop parameters:

**1. RC Auxiliary Function (Motor Emergency Stop)**

- **RCx_OPTION = 31**: Motor Emergency Stop auxiliary function
  - Assign to any RC channel (e.g., RC7_OPTION = 31 for channel 7)
  - Triggered when PWM > 1800, deactivated when PWM < 1200
  - Immediately stops motors while maintaining armed state
  - Requires manual clear before resuming operation

**2. Hold Mode for Controlled Stop**

- **FS_ACTION = 2**: Use Hold mode as failsafe action
  - Hold mode calls `stop_vehicle()` function internally
  - Provides controlled deceleration using attitude controllers
  - Available as failsafe response for RC loss, GCS loss, battery low, etc.

**3. Attitude Controller Parameters**

- **ATC_STR_RAT_P**: Steering rate controller P gain
  - Used for maintaining heading during stop (rate = 0)
  - Default values vehicle-specific, typically 0.2-0.5

- **ATC_SPEED_P**: Speed controller P gain
  - Controls deceleration rate during controlled stop
  - Higher values = faster deceleration
  - Typical range: 0.2-2.0

**4. Mode Configuration**

- **MODE_CH**: RC channel for mode selection (default: channel 8)
  - Allows switching to Hold mode for controlled stop
  - Used in combination with MODE1-6 parameters

For pico_trail Phase 1, we follow ArduPilot's approach: use RCx_OPTION for emergency stop trigger, Hold mode for controlled deceleration, and ATC\_\* parameters for tuning stop behavior. No custom ESTOP\_\* parameters needed.

## Risk Assessment

| Risk                                                      | Probability | Impact       | Mitigation Strategy                                                                       |
| --------------------------------------------------------- | ----------- | ------------ | ----------------------------------------------------------------------------------------- |
| **Vehicle rollover during emergency stop**                | **Medium**  | **CRITICAL** | **Use controlled deceleration, test extensively on different surfaces and speeds**        |
| **Stop takes too long (collision occurs)**                | **Low**     | **High**     | **Provide aggressive stop tier for critical situations, tune P gains via testing**        |
| **Velocity estimation error causes false stop detection** | **Medium**  | **Medium**   | **Conservative threshold (0.1 m/s), validate velocity estimator accuracy**                |
| False emergency stop triggers (operator error)            | Medium      | Medium       | Require deliberate switch action (not momentary), provide clear GCS indication            |
| Cannot resume after emergency stop (stuck stopped)        | Low         | High         | Multiple clear mechanisms (RC, GCS), timeout-based auto-clear after 60 seconds (optional) |
| Emergency stop conflicts with mode logic                  | Low         | Medium       | Clear priority: emergency stop overrides mode, document interaction in ADR                |
| Stop deceleration insufficient (vehicle slides)           | Medium      | High         | Tune P gain per vehicle mass and terrain, provide parameter for adjustment                |
| Emergency stop memory overhead exceeds budget             | Low         | Low          | Profile early, optimize state structure if needed                                         |

## Open Questions

- [ ] Should emergency stop auto-clear after timeout (e.g., 60s)? → Method: Phase 1 = manual clear only, Phase 2 = add optional timeout parameter
- [ ] Which stop tier should RC switch trigger? → Decision: Controlled stop (safer than aggressive for operator-initiated stops)
- [ ] Should emergency stop be available while disarmed? → Decision: No, only available when armed (cannot stop what's not moving)
- [ ] How to handle emergency stop during mode transitions? → Method: Emergency stop has highest priority, interrupts any mode change
- [ ] Should aggressive stop automatically disarm after stopped? → Decision: No, remain armed to allow recovery; operator can manually disarm if desired
- [ ] Do we need separate stop gains for forward vs reverse? → Method: Phase 1 = same gain both directions, Phase 2 = add separate parameters if needed
- [ ] Should emergency stop integrate with geofence? → Method: Out of scope for Phase 1, add in Phase 2 when geofence implemented
- [ ] How to test emergency stop safety without actual vehicle? → Method: Use SITL/simulation first, then careful field testing at low speeds

## Recommendations

### Immediate Actions

1. **Adopt Option B architecture**: Controlled stop with P controller
2. **Implement two tiers initially**: Controlled and EmergencyDisarm (defer Aggressive to Phase 2)
3. **Start with RC and GCS triggers**: Defer failsafe integration until after basic stop works
4. **Use conservative P gain**: Start with ArduPilot-proven value (10.0), tune via testing
5. **Require manual clear**: No auto-clear in Phase 1 for safety

### Next Steps

1. [ ] Create formal requirements: FR-<id> (controlled stop), FR-<id> (stop tiers), FR-<id> (stop state), FR-<id> (stop triggers), FR-<id> (stop recovery), FR-<id> (failsafe integration), NFR-<id> (response time), NFR-<id> (stop time), NFR-<id> (memory), NFR-<id> (logging), NFR-<id> (no rollover)
2. [ ] Draft ADR for: Emergency stop architecture (tiers, state machine, triggers)
3. [ ] Draft ADR for: Stop tier selection policy (criteria for choosing tier)
4. [ ] Draft ADR for: Emergency stop recovery (clear mechanisms, timeout policy)
5. [ ] Create task for: Emergency stop implementation (Phase 1: Controlled stop + RC/GCS triggers)
6. [ ] Plan testing: Low-speed field tests, rollover risk assessment, stop time measurement

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **Aggressive stop tier**: Defer to Phase 2 (controlled and disarm sufficient initially)
- **PID stop controller**: P controller sufficient for Phase 1, upgrade to PID if needed
- **Advanced steering management**: Phase 1 uses simple steering lockout, defer active heading hold
- **Multiple velocity sources**: Phase 1 uses single velocity estimate (wheel encoder or GPS)
- **Terrain-adaptive stopping**: No surface-specific stop parameters (Phase 2 feature)
- **Stop performance monitoring**: No logging of deceleration rates or stop quality metrics
- **Automated stop testing**: Manual testing only in Phase 1, defer automated test framework
- **Stop visualization**: No telemetry visualization of stop progress (Phase 2 feature)
- **Redundant stop triggers**: No failover if primary trigger mechanism fails
- **Stop cancellation**: Once stop initiated, cannot cancel (must complete then clear)

## Appendix

### References

- ArduPilot Rover Mode Implementation: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.cpp>
- ArduPilot Motor Test: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/motor_test.cpp>
- ArduPilot Arming Library: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Arming/AP_Arming.cpp>
- MAVLink MAV_CMD_DO_MOTOR_TEST: <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOTOR_TEST>
- MAVLink MAV_CMD_COMPONENT_ARM_DISARM: <https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM>

### Raw Data

**ArduPilot stop_vehicle() Implementation** (from mode.cpp):

```cpp
bool Mode::stop_vehicle()
{
    bool stopped = false;

    if (rover.is_balancebot()) {
        float throttle_out = attitude_control.get_throttle_out_speed(
            0.0f, true, stopped);
        g2.motors.set_throttle(throttle_out);
    } else {
        float throttle_out = attitude_control.get_throttle_out_stop(
            true, stopped);
        g2.motors.set_throttle(throttle_out);
    }

    float steering_out = 0.0f;
    if (!stopped) {
        steering_out = attitude_control.get_steering_out_rate(
            0.0f, true);
    }
    g2.motors.set_steering(steering_out);

    g2.sailboat.relax(true);

    return stopped;
}
```

**Proposed Stop State Machine**:

```
                    ┌─────────────┐
                    │  OPERATING  │
                    └──────┬──────┘
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   RC switch          GCS command        Failsafe
   trigger            trigger            trigger
        │                  │                  │
        ▼                  ▼                  ▼
   ┌─────────────────────────────────────────────┐
   │          Start controlled stop              │
   │  - Set state = Stopping                     │
   │  - Record trigger source                    │
   │  - Start stop timer                         │
   └────────────────┬────────────────────────────┘
                    │
                    ▼
             ┌──────────────┐
             │   STOPPING   │
             │              │
             │ Every cycle: │
             │ - Calculate  │
             │   stop       │
             │   throttle   │
             │ - Maintain   │
             │   heading    │
             │ - Check      │
             │   velocity   │
             └──────┬───────┘
                    │
          Velocity < 0.1 m/s
                    │
                    ▼
             ┌──────────────┐
             │   STOPPED    │
             │              │
             │ - Throttle 0 │
             │ - Steering 0 │
             │ - Log event  │
             └──────┬───────┘
                    │
          Operator clear
          (RC or GCS)
                    │
                    ▼
             ┌──────────────┐
             │  OPERATING   │
             └──────────────┘
```
