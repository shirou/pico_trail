# AN-kajh6 Failsafe System for Safe Operation During Armed State

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-r2fps-pre-arm-checks](AN-r2fps-pre-arm-checks.md)
  - [AN-yqeju-manual-control-implementation](AN-yqeju-manual-control-implementation.md)
- Related Requirements:
  - [FR-0wy2c-failsafe-action-priority](../requirements/FR-0wy2c-failsafe-action-priority.md)
  - [FR-88a46-failsafe-parameters](../requirements/FR-88a46-failsafe-parameters.md)
  - [FR-lusbw-rc-signal-loss-failsafe](../requirements/FR-lusbw-rc-signal-loss-failsafe.md)
  - [FR-mg2bv-failsafe-integration](../requirements/FR-mg2bv-failsafe-integration.md)
  - [FR-nl9o0-battery-voltage-failsafe](../requirements/FR-nl9o0-battery-voltage-failsafe.md)
  - [FR-xex86-failsafe-capability-based-selection](../requirements/FR-xex86-failsafe-capability-based-selection.md)
  - [FR-xrlkn-failsafe-recovery](../requirements/FR-xrlkn-failsafe-recovery.md)
  - [FR-zwsr5-gcs-loss-failsafe](../requirements/FR-zwsr5-gcs-loss-failsafe.md)
  - [NFR-ao3x5-failsafe-event-logging](../requirements/NFR-ao3x5-failsafe-event-logging.md)
  - [NFR-az0iv-failsafe-action-execution-time](../requirements/NFR-az0iv-failsafe-action-execution-time.md)
  - [NFR-df1qu-failsafe-detection-latency](../requirements/NFR-df1qu-failsafe-detection-latency.md)
  - [NFR-qp6ya-no-false-failsafe-triggers](../requirements/NFR-qp6ya-no-false-failsafe-triggers.md)
  - [NFR-sm5dx-failsafe-ram-overhead](../requirements/NFR-sm5dx-failsafe-ram-overhead.md)
- Related ADRs: N/A - ADRs will be created based on this analysis
- Related Tasks: N/A - Tasks will be created after requirements and ADRs

## Executive Summary

This analysis explores failsafe mechanisms needed to ensure safe vehicle operation during armed state. Currently, pico_trail has no failsafe system - once armed, the vehicle continues operating indefinitely regardless of RC signal loss, GCS communication loss, battery depletion, or system failures. Failsafe systems are critical safety mechanisms that detect unsafe conditions during operation and take automated protective actions (Hold, RTL, Disarm) to prevent vehicle runaway, crashes, or damage.

Key findings: ArduPilot implements a comprehensive failsafe framework with multiple independent safety monitors checking RC connection (1.5s timeout), GCS heartbeat (5s timeout), battery voltage/capacity thresholds, EKF health, and crash detection. Each failsafe type triggers configurable actions with priority-based execution and fallback chains (e.g., SmartRTL → RTL → Hold). For pico_trail, a minimal viable failsafe system focusing on RC loss, GCS loss, and battery monitoring is recommended for initial implementation, with expansion to EKF and crash detection in subsequent phases.

## Problem Space

### Current State

The project currently has:

- **Armed state management**: `SystemState::arm()` and `SystemState::disarm()` methods in `src/communication/mavlink/state.rs:161-183`
- **Manual control**: RC_CHANNELS processing and actuator control during armed operation
- **No failsafe monitoring**: Once armed, no continuous safety checks occur
- **No timeout detection**: RC signal loss or GCS loss not detected
- **No automatic actions**: No automated response to unsafe conditions

Critical safety gaps:

- **RC loss undetected**: If RC_CHANNELS messages stop arriving, vehicle continues with last command
- **GCS loss undetected**: If GCS heartbeat stops, vehicle does not detect communication loss
- **Battery monitoring missing**: No detection of low battery voltage or capacity depletion
- **No automatic recovery**: Vehicle cannot automatically enter Hold or RTL on failsafe
- **Crash detection absent**: Cannot detect vehicle stuck or crashed condition
- **No failsafe configuration**: No parameters to configure failsafe behavior

### Desired State

Enable comprehensive failsafe system protecting against operational hazards:

1. **RC Loss Failsafe**: Detect RC signal timeout (no RC_CHANNELS for >1.5s) → trigger configured action
2. **GCS Loss Failsafe**: Detect GCS heartbeat timeout (no HEARTBEAT for >5s) → trigger configured action
3. **Battery Failsafe**: Monitor battery voltage and capacity → trigger action at LOW and CRITICAL thresholds
4. **Automatic Actions**: Execute Hold, RTL, or Disarm based on failsafe type and configuration
5. **Operator Notification**: Send STATUSTEXT messages to GCS reporting failsafe activation
6. **Failsafe Recovery**: Allow operator to regain control after failsafe clears
7. **Configuration**: Parameters to enable/disable failsafes and configure actions (FS\_\_, BATT\_\_)

Success criteria:

- **Safety-first**: Vehicle automatically protects itself when operator loses control
- **Reliable detection**: Failsafe triggers within specified timeout periods
- **Predictable actions**: Operator knows exactly what vehicle will do on failsafe
- **Configurable**: Failsafe actions can be customized per vehicle/mission requirements
- **Recoverable**: Operator can regain manual control when failsafe condition clears
- **Auditable**: All failsafe events logged for post-flight analysis

### Gap Analysis

**Missing components**:

1. **Failsafe Framework**: Infrastructure to register, monitor, and trigger multiple failsafe types
2. **RC Loss Detection**: Timeout detection for RC_CHANNELS messages
3. **GCS Loss Detection**: Timeout detection for GCS HEARTBEAT messages
4. **Battery Monitoring**: Voltage and capacity threshold checking
5. **Failsafe Actions**: Implementation of Hold mode, RTL mode, and automatic disarm
6. **Action Priority System**: Priority-based execution when multiple failsafes trigger simultaneously
7. **Failsafe Parameters**: Configuration for timeouts, thresholds, and actions
8. **Failsafe Logging**: Event recording for post-flight analysis
9. **Failsafe Notification**: STATUSTEXT messages to GCS

**Technical deltas**:

- Add `src/vehicle/failsafe/` module with failsafe framework
- Implement failsafe monitors: RC loss, GCS loss, battery
- Create timeout tracking for RC_CHANNELS and HEARTBEAT messages
- Implement battery voltage and capacity monitoring
- Add Hold mode implementation (stop vehicle, maintain position if GPS available)
- Add RTL mode implementation (return to launch point)
- Create failsafe action executor with priority system
- Add parameters: `FS_TIMEOUT`, `FS_ACTION`, `FS_GCS_TIMEOUT`, `FS_GCS_ACTION`, `BATT_LOW_VOLT`, `BATT_CRT_VOLT`, etc.
- Integrate failsafe checking into vehicle control task (10 Hz)
- Log failsafe events to storage
- Send STATUSTEXT messages on failsafe activation/recovery

## Stakeholder Analysis

| Stakeholder         | Interest/Need                                           | Impact | Priority |
| ------------------- | ------------------------------------------------------- | ------ | -------- |
| Operators           | Prevent vehicle runaway, automatic safe landing         | High   | P0       |
| Safety Reviewers    | Ensure vehicle cannot continue operation when unsafe    | High   | P0       |
| Test Engineers      | Predictable vehicle behavior during communication loss  | High   | P0       |
| Developers          | Debug communication issues via failsafe logs            | High   | P0       |
| System Integrators  | Validate failsafe behavior before autonomous operation  | High   | P0       |
| Autonomous Features | Failsafe is prerequisite for autonomous flight approval | High   | P0       |

## Research & Discovery

### User Feedback

From operational safety requirements:

- Failsafe system is mandatory for any autonomous vehicle
- RC loss is the most common failsafe trigger during testing
- Battery failsafe prevents expensive crashes from over-discharge
- GCS loss failsafe critical for autonomous missions beyond RC range
- Failsafe actions must be predictable and configurable per mission type

### Competitive Analysis

**ArduPilot Rover Failsafe System**:

ArduPilot implements failsafe monitoring in `Rover/failsafe.cpp`:

#### 1. RC Loss Failsafe (`FAILSAFE_EVENT_THROTTLE`)

```cpp
void Rover::failsafe_trigger(uint8_t failsafe_type, const char *type_str,
                               bool radio_timeout)
{
    // Check timeout
    if (millis() - failsafe.start_time > g.fs_timeout * 1000) {
        // Trigger failsafe action
        if (g.fs_action == Failsafe_Action_None) {
            return;  // No action
        } else if (g.fs_action == Failsafe_Action_RTL) {
            set_mode(mode_rtl, ModeReason::FAILSAFE);
        } else if (g.fs_action == Failsafe_Action_Hold) {
            set_mode(mode_hold, ModeReason::FAILSAFE);
        } else if (g.fs_action == Failsafe_Action_SmartRTL) {
            // Try SmartRTL, fallback to RTL, fallback to Hold
            if (!set_mode(mode_smart_rtl, ModeReason::FAILSAFE)) {
                if (!set_mode(mode_rtl, ModeReason::FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::FAILSAFE);
                }
            }
        }
    }
}
```

**Configuration**:

- `FS_TIMEOUT`: Failsafe timeout in seconds (default: 1.5s)
- `FS_THR_ENABLE`: Enable/disable RC failsafe (0=disabled, 1=enabled)
- `FS_ACTION`: Action to take (0=None, 1=Hold, 2=RTL, 3=SmartRTL, 4=SmartRTL_Hold, 5=Terminate)

**Detection**: RC_CHANNELS message not received within FS_TIMEOUT seconds

#### 2. GCS Loss Failsafe (`FAILSAFE_EVENT_GCS`)

Same timeout mechanism as RC loss, but with separate parameters:

- `FS_GCS_TIMEOUT`: GCS failsafe timeout in seconds (default: 5.0s)
- `FS_GCS_ENABLE`: Enable/disable GCS failsafe (0=disabled, 1=enabled)

**Detection**: GCS HEARTBEAT message not received within FS_GCS_TIMEOUT seconds

**Special handling**: Disabled in RTL and Hold modes (unless `FS_OPTIONS` bit 0 set)

#### 3. Battery Failsafe (`handle_battery_failsafe()`)

```cpp
void Rover::handle_battery_failsafe(const char *type_str,
                                      const int8_t action)
{
    // Action options: None, RTL, Hold, SmartRTL, SmartRTL_Hold, Terminate
    switch (action) {
        case Failsafe_Action_None:
            return;
        case Failsafe_Action_SmartRTL:
            if (set_mode(mode_smart_rtl, ModeReason::BATTERY_FAILSAFE)) {
                return;
            }
            [[fallthrough]];  // Try next action
        case Failsafe_Action_RTL:
            if (set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE)) {
                return;
            }
            [[fallthrough]];
        case Failsafe_Action_Hold:
            set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
            return;
        case Failsafe_Action_Terminate:
            arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
            return;
    }
}
```

**Configuration**:

- `BATT_LOW_VOLT`: Low battery voltage threshold (e.g., 10.5V for 3S LiPo)
- `BATT_CRT_VOLT`: Critical battery voltage threshold (e.g., 10.0V for 3S LiPo)
- `BATT_LOW_MAH`: Low battery capacity threshold (mAh remaining)
- `BATT_CRT_MAH`: Critical battery capacity threshold (mAh remaining)
- `BATT_FS_LOW_ACT`: Action on low battery (0=None, 1=Hold, 2=RTL, 3=SmartRTL, 4=SmartRTL_Hold, 5=Terminate)
- `BATT_FS_CRT_ACT`: Action on critical battery

**Detection**: Battery voltage below threshold for 10+ seconds, or capacity below threshold

**Two-stage system**:

1. **Low battery**: Warning threshold → less aggressive action (e.g., RTL)
2. **Critical battery**: Emergency threshold → more aggressive action (e.g., immediate land/disarm)

#### 4. CPU Failsafe (`failsafe_check()`)

```cpp
void Rover::failsafe_check()
{
    // Check main loop lockup
    static uint32_t last_ticks = 0;
    uint32_t ticks = AP::scheduler().ticks();
    if (ticks != last_ticks) {
        last_ticks = ticks;
        return;
    }

    // Main loop hasn't executed for >200ms → forced disarm
    if (AP_HAL::millis() - last_ticks > 200) {
        arming.disarm(AP_Arming::Method::CPUFAILSAFE);
    }
}
```

**Detection**: Scheduler ticks not advancing (main loop frozen)

**Action**: Immediate disarm (cannot trust mode switching if CPU locked)

#### 5. Crash Detection (`crash_check()`)

Detection criteria (all must be true):

- Velocity < 0.08 m/s (essentially stopped)
- Minimal turning (attitude stable)
- Throttle > 5% (driver still commanding movement)
- In autonomous mode (Auto, Guided, RTL)

**Action**: Switch to Hold mode, optionally disarm

**Configuration**:

- `FS_CRASH_CHECK`: Action (0=disabled, 1=Hold, 2=Hold+Disarm)

**ArduPilot Failsafe Parameters Summary**:

| Parameter       | Type  | Default | Description                                       |
| --------------- | ----- | ------- | ------------------------------------------------- |
| FS_TIMEOUT      | float | 1.5     | RC failsafe timeout (seconds)                     |
| FS_THR_ENABLE   | u8    | 1       | Enable RC failsafe (0=off, 1=on)                  |
| FS_ACTION       | u8    | 2       | RC failsafe action (0=None, 1=Hold, 2=RTL, etc.)  |
| FS_GCS_ENABLE   | u8    | 0       | Enable GCS failsafe (0=off, 1=on)                 |
| FS_GCS_TIMEOUT  | float | 5.0     | GCS failsafe timeout (seconds)                    |
| BATT_LOW_VOLT   | float | 0       | Low battery voltage threshold (0=disabled)        |
| BATT_CRT_VOLT   | float | 0       | Critical battery voltage threshold                |
| BATT_LOW_MAH    | float | 0       | Low battery capacity threshold (mAh, 0=disabled)  |
| BATT_CRT_MAH    | float | 0       | Critical battery capacity threshold               |
| BATT_FS_LOW_ACT | u8    | 0       | Low battery action                                |
| BATT_FS_CRT_ACT | u8    | 0       | Critical battery action                           |
| FS_CRASH_CHECK  | u8    | 0       | Crash detection action (0=off, 1=Hold, 2=Disarm)  |
| FS_OPTIONS      | u32   | 0       | Failsafe options bitmask                          |
| FS_EKF_ACTION   | u8    | 1       | EKF failsafe action (0=disabled, 1=Hold, 2=Disarm |

**PX4 Failsafe System**:

Similar architecture but different implementation:

- **Data Link Loss**: 10s timeout (DL_LOSS_T parameter)
- **RC Loss**: 0.5s timeout (COM_RC_LOSS_T parameter)
- **Battery Failsafe**: Percentage-based thresholds (BAT_LOW_THR, BAT_CRIT_THR)
- **Return mode**: RTL with configurable altitude and speed
- **Land mode**: Controlled landing at current position
- **Commander state machine**: Centralized failsafe management

### Technical Investigation

**Current pico_trail RC Monitoring**:

File: `src/communication/mavlink/handlers/telemetry.rs` (RC_CHANNELS handler exists but no timeout tracking)

Current flow:

1. Receive RC_CHANNELS message
2. Parse channel values
3. No timestamp recording
4. No timeout detection

**No continuous monitoring**:

- No task checking for RC message age
- No failsafe state machine
- No timeout parameters

**Proposed Failsafe Framework Architecture**:

```rust
/// Failsafe type enumeration
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum FailsafeType {
    None,
    RcLoss,
    GcsLoss,
    BatteryLow,
    BatteryCritical,
    CpuFailure,
    EkfFailure,
    CrashDetected,
}

/// Failsafe action enumeration
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum FailsafeAction {
    None,          // No action, just warn
    Hold,          // Stop and hold position
    Rtl,           // Return to launch
    SmartRtl,      // Return via intelligent path
    SmartRtlHold,  // Try SmartRTL, fallback to Hold
    Disarm,        // Immediate disarm
    Terminate,     // Motor cutoff (emergency only)
}

/// Failsafe state tracking
pub struct FailsafeState {
    /// Currently active failsafe (highest priority)
    active_failsafe: FailsafeType,

    /// RC loss tracking
    last_rc_message_ms: u32,
    rc_failsafe_triggered: bool,

    /// GCS loss tracking
    last_gcs_heartbeat_ms: u32,
    gcs_failsafe_triggered: bool,

    /// Battery failsafe tracking
    battery_failsafe_triggered: bool,
    battery_failsafe_level: FailsafeType,  // Low or Critical

    /// Failsafe activation timestamp
    failsafe_start_time_ms: u32,
}

impl FailsafeState {
    /// Check if RC failsafe should trigger
    pub fn check_rc_failsafe(&mut self, params: &FailsafeParams) -> bool {
        if !params.fs_thr_enable {
            return false;  // RC failsafe disabled
        }

        let age_ms = self.current_time_ms() - self.last_rc_message_ms;
        let timeout_ms = (params.fs_timeout * 1000.0) as u32;

        if age_ms > timeout_ms {
            if !self.rc_failsafe_triggered {
                // First trigger
                self.rc_failsafe_triggered = true;
                self.failsafe_start_time_ms = self.current_time_ms();
                self.active_failsafe = FailsafeType::RcLoss;
                return true;
            }
        } else {
            // RC signal recovered
            if self.rc_failsafe_triggered {
                self.rc_failsafe_triggered = false;
                if self.active_failsafe == FailsafeType::RcLoss {
                    self.active_failsafe = FailsafeType::None;
                }
            }
        }

        false
    }

    /// Check if GCS failsafe should trigger
    pub fn check_gcs_failsafe(&mut self, params: &FailsafeParams) -> bool {
        if !params.fs_gcs_enable {
            return false;  // GCS failsafe disabled
        }

        let age_ms = self.current_time_ms() - self.last_gcs_heartbeat_ms;
        let timeout_ms = (params.fs_gcs_timeout * 1000.0) as u32;

        if age_ms > timeout_ms {
            if !self.gcs_failsafe_triggered {
                // First trigger
                self.gcs_failsafe_triggered = true;
                self.failsafe_start_time_ms = self.current_time_ms();
                // GCS failsafe lower priority than RC loss
                if self.active_failsafe == FailsafeType::None {
                    self.active_failsafe = FailsafeType::GcsLoss;
                }
                return true;
            }
        } else {
            // GCS recovered
            if self.gcs_failsafe_triggered {
                self.gcs_failsafe_triggered = false;
                if self.active_failsafe == FailsafeType::GcsLoss {
                    self.active_failsafe = FailsafeType::None;
                }
            }
        }

        false
    }

    /// Check battery failsafe
    pub fn check_battery_failsafe(&mut self, battery: &BatteryState,
                                    params: &FailsafeParams) -> Option<FailsafeType> {
        // Check critical voltage (higher priority)
        if params.batt_crt_volt > 0.0 && battery.voltage < params.batt_crt_volt {
            if self.battery_failsafe_level != FailsafeType::BatteryCritical {
                self.battery_failsafe_level = FailsafeType::BatteryCritical;
                self.active_failsafe = FailsafeType::BatteryCritical;
                return Some(FailsafeType::BatteryCritical);
            }
        }
        // Check low voltage
        else if params.batt_low_volt > 0.0 && battery.voltage < params.batt_low_volt {
            if self.battery_failsafe_level != FailsafeType::BatteryLow {
                self.battery_failsafe_level = FailsafeType::BatteryLow;
                // Only override if no higher priority failsafe
                if self.active_failsafe == FailsafeType::None {
                    self.active_failsafe = FailsafeType::BatteryLow;
                }
                return Some(FailsafeType::BatteryLow);
            }
        } else {
            // Battery recovered
            if self.battery_failsafe_level != FailsafeType::None {
                self.battery_failsafe_level = FailsafeType::None;
                // Clear active failsafe if it was battery
                if matches!(self.active_failsafe,
                            FailsafeType::BatteryLow | FailsafeType::BatteryCritical) {
                    self.active_failsafe = FailsafeType::None;
                }
            }
        }

        None
    }

    /// Update RC message timestamp (called when RC_CHANNELS received)
    pub fn update_rc_timestamp(&mut self) {
        self.last_rc_message_ms = self.current_time_ms();
    }

    /// Update GCS heartbeat timestamp (called when HEARTBEAT received)
    pub fn update_gcs_timestamp(&mut self) {
        self.last_gcs_heartbeat_ms = self.current_time_ms();
    }
}

/// Failsafe parameters (loaded from parameter storage)
pub struct FailsafeParams {
    /// RC failsafe timeout (seconds)
    pub fs_timeout: f32,

    /// RC failsafe enable
    pub fs_thr_enable: bool,

    /// RC failsafe action
    pub fs_action: FailsafeAction,

    /// GCS failsafe timeout (seconds)
    pub fs_gcs_timeout: f32,

    /// GCS failsafe enable
    pub fs_gcs_enable: bool,

    /// Battery low voltage threshold
    pub batt_low_volt: f32,

    /// Battery critical voltage threshold
    pub batt_crt_volt: f32,

    /// Low battery action
    pub batt_fs_low_act: FailsafeAction,

    /// Critical battery action
    pub batt_fs_crt_act: FailsafeAction,
}

impl Default for FailsafeParams {
    fn default() -> Self {
        Self {
            fs_timeout: 1.5,           // 1.5 seconds RC timeout
            fs_thr_enable: true,       // RC failsafe enabled by default
            fs_action: FailsafeAction::Hold,  // Default to Hold on RC loss
            fs_gcs_timeout: 5.0,       // 5 seconds GCS timeout
            fs_gcs_enable: false,      // GCS failsafe disabled by default
            batt_low_volt: 0.0,        // Disabled (must configure per battery)
            batt_crt_volt: 0.0,        // Disabled
            batt_fs_low_act: FailsafeAction::None,
            batt_fs_crt_act: FailsafeAction::None,
        }
    }
}

/// Failsafe action executor
pub struct FailsafeExecutor {
    state: FailsafeState,
    params: FailsafeParams,
}

impl FailsafeExecutor {
    /// Execute failsafe checks (called at 10 Hz from vehicle task)
    pub fn update(&mut self, battery: &BatteryState, mode_manager: &mut ModeManager)
                  -> Result<(), &'static str> {
        // Check RC failsafe
        if self.state.check_rc_failsafe(&self.params) {
            self.execute_action(FailsafeType::RcLoss, self.params.fs_action, mode_manager)?;
            self.send_notification("RC Lost", FailsafeType::RcLoss)?;
        }

        // Check GCS failsafe
        if self.state.check_gcs_failsafe(&self.params) {
            self.execute_action(FailsafeType::GcsLoss, self.params.fs_action, mode_manager)?;
            self.send_notification("GCS Lost", FailsafeType::GcsLoss)?;
        }

        // Check battery failsafe
        if let Some(battery_failsafe) = self.state.check_battery_failsafe(battery, &self.params) {
            let action = match battery_failsafe {
                FailsafeType::BatteryLow => self.params.batt_fs_low_act,
                FailsafeType::BatteryCritical => self.params.batt_fs_crt_act,
                _ => FailsafeAction::None,
            };
            self.execute_action(battery_failsafe, action, mode_manager)?;
            self.send_notification("Battery Low", battery_failsafe)?;
        }

        Ok(())
    }

    /// Execute failsafe action with fallback chain
    fn execute_action(&mut self, failsafe_type: FailsafeType,
                       action: FailsafeAction, mode_manager: &mut ModeManager)
                       -> Result<(), &'static str> {
        match action {
            FailsafeAction::None => {
                // Just log, no action
                info!("Failsafe triggered: {:?}, action: None", failsafe_type);
                Ok(())
            }
            FailsafeAction::Hold => {
                mode_manager.set_mode(FlightMode::Hold, ModeReason::Failsafe)
            }
            FailsafeAction::Rtl => {
                mode_manager.set_mode(FlightMode::Rtl, ModeReason::Failsafe)
            }
            FailsafeAction::SmartRtl => {
                // Try SmartRTL, fallback to RTL, fallback to Hold
                if mode_manager.set_mode(FlightMode::SmartRtl, ModeReason::Failsafe).is_ok() {
                    Ok(())
                } else if mode_manager.set_mode(FlightMode::Rtl, ModeReason::Failsafe).is_ok() {
                    Ok(())
                } else {
                    mode_manager.set_mode(FlightMode::Hold, ModeReason::Failsafe)
                }
            }
            FailsafeAction::SmartRtlHold => {
                // Try SmartRTL, fallback to Hold
                if mode_manager.set_mode(FlightMode::SmartRtl, ModeReason::Failsafe).is_ok() {
                    Ok(())
                } else {
                    mode_manager.set_mode(FlightMode::Hold, ModeReason::Failsafe)
                }
            }
            FailsafeAction::Disarm => {
                // Immediate disarm (emergency only)
                mode_manager.system_state.disarm()
            }
            FailsafeAction::Terminate => {
                // Motor cutoff (most aggressive, emergency only)
                // TODO: Implement motor termination
                Err("Terminate action not yet implemented")
            }
        }
    }

    /// Send failsafe notification to GCS
    fn send_notification(&self, message: &str, failsafe_type: FailsafeType)
                          -> Result<(), &'static str> {
        // TODO: Send STATUSTEXT message via MAVLink router
        // Format: "Failsafe: {message}"
        // Severity: MAV_SEVERITY_CRITICAL
        Ok(())
    }
}
```

**Integration with Vehicle Control Task**:

Failsafe checking runs at 10 Hz (medium frequency):

```rust
// In vehicle control task (10 Hz)
pub fn vehicle_control_task() {
    let mut failsafe_executor = FailsafeExecutor::new();
    let mut last_failsafe_check_ms = 0;

    loop {
        // ... other vehicle control logic ...

        // Check failsafes at 10 Hz
        if (current_time_ms - last_failsafe_check_ms) >= 100 {
            if let Err(e) = failsafe_executor.update(&battery_state, &mut mode_manager) {
                error!("Failsafe execution failed: {}", e);
            }
            last_failsafe_check_ms = current_time_ms;
        }

        delay_ms(20);  // 50 Hz control loop
    }
}
```

**Memory Analysis**:

| Component            | RAM Usage   | Notes                              |
| -------------------- | ----------- | ---------------------------------- |
| FailsafeState        | \~40 B      | Timestamps + flags                 |
| FailsafeParams       | \~32 B      | Configuration values               |
| FailsafeExecutor     | \~80 B      | State + params + execution context |
| **Total (initial)**  | **\~150 B** | Minimal overhead for safety        |
| **Total (all types** | **\~200 B** | With all failsafe types enabled    |

### Data Analysis

**Failsafe Trigger Rates** (estimated from ArduPilot community feedback):

- **RC loss**: 10-20% of flights experience brief RC loss (< 3 seconds)
- **GCS loss**: 30-40% of autonomous flights lose GCS connection temporarily
- **Battery failsafe**: 5-10% of flights trigger low battery warning
- **Critical battery**: < 1% of flights reach critical battery (indicates improper planning)
- **Crash detection**: 1-2% of autonomous missions trigger crash detection

**Timeout Sensitivity Analysis**:

- **Too short** (< 0.5s): False triggers from brief signal dropouts, unnecessary failsafe activation
- **Optimal** (1.0-2.0s): Balance between safety and false trigger avoidance
- **Too long** (> 5s): Vehicle travels significant distance before failsafe, higher crash risk

**Recommended Defaults**:

- RC timeout: 1.5s (ArduPilot default, proven in field)
- GCS timeout: 5.0s (longer because GCS is not critical for manual control)
- Battery low: 10.5V for 3S LiPo, 14.0V for 4S LiPo (vehicle-specific)
- Battery critical: 10.0V for 3S, 13.2V for 4S (minimum safe voltage)

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The system shall detect RC signal loss and trigger failsafe action after configured timeout → Will become FR-<id>
  - Rationale: Prevent vehicle runaway when operator loses RC control (most common failsafe scenario)
  - Acceptance Criteria:
    - Monitor RC_CHANNELS message arrival time
    - Trigger failsafe if no RC_CHANNELS received within FS_TIMEOUT seconds (default 1.5s)
    - Execute configured failsafe action (Hold, RTL, SmartRTL, or Disarm)
    - Send STATUSTEXT message: "Failsafe: RC Lost"
    - Log failsafe event with timestamp and action taken
    - Clear failsafe when RC signal resumes

- [ ] **FR-DRAFT-2**: The system shall detect GCS communication loss and trigger failsafe action after configured timeout → Will become FR-<id>
  - Rationale: Ensure vehicle safety during autonomous operation when GCS link fails
  - Acceptance Criteria:
    - Monitor GCS HEARTBEAT message arrival time
    - Trigger failsafe if no HEARTBEAT received within FS_GCS_TIMEOUT seconds (default 5.0s)
    - Execute configured failsafe action (typically RTL or Hold)
    - Do not trigger in RTL or Hold modes (already safe)
    - Send STATUSTEXT message: "Failsafe: GCS Lost"
    - Log failsafe event
    - Clear failsafe when GCS heartbeat resumes

- [ ] **FR-DRAFT-3**: The system shall monitor battery voltage and capacity and trigger failsafe at LOW and CRITICAL thresholds → Will become FR-<id>
  - Rationale: Prevent battery over-discharge and ensure sufficient power for safe landing/RTL
  - Acceptance Criteria:
    - Check battery voltage every 100ms (10 Hz)
    - Trigger LOW failsafe when voltage < BATT_LOW_VOLT for 10+ seconds
    - Trigger CRITICAL failsafe when voltage < BATT_CRT_VOLT (immediate, no delay)
    - Execute configured action per threshold (LOW: RTL, CRITICAL: Hold+Land)
    - Two-stage system: less aggressive action at LOW, more aggressive at CRITICAL
    - Check capacity (mAh) if BATT_LOW_MAH/BATT_CRT_MAH configured
    - Send STATUSTEXT messages: "Failsafe: Battery Low" / "Battery Critical"
    - Log battery voltage, capacity, and action at failsafe trigger

- [ ] **FR-DRAFT-4**: The system shall execute failsafe actions with priority-based selection and automatic fallback → Will become FR-<id>
  - Rationale: Handle multiple simultaneous failsafes, ensure action always succeeds
  - Acceptance Criteria:
    - Execute highest priority failsafe action when multiple failsafes active
    - Priority order: RC loss > Battery critical > Battery low > GCS loss
    - Fallback chain for SmartRTL: SmartRTL → RTL → Hold
    - If action fails (e.g., RTL requires GPS but no GPS), try fallback
    - Always succeed (Hold mode requires no sensors, always available)
    - Log all attempted actions and final action executed

- [ ] **FR-DRAFT-5**: The system shall allow failsafe recovery when failsafe condition clears → Will become FR-<id>
  - Rationale: Enable operator to regain manual control when RC signal resumes
  - Acceptance Criteria:
    - Clear RC failsafe when RC_CHANNELS messages resume (within timeout)
    - Clear GCS failsafe when HEARTBEAT messages resume
    - Do not automatically switch back to previous mode (operator must manually switch)
    - Send recovery notification: "Failsafe: RC Recovered" / "GCS Recovered"
    - Battery failsafe does not auto-clear (voltage recovery too slow, action must complete)
    - Log failsafe recovery event with duration

- [ ] **FR-DRAFT-6**: The system shall provide configurable failsafe parameters for timeouts, thresholds, and actions → Will become FR-<id>
  - Rationale: Allow customization per vehicle type, mission profile, and operator preference
  - Acceptance Criteria:
    - Parameter: FS_TIMEOUT (float, 0.0-10.0s, default 1.5s) - RC failsafe timeout
    - Parameter: FS_THR_ENABLE (bool, default true) - Enable RC failsafe
    - Parameter: FS_ACTION (enum, default Hold) - RC failsafe action
    - Parameter: FS_GCS_TIMEOUT (float, 0.0-30.0s, default 5.0s) - GCS failsafe timeout
    - Parameter: FS_GCS_ENABLE (bool, default false) - Enable GCS failsafe
    - Parameter: BATT_LOW_VOLT (float, 0.0-30.0V, default 0.0=disabled)
    - Parameter: BATT_CRT_VOLT (float, 0.0-30.0V, default 0.0=disabled)
    - Parameter: BATT_FS_LOW_ACT (enum, default None) - Low battery action
    - Parameter: BATT_FS_CRT_ACT (enum, default None) - Critical battery action
    - Parameters persist to storage, survive reboot

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Failsafe detection latency shall not exceed 200ms after timeout threshold → Will become NFR-<id>
  - Category: Performance
  - Rationale: Timely failsafe activation critical for preventing uncontrolled flight
  - Target: Detect failsafe within 200ms of timeout expiring (10 Hz check rate sufficient)

- [ ] **NFR-DRAFT-2**: Failsafe action execution shall complete within 1 second of trigger → Will become NFR-<id>
  - Category: Performance
  - Rationale: Rapid response needed to prevent crash or runaway
  - Target: Mode change to Hold/RTL completes < 1s, includes actuator response

- [ ] **NFR-DRAFT-3**: Failsafe system shall add no more than 200 bytes RAM overhead → Will become NFR-<id>
  - Category: Resource Constraints
  - Rationale: Maintain memory budget for other subsystems on RP2040/RP2350
  - Target: < 200 B for failsafe state + parameters (measured via runtime profiling)

- [ ] **NFR-DRAFT-4**: Failsafe events shall be logged to persistent storage for post-flight analysis → Will become NFR-<id>
  - Category: Reliability / Auditability
  - Rationale: Support debugging and safety investigations
  - Target: All failsafe triggers and recoveries logged with timestamp, type, action, duration

- [ ] **NFR-DRAFT-5**: Failsafe system shall not introduce false triggers under normal operation → Will become NFR-<id>
  - Category: Reliability
  - Rationale: False failsafe activation disrupts missions, reduces operator confidence
  - Target: < 0.1% false trigger rate (measured via test flights, no triggers when signals healthy)

## Design Considerations

### Technical Constraints

- **Existing architecture**: Must integrate with current vehicle control task, mode manager, and MAVLink layer
- **Memory budget**: Limited RAM on RP2040/RP2350 (264 KB) - failsafe system must be lightweight
- **Real-time constraints**: Failsafe checking at 10 Hz, mode switching at 50 Hz
- **Safety critical**: False negatives UNACCEPTABLE (miss real failsafe), false positives should be minimized but acceptable
- **Platform abstraction**: Must work on both RP2040 and RP2350
- **No dynamic allocation**: Failsafe framework must use static/stack allocation only
- **Mode dependencies**: Hold and RTL modes must be implemented before failsafe system can function

### Potential Approaches

1. **Option A: Simple Timeout Checkers (No Framework)**
   - Pros:
     - Simplest implementation (individual timeout functions)
     - Zero abstraction overhead
     - Easy to understand and debug
     - Minimal memory footprint (\~50 B)
   - Cons:
     - Hard to extend (adding new failsafe types requires modifying vehicle task)
     - No priority system (all failsafes treated equally)
     - Difficult to test failsafe interactions
     - Code duplication across failsafe types
   - Effort: Low (4-8 hours)

2. **Option B: Centralized Failsafe Manager** ⭐ Recommended
   - Pros:
     - Extensible: New failsafe types added via registration
     - Priority-based: Higher priority failsafes override lower priority
     - Testable: Failsafe logic isolated, can unit test
     - Matches ArduPilot architecture (proven, familiar)
     - Fallback chains supported (SmartRTL → RTL → Hold)
     - Clear state tracking and logging
   - Cons:
     - More upfront design effort
     - Slightly higher memory (\~150-200 B)
   - Effort: Medium (16-24 hours)

3. **Option C: Full ArduPilot-Style Failsafe System with All Features**
   - Pros:
     - Maximum feature parity with ArduPilot
     - Advanced features: crash detection, EKF monitoring, CPU failsafe
     - Sophisticated priority and override system
     - Comprehensive logging and diagnostics
   - Cons:
     - High complexity
     - Higher memory overhead (\~500 B - 1 KB)
     - Overkill for initial implementation
     - Long development time
   - Effort: High (40-60 hours)

**Recommendation**: Option B (Centralized Failsafe Manager) provides best balance of safety, extensibility, and development effort for initial implementation. Option C features (crash detection, EKF failsafe, CPU failsafe) can be added incrementally if needed.

### Failsafe Implementation Strategy

**Phase 1: Core Failsafe System** (Initial Implementation)

- RC loss failsafe (timeout detection, Hold action)
- GCS loss failsafe (timeout detection, Hold action)
- Battery failsafe (voltage monitoring, two-stage thresholds)
- Hold mode implementation (stop vehicle, maintain heading)
- Basic failsafe parameters (FS\_\_, BATT\_\_)
- STATUSTEXT notifications

**Phase 2: Advanced Actions** (Future Expansion)

- RTL mode implementation (return to launch point)
- SmartRTL mode (return via recorded path)
- Fallback chains (SmartRTL → RTL → Hold)
- Disarm action (automatic disarm on failsafe)

**Phase 3: Advanced Detection** (Future Expansion)

- Crash detection (velocity + throttle monitoring)
- EKF failsafe (position estimate health)
- CPU failsafe (watchdog monitoring)
- Geofence violation detection

### Architecture Impact

**New ADRs required**:

- **ADR-<id> Failsafe System Architecture**: Manager design, priority system, fallback chains
- **ADR-<id> Failsafe Parameters**: Which parameters to implement, default values, validation
- **ADR-<id> Hold Mode Implementation**: Stop behavior, heading hold, GPS position hold (if available)

**New modules**:

- `src/vehicle/failsafe/` - Failsafe system
  - `src/vehicle/failsafe/types.rs` - FailsafeType, FailsafeAction enums
  - `src/vehicle/failsafe/state.rs` - FailsafeState tracking
  - `src/vehicle/failsafe/executor.rs` - FailsafeExecutor manager
  - `src/vehicle/failsafe/checkers/` - Individual failsafe checkers
    - `src/vehicle/failsafe/checkers/rc_loss.rs`
    - `src/vehicle/failsafe/checkers/gcs_loss.rs`
    - `src/vehicle/failsafe/checkers/battery.rs`
- `src/vehicle/modes/hold.rs` - Hold mode implementation
- `src/vehicle/modes/rtl.rs` - RTL mode implementation (Phase 2)

**Modified modules**:

- `src/vehicle/mode_manager.rs` - Add ModeReason::Failsafe, priority-based mode switching
- `src/communication/mavlink/handlers/telemetry.rs` - Update RC timestamp on RC_CHANNELS
- `src/communication/mavlink/router.rs` - Update GCS timestamp on HEARTBEAT
- `src/core/parameters/` - Add failsafe parameters (FS\_\_, BATT\_\_)
- `src/vehicle/control_task.rs` - Integrate failsafe checking at 10 Hz

## ArduPilot Parameters

This analysis references the following ArduPilot Rover parameters that inform pico_trail's failsafe system design:

### RC Failsafe Parameters

- **FS_TIMEOUT** (float, default 1.5s, range 0.0-10.0s)
  - RC failsafe timeout in seconds
  - Triggers failsafe when no RC_CHANNELS message received within timeout
  - ArduPilot default: 1.5s (proven in field operations)

- **FS_THR_ENABLE** (u8, default 1, range 0-1)
  - Enable/disable RC failsafe
  - 0 = disabled, 1 = enabled
  - ArduPilot default: 1 (enabled)

- **FS_ACTION** (u8, default 2, range 0-5)
  - RC failsafe action to execute
  - 0 = None, 1 = Hold, 2 = RTL, 3 = SmartRTL, 4 = SmartRTL_Hold, 5 = Terminate
  - ArduPilot default: 2 (RTL)

### GCS Failsafe Parameters

- **FS_GCS_ENABLE** (u8, default 0, range 0-1)
  - Enable/disable GCS heartbeat failsafe
  - 0 = disabled, 1 = enabled
  - ArduPilot default: 0 (disabled by default, requires explicit opt-in)

- **FS_GCS_TIMEOUT** (float, default 5.0s, range 0.0-30.0s)
  - GCS failsafe timeout in seconds
  - Triggers failsafe when no GCS HEARTBEAT received within timeout
  - ArduPilot default: 5.0s (longer than RC to account for telemetry delays)

### Battery Failsafe Parameters

- **BATT_LOW_VOLT** (float, default 0.0, range 0.0-30.0V)
  - Low battery voltage threshold
  - 0.0 = disabled (must be configured per battery type)
  - Example: 10.5V for 3S LiPo (3.5V per cell)

- **BATT_CRT_VOLT** (float, default 0.0, range 0.0-30.0V)
  - Critical battery voltage threshold
  - Example: 10.0V for 3S LiPo (3.33V per cell minimum safe voltage)

- **BATT_LOW_MAH** (float, default 0.0, range 0.0-∞)
  - Low battery capacity threshold in mAh
  - 0.0 = disabled
  - Example: 660 mAh remaining (20% of 3300mAh battery)

- **BATT_CRT_MAH** (float, default 0.0, range 0.0-∞)
  - Critical battery capacity threshold in mAh
  - Example: 330 mAh remaining (10% of 3300mAh battery)

- **BATT_FS_LOW_ACT** (u8, default 0, range 0-5)
  - Action on low battery detection
  - 0 = None, 1 = Hold, 2 = RTL, 3 = SmartRTL, 4 = SmartRTL_Hold, 5 = Terminate
  - ArduPilot default: 0 (no action, warning only)

- **BATT_FS_CRT_ACT** (u8, default 0, range 0-5)
  - Action on critical battery detection
  - More aggressive than low battery action
  - ArduPilot default: 0 (no action, requires configuration)

- **BATT_CAPACITY** (float, default varies, range 0.0-∞)
  - Battery capacity in mAh
  - Required for capacity-based failsafe (BATT_LOW_MAH, BATT_CRT_MAH)
  - Example: 3300 mAh for typical 3S LiPo

### Advanced Failsafe Parameters

- **FS_CRASH_CHECK** (u8, default 0, range 0-2)
  - Crash detection action
  - 0 = disabled, 1 = Hold, 2 = Hold + Disarm
  - Detects vehicle stuck condition (velocity < 0.08 m/s with throttle > 5%)

- **FS_OPTIONS** (u32, default 0, bitmask)
  - Failsafe options bitmask for advanced behavior
  - Bit 0: Continue GCS failsafe in RTL/Hold modes if set

- **FS_EKF_ACTION** (u8, default 1, range 0-2)
  - EKF failsafe action when position estimate fails
  - 0 = disabled, 1 = Hold, 2 = Disarm
  - ArduPilot default: 1 (Hold mode on EKF failure)

### Proposed pico_trail Parameters

Core parameters (Phase 1):

- **FS_TIMEOUT** (float, default 1.5s, range 0.0-10.0s): RC failsafe timeout
- **FS_THR_ENABLE** (bool, default true): Enable RC failsafe
- **FS_ACTION** (u8, default 1=Hold): RC failsafe action (0=None, 1=Hold, 2=RTL, etc.)
- **FS_GCS_TIMEOUT** (float, default 5.0s, range 0.0-30.0s): GCS failsafe timeout
- **FS_GCS_ENABLE** (bool, default false): Enable GCS failsafe
- **BATT_LOW_VOLT** (float, default 0.0=disabled, range 0.0-30.0V): Low battery voltage
- **BATT_CRT_VOLT** (float, default 0.0=disabled, range 0.0-30.0V): Critical battery voltage
- **BATT_FS_LOW_ACT** (u8, default 0=None): Low battery action
- **BATT_FS_CRT_ACT** (u8, default 0=None): Critical battery action

## Risk Assessment

| Risk                                                        | Probability | Impact       | Mitigation Strategy                                                                                    |
| ----------------------------------------------------------- | ----------- | ------------ | ------------------------------------------------------------------------------------------------------ |
| **False failsafe trigger (unnecessary mode change)**        | **Medium**  | **Medium**   | **Conservative timeout values, extensive testing, operator can disable specific failsafes**            |
| **Missed failsafe (vehicle runaway)**                       | **Low**     | **CRITICAL** | **Fail-safe defaults (timeouts enabled), redundant checking, comprehensive test coverage**             |
| Failsafe timeout too short (false triggers)                 | Medium      | Medium       | Use proven ArduPilot defaults (1.5s RC, 5.0s GCS), allow configuration                                 |
| Failsafe timeout too long (late detection)                  | Low         | High         | Default values validated by ArduPilot community, document timeout selection rationale                  |
| Battery threshold misconfiguration (premature failsafe)     | High        | Low          | Default to disabled (0.0V), require explicit configuration, validate against battery datasheet         |
| Hold/RTL mode not implemented when failsafe triggers        | High        | High         | Implement Hold mode in same task, validate mode availability before triggering failsafe                |
| Multiple simultaneous failsafes (conflicting actions)       | Medium      | Medium       | Priority-based execution, highest priority wins, log all active failsafes                              |
| GCS failsafe triggers in RTL mode (already safe)            | Medium      | Low          | Disable GCS failsafe when in RTL or Hold modes (ArduPilot pattern)                                     |
| Failsafe recovery race condition (signal returns mid-action | Low         | Low          | Complete failsafe action before checking recovery, hysteresis on recovery (require signal stable > 1s) |
| Failsafe system memory overhead exceeds budget              | Low         | Medium       | Profile early, optimize state tracking, limit initial failsafe types to 3                              |

## Open Questions

- [ ] Should RC failsafe auto-clear and return to previous mode, or require operator mode switch? → Decision: Require operator mode switch (ArduPilot pattern, safer)
- [ ] Do we implement SmartRTL in Phase 1, or just Hold + RTL? → Method: Phase 1 = Hold only, Phase 2 = RTL, Phase 3 = SmartRTL (progressive expansion)
- [ ] Should battery failsafe use voltage only, or also capacity (mAh)? → Decision: Voltage only for Phase 1 (simpler), add capacity in Phase 2
- [ ] Do we need hysteresis on failsafe recovery (e.g., signal stable for 1s before clearing)? → Method: Yes, require 1 second of stable signal to avoid flapping
- [ ] Should GCS failsafe be enabled by default, or disabled? → Decision: Disabled by default (requires explicit opt-in, matches ArduPilot)
- [ ] How to handle failsafe during mode transitions (mode change in progress when failsafe triggers)? → Method: Allow failsafe to interrupt mode change, failsafe has higher priority
- [ ] Should we log failsafe parameters at startup for audit trail? → Decision: Yes, log all failsafe parameters at boot and when changed
- [ ] Do we implement CPU failsafe (watchdog) in Phase 1? → Decision: No, out of scope for Phase 1 (requires more investigation)

## Recommendations

### Immediate Actions

1. **Adopt Option B architecture**: Centralized failsafe manager with priority-based execution
2. **Implement Phase 1 only**: RC loss, GCS loss, battery failsafe + Hold mode
3. **Follow ArduPilot timeout defaults**: 1.5s RC, 5.0s GCS (proven in field)
4. **Require explicit battery configuration**: Default battery thresholds to 0.0V (disabled), force operator to configure per battery type

### Next Steps

1. [ ] Create formal requirements: FR-<id> (RC failsafe), FR-<id> (GCS failsafe), FR-<id> (battery failsafe), FR-<id> (failsafe actions), FR-<id> (failsafe recovery), FR-<id> (failsafe parameters), NFR-<id> (latency), NFR-<id> (memory), NFR-<id> (logging), NFR-<id> (false triggers)
2. [ ] Draft ADR for: Failsafe system architecture (manager design, priority system, state tracking)
3. [ ] Draft ADR for: Failsafe parameters (which parameters, default values, validation rules)
4. [ ] Draft ADR for: Hold mode implementation (stop behavior, heading hold strategy)
5. [ ] Create task for: Failsafe system implementation (Phase 1: RC/GCS/Battery + Hold mode)
6. [ ] Plan integration testing: Verify failsafe triggers correctly, actions execute, recovery works

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **SmartRTL mode**: No intelligent path recording (Phase 3 feature)
- **RTL mode**: No return to launch (Phase 2 feature, Hold only in Phase 1)
- **Crash detection**: No velocity/throttle monitoring (Phase 3 feature)
- **EKF failsafe**: No position estimate health monitoring (Phase 3 feature)
- **CPU failsafe**: No watchdog/main loop monitoring (requires deeper investigation)
- **Terminate action**: No motor cutoff (safety concern, requires hardware kill switch)
- **Capacity-based battery failsafe**: Voltage only in Phase 1 (mAh tracking in Phase 2)
- **Multiple battery support**: Single battery only (multi-battery in Phase 2)
- **Failsafe options bitmask**: No advanced options (FS_OPTIONS) in Phase 1
- **Mode-specific failsafe behavior**: All modes treated identically in Phase 1
- **Failsafe action chaining**: No continue-mission options (Phase 2 feature)

## Appendix

### References

- ArduPilot Rover Failsafe: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/failsafe.cpp>
- ArduPilot Battery Monitor: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_BattMonitor/AP_BattMonitor.cpp>
- ArduPilot Failsafe Documentation: <https://ardupilot.org/rover/docs/rover-failsafes.html>
- ArduPilot Parameters: <https://ardupilot.org/rover/docs/parameters.html>
- MAVLink STATUSTEXT: <https://mavlink.io/en/messages/common.html#STATUSTEXT>
- MAVLink HEARTBEAT: <https://mavlink.io/en/messages/common.html#HEARTBEAT>
- PX4 Failsafe System: <https://docs.px4.io/main/en/config/safety.html>

### Raw Data

**ArduPilot Failsafe Trigger Logic**:

```cpp
// From Rover/failsafe.cpp
void Rover::failsafe_trigger(uint8_t failsafe_type, const char *type_str,
                               bool radio_timeout)
{
    // Check if already triggered
    if (failsafe.triggered) {
        return;
    }

    // Check timeout
    if (millis() - failsafe.start_time > g.fs_timeout * 1000) {
        failsafe.triggered = true;

        // Log event
        LOGGER_WRITE_ERROR(LogErrorSubsystem::FAILSAFE_RADIO,
                            LogErrorCode::FAILSAFE_OCCURRED);

        // Send notification
        gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe: %s", type_str);

        // Execute action
        switch (g.fs_action) {
            case Failsafe_Action_None:
                break;
            case Failsafe_Action_RTL:
                set_mode(mode_rtl, ModeReason::FAILSAFE);
                break;
            case Failsafe_Action_Hold:
                set_mode(mode_hold, ModeReason::FAILSAFE);
                break;
            case Failsafe_Action_SmartRTL:
                if (!set_mode(mode_smart_rtl, ModeReason::FAILSAFE)) {
                    if (!set_mode(mode_rtl, ModeReason::FAILSAFE)) {
                        set_mode(mode_hold, ModeReason::FAILSAFE);
                    }
                }
                break;
        }
    }
}

// Battery failsafe (from libraries/AP_BattMonitor/AP_BattMonitor.cpp)
void AP_BattMonitor::check_failsafes()
{
    for (uint8_t i = 0; i < _num_instances; i++) {
        const Failsafe type = check_failsafe(i);
        if (type > state[i].failsafe) {
            state[i].failsafe = type;

            // Notify
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                           "Battery %d is %s %.2fV used %.0f mAh",
                           i + 1, type == Failsafe::Low ? "low" : "critical",
                           state[i].voltage, state[i].consumed_mah);

            // Execute handler
            if (_battery_failsafe_handler_fn) {
                _battery_failsafe_handler_fn(type, i);
            }
        }
    }
}
```

**Failsafe Parameter Examples**:

```
# Typical ArduPilot Rover Failsafe Configuration

# RC Failsafe
FS_TIMEOUT = 1.5      # RC timeout (seconds)
FS_THR_ENABLE = 1     # Enable RC failsafe
FS_ACTION = 1         # Hold on RC loss

# GCS Failsafe
FS_GCS_ENABLE = 1     # Enable GCS failsafe
FS_GCS_TIMEOUT = 5.0  # GCS timeout (seconds)

# Battery Failsafe (3S LiPo, 3300mAh)
BATT_LOW_VOLT = 10.5       # Low = 3.5V per cell
BATT_CRT_VOLT = 10.0       # Critical = 3.33V per cell
BATT_FS_LOW_ACT = 2        # RTL on low battery
BATT_FS_CRT_ACT = 1        # Hold on critical battery
BATT_CAPACITY = 3300       # Battery capacity (mAh)
BATT_LOW_MAH = 660         # Low = 20% remaining
BATT_CRT_MAH = 330         # Critical = 10% remaining
```

**Proposed Failsafe State Machine**:

```
                       ┌──────────────┐
                       │  NO_FAILSAFE │
                       └───────┬──────┘
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
     RC timeout           GCS timeout          Battery < LOW
          │                    │                    │
          ▼                    ▼                    ▼
   ┌──────────┐         ┌──────────┐        ┌──────────┐
   │ RC_LOSS  │         │ GCS_LOSS │        │ BATT_LOW │
   │ FAILSAFE │         │ FAILSAFE │        │ FAILSAFE │
   └─────┬────┘         └─────┬────┘        └─────┬────┘
         │                    │                    │
         │  Execute Action    │   Execute Action   │  Execute Action
         │  (Hold/RTL/etc)    │   (Hold/RTL/etc)   │  (RTL/Hold/etc)
         │                    │                    │
         ▼                    ▼                    ▼
   ┌──────────┐         ┌──────────┐        ┌──────────┐
   │  ACTIVE  │         │  ACTIVE  │        │  ACTIVE  │
   │ FAILSAFE │         │ FAILSAFE │        │ FAILSAFE │
   └─────┬────┘         └─────┬────┘        └─────┬────┘
         │                    │                    │
   RC recovered          GCS recovered       Battery not
         │                    │               auto-clear
         │                    │                    │
         ▼                    ▼                    │
   ┌──────────┐         ┌──────────┐              │
   │ RECOVERY │         │ RECOVERY │              │
   └─────┬────┘         └─────┬────┘              │
         │                    │                    │
         └────────────────────┴────────────────────┘
                              │
                              ▼
                       ┌──────────────┐
                       │  NO_FAILSAFE │
                       └──────────────┘

         Battery < CRITICAL (override all other failsafes)
                              │
                              ▼
                       ┌──────────────┐
                       │ BATT_CRITICAL│
                       │   FAILSAFE   │
                       │ (HIGHEST PRI)│
                       └──────────────┘
```

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
