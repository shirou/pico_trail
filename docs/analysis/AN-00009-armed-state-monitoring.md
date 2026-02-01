# AN-00009 Armed State Continuous Monitoring

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-00011-failsafe-system](AN-00011-failsafe-system.md)
  - [AN-00008-pre-arm-checks](AN-00008-pre-arm-checks.md)
  - [AN-00007-manual-control-implementation](AN-00007-manual-control-implementation.md)
- Related Requirements:
  - [FR-00024-cpu-performance-monitoring](../requirements/FR-00024-cpu-performance-monitoring.md)
  - [FR-00042-health-telemetry-reporting](../requirements/FR-00042-health-telemetry-reporting.md)
  - [FR-00021-battery-voltage-monitoring](../requirements/FR-00021-battery-voltage-monitoring.md)
  - [FR-00039-fallback-mode-selection](../requirements/FR-00039-fallback-mode-selection.md)
  - [FR-00052-rc-input-health-monitoring](../requirements/FR-00052-rc-input-health-monitoring.md)
  - [FR-00057-system-health-status-tracking](../requirements/FR-00057-system-health-status-tracking.md)
  - [FR-00054-sensor-health-monitoring](../requirements/FR-00054-sensor-health-monitoring.md)
  - [NFR-00042-monitoring-cpu-overhead](../requirements/NFR-00042-monitoring-cpu-overhead.md)
  - [NFR-00035-health-status-change-logging](../requirements/NFR-00035-health-status-change-logging.md)
  - [NFR-00043-monitoring-ram-overhead](../requirements/NFR-00043-monitoring-ram-overhead.md)
  - [NFR-00034-graceful-degradation-support](../requirements/NFR-00034-graceful-degradation-support.md)
  - [NFR-00036-high-frequency-monitor-detection-time](../requirements/NFR-00036-high-frequency-monitor-detection-time.md)
- Related ADRs:
  - [ADR-00012-arming-system-architecture](../adr/ADR-00012-arming-system-architecture.md)
- Related Tasks:
  - [T-00008-arming-system-implementation](../tasks/T-00008-arming-system-implementation/README.md)

## Executive Summary

This analysis explores the need for continuous system health monitoring during armed operation. Currently, pico_trail performs pre-arm checks but has no continuous monitoring after arming succeeds - once armed, the system does not actively monitor RC signal health, sensor status, battery levels, or other critical parameters that could degrade during flight. Continuous monitoring is essential for detecting failures that occur after arming and triggering appropriate failsafe responses before they lead to crashes or loss of control.

Key findings: ArduPilot implements a multi-frequency monitoring architecture with high-frequency tasks (50-400 Hz) for RC input and sensor processing, medium-frequency tasks (10 Hz) for failsafe checks and EKF validation, and low-frequency tasks (1 Hz) for housekeeping. This layered approach balances responsiveness with CPU efficiency. For pico_trail, a similar multi-rate monitoring system is recommended, focusing initially on critical safety monitors (RC input, battery voltage, sensor health) at appropriate frequencies to detect failures quickly while maintaining real-time control loop performance.

## Problem Space

### Current State

The project currently has:

- **Pre-arm checks**: Battery voltage and system state validation before arming (AN-00008)
- **One-time arming**: `SystemState::arm()` in `src/communication/mavlink/state.rs:157-171`
- **No continuous monitoring**: After `arm()` succeeds, no health checks occur
- **No monitoring task**: No dedicated task or loop checking system health
- **No health state tracking**: No data structures tracking RC signal age, sensor status, or battery trends
- **No degraded mode handling**: Cannot detect or respond to partial sensor failures

Critical safety gaps:

- **RC signal age unknown**: No tracking of when last RC_CHANNELS message arrived
- **Battery drain undetected**: No continuous monitoring of voltage decline during operation
- **Sensor failures invisible**: IMU, GPS, compass failures not detected after arming
- **EKF health unmonitored**: No validation of attitude/position estimation quality
- **No health reporting**: GCS receives no periodic health status updates
- **Fence violations undetected**: No geofence monitoring (if fence implemented)

### Desired State

Enable comprehensive continuous monitoring system protecting flight safety:

1. **High-Frequency Monitoring (50-400 Hz)**: RC input freshness, sensor data validity, emergency stop conditions
2. **Medium-Frequency Monitoring (10 Hz)**: Failsafe checks, battery voltage, EKF health, fence validation
3. **Low-Frequency Monitoring (1 Hz)**: Parameter synchronization, home position updates, status reporting
4. **Health State Tracking**: Data structures recording last message times, sensor status, health flags
5. **Degraded Mode Support**: Continue operation with partial sensor loss when safe to do so
6. **Performance Monitoring**: Track CPU load, task overruns, scheduler health
7. **Health Reporting**: Periodic SYS_STATUS and HEARTBEAT messages with health flags

Success criteria:

- **Timely detection**: Critical failures detected within 200ms of occurrence
- **Frequency-appropriate checking**: High-priority checks at high frequency, housekeeping at low frequency
- **CPU efficiency**: Monitoring overhead < 10% of CPU time (RP2040/RP2350)
- **Actionable health data**: GCS receives sufficient information to assess vehicle status
- **Graceful degradation**: Vehicle continues safe operation when non-critical sensors fail
- **Integration with failsafes**: Monitoring system triggers failsafe actions when thresholds exceeded

### Gap Analysis

**Missing components**:

1. **Monitoring Framework**: Architecture for registering and executing health checks at different frequencies
2. **Task Scheduler**: Multi-rate scheduler supporting high (50-400 Hz), medium (10 Hz), and low (1 Hz) tasks
3. **RC Input Monitor**: Track RC_CHANNELS message arrival time, detect timeout
4. **Battery Monitor**: Continuous voltage checking, trend analysis, low/critical threshold detection
5. **Sensor Health Checks**: IMU validity, GPS fix quality, compass status monitoring
6. **EKF Health Monitor**: Attitude estimation quality, position estimate validity
7. **Performance Monitor**: CPU load tracking, task execution time measurement
8. **Health State Structure**: Data structure tracking all monitored parameters and their status
9. **Health Reporting**: SYS_STATUS message generation with health flags

**Technical deltas**:

- Add `src/vehicle/monitoring/` module with monitoring framework
- Implement multi-rate scheduler supporting task registration at different frequencies
- Create health state tracker recording status of all monitored systems
- Add RC input age tracking to RC_CHANNELS handler
- Implement battery voltage monitoring task (10 Hz)
- Add sensor health checking (IMU, GPS, compass availability)
- Create performance monitoring (CPU load, task timing)
- Generate SYS_STATUS messages with health bits (battery, RC, sensors)
- Integrate monitoring with failsafe system (AN-00011)
- Define monitoring task priorities and timing budgets

## Stakeholder Analysis

| Stakeholder         | Interest/Need                                          | Impact | Priority |
| ------------------- | ------------------------------------------------------ | ------ | -------- |
| Operators           | Know vehicle health status in real-time                | High   | P0       |
| Safety Reviewers    | Ensure failures detected before becoming catastrophic  | High   | P0       |
| Failsafe System     | Depends on monitoring to trigger protective actions    | High   | P0       |
| GCS Software        | Receive health telemetry for display to operator       | High   | P0       |
| Test Engineers      | Debug issues via health status logs                    | High   | P1       |
| Autonomous Features | Require continuous sensor validation for safe autonomy | High   | P1       |

## Research & Discovery

### User Feedback

From operational requirements:

- Continuous monitoring is fundamental to safe flight operation
- Operators need real-time visibility into vehicle health status
- Battery monitoring critical for preventing over-discharge during flight
- RC signal monitoring essential for manual control safety
- Sensor health checks required before trusting autonomous navigation
- Performance monitoring helps identify CPU overload issues early

### Competitive Analysis

**ArduPilot Rover Monitoring System**:

ArduPilot implements a sophisticated multi-frequency monitoring architecture via its scheduler system.

#### Task Scheduling Architecture

Tasks are registered in a priority-ordered table with frequency and timing budget specifications. The scheduler uses a tick-based system where:

```cpp
// From AP_Scheduler.cpp
interval_ticks = (is_zero(task.rate_hz) ? 1 : _loop_rate_hz / task.rate_hz)
```

High-priority tasks (priority ≤ MAX_FAST_TASK_PRIORITIES) receive full loop period budgets and execute if time available. Lower-priority tasks check elapsed time since last execution.

#### High-Frequency Tasks (50-400 Hz)

Critical control and sensing:

```cpp
// From Rover.cpp - Scheduler table
{ SCHEDULER_FUNC(read_radio),                 50,    200,   3 },  // RC input
{ SCHEDULER_FUNC(ahrs_update),               400,    400,   4 },  // AHRS/IMU
{ SCHEDULER_FUNC(update_current_mode),       400,    200,   5 },  // Mode logic
{ SCHEDULER_FUNC(set_servos),                400,    200,   6 },  // Actuator output
{ SCHEDULER_FUNC(update_GPS),                 50,    300,  11 },  // GPS processing
{ SCHEDULER_FUNC(update_rangefinder),         50,    200,  12 },  // Rangefinders
```

**Key high-frequency monitors**:

- **RC input**: 50 Hz ensures fresh manual control commands
- **AHRS updates**: 400 Hz processes IMU data for attitude estimation
- **Sensor data**: 50-200 Hz keeps sensor information current
- **Emergency detection**: 400 Hz enables rapid emergency stop response

#### Medium-Frequency Tasks (10 Hz)

Safety monitoring and diagnostics:

```cpp
{ SCHEDULER_FUNC(update_compass),             10,    200,  19 },  // Compass
{ SCHEDULER_FUNC(update_logging1),            10,    200,  21 },  // Logging
{ SCHEDULER_FUNC(update_logging2),            10,    200,  22 },  // More logging
{ SCHEDULER_FUNC(gcs_failsafe_check),         10,    200,  25 },  // Failsafe
{ SCHEDULER_FUNC(fence_check),                10,    100,  28 },  // Geofence
{ SCHEDULER_FUNC(ekf_check),                  10,    100,  30 },  // EKF health
```

**Key medium-frequency monitors**:

- **Failsafe checks**: 10 Hz balances detection speed with CPU efficiency
- **EKF validation**: 10 Hz checks attitude/position estimate quality
- **Fence monitoring**: 10 Hz validates vehicle within geofence boundaries
- **Logging**: 10 Hz records flight data for analysis

#### Low-Frequency Tasks (1-3 Hz)

Housekeeping and status:

```cpp
{ SCHEDULER_FUNC(update_batt_compass),        10,    200,  18 },  // Battery/compass
{ SCHEDULER_FUNC(one_second_loop),             1,    400,  20 },  // 1Hz updates
{ SCHEDULER_FUNC(update_smartrtl),             3,    200,  44 },  // Path recording
```

**1-second loop tasks**:

- Home position updates (preserve launch location)
- Parameter synchronization (detect changes from GCS)
- Status reporting (send periodic health telemetry)
- Arming checks (revalidate arming conditions during flight)
- Terrain data updates

#### Scheduler Design Principles

**Load Management**:

The scheduler dynamically adjusts timing budgets based on task execution patterns:

```cpp
// From AP_Scheduler.cpp
if (tasks_not_achieving_their_desired_rate) {
    extra_loop_us += 10;  // Add budget, up to 5000us
} else {
    after_50_consecutive_successful_cycles {
        extra_loop_us -= 1;  // Gradually reduce
    }
}
```

**Priority-Based Execution**:

- High-priority tasks (control, sensing) execute before low-priority tasks
- If CPU time insufficient, low-priority tasks deferred until next opportunity
- Critical tasks always execute; housekeeping tasks may skip cycles under load

**Performance Monitoring**:

```cpp
load_average = 1.0 when (filtered_loop_rate < configured_rate * 0.95)
```

Operators can monitor CPU load via telemetry to detect system overload.

#### AHRS Health Monitoring

**Health Check Mechanism** (from AP_AHRS.cpp):

```cpp
bool AP_AHRS::healthy() const {
    // Check active EKF health
    if (active_EKF_type() != EKFType::NONE) {
        return ekf->healthy();
    }
    // Fallback to DCM
    return dcm_healthy();
}
```

**Unhealthy Indicators**:

- EKF fails to initialize within startup delay
- EKF reports unhealthy due to bad sensor data
- Internal filter processing errors
- Active EKF type mismatch (fixed-wing only)

**Check Frequency**:

- Pre-arm verification validates AHRS health before flight
- Update cycle (loop rate) continuously processes sensor data
- Active EKF evaluation monitors filter faults and health flags

**Fallback Mechanism**:

ArduPilot automatically switches from EKF to DCM (Direction Cosine Matrix) if EKF becomes unhealthy during armed flight, providing graceful degradation.

#### Key Insights for pico_trail

1. **Multi-rate is essential**: Different monitors need different frequencies (RC at 50 Hz, failsafes at 10 Hz, housekeeping at 1 Hz)
2. **Priority matters**: Critical monitors execute first, housekeeping deferred under load
3. **Timing budgets**: Each task has microsecond-level budget to prevent overruns
4. **Graceful degradation**: Continue operation with reduced functionality when non-critical sensors fail
5. **Performance awareness**: Monitor CPU load to detect system stress
6. **Dynamic adaptation**: Adjust timing budgets based on actual execution patterns

### Technical Investigation

**Current pico_trail Architecture**:

File: `src/communication/mavlink/state.rs:157-171` (arm implementation)

```rust
pub fn arm(&mut self) -> Result<(), &'static str> {
    if self.is_armed() {
        return Err("Already armed");
    }

    if self.battery.is_critical() {
        return Err("Battery voltage too low");
    }

    self.armed = ArmedState::Armed;
    Ok(())
}
```

**Observations**:

- Battery check at arm time only - no continuous checking
- No health state tracking after arming
- No monitoring task or infrastructure
- No sensor health validation after arming

**Proposed Monitoring Framework Architecture**:

```rust
/// Health status for a monitored subsystem
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum HealthStatus {
    Unknown,     // Not yet checked
    Healthy,     // Operating normally
    Warning,     // Degraded but operational
    Unhealthy,   // Failed, not usable
}

/// Health state for all monitored systems
pub struct SystemHealth {
    /// RC input health
    pub rc_input: HealthStatus,
    pub rc_last_update_ms: u32,

    /// Battery health
    pub battery: HealthStatus,
    pub battery_voltage: f32,
    pub battery_voltage_trend: f32,  // V/s

    /// Sensor health
    pub imu: HealthStatus,
    pub gps: HealthStatus,
    pub compass: HealthStatus,

    /// EKF health (if implemented)
    pub ekf: HealthStatus,

    /// System performance
    pub cpu_load: f32,  // 0.0-1.0
    pub cpu_overload_count: u32,

    /// Overall health (worst of all subsystems)
    pub overall: HealthStatus,
}

impl SystemHealth {
    /// Update overall health based on subsystem status
    pub fn update_overall(&mut self) {
        // Worst status among critical subsystems
        self.overall = HealthStatus::Healthy;

        for status in [self.rc_input, self.battery, self.imu].iter() {
            if *status == HealthStatus::Unhealthy {
                self.overall = HealthStatus::Unhealthy;
                return;
            }
            if *status == HealthStatus::Warning {
                self.overall = HealthStatus::Warning;
            }
        }
    }

    /// Check if system is safe to continue armed operation
    pub fn safe_to_arm(&self) -> bool {
        // Critical systems must be healthy
        self.rc_input == HealthStatus::Healthy
            && self.battery != HealthStatus::Unhealthy
            && self.imu == HealthStatus::Healthy
    }
}
```

**Multi-Rate Monitoring Architecture**:

```rust
/// Monitoring task with frequency specification
pub struct MonitoringTask {
    name: &'static str,
    frequency_hz: u32,
    check_fn: fn(&mut SystemHealth, &SystemState) -> Result<(), &'static str>,
}

/// High-frequency monitors (50-100 Hz)
const HIGH_FREQ_MONITORS: &[MonitoringTask] = &[
    MonitoringTask {
        name: "RC Input",
        frequency_hz: 50,
        check_fn: check_rc_input_health,
    },
    MonitoringTask {
        name: "IMU Data",
        frequency_hz: 50,
        check_fn: check_imu_health,
    },
];

/// Medium-frequency monitors (10 Hz)
const MEDIUM_FREQ_MONITORS: &[MonitoringTask] = &[
    MonitoringTask {
        name: "Battery Voltage",
        frequency_hz: 10,
        check_fn: check_battery_health,
    },
    MonitoringTask {
        name: "GPS Status",
        frequency_hz: 10,
        check_fn: check_gps_health,
    },
    MonitoringTask {
        name: "Compass Status",
        frequency_hz: 10,
        check_fn: check_compass_health,
    },
];

/// Low-frequency monitors (1 Hz)
const LOW_FREQ_MONITORS: &[MonitoringTask] = &[
    MonitoringTask {
        name: "CPU Performance",
        frequency_hz: 1,
        check_fn: check_cpu_performance,
    },
    MonitoringTask {
        name: "Status Reporting",
        frequency_hz: 1,
        check_fn: send_health_telemetry,
    },
];
```

**RC Input Health Check**:

```rust
/// Check RC input health (50 Hz)
fn check_rc_input_health(
    health: &mut SystemHealth,
    state: &SystemState,
) -> Result<(), &'static str> {
    let current_time_ms = get_time_ms();
    let rc_age_ms = current_time_ms - health.rc_last_update_ms;

    // RC message should arrive every 20ms (50 Hz)
    // Warning at 100ms, unhealthy at 500ms
    if rc_age_ms > 500 {
        health.rc_input = HealthStatus::Unhealthy;
        Err("RC signal lost")
    } else if rc_age_ms > 100 {
        health.rc_input = HealthStatus::Warning;
        Ok(())  // Warning but continue
    } else {
        health.rc_input = HealthStatus::Healthy;
        Ok(())
    }
}

/// Called when RC_CHANNELS message received
pub fn update_rc_timestamp(health: &mut SystemHealth) {
    health.rc_last_update_ms = get_time_ms();
}
```

**Battery Health Check**:

```rust
/// Check battery health (10 Hz)
fn check_battery_health(
    health: &mut SystemHealth,
    state: &SystemState,
) -> Result<(), &'static str> {
    let voltage = state.battery.voltage;

    // Track voltage trend (V/s)
    let dt = 0.1;  // 10 Hz = 100ms = 0.1s
    health.battery_voltage_trend = (voltage - health.battery_voltage) / dt;
    health.battery_voltage = voltage;

    // Check thresholds (example for 3S LiPo)
    if voltage < 10.0 {
        health.battery = HealthStatus::Unhealthy;
        Err("Battery critical")
    } else if voltage < 10.5 {
        health.battery = HealthStatus::Warning;
        Ok(())  // Warning but continue
    } else {
        health.battery = HealthStatus::Healthy;
        Ok(())
    }
}
```

**IMU Health Check**:

```rust
/// Check IMU health (50 Hz)
fn check_imu_health(
    health: &mut SystemHealth,
    state: &SystemState,
) -> Result<(), &'static str> {
    // Check if IMU data is valid
    // (Placeholder - actual implementation depends on IMU driver)
    if !state.sensors.imu_available {
        health.imu = HealthStatus::Unhealthy;
        Err("IMU unavailable")
    } else if state.sensors.imu_error_count > 10 {
        health.imu = HealthStatus::Warning;
        Ok(())  // Degraded but functional
    } else {
        health.imu = HealthStatus::Healthy;
        Ok(())
    }
}
```

**CPU Performance Monitor**:

```rust
/// Check CPU performance (1 Hz)
fn check_cpu_performance(
    health: &mut SystemHealth,
    state: &SystemState,
) -> Result<(), &'static str> {
    // Calculate CPU load based on task execution times
    // (Placeholder - actual implementation depends on scheduler)
    health.cpu_load = calculate_cpu_load();

    if health.cpu_load > 0.95 {
        health.cpu_overload_count += 1;
        warn!("CPU overload: {:.1}%", health.cpu_load * 100.0);
    } else {
        health.cpu_overload_count = 0;
    }

    Ok(())
}
```

**Health Telemetry Reporting**:

```rust
/// Send health status to GCS (1 Hz)
fn send_health_telemetry(
    health: &mut SystemHealth,
    state: &SystemState,
) -> Result<(), &'static str> {
    // Generate MAVLink SYS_STATUS message
    let sys_status = mavlink::common::SYS_STATUS_DATA {
        onboard_control_sensors_present: sensor_bits_present(),
        onboard_control_sensors_enabled: sensor_bits_enabled(),
        onboard_control_sensors_health: sensor_bits_health(health),
        load: (health.cpu_load * 1000.0) as u16,  // 0-1000
        voltage_battery: (health.battery_voltage * 1000.0) as u16,  // mV
        current_battery: -1,  // Unknown
        battery_remaining: calculate_battery_remaining(health.battery_voltage),
        drop_rate_comm: 0,  // TODO: Calculate packet loss
        errors_comm: 0,
        errors_count1: 0,
        errors_count2: 0,
        errors_count3: 0,
        errors_count4: 0,
    };

    // Send via MAVLink router
    send_mavlink_message(MavMessage::SYS_STATUS(sys_status))?;
    Ok(())
}

/// Encode health status as MAVLink sensor health bits
fn sensor_bits_health(health: &SystemHealth) -> u32 {
    let mut bits = 0u32;

    if health.rc_input == HealthStatus::Healthy {
        bits |= 1 << 0;  // MAV_SYS_STATUS_SENSOR_RC_RECEIVER
    }
    if health.imu == HealthStatus::Healthy {
        bits |= 1 << 3;  // MAV_SYS_STATUS_SENSOR_3D_GYRO
        bits |= 1 << 4;  // MAV_SYS_STATUS_SENSOR_3D_ACCEL
    }
    if health.compass == HealthStatus::Healthy {
        bits |= 1 << 5;  // MAV_SYS_STATUS_SENSOR_3D_MAG
    }
    if health.gps == HealthStatus::Healthy {
        bits |= 1 << 6;  // MAV_SYS_STATUS_SENSOR_GPS
    }
    if health.battery != HealthStatus::Unhealthy {
        bits |= 1 << 11;  // MAV_SYS_STATUS_SENSOR_BATTERY
    }

    bits
}
```

**Integration with Vehicle Control Loop**:

```rust
/// Vehicle monitoring task (runs continuously during armed state)
pub fn vehicle_monitoring_task(
    health: &mut SystemHealth,
    state: &SystemState,
    failsafe: &mut FailsafeExecutor,
) -> Result<(), &'static str> {
    let mut last_high_freq_ms = 0;
    let mut last_medium_freq_ms = 0;
    let mut last_low_freq_ms = 0;

    loop {
        let current_time_ms = get_time_ms();

        // High-frequency checks (50 Hz = 20ms interval)
        if current_time_ms - last_high_freq_ms >= 20 {
            for task in HIGH_FREQ_MONITORS {
                if let Err(e) = (task.check_fn)(health, state) {
                    warn!("High-freq monitor '{}' failed: {}", task.name, e);
                    // Continue checking other monitors
                }
            }
            last_high_freq_ms = current_time_ms;
        }

        // Medium-frequency checks (10 Hz = 100ms interval)
        if current_time_ms - last_medium_freq_ms >= 100 {
            for task in MEDIUM_FREQ_MONITORS {
                if let Err(e) = (task.check_fn)(health, state) {
                    warn!("Medium-freq monitor '{}' failed: {}", task.name, e);
                }
            }

            // Update overall health
            health.update_overall();

            // Trigger failsafes if needed (failsafe system checks health state)
            failsafe.check_health(health)?;

            last_medium_freq_ms = current_time_ms;
        }

        // Low-frequency checks (1 Hz = 1000ms interval)
        if current_time_ms - last_low_freq_ms >= 1000 {
            for task in LOW_FREQ_MONITORS {
                if let Err(e) = (task.check_fn)(health, state) {
                    warn!("Low-freq monitor '{}' failed: {}", task.name, e);
                }
            }
            last_low_freq_ms = current_time_ms;
        }

        // Yield to other tasks
        delay_ms(5);  // Check at ~200 Hz base rate
    }
}
```

**Memory Analysis**:

| Component                | RAM Usage   | Notes                          |
| ------------------------ | ----------- | ------------------------------ |
| SystemHealth             | \~60 B      | Health status + timestamps     |
| MonitoringTask table     | \~100 B     | Function pointers + metadata   |
| Task timing state        | \~20 B      | Last execution timestamps      |
| **Total (monitoring)**   | **\~180 B** | Minimal overhead for safety    |
| \*\*Total (with failsafe | **\~350 B** | Combined monitoring + failsafe |

### Data Analysis

**Monitoring Frequency Analysis**:

| Subsystem        | Check Frequency | Rationale                                                    |
| ---------------- | --------------- | ------------------------------------------------------------ |
| RC Input         | 50 Hz           | Match RC_CHANNELS message rate (50 Hz typical)               |
| IMU Data         | 50-400 Hz       | High-rate for AHRS updates (can subsample for health checks) |
| Battery          | 10 Hz           | Voltage changes slowly, 10 Hz sufficient                     |
| GPS              | 10 Hz           | GPS update rate typically 5-10 Hz                            |
| Compass          | 10 Hz           | Compass data updates slowly                                  |
| Failsafes        | 10 Hz           | Balance detection speed with CPU efficiency                  |
| EKF Health       | 10 Hz           | Position estimate quality changes gradually                  |
| CPU Performance  | 1 Hz            | Load averaging over 1 second provides meaningful data        |
| Status Reporting | 1 Hz            | GCS telemetry rate, more frequent would saturate link        |

**Detection Latency Requirements**:

- **Critical failures** (RC loss, IMU failure): Detect within 100-200ms
- **Important failures** (battery low, GPS loss): Detect within 1 second
- **Non-critical issues** (CPU overload): Detect within 5 seconds

**CPU Budget Analysis** (RP2040/RP2350 at 133 MHz):

| Task Type          | Frequency | Time per Check | Total CPU Time | % of CPU |
| ------------------ | --------- | -------------- | -------------- | -------- |
| High-freq checks   | 50 Hz     | 50 µs          | 2.5 ms/s       | 0.25%    |
| Medium-freq checks | 10 Hz     | 100 µs         | 1.0 ms/s       | 0.10%    |
| Low-freq checks    | 1 Hz      | 500 µs         | 0.5 ms/s       | 0.05%    |
| **Total**          |           |                | **4.0 ms/s**   | **0.4%** |

Monitoring overhead is negligible compared to control loop and communication tasks.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The system shall continuously monitor RC input health at 50 Hz during armed operation → Will become FR-<id>
  - Rationale: Detect RC signal loss quickly to trigger failsafe before vehicle travels too far
  - Acceptance Criteria:
    - Track timestamp of last RC_CHANNELS message received
    - Check RC signal age every 20ms (50 Hz)
    - Mark RC input as Warning if age > 100ms
    - Mark RC input as Unhealthy if age > 500ms
    - Integrate with failsafe system (AN-00011) to trigger RC loss failsafe

- [ ] **FR-DRAFT-2**: The system shall continuously monitor battery voltage at 10 Hz during armed operation → Will become FR-<id>
  - Rationale: Detect battery depletion before reaching unsafe voltage levels
  - Acceptance Criteria:
    - Read battery voltage every 100ms (10 Hz)
    - Track voltage trend (V/s) for predictive monitoring
    - Mark battery as Warning if voltage < LOW threshold (configurable, e.g., 10.5V)
    - Mark battery as Unhealthy if voltage < CRITICAL threshold (e.g., 10.0V)
    - Integrate with failsafe system to trigger battery failsafe

- [ ] **FR-DRAFT-3**: The system shall continuously monitor sensor health (IMU, GPS, compass) at 10-50 Hz → Will become FR-<id>
  - Rationale: Detect sensor failures that occur after arming
  - Acceptance Criteria:
    - Check IMU data validity at 50 Hz (match IMU sample rate)
    - Check GPS fix status at 10 Hz
    - Check compass health at 10 Hz
    - Mark sensors as Unhealthy if data invalid or unavailable
    - Support graceful degradation (continue operation if non-critical sensor fails)

- [ ] **FR-DRAFT-4**: The system shall track overall system health status combining all monitored subsystems → Will become FR-<id>
  - Rationale: Provide single health indicator for vehicle state assessment
  - Acceptance Criteria:
    - Maintain health status (Unknown/Healthy/Warning/Unhealthy) for each subsystem
    - Calculate overall health as worst status among critical subsystems
    - Critical subsystems: RC input, battery, IMU
    - Non-critical subsystems: GPS, compass (vehicle can operate without them in Manual mode)

- [ ] **FR-DRAFT-5**: The system shall send periodic health telemetry to GCS at 1 Hz → Will become FR-<id>
  - Rationale: Operators need real-time visibility into vehicle health status
  - Acceptance Criteria:
    - Generate MAVLink SYS_STATUS message every 1 second
    - Include sensor health bits (RC, IMU, GPS, compass, battery)
    - Include CPU load percentage
    - Include battery voltage and remaining capacity estimate
    - Include packet loss statistics

- [ ] **FR-DRAFT-6**: The system shall monitor CPU performance and detect overload conditions → Will become FR-<id>
  - Rationale: CPU overload can cause control loop delays and missed deadlines
  - Acceptance Criteria:
    - Calculate CPU load as percentage (0-100%)
    - Track task execution times and scheduler overruns
    - Warn operator if CPU load > 95% for more than 5 seconds
    - Log CPU overload events for debugging

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Monitoring system shall add no more than 5% CPU overhead on RP2040/RP2350 → Will become NFR-<id>
  - Category: Performance
  - Rationale: Monitoring must not interfere with real-time control loop
  - Target: < 5% CPU time (measured via execution time profiling)

- [ ] **NFR-DRAFT-2**: High-frequency monitors shall detect failures within 200ms of occurrence → Will become NFR-<id>
  - Category: Performance / Safety
  - Rationale: Rapid detection enables timely failsafe response
  - Target: RC loss detected within 200ms (50 Hz check rate supports this)

- [ ] **NFR-DRAFT-3**: Monitoring system shall add no more than 200 bytes RAM overhead → Will become NFR-<id>
  - Category: Resource Constraints
  - Rationale: Maintain memory budget for other subsystems
  - Target: < 200 B for health state + task tracking

- [ ] **NFR-DRAFT-4**: Health status changes shall be logged to persistent storage → Will become NFR-<id>
  - Category: Reliability / Auditability
  - Rationale: Support post-flight analysis and debugging
  - Target: Log all health transitions (Healthy → Warning → Unhealthy) with timestamps

- [ ] **NFR-DRAFT-5**: Monitoring system shall support graceful degradation with partial sensor loss → Will become NFR-<id>
  - Category: Reliability
  - Rationale: Vehicle should continue safe operation when non-critical sensors fail
  - Target: Continue Manual mode operation with IMU + RC only (GPS/compass optional)

## Design Considerations

### Technical Constraints

- **Real-time requirements**: Monitoring must not interfere with control loop timing (50 Hz minimum)
- **Memory budget**: Limited RAM on RP2040/RP2350 (264 KB)
- **CPU budget**: RP2040 at 133 MHz, must maintain control loop + monitoring + communication
- **No dynamic allocation**: Monitoring framework must use static/stack allocation only
- **Existing architecture**: Must integrate with current MAVLink router and vehicle state
- **Failsafe integration**: Monitoring system feeds data to failsafe system (AN-00011)
- **Sensor abstraction**: Monitoring should work with different sensor configurations

### Potential Approaches

1. **Option A: Simple Polling in Main Loop**
   - Pros:
     - Simplest implementation (single loop checking all monitors)
     - No scheduler needed
     - Easy to understand and debug
     - Minimal memory overhead (\~100 B)
   - Cons:
     - All monitors run at same frequency (wasteful)
     - Hard to add new monitors with different frequencies
     - No priority system (critical and non-critical checks treated equally)
     - High CPU usage if all checks run every iteration
   - Effort: Low (8-16 hours)

2. **Option B: Multi-Rate Monitoring with Simple Scheduler** ⭐ Recommended
   - Pros:
     - Different monitors run at appropriate frequencies
     - Lower CPU usage (run only what's needed when needed)
     - Extensible: New monitors added by registration
     - Priority support (high-freq monitors execute first)
     - Matches ArduPilot architecture (proven, familiar)
     - Moderate memory overhead (\~180 B)
   - Cons:
     - More complex than Option A
     - Requires timing management
     - Slightly more memory than simple polling
   - Effort: Medium (24-32 hours)

3. **Option C: Full ArduPilot-Style Scheduler with Tick System**
   - Pros:
     - Maximum flexibility and performance
     - Sophisticated load management (dynamic budget adjustment)
     - Advanced features: task priorities, timing budgets, overrun detection
     - Best CPU efficiency
   - Cons:
     - High complexity
     - Longer development time
     - Higher memory overhead (\~500 B)
     - Overkill for initial implementation
   - Effort: High (60-80 hours)

**Recommendation**: Option B (Multi-Rate Monitoring with Simple Scheduler) provides best balance of functionality, CPU efficiency, and development effort. Option C features (dynamic load management, timing budgets) can be added incrementally if needed.

### Architecture Impact

**New ADRs required**:

- **ADR-<id> Monitoring System Architecture**: Multi-rate design, task registration, health state management
- **ADR-<id> Health Status Definition**: What constitutes healthy/warning/unhealthy for each subsystem
- **ADR-<id> Monitoring Frequencies**: Rationale for chosen check rates (50 Hz / 10 Hz / 1 Hz)
- **ADR-<id> Graceful Degradation Policy**: Which sensors are critical vs. optional per flight mode

**New modules**:

- `src/vehicle/monitoring/` - Monitoring system
  - `src/vehicle/monitoring/health.rs` - SystemHealth structure and status enums
  - `src/vehicle/monitoring/scheduler.rs` - Multi-rate task scheduler
  - `src/vehicle/monitoring/tasks.rs` - Monitoring task implementations
  - `src/vehicle/monitoring/checkers/` - Individual health check implementations
    - `src/vehicle/monitoring/checkers/rc_input.rs`
    - `src/vehicle/monitoring/checkers/battery.rs`
    - `src/vehicle/monitoring/checkers/sensors.rs`
    - `src/vehicle/monitoring/checkers/cpu.rs`
- `src/vehicle/control_task.rs` - Vehicle control task integrating monitoring

**Modified modules**:

- `src/communication/mavlink/handlers/telemetry.rs` - Update RC timestamp on RC_CHANNELS
- `src/communication/mavlink/router.rs` - Generate SYS_STATUS messages with health bits
- `src/communication/mavlink/state.rs` - Add health state to SystemState
- `src/vehicle/failsafe/` - Use health state to trigger failsafes (AN-00011 integration)

## Parameters

This analysis does not reference specific ArduPilot parameters. The monitoring system is an architectural component that uses existing failsafe parameters (see AN-00011-failsafe-system.md) for threshold configuration.

The monitoring frequencies (50 Hz, 10 Hz, 1 Hz) are derived from ArduPilot's scheduler task table but are implemented as architectural constants rather than configurable parameters.

## Risk Assessment

| Risk                                                         | Probability | Impact       | Mitigation Strategy                                                                           |
| ------------------------------------------------------------ | ----------- | ------------ | --------------------------------------------------------------------------------------------- |
| **Monitoring overhead degrades control loop performance**    | **Medium**  | **CRITICAL** | **Profile early, optimize hot paths, limit initial monitors to essential only**               |
| **False health warnings (unnecessary operator concern)**     | **Medium**  | **Medium**   | **Conservative thresholds, hysteresis on health transitions, extensive testing**              |
| Missed health failures (monitor doesn't detect real problem) | Low         | Critical     | Comprehensive test suite, validate detection thresholds, compare with ArduPilot behavior      |
| Health checks too frequent (CPU overload)                    | Low         | High         | Use ArduPilot-proven frequencies (50/10/1 Hz), measure CPU time, reduce frequency if needed   |
| Health checks too infrequent (late detection)                | Low         | High         | Use ArduPilot-proven frequencies, validate detection latency requirements                     |
| Memory budget exceeded                                       | Low         | Medium       | Profile memory usage early, optimize health state structure, limit task metadata              |
| Monitoring task crashes vehicle                              | Low         | Critical     | Defensive error handling, continue operation even if monitor fails, log errors                |
| Integration conflicts with failsafe system                   | Low         | Medium       | Design interfaces carefully, clear ownership (monitoring produces data, failsafe consumes it) |
| Monitoring task starves other tasks (priority inversion)     | Low         | Medium       | Run monitoring at appropriate priority (below control, above housekeeping), yield regularly   |
| Health telemetry saturates MAVLink bandwidth                 | Low         | Low          | 1 Hz SYS_STATUS rate is standard and tested, packet size is small (\~50 bytes)                |

## Open Questions

- [ ] Should monitoring run as separate task or integrated into vehicle control task? → Method: Start with integrated approach (simpler), split to separate task if performance issues arise
- [ ] What is acceptable CPU overhead percentage for monitoring? → Decision: < 5% based on RP2040 capabilities and ArduPilot overhead measurements
- [ ] Which sensors are critical vs. optional in Manual mode? → Decision: Critical = RC + IMU + battery, Optional = GPS + compass (create ADR to document this)
- [ ] Should GPS health consider fix quality (2D vs 3D) or just availability? → Method: Phase 1 = availability only, Phase 2 = add fix quality assessment
- [ ] Do we implement EKF health monitoring in Phase 1? → Decision: No, out of scope for Phase 1 (requires EKF implementation first)
- [ ] Should health warnings trigger audio/visual alerts on vehicle? → Method: Phase 1 = telemetry only, Phase 2 = add buzzer/LED support
- [ ] How to handle health status during mode transitions? → Method: Clear health warnings that are mode-specific, retain sensor health status
- [ ] Should we log all health checks or only health transitions? → Decision: Log transitions only to reduce storage usage

## Recommendations

### Immediate Actions

1. **Adopt Option B architecture**: Multi-rate monitoring with simple scheduler (50 Hz / 10 Hz / 1 Hz)
2. **Implement Phase 1 monitors only**: RC input, battery, IMU, CPU performance
3. **Use ArduPilot-proven frequencies**: 50 Hz (RC/IMU), 10 Hz (battery), 1 Hz (telemetry)
4. **Start with conservative thresholds**: Prefer false negatives over false positives initially
5. **Integrate with failsafe system**: Monitoring produces health data, failsafe consumes it

### Next Steps

1. [ ] Create formal requirements: FR-<id> (RC monitoring), FR-<id> (battery monitoring), FR-<id> (sensor monitoring), FR-<id> (health tracking), FR-<id> (health telemetry), FR-<id> (CPU monitoring), NFR-<id> (CPU overhead), NFR-<id> (detection latency), NFR-<id> (memory overhead), NFR-<id> (logging), NFR-<id> (graceful degradation)
2. [ ] Draft ADR for: Monitoring system architecture (multi-rate design, task registration, health state)
3. [ ] Draft ADR for: Health status definitions (what constitutes healthy/warning/unhealthy per subsystem)
4. [ ] Draft ADR for: Monitoring frequencies (rationale for 50/10/1 Hz choices)
5. [ ] Draft ADR for: Graceful degradation policy (critical vs. optional sensors per mode)
6. [ ] Create task for: Monitoring system implementation (Phase 1: RC/battery/IMU/CPU)
7. [ ] Plan integration testing: Verify monitoring detects failures correctly, CPU overhead acceptable

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **EKF health monitoring**: No position estimate quality checking (requires EKF implementation first)
- **Geofence monitoring**: No boundary checking (requires geofence implementation first)
- **Advanced scheduler features**: No dynamic load management or timing budgets (can add later if needed)
- **Compass monitoring**: Optional sensor, defer to Phase 2
- **GPS quality assessment**: Phase 1 checks availability only, defer fix quality (2D/3D/RTK) to Phase 2
- **Terrain monitoring**: No terrain database or altitude checking
- **Airspeed monitoring**: Not applicable to ground vehicle
- **Motor/ESC monitoring**: No individual motor health checking
- **Advanced diagnostics**: No watchdog monitoring, no stack overflow detection (Phase 2)
- **Health prediction**: No predictive failure detection based on trends
- **Multiple IMU support**: Single IMU only in Phase 1

## Appendix

### References

- ArduPilot Rover Scheduler: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/Rover.cpp>
- ArduPilot Scheduler Library: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scheduler/AP_Scheduler.cpp>
- ArduPilot AHRS Health: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_AHRS/AP_AHRS.cpp>
- MAVLink SYS_STATUS: <https://mavlink.io/en/messages/common.html#SYS_STATUS>
- MAVLink HEARTBEAT: <https://mavlink.io/en/messages/common.html#HEARTBEAT>
- ArduPilot Pre-Arm Checks: <https://ardupilot.org/rover/docs/common-prearm-safety-checks.html>

### Raw Data

**ArduPilot Scheduler Task Table** (from Rover.cpp):

```cpp
const AP_Scheduler::Task Rover::scheduler_tasks[] = {
    // High-frequency tasks (50-400 Hz)
    { SCHEDULER_FUNC(read_radio),                 50,    200,   3 },
    { SCHEDULER_FUNC(ahrs_update),               400,    400,   4 },
    { SCHEDULER_FUNC(update_current_mode),       400,    200,   5 },
    { SCHEDULER_FUNC(set_servos),                400,    200,   6 },
    { SCHEDULER_FUNC(update_GPS),                 50,    300,  11 },
    { SCHEDULER_FUNC(update_rangefinder),         50,    200,  12 },

    // Medium-frequency tasks (10 Hz)
    { SCHEDULER_FUNC(update_compass),             10,    200,  19 },
    { SCHEDULER_FUNC(update_logging1),            10,    200,  21 },
    { SCHEDULER_FUNC(update_logging2),            10,    200,  22 },
    { SCHEDULER_FUNC(gcs_failsafe_check),         10,    200,  25 },
    { SCHEDULER_FUNC(fence_check),                10,    100,  28 },
    { SCHEDULER_FUNC(ekf_check),                  10,    100,  30 },

    // Low-frequency tasks (1-3 Hz)
    { SCHEDULER_FUNC(one_second_loop),             1,    400,  20 },
    { SCHEDULER_FUNC(update_smartrtl),             3,    200,  44 },
};
```

**ArduPilot AHRS Health Check** (from AP_AHRS.cpp):

```cpp
bool AP_AHRS::healthy() const
{
    // Check active EKF health
    if (active_EKF_type() != EKFType::NONE) {
        return ekf->healthy();
    }

    // If EKF is started we switch away if it reports unhealthy.
    // This could be due to bad sensor data.
    if (ekf_type != EKFType::NONE) {
        if (!ekf->healthy()) {
            // Switch to DCM fallback
            return false;
        }
    }

    // DCM is always healthy if initialized
    return dcm_healthy();
}
```

**Proposed Monitoring Task Timing**:

```
High-frequency (50 Hz = 20ms period):
│  0ms  │ 20ms  │ 40ms  │ 60ms  │ 80ms  │ 100ms │
├───────┼───────┼───────┼───────┼───────┼───────┤
│ RC/IMU│ RC/IMU│ RC/IMU│ RC/IMU│ RC/IMU│ RC/IMU│
├───────┴───────┴───────┴───────┴───────┴───────┤

Medium-frequency (10 Hz = 100ms period):
│   0ms   │  100ms  │  200ms  │  300ms  │  400ms  │
├─────────┼─────────┼─────────┼─────────┼─────────┤
│ Batt/GPS│ Batt/GPS│ Batt/GPS│ Batt/GPS│ Batt/GPS│
├─────────┴─────────┴─────────┴─────────┴─────────┤

Low-frequency (1 Hz = 1000ms period):
│     0ms      │    1000ms     │    2000ms     │
├──────────────┼───────────────┼───────────────┤
│ CPU/Telemetry│ CPU/Telemetry │ CPU/Telemetry │
└──────────────┴───────────────┴───────────────┘
```
