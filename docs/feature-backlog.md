# Feature Backlog

This document tracks planned features outside the formal TDL process. Features here are candidates for future Analysis documents when prioritized for implementation.

## Status Legend

- **Proposed**: Initial idea, needs refinement
- **Refined**: Detailed enough for Analysis
- **In Progress**: Analysis/Requirements started
- **Done**: Implemented

---

## Safety Features

### FB-005: Communication Lost Action

**Status**: Proposed

**Description**: Configurable behavior when GCS communication is lost (no heartbeat received). Parameters (`FS_GCS_ENABLE`, `FS_ACTION`, `FS_TIMEOUT`) and `FailsafeAction` enum exist in core, but the active GCS heartbeat monitoring loop and mode transition trigger are not wired.

**Use Cases**:

- Graceful handling of WiFi dropout
- Predictable behavior for safety
- Mission continuity vs. safety trade-off

**ArduPilot Reference**: [GCS Failsafe](https://ardupilot.org/rover/docs/rover-failsafes.html#ground-station-failsafe)

**Key Parameters** (ArduPilot standard):

| Parameter       | Description                                 | Default |
| --------------- | ------------------------------------------- | ------- |
| `FS_GCS_ENABLE` | GCS failsafe enable (0=off, 1=on)           | 0       |
| `FS_ACTION`     | Failsafe action (0=hold, 1=RTL, 2=SmartRTL) | 1       |
| `FS_TIMEOUT`    | Failsafe timeout in seconds                 | 5       |

**Behavior Options**:

| Action | Description                        |
| ------ | ---------------------------------- |
| 0      | Hold position (stop and wait)      |
| 1      | RTL (direct return to launch)      |
| 2      | SmartRTL (retrace path home)       |
| 3      | Continue mission (if in Auto mode) |

**Implementation Considerations**:

- Track last GCS heartbeat timestamp
- Timeout check in main loop or dedicated task
- Mode transition on timeout
- Recovery: resume previous mode when communication restored?
- Consider RC failsafe interaction (separate or combined?)

**Dependencies**:

- Heartbeat tracking (already in MAVLink handler)
- RTL/SmartRTL/Hold modes (implemented)
- Mode switching infrastructure (implemented)

---

### FB-017: Geofence

**Status**: Proposed

**Description**: Define geographic boundaries that the vehicle must stay within (inclusion fence) or avoid (exclusion fence). Trigger configurable action on breach.

**Use Cases**:

- Prevent rover from leaving designated area
- Keep boat within safe water boundaries
- Regulatory compliance for autonomous operation

**ArduPilot Reference**: [Rover Fence](https://ardupilot.org/rover/docs/rover-fence.html)

**Key Parameters** (ArduPilot standard):

| Parameter      | Description                                         | Default |
| -------------- | --------------------------------------------------- | ------- |
| `FENCE_ENABLE` | Enable/disable fence                                | 0       |
| `FENCE_TYPE`   | Bitmask: 1=max altitude, 2=circle, 4=polygon        | 6       |
| `FENCE_ACTION` | Breach action (0=report, 1=RTL, 2=hold, 3=SmartRTL) | 1       |
| `FENCE_RADIUS` | Circular fence radius (m)                           | 100     |
| `FENCE_MARGIN` | Distance from fence to trigger warning (m)          | 2       |

**Behavior**:

1. **Circular fence**: Simple radius from home position
2. **Polygon fence**: Uploaded via mission protocol as FENCE_POINT items
3. On breach: execute configured action (RTL, Hold, SmartRTL, or report-only)
4. Warning zone inside fence boundary for early notification

**Implementation Considerations**:

- Circular fence is simplest (distance from home < radius)
- Polygon fence requires point-in-polygon algorithm (ray casting)
- Check position against fence at navigation update rate
- Memory: polygon points stored similarly to waypoints
- RP2350 can handle \~50 polygon points easily

**Dependencies**:

- GPS position
- Home position (FB-014)
- RTL / Hold modes

---

### FB-018: LED Status Patterns

**Status**: Proposed

**Description**: Define LED blink patterns to indicate vehicle state without GCS connection.

**Use Cases**:

- Field debugging without laptop
- Visual confirmation of GPS fix, armed state, errors
- Safety indicator visible from distance

**Patterns**:

| State                | Pattern           |
| -------------------- | ----------------- |
| Disarmed, no GPS     | Slow blink (1 Hz) |
| Disarmed, GPS fix    | Double blink      |
| Armed                | Solid on          |
| Failsafe active      | Fast blink (4 Hz) |
| Error / pre-arm fail | Triple blink      |
| Calibrating          | Breathing / fade  |

**Implementation Considerations**:

- PIN_LED already defined in board configuration
- State machine driven by vehicle state (armed, GPS fix, failsafe, etc.)
- Run as low-priority scheduler task (\~10 Hz)
- PWM for brightness control / breathing effect (optional)

**Dependencies**:

- GPIO output (existing)
- Vehicle state (existing SYSTEM_STATE)

---

### FB-019: Buzzer Alert Patterns

**Status**: Proposed

**Description**: Define buzzer tone patterns for audible state indication and alerts.

**Use Cases**:

- Arming/disarming confirmation
- Low battery audible warning
- GPS fix acquired notification
- Pre-arm check failure alert

**Patterns**:

| Event               | Pattern                  |
| ------------------- | ------------------------ |
| Arm success         | Rising tone              |
| Disarm              | Falling tone             |
| GPS fix acquired    | Two short beeps          |
| Low battery warning | Periodic beep (every 5s) |
| Critical battery    | Continuous rapid beep    |
| Pre-arm failure     | Three descending tones   |
| Failsafe triggered  | Siren pattern            |

**Implementation Considerations**:

- PIN_BUZZER already defined in board configuration
- PWM frequency control for different tones
- Non-blocking: queue patterns, play asynchronously
- Respect user preference (enable/disable via parameter)

**Dependencies**:

- PWM output (existing)
- Vehicle state events

---

### FB-020: Watchdog Timer

**Status**: Proposed

**Description**: Hardware watchdog to detect and recover from software hangs on the embedded target.

**Use Cases**:

- Recover from infinite loops or deadlocks
- Safety reset if main loop stalls
- Critical for unattended autonomous operation

**Behavior**:

1. Initialize hardware watchdog with timeout (e.g., 2 seconds)
2. Main loop feeds (kicks) the watchdog each iteration
3. If main loop stalls, watchdog triggers hardware reset
4. After reset, detect watchdog-caused boot and log it
5. Optionally: enter safe mode after watchdog reset

**Implementation Considerations**:

- RP2350 has built-in watchdog peripheral
- `embassy-rp` provides watchdog API
- Timeout must be longer than worst-case main loop iteration
- Log watchdog resets to flash for post-mortem analysis
- Consider: should watchdog reset enter a safe mode or resume normal operation?

**Dependencies**:

- RP2350 watchdog peripheral
- Boot detection (watchdog vs. normal boot)

---

## Control Modes

### FB-016: Hold Mode

**Status**: Done

**Description**: Explicit "stop and hold" mode. Vehicle stops all motor output and remains stationary. `FlightMode::Hold` variant exists in the enum but no `Mode` trait implementation struct exists yet.

**Use Cases**:

- Operator-initiated stop without switching to Manual
- Default safe state for unknown situations
- Failsafe fallback when GPS is unavailable (can't do RTL or Loiter)

**ArduPilot Reference**: [Hold Mode](https://ardupilot.org/rover/docs/hold-mode.html)

**Behavior**:

1. Zero throttle and steering outputs
2. No GPS required (works without position fix)
3. No active position correction (contrast with Loiter Type 1)
4. Remain in Hold until operator changes mode

**Implementation Considerations**:

- Simplest possible mode: just zero all outputs
- Good candidate for failsafe fallback when GPS unavailable
- ArduPilot custom mode number: 4

**Dependencies**:

- Mode framework (existing)

---

## Mission Features

### FB-006: Mission Pause/Resume

**Status**: Proposed

**Description**: Pause current mission execution and resume from the same point later.

**Use Cases**:

- Temporary stop for obstacle or situation assessment
- Operator intervention during mission
- Battery swap mid-mission

**ArduPilot Reference**: Part of Mission Protocol

**Behavior**:

**Pause**:

1. Triggered by: RC switch, GCS command, or condition
2. Record current mission state:
   - Current waypoint index
   - Progress toward waypoint (optional: exact position)
3. Enter Hold/Loiter mode
4. Maintain mission data in memory

**Resume**:

1. Triggered by: RC switch or GCS command
2. Options:
   - Resume from pause position → current waypoint
   - Resume from current waypoint (restart segment)
3. Re-enter Auto mode
4. Continue mission execution

**MAVLink Commands**:

- `MAV_CMD_DO_PAUSE_CONTINUE` (193): Pause/resume mission
  - param1: 0=pause, 1=resume

**Implementation Considerations**:

- Mission state persistence: RAM only (lost on reboot) vs. Flash (survives reboot)
- Resume position: exact pause point vs. restart current segment
- Interaction with failsafe (pause on RC loss?)
- Multiple pause/resume cycles

**Dependencies**:

- Auto mode (mission execution)
- Mission state tracking
- Hold/Loiter mode

---

## MAVLink Protocol Completeness

### FB-013: Log Download Protocol

**Status**: Proposed

**Description**: MAVLink log download protocol for retrieving flight logs from vehicle to GCS for post-analysis.

**Use Cases**:

- Download flight logs in Mission Planner
- Post-mission analysis and debugging
- Long-term data collection

**MAVLink Messages**:

| Message            | Direction     | Description                          |
| ------------------ | ------------- | ------------------------------------ |
| `LOG_REQUEST_LIST` | GCS → Vehicle | Request available log list           |
| `LOG_ENTRY`        | Vehicle → GCS | Describe a log file (id, size, time) |
| `LOG_REQUEST_DATA` | GCS → Vehicle | Request log data bytes               |
| `LOG_DATA`         | Vehicle → GCS | Send log data chunk (90 bytes)       |
| `LOG_ERASE`        | GCS → Vehicle | Erase all logs                       |

**Implementation Considerations**:

- Flash-based log storage with index (existing ring buffer in core)
- Log segmentation: new log per arm/disarm cycle
- Chunk-based transfer with offset/count
- Memory-efficient streaming (don't load entire log to RAM)

**Dependencies**:

- Logging infrastructure (FR-00002, FR-00095–FR-00097)
- Flash storage

---

### FB-014: Home Position Management

**Status**: Proposed

**Description**: Track, report, and allow setting of home position for RTL and distance calculations.

**Use Cases**:

- RTL destination
- Distance-to-home display in GCS
- Allow GCS to override home position

**MAVLink Messages**:

| Message               | Direction     | Description                                           |
| --------------------- | ------------- | ----------------------------------------------------- |
| `HOME_POSITION`       | Vehicle → GCS | Report current home position (periodic)               |
| `SET_HOME_POSITION`   | GCS → Vehicle | Override home position                                |
| `MAV_CMD_DO_SET_HOME` | GCS → Vehicle | Set home to current position or specified coordinates |

**Behavior**:

1. Home position set automatically on first GPS fix after arming
2. GCS can override via SET_HOME_POSITION or DO_SET_HOME
3. Reported in HOME_POSITION message at \~1 Hz
4. Used by RTL, SmartRTL, battery RTL, and distance-to-home calculations

**Dependencies**:

- GPS position
- RTL mode (implemented)

---

### FB-015: Time Synchronization

**Status**: Proposed

**Description**: GPS-based time synchronization and SYSTEM_TIME MAVLink message for correlating vehicle logs with GCS timestamps.

**Use Cases**:

- Correlate flight logs with GCS event log
- Accurate timestamps in telemetry messages
- Sync multi-vehicle logs in post-analysis

**MAVLink Messages**:

| Message       | Direction     | Description                     |
| ------------- | ------------- | ------------------------------- |
| `SYSTEM_TIME` | Vehicle → GCS | Report Unix time and boot time  |
| `TIMESYNC`    | Both          | Round-trip time synchronization |

**Implementation Considerations**:

- GPS provides UTC time after fix
- Boot time from embassy monotonic clock
- Emit SYSTEM_TIME at \~1 Hz

**Dependencies**:

- GPS time (from NMEA/UBX time fields)
- Embassy time (existing)

---

## Telemetry & Statistics

### FB-008: Trip Statistics

**Status**: Proposed

**Description**: Track and report operational statistics: distance traveled, time, energy consumed.

**Use Cases**:

- Maintenance scheduling (distance-based service)
- Performance monitoring
- Usage logging and analytics

**Statistics to Track**:

| Statistic        | Description         | Unit    |
| ---------------- | ------------------- | ------- |
| `trip_distance`  | Distance since arm  | meters  |
| `total_distance` | Lifetime distance   | meters  |
| `trip_time`      | Time since arm      | seconds |
| `total_time`     | Lifetime armed time | seconds |
| `trip_energy`    | Energy since arm    | mAh     |
| `total_energy`   | Lifetime energy     | mAh     |

**MAVLink Reporting**:

- Use `SYS_STATUS` for current values
- Custom message or parameter for lifetime stats
- Log to Flash for persistence

**Implementation Considerations**:

- Distance: integrate GPS position deltas (handle GPS noise)
- Time: simple counter while armed
- Energy: integrate current (requires current sensor)
- Persistence: save to Flash periodically (wear consideration)
- Reset: trip stats on arm, lifetime stats never (or manual reset)

**Dependencies**:

- GPS position (for distance)
- Current sensor (for energy, optional)
- Flash storage (for persistence)
- Timer infrastructure

---

### FB-009: Battery Prediction

**Status**: Proposed

**Description**: Estimate remaining range and time based on current consumption rate.

**Use Cases**:

- Mission planning (can we complete this mission?)
- Safety decisions (should we RTL now?)
- Operator awareness

**Predictions**:

| Metric             | Description               |
| ------------------ | ------------------------- |
| Remaining time     | Minutes of operation left |
| Remaining distance | Meters of travel left     |
| Return feasibility | Can we make it home?      |

**Calculation Method**:

```
consumption_rate = mAh_used / distance_traveled  (mAh/m)
remaining_mAh = capacity - mAh_used
remaining_distance = remaining_mAh / consumption_rate

time_rate = mAh_used / time_elapsed  (mAh/s)
remaining_time = remaining_mAh / time_rate
```

**Accuracy Considerations**:

- Consumption varies with speed, terrain, load
- Use rolling average for rate calculation
- Conservative estimates (safety margin)
- Invalid when insufficient data (early in trip)

**MAVLink Reporting**:

- `BATTERY_STATUS` message extended fields
- Or custom telemetry message

**Implementation Considerations**:

- Requires current sensor for mAh tracking
- Minimum data threshold before predictions valid
- Rolling window for rate smoothing (e.g., last 60 seconds)
- Handle edge cases: stationary, very slow, just armed

**Dependencies**:

- Current sensor
- Trip statistics (FB-008)
- Battery capacity parameter

---

### FB-010: Mission Progress

**Status**: Proposed

**Description**: Report mission completion status: percentage complete, ETA, waypoints remaining.

**Use Cases**:

- Operator awareness during long missions
- Progress monitoring
- Time estimation for planning

**Metrics**:

| Metric                | Description                        |
| --------------------- | ---------------------------------- |
| `waypoints_total`     | Total waypoints in mission         |
| `waypoints_completed` | Waypoints passed                   |
| `waypoints_remaining` | Waypoints left                     |
| `percent_complete`    | Progress percentage                |
| `distance_remaining`  | Sum of remaining segment distances |
| `eta_seconds`         | Estimated time to completion       |

**Calculation**:

```
percent_complete = waypoints_completed / waypoints_total × 100

distance_remaining = sum(distance(wp[i], wp[i+1])) for i = current to end

eta = distance_remaining / current_speed
```

**MAVLink Reporting**:

- `MISSION_CURRENT` (42): Current waypoint sequence
- `NAV_CONTROLLER_OUTPUT` (62): Distance to waypoint
- Custom extension for ETA/percentage

**Implementation Considerations**:

- Waypoint count available from mission storage
- Distance calculation: sum of great-circle distances
- ETA: use average speed or current speed
- Update on waypoint transitions and periodically

**Dependencies**:

- Mission storage
- Current waypoint tracking
- Speed measurement
- Distance calculation utilities

---

## SITL Enhancements

### FB-021: SITL Sensor Fault Injection

**Status**: Proposed

**Description**: Simulate sensor failures and degraded conditions in SITL to test failsafe behavior.

**Use Cases**:

- Test GPS loss failsafe triggers RTL/Hold correctly
- Test battery low voltage triggers Battery RTL
- Verify RC loss handling
- Regression testing for safety-critical code paths

**Fault Types**:

| Fault                | Description                                     |
| -------------------- | ----------------------------------------------- |
| GPS loss             | Stop providing GPS data after N steps           |
| GPS drift            | Add large position offset to simulate multipath |
| Battery drain        | Simulate voltage drop over time                 |
| Battery critical     | Instant drop to critical voltage                |
| RC loss              | Stop RC input messages                          |
| IMU failure          | Send invalid/frozen IMU data                    |
| Compass interference | Add large magnetic offset                       |

**Implementation Considerations**:

- Inject at adapter level (LightweightAdapter or GazeboAdapter)
- Time-triggered or command-triggered (via test API or special MAVLink command)
- Deterministic with lockstep mode for repeatable CI tests
- Record fault injection events in test logs

**Dependencies**:

- SITL bridge (existing)
- Failsafe system (existing parameters)
- Lockstep time mode (existing)

---

### FB-022: SITL Scenario Test Framework

**Status**: Proposed

**Description**: Automated scenario tests using LightweightAdapter + lockstep for deterministic CI verification of autopilot behavior.

**Use Cases**:

- Regression testing: arm → auto mode → complete mission → disarm
- Failsafe verification: GPS loss during mission triggers RTL
- Mode transition testing: Manual → Guided → Auto → RTL
- Navigation accuracy: verify waypoint arrival within tolerance

**Scenario Examples**:

| Scenario         | Description                                       |
| ---------------- | ------------------------------------------------- |
| Basic mission    | Upload 3 waypoints, run Auto, verify all reached  |
| RTL on GPS loss  | Fly to waypoint 2, inject GPS loss, verify RTL    |
| Mode transitions | Cycle through all modes, verify state consistency |
| Arm/disarm cycle | Arm, run, disarm, verify clean shutdown           |
| Battery failsafe | Inject low voltage, verify Battery RTL triggers   |
| Geofence breach  | Navigate outside fence, verify RTL triggers       |

**Implementation Considerations**:

- Build on existing `cargo test -p pico_trail_sitl` infrastructure
- Each scenario: setup bridge → inject commands → step N times → assert state
- Deterministic via lockstep + seeded RNG
- Timeout protection for stuck tests
- Report: pass/fail + final vehicle state + step count

**Dependencies**:

- LightweightAdapter (existing)
- Lockstep time mode (existing)
- Fault injection (FB-021)

---

## Hardware Features

### FB-023: Servo/PWM Output Calibration

**Status**: Proposed

**Description**: Calibrate PWM output ranges (min/max/trim) to match physical actuators.

**Use Cases**:

- Different motors/ESCs have different PWM ranges
- Trim adjustment for mechanical asymmetry
- Reverse direction for incorrectly wired motors

**Key Parameters** (ArduPilot standard):

| Parameter         | Description                | Default |
| ----------------- | -------------------------- | ------- |
| `SERVO1_MIN`      | Minimum PWM (us)           | 1000    |
| `SERVO1_MAX`      | Maximum PWM (us)           | 2000    |
| `SERVO1_TRIM`     | Neutral PWM (us)           | 1500    |
| `SERVO1_REVERSED` | Reverse output             | 0       |
| `SERVO1_FUNCTION` | Output function assignment | 0       |

**Implementation Considerations**:

- Map normalized output (-1.0 to 1.0) to calibrated PWM range
- Per-channel configuration (up to 4 channels for differential drive)
- Calibration wizard via GCS (optional, manual parameter entry sufficient)

**Dependencies**:

- PWM output (existing)
- Parameter system (existing)

---

### FB-024: OTA Firmware Update

**Status**: Proposed

**Description**: Update firmware over WiFi or USB without physical BOOTSEL button press.

**Use Cases**:

- Field firmware update without disassembly
- Remote fleet management
- Faster development cycle

**Approaches**:

| Approach                | Pros             | Cons                                 |
| ----------------------- | ---------------- | ------------------------------------ |
| USB UF2 (current)       | Simple, reliable | Requires physical access             |
| WiFi HTTP upload        | Remote capable   | Needs bootloader, flash partitioning |
| MAVLink firmware upload | GCS integrated   | Complex, slow over MAVLink           |

**Implementation Considerations**:

- RP2350 has 4MB flash; dual-bank A/B partitioning possible
- Write new firmware to inactive bank, swap on reboot
- Verify firmware CRC before swap
- Fallback: if new firmware fails boot, revert to previous
- WiFi HTTP server for upload (simplest remote approach)

**Dependencies**:

- Flash partitioning
- CRC verification
- WiFi stack (existing cyw43)

---

## Boat-Specific Features

### FB-025: Boat Station Keeping

**Status**: Proposed

**Description**: Active position holding for boats, compensating for wind and current drift. More aggressive than rover Loiter due to continuous environmental forces.

**Use Cases**:

- Hold position during data collection
- Anchor-free station keeping
- Waiting at rendezvous point

**Behavior**:

1. Record target position
2. Continuously calculate position error
3. Apply proportional corrections (throttle + steering)
4. Handle heading into current/wind for efficiency
5. Configurable correction aggressiveness

**Implementation Considerations**:

- Extension of Loiter Type 1 with more aggressive tuning
- May need separate PID gains from rover navigation
- Optional: estimate current direction from drift pattern
- Power consumption: continuous motor operation

**Dependencies**:

- GPS position
- Navigation controller (existing)
- Loiter mode framework (implemented)

---

## Priority Matrix

| Feature                   | Complexity | Value  | Dependencies    | Priority     |
| ------------------------- | ---------- | ------ | --------------- | ------------ |
| FB-016 Hold Mode          | Very Low   | High   | Mode framework  | **Critical** |
| FB-014 Home Position      | Low        | High   | GPS             | **Critical** |
| FB-005 Comm Lost          | Low        | High   | Existing modes  | **High**     |
| FB-017 Geofence           | Medium     | High   | GPS, Home       | High         |
| FB-006 Pause/Resume       | Medium     | High   | Auto mode       | High         |
| FB-018 LED Patterns       | Very Low   | Medium | GPIO            | High         |
| FB-019 Buzzer Patterns    | Low        | Medium | PWM             | High         |
| FB-020 Watchdog           | Low        | High   | RP2350 HW       | Medium       |
| FB-021 Fault Injection    | Medium     | High   | SITL bridge     | Medium       |
| FB-022 Scenario Tests     | Medium     | High   | SITL, missions  | Medium       |
| FB-023 Servo Calibration  | Low        | Medium | PWM, params     | Medium       |
| FB-015 Time Sync          | Low        | Low    | GPS time        | Low          |
| FB-008 Trip Stats         | Low        | Medium | GPS, timer      | Low          |
| FB-010 Mission Progress   | Low        | Medium | Mission storage | Low          |
| FB-009 Battery Prediction | Medium     | Medium | Current sensor  | Low          |
| FB-013 Log Download       | Medium     | Medium | Flash storage   | Low          |
| FB-024 OTA Update         | High       | Medium | Flash, WiFi     | Low          |
| FB-025 Boat Station Keep  | Medium     | Medium | Loiter, nav     | Low          |

---

## Implementation Order Suggestion

**Phase 1: Critical Foundation**

1. Hold Mode (FB-016) - Simplest mode, failsafe fallback
2. Home Position (FB-014) - RTL/geofence prerequisite
3. Communication Lost Action (FB-005) - Safety critical, params already exist

**Phase 2: Safety & Operator UX**

4. Geofence (FB-017) - Area restriction
5. LED Patterns (FB-018) - Field status visibility
6. Buzzer Patterns (FB-019) - Audible alerts
7. Watchdog (FB-020) - Hang recovery

**Phase 3: Mission Enhancement**

8. Mission Pause/Resume (FB-006) - Common operational need

**Phase 4: SITL Testing Infrastructure**

9. Sensor Fault Injection (FB-021) - Failsafe test enabler
10. Scenario Test Framework (FB-022) - CI regression tests

**Phase 5: Telemetry & Hardware**

11. Servo Calibration (FB-023) - Hardware tuning
12. Trip Statistics (FB-008) - Foundation for predictions
13. Mission Progress (FB-010) - User awareness
14. Time Sync (FB-015) - Log correlation

**Phase 6: Advanced**

15. Battery Prediction (FB-009) - Requires current sensor
16. Log Download (FB-013) - Post-flight analysis
17. Boat Station Keeping (FB-025) - Boat-specific
18. OTA Update (FB-024) - Remote updates

---

## Notes

- All parameters follow ArduPilot naming conventions where applicable
- Features should be designed for both Rover and Boat vehicles
- Memory constraints (especially RP2040) should be considered
- Each feature should have corresponding Analysis document before implementation
