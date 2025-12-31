# Feature Backlog

This document tracks planned features outside the formal TDL process. Features here are candidates for future Analysis documents when prioritized for implementation.

## Status Legend

- **Proposed**: Initial idea, needs refinement
- **Refined**: Detailed enough for Analysis
- **In Progress**: Analysis/Requirements started
- **Done**: Implemented

---

## Control Modes

### FB-001: Circle Mode

**Status**: In Progress

**Description**: Navigate in a circular path around a specified center point at a configurable radius and speed.

**Use Cases**:

- Survey/inspection of a specific area
- Aerial photography patterns (adapted for ground)
- Perimeter patrol around a fixed point

**ArduPilot Reference**: [Circle Mode](https://ardupilot.org/rover/docs/circle-mode.html)

**Key Parameters** (ArduPilot standard):

| Parameter        | Description                         | Default |
| ---------------- | ----------------------------------- | ------- |
| `CIRCLE_RADIUS`  | Circle radius in meters             | 20      |
| `CIRCLE_RATE`    | Angular rate in deg/sec (+ = CW)    | 20      |
| `CIRCLE_OPTIONS` | Bitmask for circle behavior options | 0       |

**Behavior**:

1. When entering Circle mode, current position becomes initial center (or use commanded center)
2. Vehicle moves to circle perimeter at closest point
3. Follows circular path at specified radius and rate
4. Maintains constant ground speed based on radius and angular rate
5. Exit via mode change or RC input

**Implementation Considerations**:

- Requires heading control (steering) and speed control (throttle)
- Center point can be current position or GCS-commanded
- Direction (CW/CCW) controlled by sign of `CIRCLE_RATE`
- Speed = radius × angular_rate (rad/s)

**Dependencies**:

- GPS position
- Heading control (AHRS)
- Waypoint navigation infrastructure

---

### FB-002: Loiter Mode

**Status**: Proposed

**Description**: Hold current position. For rovers, this means stopping and actively correcting position drift.

**Use Cases**:

- Temporary stop during operation
- Position hold while waiting for commands
- Station keeping for boats

**ArduPilot Reference**: [Loiter Mode (Rover)](https://ardupilot.org/rover/docs/loiter-mode.html)

**Key Parameters** (ArduPilot standard):

| Parameter     | Description                      | Default |
| ------------- | -------------------------------- | ------- |
| `LOIT_RADIUS` | Loiter radius (0 = point loiter) | 2       |
| `LOIT_TYPE`   | 0=stop, 1=hold position          | 0       |

**Behavior**:

- **Type 0 (Stop)**: Simply stop motors, no active position holding
- **Type 1 (Hold)**: Actively correct position if drifted beyond threshold

**For Rovers**:

1. Record current position as loiter point
2. Stop motors
3. If position drifts > threshold, navigate back to loiter point
4. Repeat until mode change

**For Boats**:

1. More active station keeping due to current/wind
2. Continuous small corrections to maintain position
3. May involve forward/reverse and steering adjustments

**Implementation Considerations**:

- Simple stop mode is trivial (just zero outputs)
- Active hold requires position error calculation and correction
- Boat variant needs more aggressive correction due to drift
- Threshold for "close enough" prevents oscillation

**Dependencies**:

- GPS position
- Position error calculation
- Basic navigation to point

---

### FB-003: Smart RTL (Return to Launch)

**Status**: Proposed

**Description**: Return to launch point by retracing the path taken, rather than direct line. Useful for avoiding obstacles encountered on the way out.

**Use Cases**:

- Return through complex terrain
- Avoid retracing through obstacles
- Safe return when direct path is unknown

**ArduPilot Reference**: [SmartRTL](https://ardupilot.org/rover/docs/smartrtl-mode.html)

**Key Parameters** (ArduPilot standard):

| Parameter       | Description                           | Default |
| --------------- | ------------------------------------- | ------- |
| `SRTL_ACCURACY` | Path simplification accuracy (meters) | 2       |
| `SRTL_POINTS`   | Max breadcrumb points to store        | 300     |

**Behavior**:

1. During operation, periodically record "breadcrumb" positions
2. Apply path simplification (Douglas-Peucker algorithm) to reduce points
3. When SmartRTL activated:
   - Navigate to nearest breadcrumb
   - Follow breadcrumbs in reverse order
   - End at launch point
4. If breadcrumb buffer full, oldest points pruned (with path simplification)

**Path Simplification**:

- Douglas-Peucker algorithm reduces points while preserving path shape
- `SRTL_ACCURACY` controls how much simplification is allowed
- Balance between memory usage and path fidelity

**Memory Considerations** (RP2350: 520KB RAM):

- Each point: \~12 bytes (lat/lon as i32 + alt as i16)
- 300 points: \~3.6KB
- Acceptable for RP2350, may need reduction for RP2040

**Implementation Considerations**:

- Breadcrumb recording interval: distance-based (e.g., every 3m) not time-based
- Path simplification runs periodically or when buffer near full
- Waypoint navigation infrastructure reused for following breadcrumbs
- Fallback to direct RTL if breadcrumbs unavailable

**Dependencies**:

- GPS position
- Waypoint navigation
- Path simplification algorithm
- Persistent breadcrumb storage (RAM, not Flash)

---

## Safety Features

### FB-004: Battery RTL

**Status**: In Progress

**Analysis**: [AN-vhavw-battery-rtl](analysis/AN-vhavw-battery-rtl.md)

**Description**: Automatic Return to Launch when battery reaches critical level, with distance-aware thresholds.

**Use Cases**:

- Prevent vehicle stranding due to empty battery
- Automatic safety return without operator intervention
- Distance-aware decision making

**ArduPilot Reference**: [Failsafe](https://ardupilot.org/rover/docs/rover-failsafes.html)

**Key Parameters** (ArduPilot standard):

| Parameter         | Description                                            | Default |
| ----------------- | ------------------------------------------------------ | ------- |
| `BATT_LOW_VOLT`   | Low battery voltage threshold                          | 10.5    |
| `BATT_CRT_VOLT`   | Critical battery voltage threshold                     | 10.0    |
| `BATT_FS_LOW_ACT` | Low battery action (0=none, 1=RTL, 2=hold, 3=SmartRTL) | 0       |
| `BATT_FS_CRT_ACT` | Critical battery action                                | 0       |
| `BATT_CAPACITY`   | Battery capacity in mAh                                | 0       |
| `BATT_LOW_MAH`    | Low battery remaining mAh                              | 0       |
| `BATT_CRT_MAH`    | Critical battery remaining mAh                         | 0       |

**Behavior Levels**:

1. **Low Battery** (`BATT_LOW_VOLT` or `BATT_LOW_MAH`):
   - Warning notification (buzzer, telemetry)
   - Optional action: RTL, Hold, SmartRTL, or continue

2. **Critical Battery** (`BATT_CRT_VOLT` or `BATT_CRT_MAH`):
   - Urgent notification
   - Forced action: typically RTL or SmartRTL

**Distance-Aware Enhancement** (beyond ArduPilot standard):

- Calculate estimated battery needed to return home
- Factor in: distance to home, average consumption rate, safety margin
- Trigger RTL when: remaining_capacity < return_estimate × safety_factor

**Implementation Considerations**:

- Requires battery voltage/current monitoring (ADC)
- mAh tracking requires current sensor integration
- Consumption estimation from historical data
- Hysteresis to prevent oscillation near threshold

**Dependencies**:

- Battery voltage ADC
- Optional: Current sensor
- RTL or SmartRTL mode
- Distance to home calculation

---

### FB-005: Communication Lost Action

**Status**: Proposed

**Description**: Configurable behavior when GCS communication is lost (no heartbeat received).

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
- RTL/SmartRTL/Hold modes
- Mode switching infrastructure

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

### FB-007: Speed Profile

**Status**: Proposed

**Description**: Configure different maximum speeds for different mission segments or areas.

**Use Cases**:

- Slow down in confined areas
- Speed up on open terrain
- Precision vs. speed trade-off per waypoint

**ArduPilot Reference**: `DO_CHANGE_SPEED` command

**MAVLink Implementation**:

Mission command `MAV_CMD_DO_CHANGE_SPEED` (178):

| Parameter | Description                   |
| --------- | ----------------------------- |
| param1    | Speed type (0=ground speed)   |
| param2    | Speed in m/s (-1 = no change) |
| param3    | Throttle % (-1 = no change)   |

**Behavior**:

1. Default speed from `WP_SPEED` parameter
2. `DO_CHANGE_SPEED` command in mission changes speed
3. New speed applies to subsequent waypoints
4. Persists until next `DO_CHANGE_SPEED` or mission end

**Key Parameters** (ArduPilot standard):

| Parameter      | Description                  | Default |
| -------------- | ---------------------------- | ------- |
| `WP_SPEED`     | Default waypoint speed (m/s) | 2       |
| `WP_SPEED_MIN` | Minimum speed (m/s)          | 0       |

**Implementation Considerations**:

- Speed stored in mission item, not separate data structure
- Apply speed limit in throttle/speed controller
- Smooth speed transitions (acceleration limiting)
- Interaction with terrain/slope (future)

**Dependencies**:

- Mission item parsing
- Speed controller
- Acceleration limiting (existing S-curve planner)

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

## Priority Matrix

| Feature                   | Complexity | Value  | Dependencies      | Priority |
| ------------------------- | ---------- | ------ | ----------------- | -------- |
| FB-002 Loiter             | Low        | High   | GPS, basic nav    | High     |
| FB-005 Comm Lost          | Low        | High   | Existing modes    | High     |
| FB-006 Pause/Resume       | Medium     | High   | Auto mode         | High     |
| FB-001 Circle             | Medium     | Medium | Heading control   | Medium   |
| FB-003 SmartRTL           | High       | High   | Path storage, nav | Medium   |
| FB-004 Battery RTL        | Medium     | High   | Battery monitor   | Medium   |
| FB-007 Speed Profile      | Low        | Medium | Mission parser    | Medium   |
| FB-008 Trip Stats         | Low        | Medium | GPS, timer        | Low      |
| FB-010 Mission Progress   | Low        | Medium | Mission storage   | Low      |
| FB-009 Battery Prediction | Medium     | Medium | Current sensor    | Low      |

---

## Implementation Order Suggestion

**Phase 1: Foundation Modes**

1. Loiter (FB-002) - Simple, high value
2. Communication Lost Action (FB-005) - Safety critical

**Phase 2: Mission Enhancement**

3. Mission Pause/Resume (FB-006) - Common need
4. Speed Profile (FB-007) - Low effort, useful

**Phase 3: Advanced Modes**

5. Circle (FB-001) - Survey capability
6. SmartRTL (FB-003) - Complex but valuable

**Phase 4: Safety & Monitoring**

7. Battery RTL (FB-004) - Requires battery monitoring
8. Trip Statistics (FB-008) - Foundation for predictions
9. Mission Progress (FB-010) - User awareness
10. Battery Prediction (FB-009) - Requires current sensor

---

## Notes

- All parameters follow ArduPilot naming conventions where applicable
- Features should be designed for both Rover and Boat vehicles
- Memory constraints (especially RP2040) should be considered
- Each feature should have corresponding Analysis document before implementation
