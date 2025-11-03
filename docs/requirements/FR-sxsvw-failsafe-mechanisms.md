# FR-sxsvw Failsafe Mechanisms

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-5nucb-core-systems](../analysis/AN-5nucb-core-systems.md)
- Prerequisite Requirements:
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
  - [FR-sp3at-vehicle-modes](FR-sp3at-vehicle-modes.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall implement GPS and RC failsafe mechanisms that detect signal loss within 1 second and execute a safe action (Hold mode or Return-to-Launch), preventing unsafe autonomous operation with degraded sensors.

## Rationale

Failsafe mechanisms are critical safety features that prevent catastrophic failures:

- **GPS Failsafe**: Loss of GPS fix means vehicle cannot navigate autonomously - must stop or return to known good location
- **RC Failsafe**: Loss of RC signal means operator cannot override autopilot - must take safe action
- **Battery Failsafe**: Low battery risks stranding vehicle or damaging battery - must return to launch or land

ArduPilot implements multiple failsafe layers with configurable actions. This ensures safe operation even with sensor or communication failures.

## User Story (if applicable)

As a safety officer, I want the autopilot to detect GPS and RC signal loss within 1 second and automatically transition to a safe mode (Hold or RTL), so that the vehicle does not continue operating dangerously when critical sensors or communication are unavailable.

## Acceptance Criteria

- [ ] GPS failsafe triggers within 1 second of losing GPS fix (no valid position)
- [ ] RC failsafe triggers within 1 second of losing RC signal
- [ ] Battery failsafe triggers when battery voltage drops below configurable threshold
- [ ] Failsafe action configurable: Hold (stop in place) or RTL (return to launch point)
- [ ] Failsafe events logged to Flash and reported via MAVLink
- [ ] Failsafe cleared automatically when sensor/signal restored (with hysteresis)
- [ ] Multiple simultaneous failsafes handled correctly (e.g., GPS + RC loss)
- [ ] GCS displays failsafe warnings in real-time

## Technical Details (if applicable)

### Functional Requirement Details

**GPS Failsafe:**

Trigger conditions:

- GPS fix type < 3D (no valid position)
- GPS fix lost for > 1 second continuous
- Horizontal accuracy > 10 meters (degraded fix)

Actions:

- If in Auto/Guided mode: Transition to Hold or RTL (configurable via `FS_GPS_ACTION`)
- If in Manual mode: No action (operator in control)
- Log event: "GPS Failsafe: Fix Lost"
- Send MAVLink message: `STATUSTEXT` with severity WARNING

Recovery:

- GPS fix restored and stable for > 5 seconds
- Clear failsafe, resume previous mode
- Log event: "GPS Failsafe: Cleared"

**RC Failsafe:**

Trigger conditions:

- RC signal lost for > 1 second continuous
- RC receiver reports failsafe condition

Actions:

- Transition to Hold or RTL (configurable via `FS_RC_ACTION`)
- Disarm after 10 seconds in Hold (prevent flyaway)
- Log event: "RC Failsafe: Signal Lost"
- Send MAVLink message: `STATUSTEXT` with severity CRITICAL

Recovery:

- RC signal restored and valid for > 2 seconds
- Clear failsafe, transition to Manual mode (safe default)
- Log event: "RC Failsafe: Cleared"

**Battery Failsafe:**

Trigger conditions:

- Battery voltage < `FS_BATT_VOLTAGE` (configurable, default 10.5V for 3S LiPo)
- Battery remaining capacity < `FS_BATT_CAPACITY` (configurable, default 20%)

Actions:

- Transition to RTL (return to launch to prevent stranding)
- Reduce maximum speed to conserve power
- Log event: "Battery Failsafe: Low Voltage"
- Send MAVLink message: `STATUSTEXT` with severity CRITICAL

Recovery:

- Voltage recovers above threshold + 0.5V (hysteresis)
- Clear failsafe, resume normal operation
- Log event: "Battery Failsafe: Cleared"

**Failsafe Parameters:**

```
FS_GPS_ACTION: 0=Hold, 1=RTL (default 0)
FS_RC_ACTION: 0=Hold, 1=RTL (default 0)
FS_BATT_VOLTAGE: Battery failsafe voltage (V), default 10.5
FS_BATT_CAPACITY: Battery failsafe capacity (%), default 20
FS_BATT_ACTION: 0=None, 1=RTL (default 1)
```

## Platform Considerations

### Pico W (RP2040)

N/A - Platform agnostic (failsafe logic independent of hardware)

### Pico 2 W (RP2350)

N/A - Platform agnostic

### Cross-Platform

Failsafe implementation should be platform-independent. Use platform abstraction for RC and battery voltage reading.

## Risks & Mitigation

| Risk                                        | Impact | Likelihood | Mitigation                                                  | Validation                                     |
| ------------------------------------------- | ------ | ---------- | ----------------------------------------------------------- | ---------------------------------------------- |
| False failsafe triggers (intermittent GPS)  | Medium | Medium     | Require continuous loss for 1 second, implement hysteresis  | Test with GPS antenna disconnected/reconnected |
| Failsafe action makes situation worse       | High   | Low        | Default to Hold (safest), make RTL optional, test scenarios | Review failsafe behavior in various modes      |
| Multiple failsafes conflict (GPS + RC loss) | Medium | Low        | Priority order: RC > GPS > Battery, handle sequentially     | Test with simultaneous failsafe triggers       |
| Delayed failsafe response (>1 second)       | High   | Low        | Monitor failsafe check timing, run at 10Hz minimum          | Measure actual failsafe trigger latency        |

## Implementation Notes

Preferred approaches:

- Implement **failsafe state machine** with clear states (NORMAL, GPS_FAILSAFE, RC_FAILSAFE, BATT_FAILSAFE)
- Run **failsafe checks at 10Hz** (every 100ms) to ensure 1-second detection time
- Use **hysteresis** for recovery (require stable signal for > 1x trigger time)

Known pitfalls:

- GPS fix can fluctuate rapidly (require sustained loss before triggering)
- Battery voltage drops under load (use moving average, not instantaneous reading)
- Failsafe in Manual mode should not override operator control
- RTL requires valid GPS fix - if GPS lost, fallback to Hold

Related code areas:

- `src/core/safety/failsafe.rs` - Failsafe state machine
- `src/vehicle/modes/` - Mode transition logic
- `src/devices/rc/` - RC receiver interface
- `src/devices/battery/` - Battery monitor

## External References

- ArduPilot Failsafe Documentation: <https://ardupilot.org/rover/docs/rover-failsafes.html>
- ArduPilot RC Failsafe: <https://ardupilot.org/rover/docs/apms-failsafe-function.html>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
