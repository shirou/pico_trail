# AN-00035 Battery RTL | Automatic Return on Low Battery

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-00011-failsafe-system](AN-00011-failsafe-system.md)
  - [AN-00019-battery-telemetry](AN-00019-battery-telemetry.md)
  - [AN-00036-rtl-mode](AN-00036-rtl-mode.md) (prerequisite)
- Related Requirements: (To be created after approval)
- Related ADRs: (To be created after approval)
- Related Tasks: (To be created after approval)

## Executive Summary

This analysis explores automatic Return to Launch (RTL) functionality triggered by battery voltage or capacity thresholds. Currently, pico_trail has battery voltage monitoring and basic failsafe parameters (BATT_CRT_VOLT, BATT_FS_CRT_ACT), but lacks distance-aware battery failsafe logic that can determine whether the vehicle has sufficient power to return home. Without Battery RTL, vehicles risk stranding when batteries deplete during operation, potentially causing damage or loss.

Key findings: ArduPilot implements a two-stage battery failsafe system (LOW and CRITICAL thresholds) with configurable actions including RTL, Hold, SmartRTL, and Disarm. The recommended approach for pico_trail is to implement voltage-based battery failsafe first (Phase 1), then add capacity-based monitoring and distance-aware RTL decision logic in subsequent phases.

## Problem Space

### Current State

The project currently has:

- **Battery State Tracking**: `BatteryState` struct in `src/communication/mavlink/state.rs:97` with voltage, current, and remaining_percent fields
- **Battery Parameters**: `BatteryParams` in `src/parameters/battery.rs` with BATT_ARM_VOLT, BATT_CRT_VOLT, BATT_FS_CRT_ACT, BATT_VOLT_MULT
- **Battery Telemetry**: SYS_STATUS message includes battery voltage (placeholder values currently)
- **RTL Mode Definition**: `FlightMode::Rtl` enum exists in `src/communication/mavlink/state.rs:88` but **NOT implemented** (no `src/rover/mode/rtl.rs`)
- **Home Position Tracking**: `HomePosition` struct in `src/communication/mavlink/state.rs:213` stores RTL destination
- **Failsafe Framework Analysis**: [AN-00011-failsafe-system](AN-00011-failsafe-system.md) provides comprehensive failsafe architecture

**Critical gaps**:

- **No automatic RTL trigger**: Battery voltage crossing threshold does not trigger mode change
- **No LOW threshold**: Only BATT_CRT_VOLT exists, no BATT_LOW_VOLT for early warning
- **No distance-aware logic**: Cannot estimate if battery is sufficient to return home
- **No capacity monitoring**: No BATT_LOW_MAH or BATT_CRT_MAH parameters
- **RTL mode not implemented**: Mode exists in enum but no navigation logic (**MAJOR PREREQUISITE** - requires separate implementation before Battery RTL can use RTL action)

### Desired State

Implement automatic battery-based RTL with the following capabilities:

1. **Two-Stage Battery Failsafe**: LOW threshold (warning + configurable action) and CRITICAL threshold (urgent action)
2. **Distance-Aware Decision**: Estimate battery needed for return, trigger RTL before point of no return
3. **Configurable Actions**: Hold, RTL, SmartRTL, or Disarm per threshold level
4. **Operator Notification**: STATUSTEXT messages for battery warnings and failsafe activation
5. **GCS Integration**: Battery thresholds visible and configurable from Mission Planner/QGC

Success criteria:

- Vehicle automatically initiates RTL when battery drops below configured threshold
- Distance-aware mode ensures vehicle can always return home if RTL action is configured
- No vehicle stranding due to battery depletion (assuming proper threshold configuration)
- All battery failsafe events logged for post-operation analysis

### Gap Analysis

**Missing components**:

1. **Battery Failsafe Monitor**: Check voltage/capacity against thresholds in vehicle control loop
2. **LOW Battery Parameters**: BATT_LOW_VOLT and BATT_LOW_MAH for early warning
3. **LOW Battery Action**: BATT_FS_LOW_ACT parameter for warning-level response
4. **Distance Estimation**: Calculate distance to home and required battery
5. **Consumption Rate Tracking**: Track mAh/meter or mAh/second for prediction
6. **RTL Mode Implementation**: Navigate from current position to home
7. **Failsafe Integration**: Connect battery monitor to failsafe executor

**Technical deltas**:

- Add BATT_LOW_VOLT, BATT_LOW_MAH, BATT_FS_LOW_ACT parameters
- Implement battery failsafe checker in `src/vehicle/failsafe/`
- Integrate battery checker with FailsafeExecutor from AN-00011
- Implement RTL mode navigation logic
- Add distance-to-home calculation
- Add consumption rate estimation (optional for Phase 1)

## Stakeholder Analysis

| Stakeholder        | Interest/Need                                      | Impact | Priority |
| ------------------ | -------------------------------------------------- | ------ | -------- |
| Operators          | Prevent vehicle loss due to battery depletion      | High   | P0       |
| Autonomous Mission | Complete missions safely with automatic RTL        | High   | P0       |
| Safety Reviewers   | Predictable battery failsafe behavior              | High   | P0       |
| Ground Control     | Configure and monitor battery thresholds           | High   | P0       |
| Developers         | Test battery failsafe without physical low battery | Medium | P1       |

## Research & Discovery

### User Feedback

From operational safety requirements and common ArduPilot community feedback:

- Battery failsafe is essential for any autonomous vehicle
- Two-stage thresholds (LOW and CRITICAL) provide graduated response
- Distance-aware RTL prevents stranding at remote locations
- Voltage-based thresholds are simpler but less accurate than capacity-based
- Operators want configurable actions (some prefer Hold over RTL in confined spaces)

### Competitive Analysis

**ArduPilot Rover Battery Failsafe**:

ArduPilot implements battery failsafe in `libraries/AP_BattMonitor/` with handler callback to vehicle:

```cpp
// Battery failsafe handler (from Rover/failsafe.cpp)
void Rover::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    switch (action) {
        case Failsafe_Action_None:
            return;
        case Failsafe_Action_SmartRTL:
            if (set_mode(mode_smart_rtl, ModeReason::BATTERY_FAILSAFE)) {
                return;
            }
            [[fallthrough]];
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

**ArduPilot Battery Parameters**:

| Parameter       | Type  | Default | Description                           |
| --------------- | ----- | ------- | ------------------------------------- |
| BATT_LOW_VOLT   | float | 0       | Low battery voltage threshold (0=off) |
| BATT_CRT_VOLT   | float | 0       | Critical battery voltage threshold    |
| BATT_LOW_MAH    | float | 0       | Low battery capacity threshold (mAh)  |
| BATT_CRT_MAH    | float | 0       | Critical battery capacity threshold   |
| BATT_FS_LOW_ACT | u8    | 0       | Low battery action (0-5)              |
| BATT_FS_CRT_ACT | u8    | 0       | Critical battery action               |
| BATT_CAPACITY   | float | 0       | Battery capacity in mAh               |

**Failsafe Action Values**:

| Value | Action           | Description                    |
| ----- | ---------------- | ------------------------------ |
| 0     | None             | Warning only, no mode change   |
| 1     | Hold             | Stop and hold position         |
| 2     | RTL              | Return to launch point         |
| 3     | SmartRTL         | Return via recorded path       |
| 4     | SmartRTL_Hold    | Try SmartRTL, fallback to Hold |
| 5     | Terminate/Disarm | Immediate motor stop           |

**Voltage vs Capacity Monitoring**:

- **Voltage-based**: Simpler, no current sensor needed, but less accurate (voltage sag under load)
- **Capacity-based**: More accurate, requires current sensor, tracks mAh consumed
- **ArduPilot approach**: Support both, use whichever threshold triggers first

### Technical Investigation

**Current pico_trail Battery Infrastructure**:

File: `src/parameters/battery.rs`

- `BATT_ARM_VOLT`: Minimum voltage to arm (default 5.0V)
- `BATT_CRT_VOLT`: Critical voltage threshold (default 3.0V)
- `BATT_FS_CRT_ACT`: Critical action (default Land)
- `BATT_VOLT_MULT`: ADC conversion coefficient (default 3.95)

**Missing Parameters**:

- `BATT_LOW_VOLT`: Low voltage threshold (warning level)
- `BATT_LOW_MAH`: Low capacity threshold
- `BATT_CRT_MAH`: Critical capacity threshold
- `BATT_FS_LOW_ACT`: Low battery action
- `BATT_CAPACITY`: Total battery capacity

**Proposed Battery Failsafe Implementation**:

```rust
/// Battery failsafe state
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BatteryFailsafeLevel {
    None,
    Low,
    Critical,
}

/// Battery failsafe checker
pub struct BatteryFailsafeChecker {
    /// Current failsafe level
    level: BatteryFailsafeLevel,
    /// Time when LOW threshold first crossed
    low_trigger_time_ms: Option<u32>,
    /// Hysteresis: require voltage below threshold for this duration
    hysteresis_ms: u32,
}

impl BatteryFailsafeChecker {
    const DEFAULT_HYSTERESIS_MS: u32 = 10_000; // 10 seconds

    pub fn new() -> Self {
        Self {
            level: BatteryFailsafeLevel::None,
            low_trigger_time_ms: None,
            hysteresis_ms: Self::DEFAULT_HYSTERESIS_MS,
        }
    }

    /// Check battery status and return new failsafe level if changed
    pub fn check(&mut self, battery: &BatteryState, params: &BatteryParams,
                 current_time_ms: u32) -> Option<BatteryFailsafeLevel> {
        let voltage = battery.voltage;

        // Check CRITICAL first (no hysteresis, immediate)
        if params.batt_crt_volt > 0.0 && voltage < params.batt_crt_volt {
            if self.level != BatteryFailsafeLevel::Critical {
                self.level = BatteryFailsafeLevel::Critical;
                return Some(BatteryFailsafeLevel::Critical);
            }
            return None;
        }

        // Check LOW with hysteresis
        if params.batt_low_volt > 0.0 && voltage < params.batt_low_volt {
            match self.low_trigger_time_ms {
                None => {
                    // Start hysteresis timer
                    self.low_trigger_time_ms = Some(current_time_ms);
                }
                Some(start_time) => {
                    // Check if hysteresis period elapsed
                    if current_time_ms.wrapping_sub(start_time) >= self.hysteresis_ms {
                        if self.level != BatteryFailsafeLevel::Low {
                            self.level = BatteryFailsafeLevel::Low;
                            return Some(BatteryFailsafeLevel::Low);
                        }
                    }
                }
            }
        } else {
            // Voltage recovered, reset LOW timer
            self.low_trigger_time_ms = None;
            if self.level == BatteryFailsafeLevel::Low {
                self.level = BatteryFailsafeLevel::None;
                // Note: Don't auto-recover from CRITICAL
            }
        }

        None
    }
}
```

**Distance-Aware RTL Logic** (Phase 2):

```rust
/// Estimate if vehicle can return home with current battery
pub fn can_return_home(
    current_pos: &Position,
    home_pos: &HomePosition,
    battery: &BatteryState,
    consumption_rate_mah_per_meter: f32,
    safety_margin: f32,  // e.g., 1.3 = 30% safety margin
) -> bool {
    let distance_to_home = current_pos.distance_to(home_pos);
    let required_mah = distance_to_home * consumption_rate_mah_per_meter * safety_margin;
    let remaining_mah = battery.remaining_mah();

    remaining_mah > required_mah
}

/// Calculate point-of-no-return distance
pub fn max_outbound_distance(
    battery_capacity_mah: f32,
    consumption_rate_mah_per_meter: f32,
    safety_margin: f32,
) -> f32 {
    // Round trip: out + back
    let usable_mah = battery_capacity_mah / safety_margin;
    usable_mah / (2.0 * consumption_rate_mah_per_meter)
}
```

**Memory Analysis**:

| Component              | RAM Usage  | Notes                      |
| ---------------------- | ---------- | -------------------------- |
| BatteryFailsafeChecker | \~16 B     | Level, timer, hysteresis   |
| Battery params (new)   | \~20 B     | 5 new float parameters     |
| Consumption tracking   | \~12 B     | Rate calculation state     |
| **Total (Phase 1)**    | **\~36 B** | Voltage-based only         |
| **Total (Phase 2)**    | **\~48 B** | With capacity and distance |

### Data Analysis

**Typical 3S LiPo Voltage Thresholds**:

| State            | Voltage | Per Cell | Action              |
| ---------------- | ------- | -------- | ------------------- |
| Fully charged    | 12.6V   | 4.20V    | Normal operation    |
| Nominal          | 11.1V   | 3.70V    | Normal operation    |
| Low warning      | 10.5V   | 3.50V    | Initiate RTL        |
| Critical         | 10.0V   | 3.33V    | Immediate Hold/Land |
| Absolute minimum | 9.0V    | 3.00V    | Damage threshold    |

**Recommended Default Values**:

- `BATT_LOW_VOLT`: 10.5V (3.5V/cell, 20% remaining capacity)
- `BATT_CRT_VOLT`: 10.0V (3.33V/cell, 10% remaining capacity)
- `BATT_FS_LOW_ACT`: RTL (2) - Return home on low battery
- `BATT_FS_CRT_ACT`: Hold (1) - Stop immediately on critical

**Consumption Rate Estimation** (for distance-aware logic):

- Typical rover consumption: 1-5 A depending on motors and speed
- Average consumption: \~2A at moderate speed
- 3S 3300mAh battery: \~1 hour runtime at 2A
- At 2 m/s speed: \~7200m range, consumption \~0.46 mAh/m
- Safety margin: 30% recommended (multiply required by 1.3)

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: System shall trigger configurable action when battery voltage drops below LOW threshold
  - Rationale: Provide early warning before battery reaches critical level
  - Acceptance Criteria:
    - Monitor battery voltage at 10 Hz minimum
    - Trigger when voltage < BATT_LOW_VOLT for 10+ seconds (hysteresis)
    - Execute BATT_FS_LOW_ACT action (None, Hold, RTL, SmartRTL, Disarm)
    - Send STATUSTEXT: "Battery LOW: {voltage}V"
    - Do not trigger if BATT_LOW_VOLT = 0 (disabled)

- [ ] **FR-DRAFT-2**: System shall trigger immediate action when battery voltage drops below CRITICAL threshold
  - Rationale: Prevent damage from over-discharge, ensure minimum safe landing power
  - Acceptance Criteria:
    - Trigger immediately when voltage < BATT_CRT_VOLT (no hysteresis)
    - Execute BATT_FS_CRT_ACT action (typically Hold or Land)
    - CRITICAL overrides LOW action (higher priority)
    - Send STATUSTEXT: "Battery CRITICAL: {voltage}V"
    - Log failsafe event with voltage and action

- [ ] **FR-DRAFT-3**: System shall provide configurable LOW battery parameters
  - Rationale: Allow operators to customize warning threshold per battery type
  - Acceptance Criteria:
    - Parameter: BATT_LOW_VOLT (float, 0.0-30.0V, default 0.0=disabled)
    - Parameter: BATT_FS_LOW_ACT (u8, 0-5, default 0=None)
    - Parameters visible and editable in Mission Planner/QGC
    - Parameters persist to storage

- [ ] **FR-DRAFT-4**: System shall execute RTL action when configured for battery failsafe
  - Rationale: Automatically return vehicle to safe location before battery depletes
  - Acceptance Criteria:
    - Switch to RTL mode when battery failsafe action = RTL (2)
    - Navigate toward home position (straight-line or waypoint path)
    - Maintain RTL mode until reaching home or operator override
    - If RTL not possible (no GPS, no home), fallback to Hold
    - Log RTL initiation and progress

- [ ] **FR-DRAFT-5**: System shall track battery consumption for prediction (Phase 2)
  - Rationale: Enable distance-aware RTL decision before point of no return
  - Acceptance Criteria:
    - Track mAh consumed since arm (requires current sensor)
    - Calculate consumption rate (mAh/meter or mAh/second)
    - Estimate remaining range based on consumption rate
    - Trigger RTL when remaining range < distance to home + margin
    - Parameter: BATT_CAPACITY (float, mAh, for percentage calculation)

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Battery failsafe check shall complete within 10ms
  - Category: Performance
  - Rationale: Must not delay vehicle control loop
  - Target: Check voltage, compare threshold, return in <10ms

- [ ] **NFR-DRAFT-2**: Battery failsafe shall trigger within 1 second of threshold crossing
  - Category: Reliability
  - Rationale: Timely response critical for safety
  - Target: Detection + action initiation < 1s for CRITICAL, < 11s for LOW (with hysteresis)

- [ ] **NFR-DRAFT-3**: Battery failsafe parameters shall add no more than 50 bytes to parameter storage
  - Category: Resource Constraints
  - Rationale: Limited parameter storage (256 entries)
  - Target: 5 new parameters × 8 bytes = 40 bytes

- [ ] **NFR-DRAFT-4**: All battery failsafe events shall be logged
  - Category: Auditability
  - Rationale: Support post-operation analysis and debugging
  - Target: Log voltage, threshold, action, timestamp for each event

## Design Considerations

### Technical Constraints

- **RTL mode dependency**: RTL action requires RTL mode to be implemented
- **GPS dependency**: RTL requires valid GPS fix and home position
- **Current sensor optional**: Capacity-based monitoring requires current sensor (not on Freenove)
- **Memory budget**: Limited RAM on RP2040/RP2350
- **Failsafe framework**: Must integrate with existing failsafe system from AN-00011
- **Existing parameters**: Must not conflict with current BATT\_\* parameters

### Potential Approaches

1. **Option A: Voltage-Only Battery Failsafe (Minimal)**
   - Pros:
     - No current sensor required
     - Simpler implementation
     - Matches current Freenove hardware
   - Cons:
     - Less accurate (voltage sag under load)
     - No distance-aware capability
     - Cannot predict remaining range
   - Effort: Low (8-16 hours)

2. **Option B: Full ArduPilot-Compatible Battery Failsafe** (Recommended)
   - Pros:
     - ArduPilot parameter compatibility
     - Supports both voltage and capacity thresholds
     - Two-stage (LOW + CRITICAL) response
     - Extensible for distance-aware logic
   - Cons:
     - More parameters to implement
     - Capacity features require current sensor
   - Effort: Medium (24-40 hours)

3. **Option C: Distance-Aware Smart RTL**
   - Pros:
     - Optimal safety - never strand vehicle
     - Dynamic threshold based on distance
     - Accounts for variable consumption
   - Cons:
     - Complex consumption modeling
     - Requires current sensor
     - Terrain/wind not accounted for
   - Effort: High (40-60 hours)

**Recommendation**: Option B for Phase 1 (voltage-based with two thresholds), then Option C in Phase 2 (add distance awareness when current sensing available).

### Architecture Impact

**New ADRs required**:

- ADR for battery failsafe integration with failsafe executor
- ADR for RTL mode navigation strategy

**New modules**:

- `src/vehicle/failsafe/battery.rs` - Battery failsafe checker
- `src/rover/mode/rtl.rs` - RTL mode implementation

**Modified modules**:

- `src/parameters/battery.rs` - Add BATT_LOW_VOLT, BATT_FS_LOW_ACT, BATT_CAPACITY
- `src/vehicle/failsafe/executor.rs` - Integrate battery checker
- `src/rover/mode/mod.rs` - Register RTL mode

## ArduPilot Parameters

This analysis references the following ArduPilot Rover parameters:

### Existing Parameters (already in pico_trail)

- **BATT_ARM_VOLT** (float, default varies): Minimum voltage to arm
- **BATT_CRT_VOLT** (float, default 0): Critical battery voltage threshold
- **BATT_FS_CRT_ACT** (u8, default 0): Critical battery failsafe action

### New Parameters (to be added)

- **BATT_LOW_VOLT** (float, default 0, range 0-30V)
  - Low battery voltage threshold
  - 0 = disabled
  - Typical: 10.5V for 3S LiPo (3.5V/cell)
  - ArduPilot reference: <https://ardupilot.org/rover/docs/parameters.html#batt-low-volt>

- **BATT_FS_LOW_ACT** (u8, default 0, range 0-5)
  - Action when low battery threshold reached
  - 0=None, 1=Hold, 2=RTL, 3=SmartRTL, 4=SmartRTL_Hold, 5=Disarm
  - ArduPilot reference: <https://ardupilot.org/rover/docs/parameters.html#batt-fs-low-act>

- **BATT_LOW_MAH** (float, default 0, range 0-∞) (Phase 2)
  - Low battery capacity threshold in mAh remaining
  - 0 = disabled
  - Requires current sensor

- **BATT_CRT_MAH** (float, default 0, range 0-∞) (Phase 2)
  - Critical battery capacity threshold in mAh remaining
  - Requires current sensor

- **BATT_CAPACITY** (float, default 0, range 0-∞) (Phase 2)
  - Battery capacity in mAh
  - Used for percentage and range calculations
  - Typical: 3300 for 3S 3300mAh pack

## Risk Assessment

| Risk                                                      | Probability | Impact | Mitigation Strategy                                         |
| --------------------------------------------------------- | ----------- | ------ | ----------------------------------------------------------- |
| Voltage sag causes false LOW trigger during acceleration  | Medium      | Medium | 10-second hysteresis on LOW threshold                       |
| RTL mode not available when failsafe triggers             | Medium      | High   | Fallback chain: RTL → Hold, always succeed                  |
| Home position not set when RTL needed                     | Medium      | High   | Check home_position before RTL, fallback to Hold if not set |
| GPS loss during RTL navigation                            | Low         | High   | Switch to Hold on GPS loss, notify operator                 |
| Threshold misconfiguration (too high voltage)             | High        | Low    | Validate parameters, warn if LOW > 12V or CRT > LOW         |
| Battery recovers above threshold during RTL (oscillation) | Medium      | Low    | Do not auto-exit failsafe mode, require manual override     |
| Insufficient battery to complete RTL                      | Low         | High   | Phase 2: Distance-aware trigger before point of no return   |
| Current sensor not available for capacity monitoring      | Certain     | Medium | Phase 1 uses voltage only, document limitation              |

## Open Questions

- [x] Should LOW failsafe auto-clear when voltage recovers? → Decision: No, require manual mode switch (safer)
- [x] What hysteresis value for LOW threshold? → Decision: 10 seconds (matches ArduPilot)
- [ ] Should CRITICAL have any hysteresis? → Method: Review ArduPilot implementation, likely immediate
- [ ] Do we implement SmartRTL option in Phase 1? → Method: Check SmartRTL dependency, may defer to Phase 2
- [ ] What fallback if RTL not possible (no GPS/home)? → Decision: Fallback to Hold mode
- [ ] Should battery failsafe work when disarmed? → Decision: No, only check when armed
- [ ] How to handle multiple batteries? → Method: Defer to future analysis (single battery for now)

## Recommendations

### Immediate Actions

**Prerequisites** (must be implemented first):

1. **Implement RTL mode** (`src/rover/mode/rtl.rs`) - Currently only enum exists, no navigation logic
2. **Implement Hold mode** - Fallback when RTL not possible (no GPS/home)

**Battery RTL Implementation**:

3. **Add BATT_LOW_VOLT and BATT_FS_LOW_ACT parameters** to `src/parameters/battery.rs`
4. **Implement BatteryFailsafeChecker** in `src/vehicle/failsafe/battery.rs`
5. **Integrate with FailsafeExecutor** from AN-00011 failsafe framework

### Next Steps

1. [ ] Create formal requirements: FR for battery failsafe, FR for RTL mode, NFR for latency
2. [ ] Draft ADR for: Battery failsafe integration strategy
3. [ ] Draft ADR for: RTL navigation approach (direct line vs. waypoint path)
4. [ ] Create task for: Implement Battery RTL Phase 1 (voltage-based)
5. [ ] Further investigation: Current sensor integration for capacity monitoring (Phase 2)

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **Capacity-based thresholds**: BATT_LOW_MAH, BATT_CRT_MAH (requires current sensor, Phase 2)
- **Distance-aware RTL**: Automatic trigger based on return distance (Phase 2)
- **SmartRTL integration**: Return via recorded path (depends on SmartRTL mode, Phase 2)
- **Multiple battery support**: BATT2\_\* parameters (single battery sufficient initially)
- **Cell voltage monitoring**: Per-cell voltage (requires cell tap hardware)
- **Temperature failsafe**: Battery temperature monitoring (no temperature sensor)
- **Predictive remaining time**: Battery runtime estimation (Phase 2 with consumption tracking)

## Appendix

### References

- ArduPilot Rover Failsafes: <https://ardupilot.org/rover/docs/rover-failsafes.html>
- ArduPilot Battery Parameters: <https://ardupilot.org/rover/docs/parameters.html#batt-parameters>
- ArduPilot Battery Failsafe Code: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/failsafe.cpp>
- MAVLink BATTERY_STATUS: <https://mavlink.io/en/messages/common.html#BATTERY_STATUS>
- Project failsafe analysis: [AN-00011-failsafe-system](AN-00011-failsafe-system.md)
- Project battery telemetry: [AN-00019-battery-telemetry](AN-00019-battery-telemetry.md)
- Feature backlog: `docs/feature-backlog.md` (FB-004)

### Raw Data

**ArduPilot Battery Failsafe Code**:

```cpp
// From libraries/AP_BattMonitor/AP_BattMonitor.cpp
AP_BattMonitor::Failsafe AP_BattMonitor::check_failsafe(uint8_t instance)
{
    const float voltage = state[instance].voltage;
    const float consumed_mah = state[instance].consumed_mah;

    // Check voltage threshold
    if (_params[instance]._critical_voltage > 0 &&
        voltage > 0 &&
        voltage < _params[instance]._critical_voltage) {
        return Failsafe::Critical;
    }

    if (_params[instance]._low_voltage > 0 &&
        voltage > 0 &&
        voltage < _params[instance]._low_voltage) {
        return Failsafe::Low;
    }

    // Check capacity threshold
    if (_params[instance]._critical_capacity > 0 &&
        consumed_mah > 0 &&
        _params[instance]._pack_capacity - consumed_mah < _params[instance]._critical_capacity) {
        return Failsafe::Critical;
    }

    if (_params[instance]._low_capacity > 0 &&
        consumed_mah > 0 &&
        _params[instance]._pack_capacity - consumed_mah < _params[instance]._low_capacity) {
        return Failsafe::Low;
    }

    return Failsafe::None;
}
```

**Battery Voltage Discharge Curve (typical 3S LiPo)**:

```
Voltage | Capacity | Status
--------|----------|--------
12.6V   | 100%     | Full
12.0V   | 80%      | Good
11.4V   | 60%      | Normal
11.1V   | 50%      | Nominal
10.8V   | 40%      | Low
10.5V   | 20%      | WARNING (BATT_LOW_VOLT)
10.2V   | 10%      | CRITICAL (BATT_CRT_VOLT)
9.9V    | 5%       | Damage risk
9.0V    | 0%       | Cell damage
```

**Proposed Parameter Defaults (pico_trail)**:

```rust
// Battery failsafe parameters
BATT_LOW_VOLT: 0.0,        // Disabled by default (must configure)
BATT_CRT_VOLT: 0.0,        // Disabled by default (must configure)
BATT_FS_LOW_ACT: 0,        // None (warning only)
BATT_FS_CRT_ACT: 1,        // Hold

// Recommended values for 3S LiPo
// BATT_LOW_VOLT: 10.5
// BATT_CRT_VOLT: 10.0
// BATT_FS_LOW_ACT: 2  (RTL)
// BATT_FS_CRT_ACT: 1  (Hold)
```
