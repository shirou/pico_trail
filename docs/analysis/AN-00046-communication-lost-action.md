# AN-00046 Communication Lost Action | GCS Failsafe Implementation

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-00011-failsafe-system](AN-00011-failsafe-system.md) (parent failsafe framework)
  - [AN-00036-rtl-mode](AN-00036-rtl-mode.md) (RTL action dependency)
  - [AN-00035-battery-rtl](AN-00035-battery-rtl.md) (battery failsafe reference pattern)
- Related Requirements:
  - [FR-00041-gcs-loss-failsafe](../requirements/FR-00041-gcs-loss-failsafe.md)
  - [FR-00034-failsafe-action-priority](../requirements/FR-00034-failsafe-action-priority.md)
  - [FR-00038-failsafe-recovery](../requirements/FR-00038-failsafe-recovery.md)
  - [FR-00037-failsafe-parameters](../requirements/FR-00037-failsafe-parameters.md)
- Related ADRs: (To be created after approval)
- Related Tasks:
  - [T-00167-gcs-failsafe-implementation](../tasks/T-00167-gcs-failsafe-implementation/README.md)

## Executive Summary

This analysis explores the implementation of GCS communication lost failsafe, which triggers configurable safety actions when the ground control station heartbeat is not received within a timeout period. Currently, pico_trail has heartbeat timestamp tracking infrastructure (`ConnectionState` in core, `ArmedStateMonitor` in firmware) and failsafe parameters (`FS_ACTION`, `FS_TIMEOUT`, `FS_GCS_ENABLE`) registered in the parameter store, but no end-to-end action execution exists -- the detected GCS loss never triggers a mode change or notification.

Key findings: ArduPilot implements a two-stage timeout system where `FS_GCS_TIMEOUT` (default 5s) detects the loss condition, then `FS_TIMEOUT` (default 1.5s) requires the condition to persist before action executes. ArduPilot also counts `RC_CHANNELS_OVERRIDE` and `MANUAL_CONTROL` messages as "GCS seen" events, skips failsafe in RTL/Hold modes, and supports a "continue mission" option. For pico_trail, the recommended approach adopts ArduPilot's two-timeout system (`FS_GCS_TIMEOUT` + `FS_TIMEOUT`) and aligns `FS_ACTION` values with ArduPilot numbering (0=None, 1=RTL, 2=Hold). This requires updating the existing `FailsafeAction` enum and parameter defaults, which currently differ from ArduPilot. The GCS failsafe checker leverages existing battery failsafe patterns in `MavlinkLoopRunner`.

## Problem Space

### Current State

The project has partial infrastructure for GCS failsafe detection, but no action execution:

**Working components**:

- **Heartbeat tracking (core)**: `ConnectionState` in `crates/core/src/communication/dispatcher.rs:32-48` records `last_heartbeat_us` and provides `is_active()` method with configurable timeout
- **Heartbeat tracking (firmware)**: `ArmedStateMonitor` in `crates/firmware/src/core/arming/monitoring.rs:116` tracks `gcs_last_heartbeat_ms`
- **Failsafe parameters**: `FailsafeParams` in `crates/core/src/parameters/failsafe.rs` defines `FS_ACTION`, `FS_TIMEOUT`, `FS_GCS_ENABLE` in the parameter store
- **Failsafe action enum**: `FailsafeAction` (None=0, Hold=1, RTL=2, Disarm=3) in `crates/core/src/parameters/failsafe.rs:22-32` -- **NOTE: current enum numbering diverges from ArduPilot** (ArduPilot: 0=None, 1=RTL, 2=Hold); must be corrected during implementation
- **Parameter semantics divergence**: Current `FS_TIMEOUT` (default 5.0s) conflates ArduPilot's `FS_GCS_TIMEOUT` (detection) and `FS_TIMEOUT` (persistence) into a single parameter with different semantics; must be split to match ArduPilot
- **Battery failsafe (reference pattern)**: `BatteryFailsafeChecker` in `crates/core/src/autopilot/battery.rs` provides a complete end-to-end failsafe with hysteresis, action execution, and STATUSTEXT notification
- **Mode system**: 9 `FlightMode` variants including Hold and RTL, with `ModeExecutor` for lifecycle management
- **Outbound heartbeats**: `TelemetryStreamer::build_heartbeat()` sends vehicle heartbeat to GCS at 1 Hz

**Critical gaps**:

- **No GCS failsafe action executor**: `ArmedStateMonitor.update_slow()` returns `FailsafeReason::GcsLoss` but has `// TODO: Trigger failsafe action via channel` -- no mode change occurs
- **`ConnectionState.is_active()` never called**: The core-level timeout check exists but is not consumed by any failsafe logic
- **Parameters never consumed**: `FailsafeParams` is registered in the store but never read at runtime for GCS failsafe checking
- **No mode exemption logic**: No code skips failsafe when already in RTL or Hold modes
- **No failsafe recovery**: No logic clears the failsafe state when heartbeats resume
- **No STATUSTEXT notification**: No "Failsafe: GCS Lost" or "GCS Failsafe Cleared" messages sent
- **No `ModeReason` tracking**: Cannot distinguish operator-initiated vs. failsafe-initiated mode changes
- **No "never-seen" protection**: No guard against false triggers when GCS has never connected
- **Duplicate GCS loss detection**: Both core `ConnectionState` and firmware `ArmedStateMonitor` track heartbeat timeout independently; firmware detection should be removed in favor of core-based checker

### Desired State

Implement complete GCS communication lost failsafe that:

1. **Detects GCS loss**: No HEARTBEAT received within `FS_GCS_TIMEOUT` seconds (default 5.0s), then condition persists for `FS_TIMEOUT` seconds (default 1.5s) before action
2. **Executes configured action**: Hold, RTL, or Disarm based on `FS_ACTION` parameter (ArduPilot numbering: 0=None, 1=RTL, 2=Hold)
3. **Skips when safe**: Does not trigger in Hold or RTL modes (vehicle already safe)
4. **Notifies operator**: Sends STATUSTEXT "Failsafe: GCS Lost" on trigger, "GCS Failsafe Cleared" on recovery
5. **Recovers gracefully**: Clears failsafe state immediately when heartbeats resume (does not auto-switch mode)
6. **Guards against false positives**: Does not trigger if GCS was never connected
7. **Is configurable**: `FS_GCS_ENABLE` to enable/disable, `FS_GCS_TIMEOUT` for detection timeout, `FS_TIMEOUT` for persistence, `FS_ACTION` for action

Success criteria:

- Vehicle stops (Hold) or returns home (RTL) within `FS_GCS_TIMEOUT + FS_TIMEOUT + detection_latency` of last heartbeat (default \~6.7s)
- GCS failsafe only checks when vehicle is armed (no action when disarmed)
- No false triggers under normal telemetry conditions (WiFi, serial)
- Operator receives STATUSTEXT notifications for failsafe events (queued unconditionally, delivered on reconnect)
- Failsafe events logged locally via `log_warn!` / `log_info!` macros
- Failsafe does not interfere when vehicle is already in a safe mode
- GCS failsafe is enabled by default (`FS_GCS_ENABLE=1`)

### Gap Analysis

**Missing components**:

1. **GCS failsafe checker**: Periodic check of heartbeat age against timeout in vehicle control loop (caller passes `last_heartbeat_us` and `is_armed` as parameters, following `check_battery()` pattern)
2. **Armed-state guard**: Only check GCS failsafe when vehicle is armed (disarmed vehicles don't need failsafe protection)
3. **Action execution bridge**: Convert failsafe detection into `set_mode()` call on `SYSTEM_STATE`
4. **Mode exemption**: Skip failsafe action in RTL and Hold modes
5. **Never-seen guard**: Do not trigger if no heartbeat has ever been received
6. **Recovery logic**: Clear failsafe state and send notification when heartbeats resume
7. **STATUSTEXT integration**: Queue failsafe event notifications unconditionally (matching ArduPilot); log locally via `log_warn!` / `log_info!` macros as primary value during GCS disconnection
8. **Failsafe state tracking**: Track whether GCS failsafe is currently active (prevent re-triggering)

**Technical deltas**:

- **Fix `FailsafeAction` enum**: Change numbering to match ArduPilot (0=None, 1=RTL, 2=Hold, 3=SmartRTL, 4=SmartRTL_Hold, 5=Terminate)
- **Fix `FS_TIMEOUT` semantics**: Rename current `FS_TIMEOUT` (5.0s) to `FS_GCS_TIMEOUT`, add new `FS_TIMEOUT` (1.5s) for condition persistence
- **Remove firmware duplicate**: Remove GCS heartbeat detection from `ArmedStateMonitor` (firmware); core checker is authoritative
- Add `GcsFailsafeChecker` in `crates/core/src/autopilot/` (following `BatteryFailsafeChecker` pattern)
- Wire GCS failsafe check into `MavlinkLoopRunner` control loop (same location as battery failsafe)
- Add mode exemption check before executing failsafe action
- Add never-seen guard in ConnectionState or the checker itself
- Add failsafe STATUSTEXT messages via existing `StatusTextSender`
- Consume `FailsafeParams` at runtime (currently registered but unused)

## Stakeholder Analysis

| Stakeholder         | Interest/Need                                             | Impact | Priority |
| ------------------- | --------------------------------------------------------- | ------ | -------- |
| Operators           | Vehicle safety during autonomous missions beyond RC range | High   | P0       |
| Safety Reviewers    | Predictable behavior on communication loss                | High   | P0       |
| Test Engineers      | Reproducible failsafe behavior for validation             | High   | P0       |
| SITL Developers     | Testable in simulation without hardware                   | Medium | P1       |
| Autonomous Missions | Prerequisite for operating without continuous GCS link    | High   | P0       |

## Research & Discovery

### ArduPilot Rover GCS Failsafe Implementation

Source: `repo/ardupilot/Rover/Rover.cpp:347-367` and `repo/ardupilot/Rover/failsafe.cpp:47-118`

#### Detection Architecture

ArduPilot uses a two-layer detection system:

**Layer 1 -- Heartbeat reception** (`libraries/GCS_MAVLink/GCS_Common.cpp:4329`):

When a HEARTBEAT arrives from a recognized GCS system ID (`MAV_GCS_SYSID`, default 255), `sysid_mygcs_seen()` updates `_sysid_gcs_last_seen_time_ms`. Importantly, two other message types also reset the timer:

- `RC_CHANNELS_OVERRIDE` (line 4215)
- `MANUAL_CONTROL` (line 7440) -- explicitly documented as "a manual control message is considered to be a 'heartbeat' from the ground station for failsafe purposes"

**Layer 2 -- Periodic timeout check** (`Rover.cpp:347`, 10 Hz scheduler):

```cpp
void Rover::gcs_failsafe_check(void)
{
    if (g.fs_gcs_enabled == FS_GCS_DISABLED) return;

    const uint32_t gcs_last_seen_ms = gcs().sysid_mygcs_last_seen_time_ms();
    if (gcs_last_seen_ms == 0) return;  // never-seen guard

    const uint32_t last_gcs_update_ms = millis() - gcs_last_seen_ms;
    const uint32_t gcs_timeout_ms = uint32_t(constrain_float(
        g2.fs_gcs_timeout * 1000.0f, 0.0f, UINT32_MAX));

    const bool do_failsafe = last_gcs_update_ms >= gcs_timeout_ms;
    failsafe_trigger(FAILSAFE_EVENT_GCS, "GCS", do_failsafe);
}
```

Key design decisions:

- **Never-seen guard**: If `gcs_last_seen_ms == 0` (GCS never connected), failsafe never triggers -- prevents false positives during startup
- **10 Hz check rate**: Sufficient for 5-second timeout detection with <200ms latency

#### Two-Stage Timeout System

ArduPilot uses two sequential timeouts:

1. **`FS_GCS_TIMEOUT`** (default 5.0s, range 2-120s): Time since last GCS message before condition is detected
2. **`FS_TIMEOUT`** (default 1.5s, range 1-100s): Time the condition must persist before action executes

Total time from last heartbeat to action: `FS_GCS_TIMEOUT + FS_TIMEOUT` (default 6.5s).

The second timeout prevents transient failsafe activation on brief signal drops.

#### Mode Exemptions

```cpp
// failsafe.cpp:73-74
(control_mode != &mode_rtl) &&
(control_mode != &mode_hold ||
    (g2.fs_options & Failsafe_Options::Failsafe_Option_Active_In_Hold))
```

- **RTL mode**: Always exempt (vehicle already returning home)
- **Hold mode**: Exempt by default (vehicle already stopped); override with `FS_OPTIONS` bit 0
- **Auto mode + FS_GCS_ENABLE=2**: Continue mission instead of switching modes

#### Action Execution

Actions are shared with RC failsafe via `FS_ACTION` parameter:

| Value | Action           | Behavior                                 |
| ----- | ---------------- | ---------------------------------------- |
| 0     | None             | No mode change, warning only             |
| 1     | RTL              | Return to launch                         |
| 2     | Hold             | Stop and hold position (default)         |
| 3     | SmartRTL or RTL  | Try SmartRTL, fallback to RTL, then Hold |
| 4     | SmartRTL or Hold | Try SmartRTL, fallback to Hold           |
| 5     | Terminate        | Immediate disarm                         |

SmartRTL uses cascading fallbacks when `set_mode()` fails.

#### Recovery

When GCS heartbeats resume, `gcs_failsafe_check()` calls `failsafe_trigger(FAILSAFE_EVENT_GCS, "GCS", false)`:

- Failsafe bit is cleared from `failsafe.bits`
- If all failsafe bits are zero, "GCS Failsafe Cleared" notification sent
- **Vehicle does NOT auto-return to previous mode** -- operator must manually switch

#### Additional Behaviors

- **RC override clearing**: `RC_Channels::clear_overrides()` called when failsafe triggers
- **Notification flags**: `AP_Notify::flags.failsafe_gcs` set for LED/buzzer
- **Bitmask tracking**: GCS and RC failsafes share `failsafe.bits` bitmask, both must clear for full recovery

### Current pico_trail Patterns (Battery Failsafe Reference)

The battery failsafe provides the implementation pattern to follow:

**`crates/core/src/autopilot/battery.rs`** -- `BatteryFailsafeChecker`:

- Hysteresis: 100 consecutive samples below threshold before triggering LOW
- Immediate trigger for CRITICAL (no delay)
- Sticky flags: once triggered, stays triggered until voltage recovers AND count resets
- Returns `BatteryAction` enum (None, SetMode(Hold), SetMode(Rtl), Disarm)

**`crates/core/src/autopilot/mavlink_runner.rs`** -- `check_battery()`:

- Called in the main control loop
- Reads `BatteryParams` from parameter store
- Calls `BatteryFailsafeChecker::check()`
- Applies action via `SYSTEM_STATE.lock().set_mode()`

This pattern (checker + runner integration + parameter consumption) is directly applicable to GCS failsafe.

### Data Analysis

**GCS Communication Patterns**:

- **WiFi (pico_trail primary link)**: Higher packet loss than wired, 1-3% loss typical, brief dropouts (100-500ms) common
- **Mission Planner heartbeat rate**: Typically 1 Hz HEARTBEAT + control messages
- **QGroundControl heartbeat rate**: 1 Hz HEARTBEAT
- **Typical dropout duration**: WiFi reconnect takes 1-5 seconds, router failover 5-15 seconds

**Timeout Sensitivity**:

| Timeout | False Trigger Risk | Detection Speed | Use Case                    |
| ------- | ------------------ | --------------- | --------------------------- |
| 2s      | High (WiFi drops)  | Fast            | Not recommended for WiFi    |
| 5s      | Low                | Moderate        | Good for WiFi links         |
| 10s     | Very Low           | Slow            | Conservative, long missions |
| 15-30s  | Negligible         | Very Slow       | Extended autonomous ops     |

**Recommendation**: 5.0s default matches ArduPilot and provides good balance for WiFi links.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The system shall detect GCS communication loss when no HEARTBEAT message is received within `FS_GCS_TIMEOUT` seconds, and execute the configured failsafe action after the condition persists for `FS_TIMEOUT` seconds
  - Rationale: Prevent uncontrolled vehicle operation when GCS link fails; two-stage timeout matches ArduPilot and prevents transient false triggers
  - Acceptance Criteria:
    - Track timestamp of last received HEARTBEAT from GCS
    - Only check GCS failsafe when vehicle is armed (skip when disarmed)
    - Detect GCS loss when heartbeat age exceeds `FS_GCS_TIMEOUT` (default 5.0s)
    - Execute action after condition persists for `FS_TIMEOUT` (default 1.5s)
    - Total time from last heartbeat to action: `FS_GCS_TIMEOUT + FS_TIMEOUT` (default 6.5s)
    - Execute action from `FS_ACTION` parameter (ArduPilot numbering: 0=None, 1=RTL, 2=Hold)
    - Check at 10 Hz (100ms interval) in vehicle control loop

- [ ] **FR-DRAFT-2**: The system shall not trigger GCS failsafe when the vehicle is in Hold or RTL mode
  - Rationale: Vehicle is already in a safe state; mode change would be redundant or counterproductive
  - Acceptance Criteria:
    - Check current `FlightMode` before executing failsafe action
    - Skip action if mode is `Hold` or `Rtl`
    - Still track failsafe state (for notification/logging) even if action is skipped
    - SmartRtl mode also considered safe (vehicle returning)

- [ ] **FR-DRAFT-3**: The system shall not trigger GCS failsafe if no GCS heartbeat has ever been received
  - Rationale: Prevents false triggers during startup or when operating without GCS
  - Acceptance Criteria:
    - Track whether at least one heartbeat has been received
    - Skip failsafe check entirely if no heartbeat ever received
    - Begin monitoring only after first heartbeat is received

- [ ] **FR-DRAFT-4**: The system shall clear GCS failsafe state and send notification when heartbeats resume, without automatically changing mode
  - Rationale: Allow operator to assess situation and manually choose mode after communication restores
  - Acceptance Criteria:
    - Clear failsafe-active flag immediately when heartbeat received (no hysteresis, matching ArduPilot behavior)
    - Send STATUSTEXT "GCS Failsafe Cleared" with severity INFO
    - Do NOT auto-switch to previous mode (operator must manually switch)

- [ ] **FR-DRAFT-5**: The system shall send STATUSTEXT notifications for GCS failsafe activation and recovery
  - Rationale: Operator awareness of failsafe events for real-time and post-flight analysis; ArduPilot queues STATUSTEXT unconditionally regardless of GCS connection state
  - Acceptance Criteria:
    - Queue "Failsafe: GCS Lost" with severity WARNING on activation via `status_notifier` (existing pattern)
    - Queue "GCS Failsafe Cleared" with severity INFO on recovery
    - Include failsafe action in activation message (e.g., "Failsafe: GCS Lost - Hold")
    - Log failsafe events locally via `log_warn!` / `log_info!` macros (primary value during GCS disconnection)
    - Queued STATUSTEXT messages are delivered when GCS reconnects (pruned after 5s if queue full, matching ArduPilot behavior)

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: GCS failsafe action execution latency shall not exceed `FS_GCS_TIMEOUT + FS_TIMEOUT` + 200ms from last heartbeat
  - Category: Performance
  - Rationale: 10 Hz check rate provides maximum 100ms detection latency after each timeout expires
  - Target: Total time from last heartbeat to action < `FS_GCS_TIMEOUT + FS_TIMEOUT` + 200ms (default < 6.7s)

- [ ] **NFR-DRAFT-2**: GCS failsafe shall not produce false triggers under normal WiFi operation with up to 3% packet loss
  - Category: Reliability
  - Rationale: WiFi links commonly experience brief dropouts; 5s timeout accommodates this
  - Target: Zero false triggers when GCS sends heartbeats at 1 Hz with up to 3% packet loss

## Design Considerations

### Technical Constraints

- **Existing pattern**: Must follow `BatteryFailsafeChecker` pattern for consistency (checker struct + runner integration)
- **Parameter alignment**: Adopt ArduPilot's two-timeout system (`FS_GCS_TIMEOUT` + `FS_TIMEOUT`) and action numbering (0=None, 1=RTL, 2=Hold). Existing `FailsafeAction` enum and `FS_TIMEOUT` default must be corrected
- **Core crate**: GCS failsafe checker must live in `crates/core/` (not firmware) for SITL testability; firmware `ArmedStateMonitor` GCS detection to be removed
- **Embassy feature gates**: Heartbeat timestamp updates require embassy feature for async Mutex access
- **No dynamic allocation**: Failsafe state must use static/stack allocation only

### Potential Approaches

1. **Option A: Extend ArmedStateMonitor (firmware only)**
   - Pros:
     - Minimal changes: add action dispatch to existing `update_slow()` TODO
     - Already detects GCS loss with hardcoded 5s timeout
   - Cons:
     - Firmware-only: cannot test in SITL
     - Hardcoded timeout: does not use parameter store
     - No mode exemption or recovery logic
     - Breaks architecture: failsafe logic should be in core, not firmware
   - Effort: Low (2-4 hours)

2. **Option B: New GcsFailsafeChecker in core (following battery pattern)** -- Recommended
   - Pros:
     - Follows established `BatteryFailsafeChecker` pattern
     - Lives in core crate: testable in SITL and host tests
     - Extends existing `FailsafeParams` from parameter store (add `FS_GCS_TIMEOUT`, fix `FS_ACTION` numbering)
     - Integrates into `MavlinkLoopRunner` alongside battery failsafe
     - Clean separation: checker detects, runner executes
     - Mode exemption and recovery logic included
     - Removes duplicate firmware GCS detection (single source of truth in core)
   - Cons:
     - Requires wiring heartbeat timestamps from `ConnectionState` to the checker
     - Moderate implementation effort
   - Effort: Medium (8-12 hours)

3. **Option C: Unified failsafe framework replacing all individual checkers**
   - Pros:
     - Single framework for RC, GCS, and battery failsafes
     - Priority-based action resolution
     - Clean extension point for future failsafe types
   - Cons:
     - Over-engineering for current needs (only battery + GCS)
     - Would require refactoring working battery failsafe
     - Higher risk of introducing regressions
   - Effort: High (24-40 hours)

**Recommendation**: Option B provides the best balance. It follows the proven battery failsafe pattern, keeps logic in core for testability, and avoids over-engineering. The unified framework (Option C) can be introduced later if/when a third failsafe type is needed.

### Implementation Strategy

**Phase 1: Parameter Alignment + GCS Failsafe Checker**

- Fix `FailsafeAction` enum to match ArduPilot numbering (0=None, 1=RTL, 2=Hold)
- Rename existing `FS_TIMEOUT` (5.0s) to `FS_GCS_TIMEOUT`, add new `FS_TIMEOUT` (1.5s) for condition persistence
- Create `GcsFailsafeChecker` struct in `crates/core/src/autopilot/gcs_failsafe.rs`
- Track: `failsafe_active`, `gcs_ever_seen`, `failsafe_condition_start_us` (no internal timestamp storage -- caller passes heartbeat data as parameters)
- Two-stage check: detect GCS loss via `FS_GCS_TIMEOUT`, require persistence via `FS_TIMEOUT`
- Checker method signature follows `check_battery()` pattern: `check_gcs(last_heartbeat_us, current_time_us, is_armed, param_store)` -- caller reads `last_heartbeat_us` from `dispatcher.connection().last_heartbeat_us`
- Only check when armed (`is_armed` parameter); skip when disarmed
- Wire into `MavlinkLoopRunner` control loop (alongside battery check)
- Execute action via `SYSTEM_STATE.lock().set_mode()`
- Remove GCS heartbeat tracking from firmware `ArmedStateMonitor`

**Phase 2: Mode Exemptions and Recovery**

- Check current mode before executing action (skip Hold, RTL, SmartRTL)
- Clear failsafe state immediately when heartbeat received (no hysteresis, matching ArduPilot)

**Phase 3: Notifications and Testing**

- Send STATUSTEXT on failsafe activation and recovery
- Unit tests for checker logic (timeout, mode exemption, recovery, never-seen guard)
- SITL integration test: simulate heartbeat loss and verify mode change

### Architecture Impact

**New modules**:

- `crates/core/src/autopilot/gcs_failsafe.rs` -- `GcsFailsafeChecker` (following battery.rs pattern)

**Modified modules**:

- `crates/core/src/parameters/failsafe.rs` -- Fix `FailsafeAction` enum numbering, rename `FS_TIMEOUT` to `FS_GCS_TIMEOUT`, add new `FS_TIMEOUT`
- `crates/core/src/autopilot/mavlink_runner.rs` -- Add `check_gcs_failsafe()` method, call in control loop
- `crates/core/src/communication/dispatcher.rs` -- `connection()` getter already exposes `last_heartbeat_us` and `heartbeat_count`; caller reads these and passes as parameters to checker
- `crates/sitl/src/autopilot.rs` -- Wire GCS failsafe into SITL control loop
- `crates/firmware/src/core/arming/monitoring.rs` -- Remove GCS heartbeat tracking and `FailsafeReason::GcsLoss` detection (moved to core)

**Memory Impact**:

| Component            | RAM Usage  | Notes                                                           |
| -------------------- | ---------- | --------------------------------------------------------------- |
| GcsFailsafeChecker   | \~16 B     | Flags + condition_start_us only (no internal timestamp storage) |
| FailsafeParams read  | 0 B        | Already allocated (just unused)                                 |
| **Total additional** | **\~16 B** | Minimal overhead                                                |

## ArduPilot Parameters

This analysis uses the following standard ArduPilot Rover parameters:

### GCS Failsafe Parameters

- **FS_GCS_ENABLE** (u8, default 0, range 0-2)
  - Enable GCS heartbeat failsafe
  - 0 = Disabled, 1 = Enabled, 2 = Enabled Continue with Mission in Auto
  - ArduPilot default: 0 (disabled, requires explicit opt-in)

- **FS_TIMEOUT** (float, default 1.5s, range 1-100s)
  - Failsafe timeout -- how long the failsafe condition must persist before action
  - Shared between RC and GCS failsafes in ArduPilot
  - pico_trail adopts same semantics and default

- **FS_ACTION** (u8, default 2, range 0-6)
  - Failsafe action to execute
  - 0 = None, 1 = RTL, 2 = Hold, 3 = SmartRTL or RTL, 4 = SmartRTL or Hold, 5 = Terminate
  - ArduPilot default: 2 (Hold)
  - pico_trail adopts same numbering (current code has Hold=1, RTL=2 -- **must be corrected**)

- **FS_GCS_TIMEOUT** (float, default 5.0s, range 2-120s)
  - GCS-specific detection timeout: time since last GCS message before condition is detected
  - pico_trail adopts this parameter (current code misuses `FS_TIMEOUT` for this role -- **must be renamed**)

- **FS_OPTIONS** (u32, default 0, bitmask)
  - Bit 0: Failsafe active in Hold mode
  - Not in initial implementation scope

### pico_trail Parameter Mapping

| ArduPilot Parameter | pico_trail Parameter | Default  | Current Code Status                                      |
| ------------------- | -------------------- | -------- | -------------------------------------------------------- |
| FS_GCS_ENABLE       | FS_GCS_ENABLE        | 1        | Already registered, correct                              |
| FS_GCS_TIMEOUT      | FS_GCS_TIMEOUT       | 5.0      | **NEW** -- current `FS_TIMEOUT` must be renamed to this  |
| FS_TIMEOUT          | FS_TIMEOUT           | 1.5      | **FIX** -- change default from 5.0 to 1.5, fix semantics |
| FS_ACTION           | FS_ACTION            | 2 (Hold) | **FIX** -- change enum: Hold=1→2, RTL=2→1                |

**Code changes required** in `crates/core/src/parameters/failsafe.rs`:

1. Rename current `FS_TIMEOUT` (5.0s) registration to `FS_GCS_TIMEOUT`
2. Add new `FS_TIMEOUT` registration with default 1.5s
3. Change `FailsafeAction` enum: `RTL = 1, Hold = 2` (swap current values)
4. Update `FailsafeParams` struct to include `gcs_timeout` field
5. Update all tests referencing old defaults

## Risk Assessment

| Risk                                                | Probability | Impact       | Mitigation Strategy                                                                   |
| --------------------------------------------------- | ----------- | ------------ | ------------------------------------------------------------------------------------- |
| **False trigger from WiFi dropout**                 | Medium      | Medium       | Two-stage timeout (5s + 1.5s = 6.5s) accommodates typical WiFi dropouts               |
| **Missed trigger (vehicle continues uncontrolled)** | Low         | **Critical** | 10 Hz check rate ensures <200ms detection latency; enabled by default                 |
| GCS never connected causes immediate trigger        | Medium      | Medium       | Never-seen guard: skip failsafe if no heartbeat ever received                         |
| Failsafe during active mission causes confusion     | Medium      | Low          | Mode exemption for RTL/Hold; STATUSTEXT notification explains what happened           |
| Interference with battery failsafe                  | Low         | Medium       | Both failsafes target Hold mode by default; battery critical overrides via priority   |
| FS_ACTION numbering change breaks existing configs  | Low         | Medium       | Document migration in release notes; default Hold still works (just different number) |
| Failsafe in SITL without real heartbeats            | Medium      | Low          | SITL generates synthetic heartbeats; FS_GCS_ENABLE configurable                       |

## Open Questions

- [x] Should pico_trail use ArduPilot's two-timeout system (FS_GCS_TIMEOUT + FS_TIMEOUT)? --> Decision: Yes, adopt ArduPilot's two-stage system. `FS_GCS_TIMEOUT` (5.0s) for detection, `FS_TIMEOUT` (1.5s) for condition persistence. This prevents transient false triggers and matches ArduPilot semantics exactly. Current code's `FS_TIMEOUT=5.0` must be renamed to `FS_GCS_TIMEOUT`.
- [x] Should GCS failsafe be enabled or disabled by default? --> Decision: Enabled by default (`FS_GCS_ENABLE=1`). This differs from ArduPilot (disabled) but pico_trail operates primarily over WiFi where GCS failsafe is important.
- [x] Should `RC_CHANNELS_OVERRIDE` and `MANUAL_CONTROL` also reset the heartbeat timer? --> Decision: Not in Phase 1. ArduPilot does this but pico_trail can add it later if needed.
- [x] Should the system support "continue mission" mode (ArduPilot's FS_GCS_ENABLE=2)? --> Decision: Not in Phase 1. Add as future enhancement when Auto mode is mature.
- [ ] Should GCS failsafe interact with RC failsafe (e.g., shared bitmask like ArduPilot)? --> Method: Defer to unified failsafe framework analysis if/when RC failsafe is implemented
- [ ] Should mode change reason tracking (`ModeReason`) be added as part of this work? --> Recommendation: Yes, minimal `ModeReason` enum (Operator, Failsafe) aids debugging and is low effort

## Recommendations

### Immediate Actions

1. **Adopt Option B**: New `GcsFailsafeChecker` in core crate following `BatteryFailsafeChecker` pattern
2. **Align parameters with ArduPilot**: Fix `FailsafeAction` enum numbering (0=None, 1=RTL, 2=Hold), rename `FS_TIMEOUT` to `FS_GCS_TIMEOUT`, add new `FS_TIMEOUT` (1.5s)
3. **Follow battery failsafe pattern**: Checker struct in `crates/core/src/autopilot/`, integration in `MavlinkLoopRunner`
4. **Remove firmware duplicate**: Delete GCS heartbeat tracking from `ArmedStateMonitor`; core checker is authoritative
5. **Implement mode exemptions**: Skip action in Hold, RTL, SmartRTL modes
6. **Add never-seen guard**: Do not trigger if GCS has never connected

### Next Steps

1. [ ] Review and approve this analysis
2. [x] Update FR-00041 acceptance criteria based on ArduPilot findings (two-stage timeout, never-seen guard, ArduPilot parameter alignment)
3. [x] Create additional requirements if needed (mode exemption, never-seen guard, recovery) -- embedded in FR-00041; FR-00037/FR-00038/NFR-00029/NFR-00044 updated
4. [x] Create ADR for GCS failsafe implementation approach -- N/A, follows existing BatteryFailsafeChecker pattern, no new architecture decision needed
5. [x] Create task package with phased implementation plan -- [T-00167-gcs-failsafe-implementation](../tasks/T-00167-gcs-failsafe-implementation/README.md)

### Out of Scope

The following features are explicitly excluded from initial implementation:

- **RC failsafe**: Separate feature (RC loss detection and action) -- requires RC timeout infrastructure
- **Continue-mission mode**: ArduPilot's `FS_GCS_ENABLE=2` for Auto mode -- defer until Auto mode is mature
- **FS_OPTIONS bitmask**: Advanced options (failsafe active in Hold mode) -- not needed initially
- **Unified failsafe framework**: Shared priority system for RC + GCS + battery -- defer until third failsafe type needed
- **Terminate action**: Motor cutoff on GCS loss -- safety concern, requires hardware kill switch
- **Multiple GCS support**: Single GCS system ID tracking only
- **Heartbeat from non-HEARTBEAT messages**: ArduPilot counts RC_CHANNELS_OVERRIDE and MANUAL_CONTROL as GCS seen -- simplify to HEARTBEAT only

## Appendix

### References

- ArduPilot GCS Failsafe Check: `repo/ardupilot/Rover/Rover.cpp:347-367`
- ArduPilot Failsafe Trigger: `repo/ardupilot/Rover/failsafe.cpp:47-118`
- ArduPilot Heartbeat Handler: `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:4329`
- ArduPilot GCS Parameters: `repo/ardupilot/Rover/Parameters.cpp:123-128, 624-631`
- ArduPilot Failsafe Documentation: <https://ardupilot.org/rover/docs/rover-failsafes.html>
- ArduPilot Parameters: <https://ardupilot.org/rover/docs/parameters.html>
- pico_trail Failsafe System Analysis: [AN-00011-failsafe-system](AN-00011-failsafe-system.md)
- pico_trail Battery Failsafe: `crates/core/src/autopilot/battery.rs`
- pico_trail Connection State: `crates/core/src/communication/dispatcher.rs:32-48`
- pico_trail Failsafe Parameters: `crates/core/src/parameters/failsafe.rs`

### ArduPilot GCS Failsafe Flow

```
GCS sends HEARTBEAT (sysid matches MAV_GCS_SYSID)
    --> handle_heartbeat() updates _sysid_gcs_last_seen_time_ms

Every 100ms (10 Hz scheduler):
    gcs_failsafe_check():
        1. If FS_GCS_ENABLE == 0 --> return (disabled)
        2. If GCS never seen --> return (no false positives)
        3. If (now - last_seen) >= FS_GCS_TIMEOUT --> trigger = true
        4. failsafe_trigger(FAILSAFE_EVENT_GCS, "GCS", trigger)
            a. Sets/clears failsafe.bits
            b. If bits just went non-zero, records start_time
            c. If triggered was set and bits now 0, announces "Cleared"
            d. If not yet triggered AND bits != 0 AND persisted > FS_TIMEOUT:
                - Skip if in RTL mode
                - Skip if in Hold mode (unless FS_OPTIONS bit 0 set)
                - Execute FS_ACTION (None/RTL/Hold/SmartRTL/Terminate)
                - Clear RC overrides
```

### Proposed pico_trail GCS Failsafe Flow

```
HEARTBEAT received in dispatcher.dispatch()
    --> connection.update_heartbeat(timestamp_us)
        (updates last_heartbeat_us and heartbeat_count in ConnectionState)
        (NO direct call to failsafe checker -- checker reads data via parameters)

Every 100ms (in MavlinkLoopRunner control loop, called by SITL autopilot or firmware task):
    Caller reads from dispatcher:
        last_heartbeat_us = dispatcher.connection().last_heartbeat_us
        heartbeat_count = dispatcher.connection().heartbeat_count
        is_armed = SYSTEM_STATE.is_armed()

    check_gcs_failsafe(last_heartbeat_us, heartbeat_count, current_time_us, is_armed, param_store):
        1. If !is_armed --> return None (skip when disarmed)
        2. If !params.fs_gcs_enable --> return None
        3. If heartbeat_count == 0 --> return None (never-seen guard)
        4. Set checker.gcs_ever_seen = true (if not already)
        5. heartbeat_age_us = current_time_us - last_heartbeat_us
        6. If heartbeat_age_us >= params.fs_gcs_timeout * 1_000_000:
            a. If !checker.condition_detected:
                - Set condition_detected = true
                - Record condition_start_us = current_time_us
            b. If condition_detected AND (current_time_us - condition_start_us) >= params.fs_timeout * 1_000_000:
                - If !checker.failsafe_active:
                    - Set failsafe_active = true
                    - Check current mode: skip action if Hold/RTL/SmartRTL
                    - Queue STATUSTEXT: "Failsafe: GCS Lost - {action}"
                    - log_warn!("GCS failsafe triggered: {action}")
                    - Return Some(action) from params.fs_action
        7. Else (heartbeat_age within threshold):
            a. Set condition_detected = false
            b. If checker.failsafe_active:
                - Set failsafe_active = false (immediate clear, no hysteresis)
                - Queue STATUSTEXT: "GCS Failsafe Cleared"
                - log_info!("GCS failsafe cleared")
        8. Return None
```

Note: This follows the `check_battery(voltage, is_armed, param_store)` pattern where the caller reads sensor data from its source and passes it as parameters to the checker. The checker has no direct reference to `ConnectionState` or `MessageDispatcher`.
