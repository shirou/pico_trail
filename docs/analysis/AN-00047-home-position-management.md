# AN-00047 Home Position Management | Tracking, Reporting, and GCS Home Override

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-00036-rtl-mode](AN-00036-rtl-mode.md) (RTL depends on home position)
  - [AN-00035-battery-rtl](AN-00035-battery-rtl.md) (battery RTL uses home position)
  - [AN-00022-gps-position-telemetry-and-autonomous-navigation-foundation](AN-00022-gps-position-telemetry-and-autonomous-navigation-foundation.md)
  - [AN-00046-communication-lost-action](AN-00046-communication-lost-action.md) (GCS failsafe RTL uses home)
- Related Requirements:
  - [FR-00171-home-auto-set-gps-fix](../requirements/FR-00171-home-auto-set-gps-fix.md)
  - [FR-00172-home-update-disarmed](../requirements/FR-00172-home-update-disarmed.md)
  - [FR-00173-home-position-broadcast](../requirements/FR-00173-home-position-broadcast.md)
  - [FR-00174-request-message-home-position](../requirements/FR-00174-request-message-home-position.md)
  - [FR-00175-home-prearm-check](../requirements/FR-00175-home-prearm-check.md)
  - [FR-00176-home-lock-mechanism](../requirements/FR-00176-home-lock-mechanism.md)
  - [FR-00177-global-position-int-relative-alt](../requirements/FR-00177-global-position-int-relative-alt.md)
  - [FR-00178-get-home-position-command](../requirements/FR-00178-get-home-position-command.md)
  - [NFR-00179-home-auto-set-latency](../requirements/NFR-00179-home-auto-set-latency.md)
  - [NFR-00180-home-broadcast-bandwidth](../requirements/NFR-00180-home-broadcast-bandwidth.md)
- Related ADRs: N/A -- follows existing MavlinkLoopRunner pattern
- Related Tasks:
  - [T-00181-home-position-management](../tasks/T-00181-home-position-management/README.md)

## Executive Summary

This analysis explores the implementation of complete home position management for the pico_trail rover. Home position is a fundamental concept required by RTL, distance-to-home telemetry, and GCS interaction. Currently, the `HomePosition` struct and `MAV_CMD_DO_SET_HOME` (via COMMAND_INT) are implemented, but critical gaps exist: home is never auto-set, HOME_POSITION is not broadcast on change or requestable, and Mission Planner reports "failed to set home" when attempting Guided mode because the vehicle has no home position established.

Key findings from ArduPilot source code (`repo/ardupilot`): ArduPilot auto-sets home on first valid EKF/GPS position in `ahrs_update()` (400 Hz), then refines it at 1 Hz while disarmed via `update_home()`. Home is a **pre-arm prerequisite** -- arming fails with "AHRS: waiting for home" if home is not set. HOME_POSITION is broadcast **on-change only** (inside `AP_AHRS::set_home()`), not periodically -- it is not in any stream group. GCS-set home is "locked" (not auto-updated), while auto-set home is "unlocked" (refined while disarmed).

The recommended approach for pico_trail: auto-set home on first GPS 3D fix, update home while disarmed, broadcast HOME_POSITION on change, add `MAV_CMD_REQUEST_MESSAGE` support for HOME_POSITION (message ID 242), and make home a pre-arm check prerequisite.

## Problem Space

### Current State

The project has partial home position infrastructure:

**Working components**:

- **HomePosition struct**: `crates/core/src/autopilot/state.rs:178` with lat/lon/alt fields
- **HomePosition::from_gps()**: Converts GPS position to HomePosition
- **HomePosition::from_command_int()**: Parses COMMAND_INT x/y/z (degE7) to HomePosition
- **SystemState.home_position**: `Option<HomePosition>` field, with `set_home()`, `set_home_to_current()`, `has_home()` methods
- **MAV_CMD_DO_SET_HOME handler**: `crates/core/src/communication/handlers/command.rs:572` handles COMMAND_INT, supports both "use current" (param1>0.5) and "specified location" (param1<=0.5) modes
- **build_home_position_message()**: Constructs `HOME_POSITION_DATA` with degE7 lat/lon and mm altitude
- **HOME_POSITION response**: Sent back as part of MAV_CMD_DO_SET_HOME response
- **RTL mode home_provider**: `RtlMode` and `SmartRtlMode` accept `fn() -> Option<(f32, f32)>` to retrieve home

**Critical gaps**:

- **No auto-set home on GPS fix**: ArduPilot auto-sets home on first valid EKF/GPS position in `ahrs_update()` (`repo/ardupilot/Rover/Rover.cpp:297-314`). pico_trail never auto-sets home -- it is `None` unless GCS explicitly sends `MAV_CMD_DO_SET_HOME`
- **No home update while disarmed**: ArduPilot calls `update_home()` at 1 Hz while disarmed (`repo/ardupilot/Rover/commands.cpp:44-65`), refining home position as GPS accuracy improves or vehicle is relocated. pico_trail does not update home after initial set
- **No pre-arm check for home**: ArduPilot requires home to be set before arming (`AP_Arming::gps_checks()` at `repo/ardupilot/libraries/AP_Arming/AP_Arming.cpp:694-738` -- "AHRS: waiting for home"). pico_trail allows arming without home
- **HOME_POSITION not broadcast on change**: ArduPilot broadcasts HOME_POSITION inside `AP_AHRS::set_home()` (`repo/ardupilot/libraries/AP_AHRS/AP_AHRS.cpp:3035-3085`) via `GCS_SEND_MESSAGE(MSG_HOME)`. pico_trail only sends HOME_POSITION as a DO_SET_HOME command response
- **MAV_CMD_REQUEST_MESSAGE ignores HOME_POSITION**: `handle_request_message()` handles message IDs 148 (AUTOPILOT_VERSION) and 300 (PROTOCOL_VERSION) but not 242 (HOME_POSITION). ArduPilot maps message ID 242 to `MSG_HOME` in `mavlink_id_to_ap_message_id()` (`repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:1046`)
- **Mission Planner "failed to set home"**: Since pico_trail never broadcasts HOME_POSITION, Mission Planner cannot resolve home and reports an error when entering Guided mode
- **No home lock mechanism**: ArduPilot distinguishes "locked" (GCS-set, not auto-updated) vs "unlocked" (auto-set, refined while disarmed) home. pico_trail has no such distinction
- **GLOBAL_POSITION_INT.relative_alt hardcoded to 0**: `crates/core/src/communication/handlers/telemetry.rs:308` sets `relative_alt: 0` instead of computing altitude relative to home. ArduPilot computes this as current altitude minus home altitude (`repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:6105-6113`). Mission Planner uses this field for altitude display
- **MAV_CMD_DO_SET_HOME not handled via COMMAND_LONG**: Only COMMAND_INT is supported (`command.rs:552`). ArduPilot converts COMMAND_LONG to COMMAND_INT for location-bearing commands (`repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:5285-5346`). Some GCS may send COMMAND_LONG
- **MAV_CMD_GET_HOME_POSITION not supported**: ArduPilot handles this deprecated command (`repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:5092-5107`) and responds with both HOME_POSITION and GPS_GLOBAL_ORIGIN. Mission Planner may send this on connect to retrieve home position

### Desired State

Implement complete home position management that:

1. **Auto-sets home on first GPS 3D fix**: Set home when valid GPS position first becomes available, matching ArduPilot's `ahrs_update()` pattern
2. **Updates home while disarmed**: Periodically refine home position while disarmed (1 Hz), if vehicle has moved significantly (>0.5m, matching ArduPilot's `DISTANCE_HOME_MINCHANGE`)
3. **Requires home for arming**: Add pre-arm check that home is set (GPS check category)
4. **Broadcasts HOME_POSITION on change**: Send HOME_POSITION on active MAVLink channel whenever home is set or updated -- on-change only, not periodic (matching ArduPilot)
5. **Supports MAV_CMD_REQUEST_MESSAGE for HOME_POSITION**: Allow GCS to request HOME_POSITION (message ID 242) on demand
6. **Supports home lock**: GCS-set home is locked (not auto-updated); auto-set home is unlocked (updated while disarmed)

Success criteria:

- Home is automatically set when first GPS 3D fix is acquired
- Home is refined while disarmed as GPS position improves
- Arming fails with clear message if home is not set
- GCS displays home position and distance-to-home correctly
- Mission Planner can enter Guided mode without "failed to set home" error
- Home position is available for RTL, battery failsafe RTL, and GCS failsafe RTL
- GCS can override home via `MAV_CMD_DO_SET_HOME`; overridden home is locked

### Gap Analysis

**Missing components**:

1. **Auto-set home on GPS fix**: In vehicle control loop, check if home is unset and GPS fix is valid, then set home
2. **Home update while disarmed**: Periodic check (1 Hz) while disarmed to refine home if vehicle moved >0.5m
3. **Pre-arm check for home**: Add "waiting for home" check in GPS arming checks
4. **HOME_POSITION broadcast on change**: Send HOME_POSITION whenever `set_home()` or `set_home_to_current()` is called
5. **MAV_CMD_REQUEST_MESSAGE support for 242**: Handle message ID 242 to return HOME_POSITION
6. **Home lock flag**: Boolean flag to track whether home was GCS-set (locked)
7. **GLOBAL_POSITION_INT `relative_alt`**: Compute altitude relative to home instead of hardcoded 0
8. **MAV_CMD_GET_HOME_POSITION**: Support deprecated command for backward compatibility with Mission Planner

**Technical deltas**:

- Add `home_locked: bool` field to `SystemState`
- Add home auto-set logic in vehicle control loop (MavlinkLoopRunner or equivalent)
- Add home update logic while disarmed (1 Hz)
- Add "waiting for home" pre-arm check
- Wire HOME_POSITION broadcast from `set_home()` / `set_home_to_current()` to MAVLink sender
- Add message ID 242 to `handle_request_message()` or `handle_command_long`
- Fix `build_global_position_int()` to compute `relative_alt` from home altitude
- Add `MAV_CMD_GET_HOME_POSITION` handler in COMMAND_INT (or COMMAND_LONG)

## Stakeholder Analysis

| Stakeholder      | Interest/Need                                          | Impact | Priority |
| ---------------- | ------------------------------------------------------ | ------ | -------- |
| Operators (GCS)  | See home position, distance-to-home on Mission Planner | High   | P0       |
| RTL Mode         | Needs home position set for return navigation          | High   | P0       |
| Failsafe System  | RTL/Battery-RTL actions require home position          | High   | P0       |
| Mission Planning | Home position as reference point for missions          | Medium | P1       |
| Guided Mode      | Mission Planner needs home to enter Guided mode        | High   | P0       |

## Research & Discovery

### ArduPilot Home Position Behavior (Source Code Analysis)

Source: `repo/ardupilot` local repository

#### Auto-Set on First GPS Fix (Not on Arming)

ArduPilot does **not** set home during the arming sequence itself. Home is set **before** arming, as a prerequisite:

**Mechanism 1 -- `ahrs_update()` at 400 Hz** (`repo/ardupilot/Rover/Rover.cpp:297-314`):

```cpp
void Rover::ahrs_update()
{
    // ...
    if (!ahrs.home_is_set()) {
        if (!set_home_to_current_location(false)) {
            // ignore this failure
        }
    }
}
```

Called at 400 Hz. On every iteration, if home is not yet set and EKF has a valid position, it attempts to set home (unlocked). This is the primary mechanism -- home is set as soon as GPS/EKF provides a valid position, well before arming.

**Mechanism 2 -- `update_home()` at 1 Hz while disarmed** (`repo/ardupilot/Rover/commands.cpp:44-65`):

```cpp
void Rover::update_home()
{
    if (ahrs.home_is_locked()) { return; }  // GCS-set home is preserved
    Location loc{};
    if (!ahrs.get_location(loc)) { return; }
    barometer.update_calibration();
    if (ahrs.home_is_set() &&
        loc.get_distance(ahrs.get_home()) < DISTANCE_HOME_MINCHANGE) {
        return;  // moved less than 0.5m, no update
    }
    IGNORE_RETURN(ahrs.set_home(loc));
}
```

Called from `one_second_loop()` only while disarmed. Refines home position as GPS accuracy improves or vehicle is physically relocated. Uses `DISTANCE_HOME_MINCHANGE = 0.5f` meters threshold.

**Key insight**: Home is established well before arming occurs. Arming **requires** home to be set (see pre-arm check below).

#### Home Lock Mechanism

- **Unlocked** (auto-set): `set_home_to_current_location(false)` -- home can be updated by `update_home()`
- **Locked** (GCS-set): `set_home_to_current_location(true)` or `set_home(loc, true)` -- `update_home()` skips locked home via `home_is_locked()` check
- `ahrs.lock_home()` sets the lock; no unlock API exists (reset on power cycle)

#### Pre-Arm Check: "Waiting for Home"

`repo/ardupilot/libraries/AP_Arming/AP_Arming.cpp:694-738`:

```cpp
bool AP_Arming::gps_checks(bool report)
{
    if (check_enabled(Check::GPS)) {
        // ... GPS fix and health checks ...
        if (!AP::ahrs().home_is_set()) {
            check_failed(Check::GPS, report, "AHRS: waiting for home");
            return false;
        }
    }
}
```

Home check is part of the GPS arming check category. If `ARMING_CHECK` has GPS bit enabled, arming is blocked until home is set. This ensures RTL and failsafe actions always have a valid home target.

#### HOME_POSITION Broadcasting -- On-Change Only

`repo/ardupilot/libraries/AP_AHRS/AP_AHRS.cpp:3035-3085`:

```cpp
bool AP_AHRS::set_home(const Location &loc)
{
    // ... validation ...
    _home = tmp;
    _home_is_set = true;
    Log_Write_Home_And_Origin();
    GCS_SEND_MESSAGE(MSG_HOME);    // broadcast HOME_POSITION
    GCS_SEND_MESSAGE(MSG_ORIGIN);  // broadcast GPS_GLOBAL_ORIGIN
    // ... persist to storage ...
}
```

**Every** call to `set_home()` broadcasts HOME_POSITION on all active MAVLink channels. This happens on first GPS fix, on `update_home()` refinements, and on GCS override.

**HOME_POSITION is NOT in any periodic stream group.** The `STREAM_POSITION_msgs[]` array (`repo/ardupilot/libraries/GCS_MAVLink/GCS_MAVLink_Parameters.cpp:272`) contains only `MSG_LOCATION` and `MSG_LOCAL_POSITION` -- no `MSG_HOME`. It is purely event-driven.

#### GCS Requesting Home

**MAV_CMD_REQUEST_MESSAGE with param1=242** (`repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3272-3303`):

The mapping at line 1046 connects `MAVLINK_MSG_ID_HOME_POSITION` (242) to `MSG_HOME`, which calls `send_home_position()`.

**MAV_CMD_GET_HOME_POSITION (deprecated)** (`repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:5092-5107`):

Sends both HOME_POSITION and GPS_GLOBAL_ORIGIN. Returns `MAV_RESULT_FAILED` if home not set.

**SET_HOME_POSITION message (ID 243) is NOT handled** -- ArduPilot has no handler for this message. Only `MAV_CMD_DO_SET_HOME` via COMMAND_INT is supported.

#### Rover RTL Home Usage

`repo/ardupilot/Rover/mode_rtl.cpp:1-28`:

```cpp
bool ModeRTL::_enter()
{
    if (!AP::ahrs().home_is_set()) { return false; }
    // Navigate to rally point or home
}
```

RTL refuses entry if home is not set.

#### GLOBAL_POSITION_INT `relative_alt` is Home-Relative

`repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:6105-6138`:

```cpp
int32_t GCS_MAVLINK::global_position_int_relative_alt() const {
    float posD;
    AP::ahrs().get_relative_position_D_home(posD);
    posD *= -1000.0f; // change from down to up and metres to millimeters
    return posD;
}
```

The `relative_alt` field in GLOBAL_POSITION_INT is **altitude above home** in millimeters. Mission Planner uses this for altitude display on the HUD. pico_trail currently hardcodes this to `0` (`telemetry.rs:308`).

#### DO_SET_HOME via COMMAND_LONG

ArduPilot supports `MAV_CMD_DO_SET_HOME` via both COMMAND_INT and COMMAND_LONG. For COMMAND_LONG, it converts to COMMAND_INT automatically via `try_command_long_as_command_int()` (`repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:5285-5346`), forcing `MAV_FRAME_GLOBAL` (MSL altitude). pico_trail only handles COMMAND_INT.

#### Home Persists After GPS Loss

Once home is set, the `_home_is_set` flag is never cleared. If GPS fix is lost, `update_home()` simply returns early (cannot get current location). Home remains valid for RTL and failsafe actions.

#### Mission Waypoint 0 = Home

ArduPilot saves home to mission storage as waypoint index 0 every time home changes (`repo/ardupilot/libraries/AP_AHRS/AP_AHRS.cpp:3076`). GCS clients reading mission items expect index 0 to be the home position. This is relevant for future mission protocol compatibility but not required for the initial home position implementation.

#### Home on Disarm

Home is **not** reset or modified during disarm. After disarming, `update_home()` resumes at 1 Hz, so unlocked home will drift to current location. Locked home (GCS-set) is preserved.

### Mission Planner Guided Mode Interaction

When Mission Planner initiates Guided mode:

1. It expects HOME_POSITION to already be known (received via on-change broadcast when home was first set on GPS fix)
2. It uses home for distance-to-home calculations shown on the HUD
3. If home is unknown, it reports "failed to set home" because it cannot compute relative position
4. This error prevents Guided mode entry from the Mission Planner UI

The fix requires pico_trail to auto-set home on GPS fix and broadcast HOME_POSITION, so Mission Planner knows the home position before any mode changes are requested.

### Current pico_trail Patterns

**TelemetryStreamer pattern** (`crates/core/src/communication/handlers/telemetry.rs`):

Each telemetry message has a `StreamConfig` with configurable rate. The `update()` method checks `should_send()` for each stream and builds the message from `SystemState`.

**MAV_CMD_REQUEST_MESSAGE pattern** (`crates/core/src/communication/handlers/command.rs:313`):

Currently handles message IDs 148 and 300. Returns `MavResult` only (no extra messages). To return HOME_POSITION data, the pattern needs extension -- either refactor `handle_request_message` to return `(MavResult, Vec<MavMessage>)` or handle HOME_POSITION request at a higher level in `handle_command_long` where the return type already supports extra messages.

### Data Analysis

**Home Position Update Frequency**:

| Event                  | Frequency    | When                             |
| ---------------------- | ------------ | -------------------------------- |
| First GPS fix auto-set | Once         | On first valid 3D GPS position   |
| Disarmed refinement    | 1 Hz         | While disarmed, if moved >0.5m   |
| GCS override           | On command   | Any time via MAV_CMD_DO_SET_HOME |
| Arming SmartRTL home   | Once per arm | SmartRTL records its own home    |

**HOME_POSITION Bandwidth**:

HOME_POSITION_DATA is \~52 bytes. With on-change-only broadcasting, typical bandwidth is near zero -- only a few messages per flight session (first fix + occasional disarmed updates). No periodic stream overhead.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: The system shall automatically set home position to the current GPS location when a valid 3D GPS fix is first acquired
  - Rationale: ArduPilot sets home on first valid EKF/GPS position via `ahrs_update()` (400 Hz check); home must be available before arming
  - Acceptance Criteria:
    - Check in vehicle control loop: if home is not set and GPS fix is 3D or better, set home
    - Use existing `set_home_to_current()` method
    - Set home as unlocked (can be refined by disarmed update)
    - Broadcast HOME_POSITION message on MAVLink channel after setting
    - Log "Home set to {lat}, {lon}" via `log_info!`

- [ ] **FR-DRAFT-2**: The system shall update home position at 1 Hz while disarmed if the vehicle has moved more than 0.5 meters from current home
  - Rationale: ArduPilot's `update_home()` refines home as GPS accuracy improves or vehicle is physically relocated before flight
  - Acceptance Criteria:
    - Only update while disarmed
    - Skip if home is locked (GCS-set)
    - Calculate distance from current GPS to current home; skip if < 0.5m
    - Update home and broadcast HOME_POSITION
    - Use `DISTANCE_HOME_MINCHANGE = 0.5` meters threshold (matching ArduPilot `Rover/defines.h:81`)

- [ ] **FR-DRAFT-3**: The system shall broadcast HOME_POSITION on all active MAVLink channels whenever home position is set or updated
  - Rationale: ArduPilot broadcasts inside `AP_AHRS::set_home()` via `GCS_SEND_MESSAGE(MSG_HOME)`; GCS needs immediate notification of home changes
  - Acceptance Criteria:
    - Send HOME_POSITION when `set_home()` or `set_home_to_current()` is called
    - Include lat/lon in degE7 and altitude in mm format
    - On-change only -- no periodic stream (matching ArduPilot)

- [ ] **FR-DRAFT-4**: The system shall respond to `MAV_CMD_REQUEST_MESSAGE` with param1=242 by sending the current HOME_POSITION
  - Rationale: Standard MAVLink protocol; GCS uses this to request home on demand (e.g., after reconnection)
  - Acceptance Criteria:
    - Handle message ID 242 in command handler
    - Return HOME_POSITION message in response
    - Return `MAV_RESULT_ACCEPTED` if home is set
    - Return `MAV_RESULT_FAILED` if home is not set (home_position is None)

- [ ] **FR-DRAFT-5**: The system shall require home position to be set as a pre-arm check (GPS category)
  - Rationale: ArduPilot blocks arming with "AHRS: waiting for home" if home is not set; ensures RTL and failsafe actions always have a valid destination
  - Acceptance Criteria:
    - Add home check in GPS arming check category
    - Fail with message "waiting for home" if `home_position` is `None`
    - Check is skippable via `ARMING_CHECK` bitmask (matching ArduPilot behavior)

- [ ] **FR-DRAFT-6**: The system shall support home lock: GCS-set home shall not be overwritten by automatic home updates
  - Rationale: ArduPilot distinguishes locked (GCS-set) vs unlocked (auto-set) home; `update_home()` skips locked home
  - Acceptance Criteria:
    - Add `home_locked: bool` flag to `SystemState`
    - `MAV_CMD_DO_SET_HOME` sets `home_locked = true`
    - Auto-set and disarmed update set `home_locked = false`
    - Disarmed update skips if `home_locked == true`
    - Lock resets on power cycle (no persistence needed)

- [ ] **FR-DRAFT-7**: The system shall compute GLOBAL_POSITION_INT `relative_alt` as altitude relative to home position
  - Rationale: ArduPilot computes `relative_alt` as current altitude minus home altitude (`GCS_Common.cpp:6105-6113`); Mission Planner uses this for altitude display on the HUD
  - Acceptance Criteria:
    - If home is set: `relative_alt = (current_gps_alt - home_alt) * 1000` (meters to mm)
    - If home is not set: `relative_alt = 0` (current behavior, acceptable fallback)
    - Update `build_global_position_int()` in `TelemetryStreamer`

- [ ] **FR-DRAFT-8**: The system shall handle `MAV_CMD_GET_HOME_POSITION` (deprecated) by responding with HOME_POSITION
  - Rationale: Mission Planner may send this command on connect to retrieve home position; ArduPilot supports it for backward compatibility (`GCS_Common.cpp:5092-5107`)
  - Acceptance Criteria:
    - Handle in COMMAND_INT handler (matching ArduPilot)
    - Return HOME_POSITION message in response
    - Return `MAV_RESULT_ACCEPTED` if home set, `MAV_RESULT_FAILED` if not

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Home position auto-set shall complete within 1ms of detecting valid GPS fix
  - Category: Performance
  - Rationale: Single GPS read + memory write is trivially fast
  - Target: < 1ms latency

- [ ] **NFR-DRAFT-2**: HOME_POSITION broadcast bandwidth shall be near zero under normal operation
  - Category: Resource Constraints
  - Rationale: On-change-only broadcasting means only a few messages per session
  - Target: < 10 messages per typical flight session (\~520 bytes total)

## Design Considerations

### Technical Constraints

- **Vehicle control loop**: Auto-set and disarmed-update logic should live in the vehicle control loop (MavlinkLoopRunner or equivalent), not in SystemState itself -- SystemState is a data struct, not a controller
- **Broadcast mechanism**: `set_home()` / `set_home_to_current()` are called inside `critical_section::with()` blocks. Broadcasting HOME_POSITION from inside a critical section is not possible (MAVLink send requires non-trivial work). Instead, the caller should build and send HOME_POSITION after the critical section
- **TelemetryStreamer** takes `&SystemState` in `update()` -- no periodic HOME_POSITION stream needed (matches ArduPilot)
- **handle_request_message** currently returns only `MavResult` -- the `handle_command_long` dispatcher already supports extra messages via `Vec<MavMessage, 4>`, so HOME_POSITION request can be handled at that level
- **Pre-arm checks**: Existing arming flow in `crates/core/src/arming/` has check categories; adding a home check fits the established pattern

### Potential Approaches

1. **Option A: Minimal -- Auto-set on GPS fix + Broadcast on change**
   - Pros:
     - Resolves the core Mission Planner issue
     - Small change surface
     - Auto-set ensures home is available before arming
   - Cons:
     - No REQUEST_MESSAGE support (GCS must wait for broadcast)
     - No home lock mechanism
     - No disarmed refinement
   - Effort: Low (4-6 hours)

2. **Option B: ArduPilot-aligned -- Auto-set + Disarmed update + Broadcast + REQUEST_MESSAGE + Pre-arm check** -- Recommended
   - Pros:
     - Matches ArduPilot behavior closely (verified against source code)
     - On-change broadcast only (no periodic stream overhead)
     - Home lock prevents GCS override from being overwritten
     - Pre-arm check ensures RTL always has a valid home
     - REQUEST_MESSAGE for GCS on-demand queries
   - Cons:
     - Moderate implementation effort
     - Need to wire broadcast mechanism from control loop to MAVLink sender
   - Effort: Medium (8-12 hours)

3. **Option C: Full ArduPilot parity with EKF origin**
   - Pros:
     - Full ArduPilot compatibility including EKF origin handling
     - GPS_GLOBAL_ORIGIN message support
   - Cons:
     - pico_trail does not have EKF yet -- premature
     - Over-engineering for current needs
   - Effort: High (20-30 hours)

**Recommendation**: Option B provides the best balance. It closely matches ArduPilot's actual implementation (verified against source), resolves the Mission Planner issue, and avoids unnecessary periodic streaming.

### Implementation Strategy

**Phase 1: Auto-set home on GPS fix + Broadcast on change**

- Add home auto-set check in vehicle control loop: if `!state.has_home() && state.gps_position.is_some() && gps.fix_type >= Fix3D`, set home
- After `set_home_to_current()`, build HOME_POSITION message and send via MAVLink
- Wire broadcast: the control loop (MavlinkLoopRunner) already has access to both SYSTEM_STATE and the MAVLink sender

**Phase 2: Disarmed home update + Home lock**

- Add `home_locked: bool` field to `SystemState` (default `false`)
- In control loop at 1 Hz while disarmed: if `!home_locked && has_home()`, calculate distance to current home, update if >0.5m
- `MAV_CMD_DO_SET_HOME` handler sets `home_locked = true`
- Auto-set and disarmed update set `home_locked = false`

**Phase 3: Pre-arm check + MAV_CMD_REQUEST_MESSAGE + GET_HOME_POSITION**

- Add "waiting for home" check in GPS arming check category
- Add message ID 242 handling in `handle_command_long` (return HOME_POSITION in extra messages Vec)
- Add `MAV_CMD_GET_HOME_POSITION` handler in COMMAND_INT (return HOME_POSITION, `MAV_RESULT_FAILED` if home not set)

**Phase 4: GLOBAL_POSITION_INT `relative_alt` fix**

- Update `build_global_position_int()` in TelemetryStreamer to compute `relative_alt` from home altitude
- If home set: `relative_alt = meters_to_mm(gps_alt - home_alt)`
- If home not set: `relative_alt = 0` (unchanged fallback)

### Architecture Impact

**Modified modules**:

- `crates/core/src/autopilot/state.rs` -- Add `home_locked: bool` field; update `set_home()` to accept lock parameter
- `crates/core/src/autopilot/mavlink_runner.rs` -- Add home auto-set check, disarmed update logic, and HOME_POSITION broadcast in control loop
- `crates/core/src/communication/handlers/command.rs` -- Add HOME_POSITION to REQUEST_MESSAGE handler; add GET_HOME_POSITION handler; set `home_locked = true` in DO_SET_HOME; move `build_home_position_message()` to shared location
- `crates/core/src/communication/handlers/telemetry.rs` -- Fix `build_global_position_int()` to compute `relative_alt` from home
- `crates/core/src/arming/` -- Add "waiting for home" pre-arm check in GPS check category

**No new modules required** -- all changes fit within existing structure.

**Memory Impact**:

| Component                 | RAM Usage | Notes               |
| ------------------------- | --------- | ------------------- |
| `home_locked: bool` field | 1 B       | In SystemState      |
| **Total additional**      | **1 B**   | Negligible overhead |

## ArduPilot Parameters

This feature does not require new parameters. Home position management is architectural behavior:

- **Home is set automatically** on first GPS fix (not parameter-controlled in ArduPilot)
- **Home update while disarmed** uses hardcoded threshold `DISTANCE_HOME_MINCHANGE = 0.5m` (not a parameter)
- **Pre-arm check** is gated by existing `ARMING_CHECK` bitmask (GPS bit)
- **MAV_CMD_DO_SET_HOME** (command 179) allows GCS override (already implemented)
- **MAV_CMD_REQUEST_MESSAGE** (command 512) with param1=242 requests HOME_POSITION (standard MAVLink)

No custom parameters needed.

## Risk Assessment

| Risk                                                        | Probability | Impact | Mitigation Strategy                                                                                 |
| ----------------------------------------------------------- | ----------- | ------ | --------------------------------------------------------------------------------------------------- |
| GPS fix not available before arm attempt                    | Medium      | Medium | Pre-arm check blocks arming with clear "waiting for home" message                                   |
| Disarmed update overwrites GCS-set home                     | Low         | Medium | Home lock mechanism prevents this; GCS-set home is locked                                           |
| HOME_POSITION broadcast missed by GCS                       | Low         | Low    | GCS can request via MAV_CMD_REQUEST_MESSAGE(242)                                                    |
| Refactoring handle_request_message breaks existing behavior | Low         | Medium | Handle HOME_POSITION at handle_command_long level (no refactoring of handle_request_message needed) |
| Mission Planner still fails after fix                       | Low         | Medium | Test with Mission Planner to verify HOME_POSITION reception resolves "failed to set home"           |
| Distance calculation for home update threshold              | Low         | Low    | Use existing navigation distance functions; 0.5m threshold is generous                              |

## Open Questions

- [x] Should home be set on GPS fix or on arming? --> Decision: On first GPS fix, matching ArduPilot. Home is a pre-arm prerequisite, not set during arming. ArduPilot's `ahrs_update()` sets home as soon as EKF/GPS is valid, then arming checks require it.
- [x] Should home be updated while disarmed? --> Decision: Yes, at 1 Hz if moved >0.5m, matching ArduPilot's `update_home()`. This refines home as GPS warms up and allows vehicle relocation between flights.
- [x] Should HOME_POSITION be periodic or on-change? --> Decision: On-change only, matching ArduPilot. HOME_POSITION is not in any ArduPilot stream group. On-change broadcast is sufficient; GCS can request on demand via MAV_CMD_REQUEST_MESSAGE.
- [x] Should home be a pre-arm requirement? --> Decision: Yes, matching ArduPilot's "AHRS: waiting for home" check. Ensures RTL and failsafe actions always have valid home.
- [x] Should GCS-set home be protected from auto-update? --> Decision: Yes, via home lock. GCS `MAV_CMD_DO_SET_HOME` locks home; disarmed update skips locked home.
- [ ] Should `SET_HOME_POSITION` message (ID 243) be handled? --> Recommendation: No. ArduPilot does not handle this message either. `MAV_CMD_DO_SET_HOME` via COMMAND_INT is the only supported method.
- [x] Should `MAV_CMD_DO_SET_HOME` be supported via COMMAND_LONG? --> Decision: Not in initial implementation. ArduPilot auto-converts COMMAND_LONG to COMMAND_INT, but pico_trail already handles DO_SET_HOME via COMMAND_INT which is the preferred protocol. Add COMMAND_LONG support later if GCS compatibility issues arise.
- [x] Should `MAV_CMD_GET_HOME_POSITION` be supported? --> Decision: Yes. Mission Planner may send this on connect. ArduPilot supports it for backward compatibility. Low effort to implement.
- [x] Should `GPS_GLOBAL_ORIGIN` be sent alongside `HOME_POSITION`? --> Decision: Not initially. pico_trail does not have EKF origin concept. ArduPilot sends both together, but GPS_GLOBAL_ORIGIN requires EKF infrastructure we lack.

## Recommendations

### Immediate Actions

1. **Adopt Option B**: ArduPilot-aligned implementation with auto-set, disarmed update, on-change broadcast, REQUEST_MESSAGE, pre-arm check, and relative_alt fix
2. **Auto-set on GPS fix**: Check in control loop; set home when first 3D fix acquired
3. **Disarmed home update**: 1 Hz update while disarmed, skip if locked, skip if moved <0.5m
4. **On-change broadcast**: Send HOME_POSITION after every `set_home()` call (no periodic stream)
5. **Pre-arm check**: Block arming with "waiting for home" if home not set
6. **Home lock**: GCS override locks home; auto-set/update does not
7. **Fix GLOBAL_POSITION_INT `relative_alt`**: Compute from home altitude instead of hardcoded 0
8. **Support MAV_CMD_GET_HOME_POSITION**: For Mission Planner backward compatibility

### Next Steps

1. [ ] Review and approve this analysis
2. [ ] Create formal requirements (FR/NFR) for home position management
3. [ ] Create task package with phased implementation plan
4. [ ] Test with Mission Planner to verify "failed to set home" is resolved

### Out of Scope

- **EKF origin management**: ArduPilot links EKF origin to home; pico_trail does not have EKF yet
- **GPS_GLOBAL_ORIGIN message**: Related to EKF origin, not needed without EKF
- **SET_HOME_POSITION message handler**: Message ID 243 not handled by ArduPilot either; only MAV_CMD_DO_SET_HOME supported
- **MAV_CMD_DO_SET_HOME via COMMAND_LONG**: ArduPilot auto-converts COMMAND_LONG to COMMAND_INT; pico_trail handles COMMAND_INT only. Add later if GCS compatibility issues arise
- **GPS_GLOBAL_ORIGIN message**: ArduPilot sends alongside HOME_POSITION; requires EKF origin concept we lack
- **Rally points**: ArduPilot feature for alternate return locations
- **Home altitude calibration**: Barometric altitude correction for home altitude
- **Home persistence**: ArduPilot persists home to backup registers for watchdog recovery; pico_trail resets on power cycle (acceptable for now)
- **Mission waypoint 0 = home**: ArduPilot saves home to mission storage as waypoint index 0; relevant for future mission protocol but not needed for initial home implementation
- **NAV_CONTROLLER_OUTPUT `wp_dist`**: ArduPilot sends distance-to-waypoint (equals distance-to-home during RTL) in NAV_CONTROLLER_OUTPUT; deferred to navigation telemetry work

## Appendix

### References

- ArduPilot `ahrs_update()` home auto-set: `repo/ardupilot/Rover/Rover.cpp:297-314`
- ArduPilot `update_home()` disarmed refinement: `repo/ardupilot/Rover/commands.cpp:44-65`
- ArduPilot `set_home_to_current_location()`: `repo/ardupilot/Rover/commands.cpp:4-40`
- ArduPilot `AP_AHRS::set_home()` with broadcast: `repo/ardupilot/libraries/AP_AHRS/AP_AHRS.cpp:3035-3085`
- ArduPilot `send_home_position()`: `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3095-3125`
- ArduPilot `gps_checks()` pre-arm: `repo/ardupilot/libraries/AP_Arming/AP_Arming.cpp:694-738`
- ArduPilot `handle_command_request_message()`: `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:3272-3303`
- ArduPilot `handle_command_get_home_position()`: `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:5092-5107`
- ArduPilot `handle_command_do_set_home()`: `repo/ardupilot/libraries/GCS_MAVLink/GCS_Common.cpp:5492-5513`
- ArduPilot `DISTANCE_HOME_MINCHANGE`: `repo/ardupilot/Rover/defines.h:81` (0.5f meters)
- ArduPilot Setting Home documentation: <https://ardupilot.org/dev/docs/mavlink-get-set-home-and-origin.html>
- MAVLink HOME_POSITION message: <https://mavlink.io/en/messages/common.html#HOME_POSITION>
- MAVLink MAV_CMD_DO_SET_HOME: <https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME>
- MAVLink MAV_CMD_REQUEST_MESSAGE: <https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE>
- pico_trail HomePosition struct: `crates/core/src/autopilot/state.rs:178`
- pico_trail MAV_CMD_DO_SET_HOME handler: `crates/core/src/communication/handlers/command.rs:572`
- pico_trail TelemetryStreamer: `crates/core/src/communication/handlers/telemetry.rs:106`
- pico_trail PostArmInitializer: `crates/core/src/arming/initialization.rs:71`
- GitHub Issue: <https://github.com/shirou/pico_trail/issues/20>

### ArduPilot Home Position Lifecycle (Verified from Source)

```
Power on
    --> ahrs_update() runs at 400 Hz
    --> GPS acquires satellites, eventually gets 3D fix
    --> EKF initializes with GPS position

First valid EKF position:
    --> ahrs_update(): !home_is_set() -> set_home_to_current_location(false)
        --> AP_AHRS::set_home(loc)
            --> _home = loc, _home_is_set = true
            --> GCS_SEND_MESSAGE(MSG_HOME)      // broadcast HOME_POSITION
            --> GCS_SEND_MESSAGE(MSG_ORIGIN)     // broadcast GPS_GLOBAL_ORIGIN
    --> GCS receives HOME_POSITION, displays home marker

While disarmed (1 Hz via one_second_loop):
    --> update_home():
        1. If home_is_locked() -> return (GCS set it, don't touch)
        2. Get current GPS location
        3. If distance(current, home) < 0.5m -> return (hasn't moved enough)
        4. ahrs.set_home(current_loc) -> broadcast HOME_POSITION

GCS requests arm (MAV_CMD_COMPONENT_ARM_DISARM):
    --> pre_arm_checks()
        --> gps_checks(): !home_is_set() -> fail "AHRS: waiting for home"
        --> (home IS set, so check passes)
    --> arm() succeeds
    --> SmartRTL records its own home

GCS overrides home:
    --> MAV_CMD_DO_SET_HOME (COMMAND_INT)
        --> set_home(loc, true)  // locked = true
        --> broadcast HOME_POSITION
        --> update_home() will now skip (home_is_locked)

GCS requests home on demand:
    --> MAV_CMD_REQUEST_MESSAGE(param1=242)
        --> send_home_position() -> HOME_POSITION message

Vehicle enters RTL:
    --> ModeRTL::_enter()
        --> if !home_is_set() -> refuse entry
        --> navigate to home (or closest rally point)
```

### Proposed pico_trail Home Position Lifecycle

```
Power on / Boot
    --> SystemState.home_position = None, home_locked = false

GPS acquires 3D fix:
    --> Control loop detects: !has_home() && gps.fix_type >= Fix3D
        --> set_home_to_current() -> home_position = Some(...)
        --> home_locked = false (unlocked, can be refined)
        --> Build and send HOME_POSITION on MAVLink channel
    --> GCS receives HOME_POSITION, displays home marker

While disarmed (1 Hz in control loop):
    --> If home_locked -> skip
    --> If has_home() && distance(gps, home) >= 0.5m:
        --> set_home_to_current() -> update home_position
        --> Build and send HOME_POSITION

GCS requests arm:
    --> Pre-arm checks: has_home()? If not -> fail "waiting for home"
    --> arm() succeeds

GCS overrides home:
    --> MAV_CMD_DO_SET_HOME (COMMAND_INT)
        --> set_home(loc) + home_locked = true
        --> Build and send HOME_POSITION
        --> Disarmed update will now skip (locked)

GCS requests home on demand:
    --> MAV_CMD_REQUEST_MESSAGE(param1=242)
        --> Return HOME_POSITION in command response

Vehicle enters RTL (failsafe or operator):
    --> RtlMode reads SystemState.home_position
    --> Navigate to home
```
