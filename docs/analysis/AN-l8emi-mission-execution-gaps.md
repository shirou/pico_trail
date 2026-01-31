# AN-l8emi Mission Execution Gaps

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - [AN-zd6uw-mission-execution](../analysis/AN-zd6uw-mission-execution.md)
- Related Requirements:
  - [FR-jm7mj-auto-mode-mission-execution](../requirements/FR-jm7mj-auto-mode-mission-execution.md)
  - [FR-v6571-mission-execution-state](../requirements/FR-v6571-mission-execution-state.md)
  - [FR-m2c9z-mission-waypoint-navigation](../requirements/FR-m2c9z-mission-waypoint-navigation.md)
  - [FR-w893v-mission-start-command](../requirements/FR-w893v-mission-start-command.md)
  - [FR-sifsm-mission-current-telemetry](../requirements/FR-sifsm-mission-current-telemetry.md)
  - [FR-zcnqw-mission-item-reached](../requirements/FR-zcnqw-mission-item-reached.md)
  - [FR-f2f8q-mission-clear-all](../requirements/FR-f2f8q-mission-clear-all.md)
  - [FR-aulp3-mission-set-current](../requirements/FR-aulp3-mission-set-current.md)
  - [FR-5zqns-nav-do-command-separation](../requirements/FR-5zqns-nav-do-command-separation.md)
  - [FR-0q1tf-do-change-speed](../requirements/FR-0q1tf-do-change-speed.md)
  - [FR-4dq92-waypoint-hold-time](../requirements/FR-4dq92-waypoint-hold-time.md)
  - [FR-ahzhr-mission-command-preservation](../requirements/FR-ahzhr-mission-command-preservation.md)
  - [NFR-at4uq-mission-telemetry-latency](../requirements/NFR-at4uq-mission-telemetry-latency.md)
  - [NFR-8oq1h-mission-command-execution-overhead](../requirements/NFR-8oq1h-mission-command-execution-overhead.md)
- Related ADRs:
  - [ADR-2hs12-unified-waypoint-navigation](../adr/ADR-2hs12-unified-waypoint-navigation.md)
  - [ADR-0yapl-mission-execution-telemetry-architecture](../adr/ADR-0yapl-mission-execution-telemetry-architecture.md)
- Related Tasks:
  - [T-c26bh-unified-waypoint-navigation](../tasks/T-c26bh-unified-waypoint-navigation/README.md)
  - [T-r7k2p-mission-execution-sequencer](../tasks/T-r7k2p-mission-execution-sequencer/README.md)

## Executive Summary

This analysis investigates the gaps between the current pico_trail mission implementation and a fully functional mission execution system. The previous analysis (AN-zd6uw) established the unified waypoint architecture and basic Auto mode operation. This follow-up analysis focuses on three areas that remain unimplemented: (1) mission progress telemetry (MISSION_CURRENT, MISSION_ITEM_REACHED), (2) non-NAV mission command execution (DO_CHANGE_SPEED, DO_SET_SERVO, etc.), and (3) mission management operations (MISSION_CLEAR_ALL, MISSION_SET_CURRENT).

**Key Finding**: The current implementation navigates through NAV_WAYPOINT items but does not report progress to the GCS, does not execute DO\_\* commands embedded in missions, and lacks basic mission management operations. ArduPilot's mission system provides a well-documented reference architecture with a NAV/DO dual-slot execution model that this project should follow incrementally.

## Problem Space

### Current State

The basic mission infrastructure from AN-zd6uw has been implemented:

1. **Mission upload/download**: Full MAVLink mission protocol (MISSION_COUNT, MISSION_ITEM_INT, MISSION_REQUEST_INT, MISSION_ACK) is working. Missions can be uploaded from Mission Planner and downloaded back.

2. **Auto mode navigation**: `AutoMode` (`crates/firmware/src/rover/mode/auto.rs`) navigates sequentially through NAV_WAYPOINT items. Waypoint arrival is detected via WP_RADIUS. Autocontinue is respected.

3. **Mission state machine**: `MissionState` (Idle/Running/Completed) tracks execution. Global `MISSION_STORAGE` and `MISSION_STATE` provide thread-safe access.

4. **What is missing**:
   - GCS receives no feedback about mission progress. No MISSION_CURRENT or MISSION_ITEM_REACHED messages are sent.
   - All mission items are treated as NAV_WAYPOINT regardless of their actual command ID. DO\_\* commands are stored but ignored during execution.
   - MISSION_CLEAR_ALL is not handled. Missions cannot be cleared from GCS.
   - MISSION_SET_CURRENT is not handled. GCS cannot jump to a specific waypoint.
   - NAV_WAYPOINT param1 (hold time) is ignored. The rover advances immediately on arrival.
   - The `waypoint_to_mission_item()` function in `mission.rs:514-521` falls back all command IDs to MAV_CMD_NAV_WAYPOINT, losing non-waypoint command types during download.

### Desired State

1. **Mission progress telemetry**: GCS displays real-time mission progress, showing the current waypoint and receiving notifications when each waypoint is reached.

2. **DO command execution**: Missions containing DO_CHANGE_SPEED, DO_SET_SERVO, and similar commands execute those commands at the correct point in the mission sequence.

3. **Mission management**: Operators can clear missions, jump to specific waypoints, and manage mission execution from the GCS.

4. **Hold time support**: NAV_WAYPOINT param1 causes the rover to hold position for the specified duration before advancing.

### Gap Analysis

| Component                 | Current                    | Desired                                        | Gap                     |
| ------------------------- | -------------------------- | ---------------------------------------------- | ----------------------- |
| MISSION_CURRENT           | Not sent                   | Streamed at 1Hz, sent on change                | Telemetry addition      |
| MISSION_ITEM_REACHED      | Not sent                   | Sent when NAV command completes                | Event-triggered message |
| MISSION_CLEAR_ALL         | Not handled                | Clears mission storage                         | Handler needed          |
| MISSION_SET_CURRENT       | Not handled                | Jumps to specified waypoint                    | Handler needed          |
| DO\_\* command execution  | Stored but ignored         | Executed at correct sequence point             | Execution logic needed  |
| NAV_WAYPOINT hold time    | Ignored (param1)           | Hold at waypoint for param1 seconds            | Timer logic needed      |
| Command type preservation | Falls back to NAV_WAYPOINT | Preserves original command ID                  | Conversion fix needed   |
| NAV/DO command separation | All items treated as NAV   | NAV drives navigation, DO executes in parallel | Architecture change     |

## Stakeholder Analysis

| Stakeholder     | Interest/Need                              | Impact | Priority |
| --------------- | ------------------------------------------ | ------ | -------- |
| GCS Operator    | See mission progress in real-time          | High   | P0       |
| GCS Operator    | Clear/restart missions from GCS            | High   | P0       |
| Mission Planner | Upload missions with DO commands           | Medium | P1       |
| Developer       | Clean NAV/DO separation architecture       | Medium | P1       |
| Safety          | Predictable behavior for all command types | High   | P0       |

## Research & Discovery

### User Feedback

The initial implementation (AN-zd6uw) resolved the core navigation issue. The following gaps were identified during integration testing with Mission Planner:

- Mission Planner cannot display mission progress (no MISSION_CURRENT)
- Missions with DO_CHANGE_SPEED items execute but speed changes are ignored
- No way to clear a mission from GCS without rebooting

### Competitive Analysis

#### ArduPilot Mission State Machine

ArduPilot defines three mission states in `libraries/AP_Mission/AP_Mission.h`:

```cpp
enum mission_state {
    MISSION_STOPPED  = 0,  // Paused; nav_cmd index preserved for resume
    MISSION_RUNNING  = 1,  // Actively executing
    MISSION_COMPLETE = 2   // All items executed
};
```

State transitions:

| From     | To       | Trigger                                 |
| -------- | -------- | --------------------------------------- |
| Any      | RUNNING  | `start()`                               |
| RUNNING  | STOPPED  | `stop()`                                |
| STOPPED  | RUNNING  | `resume()`                              |
| COMPLETE | RUNNING  | `resume()` (calls `start()` internally) |
| RUNNING  | COMPLETE | `complete()` (no more NAV commands)     |
| Any      | STOPPED  | `clear()`                               |

Additional internal flags in ArduPilot:

```cpp
struct Mission_Flags {
    mission_state state;
    bool nav_cmd_loaded;      // NAV command actively executing
    bool do_cmd_loaded;       // DO command actively executing
    bool do_cmd_all_done;     // All DO commands between NAVs processed
    bool in_landing_sequence;
    bool resuming_mission;
};
```

**Comparison with pico_trail**: The current `MissionState` (Idle/Running/Completed) maps roughly to ArduPilot's three states but lacks the STOPPED (paused) concept and the NAV/DO tracking flags.

#### ArduPilot NAV/DO Dual-Slot Execution

ArduPilot maintains two parallel command slots during mission execution:

- `_nav_cmd`: Currently executing navigation command (one at a time)
- `_do_cmd`: Currently executing DO/conditional command (one at a time)

The `update()` loop (called at 10Hz+):

```
1. Process NAV command:
   - If no NAV loaded: advance_current_nav_cmd()
   - If NAV loaded: verify_command(_nav_cmd)
     - If verified: clear nav_cmd_loaded, advance to next NAV

2. Process DO command (in parallel):
   - If no DO loaded: advance_current_do_cmd()
   - If DO loaded: verify_command(_do_cmd)
     - If verified: clear do_cmd_loaded
```

**Command classification**: Commands with ID <= `MAV_CMD_NAV_LAST` (95) are NAV commands. Everything else is DO or CONDITION.

**DO command scoping**: DO commands are associated with the preceding NAV command. When `advance_current_nav_cmd()` finds the next NAV command, it scans for DO commands between the old and new NAV commands. If the next NAV command is reached before pending DO commands execute, those DO commands are skipped.

**DO command execution is immediate**: Most DO commands return true from `start_command()` immediately:

```cpp
// ArduPilot Rover start_command():
case MAV_CMD_DO_CHANGE_SPEED: do_change_speed(cmd); return true;  // immediate
case MAV_CMD_DO_SET_SERVO:    do_set_servo(cmd);    return true;  // immediate
case MAV_CMD_DO_SET_RELAY:    do_set_relay(cmd);    return true;  // immediate
```

| Aspect                           | NAV Commands              | DO Commands                      |
| -------------------------------- | ------------------------- | -------------------------------- |
| Controls vehicle position        | Yes                       | No                               |
| Max concurrent                   | 1                         | 1                                |
| Drives mission progression       | Yes                       | No                               |
| verify_command() called          | Repeatedly until complete | Usually returns true immediately |
| Reported by MISSION_CURRENT      | Yes                       | No                               |
| Reported by MISSION_ITEM_REACHED | Yes                       | No                               |

#### ArduPilot MISSION_CURRENT Message

MAVLink message #42, sent in the following situations:

1. **Periodic streaming**: Part of `SRx_EXTENDED_STATUS` telemetry, nominally at 1Hz
2. **On current item change**: When `advance_current_nav_cmd()` loads a new NAV command
3. **In response to MISSION_SET_CURRENT**: As an acknowledgment

Fields:

| Field           | Type   | Description                                       |
| --------------- | ------ | ------------------------------------------------- |
| `seq`           | uint16 | Current target mission item sequence number       |
| `total`         | uint16 | Total items (excl. home); UINT16_MAX = no mission |
| `mission_state` | uint8  | MISSION_STATE enum                                |
| `mission_mode`  | uint8  | 0=Unknown, 1=In mission mode, 2=Suspended         |

Key behavior: `seq` reflects the currently targeted NAV command index, not DO commands. `total` excludes the home position (seq 0) in ArduPilot. pico_trail does not use seq 0 for home, so `total` equals the mission item count directly.

#### ArduPilot MISSION_ITEM_REACHED Message

MAVLink message #46, sent when a NAV command verification returns true:

1. `AP_Mission::update()` calls `verify_command(_nav_cmd)`
2. Vehicle's verify function returns true when waypoint is complete (including hold time)
3. `gcs().send_mission_item_reached_message(cmd.index)` is called
4. Then `advance_current_nav_cmd()` advances to the next item

**Timing**: Sent after hold time expires, before advancing. Only for NAV commands.

#### ArduPilot MISSION_CLEAR_ALL Handling

```cpp
bool AP_Mission::clear() {
    // Cannot clear while armed and running
    if (hal.util->get_soft_armed() && _flags.state == MISSION_RUNNING) {
        return false;
    }
    truncate(0);
    _nav_cmd.index = AP_MISSION_CMD_INDEX_NONE;
    _do_cmd.index  = AP_MISSION_CMD_INDEX_NONE;
    _flags.nav_cmd_loaded = false;
    _flags.do_cmd_loaded  = false;
    _flags.state = MISSION_STOPPED;
    return true;
}
```

Responds with MISSION_ACK (ACCEPTED or DENIED). Cannot clear while armed and mission is running.

#### ArduPilot MISSION_SET_CURRENT Handling

When mission is running:

1. Clears DO command state
2. Clears NAV command loaded flag
3. Calls `advance_current_nav_cmd(index)` to load new NAV command
4. Vehicle immediately navigates to the new target waypoint
5. Jump counters (DO_JUMP) are NOT reset unless jumping to seq 0

When mission is stopped/complete:

1. Loads first NAV command found at/after specified index
2. Sets state to STOPPED (not RUNNING)
3. Mission will begin from this point when resumed

#### ArduPilot Rover Waypoint Hold Time (param1)

In Rover mode_auto.cpp:

1. `do_nav_wp()`: Stores `loiter_duration` from param1
2. `verify_nav_wp()`:
   - Checks if destination reached (within WP_RADIUS)
   - On first reach: records `loiter_start_time`, sends STATUSTEXT
   - If `loiter_duration == 0`: returns true immediately
   - If `loiter_duration > 0`: returns true only when elapsed time >= loiter_duration
3. Hold is passive (motors stop, no active position correction)

#### ArduPilot Rover Mission Completion

Controlled by `MIS_DONE_BEHAVE` parameter:

- HOLD (default): Vehicle stops
- LOITER: Initiates loiter at last waypoint
- ACRO: Switches to Acro mode
- MANUAL: Switches to Manual mode

### Technical Investigation

#### Current Code Analysis

**Auto mode update loop** (`crates/firmware/src/rover/mode/auto.rs:165-248`):

The `update()` method treats every mission item as a navigation target:

```rust
let target = match get_current_target() {
    Some(t) => t,
    None => { /* mission complete */ }
};
// Navigate to target...
if output.at_target {
    if advance_waypoint() { /* next */ }
    else { /* mission complete */ }
}
```

There is no distinction between NAV and DO commands. `get_current_target()` converts whatever waypoint is at the current index into a `PositionTarget`, regardless of command type.

**Telemetry streaming** (`crates/firmware/src/communication/mavlink/handlers/telemetry.rs`):

The `TelemetryStreamer` sends 6 message types (HEARTBEAT, ATTITUDE, GPS_RAW_INT, GLOBAL_POSITION_INT, SYS_STATUS, BATTERY_STATUS). Neither MISSION_CURRENT nor MISSION_ITEM_REACHED is included.

**Mission handler command conversion** (`crates/firmware/src/communication/mavlink/handlers/mission.rs:514-521`):

```rust
let command = match wp.command {
    16 => MavCmd::MAV_CMD_NAV_WAYPOINT,
    _ => MavCmd::MAV_CMD_NAV_WAYPOINT, // Default fallback
};
```

All commands fall back to NAV_WAYPOINT, losing the original command type during download.

**Mission item storage** (`crates/core/src/mission/mod.rs:37-62`):

The `Waypoint` struct stores `command: u16` which correctly preserves the command ID during upload. The issue is only in the download conversion and the execution logic.

### Data Analysis

N/A - No runtime telemetry data available for mission execution analysis.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: Mission progress telemetry via MISSION_CURRENT
  - Rationale: GCS must display which waypoint the vehicle is targeting
  - Acceptance Criteria:
    - MISSION_CURRENT streamed at 1Hz during mission execution
    - MISSION_CURRENT sent immediately when current waypoint changes
    - `seq` field reports current NAV command index
    - `total` reports total mission item count
    - `mission_state` maps from MissionState enum

- [ ] **FR-DRAFT-2**: Waypoint arrival notification via MISSION_ITEM_REACHED
  - Rationale: GCS must know when each waypoint is reached for progress display and logging
  - Acceptance Criteria:
    - MISSION_ITEM_REACHED sent when NAV command completes (after hold time)
    - Sent before advancing to next waypoint
    - `seq` field contains the reached waypoint sequence number
    - Only sent for NAV commands, not DO commands

- [ ] **FR-DRAFT-3**: MISSION_CLEAR_ALL handling
  - Rationale: Operators must be able to clear missions from GCS
  - Acceptance Criteria:
    - MISSION_CLEAR_ALL clears MissionStorage and resets MissionState to Idle
    - Responds with MISSION_ACK (ACCEPTED)
    - Rejected if armed and mission is running (MISSION_ACK DENIED)

- [ ] **FR-DRAFT-4**: MISSION_SET_CURRENT handling
  - Rationale: Operators must be able to restart or jump to a specific waypoint
  - Acceptance Criteria:
    - MISSION_SET_CURRENT changes current waypoint index
    - If mission running: immediately navigates to specified waypoint
    - If mission stopped: sets start point for next resume
    - Responds with MISSION_CURRENT message as acknowledgment

- [ ] **FR-DRAFT-5**: NAV/DO command separation in mission execution
  - Rationale: Missions contain both navigation commands and immediate action commands; the rover must handle both correctly
  - Acceptance Criteria:
    - NAV commands (ID <= 95) drive vehicle navigation
    - DO commands execute immediately without affecting navigation
    - DO commands between NAV commands execute when the preceding NAV starts
    - Unknown commands are skipped with a warning

- [ ] **FR-DRAFT-6**: DO_CHANGE_SPEED mission command support
  - Rationale: Speed changes during missions are common for obstacle avoidance and area-specific speed limits
  - Acceptance Criteria:
    - `MAV_CMD_DO_CHANGE_SPEED` (178) changes the navigation speed
    - Speed persists until next DO_CHANGE_SPEED or mission end
    - Speed reverts to default on mission completion

- [ ] **FR-DRAFT-7**: NAV_WAYPOINT hold time support (param1)
  - Rationale: ArduPilot convention uses NAV_WAYPOINT param1 for hold time in seconds; missions may require the rover to pause at waypoints
  - Acceptance Criteria:
    - NAV_WAYPOINT param1 specifies hold time in seconds
    - Rover stops motors on arrival
    - Timer starts on first arrival (within WP_RADIUS)
    - MISSION_ITEM_REACHED sent after hold time expires
    - param1 == 0 means no hold (advance immediately)

- [ ] **FR-DRAFT-8**: Mission command type preservation in protocol
  - Rationale: The mission download handler must return original command types, not fallback to NAV_WAYPOINT
  - Acceptance Criteria:
    - `waypoint_to_mission_item()` preserves original command ID
    - `frame` field preserves original MAV_FRAME value
    - GCS receives correct command types during mission download

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Mission telemetry latency
  - Category: Performance
  - Rationale: GCS must receive timely mission progress updates
  - Target: MISSION_ITEM_REACHED sent within 100ms of waypoint arrival detection

- [ ] **NFR-DRAFT-2**: Mission command execution overhead
  - Category: Performance
  - Rationale: DO command processing must not delay the navigation update loop
  - Target: DO command execution completes within a single update cycle (< 20ms)

## Design Considerations

### Technical Constraints

1. **No heap allocation**: All mission data uses heapless Vec (max 50 items)
2. **Embassy async runtime**: Telemetry messages must be queued for async send
3. **Single-threaded execution**: Auto mode update loop runs in the main scheduler task
4. **Limited MAVLink message buffer**: `TelemetryStreamer::update()` returns `heapless::Vec<MavMessage, 6>` -- adding MISSION_CURRENT requires increasing capacity or using a separate emission path
5. **ArduPilot parameter conventions**: Parameters must use standard ArduPilot names

### Potential Approaches

1. **Option A**: Incremental telemetry-first approach
   - Add MISSION_CURRENT and MISSION_ITEM_REACHED to telemetry
   - Add MISSION_CLEAR_ALL handler
   - Defer DO command execution to a later phase
   - Pros: Quick GCS compatibility, low risk, addresses P0 gaps
   - Cons: Missions with DO commands still incomplete
   - Effort: Low

2. **Option B**: Full NAV/DO dual-slot execution
   - Implement ArduPilot-style NAV/DO separation in Auto mode
   - Add command classifier (is_nav_cmd)
   - Add DO command executor with extensible command handlers
   - Add all telemetry messages
   - Pros: Complete solution, extensible for future commands
   - Cons: Significant architectural change to Auto mode
   - Effort: Medium

3. **Option C**: Event-driven mission executor
   - Create a dedicated MissionExecutor module separate from mode logic
   - NAV commands emit navigation events, DO commands emit action events
   - Auto mode subscribes to events
   - Pros: Clean separation, highly testable
   - Cons: Over-engineered for current command set
   - Effort: High

**Recommendation**: Option A for immediate GCS usability, then Option B to add DO command execution. Option C is premature given the current scope.

### Architecture Impact

- **TelemetryStreamer**: Needs MISSION_CURRENT stream (1Hz) and event-triggered MISSION_ITEM_REACHED. May require increasing the message buffer size or adding a separate event message queue.
- **MissionHandler**: Needs MISSION_CLEAR_ALL and MISSION_SET_CURRENT handlers.
- **AutoMode**: Needs NAV/DO classification logic and hold time support. The `update()` loop must differentiate between command types.
- **Mission command conversion**: `waypoint_to_mission_item()` and the frame/command enum conversions need to handle all supported command IDs, not just NAV_WAYPOINT.

## Risk Assessment

| Risk                                           | Probability | Impact | Mitigation Strategy                                     |
| ---------------------------------------------- | ----------- | ------ | ------------------------------------------------------- |
| MISSION_CURRENT increases telemetry bandwidth  | Low         | Low    | 1Hz rate is minimal (< 20 bytes/s)                      |
| DO command affects navigation timing           | Medium      | Medium | Execute DO commands synchronously in same update cycle  |
| MISSION_CLEAR_ALL during active navigation     | Medium      | High   | Reject if armed and running (ArduPilot pattern)         |
| Hold time timer accuracy                       | Low         | Low    | Embassy timer provides millisecond accuracy             |
| Unknown command ID in mission                  | Medium      | Medium | Skip with log warning, advance to next item             |
| Message buffer overflow adding MISSION_CURRENT | Low         | Medium | Increase heapless Vec capacity or use separate emission |

## Open Questions

- [x] What mission states does ArduPilot use? -> MISSION_STOPPED, MISSION_RUNNING, MISSION_COMPLETE (see Research)
- [x] When is MISSION_ITEM_REACHED sent relative to waypoint advancement? -> After hold time, before advancing (see Research)
- [x] Does ArduPilot allow MISSION_CLEAR_ALL while armed? -> Only if mission is not running (see Research)
- [x] How does ArduPilot classify NAV vs DO commands? -> Command ID <= MAV_CMD_NAV_LAST (95) is NAV (see Research)
- [x] How does Rover handle hold time at waypoints? -> Passive hold (motors stop, no position correction) (see Research)
- [ ] Should pico_trail add a STOPPED (paused) state to match ArduPilot? -> Next step: Evaluate in requirements phase whether pause/resume is needed
- [ ] Which DO commands should be supported in Phase 1? -> Next step: Prioritize based on Mission Planner usage patterns
- [ ] Should `MIS_DONE_BEHAVE` parameter be implemented? -> Method: Check if Mission Planner expects this parameter

## Recommendations

### Immediate Actions

1. Create formal requirements from FR-DRAFT-1 (MISSION_CURRENT) and FR-DRAFT-2 (MISSION_ITEM_REACHED) -- these are P0 for GCS usability
2. Create formal requirement from FR-DRAFT-3 (MISSION_CLEAR_ALL) -- required for basic mission management
3. Fix the command/frame fallback in `waypoint_to_mission_item()` (FR-DRAFT-8) -- data corruption bug

### Next Steps

1. [ ] Create formal requirements: FR for MISSION_CURRENT telemetry, FR for MISSION_ITEM_REACHED
2. [ ] Create formal requirements: FR for MISSION_CLEAR_ALL, FR for MISSION_SET_CURRENT
3. [ ] Create formal requirements: FR for NAV/DO separation, FR for DO_CHANGE_SPEED
4. [ ] Create formal requirements: FR for hold time, FR for command type preservation
5. [ ] Create task for Phase 1 (telemetry + mission management)
6. [ ] Create task for Phase 2 (NAV/DO execution + hold time)

### Out of Scope

- Mission persistence to flash storage (separate feature, requires flash driver)
- Conditional commands (MAV_CMD_CONDITION_DELAY, CONDITION_DISTANCE) -- low priority for rover
- DO_JUMP / mission loops -- complex state management, defer
- Geofence integration during missions
- Terrain following
- `MIS_DONE_BEHAVE` parameter (Hold is sufficient for current needs)
- `MIS_RESTART` parameter (restart behavior)
- Mission change detection during execution (ArduPilot's `mis_change_detector`)

## Appendix

### References

- ArduPilot Rover Auto Mode: <https://ardupilot.org/rover/docs/auto-mode.html>
- MAVLink Mission Protocol: <https://mavlink.io/en/services/mission.html>
- MAVLink Common Messages: <https://mavlink.io/en/messages/common.html>
- MAVLink MISSION_CURRENT: <https://mavlink.io/en/messages/common.html#MISSION_CURRENT>
- MAVLink MISSION_ITEM_REACHED: <https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED>
- ArduPilot AP_Mission source: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.h>
- ArduPilot AP_Mission implementation: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.cpp>
- ArduPilot Rover mode_auto: <https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_auto.cpp>
- ArduPilot GCS_Common: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/GCS_MAVLink/GCS_Common.cpp>
- ArduPilot Mission Commands: <https://ardupilot.org/rover/docs/common-mavlink-mission-command-messages-mav_cmd.html>

### Code Locations

| File                                                            | Line    | Description                                   |
| --------------------------------------------------------------- | ------- | --------------------------------------------- |
| crates/firmware/src/rover/mode/auto.rs                          | 165-248 | Auto mode update loop (NAV-only)              |
| crates/firmware/src/rover/mode/auto.rs                          | 224-241 | Waypoint arrival handling (no hold time)      |
| crates/firmware/src/communication/mavlink/handlers/telemetry.rs | 196-252 | TelemetryStreamer update (no MISSION_CURRENT) |
| crates/firmware/src/communication/mavlink/handlers/mission.rs   | 514-521 | Command type fallback bug                     |
| crates/firmware/src/communication/mavlink/handlers/mission.rs   | 494-504 | MISSION_ACK handler (no CLEAR_ALL)            |
| crates/core/src/mission/mod.rs                                  | 37-62   | Waypoint struct (command field preserved)     |
| crates/core/src/mission/state.rs                                | 16-24   | MissionState enum (Idle/Running/Completed)    |
| crates/firmware/src/core/mission/state.rs                       | -       | Global MISSION_STORAGE, MISSION_STATE         |

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../templates/README.md#analysis-template-analysismd) in the templates README.
