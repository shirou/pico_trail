# T-00037 Mission Execution Sequencer Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design specifies a platform-agnostic MissionSequencer in the core crate, modeled after ArduPilot's AP_Mission. The sequencer owns mission command classification, NAV/DO dual-slot advancement, and hold time logic. It drives command execution through a `MissionExecutor` trait that the firmware crate implements. This architecture places sequencing logic where it can be fully tested on host, while firmware handles physical execution and telemetry.

## Success Metrics

- [ ] MissionSequencer dual-slot logic passes comprehensive unit tests on host
- [ ] Auto mode delegates sequencing to MissionSequencer (no direct index management)
- [ ] MISSION_CURRENT streams at 1Hz during mission execution
- [ ] MISSION_ITEM_REACHED sent within 100ms of waypoint arrival
- [ ] DO_CHANGE_SPEED modifies navigation speed without disrupting navigation timing
- [ ] All existing tests pass (no regressions)

## Background and Current State

- Context: Auto mode navigates through NAV_WAYPOINT items using MissionStorage as the single source of truth (ADR-00023). Three gaps remain: no telemetry, no DO command execution, no mission management.
- Current behavior:
  - `AutoMode::update()` directly manages `current_wp_index` and calls `advance_waypoint()` / `complete_mission()` in firmware's `core/mission/state.rs`
  - `TelemetryStreamer` sends 6 message types with no mission messages
  - `MissionHandler` handles upload/download but not CLEAR_ALL or SET_CURRENT
  - `waypoint_to_mission_item()` falls back all commands to NAV_WAYPOINT
- Pain points:
  - Sequencing logic is embedded in firmware's Auto mode, untestable on host
  - All mission items treated as NAV regardless of command type
  - No GCS feedback on mission progress
- Constraints:
  - Core crate: `#![no_std]`, zero `cfg` directives, no Embassy dependencies
  - Embedded: `heapless` containers, no heap allocation
  - Must not break existing GUIDED mode behavior
- Related ADRs:
  - [ADR-00034-mission-execution-telemetry-architecture](../../adr/ADR-00034-mission-execution-telemetry-architecture.md)
  - [ADR-00023-unified-waypoint-navigation](../../adr/ADR-00023-unified-waypoint-navigation.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────┐
│                    Core Crate (no_std, zero cfg)         │
│                                                          │
│  ┌──────────────────┐    ┌─────────────────────────┐    │
│  │  MissionStorage   │◄───│    MissionSequencer      │    │
│  │  (existing)       │    │                          │    │
│  │  - waypoints[]    │    │  - nav_cmd index/loaded  │    │
│  │  - current_index  │    │  - do_cmd index/loaded   │    │
│  └──────────────────┘    │  - holding_since          │    │
│                           │  - mission_speed          │    │
│  ┌──────────────────┐    │                          │    │
│  │  MissionEvent     │◄───│  update(storage,exec,t) │    │
│  │  CurrentChanged   │    │  start(storage,exec)    │    │
│  │  ItemReached      │    │  stop()                 │    │
│  │  MissionComplete  │    │  set_current(i,s,exec)  │    │
│  │  MissionCleared   │    │  clear(storage)         │    │
│  └──────────────────┘    └──────────┬──────────────┘    │
│                                      │                    │
│  ┌──────────────────┐               │ calls              │
│  │  MissionExecutor  │◄──────────────┘                    │
│  │  (trait)          │                                    │
│  │  start_command()  │  ┌──────────────────┐             │
│  │  verify_command() │  │  Command helpers  │             │
│  │  on_complete()    │  │  is_nav_command() │             │
│  └──────────────────┘  │  cmd_has_location │             │
│                          └──────────────────┘             │
└─────────────────────────────────────────────────────────┘
                          │
                          │ trait impl
                          ▼
┌─────────────────────────────────────────────────────────┐
│                  Firmware Crate                           │
│                                                          │
│  ┌────────────────────┐  ┌─────────────────────────┐    │
│  │  AutoMode           │  │  TelemetryStreamer       │    │
│  │  impl MissionExec.  │  │  + MISSION_CURRENT (1Hz) │    │
│  │  - start_command()  │  └─────────────────────────┘    │
│  │  - verify_command() │                                  │
│  │  - on_complete()    │  ┌─────────────────────────┐    │
│  │                     │  │  MissionHandler          │    │
│  │  update():          │  │  + CLEAR_ALL             │    │
│  │   sequencer.update  │  │  + SET_CURRENT           │    │
│  │   → MissionEvents   │  │  + command preservation  │    │
│  │   → MAVLink msgs    │  └─────────────────────────┘    │
│  └────────────────────┘                                   │
└─────────────────────────────────────────────────────────┘
```

### Components

#### Core Crate: `mission/executor.rs`

```rust
/// Result of starting a command
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandStartResult {
    /// Command accepted and executing (verify will be called each tick)
    Accepted,
    /// Command completed immediately (no verify needed)
    Complete,
    /// Command not recognized or not supported (skip)
    Unsupported,
}

/// Events emitted by the sequencer for telemetry coordination
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MissionEvent {
    /// Current target NAV command changed (seq index)
    CurrentChanged(u16),
    /// A mission item was reached after hold time (seq index)
    ItemReached(u16),
    /// Mission completed (no more NAV commands)
    MissionComplete,
    /// Mission cleared
    MissionCleared,
}

/// Command execution contract between sequencer and vehicle mode.
///
/// The sequencer calls these methods to delegate physical command execution
/// to the vehicle layer. Modeled after ArduPilot's AP_Mission 3-callback
/// interface (cmd_start_fn, cmd_verify_fn, mission_complete_fn).
pub trait MissionExecutor {
    /// Begin executing a mission command.
    ///
    /// For NAV commands: set navigation target, return Accepted.
    /// For DO commands: apply the effect (e.g., change speed), return Complete.
    /// For unknown commands: return Unsupported to skip.
    fn start_command(&mut self, cmd: &Waypoint) -> CommandStartResult;

    /// Check if a command has completed.
    ///
    /// Called each tick for the active NAV command slot.
    /// For NAV: return true when vehicle reached target (within WP_RADIUS).
    /// For DO: should not normally be called (most return Complete from start).
    fn verify_command(&mut self, cmd: &Waypoint) -> bool;

    /// Called when no more NAV commands remain in the mission.
    fn on_mission_complete(&mut self);
}
```

#### Core Crate: `mission/sequencer.rs`

```rust
/// Maximum mission events emitted per update cycle
pub const MAX_MISSION_EVENTS: usize = 4;

/// Internal flags for sequencer state tracking
#[derive(Clone, Copy, Debug, Default)]
struct SequencerFlags {
    /// NAV command slot has an active command
    nav_cmd_loaded: bool,
    /// DO command slot has an active command
    do_cmd_loaded: bool,
    /// All DO commands between current and next NAV are processed
    do_cmd_all_done: bool,
}

/// Mission sequencer — drives mission execution through MissionExecutor.
///
/// Platform-agnostic state machine that owns:
/// - Command classification (NAV vs DO)
/// - Dual-slot NAV/DO advancement (ArduPilot pattern)
/// - Hold time management (NAV_WAYPOINT param1)
/// - Mission speed override tracking (DO_CHANGE_SPEED)
///
/// Does not know about MAVLink, actuators, GPS, or any platform service.
pub struct MissionSequencer {
    state: MissionState,
    flags: SequencerFlags,
    /// Index of the active NAV command in MissionStorage
    nav_cmd_index: u16,
    /// Index of the active DO command in MissionStorage
    do_cmd_index: u16,
    /// Timestamp (ms) when hold started at current waypoint
    holding_since: Option<u64>,
    /// Speed override from DO_CHANGE_SPEED (None = use default)
    mission_speed: Option<f32>,
}
```

Key methods:

| Method                                     | Description                                           |
| ------------------------------------------ | ----------------------------------------------------- |
| `new() -> Self`                            | Create sequencer in Idle state                        |
| `start(storage, executor) -> Vec<Event>`   | Start mission from index 0                            |
| `stop() -> Vec<Event>`                     | Stop mission, preserve index for resume               |
| `update(storage, executor, now_ms) -> Vec` | Main tick: drive NAV/DO slots, emit events            |
| `set_current(i, storage, exec) -> Vec`     | Handle MISSION_SET_CURRENT from GCS                   |
| `clear(storage) -> Vec`                    | Handle MISSION_CLEAR_ALL from GCS                     |
| `state() -> MissionState`                  | Get current state (Idle/Running/Completed)            |
| `current_nav_index() -> u16`               | Get index of active NAV command (for MISSION_CURRENT) |
| `mission_speed() -> Option<f32>`           | Get speed override from DO_CHANGE_SPEED               |

#### Core Crate: Command Classification Helpers

```rust
/// MAV_CMD_NAV_LAST: command IDs at or below this are NAV commands
pub const MAV_CMD_NAV_LAST: u16 = 95;

/// MAV_CMD_DO_CHANGE_SPEED
pub const MAV_CMD_DO_CHANGE_SPEED: u16 = 178;

/// Classify a command as NAV (drives navigation) or DO (immediate action).
pub fn is_nav_command(command_id: u16) -> bool {
    command_id <= MAV_CMD_NAV_LAST
}

/// Check if a command carries a geographic location (lat/lon).
pub fn cmd_has_location(command_id: u16) -> bool {
    is_nav_command(command_id)
}
```

### Data Flow

#### Mission Start

1. GCS sends MAV_CMD_MISSION_START
2. Firmware calls `sequencer.start(storage, executor)`
3. Sequencer scans forward from index 0:
   - Loads first NAV command into nav slot, calls `executor.start_command(nav_cmd)`
   - Loads any preceding DO commands into do slot, calls `executor.start_command(do_cmd)`
4. Emits `CurrentChanged(nav_index)` event
5. Firmware converts event to MISSION_CURRENT MAVLink message

#### Update Tick (50Hz)

1. Auto mode calls `sequencer.update(storage, executor, now_ms)`
2. Sequencer processes DO slot:
   - If no DO loaded and not all done: scan for next DO before NAV boundary
   - If DO loaded: call `executor.verify_command(do_cmd)`, clear on true
3. Sequencer processes NAV slot:
   - If no NAV loaded: scan for next NAV, call `executor.start_command()`
   - If no more NAV: emit `MissionComplete`, call `executor.on_mission_complete()`
   - If NAV loaded: call `executor.verify_command(nav_cmd)`
   - If verified: check hold time (param1)
     - If holding and not elapsed: return (wait)
     - If hold complete or no hold: emit `ItemReached(seq)`, advance
4. Return events
5. Auto mode converts events to MAVLink messages for telemetry queue

#### Hold Time

1. `verify_command()` returns true (vehicle at target)
2. Sequencer reads `param1` from the NAV waypoint
3. If `param1 > 0` and `holding_since` is None: record `now_ms`
4. If elapsed < param1 seconds: return without advancing (executor keeps motors stopped via verify returning true)
5. When elapsed >= param1: clear `holding_since`, emit `ItemReached`, advance

#### DO Command Execution (DO_CHANGE_SPEED)

1. Sequencer encounters DO_CHANGE_SPEED at current scan position
2. Calls `executor.start_command(do_cmd)` -> executor returns `Complete`
3. Sequencer records `mission_speed = Some(param2)` from the waypoint
4. Executor can query `sequencer.mission_speed()` for speed override
5. Sequencer advances DO slot to next DO command (or marks all done at NAV boundary)

### Data Models and Types

#### MissionEvent Mapping to MAVLink

| MissionEvent          | MAVLink Message       | Trigger                     |
| --------------------- | --------------------- | --------------------------- |
| `CurrentChanged(seq)` | MISSION_CURRENT       | NAV command loaded          |
| `ItemReached(seq)`    | MISSION_ITEM_REACHED  | NAV command verified + hold |
| `MissionComplete`     | (handled by callback) | No more NAV commands        |
| `MissionCleared`      | MISSION_ACK           | CLEAR_ALL processed         |

#### AutoState Changes (Firmware)

Current AutoState (3 fields) is replaced by delegation to MissionSequencer:

```rust
struct AutoState {
    /// Navigation target set by start_command
    current_target: Option<PositionTarget>,
    /// Whether verify_command reported at_target on last tick
    at_target: bool,
}
```

Auto mode no longer tracks `current_wp_index` or `mission_complete` directly — these are owned by MissionSequencer.

### Error Handling

- Empty mission on start: `sequencer.start()` returns empty events, state stays Idle
- Invalid index on set_current: Reject if index >= storage.count()
- Unknown command in executor: Return `Unsupported`, sequencer skips to next command
- GPS loss during verify: Auto mode's `update()` returns error before calling sequencer (existing behavior)

### Security Considerations

- No new external input vectors beyond existing MAVLink handlers
- MissionStorage size limited to MAX_WAYPOINTS (50)
- MISSION_CLEAR_ALL should be rejected if armed and mission running (ArduPilot convention)

### Performance Considerations

- `MissionSequencer::update()` is O(1) per tick in steady state (verify one NAV, optionally verify one DO)
- DO command scanning is O(n) per advance but bounded by waypoint count (max 50) and happens only on NAV transitions
- `heapless::Vec<MissionEvent, 4>` avoids heap allocation for event return
- All trait method calls are monomorphized at compile time (no vtable overhead in practice since Auto mode is the only implementor)

### Platform Considerations

#### Core Crate (Host + Embedded)

- Pure `#![no_std]`, no feature gates
- `MissionSequencer` uses only stack-allocated state (no heapless containers internally)
- Time is injected as `now_ms: u64` parameter (no TimeSource dependency)
- `MissionExecutor` trait uses `&mut self` (no async, no lifetime complexity)

#### Firmware (RP2350)

- Global `MissionSequencer` wrapped in `EmbassyState<MissionSequencer>`
- Accessed via critical-section Mutex (same pattern as MISSION_STORAGE)
- Telemetry buffer increased from 6 to 8 for MISSION_CURRENT

## Alternatives Considered

1. **Sequencer in firmware crate only**
   - Pros: No trait needed, direct access to actuators and telemetry
   - Cons: Sequencing logic untestable on host, violates core/firmware boundary principle

2. **Async trait with Embassy channels for events**
   - Pros: Natural fit for Embassy async model
   - Cons: Adds Embassy dependency to core crate (violates zero-cfg), channels require static allocation

### Decision Rationale

Synchronous trait in core crate with event return values. This keeps core zero-cfg while providing the same decoupling as an async event system. The firmware layer handles the async/Embassy integration. Aligns with ADR-00034.

## Migration and Compatibility

- Auto mode's direct calls to `advance_waypoint()`, `complete_mission()`, `get_current_target()` in `firmware/src/core/mission/state.rs` will be replaced by MissionSequencer method calls
- Existing helper functions (`start_mission_from_beginning`, etc.) remain available for GUIDED mode (which does not use the sequencer)
- No changes to MAVLink protocol behavior — only additions (MISSION_CURRENT, MISSION_ITEM_REACHED, CLEAR_ALL, SET_CURRENT)
- GUIDED mode is unchanged (single waypoint, no sequencing)

## Testing Strategy

### Unit Tests (Core Crate)

```rust
// Sequencer state transitions
#[test] fn test_sequencer_start_stop_lifecycle()
#[test] fn test_sequencer_start_empty_mission()
#[test] fn test_sequencer_start_single_waypoint()

// Dual-slot advancement
#[test] fn test_nav_only_mission_advances()
#[test] fn test_do_commands_execute_between_navs()
#[test] fn test_do_commands_stop_at_nav_boundary()
#[test] fn test_mixed_nav_do_mission()

// Hold time
#[test] fn test_hold_time_zero_advances_immediately()
#[test] fn test_hold_time_waits_before_advance()
#[test] fn test_item_reached_after_hold_complete()

// DO_CHANGE_SPEED
#[test] fn test_do_change_speed_updates_mission_speed()
#[test] fn test_mission_speed_none_by_default()

// Command classification
#[test] fn test_is_nav_command_boundary()
#[test] fn test_do_command_classification()

// Mission management
#[test] fn test_set_current_updates_nav_slot()
#[test] fn test_clear_resets_state()

// Events
#[test] fn test_current_changed_on_nav_advance()
#[test] fn test_item_reached_on_nav_verify()
#[test] fn test_mission_complete_event()
```

Tests use a `MockExecutor` implementing `MissionExecutor` that records calls and returns configurable results.

### Firmware Tests

- Auto mode enter/update/exit lifecycle with sequencer
- MISSION_CURRENT message format verification
- MISSION_ITEM_REACHED message format verification
- CLEAR_ALL and SET_CURRENT handler responses
- Command type preservation in waypoint_to_mission_item()
- RP2350 build verification

## Documentation Impact

- Update `docs/architecture.md` with MissionSequencer component
- ADR-00034 already documents the architecture decision

## External References

- [ArduPilot AP_Mission.h](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.h) - Reference architecture
- [ArduPilot AP_Mission.cpp](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.cpp) - Dual-slot implementation
- [ArduPilot Rover mode_auto.cpp](https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_auto.cpp) - Executor reference
- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html) - Protocol specification

## Open Questions

- [ ] Should MissionSequencer handle DO_CHANGE_SPEED internally (update mission_speed field) or delegate to executor? -> Next step: Evaluate during Phase 2 implementation; ArduPilot handles some DO commands in AP_Mission directly
- [ ] Should the event queue use a callback pattern instead of return value? -> Method: Profile return-value approach first; switch to callback only if needed
- [ ] How should the sequencer interact with GUIDED mode's single-waypoint model? -> Next step: GUIDED mode does not use sequencer (confirmed in ADR scope)

## Appendix

### ArduPilot AP_Mission Callback Contract

```cpp
// ArduPilot's 3-functor interface (from AP_Mission.h)
AP_Mission(
    mission_cmd_fn_t cmd_start_fn,      // Start executing a command
    mission_cmd_fn_t cmd_verify_fn,     // Check if command is complete
    mission_complete_fn_t mission_complete_fn  // Mission finished
);
```

This maps directly to the `MissionExecutor` trait's three methods.

### Dual-Slot Execution Model

```text
Mission: [NAV_WP(0)] [DO_SPEED(1)] [DO_SET_ROI(2)] [NAV_WP(3)] [DO_SPEED(4)] [NAV_WP(5)]

When NAV_WP(0) is active:
  NAV slot: NAV_WP(0) — verify each tick
  DO slot:  DO_SPEED(1) → DO_SET_ROI(2) → all_done
  Boundary: NAV_WP(3) stops DO scanning

When NAV_WP(3) becomes active:
  NAV slot: NAV_WP(3) — verify each tick
  DO slot:  DO_SPEED(4) → all_done
  Boundary: NAV_WP(5) stops DO scanning
```

### Glossary

- **NAV command**: Mission command with ID <= 95 that drives vehicle navigation
- **DO command**: Mission command with ID > 95 that executes an immediate action
- **Dual-slot**: ArduPilot pattern of maintaining two parallel command execution slots (NAV + DO)
- **Hold time**: NAV_WAYPOINT param1 specifying seconds to pause at waypoint before advancing
- **MissionSequencer**: Core crate component that owns mission sequencing logic
- **MissionExecutor**: Trait implemented by vehicle mode to physically execute commands
