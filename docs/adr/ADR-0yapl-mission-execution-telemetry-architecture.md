# ADR-0yapl Mission Execution Telemetry Architecture

## Metadata

- Type: ADR
- Status: Draft

## Links

- Related Analyses:
  - [AN-l8emi-mission-execution-gaps](../analysis/AN-l8emi-mission-execution-gaps.md)
- Impacted Requirements:
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
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-r7k2p-mission-execution-sequencer](../tasks/T-r7k2p-mission-execution-sequencer/README.md)

## Context

The initial unified waypoint navigation (ADR-2hs12) established MissionStorage as the single source of truth and implemented basic Auto mode navigation through NAV_WAYPOINT items. Three categories of gaps remain between the current implementation and a fully functional mission system:

1. **No mission progress telemetry**: The GCS receives no feedback about which waypoint is being targeted or when waypoints are reached. Mission Planner cannot display mission progress without MISSION_CURRENT and MISSION_ITEM_REACHED messages.

2. **No DO command execution**: All mission items are treated as navigation targets regardless of command type. DO commands (DO_CHANGE_SPEED, DO_SET_SERVO) stored in missions are ignored. Attempting to navigate to a DO command item produces undefined behavior since it has no valid lat/lon.

3. **No mission management**: MISSION_CLEAR_ALL and MISSION_SET_CURRENT are not handled. Operators cannot clear missions or jump to specific waypoints from the GCS.

**Forces in tension:**

- GCS usability (need telemetry now) vs architectural cleanliness (full NAV/DO model)
- ArduPilot compatibility (dual-slot execution) vs implementation simplicity (incremental approach)
- Telemetry buffer constraints (`heapless::Vec<MavMessage, 6>`) vs additional message types
- Synchronous Auto mode update loop vs event-driven telemetry emission
- Core crate purity (zero cfg, no_std, no platform deps) vs execution logic that needs platform services

**Current architecture limitations:**

- `AutoState` has no holding state or DO command tracking
- `TelemetryStreamer::update()` returns a fixed-size `heapless::Vec<MavMessage, 6>` with no mission messages
- `get_current_target()` converts any waypoint to `PositionTarget` regardless of command type
- `waypoint_to_mission_item()` falls back all commands to NAV_WAYPOINT (data corruption bug)
- Mission sequencing logic is embedded directly in Auto mode's `update()` loop, making it untestable on host without firmware dependencies

## Decision

We will implement a **MissionSequencer** in the core crate following ArduPilot's AP_Mission architecture: a platform-agnostic sequencing engine that drives mission execution through a `MissionExecutor` trait. The sequencer owns command classification, dual-slot NAV/DO advancement, and hold time logic. The firmware crate provides the `MissionExecutor` implementation (Auto mode) and telemetry integration.

This is based on Option C (event-driven mission executor as a separate module) from the analysis, adapted for the core/firmware crate boundary.

### Decision Drivers

- Mission sequencing is pure algorithm — it belongs in the core crate for host testability
- ArduPilot's 3-callback interface (start/verify/complete) is proven and maps naturally to a Rust trait
- Architecture changes at the sequencing level are costly to retrofit later
- Core crate's zero-cfg constraint requires the sequencer to be platform-agnostic

### Considered Options

- **Option A**: Incremental two-phase approach (telemetry first in firmware, then NAV/DO in firmware)
- **Option B**: Full NAV/DO dual-slot implementation directly in firmware's Auto mode
- **Option C**: Event-driven mission sequencer as a separate module in core crate

### Option Analysis

- **Option A** — Pros: GCS usability delivered first, lower risk per phase, Auto mode changes isolated | Cons: Sequencing logic stays embedded in firmware, hard to test on host, retrofitting to proper architecture later is costly
- **Option B** — Pros: Complete solution in one pass, no temporary workarounds | Cons: Larger change set in firmware, sequencing logic untestable on host, monolithic Auto mode
- **Option C** — Pros: Clean separation, sequencing logic fully testable on host, follows ArduPilot's proven architecture, future commands require no architecture change | Cons: More upfront design work, trait interface must be designed carefully

## Rationale

Option C is selected because:

1. **Architecture changes are costly to retrofit**: Mission sequencing is a core subsystem. Building it correctly from the start avoids a rewrite when command complexity grows. The ArduPilot project evolved AP_Mission over years — we can learn from their final architecture without repeating their incremental path.

2. **ArduPilot's interface is proven and small**: The 3-callback contract (`start_command`, `verify_command`, `on_mission_complete`) has been validated across Rover, Copter, and Plane for over a decade. With only 3 methods, the risk of "wrong abstraction" is minimal.

3. **Core crate testability**: The sequencer's dual-slot logic, command classification, hold time management, and advancement rules are pure algorithms. Placing them in the core crate enables comprehensive unit tests on host (`cargo test --lib`) without Embassy or hardware dependencies.

4. **Platform separation**: The core crate maintains its zero-cfg principle. The sequencer depends only on `MissionStorage`, `Waypoint`, and the `MissionExecutor` trait — all platform-agnostic types. The firmware crate implements `MissionExecutor` with actual actuator control and telemetry.

Option A was rejected because embedding sequencing logic in firmware's Auto mode creates an untestable monolith. Phase 1 would deliver telemetry without the architecture to support it cleanly, and Phase 2 would require restructuring Phase 1's work.

Option B was rejected for similar reasons — it places all logic in firmware where host testing is limited.

## Consequences

### Positive

- Mission sequencing logic is fully testable on host without embedded dependencies
- ArduPilot-compatible dual-slot NAV/DO model from the start
- Adding new DO commands requires only extending `start_command`/`verify_command` in the executor — no sequencer changes
- GCS displays mission progress (MISSION_CURRENT, MISSION_ITEM_REACHED)
- Mission management (clear, set current) available
- Command type preservation bug fixed
- Auto mode becomes simpler — delegates sequencing to MissionSequencer

### Negative

- More upfront design and implementation work than Option A
- MissionExecutor trait introduces an abstraction boundary that all mode implementations must respect
- Trait-based design requires careful consideration of no_std constraints (no dynamic dispatch overhead concerns on embedded)

### Neutral

- Existing GUIDED mode behavior unchanged (single waypoint, no sequencing)
- MissionStorage API unchanged
- MissionState enum unchanged (Idle/Running/Completed)

## Implementation Notes

### Architecture Overview

```text
┌─────────────────────────────────────────────────────────┐
│                    Core Crate                            │
│                                                          │
│  ┌──────────────────┐    ┌─────────────────────────┐    │
│  │  MissionStorage   │◄───│    MissionSequencer      │    │
│  │  (existing)       │    │                          │    │
│  │  - waypoints[]    │    │  - nav_cmd slot          │    │
│  │  - current_index  │    │  - do_cmd slot           │    │
│  └──────────────────┘    │  - state flags           │    │
│                           │  - hold timer            │    │
│  ┌──────────────────┐    │                          │    │
│  │  MissionEvent     │◄───│  update()               │    │
│  │  (enum)           │    │  set_current()          │    │
│  └──────────────────┘    │  clear()                 │    │
│                           │  start() / stop()       │    │
│  ┌──────────────────┐    └──────────┬──────────────┘    │
│  │  MissionExecutor  │◄─────────────┘                    │
│  │  (trait)          │    calls start/verify/complete    │
│  └──────────────────┘                                    │
│                                                          │
│  ┌──────────────────┐                                    │
│  │  Command helpers  │                                    │
│  │  is_nav_command() │                                    │
│  │  cmd_has_location │                                    │
│  └──────────────────┘                                    │
└─────────────────────────────────────────────────────────┘
                          │
                          │ trait impl
                          ▼
┌─────────────────────────────────────────────────────────┐
│                  Firmware Crate                           │
│                                                          │
│  ┌──────────────────┐    ┌─────────────────────────┐    │
│  │  AutoMode         │    │  TelemetryStreamer       │    │
│  │  impl Executor    │    │  + MISSION_CURRENT       │    │
│  │  - start_command  │    │  + MISSION_ITEM_REACHED  │    │
│  │  - verify_command │    └─────────────────────────┘    │
│  │  - on_complete    │                                    │
│  └──────────────────┘    ┌─────────────────────────┐    │
│                           │  MissionHandler          │    │
│                           │  + CLEAR_ALL             │    │
│                           │  + SET_CURRENT           │    │
│                           └─────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
```

### Core Crate: MissionExecutor Trait

Derived from ArduPilot's 3-functor contract:

```rust
/// Result of starting a command
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandStartResult {
    /// Command accepted and executing (verify will be called)
    Accepted,
    /// Command completed immediately (no verify needed)
    Complete,
    /// Command not recognized or not supported
    Unsupported,
}

/// Events emitted by the sequencer for telemetry and coordination
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MissionEvent {
    /// Current target waypoint changed (seq index)
    CurrentChanged(u16),
    /// A mission item was reached (seq index)
    ItemReached(u16),
    /// Mission completed (no more NAV commands)
    MissionComplete,
    /// Mission cleared
    MissionCleared,
}

/// Command execution contract between sequencer and vehicle mode.
///
/// The sequencer calls these methods to delegate physical command
/// execution to the vehicle layer. Modeled after ArduPilot's
/// AP_Mission callback interface.
pub trait MissionExecutor {
    /// Begin executing a mission command.
    ///
    /// For NAV commands: set navigation target.
    /// For DO commands: apply the command effect (e.g., change speed).
    fn start_command(&mut self, cmd: &Waypoint) -> CommandStartResult;

    /// Check if a command has completed.
    ///
    /// Called each tick for the active NAV command.
    /// For NAV: return true when the vehicle has reached the target.
    /// For DO: return true when the command effect is applied.
    fn verify_command(&mut self, cmd: &Waypoint) -> bool;

    /// Called when no more NAV commands remain.
    fn on_mission_complete(&mut self);
}
```

### Core Crate: MissionSequencer

The sequencer is a pure state machine that owns dual-slot logic:

```rust
/// Maximum number of mission events per update cycle
pub const MAX_MISSION_EVENTS: usize = 4;

/// Internal flags for sequencer state
#[derive(Clone, Copy, Debug, Default)]
struct SequencerFlags {
    nav_cmd_loaded: bool,
    do_cmd_loaded: bool,
    do_cmd_all_done: bool,
}

/// Mission sequencer — drives mission execution through MissionExecutor.
///
/// Platform-agnostic. Owns command classification, dual-slot advancement,
/// and hold time logic. Does not know about MAVLink, actuators, or GPS.
pub struct MissionSequencer {
    state: MissionState,
    flags: SequencerFlags,
    nav_cmd_index: u16,
    do_cmd_index: u16,
    holding_since: Option<u64>,   // Timestamp (ms) when hold started
    mission_speed: Option<f32>,   // Speed override from DO_CHANGE_SPEED
}

impl MissionSequencer {
    pub fn new() -> Self { /* ... */ }

    /// Start mission from a given index.
    pub fn start(&mut self, storage: &MissionStorage,
                 executor: &mut dyn MissionExecutor)
                 -> heapless::Vec<MissionEvent, MAX_MISSION_EVENTS> { /* ... */ }

    /// Stop mission (preserves current index for resume).
    pub fn stop(&mut self) -> heapless::Vec<MissionEvent, MAX_MISSION_EVENTS> { /* ... */ }

    /// Main update tick. Called at 50Hz.
    ///
    /// Drives the dual-slot NAV/DO model:
    /// 1. Process DO commands at current position (execute immediately)
    /// 2. Verify active NAV command (check arrival + hold time)
    /// 3. Advance when complete
    ///
    /// Returns events for the firmware layer to convert to telemetry.
    pub fn update(&mut self, storage: &MissionStorage,
                  executor: &mut dyn MissionExecutor,
                  now_ms: u64)
                  -> heapless::Vec<MissionEvent, MAX_MISSION_EVENTS> { /* ... */ }

    /// Handle MISSION_SET_CURRENT from GCS.
    pub fn set_current(&mut self, index: u16, storage: &MissionStorage,
                       executor: &mut dyn MissionExecutor)
                       -> heapless::Vec<MissionEvent, MAX_MISSION_EVENTS> { /* ... */ }

    /// Handle MISSION_CLEAR_ALL from GCS.
    pub fn clear(&mut self, storage: &mut MissionStorage)
                 -> heapless::Vec<MissionEvent, MAX_MISSION_EVENTS> { /* ... */ }

    /// Get current state.
    pub fn state(&self) -> MissionState { self.state }

    /// Get current NAV command index.
    pub fn current_nav_index(&self) -> u16 { self.nav_cmd_index }

    /// Get mission speed override (from DO_CHANGE_SPEED).
    pub fn mission_speed(&self) -> Option<f32> { self.mission_speed }
}
```

### Core Crate: Command Classification

```rust
/// MAV_CMD_NAV_LAST: All command IDs at or below this are NAV commands.
pub const MAV_CMD_NAV_LAST: u16 = 95;

/// MAV_CMD_DO_CHANGE_SPEED
pub const MAV_CMD_DO_CHANGE_SPEED: u16 = 178;

/// Classify a command as NAV (navigation) or DO (immediate action).
pub fn is_nav_command(command_id: u16) -> bool {
    command_id <= MAV_CMD_NAV_LAST
}

/// Check if a command has a geographic location (lat/lon).
pub fn cmd_has_location(command_id: u16) -> bool {
    // NAV commands generally have locations, with exceptions
    // that can be added as needed
    is_nav_command(command_id)
}
```

### Core Crate: Update Loop Logic

```text
MissionSequencer::update(storage, executor, now_ms)
  if state != Running:
    return []

  events = []

  // === DO Slot ===
  if !flags.do_cmd_loaded && !flags.do_cmd_all_done:
    advance_do_cmd(storage, executor)

  if flags.do_cmd_loaded:
    if executor.verify_command(do_cmd):
      flags.do_cmd_loaded = false

  // === NAV Slot ===
  if !flags.nav_cmd_loaded:
    advance_nav_cmd(storage, executor, &mut events)
    if !flags.nav_cmd_loaded:
      // No more NAV commands
      state = Completed
      executor.on_mission_complete()
      events.push(MissionComplete)
      return events

  if flags.nav_cmd_loaded:
    if executor.verify_command(nav_cmd):
      // NAV command complete — check hold time
      hold_time_s = storage.get_waypoint(nav_cmd_index).param1
      if hold_time_s > 0:
        if holding_since.is_none():
          holding_since = Some(now_ms)
          // executor should stop motors on verify=true
        if elapsed(now_ms, holding_since) < hold_time_s:
          return events  // Still holding
      // Hold complete (or no hold)
      holding_since = None
      events.push(ItemReached(nav_cmd_index))
      flags.nav_cmd_loaded = false
      advance_nav_cmd(storage, executor, &mut events)
      if !flags.nav_cmd_loaded:
        state = Completed
        executor.on_mission_complete()
        events.push(MissionComplete)

  return events
```

### Firmware Crate: Auto Mode as MissionExecutor

Auto mode implements `MissionExecutor` and delegates sequencing:

```text
AutoMode
├── state: AutoState (navigation_active, current_target, etc.)
├── nav_controller: SimpleNavigationController
├── actuators: &mut dyn ActuatorInterface
└── Methods:
    ├── start_command(cmd):
    │   NAV_WAYPOINT → set target, reset nav controller → Accepted
    │   DO_CHANGE_SPEED → (handled by sequencer's mission_speed) → Complete
    │   Unknown NAV → treat as waypoint → Accepted
    │   Unknown DO → log warning → Unsupported
    ├── verify_command(cmd):
    │   NAV → run nav controller, return at_target
    │   DO → return true (immediate)
    └── on_mission_complete():
        Stop motors, set mission_complete flag
```

Auto mode `update()` simplifies to:

```text
AutoMode::update(dt)
  events = sequencer.update(storage, self_as_executor, now_ms)
  for event in events:
    match event:
      CurrentChanged(seq) → queue MISSION_CURRENT
      ItemReached(seq) → queue MISSION_ITEM_REACHED
      MissionComplete → (handled by on_mission_complete callback)
      MissionCleared → (handled by clear)
  // Navigation output applied in verify_command()
```

### Firmware Crate: Telemetry Integration

```text
TelemetryStreamer
├── Existing: HEARTBEAT, ATTITUDE, GPS_RAW_INT, GLOBAL_POSITION_INT,
│             SYS_STATUS, BATTERY_STATUS
└── New:      MISSION_CURRENT (1Hz periodic)

Buffer capacity: 6 → 8
```

MISSION_CURRENT is periodic (1Hz). MISSION_ITEM_REACHED is event-driven, queued by Auto mode when the sequencer emits `ItemReached` events. The event queue uses a `heapless::Vec<MavMessage, 4>` returned from the update path.

### Firmware Crate: Mission Management Handlers

```text
MissionHandler
├── Existing: MISSION_COUNT, MISSION_ITEM_INT, MISSION_REQUEST_INT, MISSION_ACK
└── New:      MISSION_CLEAR_ALL → sequencer.clear(storage)
              MISSION_SET_CURRENT → sequencer.set_current(index, storage, executor)
```

### Command Type Preservation Fix

Fix `waypoint_to_mission_item()` to pass through the original `command` and `frame` values instead of falling back to NAV_WAYPOINT.

### Module Impact Summary

| Module                       | Crate    | Changes                                                       |
| ---------------------------- | -------- | ------------------------------------------------------------- |
| `mission/sequencer.rs` (NEW) | core     | MissionSequencer, dual-slot logic, hold time, command helpers |
| `mission/executor.rs` (NEW)  | core     | MissionExecutor trait, MissionEvent, CommandStartResult       |
| `mission/mod.rs`             | core     | Re-export new modules, add `is_nav_command()` helpers         |
| `rover/mode/auto.rs`         | firmware | Implement MissionExecutor, delegate to sequencer              |
| `handlers/telemetry.rs`      | firmware | Add MISSION_CURRENT stream, increase buffer to 8              |
| `handlers/mission.rs`        | firmware | Add CLEAR_ALL, SET_CURRENT handlers; fix command preservation |
| `handlers/dispatcher.rs`     | firmware | Wire new mission message handlers                             |
| `core/mission/state.rs`      | firmware | Integrate MissionSequencer with global state                  |

## Open Questions

- [x] Should Phase 1 skip DO commands or navigate to them? -> The sequencer skips DO commands in the NAV slot and executes them in the DO slot
- [ ] Should MISSION_ITEM_REACHED use the status_notifier channel or a dedicated mission event channel? -> Next step: Evaluate during task design; MissionEvent enum provides the abstraction regardless of transport
- [ ] Should `mission_speed` persist across mode changes or reset? -> Next step: Evaluate in task design; ArduPilot resets speed on mode change
- [ ] Should unknown NAV commands (ID <= 95 but not NAV_WAYPOINT) be treated as waypoints or skipped? -> Method: Check ArduPilot behavior for NAV_LOITER, NAV_RETURN_TO_LAUNCH
- [ ] Should MissionSequencer hold a reference to MissionStorage or receive it as a parameter? -> Receiving as parameter avoids ownership issues in no_std and matches the current global-state access pattern

## External References

- [ArduPilot AP_Mission](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.h) - Mission state machine and dual-slot execution
- [ArduPilot AP_Mission implementation](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Mission/AP_Mission.cpp) - Sequencing logic and command dispatch
- [ArduPilot Rover mode_auto.cpp](https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode_auto.cpp) - Rover-specific mission command execution
- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html) - Protocol specification
- [MAVLink MISSION_CURRENT](https://mavlink.io/en/messages/common.html#MISSION_CURRENT) - Message definition
- [MAVLink MISSION_ITEM_REACHED](https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED) - Message definition
