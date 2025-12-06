# T-c26bh Unified Waypoint Navigation Design

## Metadata

- Type: Design
- Status: Done

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design document specifies the implementation of unified waypoint navigation using MissionStorage as the single source of truth for both GUIDED and AUTO modes. The architecture eliminates the disconnect between mission waypoints (stored via MISSION_ITEM protocol) and navigation target (previously NAV_TARGET), enabling Mission Planner's standard workflow.

## Success Metrics

- [x] Rover navigates when armed in GUIDED mode with uploaded waypoint
- [x] AUTO mode navigates through sequential waypoints
- [x] MissionState correctly tracks Idle/Running/Completed states
- [x] SET_POSITION_TARGET integrates with MissionStorage
- [x] All existing tests pass (no regressions)

## Background and Current State

- Context: Mission Planner uploads waypoints via MISSION_ITEM protocol, but navigation reads from NAV_TARGET
- Current behavior: Rover does not move when armed in GUIDED mode with mission set
- Pain points:
  - MISSION_ITEM waypoints stored in MissionStorage are never read by navigation
  - SET_POSITION_TARGET updates NAV_TARGET, creating two separate data sources
  - No mission execution state management
- Constraints: Must maintain backward compatibility with SET_POSITION_TARGET workflow
- Related ADRs: [ADR-2hs12-unified-waypoint-navigation](../../adr/ADR-2hs12-unified-waypoint-navigation.md)

## Proposed Design

### High-Level Architecture

```text
                     +------------------+
                     |  MissionStorage  |
                     |  (unified source)|
                     +--------+---------+
                              |
              +---------------+---------------+
              |                               |
    +---------v---------+           +---------v---------+
    |   MISSION_ITEM    |           | SET_POSITION_     |
    |   Protocol        |           | TARGET_GLOBAL_INT |
    +-------------------+           +-------------------+
              |                               |
              +---------------+---------------+
                              |
                     +--------v---------+
                     |  navigation_task |
                     +--------+---------+
                              |
              +---------------+---------------+
              |                               |
    +---------v---------+           +---------v---------+
    |   GUIDED Mode     |           |    AUTO Mode      |
    |   (single WP)     |           |   (sequential)    |
    +-------------------+           +-------------------+
```

### Components

#### MissionState Enum

```rust
/// Mission execution state
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MissionState {
    /// No mission active, navigation idle
    #[default]
    Idle,
    /// Mission running, navigating to waypoints
    Running,
    /// Mission completed, all waypoints reached
    Completed,
}
```

#### Global State

```rust
/// Global mission state (accessible from navigation and mode logic)
#[cfg(feature = "embassy")]
pub static MISSION_STATE: Mutex<CriticalSectionRawMutex, MissionState> =
    Mutex::new(MissionState::Idle);

/// Global mission storage (replaces NAV_TARGET as unified source)
#[cfg(feature = "embassy")]
pub static MISSION_STORAGE: Mutex<CriticalSectionRawMutex, MissionStorage> =
    Mutex::new(MissionStorage::new_const());
```

#### Module Structure Changes

```text
src/core/mission/
├── mod.rs              # MissionStorage (existing) + MissionState (new)
└── state.rs            # MissionState enum and global state (new)

src/subsystems/navigation/
├── mod.rs              # Remove NAV_TARGET, add MISSION_STORAGE reference
├── controller.rs       # Unchanged
├── geo.rs              # Unchanged
└── types.rs            # Unchanged
```

### Data Flow

#### GUIDED Mode Flow

1. User uploads waypoint via MISSION_ITEM protocol
2. Waypoint stored in MISSION_STORAGE
3. User arms vehicle in GUIDED mode
4. Mode change handler detects ARM in GUIDED:
   - If waypoint exists in MISSION_STORAGE: Set MissionState::Running
   - navigation_task starts navigating to current waypoint
5. On waypoint arrival: MissionState::Completed, hold position

#### AUTO Mode Flow

1. User uploads mission waypoints via MISSION_ITEM protocol
2. Waypoints stored in MISSION_STORAGE
3. User sends MAV_CMD_MISSION_START command
4. Command handler:
   - Verifies mission not empty
   - Sets current_index to 0
   - Sets MissionState::Running
5. navigation_task:
   - Reads current waypoint from MISSION_STORAGE
   - Navigates to waypoint
   - On arrival: advance current_index
   - If more waypoints: continue navigation
   - If last waypoint: MissionState::Completed, transition to Hold mode

#### SET_POSITION_TARGET Flow

1. GCS sends SET_POSITION_TARGET_GLOBAL_INT
2. Handler:
   - Creates Waypoint from position data
   - Clears MISSION_STORAGE
   - Adds single waypoint to MISSION_STORAGE
   - Sets MissionState::Running (if in GUIDED mode)
3. navigation_task picks up new target

### Data Models and Types

#### Waypoint to PositionTarget Conversion

```rust
impl From<&Waypoint> for PositionTarget {
    fn from(wp: &Waypoint) -> Self {
        Self {
            latitude: (wp.x as f32) / 1e7,
            longitude: (wp.y as f32) / 1e7,
            altitude: Some(wp.z),
        }
    }
}
```

#### Mode Behavior Summary

| Mode   | Trigger       | Behavior                          | Waypoint Advance |
| ------ | ------------- | --------------------------------- | ---------------- |
| GUIDED | ARM           | Navigate to current waypoint      | No               |
| GUIDED | SET_POSITION  | Update current waypoint, navigate | No               |
| AUTO   | MISSION_START | Navigate through all waypoints    | Yes              |

### Error Handling

- Empty mission on ARM in GUIDED: Remain in Idle state, no navigation
- Empty mission on MISSION_START: Return MAV_RESULT_FAILED
- Invalid waypoint index: Reset to index 0 or Completed state
- GPS unavailable: Navigation controller handles gracefully (already implemented)

### Security Considerations

- No new external input vectors
- MissionStorage size limited to MAX_WAYPOINTS (50)
- All inputs validated through existing MAVLink handlers

### Performance Considerations

- MissionStorage access via async Mutex (minimal overhead)
- No heap allocations in hot path
- Waypoint lookup is O(1) array access

### Platform Considerations

#### Embedded (RP2350)

- Uses embassy-sync Mutex with CriticalSectionRawMutex
- No_std compatible
- Const initialization for static globals

#### Host Tests

- Feature-gated globals for test isolation
- Mock implementations for Mutex when needed

## Alternatives Considered

1. **Keep NAV_TARGET alongside MissionStorage**
   - Pros: Minimal changes
   - Cons: Two sources of truth, inconsistent behavior

2. **Separate Mission Executor module**
   - Pros: Clean separation
   - Cons: More abstraction, overkill for current needs

### Decision Rationale

Using MissionStorage as the unified source provides the simplest path to Mission Planner compatibility while maintaining clear behavior across modes. The architecture allows future migration to a separate executor if needed.

## Migration and Compatibility

- NAV_TARGET will be deprecated (warnings for direct usage)
- Existing SET_POSITION_TARGET workflow preserved via MissionStorage integration
- No changes to external MAVLink protocol behavior

## Testing Strategy

### Unit Tests

```rust
#[test]
fn test_mission_state_transitions() {
    // Idle -> Running on ARM with waypoint
    // Running -> Completed on arrival
    // Completed -> Idle on new mission upload
}

#[test]
fn test_waypoint_to_position_target() {
    let wp = Waypoint {
        x: 357000000, // 35.7 degrees
        y: 1396000000, // 139.6 degrees
        z: 100.0,
        ..Default::default()
    };
    let target = PositionTarget::from(&wp);
    assert!((target.latitude - 35.7).abs() < 0.0001);
    assert!((target.longitude - 139.6).abs() < 0.0001);
}

#[test]
fn test_auto_mode_waypoint_advance() {
    // Upload 3 waypoints
    // Start mission
    // Verify navigation to each in sequence
    // Verify Completed state after last
}
```

### Integration Tests

- MISSION_ITEM upload followed by ARM in GUIDED mode
- MAV_CMD_MISSION_START followed by waypoint navigation
- SET_POSITION_TARGET updates MissionStorage correctly

## Documentation Impact

- Update `docs/architecture.md` with unified navigation architecture
- Document MissionState in API documentation

## External References

- [ArduPilot Rover AUTO mode](https://ardupilot.org/rover/docs/auto-mode.html)
- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html)
- [MAV_CMD_MISSION_START](https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START)

## Open Questions

- [x] Should GUIDED mode use mission waypoints? -> Yes, unified source (per ADR)
- [x] What triggers mission start in AUTO mode? -> MAV_CMD_MISSION_START (per ADR)
- [ ] Should waypoint arrival trigger MISSION_ITEM_REACHED message? -> Defer to implementation phase

## Appendix

### MissionStorage API Summary

| Method                 | Description                  |
| ---------------------- | ---------------------------- |
| `new()`                | Create empty storage         |
| `count()`              | Get waypoint count           |
| `is_empty()`           | Check if empty               |
| `clear()`              | Remove all waypoints         |
| `add_waypoint(wp)`     | Add waypoint                 |
| `get_waypoint(i)`      | Get waypoint by index        |
| `current_index()`      | Get current navigation index |
| `set_current_index(i)` | Set current navigation index |
| `current_waypoint()`   | Get current target waypoint  |

### Glossary

- **MissionStorage**: Fixed-size array storing mission waypoints
- **MissionState**: Enum tracking mission execution status (Idle/Running/Completed)
- **NAV_TARGET**: Deprecated global for navigation target (replaced by MissionStorage)
- **WP_RADIUS**: Waypoint acceptance radius (ArduPilot parameter, default 2.0m)

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
