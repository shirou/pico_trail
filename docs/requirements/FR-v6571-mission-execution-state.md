# FR-v6571 Mission Execution State

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-zd6uw-mission-execution](../analysis/AN-zd6uw-mission-execution.md)
- Related ADRs:
  - [ADR-2hs12-unified-waypoint-navigation](../adr/ADR-2hs12-unified-waypoint-navigation.md)
- Prerequisite Requirements: N/A
- Dependent Requirements:
  - [FR-w893v-mission-start-command](FR-w893v-mission-start-command.md)
  - [FR-jm7mj-auto-mode-mission-execution](FR-jm7mj-auto-mode-mission-execution.md)
- Related Tasks:
  - [T-c26bh-unified-waypoint-navigation](../tasks/T-c26bh-unified-waypoint-navigation/README.md)

## Requirement Statement

The system shall maintain mission execution state (Idle, Running, Completed) to track mission progress and control navigation behavior.

## Rationale

Mission execution requires state tracking to:

1. Know whether mission is active (Running) or waiting (Idle)
2. Track progress through waypoints
3. Detect mission completion
4. Report status via telemetry (MISSION_CURRENT, MISSION_ITEM_REACHED)

Without execution state, the system cannot distinguish between "mission loaded but not started" and "mission actively executing".

## User Story (if applicable)

As the navigation system, I need to know the mission execution state, so that I can determine whether to navigate through waypoints or hold position.

## Acceptance Criteria

- [ ] Mission state enum defined: Idle, Running, Completed
- [ ] Global MISSION_STATE accessible via Mutex
- [ ] State transitions: Idle → Running (on MISSION_START)
- [ ] State transitions: Running → Completed (on last waypoint reached)
- [ ] State transitions: Running → Idle (on mission clear/reset)
- [ ] Current waypoint index tracked in state
- [ ] Waypoint advancement (Running only) increments index
- [ ] State accessible for telemetry reporting
- [ ] Thread-safe access via critical_section

## Technical Details (if applicable)

### Functional Requirement Details

**Mission State Enum:**

```rust
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum MissionExecutionState {
    /// Mission loaded but not started
    Idle,
    /// Mission actively executing
    Running,
    /// Mission completed (all waypoints reached)
    Completed,
}
```

**Mission State Structure:**

```rust
pub struct MissionState {
    /// Current execution state
    pub state: MissionExecutionState,
    /// Current waypoint index (0-based)
    pub current_index: usize,
    /// First waypoint in execution range
    pub first_index: usize,
    /// Last waypoint in execution range
    pub last_index: usize,
}

impl MissionState {
    pub fn new() -> Self {
        Self {
            state: MissionExecutionState::Idle,
            current_index: 0,
            first_index: 0,
            last_index: 0,
        }
    }

    pub fn start(&mut self, first: usize, last: usize) {
        self.state = MissionExecutionState::Running;
        self.first_index = first;
        self.last_index = last;
        self.current_index = first;
    }

    pub fn advance(&mut self) -> bool {
        if self.state != MissionExecutionState::Running {
            return false;
        }
        if self.current_index >= self.last_index {
            self.state = MissionExecutionState::Completed;
            return false;
        }
        self.current_index += 1;
        true
    }

    pub fn reset(&mut self) {
        self.state = MissionExecutionState::Idle;
        self.current_index = 0;
    }
}
```

**Global State:**

```rust
#[cfg(feature = "embassy")]
pub static MISSION_STATE: Mutex<CriticalSectionRawMutex, MissionState> =
    Mutex::new(MissionState::new());
```

**State Transition Diagram:**

```
          MISSION_START
    [Idle] ───────────────> [Running]
       ^                        |
       |                        | Last WP reached
       | MISSION_CLEAR          v
       +──────────────────  [Completed]
```

**Telemetry Integration:**

- MISSION_CURRENT: Reports `current_index`
- MISSION_ITEM_REACHED: Sent when waypoint reached before advancing

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                     | Impact | Likelihood | Mitigation                           | Validation                      |
| ------------------------ | ------ | ---------- | ------------------------------------ | ------------------------------- |
| State corruption         | High   | Low        | Use critical_section for all access  | Test concurrent state access    |
| State/index mismatch     | Medium | Low        | Validate index against mission count | Test with mission modifications |
| Missed state transitions | Medium | Medium     | Clear state transition triggers      | Test all transition paths       |

## Implementation Notes

Preferred approaches:

- Add MissionState to `src/core/mission/mod.rs`
- Follow existing global state patterns (NAV_TARGET, NAV_OUTPUT)
- Consider integrating with existing MissionStorage

Known pitfalls:

- MissionStorage already has `current_index` field - consider unifying
- State must persist across mode changes
- Mission clear should reset state to Idle

Related code areas:

- `src/core/mission/mod.rs` - MissionStorage
- `src/communication/mavlink/handlers/command.rs` - MISSION_START handler
- `src/rover/mode/` - AUTO mode implementation
- `examples/pico_trail_rover.rs` - navigation_task

## External References

- [MAVLink Mission Protocol](https://mavlink.io/en/services/mission.html)
