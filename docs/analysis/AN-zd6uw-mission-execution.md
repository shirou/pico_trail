# AN-zd6uw Mission Execution

## Metadata

- Type: Analysis
- Status: Approved

## Links

- Related Analyses:
  - N/A - Initial analysis for this feature
- Related Requirements:
  - [FR-m2c9z-mission-waypoint-navigation](../requirements/FR-m2c9z-mission-waypoint-navigation.md)
  - [FR-pbuh7-guided-mode-arm-start](../requirements/FR-pbuh7-guided-mode-arm-start.md)
  - [FR-w893v-mission-start-command](../requirements/FR-w893v-mission-start-command.md)
  - [FR-v6571-mission-execution-state](../requirements/FR-v6571-mission-execution-state.md)
  - [FR-evl5k-position-target-mission-integration](../requirements/FR-evl5k-position-target-mission-integration.md)
  - [NFR-iuk5h-navigation-update-rate](../requirements/NFR-iuk5h-navigation-update-rate.md)
- Related ADRs:
  - [ADR-2hs12-unified-waypoint-navigation](../adr/ADR-2hs12-unified-waypoint-navigation.md)
- Related Tasks:
  - [T-c26bh-unified-waypoint-navigation](../tasks/T-c26bh-unified-waypoint-navigation/README.md)

## Executive Summary

This analysis investigates why the rover does not move when armed in GUIDED mode with a mission set. The investigation reveals that mission waypoints stored via MISSION_ITEM protocol are not connected to the navigation system.

**Key Finding**: Mission Planner does not send SET_POSITION_TARGET - it uploads waypoints via MISSION_ITEM protocol. Therefore, both GUIDED and AUTO modes should use waypoints from MissionStorage:

- **GUIDED mode**: Navigate to current waypoint, start immediately on ARM
- **AUTO mode**: Navigate through all waypoints sequentially, start on MAV_CMD_MISSION_START
- **SET_POSITION_TARGET**: Treated as a single waypoint added to mission

## Problem Space

### Current State

1. **Mission Upload Works**: MISSION_ITEM/MISSION_ITEM_INT messages are correctly received and stored in `MissionStorage` (`src/core/mission/mod.rs`)

2. **Navigation System Exists**: `navigation_task` computes steering/throttle from GPS position to target, updating `NAV_OUTPUT`

3. **Motor Control Works**: `motor_control_task` reads `NAV_OUTPUT` in GUIDED mode and applies differential drive

4. **Disconnection**: Mission waypoints in `MissionStorage` are never read by `navigation_task`:
   - `NAV_TARGET` is only set by `SET_POSITION_TARGET_GLOBAL_INT` or `MAV_CMD_DO_REPOSITION`
   - `navigation_task` only runs in GUIDED mode
   - AUTO mode falls back to RC input (line 740-747 in pico_trail_rover.rs)

### Desired State

1. **GUIDED Mode**: When ARM occurs with waypoint set:
   - Read current waypoint from MissionStorage
   - Navigate to that single waypoint immediately
   - Stop when waypoint is reached (do not advance)

2. **AUTO Mode**: When MAV_CMD_MISSION_START is received:
   - Read waypoints from MissionStorage
   - Navigate sequentially through waypoints
   - Advance to next waypoint when current is reached
   - Handle mission completion

3. **SET_POSITION_TARGET Integration**:
   - When SET_POSITION_TARGET_GLOBAL_INT is received, add as waypoint to MissionStorage
   - Set as current waypoint (index 0 or append)

### Gap Analysis

| Component                | Current          | Desired                       | Gap            |
| ------------------------ | ---------------- | ----------------------------- | -------------- |
| MissionStorage           | Stores waypoints | Same                          | None           |
| MAV_CMD_MISSION_START    | Not implemented  | Starts AUTO mission execution | Handler needed |
| Mission execution state  | None             | Idle/Running/Completed        | State machine  |
| navigation_task (AUTO)   | Falls back to RC | Reads from MissionStorage     | Logic needed   |
| navigation_task (GUIDED) | Reads NAV_TARGET | Reads from MissionStorage     | Logic change   |
| SET_POSITION_TARGET      | Sets NAV_TARGET  | Adds waypoint to mission      | Handler change |
| Waypoint advancement     | None             | Auto-advance (AUTO only)      | Logic needed   |

## Stakeholder Analysis

| Stakeholder  | Interest/Need                                 | Impact | Priority |
| ------------ | --------------------------------------------- | ------ | -------- |
| GCS Operator | Execute uploaded missions via Mission Planner | High   | P0       |
| Developer    | Clean separation of AUTO/GUIDED mode behavior | Medium | P1       |
| Safety       | Predictable behavior when armed               | High   | P0       |

## Research & Discovery

### User Feedback

User reported:

- MISSION_ITEM received in GUIDED mode
- Mission set successfully
- Vehicle armed
- Motors do not move (`motor set_speed = 0` output continues)

### Technical Investigation

**Code Analysis Results:**

1. **MissionStorage** (`src/core/mission/mod.rs:124-134`):
   - Stores up to 50 waypoints
   - Has `current_index` field (unused)
   - No execution state tracking

2. **navigation_task** (`examples/pico_trail_rover.rs:829-908`):
   - Only runs when `mode == FlightMode::Guided` (line 865)
   - Reads from `NAV_TARGET` (set by SET_POSITION_TARGET or DO_REPOSITION)
   - Does not read from MissionStorage

3. **motor_control_task** (`examples/pico_trail_rover.rs:691-795`):
   - GUIDED: reads NAV_OUTPUT
   - AUTO: falls back to RC input (not implemented)

4. **CommandHandler** (`src/communication/mavlink/handlers/command.rs`):
   - No MAV_CMD_MISSION_START handler

### Competitive Analysis

ArduPilot Rover behavior:

- **AUTO mode**: Requires MAV_CMD_MISSION_START to begin mission execution
- **GUIDED mode**: Navigates to SET_POSITION_TARGET immediately when armed
- Clear separation between mission-based (AUTO) and single-target (GUIDED) navigation

**Deviation from ArduPilot**: This implementation uses MissionStorage as the unified waypoint source for both modes. This aligns with Mission Planner's behavior of uploading waypoints rather than sending SET_POSITION_TARGET commands.

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: Unified waypoint navigation from MissionStorage
  - Rationale: Both GUIDED and AUTO modes need to read waypoints from MissionStorage
  - Acceptance Criteria:
    - navigation_task reads current waypoint from MissionStorage
    - Waypoint position converted to navigation target
    - Navigation output (steering/throttle) computed from GPS to waypoint

- [ ] **FR-DRAFT-2**: GUIDED mode single waypoint navigation
  - Rationale: GUIDED mode navigates to current waypoint on ARM
  - Acceptance Criteria:
    - ARM + GUIDED + waypoint set = immediate navigation
    - Navigate to current waypoint only (no advancement)
    - Stop when waypoint reached

- [ ] **FR-DRAFT-3**: AUTO mode sequential waypoint execution
  - Rationale: AUTO mode executes full mission
  - Acceptance Criteria:
    - MAV_CMD_MISSION_START triggers mission execution
    - Rover navigates to each waypoint in sequence
    - Rover advances to next waypoint when current is reached
    - Mission completion handled (Hold mode)

- [ ] **FR-DRAFT-4**: Mission execution state management
  - Rationale: System needs to track mission progress
  - Acceptance Criteria:
    - Mission state (Idle/Running/Completed) is tracked
    - Current waypoint index is maintained
    - State is accessible for telemetry reporting

- [ ] **FR-DRAFT-5**: SET_POSITION_TARGET waypoint integration
  - Rationale: SET_POSITION_TARGET should add waypoint to mission
  - Acceptance Criteria:
    - SET_POSITION_TARGET_GLOBAL_INT adds waypoint to MissionStorage
    - Waypoint set as current target
    - Compatible with both GUIDED and AUTO modes

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Navigation update rate
  - Category: Performance
  - Rationale: Smooth navigation requires consistent update rate
  - Target: 50Hz navigation update (20ms interval)

## Design Considerations

### Technical Constraints

1. **No heap allocation**: MissionStorage uses heapless Vec
2. **Embassy async**: All tasks use Embassy executor
3. **Global state via Mutex**: Follows existing NAV_TARGET/NAV_OUTPUT pattern
4. **ArduPilot compatibility**: Follow ArduPilot MAVLink conventions

### Potential Approaches

1. **Option A**: Minimal changes - Add AUTO mode to navigation_task
   - Pros: Minimal code changes, reuses existing navigation
   - Cons: Mission state management mixed with navigation
   - Effort: Low

2. **Option B**: Mission executor module
   - Pros: Clean separation of concerns, testable
   - Cons: More code, new module
   - Effort: Medium

3. **Option C**: Full ArduPilot-style mission manager
   - Pros: Complete feature set, familiar to ArduPilot users
   - Cons: Significant complexity, overkill for current needs
   - Effort: High

**Recommendation**: Option A for initial implementation, with clean interfaces allowing future expansion to Option B.

### Architecture Impact

- No new ADR required for basic implementation
- May need ADR if mission manager becomes complex
- Consider ADR for waypoint acceptance radius and timing parameters

## Risk Assessment

| Risk                                    | Probability | Impact | Mitigation Strategy                   |
| --------------------------------------- | ----------- | ------ | ------------------------------------- |
| GPS accuracy affects waypoint detection | Medium      | Medium | Use configurable acceptance radius    |
| Mission state corruption                | Low         | High   | Use critical_section for state access |
| Mode switching during mission           | Medium      | Medium | Define clear behavior (pause/abort)   |
| No GPS fix during mission               | Medium      | High   | Output zero speed, don't advance      |

## Open Questions

- [x] Should GUIDED mode also use mission waypoints? → **Yes, both modes use MissionStorage**
- [x] What triggers mission start in AUTO mode? → **MAV_CMD_MISSION_START**
- [x] What triggers movement in GUIDED mode? → **ARM (immediate start)**
- [x] How does SET_POSITION_TARGET work? → **Adds waypoint to MissionStorage**
- [x] What happens when mission completes? → **Hold mode**
- [ ] Should mission support pause/resume? → Method: Defer to future enhancement
- [ ] What is the waypoint acceptance radius? → Method: Check ArduPilot defaults (WP_RADIUS)

## Recommendations

### Immediate Actions

1. Create formal requirements from FR-DRAFT-1 through FR-DRAFT-5
2. Create ADR for unified waypoint navigation architecture

### Next Steps

1. [ ] Create formal requirements: FR-<id>-waypoint-navigation, FR-<id>-guided-mode, FR-<id>-auto-mode
2. [ ] Draft ADR for unified waypoint source design
3. [ ] Create task for implementation
4. [ ] Remove NAV_TARGET global (replaced by MissionStorage)

### Out of Scope

- Mission pause/resume (future enhancement)
- Geofence integration (separate feature)
- Terrain following (requires additional sensors)
- Complex mission commands (DO_CHANGE_SPEED, etc.)

## Appendix

### References

- ArduPilot Rover AUTO mode: <https://ardupilot.org/rover/docs/auto-mode.html>
- MAVLink Mission Protocol: <https://mavlink.io/en/services/mission.html>
- MAV_CMD_MISSION_START: <https://mavlink.io/en/messages/common.html#MAV_CMD_MISSION_START>

### Code Locations

| File                                          | Line    | Description                           |
| --------------------------------------------- | ------- | ------------------------------------- |
| src/core/mission/mod.rs                       | 124-134 | MissionStorage struct                 |
| src/communication/mavlink/handlers/mission.rs | -       | Mission protocol handler              |
| src/communication/mavlink/handlers/command.rs | -       | Command handler (needs MISSION_START) |
| examples/pico_trail_rover.rs                  | 829-908 | navigation_task                       |
| examples/pico_trail_rover.rs                  | 691-795 | motor_control_task                    |
| src/subsystems/navigation/mod.rs              | 58-78   | NAV_TARGET, NAV_OUTPUT globals        |
