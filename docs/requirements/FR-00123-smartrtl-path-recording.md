# FR-00123 SmartRTL Path Recording

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements: None
- Dependent Requirements:
  - [FR-00124-smartrtl-path-simplification](FR-00124-smartrtl-path-simplification.md)
  - [FR-00125-smartrtl-return-navigation](FR-00125-smartrtl-return-navigation.md)
  - [NFR-00086-smartrtl-memory-budget](NFR-00086-smartrtl-memory-budget.md)
- Related Analyses:
  - [AN-00036-rtl-mode](../analysis/AN-00036-rtl-mode.md)
- Related Tasks:
  - [T-00030-rtl-smartrtl-implementation](../tasks/T-00030-rtl-smartrtl-implementation/README.md)

## Requirement Statement

The system shall continuously record the vehicle's GPS path during armed operation, storing position points in a ring buffer for use by SmartRTL mode when returning to home.

## Rationale

SmartRTL requires a recorded path to retrace the vehicle's route safely. Unlike direct RTL which navigates in a straight line, SmartRTL follows the previously traveled path in reverse, avoiding obstacles the vehicle already navigated around. Path recording must occur automatically during normal operation to ensure SmartRTL is available when needed.

## User Story

As an operator, I want the vehicle to automatically record its path while armed, so that SmartRTL can safely retrace the route home through the same terrain.

## Acceptance Criteria

- [ ] Path recording begins automatically when vehicle is armed
- [ ] GPS positions recorded at configurable interval (default: every 3 seconds or 5 meters)
- [ ] Points stored in ring buffer with SRTL_POINTS capacity (default: 300)
- [ ] STATUSTEXT "SmartRTL low on space" sent when buffer 90% full
- [ ] Path recording pauses when GPS fix is lost
- [ ] Path recording stops when vehicle disarms
- [ ] Buffer clears on arm to start fresh path

## Technical Details

### Functional Requirement Details

**Path Recording Trigger:**

- Start: On arm event with valid GPS
- Stop: On disarm event
- Pause: On GPS fix loss
- Resume: On GPS fix recovery

**Recording Interval:**

Two strategies combined (whichever triggers first):

1. Time-based: Every 3 seconds
2. Distance-based: Every 5 meters from last recorded point

**Ring Buffer Behavior:**

- When buffer full, oldest points are overwritten
- Low-space warning at 90% capacity
- Full buffer does not disable SmartRTL (uses available points)

**Data Structure:**

```rust
pub struct PathPoint {
    pub latitude: f64,   // degrees
    pub longitude: f64,  // degrees
    pub timestamp: u32,  // ms since arm
}
// Size: ~20 bytes per point
```

## Platform Considerations

N/A - Platform agnostic (uses GPS position from SYSTEM_STATE)

## Risks & Mitigation

| Risk                       | Impact | Likelihood | Mitigation                     | Validation                 |
| -------------------------- | ------ | ---------- | ------------------------------ | -------------------------- |
| Buffer overflow overwrites | Low    | Medium     | Ring buffer design is expected | Test with long paths       |
| GPS dropout gaps in path   | Medium | Medium     | Pause recording, interpolate   | Test GPS loss scenarios    |
| Memory exhaustion          | High   | Low        | Fixed buffer size, no alloc    | Verify static memory usage |

## Implementation Notes

- Path recorder: `src/subsystems/navigation/path_recorder.rs`
- Integrates with arming system for start/stop
- Uses SYSTEM_STATE for GPS position access
- Consider: Path compression during recording

## External References

- [ArduPilot SmartRTL Mode](https://ardupilot.org/rover/docs/smartrtl-mode.html) - Reference implementation
