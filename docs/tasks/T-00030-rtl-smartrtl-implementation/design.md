# T-00030 RTL and SmartRTL Mode | Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design specifies the implementation of RTL (Return to Launch) and SmartRTL modes for the pico_trail rover. SmartRTL provides safe navigation by retracing the recorded path in reverse, while direct RTL offers straight-line navigation to home as a fallback. The implementation leverages the existing Mode trait, SimpleNavigationController, and navigation infrastructure.

## Success Metrics

- [ ] SmartRTL successfully retraces recorded path to home
- [ ] Direct RTL navigates to home when path unavailable
- [ ] Path recording uses < 10 KB RAM (300 points default)
- [ ] Navigation updates complete within 1ms at 50 Hz
- [ ] GPS loss triggers transition to Hold mode

## Background and Current State

- Context: RTL is a critical safety mode for autonomous vehicle recovery
- Current behavior: `FlightMode::Rtl` exists as enum but has no implementation
- Pain points: No autonomous return capability, failsafe cannot trigger RTL
- Constraints: Limited RAM (\~264 KB on RP2350), no_std environment
- Related ADRs: [ADR-00031-rtl-smartrtl-architecture](../../adr/ADR-00031-rtl-smartrtl-architecture.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                    Mode Manager                              │
│              (selects RTL or SmartRTL)                       │
└──────────────────────┬──────────────────────────────────────┘
                       │
        ┌──────────────┴──────────────┐
        │                             │
┌───────▼───────┐           ┌────────▼────────┐
│  SmartRtlMode │           │    RtlMode      │
│  (default)    │           │   (fallback)    │
│               │           │                 │
│ - waypoint_idx│           │ - target        │
│ - total_wps   │           │ - arrived       │
└───────┬───────┘           └────────┬────────┘
        │                            │
        └────────────┬───────────────┘
                     │
        ┌────────────▼────────────────┐
        │  SimpleNavigationController  │
        │  (steering/throttle calc)    │
        └────────────┬────────────────┘
                     │
        ┌────────────▼────────────────┐
        │       PathRecorder           │
        │  - ring buffer (300 pts)     │
        │  - record on arm             │
        │  - simplify path             │
        └─────────────────────────────┘
```

### Components

**PathRecorder** (`src/subsystems/navigation/path_recorder.rs`):

- Ring buffer storing PathPoint (lat, lon, timestamp)
- Recording starts on arm, stops on disarm
- Records point when distance > 5m OR time > 3s from last
- Path simplification using SRTL_ACCURACY parameter
- Thread-safe access via Mutex

**RtlMode** (`src/rover/mode/rtl.rs`):

- Direct navigation to home position
- Uses SimpleNavigationController
- Entry validation (GPS fix, home position)
- Arrival detection (within WP_RADIUS)

**SmartRtlMode** (`src/rover/mode/smartrtl.rs`):

- Follows recorded path in reverse order
- Waypoint-by-waypoint navigation
- Falls back to direct RTL if path empty
- Uses SimpleNavigationController for each segment

### Data Flow

1. **Recording Phase** (while armed):
   - GPS position received → PathRecorder.record()
   - Check distance/time threshold
   - Store in ring buffer if threshold met

2. **Return Phase** (RTL activated):
   - Check path availability
   - SmartRTL: Load waypoints in reverse, navigate sequentially
   - Direct RTL: Navigate directly to home
   - On GPS loss: Transition to Hold mode

### Data Models and Types

```rust
/// Single point in recorded path
pub struct PathPoint {
    pub latitude: f64,    // degrees
    pub longitude: f64,   // degrees
    pub timestamp_ms: u32, // ms since arm
}
// Size: ~20 bytes per point

/// Path recorder constants
const SRTL_POINTS_MAX: usize = 300;      // Max points in buffer (~9 KB)
const SRTL_MIN_DISTANCE_M: f32 = 5.0;    // Min distance between points
const SRTL_MIN_TIME_MS: u32 = 3000;      // Min time between points
const SRTL_ACCURACY_M: f32 = 2.0;        // Path simplification accuracy

/// Path recorder state
pub struct PathRecorder {
    buffer: [PathPoint; SRTL_POINTS_MAX],
    count: usize,
    write_idx: usize,
    last_point: Option<PathPoint>,
    recording: bool,
}
```

### Error Handling

- RTL entry rejection: Return descriptive error ("RTL requires GPS fix")
- GPS loss during RTL: Log warning, transition to Hold mode
- Path buffer full: Continue recording (ring buffer overwrites oldest)
- Home position not set: Reject RTL entry with error

### Security Considerations

- N/A - Internal navigation system, no external inputs beyond GPS

### Performance Considerations

- **Hot paths**: Navigation update at 50 Hz must complete within 1ms
- **Memory**: Fixed-size buffer, no heap allocation
- **Path recording**: Amortized over 3-second intervals, minimal overhead
- **Distance calculation**: Haversine formula, \~10 us per calculation

### Platform Considerations

#### Embedded (RP2350)

- Static allocation for path buffer
- No heap allocation during runtime
- Mutex for thread-safe path access

#### Host Tests

- Mock GPS position for testing
- Verify path recording logic
- Test mode transitions

## Alternatives Considered

1. **Direct RTL Only**
   - Pros: Simple, minimal memory
   - Cons: Unsafe in obstacle environments
2. **SmartRTL Only**
   - Pros: Always safe path
   - Cons: Cannot return without recorded path

Decision Rationale: Dual-mode approach provides safety (SmartRTL) with reliability (fallback to direct RTL).

## Migration and Compatibility

- Backward compatibility: N/A (new feature)
- ArduPilot compatibility: Same behavior and parameters

## Testing Strategy

### Unit Tests

- PathRecorder: record, simplification, ring buffer overflow
- RtlMode: entry validation, navigation, arrival detection
- SmartRtlMode: waypoint progression, fallback to direct RTL

### Integration Tests

- RTL mode selection based on path availability
- GPS loss handling during RTL
- End-to-end navigation to home

## Documentation Impact

- Update docs/mavlink.md with RTL mode support
- Add RTL parameters to docs/parameters.md

## External References

- [ArduPilot Rover RTL Mode](https://ardupilot.org/rover/docs/rtl-mode.html)
- [ArduPilot SmartRTL Mode](https://ardupilot.org/rover/docs/smartrtl-mode.html)

## Open Questions

- [x] SmartRTL or direct RTL as default? → SmartRTL default, direct RTL fallback
- [ ] Path simplification algorithm details → Implement basic distance-based simplification first

## Appendix

### Diagrams

```text
State Machine: SmartRTL Mode

    ┌─────────┐
    │  IDLE   │
    └────┬────┘
         │ enter()
    ┌────▼────┐
    │ LOADING │ Load waypoints from PathRecorder
    └────┬────┘
         │
    ┌────▼────────┐
    │ NAVIGATING  │◄───────────────┐
    │ to waypoint │                │
    └────┬────────┘                │
         │ at_target              │
    ┌────▼────┐                    │
    │ ADVANCE │ next waypoint ─────┘
    └────┬────┘
         │ no more waypoints
    ┌────▼────┐
    │ ARRIVED │ stop at home
    └─────────┘
```

### Examples

```rust
// RTL mode selection
let mode = if SmartRtlMode::can_enter().is_ok() {
    Box::new(SmartRtlMode::new()) as Box<dyn Mode>
} else if RtlMode::can_enter().is_ok() {
    Box::new(RtlMode::new()) as Box<dyn Mode>
} else {
    return Err("Cannot enter RTL: no GPS or home");
};
```
