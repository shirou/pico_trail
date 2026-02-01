# FR-00124 SmartRTL Path Simplification

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00123-smartrtl-path-recording](FR-00123-smartrtl-path-recording.md)
- Dependent Requirements:
  - [FR-00125-smartrtl-return-navigation](FR-00125-smartrtl-return-navigation.md)
- Related Analyses:
  - [AN-00036-rtl-mode](../analysis/AN-00036-rtl-mode.md)
- Related Tasks:
  - [T-00030-rtl-smartrtl-implementation](../tasks/T-00030-rtl-smartrtl-implementation/README.md)

## Requirement Statement

The system shall simplify the recorded path by approximating curved segments with straight lines and pruning loops, keeping simplification within SRTL_ACCURACY meters of the original path.

## Rationale

Raw path recording produces many closely-spaced points that consume memory and are unnecessary for navigation. Path simplification:

- Reduces memory usage significantly
- Removes redundant points on straight segments
- Prunes loops where vehicle crossed its own path
- Maintains navigation safety within specified accuracy

## User Story

As an operator, I want the path recording to be memory-efficient, so that longer missions can be recorded without exhausting the path buffer.

## Acceptance Criteria

- [ ] Simplification keeps path within SRTL_ACCURACY meters of original (default: 2m)
- [ ] Straight segments consolidated to single line between endpoints
- [ ] Loops detected and pruned (newer path segment kept)
- [ ] Simplification runs incrementally during recording
- [ ] Original path accuracy preserved where terrain requires it

## Technical Details

### Functional Requirement Details

**Simplification Algorithm (Ramer-Douglas-Peucker variant):**

```rust
fn simplify_path(points: &[PathPoint], accuracy: f32) -> Vec<PathPoint> {
    // Keep endpoints
    // Find point furthest from line between endpoints
    // If distance > accuracy, recursively simplify both halves
    // Otherwise, remove intermediate points
}
```

**Loop Pruning:**

- Detect when current position is within SRTL_ACCURACY of earlier path
- Remove points between loop start and current position
- Keep more recent path segment

**SRTL_ACCURACY Parameter:**

- Range: 0.5 to 10 meters
- Default: 2 meters
- Lower = more points, higher fidelity
- Higher = fewer points, more aggressive simplification

**Incremental Application:**

- Simplification runs after each new point recorded
- Operates on most recent N points (sliding window)
- Prevents CPU spikes from bulk processing

## Platform Considerations

N/A - Platform agnostic (pure geometry calculations)

## Risks & Mitigation

| Risk                           | Impact | Likelihood | Mitigation                     | Validation              |
| ------------------------------ | ------ | ---------- | ------------------------------ | ----------------------- |
| Over-simplification loses path | Medium | Low        | SRTL_ACCURACY limits deviation | Test with complex paths |
| Loop pruning removes shortcuts | Low    | Low        | Keep newer segment (safer)     | Test crossing paths     |
| CPU overhead during recording  | Low    | Low        | Incremental processing         | Profile simplification  |

## Implementation Notes

- Part of `src/subsystems/navigation/path_recorder.rs`
- Consider using fixed-point math for embedded efficiency
- Ramer-Douglas-Peucker is O(n log n) average case
- Loop detection uses spatial indexing or grid-based approach

## External References

- [ArduPilot SRTL_ACCURACY](https://ardupilot.org/rover/docs/parameters.html#srtl-accuracy) - Parameter reference
- [Ramer-Douglas-Peucker Algorithm](https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm) - Simplification algorithm
