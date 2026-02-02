# NFR-00093 Arc-Pivot Transition Continuity

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00045-configurable-turn-behavior](../analysis/AN-00045-configurable-turn-behavior.md)
- Prerequisite Requirements:
  - [FR-00144-configurable-pivot-turn-threshold](FR-00144-configurable-pivot-turn-threshold.md)
  - [FR-00145-arc-turn-minimum-throttle](FR-00145-arc-turn-minimum-throttle.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00042-configurable-turn-behavior](../tasks/T-00042-configurable-turn-behavior/README.md)

## Requirement Statement

The transition between arc turn and pivot turn behavior shall be continuous, with no abrupt throttle discontinuity at the `pivot_turn_angle` boundary.

## Rationale

Throttle discontinuities at the pivot angle boundary would cause jerky vehicle behavior — the rover would experience a sudden speed change as heading error crosses the threshold. This is undesirable for all navigation modes (Guided, Auto, RTL, SmartRTL, Loiter, Circle) and could interfere with smooth path following.

The default configuration (`pivot_turn_angle=60°`, `throttle_heading_error_max=90°`) naturally produces continuity because the throttle floor (0.15) is below the natural throttle at the boundary (0.333). This NFR ensures the design maintains this property.

## User Story (if applicable)

As a rover operator, I want smooth transitions between arc turns and pivot turns, so that the rover does not jerk or change speed abruptly when crossing the pivot angle threshold.

## Acceptance Criteria

- [ ] Throttle values at heading error just below and just above `pivot_turn_angle` differ by < 0.1
- [ ] Default configuration (`pivot_turn_angle=60°`, `throttle_heading_error_max=90°`) produces natural throttle \~0.33 at boundary, above the 0.15 floor — zero discontinuity
- [ ] Unit test verifies continuity at the boundary with default configuration
- [ ] Unit test verifies continuity with at least one non-default configuration

## Technical Details (if applicable)

### Non-Functional Requirement Details

- **Stability**: Continuity at boundary
  - The throttle floor (FR-00145) only activates when natural throttle drops below `arc_turn_min_throttle`
  - At the `pivot_turn_angle` boundary with defaults: natural throttle = `1.0 - (60/90) = 0.333`
  - Since `0.333 > 0.15`, the floor does not activate at the boundary → zero discontinuity
  - Above the boundary: the floor no longer applies, but natural throttle is continuous

**Continuity proof for default configuration:**

```text
At heading_error = 59.9° (just below pivot_turn_angle):
  heading_scale = 1.0 - (59.9 / 90.0) = 0.334
  throttle = max(0.334, 0.15) = 0.334  (floor inactive)

At heading_error = 60.1° (just above pivot_turn_angle):
  heading_scale = 1.0 - (60.1 / 90.0) = 0.332
  throttle = 0.332  (floor disabled, natural curve)

Delta = |0.334 - 0.332| = 0.002 < 0.1 ✓
```

**When discontinuity could occur:**

Discontinuity would only happen if `arc_turn_min_throttle` > natural throttle at the boundary. For example, `pivot_turn_angle=30°` with `arc_turn_min_throttle=0.8`:

- Natural throttle at 30° = `1.0 - (30/90) = 0.667`
- Floor would raise throttle to 0.8 just below boundary
- Just above boundary: throttle = 0.667
- Delta = 0.133 > 0.1 — **violates this NFR**

This is a configuration error, not an implementation bug. Documentation should warn against such combinations.

## Platform Considerations

### Cross-Platform

- Continuity is a mathematical property of the throttle function — platform-independent
- Verified via host unit tests

## Risks & Mitigation

| Risk                                        | Impact | Likelihood | Mitigation                                             | Validation |
| ------------------------------------------- | ------ | ---------- | ------------------------------------------------------ | ---------- |
| Non-default config creates discontinuity    | Low    | Low        | Document safe configuration ranges; validate in tests  | Unit tests |
| Approach zone throttle interacts with floor | Low    | Low        | Floor uses `max()` — approach zone can only lower base | Unit tests |

## Implementation Notes

- This NFR constrains the relationship between `pivot_turn_angle`, `throttle_heading_error_max`, and `arc_turn_min_throttle`
- Safe condition: `arc_turn_min_throttle <= 1.0 - (pivot_turn_angle / throttle_heading_error_max)`
- Default config satisfies this: `0.15 <= 1.0 - (60/90) = 0.333` ✓
- Consider adding a debug assertion or documentation warning for configurations that violate continuity

## External References

- N/A
