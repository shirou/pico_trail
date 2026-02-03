# FR-00144 Configurable Pivot Turn Threshold

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00045-configurable-turn-behavior](../analysis/AN-00045-configurable-turn-behavior.md)
- Prerequisite Requirements:
  - [FR-00084-navigation-controller](FR-00084-navigation-controller.md)
- Dependent Requirements:
  - [FR-00145-arc-turn-minimum-throttle](FR-00145-arc-turn-minimum-throttle.md)
  - [FR-00146-navigation-parameter-store](FR-00146-navigation-parameter-store.md)
  - [NFR-00093-arc-pivot-transition-continuity](NFR-00093-arc-pivot-transition-continuity.md)
- Related ADRs:
  - N/A – To be created during task design
- Related Tasks:
  - [T-00042-configurable-turn-behavior](../tasks/T-00042-configurable-turn-behavior/README.md)

## Requirement Statement

The navigation controller shall support a configurable heading error threshold (`pivot_turn_angle`) above which pivot turns are allowed. Below this threshold, forward throttle is maintained for arc turns. This applies to all modes using `SimpleNavigationController` (Guided, Auto, RTL, SmartRTL, Loiter, Circle).

## Rationale

The current `calculate_throttle()` linearly reduces throttle to 0 at `throttle_heading_error_max` (90°). When throttle reaches 0, `DifferentialDrive::mix(steering, 0)` produces opposite wheel rotation — a pivot turn. Pivot turns are appropriate for large heading errors (target behind the rover) but undesirable for moderate corrections (30°–60°) where maintaining forward motion with arc turns is more efficient.

ArduPilot addresses this with `WP_PIVOT_ANGLE` (default 60°), which controls when pivot turns are permitted. This requirement brings equivalent configurability to pico_trail's navigation controller.

## User Story (if applicable)

As a rover operator, I want to configure the heading error threshold at which pivot turns are allowed, so that the rover uses efficient arc turns for moderate corrections and only pivots for large heading errors.

## Acceptance Criteria

- [ ] `pivot_turn_angle` field added to `SimpleNavConfig` (type `f32`, units: degrees)
- [ ] Default value is 60.0°, matching ArduPilot `WP_PIVOT_ANGLE`
- [ ] Heading error < `pivot_turn_angle` → rover maintains forward throttle (arc turn)
- [ ] Heading error >= `pivot_turn_angle` → throttle can reach 0 (pivot turn allowed)
- [ ] Setting `pivot_turn_angle=0.0` preserves current behavior (always allow pivots)
- [ ] Setting `pivot_turn_angle=180.0` prevents all pivot turns
- [ ] Behavior is consistent across all autonomous modes (Guided, Auto, RTL, SmartRTL, Loiter, Circle)
- [ ] `WP_PIVOT_ANGLE` registered in parameter store with default 60.0°
- [ ] Value configurable at runtime via MAVLink PARAM_SET
- [ ] Unit tests verify threshold boundary behavior

## Technical Details (if applicable)

### Functional Requirement Details

**Implementation location:** `crates/core/src/navigation/controller.rs` — `calculate_throttle()`

The pivot turn angle acts as a boundary: when heading error is below this angle and the rover is not at the target (`distance > wp_radius`), the throttle floor from FR-00145 is applied instead of allowing throttle to drop to 0.

```rust
// In calculate_throttle(), after computing distance_throttle * heading_scale:
if heading_error_abs < self.config.pivot_turn_angle
    && distance > self.config.wp_radius
{
    throttle = throttle.max(self.config.arc_turn_min_throttle);
}
```

**Config field:**

```rust
/// Heading error threshold for pivot turns (degrees, ArduPilot: WP_PIVOT_ANGLE)
/// Below this angle, forward throttle is maintained for arc turns.
/// Set to 0 to always allow pivot turns (current behavior).
/// Set to 180 to never allow pivot turns.
pub pivot_turn_angle: f32,
```

**Edge cases:**

| `pivot_turn_angle` | Behavior                               |
| ------------------ | -------------------------------------- |
| 0.0                | Floor never applies → current behavior |
| 60.0 (default)     | Arc turns below 60°, pivots above      |
| 180.0              | Floor always applies → no pivot turns  |

## Platform Considerations

### Cross-Platform

- `SimpleNavConfig` is in `pico_trail_core` — no platform dependency
- All unit tests run on host via `cargo test --lib`

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                            | Validation            |
| ----------------------------------------- | ------ | ---------- | ----------------------------------------------------- | --------------------- |
| Default 60° too aggressive for some modes | Low    | Low        | Configurable per deployment; 0° restores old behavior | Field testing         |
| Loiter/Circle modes need different angles | Low    | Low        | Modes could override config if needed in future       | Mode-specific testing |

## Implementation Notes

- The `pivot_turn_angle` field only controls when the throttle floor activates; it does not change the steering calculation
- The actual throttle floor value is defined by FR-00145 (`arc_turn_min_throttle`)
- This requirement depends on FR-00084 (navigation controller exists) as a prerequisite

## External References

- [ArduPilot WP_PIVOT_ANGLE](https://ardupilot.org/rover/docs/parameters.html#wp-pivot-angle) — Standard parameter reference
