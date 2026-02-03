# FR-00145 Arc Turn Minimum Throttle

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00045-configurable-turn-behavior](../analysis/AN-00045-configurable-turn-behavior.md)
- Prerequisite Requirements:
  - [FR-00084-navigation-controller](FR-00084-navigation-controller.md)
  - [FR-00144-configurable-pivot-turn-threshold](FR-00144-configurable-pivot-turn-threshold.md)
- Dependent Requirements:
  - [FR-00146-navigation-parameter-store](FR-00146-navigation-parameter-store.md)
  - [NFR-00093-arc-pivot-transition-continuity](NFR-00093-arc-pivot-transition-continuity.md)
- Related ADRs:
  - N/A – To be created during task design
- Related Tasks:
  - [T-00042-configurable-turn-behavior](../tasks/T-00042-configurable-turn-behavior/README.md)

## Requirement Statement

During arc turns (heading error below pivot threshold), the navigation controller shall maintain a configurable minimum forward throttle (`arc_turn_min_throttle`) to ensure the vehicle moves forward while turning. This applies to all modes using `SimpleNavigationController` (Guided, Auto, RTL, SmartRTL, Loiter, Circle).

## Rationale

When `calculate_throttle()` reduces throttle based on heading error, the natural curve can produce very low throttle values even when the heading error is below the pivot threshold. For example, at 75° heading error (below the default 60° pivot angle would not apply, but at higher angles within the arc range), throttle can drop to near-zero, effectively producing a pivot turn despite being in the arc turn range.

A configurable minimum throttle floor ensures forward motion is maintained during arc turns, producing smoother and more efficient paths to waypoints.

## User Story (if applicable)

As a rover operator, I want the rover to maintain a minimum forward speed during arc turns, so that it follows curved paths to waypoints instead of stopping to pivot.

## Acceptance Criteria

- [ ] `arc_turn_min_throttle` field added to `SimpleNavConfig` (type `f32`, range 0.0–1.0)
- [ ] Default value is 0.15
- [ ] Throttle floor applies only when heading error < `pivot_turn_angle` AND distance > `wp_radius`
- [ ] Throttle floor does NOT apply when vehicle is at target (distance <= `wp_radius`)
- [ ] `arc_turn_min_throttle` > `spin_throttle_threshold` (0.15 > 0.1) ensures spin-cap does not activate during arc turns
- [ ] `WP_ARC_THR` registered in parameter store with default 0.15
- [ ] Value configurable at runtime via MAVLink PARAM_SET
- [ ] Unit tests verify floor activation and deactivation conditions

## Technical Details (if applicable)

### Functional Requirement Details

**Implementation location:** `crates/core/src/navigation/controller.rs` — `calculate_throttle()`

The throttle floor is applied as a `max()` operation after the normal throttle calculation:

```rust
// After computing: throttle = distance_throttle * heading_scale
if heading_error_abs < self.config.pivot_turn_angle
    && distance > self.config.wp_radius
{
    throttle = throttle.max(self.config.arc_turn_min_throttle);
}
```

**Config field:**

```rust
/// Minimum throttle during arc turns (0.0-1.0)
/// Only applies when heading error < pivot_turn_angle and distance > wp_radius.
pub arc_turn_min_throttle: f32,
```

**Interaction with spin-cap:**

The existing spin-cap (`max_spin_steering`) activates when throttle < `spin_throttle_threshold` (default 0.1). With `arc_turn_min_throttle = 0.15`, throttle during arc turns stays above the spin-cap threshold, so steering is not limited during arc turns. This is the desired behavior — arc turns need full steering authority with forward motion.

**Throttle values at key heading errors (default config):**

| Heading Error | Natural Throttle | After Floor | Spin-Cap Active? |
| ------------- | ---------------- | ----------- | ---------------- |
| 0°            | 1.00             | 1.00        | No               |
| 30°           | 0.67             | 0.67        | No               |
| 50°           | 0.44             | 0.44        | No               |
| 59°           | 0.34             | 0.34        | No               |
| 60°+          | ≤ 0.33           | (no floor)  | Depends          |
| 77°           | 0.14             | (no floor)  | Yes              |
| 90°           | 0.00             | (no floor)  | Yes              |

## Platform Considerations

### Cross-Platform

- `SimpleNavConfig` is in `pico_trail_core` — no platform dependency
- All unit tests run on host via `cargo test --lib`

## Risks & Mitigation

| Risk                                         | Impact | Likelihood | Mitigation                                     | Validation    |
| -------------------------------------------- | ------ | ---------- | ---------------------------------------------- | ------------- |
| Default 0.15 too high for tight maneuvering  | Low    | Low        | Configurable; can be reduced per deployment    | Field testing |
| Default 0.15 too low for heavy rover         | Low    | Low        | Configurable; can be increased per deployment  | Field testing |
| Floor interferes with approach zone throttle | Low    | Low        | Floor uses `max()` — only raises, never lowers | Unit tests    |

## Implementation Notes

- This requirement depends on FR-00144 (`pivot_turn_angle` must exist for the floor condition)
- The floor is a simple `max()` — it never reduces throttle, only raises it
- The `distance > wp_radius` guard prevents the floor from keeping the rover moving when it should stop at the target

## External References

- [ArduPilot WP_PIVOT_ANGLE](https://ardupilot.org/rover/docs/parameters.html#wp-pivot-angle) — Related parameter (ArduPilot handles the min throttle internally rather than as a separate parameter)
