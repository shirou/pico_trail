# FR-00146 Navigation Parameter Store Registration

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00045-configurable-turn-behavior](../analysis/AN-00045-configurable-turn-behavior.md)
- Prerequisite Requirements:
  - [FR-00084-navigation-controller](FR-00084-navigation-controller.md)
  - [FR-00144-configurable-pivot-turn-threshold](FR-00144-configurable-pivot-turn-threshold.md)
  - [FR-00145-arc-turn-minimum-throttle](FR-00145-arc-turn-minimum-throttle.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00042-configurable-turn-behavior](../tasks/T-00042-configurable-turn-behavior/README.md)

## Requirement Statement

All `SimpleNavConfig` fields shall be registered in the parameter store as a `NavigationParams` parameter group, enabling runtime configuration via MAVLink-compatible GCS tools (Mission Planner, QGroundControl). The implementation shall follow the established `register_defaults()` / `from_store()` pattern used by other parameter groups.

## Rationale

Navigation parameters are currently hardcoded as compile-time defaults in `SimpleNavConfig`. Other subsystems (Loiter, Circle, Arming, Battery, Failsafe, Fence, Compass) already register parameters with `ParameterStore`, allowing runtime tuning via GCS without recompilation. Navigation parameters — the most frequently tuned for field operation — should support the same workflow.

## User Story (if applicable)

As a rover operator, I want to adjust navigation parameters (waypoint radius, steering gains, pivot turn threshold, etc.) from Mission Planner without reflashing firmware, so that I can tune the rover's behavior in the field.

## Acceptance Criteria

- [ ] `NavigationParams` struct created in `crates/core/src/parameters/navigation.rs`
- [ ] All 12 `SimpleNavConfig` fields registered with `register_defaults()`
- [ ] `from_store()` loads values with clamping to valid ranges
- [ ] `to_nav_config()` converts `NavigationParams` to `SimpleNavConfig`
- [ ] `is_valid()` validates parameter consistency
- [ ] Parameter names follow ArduPilot conventions where standard equivalents exist
- [ ] `NavigationParams::register_defaults()` called in `ParamHandler::new()`
- [ ] Parameters visible and editable in Mission Planner / QGroundControl
- [ ] Unit tests for `register_defaults()`, `from_store()`, clamping, and validation

## Technical Details (if applicable)

### Parameter Name Mapping

| SimpleNavConfig Field        | Parameter Name   | ArduPilot Standard | Default | Range      | Unit    |
| ---------------------------- | ---------------- | ------------------ | ------- | ---------- | ------- |
| `wp_radius`                  | `WP_RADIUS`      | Yes                | 2.0     | 0.5–100.0  | meters  |
| `approach_dist`              | `WP_APPR_DIST`   | No                 | 10.0    | 1.0–200.0  | meters  |
| `max_heading_error`          | `ATC_HDG_ERR`    | No                 | 90.0    | 10.0–180.0 | degrees |
| `min_approach_throttle`      | `WP_APPR_THR`    | No                 | 0.2     | 0.0–1.0    | ratio   |
| `steering_d_gain`            | `ATC_STR_RAT_D`  | Yes                | 0.05    | 0.0–1.0    | –       |
| `max_steering_rate`          | `ATC_STR_SMAX`   | No                 | 2.0     | 0.0–10.0   | /sec    |
| `throttle_heading_error_max` | `ATC_THR_HERR`   | No                 | 90.0    | 10.0–180.0 | degrees |
| `max_spin_steering`          | `ATC_SPIN_MAX`   | No                 | 0.3     | 0.0–1.0    | ratio   |
| `spin_throttle_threshold`    | `ATC_SPIN_THR`   | No                 | 0.1     | 0.0–1.0    | ratio   |
| `heading_filter_alpha`       | `ATC_HDG_FILT`   | No                 | 0.3     | 0.0–1.0    | ratio   |
| `pivot_turn_angle`           | `WP_PIVOT_ANGLE` | Yes                | 60.0    | 0.0–180.0  | degrees |
| `arc_turn_min_throttle`      | `WP_ARC_THR`     | No                 | 0.15    | 0.0–1.0    | ratio   |

### Implementation Pattern

Follow the established pattern from `LoiterParams` (`crates/core/src/parameters/loiter.rs`):

```rust
// crates/core/src/parameters/navigation.rs
pub struct NavigationParams { /* fields matching above table */ }

impl NavigationParams {
    pub fn register_defaults(store: &mut ParameterStore) -> Result<(), ParameterError> { ... }
    pub fn from_store(store: &ParameterStore) -> Self { ... }
    pub fn to_nav_config(&self) -> SimpleNavConfig { ... }
    pub fn is_valid(&self) -> bool { ... }
}
```

### Registration Location

Add to `ParamHandler::new()` in `crates/firmware/src/communication/mavlink/handlers/param.rs`:

```rust
let _ = crate::parameters::NavigationParams::register_defaults(&mut store);
```

### Non-Standard Parameter Names

Parameters without ArduPilot equivalents use `WP_` (waypoint navigation) or `ATC_` (attitude/steering control) prefixes following ArduPilot naming conventions. These are project-specific parameters similar to the existing `PIN_*` exception documented in CLAUDE.md.

## Platform Considerations

### Cross-Platform

- `NavigationParams` is in `pico_trail_core` — no platform dependency
- `ParamHandler` registration is in `pico_trail_firmware` (platform-specific)
- All unit tests run on host via `cargo test --lib`

## Risks & Mitigation

| Risk                                        | Impact | Likelihood | Mitigation                                      | Validation     |
| ------------------------------------------- | ------ | ---------- | ----------------------------------------------- | -------------- |
| Parameter name conflicts with future params | Low    | Low        | Use distinctive prefixes; document in CLAUDE.md | Manual review  |
| Invalid parameter combinations at runtime   | Medium | Low        | `is_valid()` checks; clamp in `from_store()`    | Unit tests     |
| Flash storage space for 12 new parameters   | Low    | Low        | 12 × f32 = 48 bytes — negligible                | Embedded build |

## Implementation Notes

- `NavigationParams` loads from store and converts to `SimpleNavConfig`; the navigation controller continues using `SimpleNavConfig` unchanged
- The `to_nav_config()` conversion keeps navigation logic decoupled from parameter storage
- Existing `SimpleNavConfig::default()` remains for use in tests and fallback scenarios

## External References

- [ArduPilot WP_RADIUS](https://ardupilot.org/rover/docs/parameters.html#wp-radius)
- [ArduPilot WP_PIVOT_ANGLE](https://ardupilot.org/rover/docs/parameters.html#wp-pivot-angle)
- [ArduPilot ATC_STR_RAT_D](https://ardupilot.org/rover/docs/parameters.html#atc-str-rat-d)
