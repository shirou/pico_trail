# T-w8x3p Fix Guided Mode Heading Oscillation Design

## Metadata

- Type: Design
- Status: Implemented

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

This design addresses the rover spinning in circles during Guided mode navigation. The root causes are identified in AN-lsq0s: P-only steering, hard heading source switching, and throttle independent of heading error. The fix applies three complementary mitigations to break the oscillation loop.

## Success Metrics

- [ ] Steering change per update does not exceed `max_steering_rate * dt` (slew rate)
- [ ] PD controller damps heading rate, preventing oscillation under heading noise
- [ ] Throttle reduced to 0% when heading error >= 90 degrees
- [ ] Heading source does not switch when speed oscillates around 1.0 m/s (hysteresis)
- [ ] All existing tests pass; no regressions

## Background and Current State

- Context: Rover spins in circles during Guided mode, occasionally navigating straight
- Current behavior:
  - `SimpleNavigationController` uses P-only steering: `steering = heading_error / 90.0`
  - `FusedHeadingSource` switches between GPS COG and AHRS at a hard 1.0 m/s threshold
  - Throttle is distance-based only, ignoring heading error
- Pain points: No damping, no hysteresis, no heading-error throttle scaling
- Constraints: Must remain no_std compatible, f32-only, < 1ms update on RP2350

## Proposed Design

### High-Level Architecture

No architectural changes. All modifications are within existing structs and methods:

```text
FusedHeadingSource          SimpleNavigationController
├── get_gps_heading()       ├── calculate_steering()
│   └── ADD: hysteresis     │   └── CHANGE: P → PD + slew rate
└── (no other changes)      ├── calculate_throttle()
                            │   └── CHANGE: add heading error scaling
                            └── update()
                                └── CHANGE: pass dt, track prev_heading
```

### Component 1: PD Steering Controller with Rate Limiting

**File**: `crates/core/src/navigation/controller.rs`

Replace P-only with PD controller and add output slew rate limiting.

```rust
pub struct SimpleNavigationController {
    config: SimpleNavConfig,
    prev_heading_error: f32,  // NEW: for derivative calculation
    prev_steering: f32,       // NEW: for slew rate limiting
}
```

Steering calculation changes:

```rust
fn calculate_steering(&mut self, heading_error: f32, dt: f32) -> f32 {
    // Proportional term
    let p_term = heading_error / self.config.max_heading_error;

    // Derivative term (rate of heading error change)
    let d_term = if dt > 0.0 {
        let error_rate = (heading_error - self.prev_heading_error) / dt;
        error_rate * self.config.steering_d_gain
    } else {
        0.0
    };
    self.prev_heading_error = heading_error;

    // PD output
    let raw_steering = (p_term + d_term).clamp(-1.0, 1.0);

    // Slew rate limiting
    let max_change = self.config.max_steering_rate * dt;
    let limited_steering = if max_change > 0.0 {
        let delta = (raw_steering - self.prev_steering).clamp(-max_change, max_change);
        self.prev_steering + delta
    } else {
        raw_steering
    };
    self.prev_steering = limited_steering;

    limited_steering.clamp(-1.0, 1.0)
}
```

### Component 2: Heading-Error-Based Throttle Reduction

**File**: `crates/core/src/navigation/controller.rs`

Add heading error as a throttle scaling factor:

```rust
fn calculate_throttle(&self, distance: f32, heading_error_abs: f32) -> f32 {
    // Distance-based throttle (existing logic)
    let distance_throttle = /* existing */;

    // Heading error scaling: full throttle at 0 error, 0 at 90+ degrees
    let heading_scale = if heading_error_abs >= self.config.throttle_heading_error_max {
        0.0
    } else {
        1.0 - (heading_error_abs / self.config.throttle_heading_error_max)
    };

    (distance_throttle * heading_scale).clamp(0.0, 1.0)
}
```

### Component 3: Heading Source Hysteresis

**File**: `crates/firmware/src/subsystems/navigation/heading.rs`

Add hysteresis to prevent rapid switching at the speed threshold:

```rust
pub struct FusedHeadingSource<...> {
    // existing fields...
    speed_threshold: f32,           // existing: 1.0 m/s
    speed_hysteresis: f32,          // NEW: 0.3 m/s (half-band)
    using_gps_cog: bool,            // NEW: track current source
}
```

Hysteresis logic:

```rust
fn get_gps_heading(&self, gps: &GpsPosition) -> Option<f32> {
    let threshold = if self.using_gps_cog {
        // Currently using GPS: switch away at lower threshold
        self.speed_threshold - self.speed_hysteresis
    } else {
        // Currently using AHRS: switch to GPS at higher threshold
        self.speed_threshold + self.speed_hysteresis
    };

    if gps.speed >= threshold {
        gps.course_over_ground
    } else {
        None
    }
}
```

### New Configuration Fields

**File**: `crates/core/src/navigation/types.rs`

```rust
pub struct SimpleNavConfig {
    // existing fields...
    pub wp_radius: f32,
    pub approach_dist: f32,
    pub max_heading_error: f32,
    pub min_approach_throttle: f32,

    // NEW fields
    /// D-gain for steering PD controller (ArduPilot: ATC_STR_RAT_D)
    pub steering_d_gain: f32,
    /// Maximum steering change per second (slew rate, 0 = unlimited)
    pub max_steering_rate: f32,
    /// Heading error (degrees) at which throttle reaches zero
    pub throttle_heading_error_max: f32,
}

impl Default for SimpleNavConfig {
    fn default() -> Self {
        Self {
            wp_radius: 2.0,
            approach_dist: 10.0,
            max_heading_error: 90.0,
            min_approach_throttle: 0.2,
            // New defaults (conservative)
            steering_d_gain: 0.005,
            max_steering_rate: 2.0,         // full range in 1 second
            throttle_heading_error_max: 90.0, // zero throttle at 90+ degrees
        }
    }
}
```

### Data Flow (Updated)

```text
navigation_task (50Hz)
  +-- heading_source.get_heading()
  |     +-- GPS speed vs threshold WITH HYSTERESIS
  |     +-- Track using_gps_cog state
  +-- controller.update(lat, lon, target, heading, dt)
  |     +-- calculate_bearing(current -> target)
  |     +-- heading_error = wrap_180(bearing - heading)
  |     +-- steering = PD(heading_error, dt) + slew rate limit
  |     +-- throttle = f(distance) * f(heading_error)
  +-- NAV_OUTPUT = { steering, throttle }
```

### Error Handling

- `prev_heading_error` initialized to 0.0; first update produces no D-term spike
- `dt <= 0.0` disables derivative calculation (safe fallback)
- `max_steering_rate = 0.0` disables slew rate limiting
- `using_gps_cog` initialized to `false` (start with AHRS)

### Performance Considerations

- Adds 2 multiplications and 2 comparisons to steering calculation
- Adds 1 division and 1 multiplication to throttle calculation
- No additional memory allocation; 2 new `f32` fields in controller state
- Total overhead: negligible (< 10us additional on RP2350)

## Alternatives Considered

1. **Slew rate only (no PD)**: Simpler, but masks root cause without damping oscillation energy
2. **Full PID**: Overkill for initial fix; I-term requires anti-windup and is unnecessary for heading tracking
3. **Heading source blending (weighted interpolation)**: Smoother but adds complexity; hysteresis alone is sufficient as first step

### Decision Rationale

PD + slew rate + heading throttle + hysteresis addresses all critical root causes with minimal code change. Each component is independently testable and the conservative defaults ensure safe field behavior.

## Testing Strategy

### Unit Tests

- PD steering: verify D-term opposes increasing error
- Slew rate: verify steering change does not exceed limit
- Throttle scaling: verify throttle reduces with heading error
- Hysteresis: verify heading source does not flip-flop at threshold
- Backward compatibility: existing tests pass with new defaults

### Field Testing

- Guided mode to waypoint 50m away on flat ground
- Verify no spinning at start (stationary → moving transition)
- Verify stable heading tracking during straight-line navigation
- Verify recovery from large heading disturbance (manual push)

## Open Questions

- [ ] Optimal D-gain value: start with 0.005, tune via field testing
- [ ] Hysteresis band width: start with 0.3 m/s, adjust based on GPS noise
- [ ] Should slew rate be configurable via MAVLink parameter? Defer to post-fix

## External References

- [ArduPilot ATC_STR_RAT_D](https://ardupilot.org/rover/docs/parameters.html)
- [PID Control Theory](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller)
