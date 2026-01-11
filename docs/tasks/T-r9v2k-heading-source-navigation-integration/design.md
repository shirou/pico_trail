# T-r9v2k Heading Source and Navigation Integration

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-r9v2k-plan](plan.md)

## Overview

This design integrates the AHRS subsystem with navigation to enable autonomous navigation modes. The current NavigationController uses GPS Course-Over-Ground (COG) exclusively, which fails when the vehicle is stationary. By introducing a HeadingSource abstraction, the system can use AHRS heading when stationary and GPS COG when moving, providing reliable heading at all speeds.

## Success Metrics

- [ ] Heading available within 1 second of AHRS initialization
- [ ] No heading discontinuity when transitioning from stationary to moving
- [ ] All existing navigation modes continue to function
- [ ] Guided mode navigates to target position
- [ ] Auto mode executes mission waypoints

## Background and Current State

- Context: Navigation requires heading to calculate steering. GPS COG is only valid when moving.
- Current behavior: `SimpleNavigationController` uses `gps.course_over_ground.unwrap_or(0.0)`, defaulting to north when stationary.
- Pain points: Vehicle cannot navigate correctly from stationary; initial heading is always 0°.
- Constraints: Embassy async runtime; no_std environment; AHRS trait abstraction must be preserved.
- Related ADRs: [ADR-h3k9f-heading-source-integration](../../adr/ADR-h3k9f-heading-source-integration.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                    Navigation System                            │
│                                                                 │
│  ┌─────────────────┐     ┌─────────────────┐                   │
│  │ SharedAhrsState │     │  GPS Provider   │                   │
│  │ (yaw, healthy)  │     │ (position, COG) │                   │
│  └────────┬────────┘     └────────┬────────┘                   │
│           │                       │                             │
│           └───────────┬───────────┘                             │
│                       ▼                                         │
│           ┌─────────────────────┐                               │
│           │  FusedHeadingSource │                               │
│           │  (HeadingSource)    │                               │
│           └──────────┬──────────┘                               │
│                      │ get_heading() -> Option<f32>             │
│                      ▼                                          │
│           ┌─────────────────────┐                               │
│           │NavigationController │                               │
│           │ (update with heading)│                              │
│           └──────────┬──────────┘                               │
│                      │ NavigationOutput                         │
│                      ▼                                          │
│           ┌─────────────────────┐                               │
│           │  Mode (Guided/Auto) │                               │
│           └──────────┬──────────┘                               │
│                      │ steering, throttle                       │
│                      ▼                                          │
│           ┌─────────────────────┐                               │
│           │     Actuators       │                               │
│           └─────────────────────┘                               │
└─────────────────────────────────────────────────────────────────┘
```

### Components

**HeadingSource Trait** (`src/subsystems/navigation/heading.rs`):

```rust
pub trait HeadingSource {
    fn get_heading(&self) -> Option<f32>;
    fn is_valid(&self) -> bool;
    fn source_type(&self) -> HeadingSourceType;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HeadingSourceType {
    Ahrs,
    GpsCog,
    None,
}
```

**FusedHeadingSource** (`src/subsystems/navigation/heading.rs`):

```rust
pub struct FusedHeadingSource {
    ahrs_state: &'static SharedAhrsState,
    gps_provider: fn() -> Option<GpsPosition>,
    speed_threshold: f32,
}

impl FusedHeadingSource {
    pub fn new(
        ahrs_state: &'static SharedAhrsState,
        gps_provider: fn() -> Option<GpsPosition>,
        speed_threshold: f32,
    ) -> Self { ... }
}
```

**GuidedMode** (`src/rover/mode/guided.rs`):

```rust
pub struct GuidedMode<'a, H: HeadingSource> {
    actuators: &'a mut dyn ActuatorInterface,
    nav_controller: SimpleNavigationController,
    heading_source: &'a H,
    gps_provider: fn() -> Option<GpsPosition>,
    target_provider: fn() -> Option<PositionTarget>,
}
```

**AutoMode** (`src/rover/mode/auto.rs`):

```rust
pub struct AutoMode<'a, H: HeadingSource> {
    actuators: &'a mut dyn ActuatorInterface,
    nav_controller: SimpleNavigationController,
    heading_source: &'a H,
    gps_provider: fn() -> Option<GpsPosition>,
    mission_provider: fn() -> Option<&'static Mission>,
    current_wp_index: usize,
}
```

### Data Flow

1. BNO086 task updates `SharedAhrsState` at 100 Hz
2. GPS task updates position at 1-10 Hz
3. Mode calls `heading_source.get_heading()`:
   - If GPS speed >= threshold and COG available → return GPS COG
   - Else if AHRS healthy → return AHRS yaw (converted to degrees)
   - Else → return GPS COG as fallback or None
4. Mode calls `nav_controller.update(gps, target, heading, dt)`
5. NavigationController calculates steering and throttle
6. Mode applies output to actuators

### Data Models and Types

**NavigationController trait update**:

```rust
pub trait NavigationController {
    fn update(
        &mut self,
        current: &GpsPosition,
        target: &PositionTarget,
        heading: f32,  // NEW: degrees, 0-360
        dt: f32,
    ) -> NavigationOutput;

    fn reset(&mut self);
}
```

**HeadingSourceType for telemetry**:

```rust
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HeadingSourceType {
    Ahrs,
    GpsCog,
    None,
}
```

### Error Handling

- `get_heading()` returns `Option<f32>` - None indicates no valid heading
- Mode implementations must handle None by stopping or holding position
- `is_valid()` method allows pre-flight checks
- Errors logged via `log_warn!` macro

### Performance Considerations

- HeadingSource operations are synchronous (no async)
- SharedAhrsState read uses critical section (\~1μs)
- GPS provider is a function pointer (no allocation)
- All operations fit within 50 Hz mode update budget

## Alternatives Considered

1. Function pointer injection (CircleMode pattern)
   - Pros: Simple, proven
   - Cons: No encapsulation, duplicated fallback logic

2. Direct SharedAhrsState reference
   - Pros: Direct access
   - Cons: Tight coupling, no abstraction

Decision Rationale:

- HeadingSource trait provides clean abstraction for testing
- Encapsulates fusion logic in one place
- Allows future heading sources without mode changes
- See [ADR-h3k9f](../../adr/ADR-h3k9f-heading-source-integration.md) for full analysis

## Testing Strategy

### Unit Tests

- HeadingSource trait: Test FusedHeadingSource with mock AHRS/GPS
- Speed threshold switching: Verify correct source selection
- Edge cases: AHRS unhealthy, GPS unavailable, both unavailable

### Integration Tests

- Mode lifecycle: enter/update/exit with heading source
- Navigation accuracy: Target reached within WP_RADIUS
- Mode transitions: Manual → Guided → Auto → RTL

## Open Questions

- [ ] BNO086 yaw frame verification (NED, 0° = North) → Method: Hardware testing
- [ ] AHRS healthy requirement for mode entry → Next step: Define in mode implementation
