# T-00036 Firmware-to-Core Migration Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-00036-plan](./plan.md)

## Overview

This design documents the migration of platform-independent code from `crates/firmware` to `crates/core`. The workspace separation (T-00035) established the two-crate boundary, but left several pure algorithms and duplicated code in firmware. This task completes the migration by moving \~1,600 lines of platform-independent code to core.

## Success Metrics

- [ ] Zero functionally identical code between crates
- [ ] All migrated algorithms have unit tests in core
- [ ] `cargo test -p pico_trail_core --lib --quiet` passes
- [ ] `./scripts/build-rp2350.sh pico_trail_rover` compiles successfully
- [ ] Zero `#[cfg(feature` directives introduced in core

## Background and Current State

- Context: T-00035 completed the initial workspace separation. However, the migration was conservative—several pure algorithms remained in firmware, and some code was copied to core without removing the firmware version.
- Current behavior:
  - 4 files are duplicated between crates (\~560 lines)
  - DCM and geographic algorithms in firmware have zero platform dependencies (\~350 lines)
  - 5 mixed modules contain extractable business logic (\~700 lines)
- Pain points:
  - Duplicate code risks divergence during maintenance
  - Pure algorithms in firmware cannot be tested with `cargo test -p pico_trail_core`
  - Navigation algorithms are scattered between core `mode/nav.rs` and firmware `subsystems/navigation/geo.rs`
- Constraints:
  - Core must remain `no_std` with zero `cfg` directives
  - Firmware build must not break during migration
  - Existing firmware modules that use migrated code must be updated to import from core

## Proposed Design

### High-Level Architecture

```text
BEFORE:
  crates/core/                    crates/firmware/
  ├── arming/error.rs             ├── core/arming/error.rs        (DUPLICATE)
  ├── kinematics/                 ├── libraries/kinematics/       (DUPLICATE)
  ├── parameters/{block,crc}.rs   ├── core/parameters/{block,crc} (DUPLICATE)
  ├── mode/nav.rs (partial)       ├── subsystems/navigation/geo.rs
  ├── ahrs/calibration.rs         ├── subsystems/ahrs/dcm.rs
  └── navigation/types.rs         ├── subsystems/ahrs/traits.rs
                                  ├── subsystems/navigation/controller.rs
                                  ├── subsystems/navigation/heading.rs
                                  ├── subsystems/navigation/path_recorder.rs
                                  └── core/mission/state.rs

AFTER:
  crates/core/                    crates/firmware/
  ├── arming/error.rs             ├── core/arming/ (NO error.rs)
  ├── kinematics/                 ├── (NO libraries/kinematics/)
  ├── parameters/{block,crc}.rs   ├── core/parameters/ (NO block.rs, crc.rs)
  ├── ahrs/
  │   ├── calibration.rs
  │   ├── dcm.rs            ←NEW
  │   └── traits.rs          ←NEW (Ahrs trait, AhrsState)
  ├── navigation/
  │   ├── types.rs
  │   ├── geo.rs             ←NEW (consolidated haversine)
  │   ├── controller.rs      ←NEW (NavigationController trait)
  │   ├── heading.rs         ←NEW (HeadingSource trait)
  │   └── path_recorder.rs   ←NEW (PathRecorder algorithm)
  ├── mission/
  │   ├── mod.rs
  │   └── state.rs           ←NEW (state transitions)
  └── mode/nav.rs (deprecated funcs removed, use navigation/geo)
```

### Components

#### Core Position Type

A new `Position` type in `crates/core/src/navigation/types.rs`:

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position {
    pub lat: f64,
    pub lon: f64,
    pub alt: f32,
}
```

This replaces the firmware-only `GpsPosition` as the core-level type. Firmware's `GpsPosition` can implement `From<Position>` or be replaced entirely.

#### Geographic Algorithms Consolidation

Merge firmware's `geo.rs` functions into core `navigation/geo.rs`:

- `calculate_bearing(from: Position, to: Position) -> f32`
- `calculate_distance(from: Position, to: Position) -> f32`
- `wrap_180(angle: f32) -> f32`
- `wrap_360(angle: f32) -> f32`
- `offset_position(pos: Position, distance: f32, bearing: f32) -> Position`

Remove duplicate functions from `mode/nav.rs` (`haversine_distance_bearing`, `normalize_angle`).

#### DCM Algorithm

Move `dcm.rs` to core unchanged, replacing `log_warn!()` call with a silent fallback or return value.

#### Navigation Controller Trait

Extract to core with generic position type:

```rust
pub trait NavigationController {
    fn update(&mut self, current: Position, heading_deg: f32) -> NavigationOutput;
    fn set_target(&mut self, target: PositionTarget);
    fn is_arrived(&self) -> bool;
}
```

#### AHRS Traits

Move `Ahrs` trait and `AhrsState` to core. Keep `SharedAhrsState` (critical-section wrapper) in firmware.

#### Path Recorder

Move `PathPoint` and `PathRecorder` to core. Remove logging calls. Keep the global `PATH_RECORDER` static in firmware.

#### Mission State Machine

Extract state transition logic to core. Keep `EmbassyState`-wrapped global in firmware.

### Data Flow

```text
No data flow changes. Firmware modules that previously used local
implementations now import from pico_trail_core instead.
```

### Error Handling

- No new error types required
- Existing error types remain unchanged
- Migrated code preserves existing `Result<T, E>` patterns

### Performance Considerations

- Zero runtime impact: Only module location changes, no logic changes
- Compilation: Core crate gains \~1,600 lines; incremental build impact is minimal
- All core types remain `Copy` or `Clone` where appropriate

### Platform Considerations

#### Embedded (RP2350)

- Firmware imports from `pico_trail_core` instead of local modules
- No behavioral change on target hardware

#### Host (x86_64 tests)

- Core tests gain algorithm coverage (DCM, navigation, path recording)
- `cargo test -p pico_trail_core --lib --quiet` tests all migrated code

## Alternatives Considered

1. **Leave duplicates, only move new code**
   - Pros: Less churn
   - Cons: Duplicates will diverge, maintenance burden increases
   - Decision: Rejected - duplicates should be eliminated

2. **Create a third crate for navigation algorithms**
   - Pros: Finer separation
   - Cons: Over-engineering for current project size, more workspace complexity
   - Decision: Rejected - core crate already has navigation module

Decision Rationale: Consolidating into the existing two-crate structure is the simplest approach that achieves all goals.

## Testing Strategy

### Unit Tests

- Each migrated module includes its existing tests (moved from firmware)
- New tests added for consolidated geographic functions
- Tests use no feature flags: `cargo test -p pico_trail_core --lib --quiet`

### Integration Tests

- `./scripts/build-rp2350.sh pico_trail_rover` validates firmware compilation
- Hardware verification confirms no behavioral regression

## Documentation Impact

- Update `docs/architecture.md` with new module locations in core
- Update module-level documentation in core `lib.rs`

## Open Questions

- [ ] Should `mode/nav.rs` be fully deprecated in favor of `navigation/geo.rs`, or should `mode/nav.rs` re-export from `navigation/geo`? → Next step: Evaluate during Phase 2 implementation
- [ ] Should firmware's `GpsPosition` be replaced by core's `Position` everywhere, or should both coexist with `From` conversion? → Next step: Check how many firmware files use `GpsPosition` and assess migration scope
