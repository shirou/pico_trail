# T-3n2ej Workspace Separation Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-3n2ej-plan](./plan.md)

## Overview

This design documents the refactoring of pico_trail from a single-crate architecture into a Cargo Workspace with two primary crates: `crates/core` (pure `no_std` business logic) and `crates/firmware` (Embassy/RP2350 binary). The goal is to eliminate all `#[cfg(feature = ...)]` directives from core business logic, enabling straightforward host testing and clean architectural separation.

## Success Metrics

- [ ] Zero `#[cfg(feature` directives in `crates/core/src/`
- [ ] `cargo test -p pico_trail_core` passes on host without feature flags
- [ ] `./scripts/build-rp2350.sh` compiles successfully
- [ ] All existing functionality preserved (no behavioral regression)

## Background and Current State

- Context: pico_trail is an embedded rover control system using Embassy async runtime on RP2350
- Current behavior: Single crate with \~150 feature gates mixing business logic and platform code
- Pain points:
  - Host tests require complex feature flags and stub implementations
  - \~34 files use `#[cfg(feature = "embassy")]` with 120 occurrences
  - Duplicate implementations (`#[cfg(feature)]` + `#[cfg(not(feature))]` pairs)
  - Business logic tightly coupled to Embassy async runtime
- Constraints: Must maintain backward compatibility with existing examples
- Related ADRs: ADR to be created for workspace architecture decision

## Proposed Design

### High-Level Architecture

```text
pico_trail/
├── Cargo.toml                 # Workspace root
├── crates/
│   ├── core/                  # Pure no_std business logic
│   │   ├── Cargo.toml
│   │   └── src/
│   │       ├── lib.rs
│   │       ├── traits/        # TimeSource, Tick, Platform traits
│   │       ├── navigation/    # Navigation algorithms
│   │       ├── control/       # Control algorithms (PID, etc.)
│   │       ├── arming/        # Arming state machine
│   │       ├── mission/       # Mission state management
│   │       ├── mode/          # Mode state machines
│   │       └── kinematics/    # Differential drive math
│   └── firmware/              # Embassy binary
│       ├── Cargo.toml         # Depends on: core, embassy-*, defmt
│       └── src/
│           ├── main.rs        # Entry point
│           ├── tasks/         # Embassy tasks wrapping core logic
│           ├── platform/      # RP2350 HAL bindings
│           ├── formatters/    # defmt::Format impls for core types
│           └── devices/       # Hardware device drivers
├── examples/                  # Moved to firmware or workspace examples
└── scripts/
    └── check-core-no-cfg.sh   # CI lint script
```

### Components

#### Core Crate (`crates/core`)

- **traits/**: Platform-agnostic abstractions
  - `TimeSource`: Time operations (`now_ms()`, `now_us()`)
  - `MockTime`: Test implementation (no feature gate)

- **navigation/**: Navigation algorithms
  - Waypoint management
  - Path planning
  - Heading calculations

- **control/**: Control algorithms
  - PID controllers
  - Differential drive kinematics

- **arming/**: Arming state machine
  - Pre-arm checks
  - State transitions

- **mission/**: Mission state management
  - Waypoint storage
  - Mission execution state

- **mode/**: Mode types and utilities
  - Mode trait definition
  - State types (AutoState, GuidedState, RtlState)
  - Navigation calculation utilities

#### Firmware Crate (`crates/firmware`)

The firmware crate is **Embassy/RP2350-only** by design. All Embassy and platform dependencies are non-optional.

**Design Principle: No Feature Gates for Embassy**

Since firmware always targets embedded hardware (no host tests), feature flags for Embassy/defmt/HAL create unnecessary complexity. Dependencies are always enabled:

```toml
[dependencies]
# Always enabled (non-optional)
cortex-m = "0.7"
defmt = "1.0.1"
embassy-executor = { ... }
embassy-time = { ... }
embassy-rp = { ..., features = ["rp235xa", ...] }

[features]
default = ["rover"]
rover = []           # Vehicle type
gps-ublox = []       # GPS vendor initialization
usb_serial = []      # Debug USB serial
```

**Modules:**

- **tasks/**: Embassy async tasks
  - `control_task`: Wraps core Controller with Timer loop
  - `navigation_task`: Wraps core Navigator
  - `telemetry_task`: MAVLink handling

- **platform/**: RP2350-specific code
  - GPIO, I2C, SPI, UART wrappers
  - PWM motor control
  - Flash storage

- **formatters/**: defmt::Format implementations
  - External impl blocks for core types
  - Enables RTT logging without modifying core

- **devices/**: Hardware drivers
  - IMU drivers (BNO086, ICM20948)
  - GPS drivers
  - Motor drivers

### Data Flow

```text
Sensors → [firmware/devices] → SensorInput
                                    ↓
                              [core/control]
                               Controller.tick()
                                    ↓
                              ControlOutput
                                    ↓
                            [firmware/platform]
                             Motor PWM Output
```

### Data Models and Types

#### Core Types (no defmt dependency)

```rust
// crates/core/src/navigation/types.rs
#[derive(Debug, Clone, Copy)]
pub struct Position {
    pub lat: f64,
    pub lon: f64,
}

#[derive(Debug, Clone)]
pub struct NavigationState {
    pub position: Position,
    pub heading_deg: f32,
    pub speed_mps: f32,
}

// crates/core/src/control/types.rs
#[derive(Debug, Clone, Copy)]
pub struct ControlOutput {
    pub throttle: f32,
    pub steering: f32,
}
```

#### Trait Definitions

```rust
// crates/core/src/traits/time.rs
pub trait TimeSource {
    fn now_ms(&self) -> u64;
    fn now_us(&self) -> u64;
}
```

### Error Handling

- Core crate uses `Result<T, E>` with custom error types
- No `defmt::Format` on error types in core
- Firmware wraps errors with defmt-formatted messages for logging

### Security Considerations

N/A - This is an architectural refactoring, no security impact.

### Performance Considerations

- **Monomorphization**: Generics over traits enable zero-cost abstraction
- **No heap allocation**: Core remains `no_std` compatible
- **Pure functions**: Navigation utilities are pure, no async overhead
- Control loop timing: Must maintain 50Hz (20ms) cycle time

### Platform Considerations

#### Embedded (RP2350)

- Firmware crate compiles for `thumbv8m.main-none-eabihf`
- Embassy executor manages async tasks
- defmt logging via RTT

#### Host (x86_64 tests)

- Core crate compiles natively
- `MockTime` provides deterministic time for tests
- Standard `cargo test` without feature flags

## Alternatives Considered

1. **Keep single crate, reduce cfg only**
   - Pros: Less restructuring effort
   - Cons: Cannot eliminate cfg completely, still requires stubs

2. **Three-crate split (core, embassy-integration, firmware)**
   - Pros: Finer separation of concerns
   - Cons: More complex dependency graph, diminishing returns

Decision Rationale: Two-crate split provides optimal balance between separation and complexity.

## Migration and Compatibility

- Backward compatibility: Existing build commands will change (`cargo build` → workspace commands)
- Rollout plan: Phased migration by module
- Deprecation plan: Old `src/` structure removed after all modules migrated

## Testing Strategy

### Unit Tests

- Core crate tests use `MockTime` and mock inputs
- Tests located in `crates/core/src/*/tests.rs`
- Run with `cargo test -p pico_trail_core`

### Integration Tests

- Firmware tests require embedded target or QEMU
- Hardware-in-the-loop tests for motor/sensor integration

### External API Parsing (if applicable)

N/A

### Performance & Benchmarks (if applicable)

- Measure control loop timing before/after migration
- Target: ≤20ms cycle time maintained

## Documentation Impact

- Update CLAUDE.md with new build commands
- Update README with workspace structure
- Add architecture documentation for new layout

## External References

- [Cargo Workspaces](https://doc.rust-lang.org/book/ch14-03-cargo-workspaces.html)
- [Embassy Book](https://embassy.dev/book/)
- [no_std Book](https://docs.rust-embedded.org/book/intro/no-std.html)

## Open Questions

- [ ] Should MAVLink message definitions be in core or a separate crate? → Next step: Evaluate during Phase 2 migration
- [ ] How to handle parameters that need both definition (core) and persistence (firmware)? → Method: Design trait for parameter storage

## Appendix

### Module Migration Mapping

| Current Location              | Target Location                 | Notes                         |
| ----------------------------- | ------------------------------- | ----------------------------- |
| `src/libraries/kinematics/`   | `crates/core/src/kinematics/`   | ✅ Migrated (Phase 2)         |
| `src/core/arming/`            | `crates/core/src/arming/`       | ✅ Partial (error types)      |
| `src/core/mission/`           | `crates/core/src/mission/`      | ✅ Migrated (Phase 4)         |
| `src/rover/mode/`             | `crates/core/src/mode/`         | ✅ Migrated (Phase 4)         |
| `src/libraries/rc_channel/`   | `crates/core/src/rc/`           | ✅ Migrated (Phase 5)         |
| `src/libraries/srv_channel/`  | `crates/core/src/servo/`        | ✅ Migrated (Phase 5)         |
| `src/libraries/motor_driver/` | `crates/core/src/motor/`        | ✅ Migrated (Phase 5)         |
| `src/subsystems/navigation/`  | `crates/core/src/navigation/`   | Extract algorithms from async |
| `src/platform/`               | `crates/firmware/src/platform/` | Direct move                   |
| `src/devices/`                | `crates/firmware/src/devices/`  | Direct move                   |
| `examples/`                   | `crates/firmware/examples/`     | Update imports                |
