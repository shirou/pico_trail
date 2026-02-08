# T-00164 SITL Autopilot Core Integration Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

Add embassy-sync and embassy-futures as optional dependencies of `pico_trail_core` behind an `embassy` feature flag, then migrate the autopilot integration layer from firmware to core. SITL and firmware both consume the same autopilot logic from core, eliminating duplication and enabling all rover modes in SITL.

## Success Metrics

- [ ] Core compiles and tests pass: `cargo test -p pico_trail_core --features embassy --lib`
- [ ] Core without `embassy` feature is unchanged: `cargo test -p pico_trail_core --lib`
- [ ] Firmware builds: `./scripts/build-rp2350.sh pico_trail_rover`
- [ ] SITL closed-loop: Mission Planner can arm and steer rover in Gazebo
- [ ] All rover modes functional in SITL

## Background and Current State

- **Core crate**: `no_std`, platform-agnostic algorithms (kinematics, navigation, AHRS, parameters, modes, mission)
- **Firmware crate**: Contains autopilot logic (MessageDispatcher, handlers, SystemState, ModeManager, mode implementations) alongside platform-specific code (embassy-rp, cortex-m, cyw43)
- **SITL crate**: Has bridge, adapters, GcsLink (telemetry only), SitlPlatform — but no autopilot logic
- **Problem**: Firmware's autopilot logic cannot be imported by SITL because firmware depends on cortex-m/embassy-rp
- **Key insight**: embassy-sync is built on `critical-section`, which core already depends on. embassy-futures is a pure polling library. Neither is embedded-specific.
- Related ADR: [ADR-00161](../../adr/ADR-00161-sitl-autopilot-integration-layer.md) — embassy-sync in core approach

## Proposed Design

### Core Feature Configuration

```toml
# crates/core/Cargo.toml
[features]
default = []
embassy = ["dep:embassy-sync", "dep:embassy-futures"]

[dependencies]
embassy-sync = { git = "https://github.com/embassy-rs/embassy", optional = true }
embassy-futures = { git = "https://github.com/embassy-rs/embassy", optional = true }
critical-section = "1.2"  # Promoted from dev-dependency to dependency
```

Downstream crate configuration:

```toml
# crates/firmware/Cargo.toml
pico_trail_core = { path = "../core", features = ["embassy", "defmt"] }

# crates/sitl/Cargo.toml
pico_trail_core = { path = "../core", features = ["embassy"] }
```

### New Core Module Structure

```text
crates/core/src/
├── lib.rs              # Add: autopilot, communication modules (behind embassy feature)
├── traits/
│   ├── sync.rs         # SharedState trait + EmbassyState + MockState (embassy feature)
│   └── flash.rs        # FlashInterface trait (moved from firmware platform traits)
├── autopilot/
│   ├── mod.rs
│   ├── state.rs        # SystemState, ArmedState, FlightMode, BatteryState, etc.
│   └── vehicle.rs      # VehicleType trait, FlightModeOps, GroundRover, SurfaceBoat
├── communication/
│   ├── mod.rs
│   ├── dispatcher.rs   # MessageDispatcher<V>
│   └── handlers/
│       ├── mod.rs
│       ├── command.rs   # CommandHandler<V> (ARM/DISARM, SET_MODE, etc.)
│       ├── param.rs     # ParamHandler (generic over FlashInterface)
│       ├── telemetry.rs # TelemetryStreamer<V>
│       ├── mission.rs   # MissionHandler
│       ├── rc_input.rs  # RcInputHandler
│       ├── navigation.rs
│       └── status_notifier.rs  # StatusNotifier (STATUSTEXT queue)
├── mode/
│   ├── mod.rs          # Mode trait (already exists), ModeManager
│   ├── manual.rs       # ManualMode (uses embassy-sync)
│   ├── auto.rs
│   ├── guided.rs
│   ├── rtl.rs
│   ├── loiter.rs
│   ├── circle.rs
│   └── smartrtl.rs
└── navigation/
    └── types.rs        # GpsPosition (moved from firmware), GpsFixType (already here)
```

### Component Migration Details

#### 1. SharedState Trait (`core/src/traits/sync.rs`)

Gated behind `embassy` feature. Three types:

```rust
// Always available
pub trait SharedState<T> {
    fn with<F, R>(&self, f: F) -> R where F: FnOnce(&T) -> R;
    fn with_mut<F, R>(&self, f: F) -> R where F: FnOnce(&mut T) -> R;
}

// Available with embassy feature
#[cfg(feature = "embassy")]
pub struct EmbassyState<T> {
    inner: Mutex<CriticalSectionRawMutex, core::cell::RefCell<T>>,
}

// Always available (for tests)
pub struct MockState<T> {
    inner: core::cell::RefCell<T>,
}
```

#### 2. Autopilot State Types (`core/src/autopilot/state.rs`)

Move from firmware with minimal changes:

- `ArmedState` — no deps, moves directly
- `FlightMode` — change `defmt::Format` to `#[cfg_attr(feature = "defmt", derive(defmt::Format))]`
- `BatteryState`, `AttitudeState`, `HomePosition` — no deps, move directly
- `SystemState` — depends on `GpsPosition`, which also moves to core

#### 3. GpsPosition (`core/src/navigation/types.rs`)

Move from `firmware/src/devices/gps.rs`. It only depends on `GpsFixType` which is already in core:

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GpsPosition {
    pub latitude: f32,
    pub longitude: f32,
    pub altitude: f32,
    pub speed: f32,
    pub course_over_ground: Option<f32>,
    pub fix_type: GpsFixType,
    pub satellites: u8,
}
```

#### 4. VehicleType Trait (`core/src/autopilot/vehicle.rs`)

Move from `firmware/src/communication/mavlink/vehicle/`. Minimal dependencies (only MAVLink enum types).

#### 5. FlashInterface Trait (`core/src/traits/flash.rs`)

Move from `firmware/src/platform/traits/flash.rs`. The trait is a pure interface definition (`read`, `write`, `erase`, `block_size`, `capacity`) with zero firmware dependencies. Platform implementations (RP2350Flash, MockFlash) remain in firmware.

#### 6. ParamHandler (`core/src/communication/handlers/param.rs`)

Move from firmware. Refactor constructor to separate Flash loading from parameter registration:

```rust
// In core — generic over FlashInterface
pub struct ParamHandler {
    store: ParameterStore,  // ParameterStore is already in core
}

impl ParamHandler {
    /// Create from an already-initialized store.
    /// Caller (firmware/SITL) is responsible for loading from Flash
    /// and registering platform-specific parameter defaults.
    pub fn new(store: ParameterStore) -> Self { Self { store } }

    /// Save parameters to Flash (generic over platform).
    pub fn save_to_flash<F: FlashInterface>(&mut self, flash: &mut F) -> Result<()> { ... }
}
```

Firmware-specific parameter modules (`WifiParams`, `BoardParams`) stay in firmware and call `register_defaults()` before constructing `ParamHandler`. Platform-agnostic params (`ArmingParams`, `BatteryParams`, `NavigationParams`, etc.) can optionally move to core later.

Also move `load_from_flash` / `save_to_flash` serialization functions from `firmware/src/parameters/storage.rs` to core, since `ParameterStore` is already in core and these functions only depend on `FlashInterface` (now in core).

#### 7. MessageDispatcher (`core/src/communication/dispatcher.rs`)

The dispatcher routes MAVLink messages to handlers. All handlers now in core:

```rust
pub struct MessageDispatcher<V: VehicleType> {
    param_handler: ParamHandler,
    command_handler: CommandHandler<V>,
    telemetry_streamer: TelemetryStreamer<V>,
    mission_handler: MissionHandler,
    rc_input_handler: RcInputHandler,
    navigation_handler: NavigationHandler,
}
```

#### 8. ModeManager (`core/src/mode/`)

Move from `firmware/src/rover/mode_manager.rs`. Uses `Mode` trait (already in core) and `SystemState` (migrating to core).

#### 9. Mode Implementations

- **ManualMode**: Uses `embassy_sync::Mutex` and `embassy_futures::block_on` — gated behind `embassy` feature
- **Other modes** (Auto, Guided, RTL, Loiter, Circle, SmartRTL): No embassy deps, move directly

### SITL Integration Architecture

```text
Mission Planner
    │
    │ MAVLink TCP
    ▼
┌───────────────────────────────────────────┐
│                  GcsLink                   │
│                                            │
│  poll_incoming()                           │
│       │                                    │
│       ▼                                    │
│  core::MessageDispatcher::dispatch()       │
│       │                                    │
│       ├── CommandHandler (ARM, SET_MODE)    │
│       ├── RcInputHandler (RC_OVERRIDE)     │
│       ├── MissionHandler (MISSION_*)       │
│       └── TelemetryStreamer (telemetry)     │
│                                            │
│  core::ModeManager::execute()              │
│       │                                    │
│       ├── ManualMode → DifferentialDrive   │
│       ├── AutoMode → NavigationController  │
│       ├── GuidedMode → PositionTarget      │
│       └── ...                              │
│                                            │
│  poll_and_heartbeat()                      │
│       → HEARTBEAT with armed/mode state    │
│                                            │
│  send_telemetry()                          │
│       → ATTITUDE, GPS, SYS_STATUS          │
└──────────────┬────────────────────────────┘
               │ Actuator outputs from modes
               ▼
┌──────────────────────────┐
│      SitlPlatform        │
│  collect_actuator_cmds() │
└──────────┬───────────────┘
           │
           ▼
┌──────────────────────────┐
│     GazeboAdapter        │
│  send_actuators()        │
└──────────────────────────┘
```

### gazebo_bridge Main Loop Changes

```rust
loop {
    // 1. Poll GCS incoming → dispatch through core's MessageDispatcher
    for (header, msg) in gcs.poll_incoming() {
        let responses = dispatcher.dispatch(&header, &msg, sim_time_us);
        for response in responses {
            gcs.send_message(response);
        }
        dispatcher.process_rc_input(&msg, sim_time_us).await;
    }

    // 2. Execute active mode (produces actuator outputs)
    mode_manager.execute(sim_time_us);

    // 3. Step bridge (actuator → Gazebo physics → sensor update)
    bridge.step().await?;

    // 4. Send heartbeats and telemetry (from core's SystemState)
    let telemetry = dispatcher.update_telemetry(&system_state, sim_time_us);
    for msg in telemetry {
        gcs.send_message(msg);
    }
    gcs.poll_and_heartbeat(sim_time_us);
}
```

### Firmware Import Path Changes

Firmware code updates import paths from firmware-internal to core:

```rust
// Before (firmware-internal)
use crate::core::traits::SharedState;
use crate::communication::mavlink::state::SystemState;
use crate::communication::mavlink::dispatcher::MessageDispatcher;

// After (from core)
use pico_trail_core::traits::sync::SharedState;
use pico_trail_core::autopilot::state::SystemState;
use pico_trail_core::communication::dispatcher::MessageDispatcher;
```

### Data Flow

1. Mission Planner sends MAVLink command (e.g., `RC_CHANNELS_OVERRIDE`)
2. GcsLink receives and passes to core's `MessageDispatcher::dispatch()`
3. `RcInputHandler` normalizes channels and updates global `RC_INPUT` state
4. `ModeManager::execute()` calls active mode's `update(dt)`
5. `ManualMode::update()` reads RC_INPUT, calls `DifferentialDrive::mix()`, writes actuators
6. `SitlPlatform::collect_actuator_commands()` reads PWM channels
7. `GazeboAdapter::send_actuators()` sends motor values to Gazebo
8. Gazebo applies physics, returns sensor data
9. `SitlPlatform::inject_sensors()` updates vehicle state
10. `TelemetryStreamer` builds telemetry from `SystemState`, GcsLink sends to Mission Planner

### Error Handling

- Invalid RC channel values (outside 900-2100): clamp to valid range, log warning
- ARM command when already armed: ACK with `MAV_RESULT_ACCEPTED`
- SET_MODE with unknown mode: ACK with `MAV_RESULT_UNSUPPORTED`
- Mode entry failure: remain in current mode, log error

## Alternatives Considered

See [ADR-00161](../../adr/ADR-00161-sitl-autopilot-integration-layer.md) for full option analysis. The selected approach (embassy-sync in core) was chosen over:

1. **Two-layer approach** (Layer 1 GCS bridge, Layer 2 future migration) — defers autonomous modes, creates temporary duplication
2. **Duplicate in SITL** — ongoing maintenance burden, violates "code unchanged" principle
3. **Feature-gate firmware** — cortex-m/embassy-rp don't compile on host
4. **New intermediate crate** — another crate to maintain, splits core logic

## Testing Strategy

### Core Crate Tests

- SharedState/EmbassyState/MockState: basic access patterns
- SystemState: construction, field access
- MessageDispatcher: message routing to correct handlers
- ParamHandler: PARAM_REQUEST_LIST, PARAM_SET with MockFlash
- CommandHandler: ARM/DISARM, SET_MODE with ACK
- RcInputHandler: channel normalization, DifferentialDrive mixing
- ModeManager: mode transitions, enter/exit lifecycle
- Mode implementations: update logic per mode

### SITL Integration Tests

- `test_closed_loop_control` — LightweightAdapter: send RC override, step, verify actuator commands
- `test_arm_motor_flow` — ARM, send RC, verify non-zero actuator commands
- `test_disarmed_no_output` — DISARM, send RC, verify zero actuator commands
- `test_multi_vehicle_routing` — Commands routed to correct vehicle by system_id
- `test_mode_change` — SET_MODE, verify mode reflected in heartbeat

## Open Questions

- [x] Should `status_notifier` move to core or be abstracted? → **Move to core** — zero embassy deps (uses only `critical_section::Mutex` and `heapless`), required by CommandHandler and MissionHandler which are migrating to core, useful in SITL for GCS notifications
- [x] Should `ParamHandler` remain entirely in firmware, or should param validation logic move to core? → **Move to core** — `FlashInterface` trait is a pure interface (no firmware deps), `ParameterStore` is already in core, `load_from_flash`/`save_to_flash` only depend on `FlashInterface`. Refactor: separate constructor from `register_defaults()` so firmware-specific params (`WifiParams`, `BoardParams`) stay in firmware while `ParamHandler` itself lives in core.
- [x] Which `SystemState` fields need firmware-only initialization vs generic defaults? → **None** — `Default::default()` and `init()` use only constant values. Parameter values (`arming_checks`, `battery_volt_mult`, etc.) are written at runtime by `ParamHandler`, not at construction.
- [ ] Does Mission Planner send `MANUAL_CONTROL` or `RC_CHANNELS_OVERRIDE` for joystick? → Method: Test with live Mission Planner connection
