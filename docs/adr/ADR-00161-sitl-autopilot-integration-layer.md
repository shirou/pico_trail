# ADR-00161 SITL Autopilot Integration Layer

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-00154-sitl-platform-trait](../requirements/FR-00154-sitl-platform-trait.md)
  - [FR-00162-sitl-gcs-command-reception](../requirements/FR-00162-sitl-gcs-command-reception.md)
  - [FR-00163-sitl-autopilot-loop-integration](../requirements/FR-00163-sitl-autopilot-loop-integration.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-00164-sitl-autopilot-core-integration](../tasks/T-00164-sitl-autopilot-core-integration/README.md)
  - [T-00166-sitl-per-process-multi-vehicle](../tasks/T-00166-sitl-per-process-multi-vehicle/README.md)

## Context

### Problem

The SITL `gazebo_bridge` currently forwards sensor data from Gazebo to Mission Planner as telemetry, but has **no autopilot logic** running. Mission Planner cannot command the vehicle because:

1. No GCS command processing (SET_MODE, RC_CHANNELS_OVERRIDE, MISSION\_\*, ARM/DISARM)
2. No mode management (Manual, Guided, Auto, RTL)
3. No navigation/PID control loop
4. No motor output computation — actuator commands are always neutral

AN-00147 envisions each vehicle running "pico_trail core (unchanged)" and FR-00154 requires "All rover modes work in SITL with any adapter". The current implementation does not satisfy these requirements.

### Architectural Boundary

The autopilot logic is split across `core` (portable) and `firmware` (embedded-only):

| Component                            | Current Location | Embassy Dependency                                 | Portable? |
| ------------------------------------ | ---------------- | -------------------------------------------------- | --------- |
| `Mode` trait                         | `core`           | None                                               | Yes       |
| Navigation, AHRS, kinematics         | `core`           | None                                               | Yes       |
| `MessageDispatcher` + handlers       | `firmware`       | None                                               | Yes       |
| `SystemState`                        | `firmware`       | `defmt::Format` (optional)                         | Mostly    |
| `VehicleType` trait                  | `firmware`       | None                                               | Yes       |
| `SharedState` trait + `EmbassyState` | `firmware`       | `embassy_sync::Mutex`                              | See below |
| `ModeManager`                        | `firmware`       | firmware `SystemState` ref                         | Mostly    |
| Mode implementations (Manual)        | `firmware`       | `embassy_sync::Mutex`, `embassy_futures::block_on` | See below |
| Mode implementations (others)        | `firmware`       | None                                               | Yes       |
| `Platform` trait                     | `firmware`       | Embedded HAL associated types                      | **No**    |

The `firmware` crate depends on `cortex-m`, `embassy-rp`, `cyw43`, making it impossible for SITL to import any types from it.

### Key Insight: embassy-sync Is Already Portable

Analysis of the embassy dependencies used by the autopilot logic reveals:

- **`embassy-sync`**: Built on top of `critical-section`, which core **already depends on**. `Mutex<CriticalSectionRawMutex, T>` works on any target (embedded or std) as long as a critical-section implementation is available. Core already has `critical-section = { features = ["std"] }` in dev-dependencies for host tests.
- **`embassy-futures`**: Pure utility crate (`block_on`, `select`). no_std compatible, zero platform-specific code.
- **`embassy-time`**: Requires a platform-specific time driver. Used only by task-level code (parameter saver, arming tasks), **not** by autopilot core logic.

This means the embassy barrier is artificial — `embassy-sync` and `embassy-futures` are not embedded-specific crates; they are generic async synchronization primitives that happen to work on embedded targets.

### Constraints

- `pico_trail_core` must remain `no_std` compatible (embassy-sync and embassy-futures are no_std)
- `pico_trail_firmware` must not be structurally changed (only import paths change)
- SITL runs on host with `tokio`
- `embassy-time` dependent code (parameter saver, arming tasks) stays in firmware

### Prior Art

- T-00036 migrated pure algorithms (DCM, navigation, AHRS) from firmware to core
- T-00041 migrated parameter store to core
- Both established the pattern: identify platform-independent logic in firmware, move to core

## Decision

**We will add `embassy-sync` and `embassy-futures` as optional dependencies of `pico_trail_core` behind an `embassy` feature flag, then migrate the autopilot integration layer from firmware to core.**

### Core Feature Configuration

```toml
# crates/core/Cargo.toml
[features]
default = []
embassy = ["dep:embassy-sync", "dep:embassy-futures"]

[dependencies]
embassy-sync = { git = "https://github.com/embassy-rs/embassy", optional = true }
embassy-futures = { git = "https://github.com/embassy-rs/embassy", optional = true }
```

### Downstream Crate Configuration

```toml
# crates/firmware/Cargo.toml
pico_trail_core = { path = "../core", features = ["embassy", "defmt"] }

# crates/sitl/Cargo.toml
pico_trail_core = { path = "../core", features = ["embassy"] }
```

### What Migrates to Core

| Component                                          | Source                                        | Destination                      |
| -------------------------------------------------- | --------------------------------------------- | -------------------------------- |
| `SharedState` trait + `EmbassyState` + `MockState` | `firmware/src/core/traits/sync.rs`            | `core/src/traits/sync.rs`        |
| `FlashInterface` trait                             | `firmware/src/platform/traits/flash.rs`       | `core/src/traits/flash.rs`       |
| `SystemState`                                      | `firmware/src/communication/mavlink/state.rs` | `core/src/autopilot/state.rs`    |
| `GpsPosition`                                      | `firmware/src/devices/gps.rs`                 | `core/src/navigation/types.rs`   |
| `VehicleType` trait                                | `firmware/src/communication/mavlink/vehicle/` | `core/src/autopilot/vehicle.rs`  |
| `MessageDispatcher` + all handlers                 | `firmware/src/communication/mavlink/`         | `core/src/communication/`        |
| `status_notifier`                                  | `firmware/src/communication/mavlink/`         | `core/src/communication/`        |
| `load_from_flash` / `save_to_flash`                | `firmware/src/parameters/storage.rs`          | `core/src/parameters/storage.rs` |
| `ModeManager`                                      | `firmware/src/rover/mode_manager.rs`          | `core/src/mode/mode_manager.rs`  |
| Mode implementations                               | `firmware/src/rover/mode/`                    | `core/src/mode/`                 |

### What Stays in Firmware

| Component                           | Reason                                            |
| ----------------------------------- | ------------------------------------------------- |
| `Platform` trait + RP2350 impl      | Embedded HAL associated types                     |
| Device drivers (GPS, IMU, etc.)     | Hardware-specific                                 |
| Flash implementations (RP2350Flash) | Platform-specific; `FlashInterface` trait in core |
| Embassy tasks (saver, arming)       | `embassy-time` dependency                         |
| Transport layers (UART, UDP, WiFi)  | Platform-specific I/O                             |
| `ActuatorInterface` impl            | Platform-specific PWM                             |
| `WifiParams`, `BoardParams`         | Firmware-specific parameter defaults              |

### Decision Drivers

1. **Single source of truth**: One set of mode implementations, one MessageDispatcher
2. **Full SITL compliance**: All rover modes work in SITL (FR-00154)
3. **No duplication**: Eliminates need for SITL-specific command handling
4. **Host testability**: Full autopilot stack testable with `cargo test`
5. **Minimal API change**: embassy-sync and embassy-futures are no_std — core remains no_std

### Considered Options

| Option                        | Description                                                           |
| ----------------------------- | --------------------------------------------------------------------- |
| **A: embassy-sync in core**   | Add embassy-sync/futures to core, migrate autopilot layer ⭐ Selected |
| **B: Two-layer approach**     | GCS command bridge in SITL now, autopilot migration later             |
| **C: Duplicate in SITL**      | Rewrite dispatcher, modes, state in SITL crate                        |
| **D: Feature-gate firmware**  | Add `std` feature to firmware for host compilation                    |
| **E: New intermediate crate** | Create `pico_trail_autopilot` between core and firmware/sitl          |

### Option Analysis

**Option A: embassy-sync in core** ⭐ Selected

- Pros: Single source of truth, full SITL compliance, host testable, follows T-00036/T-00041 pattern
- Cons: Core gains optional embassy dependency, migration scope is larger upfront
- Effort: 3-4 weeks

**Option B: Two-layer approach**

- Pros: Immediate partial value (manual control only)
- Cons: Defers autonomous modes, temporary duplication, two separate tasks
- Effort: 1-2 weeks (Layer 1) + 3-4 weeks (Layer 2)

**Option C: Duplicate in SITL**

- Pros: No changes to existing crates
- Cons: Violates "code unchanged" principle, ongoing maintenance burden
- Effort: 3-4 weeks, plus ongoing cost

**Option D: Feature-gate firmware**

- Pros: Reuse everything as-is
- Cons: cortex-m/embassy-rp don't compile on host, massive conditional compilation
- Effort: 4+ weeks, high risk

**Option E: New intermediate crate**

- Pros: Clean separation
- Cons: Another crate to maintain, splits what is logically "core" business logic
- Effort: 4-5 weeks

## Rationale

**Option A** is selected because:

1. **embassy-sync is not an embedded dependency**: It is built on `critical-section`, which core already uses. Adding it as optional does not compromise core's no_std purity.

2. **Eliminates the need for layered approach**: Unlike Option B, all modes (Manual, Auto, Guided, RTL) work from day one. No temporary workarounds.

3. **Proven migration pattern**: T-00036 and T-00041 established firmware→core migration. This ADR extends the same pattern to the remaining autopilot logic that was deferred.

4. **Host test coverage**: The full autopilot stack (dispatcher → mode manager → modes → actuator output) becomes testable with `cargo test -p pico_trail_core --features embassy --lib`.

5. **SITL directly benefits**: `pico_trail_sitl` imports `pico_trail_core` with `embassy` feature and uses the same `MessageDispatcher`, `ModeManager`, and mode implementations as firmware.

### Trade-offs Accepted

| Trade-off                                   | Accepted Because                                                    |
| ------------------------------------------- | ------------------------------------------------------------------- |
| Core gains optional embassy-sync dependency | embassy-sync is no_std, built on critical-section core already uses |
| Larger upfront migration scope              | Eliminates need for separate Layer 2 task                           |
| embassy feature flag complexity             | Feature is optional; core without `embassy` is unchanged            |
| Git dependency (embassy-sync)               | Firmware already uses embassy git deps; consistent approach         |

## Consequences

### Positive

1. **Full SITL autopilot**: All rover modes work in SITL — Manual, Auto, Guided, RTL, Loiter, Circle, SmartRTL
2. **Single source of truth**: Mode implementations, dispatcher, state in one place
3. **Host testability**: `cargo test -p pico_trail_core --features embassy --lib` tests the full autopilot
4. **No throwaway work**: All migration work is permanent, no temporary bridges
5. **SITL simplification**: GcsLink delegates to core's MessageDispatcher instead of reimplementing

### Negative

1. **Core gains optional dependencies**: embassy-sync, embassy-futures (git deps)
2. **Migration effort**: Moving and adapting code across crate boundaries
3. **Firmware import path changes**: Existing firmware code must update imports

### Neutral

1. Core without `embassy` feature is completely unchanged
2. embassy-sync is maintained by the Embassy project (active, well-maintained)

## Implementation Notes

### embassy-sync on std Targets

`embassy-sync::Mutex<CriticalSectionRawMutex, T>` works on std because:

1. `CriticalSectionRawMutex` delegates to `critical-section` crate
2. `critical-section` with `std` feature uses `std::sync::Mutex` internally
3. Core already has `critical-section = { features = ["std"] }` in dev-dependencies
4. `embassy_futures::block_on` is a simple polling loop — no runtime needed

```rust
// This works identically on embedded (cortex-m critical section)
// and on host (std mutex critical section):
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

static STATE: Mutex<CriticalSectionRawMutex, RefCell<u32>> = Mutex::new(RefCell::new(0));

// In mode update():
embassy_futures::block_on(async {
    let guard = STATE.lock().await;
    // access state
});
```

### Migration Sequence

1. Add embassy-sync/futures to core Cargo.toml (optional)
2. Move `SharedState` trait + implementations to core
3. Move `SystemState` and related types to core
4. Move `VehicleType` trait to core
5. Move `MessageDispatcher` + handlers to core
6. Move `ModeManager` to core
7. Move mode implementations to core
8. Update firmware imports to use core modules
9. Update SITL to use core's autopilot types
10. Verify embedded build and all tests

### SITL Integration

```rust
// In SITL vehicle setup:
use pico_trail_core::communication::MessageDispatcher;
use pico_trail_core::mode::ModeManager;

// Each vehicle gets a dispatcher and mode manager from core
let dispatcher = MessageDispatcher::new(vehicle_type);
let mode_manager = ModeManager::new(system_state);

// GcsLink delegates to core dispatcher
for (header, msg) in gcs.poll_incoming() {
    let responses = dispatcher.dispatch(&header, &msg);
    for response in responses {
        gcs.send_message(response);
    }
}

// Mode manager runs in the step loop
mode_manager.execute(sim_time_us);
```

## Open Questions

- [x] Can embassy-sync work on std targets? → **Yes** — via critical-section std feature
- [ ] Should `ActuatorInterface` trait move to core or remain firmware-specific? → Method: Check if SITL can implement it without firmware dependencies
- [x] Which `SystemState` fields depend on firmware-only types (e.g., `GpsPosition`)? → **`GpsPosition` moves to core** — needed for geo fence, navigation, and other position-based logic beyond just telemetry. `GpsFixType` is already in core; `GpsPosition` only depends on it.

## External References

- [Embassy Sync Crate](https://docs.embassy.dev/embassy-sync/) — Synchronization primitives for embedded and std
- [critical-section Crate](https://docs.rs/critical-section/) — Platform-agnostic critical sections
- [ArduPilot SITL Architecture](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
