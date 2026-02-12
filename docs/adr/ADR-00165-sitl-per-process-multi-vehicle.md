# ADR-00165 SITL Per-Process Multi-Vehicle Architecture

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-00150-sitl-multi-vehicle-instances](../requirements/FR-00150-sitl-multi-vehicle-instances.md)
  - [FR-00155-sitl-vehicle-mavlink-ports](../requirements/FR-00155-sitl-vehicle-mavlink-ports.md)
  - [FR-00163-sitl-autopilot-loop-integration](../requirements/FR-00163-sitl-autopilot-loop-integration.md)
- Supersedes ADRs: N/A
- Related ADRs:
  - [ADR-00161-sitl-autopilot-integration-layer](ADR-00161-sitl-autopilot-integration-layer.md)
  - [ADR-00156-sitl-pluggable-adapter-architecture](ADR-00156-sitl-pluggable-adapter-architecture.md)
- Related Tasks:
  - [T-00166-sitl-per-process-multi-vehicle](../tasks/T-00166-sitl-per-process-multi-vehicle/README.md)
- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)

## Context

### Problem

The current `gazebo_bridge` binary runs all vehicles in a single OS process. This causes two critical issues:

**1. Global state prevents multi-vehicle autopilot**

`pico_trail_core` uses process-global statics for autopilot state:

| Global Static     | Location                                    | Purpose                    |
| ----------------- | ------------------------------------------- | -------------------------- |
| `SYSTEM_STATE`    | `core/src/autopilot/state.rs`               | Armed, mode, GPS, attitude |
| `RC_INPUT`        | `core/src/rc/mod.rs`                        | RC channel values          |
| `NOTIFIER`        | `core/src/communication/status_notifier.rs` | STATUSTEXT queue           |
| `MANUAL_RC`       | `sitl/src/autopilot.rs`                     | ManualMode RC copy         |
| `ACTUATOR_OUTPUT` | `sitl/src/autopilot.rs`                     | Motor output               |

Only one vehicle (vehicle 1) can have a `VehicleAutopilot` instance. Vehicles 2 and 3 have no autopilot: they cannot process GCS commands (ARM, SET_MODE, MISSION), run flight modes, or compute navigation.

**2. Duplicate telemetry causes vertical speed oscillation**

Vehicle 1 receives `GLOBAL_POSITION_INT` from two sources with conflicting `relative_alt` values:

- **Core dispatcher** (`TelemetryStreamer`): `relative_alt = 0` (hardcoded)
- **SITL sensor telemetry** (`gcs.send_telemetry()`): `relative_alt = alt` (from GPS)

Mission Planner receives alternating values (\~40000 mm and 0 mm), interprets this as rapid altitude changes, and displays oscillating Vertical Speed (approximately -34 to +35).

The same duplication affects `ATTITUDE` and `GPS_RAW_INT` messages.

### Design Intent vs. Current State

AN-00147 envisions each vehicle running "pico_trail core (unchanged)" independently. FR-00150 requires "Each vehicle has independent autopilot state". The current single-process implementation fails both.

### WSL2 Constraint

The development environment runs SITL inside WSL2 with Mission Planner on Windows. WSL2 port forwarding (`netsh interface portproxy`) supports TCP only — UDP does not pass through. This eliminates the ArduPilot-style "all vehicles send UDP to port 14550" approach.

## Decision

**Each vehicle runs as a separate OS process. Each process has its own TCP MAVLink port. mavp2p aggregates all vehicle connections into a single GCS-facing endpoint.**

### Architecture

```text
WSL2 (Linux)                                              Windows
┌─────────────────────────────────────────────────┐      ┌──────────────┐
│                                                   │      │              │
│  gazebo_bridge        gazebo_bridge        gazebo_bridge │              │
│  --system-id 1        --system-id 2        --system-id 3 │              │
│  --gazebo-port 9002   --gazebo-port 9004   --gazebo-port 9006          │
│  --mavlink-port 5760  --mavlink-port 5762  --mavlink-port 5764         │
│       │                    │                    │        │              │
│       │ TCP                │ TCP                │ TCP    │              │
│       └────────────────────┴────────────────────┘        │              │
│                            │                             │              │
│                     ┌──────┴──────┐                      │              │
│                     │   mavp2p    │                      │              │
│                     │  TCP 5770   │──── portproxy ──────▶│ Mission      │
│                     └─────────────┘                      │ Planner      │
│                                                   │      │              │
└─────────────────────────────────────────────────┘      └──────────────┘
```

### Per-Process Data Flow

Each process follows the canonical SITL → core → GCS path:

```text
Gazebo ──UDP──▶ GazeboAdapter ──▶ SensorData ──▶ SitlPlatform
                                                       │
                                          VehicleAutopilot.update_from_sensors()
                                                       │
                                                 SYSTEM_STATE (process-local)
                                                       │
                                          TelemetryStreamer.update()
                                                       │
                                          GLOBAL_POSITION_INT, ATTITUDE, ...
                                                       │
                                                 GCS (TCP)
```

No SITL-side telemetry bypass (`gcs.send_telemetry()`) — all telemetry flows through core's dispatcher.

### CLI Interface

```bash
# Single vehicle per process
gazebo_bridge \
  --system-id 1 \
  --gazebo-port 9002 \
  --mavlink-port 5760

# mavp2p aggregation
mavp2p \
  tcpc:127.0.0.1:5760 \
  tcpc:127.0.0.1:5762 \
  tcpc:127.0.0.1:5764 \
  tcps:0.0.0.0:5770
```

### Changes Required

| Change                                | Scope                       | Description                                                                                        |
| ------------------------------------- | --------------------------- | -------------------------------------------------------------------------------------------------- |
| `gazebo_bridge` single-vehicle mode   | `sitl/bin/gazebo_bridge.rs` | Accept `--system-id`, `--gazebo-port`, `--mavlink-port`; register one vehicle                      |
| Remove SITL sensor telemetry          | `sitl/gcs/telemetry.rs`     | Remove `build_telemetry`, `build_global_position_int` etc. — core dispatcher handles all telemetry |
| Remove `send_telemetry` from GcsLink  | `sitl/gcs/mod.rs`           | Remove the method and `TelemetrySet`; simplify GcsLink to single-vehicle                           |
| Remove `send_heartbeats` from GcsLink | `sitl/gcs/mod.rs`           | Core dispatcher already sends HEARTBEAT via `TelemetryStreamer`                                    |
| Launch script                         | `scripts/`                  | Script to start N processes + mavp2p                                                               |

### Considered Options

| Option                      | Description                                                     |
| --------------------------- | --------------------------------------------------------------- |
| **A: Per-process model**    | Each vehicle is a separate OS process ⭐ Selected               |
| **B: Instance-based state** | Refactor core globals into instance fields                      |
| **C: Fix duplicate only**   | Remove SITL sensor telemetry for vehicle 1, keep single process |
| **D: UDP shared port**      | All vehicles send UDP to port 14550                             |

### Option Analysis

**Option A: Per-process model** ⭐ Selected

- Pros: Zero core changes, each vehicle has full autopilot, matches ArduPilot SITL model
- Cons: Multiple processes to manage, requires mavp2p for unified GCS
- Effort: 1-2 weeks

**Option B: Instance-based state**

- Pros: Single process, no external tools
- Cons: Requires refactoring core's global statics (SYSTEM_STATE, RC_INPUT, NOTIFIER) into instance fields; breaks embedded design pattern where statics are needed for interrupt-safe access; firmware impact
- Effort: 4+ weeks

**Option C: Fix duplicate telemetry only**

- Pros: Minimal change
- Cons: Vehicles 2,3 still have no autopilot — cannot ARM, change modes, or run missions; does not satisfy FR-00150
- Effort: 1 day

**Option D: UDP shared port**

- Pros: Simplest network architecture, ArduPilot-native
- Cons: Blocked by WSL2 — `netsh portproxy` does not support UDP forwarding
- Effort: 1 week (if WSL2 were not a constraint)

## Rationale

**Option A** is selected because:

1. **Zero core/firmware changes**: Global statics are process-scoped by the OS. Each process gets its own `SYSTEM_STATE`, `RC_INPUT`, etc. without any refactoring.

2. **Full autopilot for every vehicle**: Every vehicle runs the complete core autopilot stack — dispatcher, mode manager, telemetry streamer. No vehicle is a second-class citizen.

3. **Eliminates duplicate telemetry**: With one telemetry source per vehicle (core dispatcher only), the vertical speed oscillation bug is structurally impossible.

4. **ArduPilot precedent**: ArduPilot's `sim_vehicle.py --instance N` runs each vehicle as a separate process. This is a proven model for SITL multi-vehicle.

5. **WSL2 compatible**: TCP per vehicle + mavp2p works with `netsh portproxy`. No UDP required.

### Trade-offs Accepted

| Trade-off                          | Accepted Because                                                                           |
| ---------------------------------- | ------------------------------------------------------------------------------------------ |
| External dependency (mavp2p)       | Single static binary (Go), trivial to install; also useful for mavlink debugging           |
| Multiple processes to manage       | Launch script automates startup/shutdown; process isolation is a feature (crash isolation) |
| More TCP ports consumed            | Localhost ports are plentiful; port assignment is deterministic                            |
| Resource duplication (N processes) | Each process is lightweight (\~10 MB RSS); 3-10 vehicles is the practical range            |

## Consequences

### Positive

1. **Bug fix**: Vertical speed oscillation resolved by eliminating duplicate telemetry
2. **Full multi-vehicle**: All vehicles can ARM, change modes, run missions, and navigate independently
3. **Crash isolation**: One vehicle crashing does not affect others
4. **Simpler per-process code**: `gazebo_bridge` handles exactly one vehicle — no multi-vehicle routing logic
5. **No core changes**: `pico_trail_core` and `pico_trail_firmware` are untouched

### Negative

1. **mavp2p dependency**: Required for single-GCS multi-vehicle; without it, GCS must connect to each vehicle separately
2. **Launch complexity**: Starting N vehicles requires a script or process manager
3. **SITL sensor telemetry removal**: The `build_telemetry` / `build_global_position_int` functions in `sitl/gcs/telemetry.rs` become dead code and should be removed

### Neutral

1. `SitlBridge` multi-vehicle registration still works — it is now used within a single process for one vehicle
2. `GcsLink` multi-vehicle registration (`register_vehicle`) simplifies to single-vehicle but remains compatible
3. Future Option D (UDP) becomes viable if WSL2 gains UDP port forwarding or development moves to native Linux

## Open Questions

- [ ] Should the launch script use a process supervisor (e.g., `supervisord`) or simple shell script with trap for cleanup?
- [ ] Should `GcsLink` be simplified to remove multi-vehicle support entirely, or kept for potential future single-process use?

## External References

- [mavp2p](https://github.com/bluenviron/mavp2p) — Bidirectional MAVLink proxy/router
- [ArduPilot SITL Multi-Vehicle](https://ardupilot.org/dev/docs/sitl-with-multiple-vehicles.html) — Per-process model
- [WSL2 Networking](https://learn.microsoft.com/en-us/windows/wsl/networking) — Port forwarding limitations
