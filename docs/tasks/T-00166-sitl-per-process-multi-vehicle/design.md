# T-00166 SITL Per-Process Multi-Vehicle Design

## Metadata

- Type: Design
- Status: Completed

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

Refactor `gazebo_bridge` from multi-vehicle single-process to single-vehicle per-process. Remove SITL-side sensor telemetry bypass so all telemetry flows through core's `TelemetryStreamer`. Add a launch script to start N processes with mavp2p aggregation for unified GCS access.

## Success Metrics

- [ ] `gazebo_bridge` accepts `--system-id`, `--gazebo-port`, `--mavlink-port` and runs exactly one vehicle
- [ ] SITL sensor telemetry functions removed (`build_telemetry`, `build_global_position_int`, etc.)
- [ ] `send_telemetry`, `send_heartbeats` removed from `GcsLink`
- [ ] No duplicate ATTITUDE/GPS/HEARTBEAT messages observed per vehicle
- [ ] Launch script starts N processes + mavp2p; all vehicles visible in Mission Planner
- [ ] All SITL tests pass

## Background and Current State

### Problems (per ADR-00165)

**1. Global state prevents multi-vehicle autopilot**

`pico_trail_core` uses process-global statics for autopilot state:

| Global Static     | Location                                    | Purpose                    |
| ----------------- | ------------------------------------------- | -------------------------- |
| `SYSTEM_STATE`    | `core/src/autopilot/state.rs`               | Armed, mode, GPS, attitude |
| `RC_INPUT`        | `core/src/rc/mod.rs`                        | RC channel values          |
| `NOTIFIER`        | `core/src/communication/status_notifier.rs` | STATUSTEXT queue           |
| `MANUAL_RC`       | `sitl/src/autopilot.rs`                     | ManualMode RC copy         |
| `ACTUATOR_OUTPUT` | `sitl/src/autopilot.rs`                     | Motor output               |

Only vehicle 1 gets a `VehicleAutopilot`. Vehicles 2-3 have no autopilot and cannot ARM, change modes, or navigate.

**2. Duplicate telemetry causes vertical speed oscillation**

Vehicle 1 receives `GLOBAL_POSITION_INT` from two sources:

- **Core TelemetryStreamer**: `relative_alt = 0` (hardcoded default)
- **SITL `gcs.send_telemetry()`**: `relative_alt = alt` (from GPS sensors)

Mission Planner sees alternating values and displays rapid vertical speed oscillation.

### Current `gazebo_bridge` Architecture

```text
Single process
├── SitlBridge (multi-vehicle)
│   ├── Vehicle 1 ← VehicleAutopilot (full autopilot)
│   ├── Vehicle 2 ← no autopilot
│   └── Vehicle 3 ← no autopilot
├── GcsLink (single TCP port, multi-vehicle heartbeats)
│   ├── send_heartbeats() ← SITL heartbeats (conflicts with core)
│   └── send_telemetry()  ← SITL sensor telemetry (conflicts with core)
└── Core TelemetryStreamer ← dispatcher telemetry (correct source)
```

## Proposed Design

### Target Architecture

```text
Process 1 (system-id=1)         Process 2 (system-id=2)         Process 3 (system-id=3)
├── SitlBridge (1 vehicle)      ├── SitlBridge (1 vehicle)      ├── SitlBridge (1 vehicle)
│   └── Vehicle 1               │   └── Vehicle 2               │   └── Vehicle 3
├── VehicleAutopilot            ├── VehicleAutopilot            ├── VehicleAutopilot
├── GcsLink (TCP 5760)          ├── GcsLink (TCP 5762)          ├── GcsLink (TCP 5764)
└── Core TelemetryStreamer      └── Core TelemetryStreamer      └── Core TelemetryStreamer
         │                               │                               │
         └───────────────────────────────┴───────────────────────────────┘
                                         │
                                    mavp2p (TCP 5770)
                                         │
                                    Mission Planner
```

Each process has its own `SYSTEM_STATE`, `RC_INPUT`, etc. — isolated by OS process boundaries. All telemetry flows through core's `TelemetryStreamer` only; no SITL sensor telemetry bypass.

### CLI Changes

**Current:**

```bash
gazebo_bridge -n 3 --gazebo-port-base 9002 --port-stride 10 --mavlink-port 14550
```

**New:**

```bash
gazebo_bridge --system-id 1 --gazebo-port 9002 --mavlink-port 5760
```

| Flag             | Type | Default | Description                         |
| ---------------- | ---- | ------- | ----------------------------------- |
| `--system-id`    | u8   | 1       | MAVLink system ID for this vehicle  |
| `--gazebo-port`  | u16  | 9002    | UDP port for Gazebo communication   |
| `--mavlink-port` | u16  | 5760    | TCP port for MAVLink GCS connection |

### GcsLink Simplification

Remove multi-vehicle support:

**Remove:**

- `VehicleState` struct (per-vehicle rate-limiting timestamps)
- `vehicles: Vec<VehicleState>` field
- `register_vehicle()` method
- `send_heartbeats()` / `send_heartbeats_with()` methods (core dispatcher sends HEARTBEAT)
- `send_telemetry()` method (core dispatcher sends all telemetry)
- Heartbeat/telemetry interval constants (`HEARTBEAT_INTERVAL_US`, `ATTITUDE_INTERVAL_US`, `POSITION_INTERVAL_US`, `SYS_STATUS_INTERVAL_US`)

**Keep:**

- `new(port)` — TCP listener setup
- `try_accept()` / `poll_incoming()` — message reception
- `send_message_as(system_id, msg)` — message sending
- `is_connected()` — connection status
- `parse_mavlink_frame()` — frame parsing

### Telemetry Module Cleanup

Remove from `crates/sitl/src/gcs/telemetry.rs`:

| Function                       | Reason for Removal                                |
| ------------------------------ | ------------------------------------------------- |
| `build_telemetry()`            | Replaced by core's `TelemetryStreamer`            |
| `build_heartbeat()`            | Core dispatcher sends HEARTBEAT                   |
| `build_heartbeat_with_state()` | Core dispatcher sends state-aware HEARTBEAT       |
| `build_attitude()`             | Core TelemetryStreamer builds ATTITUDE            |
| `build_gps_raw_int()`          | Core TelemetryStreamer builds GPS_RAW_INT         |
| `build_global_position_int()`  | Core TelemetryStreamer builds GLOBAL_POSITION_INT |
| `build_sys_status()`           | Core TelemetryStreamer builds SYS_STATUS          |
| `gps_fix_to_mav()`             | No longer needed without SITL telemetry           |
| `TelemetrySet` struct          | No longer needed without `build_telemetry()`      |

The telemetry module can be removed entirely if no functions remain.

### `gazebo_bridge` Main Loop Simplification

```rust
#[tokio::main(flavor = "current_thread")]
async fn main() {
    let args = parse_args(); // --system-id, --gazebo-port, --mavlink-port

    let mut bridge = SitlBridge::new();
    let mut gcs = GcsLink::new(args.mavlink_port)?;

    // Register single vehicle
    let id = VehicleId(args.system_id);
    let adapter = GazeboAdapter::new("gazebo", id, GazeboConfig { ... });
    bridge.register_adapter(Box::new(adapter))?;
    bridge.spawn_vehicle(VehicleConfig::new(id, VehicleType::Rover))?;
    bridge.assign_vehicle_to_adapter(id, "gazebo")?;
    // Connect adapter, setup PWM...

    let mut autopilot = VehicleAutopilot::new(args.system_id);

    loop {
        // 1. Poll GCS → dispatch through core's MessageDispatcher
        for (header, msg) in gcs.poll_incoming() {
            let responses = autopilot.dispatch(&header, &msg, wall_us);
            for response in &responses {
                gcs.send_message_as(args.system_id, response);
            }
            autopilot.process_rc_input(&msg, wall_us).await;
        }

        // 2. Step bridge (Gazebo physics)
        bridge.step().await?;

        // 3. Update SYSTEM_STATE from sensors
        if let Some(sensors) = bridge.get_vehicle(id).and_then(|v| v.platform.peek_sensors()) {
            autopilot.update_from_sensors(&sensors);
        }

        // 4. Execute active mode
        autopilot.execute_mode(wall_us);

        // 5. Apply actuator outputs
        if let Some(vehicle) = bridge.get_vehicle(id) {
            autopilot.apply_actuators_to_platform(&vehicle.platform);
        }

        // 6. Send telemetry (core dispatcher only — no SITL bypass)
        let telemetry = autopilot.update_telemetry(wall_us);
        for msg in &telemetry {
            gcs.send_message_as(args.system_id, msg);
        }
    }
}
```

Key differences from current:

- No multi-vehicle loop (`for i in 1..=args.count`)
- No `gcs.send_heartbeats()` call (core dispatcher handles heartbeats)
- No `gcs.send_telemetry()` call (core dispatcher handles all telemetry)
- No `gcs.register_vehicle()` call

### Launch Script

`scripts/sitl-multi-vehicle.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

COUNT=${1:-3}
GAZEBO_PORT_BASE=${GAZEBO_PORT_BASE:-9002}
GAZEBO_PORT_STRIDE=${GAZEBO_PORT_STRIDE:-10}
MAVLINK_PORT_BASE=${MAVLINK_PORT_BASE:-5760}
MAVLINK_PORT_STRIDE=${MAVLINK_PORT_STRIDE:-2}
MAVP2P_PORT=${MAVP2P_PORT:-5770}

PIDS=()
cleanup() {
    echo "Stopping all processes..."
    for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
    wait
}
trap cleanup EXIT INT TERM

for i in $(seq 1 "$COUNT"); do
    SYS_ID=$i
    GAZEBO_PORT=$((GAZEBO_PORT_BASE + (i - 1) * GAZEBO_PORT_STRIDE))
    MAVLINK_PORT=$((MAVLINK_PORT_BASE + (i - 1) * MAVLINK_PORT_STRIDE))

    cargo run --bin gazebo_bridge -- \
        --system-id "$SYS_ID" \
        --gazebo-port "$GAZEBO_PORT" \
        --mavlink-port "$MAVLINK_PORT" &
    PIDS+=($!)
    echo "Vehicle $i: system-id=$SYS_ID, gazebo=$GAZEBO_PORT, mavlink=TCP:$MAVLINK_PORT"
done

# Build mavp2p endpoint list
MAVP2P_ARGS=()
for i in $(seq 1 "$COUNT"); do
    PORT=$((MAVLINK_PORT_BASE + (i - 1) * MAVLINK_PORT_STRIDE))
    MAVP2P_ARGS+=("tcpc:127.0.0.1:$PORT")
done
MAVP2P_ARGS+=("tcps:0.0.0.0:$MAVP2P_PORT")

echo "Starting mavp2p on TCP:$MAVP2P_PORT..."
mavp2p "${MAVP2P_ARGS[@]}" &
PIDS+=($!)

echo "All vehicles running. Connect GCS to TCP:$MAVP2P_PORT"
wait
```

### Data Flow (Per-Process)

```text
Gazebo ──UDP──> GazeboAdapter ──> SensorData ──> SitlPlatform
                                                       │
                                          VehicleAutopilot.update_from_sensors()
                                                       │
                                                 SYSTEM_STATE (process-local)
                                                       │
                                          Core TelemetryStreamer.update()
                                                       │
                                          GLOBAL_POSITION_INT, ATTITUDE, HEARTBEAT, ...
                                                       │
                                              GcsLink.send_message_as()
                                                       │
                                                 TCP ──> mavp2p ──> Mission Planner
```

No SITL telemetry bypass — all telemetry through core's dispatcher.

### Error Handling

- Port-in-use: `GcsLink::new()` already returns `Err` on bind failure — no change needed
- Gazebo connection failure: existing retry logic in adapter — no change needed
- mavp2p not installed: launch script checks for `mavp2p` binary and prints install instructions

## Alternatives Considered

See [ADR-00165](../../adr/ADR-00165-sitl-per-process-multi-vehicle.md) for full analysis:

1. **Instance-based state** — Refactor core globals into instance fields (4+ weeks, firmware impact)
2. **Fix duplicate telemetry only** — Vehicles 2-3 still have no autopilot
3. **UDP shared port** — Blocked by WSL2 port forwarding limitation

## Testing Strategy

### Unit Tests

- `GcsLink` tests updated: remove `test_register_vehicle`, `test_multi_vehicle_heartbeats`
- `GcsLink` tests added: verify `send_message_as` works without prior `register_vehicle`
- CLI parsing: verify `--system-id`, `--gazebo-port`, `--mavlink-port` accepted

### Integration Tests

- `test_single_vehicle_closed_loop` — ARM, send RC, step, verify actuator commands and telemetry (single vehicle per process)
- Existing autopilot integration tests remain valid (single-vehicle focus)

### Manual Validation

- Start 3 processes via launch script + mavp2p
- Connect Mission Planner to mavp2p port
- Verify all 3 vehicles visible with independent ARM/mode/telemetry
- Verify no vertical speed oscillation
- Verify joystick control moves each vehicle independently

## Open Questions

- [ ] Should the launch script use `cargo run` or pre-built binaries? (affects startup time)
- [ ] Should `GcsLink` multi-vehicle support be removed entirely, or kept but unused? (ADR-00165 leaves this open)
