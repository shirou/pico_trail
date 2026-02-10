# FR-00155 SITL Vehicle MAVLink Ports

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Prerequisite Requirements:
  - [FR-00150-sitl-multi-vehicle-instances](FR-00150-sitl-multi-vehicle-instances.md)
- Dependent Requirements:
  - [FR-00162-sitl-gcs-command-reception](FR-00162-sitl-gcs-command-reception.md)
- Related Tasks:
  - [T-00160-sitl-multi-vehicle-lockstep-ci](../tasks/T-00160-sitl-multi-vehicle-lockstep-ci/README.md)
  - [T-00166-sitl-per-process-multi-vehicle](../tasks/T-00166-sitl-per-process-multi-vehicle/README.md)

## Requirement Statement

Each vehicle instance shall support independent MAVLink communication on configurable TCP ports. This enables GCS applications to connect to and control multiple vehicles simultaneously via mavp2p aggregation.

## Rationale

Multi-vehicle testing requires each vehicle to have its own MAVLink connection. Mission Planner and QGroundControl support connecting to multiple vehicles on different ports. Following ArduPilot's convention (14550 + vehicle_index) ensures compatibility.

## User Story (if applicable)

As an operator testing 3 rovers in SITL, I want to connect Mission Planner to all 3 vehicles simultaneously so that I can monitor and control them from a single GCS session.

## Acceptance Criteria

- [x] Each vehicle has configurable MAVLink TCP port in `VehicleConfig`
- [x] Default port assignment: 14550 + vehicle_index
- [x] MAVLink HEARTBEAT sent on each vehicle's port (per-process model)
- [x] Parameter read/write works independently per vehicle
- [x] Mission upload/download works independently per vehicle
- [x] Telemetry streaming on each vehicle's port (per-process model)
- [x] System ID in MAVLink messages matches `VehicleId`
- [ ] GCS can connect to multiple ports simultaneously (requires manual validation with Mission Planner + mavp2p)
- [x] Port conflicts detected and reported as error
- [x] Unit tests for multi-port MAVLink handling

## Technical Details (if applicable)

### Port Assignment

```rust
pub struct VehicleConfig {
    pub id: VehicleId,
    pub mavlink_port: u16,  // default: 14550 + id.0 as u16
    // ...
}

impl Default for VehicleConfig {
    fn default() -> Self {
        Self {
            id: VehicleId(1),
            mavlink_port: 14550,
            // ...
        }
    }
}
```

### MAVLink System ID Mapping

```text
Vehicle ID  | System ID | TCP Port
------------|-----------|----------
VehicleId(1)|     1     |  5760
VehicleId(2)|     2     |  5762
VehicleId(3)|     3     |  5764
...         |   ...     |   ...
```

### Per-Process Architecture

Each vehicle runs as a separate OS process with its own TCP port. mavp2p aggregates all vehicle connections into a single GCS endpoint.

```bash
# Start 3 vehicles via launch script
scripts/sitl-multi-vehicle.sh 3

# Or manually:
gazebo_bridge --system-id 1 --gazebo-port 9002 --mavlink-port 5760 &
gazebo_bridge --system-id 2 --gazebo-port 9012 --mavlink-port 5762 &
gazebo_bridge --system-id 3 --gazebo-port 9022 --mavlink-port 5764 &

# Aggregate via mavp2p
mavp2p tcpc:127.0.0.1:5760 tcpc:127.0.0.1:5762 tcpc:127.0.0.1:5764 tcps:0.0.0.0:5770

# Connect Mission Planner to TCP:5770
```

## Platform Considerations

### Host Only

- Uses TCP via `GcsLink` (TcpListener/TcpStream)
- Each vehicle process gets its own TCP port
- mavp2p aggregates multiple TCP connections for GCS access
- TCP chosen over UDP for WSL2 compatibility (UDP port forwarding unreliable)

### GCS Compatibility

- Mission Planner: Connect via TCP to mavp2p aggregation port
- QGroundControl: Add TCP comm link to mavp2p port
- MAVProxy: `--master=tcp:127.0.0.1:5770`

## Risks & Mitigation

| Risk                    | Impact | Likelihood | Mitigation                        | Validation         |
| ----------------------- | ------ | ---------- | --------------------------------- | ------------------ |
| Port already in use     | Medium | Medium     | Detect and report clear error     | Startup validation |
| Firewall blocking ports | Low    | Low        | Document required ports           | Setup guide        |
| Message routing errors  | Medium | Low        | Strict vehicle_id ↔ port mapping | Integration tests  |

## Implementation Notes

- TCP is used instead of UDP for WSL2 compatibility (per ADR-00165)
- Per-process model provides full autopilot state isolation (ADR-00165)
- MAVLink component ID remains 1 (autopilot) for all vehicles
- Each process logs its active port at startup

## External References

- [mavp2p](https://github.com/bluenviron/mavp2p) — MAVLink proxy for TCP aggregation
- [Mission Planner Multiple Vehicles](https://ardupilot.org/planner/docs/mission-planner-swarm.html)
