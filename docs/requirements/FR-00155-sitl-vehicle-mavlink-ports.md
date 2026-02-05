# FR-00155 SITL Vehicle MAVLink Ports

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Prerequisite Requirements:
  - [FR-00150-sitl-multi-vehicle-instances](FR-00150-sitl-multi-vehicle-instances.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00160-sitl-multi-vehicle-lockstep-ci](../tasks/T-00160-sitl-multi-vehicle-lockstep-ci/README.md)

## Requirement Statement

Each vehicle instance shall support independent MAVLink communication on configurable UDP ports. This enables GCS applications to connect to and control multiple vehicles simultaneously.

## Rationale

Multi-vehicle testing requires each vehicle to have its own MAVLink connection. Mission Planner and QGroundControl support connecting to multiple vehicles on different ports. Following ArduPilot's convention (14550 + vehicle_index) ensures compatibility.

## User Story (if applicable)

As an operator testing 3 rovers in SITL, I want to connect Mission Planner to all 3 vehicles simultaneously so that I can monitor and control them from a single GCS session.

## Acceptance Criteria

- [ ] Each vehicle has configurable MAVLink UDP port in `VehicleConfig`
- [ ] Default port assignment: 14550 + vehicle_index
- [ ] MAVLink HEARTBEAT sent on each vehicle's port
- [ ] Parameter read/write works independently per vehicle
- [ ] Mission upload/download works independently per vehicle
- [ ] Telemetry streaming on each vehicle's port
- [ ] System ID in MAVLink messages matches `VehicleId`
- [ ] GCS can connect to multiple ports simultaneously
- [ ] Port conflicts detected and reported as error
- [ ] Unit tests for multi-port MAVLink handling

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
Vehicle ID  | System ID | UDP Port
------------|-----------|----------
VehicleId(1)|     1     |  14550
VehicleId(2)|     2     |  14551
VehicleId(3)|     3     |  14552
...         |   ...     |   ...
```

### Multi-Port Listener

```rust
impl SitlBridge {
    async fn setup_mavlink(&mut self) -> Result<(), SitlError> {
        for (id, vehicle) in &mut self.vehicles {
            let port = vehicle.config.mavlink_port;
            let socket = UdpSocket::bind(format!("0.0.0.0:{}", port)).await?;
            vehicle.mavlink_socket = Some(socket);
        }
        Ok(())
    }

    async fn handle_mavlink(&mut self) {
        // Use tokio::select! to handle messages from all ports
        loop {
            tokio::select! {
                // For each vehicle's socket...
            }
        }
    }
}
```

### GCS Connection Example

```bash
# Connect Mission Planner to 3 vehicles
# Vehicle 1: UDP 14550
# Vehicle 2: UDP 14551
# Vehicle 3: UDP 14552

# In QGroundControl: Add multiple UDP links
```

## Platform Considerations

### Host Only

- Uses async UDP (tokio/async-std)
- Each vehicle gets its own socket
- No port sharing - each vehicle is independent

### GCS Compatibility

- Mission Planner: Supports multiple UDP connections
- QGroundControl: Add multiple comm links
- MAVProxy: `--master=udp:127.0.0.1:14550 --master=udp:127.0.0.1:14551`

## Risks & Mitigation

| Risk                    | Impact | Likelihood | Mitigation                        | Validation         |
| ----------------------- | ------ | ---------- | --------------------------------- | ------------------ |
| Port already in use     | Medium | Medium     | Detect and report clear error     | Startup validation |
| Firewall blocking ports | Low    | Low        | Document required ports           | Setup guide        |
| Message routing errors  | Medium | Low        | Strict vehicle_id ↔ port mapping | Integration tests  |

## Implementation Notes

- Consider supporting TCP as well as UDP for GCS connection
- MAVLink component ID can remain 1 (autopilot) for all vehicles
- Log which ports are active at startup

## External References

- [MAVLink UDP Setup](https://mavlink.io/en/mavgen_python/howto_requestmessages.html)
- [Mission Planner Multiple Vehicles](https://ardupilot.org/planner/docs/mission-planner-swarm.html)
