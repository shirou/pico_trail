# FR-00150 SITL Multi-Vehicle Instances

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Prerequisite Requirements:
  - [FR-00149-sitl-multi-adapter-registration](FR-00149-sitl-multi-adapter-registration.md)
- Dependent Requirements:
  - [FR-00155-sitl-vehicle-mavlink-ports](FR-00155-sitl-vehicle-mavlink-ports.md)
- Related Tasks:
  - [T-00160-sitl-multi-vehicle-lockstep-ci](../tasks/T-00160-sitl-multi-vehicle-lockstep-ci/README.md)

## Requirement Statement

SITL shall support multiple vehicle instances, each with independent autopilot state. Each vehicle shall be identified by a unique `VehicleId` and can be assigned to any registered simulator adapter.

## Rationale

Multi-vehicle support enables:

1. Formation flying/driving scenarios
2. Swarm behavior testing
3. Vehicle-to-vehicle coordination (future)
4. Parallel testing of different configurations

## User Story (if applicable)

As a developer testing convoy behavior, I want to run 3 rovers simultaneously in the same simulation world so that I can verify inter-vehicle coordination.

## Acceptance Criteria

- [ ] `VehicleId` type defined as `u8` (0-254, matching MAVLink system ID range)
- [ ] `spawn_vehicle()` creates a new vehicle instance with unique ID
- [ ] `despawn_vehicle()` removes a vehicle instance
- [ ] Each vehicle has independent autopilot state (modes, navigation, parameters)
- [ ] `get_vehicle()` retrieves vehicle state by ID
- [ ] `list_vehicles()` returns all active vehicle IDs
- [ ] Maximum 255 vehicles supported (practical limit likely lower)
- [ ] Vehicle IDs are reusable after despawn
- [ ] Sensor data routed to correct vehicle based on `vehicle_id` field
- [ ] Actuator commands tagged with `vehicle_id` for routing to simulator
- [ ] Unit tests for vehicle lifecycle operations

## Technical Details (if applicable)

### Vehicle Configuration

```rust
pub struct VehicleConfig {
    pub id: VehicleId,
    pub mavlink_port: u16,
    pub vehicle_type: VehicleType,
    pub initial_position: Option<GeoPosition>,
    // ...
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VehicleId(pub u8);
```

### Bridge Interface

```rust
impl SitlBridge {
    pub fn spawn_vehicle(&mut self, config: VehicleConfig) -> Result<VehicleId, SitlError>;
    pub fn despawn_vehicle(&mut self, id: VehicleId) -> Result<(), SitlError>;
    pub fn get_vehicle(&self, id: VehicleId) -> Option<&VehicleInstance>;
    pub fn get_vehicle_mut(&mut self, id: VehicleId) -> Option<&mut VehicleInstance>;
    pub fn list_vehicles(&self) -> Vec<VehicleId>;
}
```

### Vehicle Instance

```rust
pub struct VehicleInstance {
    pub id: VehicleId,
    pub config: VehicleConfig,
    pub platform: SitlPlatform,
    pub state: VehicleState,
    // pico_trail core components run here
}
```

## Platform Considerations

### Cross-Platform

- Vehicle instances are independent, no shared mutable state between vehicles
- Each vehicle runs the same pico_trail core code
- Memory scales linearly with vehicle count

## Risks & Mitigation

| Risk                                 | Impact | Likelihood | Mitigation                                        | Validation        |
| ------------------------------------ | ------ | ---------- | ------------------------------------------------- | ----------------- |
| Memory exhaustion with many vehicles | High   | Low        | Document practical limits, test with 10+ vehicles | Load tests        |
| Sensor data misrouting               | High   | Medium     | Validate vehicle_id in all data paths             | Integration tests |
| State leakage between vehicles       | High   | Low        | Strict isolation, no global state                 | Unit tests        |

## Implementation Notes

- Each vehicle is essentially a complete pico_trail instance
- Consider using separate threads or async tasks per vehicle
- Vehicle ID 255 reserved for broadcast (following MAVLink convention)

## External References

- [MAVLink System ID](https://mavlink.io/en/guide/routing.html)
