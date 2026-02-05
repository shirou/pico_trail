# FR-00148 SITL Simulator Adapter Trait

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Prerequisite Requirements: None
- Dependent Requirements:
  - [FR-00149-sitl-multi-adapter-registration](FR-00149-sitl-multi-adapter-registration.md)
  - [FR-00151-sitl-gazebo-adapter](FR-00151-sitl-gazebo-adapter.md)
  - [FR-00152-sitl-lightweight-adapter](FR-00152-sitl-lightweight-adapter.md)
  - [FR-00153-sitl-lockstep-synchronization](FR-00153-sitl-lockstep-synchronization.md)
  - [FR-00154-sitl-platform-trait](FR-00154-sitl-platform-trait.md)
  - [NFR-00094-sitl-adapter-trait-safety](NFR-00094-sitl-adapter-trait-safety.md)
  - [NFR-00097-sitl-adapter-extensibility](NFR-00097-sitl-adapter-extensibility.md)
- Related Tasks:
  - [T-00157-sitl-simulator-integration](../tasks/T-00157-sitl-simulator-integration/README.md)

## Requirement Statement

SITL shall define a `SimulatorAdapter` trait that abstracts communication with external physics simulators. The trait shall provide a unified interface for connecting, receiving sensor data, sending actuator commands, and controlling simulation time, regardless of the underlying simulator implementation.

## Rationale

A trait-based abstraction enables multiple simulator backends (Gazebo, JSBSim, custom) without changes to the core SITL infrastructure. This follows pico_trail's existing pattern of trait-based platform abstraction and allows the community to contribute new adapters.

## User Story (if applicable)

As a developer, I want to use different physics simulators interchangeably so that I can choose the best tool for each testing scenario (lightweight for CI, full Gazebo for visualization).

## Acceptance Criteria

- [ ] `SimulatorAdapter` trait defined in `crates/sitl/src/adapter/mod.rs`
- [ ] Trait includes `connect()` and `disconnect()` methods for lifecycle management
- [ ] Trait includes `receive_sensors()` returning `Option<SensorData>`
- [ ] Trait includes `send_actuators()` accepting `ActuatorCommands`
- [ ] Trait includes `step()` for lockstep time advancement
- [ ] Trait includes `sim_time_us()` for current simulation time
- [ ] Trait includes `supports_lockstep()` capability query
- [ ] Trait includes `capabilities()` returning `SimulatorCapabilities`
- [ ] `SensorData` struct normalizes data from any simulator (IMU, GPS, compass, etc.)
- [ ] `ActuatorCommands` struct normalizes motor/servo outputs
- [ ] All methods are async-compatible (`async_trait`)
- [ ] Unit tests for trait usage patterns with mock implementation

## Technical Details (if applicable)

### Trait Definition

```rust
#[async_trait]
pub trait SimulatorAdapter: Send + Sync {
    fn adapter_type(&self) -> &'static str;
    fn name(&self) -> &str;

    async fn connect(&mut self) -> Result<(), SimulatorError>;
    async fn disconnect(&mut self) -> Result<(), SimulatorError>;
    fn is_connected(&self) -> bool;

    async fn receive_sensors(&mut self) -> Result<Option<SensorData>, SimulatorError>;
    async fn send_actuators(&mut self, commands: &ActuatorCommands) -> Result<(), SimulatorError>;

    async fn step(&mut self) -> Result<(), SimulatorError>;
    fn sim_time_us(&self) -> u64;
    fn supports_lockstep(&self) -> bool;
    fn capabilities(&self) -> SimulatorCapabilities;
}
```

### Normalized Data Structures

```rust
pub struct SensorData {
    pub timestamp_us: u64,
    pub vehicle_id: VehicleId,
    pub imu: Option<ImuData>,
    pub gps: Option<GpsData>,
    pub compass: Option<CompassData>,
    pub barometer: Option<BarometerData>,
    pub rangefinders: Option<Vec<RangefinderData>>,
}

pub struct ActuatorCommands {
    pub timestamp_us: u64,
    pub vehicle_id: VehicleId,
    pub motors: Vec<f32>,
    pub servos: Vec<f32>,
}
```

## Platform Considerations

### Cross-Platform

- Trait defined in new `crates/sitl` crate (host-only, not embedded)
- Uses `async_trait` for async method support
- No embedded dependencies — runs only on host

## Risks & Mitigation

| Risk                                      | Impact | Likelihood | Mitigation                                 | Validation                                   |
| ----------------------------------------- | ------ | ---------- | ------------------------------------------ | -------------------------------------------- |
| Trait too restrictive for some simulators | Medium | Low        | Design with extensible capabilities struct | Review with multiple adapter implementations |
| Async overhead in tight loops             | Low    | Low        | Benchmark critical paths                   | Performance tests                            |

## Implementation Notes

- This trait is the foundation for all simulator adapters
- Design influenced by ArduPilot's `SIM_Aircraft` base class pattern
- Object safety required for dynamic dispatch (`Box<dyn SimulatorAdapter>`)

## External References

- [ArduPilot SITL Architecture](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
