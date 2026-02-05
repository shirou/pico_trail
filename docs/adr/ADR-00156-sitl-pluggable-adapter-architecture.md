# ADR-00156 SITL Pluggable Adapter Architecture

## Metadata

- Type: ADR
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Impacted Requirements:
  - [FR-00148-sitl-simulator-adapter-trait](../requirements/FR-00148-sitl-simulator-adapter-trait.md)
  - [FR-00149-sitl-multi-adapter-registration](../requirements/FR-00149-sitl-multi-adapter-registration.md)
  - [FR-00150-sitl-multi-vehicle-instances](../requirements/FR-00150-sitl-multi-vehicle-instances.md)
  - [FR-00151-sitl-gazebo-adapter](../requirements/FR-00151-sitl-gazebo-adapter.md)
  - [FR-00152-sitl-lightweight-adapter](../requirements/FR-00152-sitl-lightweight-adapter.md)
  - [FR-00153-sitl-lockstep-synchronization](../requirements/FR-00153-sitl-lockstep-synchronization.md)
  - [FR-00154-sitl-platform-trait](../requirements/FR-00154-sitl-platform-trait.md)
  - [FR-00155-sitl-vehicle-mavlink-ports](../requirements/FR-00155-sitl-vehicle-mavlink-ports.md)
  - [NFR-00094-sitl-adapter-trait-safety](../requirements/NFR-00094-sitl-adapter-trait-safety.md)
  - [NFR-00095-sitl-latency](../requirements/NFR-00095-sitl-latency.md)
  - [NFR-00096-sitl-lightweight-no-deps](../requirements/NFR-00096-sitl-lightweight-no-deps.md)
  - [NFR-00097-sitl-adapter-extensibility](../requirements/NFR-00097-sitl-adapter-extensibility.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-00157-sitl-simulator-integration](../tasks/T-00157-sitl-simulator-integration/README.md)
  - [T-00158-sitl-platform-and-lightweight-adapter](../tasks/T-00158-sitl-platform-and-lightweight-adapter/README.md)
  - [T-00159-sitl-gazebo-adapter](../tasks/T-00159-sitl-gazebo-adapter/README.md)
  - [T-00160-sitl-multi-vehicle-lockstep-ci](../tasks/T-00160-sitl-multi-vehicle-lockstep-ci/README.md)

## Context

pico_trail requires Software-In-The-Loop (SITL) simulation capabilities for:

1. **Development Testing**: Rapid iteration without hardware
2. **CI/CD Integration**: Automated regression tests
3. **GCS Validation**: Test Mission Planner/QGroundControl integration
4. **Multi-Vehicle Scenarios**: Formation, coordination testing

### Current State

pico_trail already has:

- **Mock Platform** (`platform/mock/`) for unit tests
- **Platform Abstraction** (ADR-00003) enabling test implementations
- **MAVLink Protocol Stack** for GCS communication

However, mock implementations cannot verify:

- End-to-end autonomous navigation
- Sensor fusion under realistic noise
- Physics-based vehicle behavior

### Problem

We need SITL that:

1. **Supports multiple simulators** (Gazebo, JSBSim, lightweight for CI)
2. **Enables multi-vehicle testing** (formation, coordination)
3. **Runs pico_trail code unchanged** (validate actual autopilot logic)
4. **Allows community extensions** (new simulator adapters without core changes)

### Constraints

- **Existing Code Unchanged**: Autopilot modes, navigation, AHRS must run as-is
- **CI Friendliness**: Must support lightweight testing without Gazebo
- **Protocol Compatibility**: Prefer ardupilot_gazebo protocol for model reuse
- **Extensibility**: Adding new simulators should not require core changes

### Prior Art

- **ArduPilot SITL**: 370+ files, supports Gazebo, JSBSim, X-Plane via subclasses of `SIM_Aircraft`
- **PX4 SITL**: Plugin architecture with Gazebo, jMAVSim, FlightGear
- **Betaflight SITL**: Built-in physics, no external dependencies

## Success Metrics

| Metric                      | Target                     | Measurement      |
| --------------------------- | -------------------------- | ---------------- |
| External simulator support  | ≥2 (Gazebo, Lightweight)   | Adapter count    |
| Multi-vehicle support       | ≥10 vehicles               | Load test        |
| CI test execution           | No Gazebo required         | CI config review |
| New adapter effort          | ≤1 week for simple adapter | Time tracking    |
| Code change for new adapter | 0 lines in core            | Git diff         |

## Decision

**We will implement a pluggable adapter architecture for SITL with the following components:**

1. **SimulatorAdapter Trait**: Abstract interface for all simulators
2. **SITL Bridge**: Orchestrator managing adapters and vehicles
3. **SITL Platform**: Platform trait implementation for simulation
4. **Built-in Adapters**: GazeboAdapter (full sim), LightweightAdapter (CI)

### Architecture

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                     External Simulators                                  │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐                │
│  │   Gazebo     │   │   JSBSim     │   │   Future     │                │
│  │              │   │   (future)   │   │   Simulators │                │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘                │
└─────────┼──────────────────┼──────────────────┼─────────────────────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                 SimulatorAdapter Implementations                         │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐                │
│  │GazeboAdapter │   │LightweightAd.│   │ Custom       │                │
│  │ (UDP/JSON)   │   │ (built-in    │   │ Adapters     │                │
│  │              │   │  kinematics) │   │              │                │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘                │
│         │                  │                  │                         │
│         └──────────────────┴──────────────────┘                         │
│                            │                                            │
│                            ▼                                            │
│  ┌───────────────────────────────────────────────────────────────────┐ │
│  │                      SITL Bridge                                   │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐   │ │
│  │  │ Adapter     │  │ Vehicle     │  │ Time                    │   │ │
│  │  │ Registry    │  │ Manager     │  │ Coordinator             │   │ │
│  │  └─────────────┘  └─────────────┘  └─────────────────────────┘   │ │
│  └───────────────────────────────────────────────────────────────────┘ │
│                            │                                            │
│         ┌──────────────────┼──────────────────┐                        │
│         ▼                  ▼                  ▼                         │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                │
│  │  Vehicle 1  │    │  Vehicle 2  │    │  Vehicle N  │                │
│  │ SitlPlatform│    │ SitlPlatform│    │ SitlPlatform│                │
│  │ pico_trail  │    │ pico_trail  │    │ pico_trail  │                │
│  │   core      │    │   core      │    │   core      │                │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘                │
│         │                  │                  │                         │
└─────────┼──────────────────┼──────────────────┼─────────────────────────┘
          │ MAVLink          │ MAVLink          │ MAVLink
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    Ground Control Station(s)                             │
└─────────────────────────────────────────────────────────────────────────┘
```

### Core Abstraction: SimulatorAdapter Trait

```rust
#[async_trait]
pub trait SimulatorAdapter: Send + Sync {
    /// Unique identifier for this adapter type
    fn adapter_type(&self) -> &'static str;

    /// Human-readable name
    fn name(&self) -> &str;

    /// Lifecycle management
    async fn connect(&mut self) -> Result<(), SimulatorError>;
    async fn disconnect(&mut self) -> Result<(), SimulatorError>;
    fn is_connected(&self) -> bool;

    /// Data exchange
    async fn receive_sensors(&mut self) -> Result<Option<SensorData>, SimulatorError>;
    async fn send_actuators(&mut self, commands: &ActuatorCommands) -> Result<(), SimulatorError>;

    /// Time synchronization
    async fn step(&mut self) -> Result<(), SimulatorError>;
    fn sim_time_us(&self) -> u64;
    fn supports_lockstep(&self) -> bool;

    /// Capability discovery
    fn capabilities(&self) -> SimulatorCapabilities;
}
```

### Decision Drivers

1. **Extensibility**: New adapters without core changes
2. **Multi-Simulator**: Compare behaviors, use different simulators for different purposes
3. **Multi-Vehicle**: Formation testing, swarm scenarios
4. **CI Friendliness**: Lightweight adapter runs without external dependencies
5. **ArduPilot Compatibility**: Reuse ardupilot_gazebo models and protocol

### Considered Options

| Option                                | Description                                                      |
| ------------------------------------- | ---------------------------------------------------------------- |
| **A: Pluggable Adapter Architecture** | Trait-based adapters, multi-simulator, multi-vehicle ⭐ Selected |
| **B: Single Simulator (Gazebo Only)** | Direct Gazebo integration, simpler but limited                   |
| **C: Protocol-Only Abstraction**      | Single implementation with configurable protocols                |
| **D: ROS2 Bridge**                    | Use ROS2 as abstraction layer                                    |

### Option Analysis

**Option A: Pluggable Adapter Architecture** ⭐ Selected

- **Pros**:
  - Maximum flexibility — add simulators without core changes
  - LightweightAdapter enables fast CI without external deps
  - Multi-vehicle built into design
  - Community can contribute adapters
  - Follows pico_trail's trait-based patterns (ADR-00003)
- **Cons**:
  - More upfront design effort
  - Trait must be carefully designed for object safety
- **Effort**: 10-12 weeks

**Option B: Single Simulator (Gazebo Only)**

- **Pros**:
  - Simpler implementation
  - Full physics fidelity from day one
- **Cons**:
  - CI requires Gazebo installation (heavy)
  - No lightweight testing option
  - Adding other simulators requires refactoring
- **Effort**: 6-8 weeks

**Option C: Protocol-Only Abstraction**

- **Pros**:
  - Single SITL implementation
  - Configurable protocol (JSON, binary)
- **Cons**:
  - Limited to protocol variations, not simulator features
  - Multi-vehicle harder to add later
  - Protocol assumptions may not fit all simulators
- **Effort**: 6-8 weeks

**Option D: ROS2 Bridge**

- **Pros**:
  - Rich ecosystem, standard messages
  - Many simulators have ROS2 support
- **Cons**:
  - Heavy dependency (ROS2 runtime)
  - Overkill for embedded autopilot testing
  - Complex setup for CI
- **Effort**: 8-10 weeks + ROS2 expertise

## Rationale

**Option A (Pluggable Adapter Architecture)** was selected because:

1. **Aligns with existing architecture**: pico_trail already uses trait-based platform abstraction (ADR-00003). Applying the same pattern to SITL maintains consistency.

2. **Enables lightweight CI**: `LightweightAdapter` with built-in kinematics requires no external processes, enabling fast `cargo test` execution.

3. **Future-proof**: Adding JSBSim, AirSim, or custom simulators requires only implementing `SimulatorAdapter` trait — zero core changes.

4. **Multi-vehicle from start**: VehicleId routing and per-vehicle state designed into the architecture, not bolted on later.

5. **Community friendly**: External contributors can create adapters in separate crates without forking pico_trail.

### Trade-offs Accepted

| Trade-off                       | Accepted Because                                           |
| ------------------------------- | ---------------------------------------------------------- |
| More upfront design             | Long-term flexibility outweighs initial effort             |
| Trait object safety constraints | `async_trait` handles this; benefits outweigh complexity   |
| Multiple adapters to maintain   | Each adapter is independent; maintenance is parallelizable |

### Rejected Alternatives

- **Single Gazebo integration**: Would require CI to have Gazebo, significantly slowing feedback loop
- **ROS2 bridge**: Dependency too heavy for embedded-focused project
- **Protocol-only**: Limits future simulator options

## Consequences

### Positive

1. **Fast CI**: `LightweightAdapter` enables `cargo test --features sitl` without Gazebo
2. **Simulator Choice**: Developers choose best tool for each scenario
3. **Multi-Vehicle Testing**: Formation and coordination tests possible from day one
4. **Community Extensions**: New adapters don't require core changes
5. **ArduPilot Model Reuse**: `GazeboAdapter` uses ardupilot_gazebo protocol

### Negative

1. **Upfront Design**: Must carefully design trait for object safety and extensibility
2. **Two Adapters Initially**: Both Gazebo and Lightweight must be implemented
3. **Documentation**: Must document how to create new adapters

### Neutral

1. **Dynamic Dispatch**: Using `Box<dyn SimulatorAdapter>` (acceptable overhead for SITL)
2. **Host-Only**: SITL runs on host, not embedded target

## Implementation Notes

### Adapter Implementation Pattern

```rust
// GazeboAdapter example
pub struct GazeboAdapter {
    config: GazeboConfig,
    sensor_socket: Option<UdpSocket>,
    actuator_socket: Option<UdpSocket>,
    sim_time_us: u64,
}

#[async_trait]
impl SimulatorAdapter for GazeboAdapter {
    fn adapter_type(&self) -> &'static str { "gazebo" }
    fn name(&self) -> &str { &self.config.name }

    async fn connect(&mut self) -> Result<(), SimulatorError> {
        self.sensor_socket = Some(UdpSocket::bind(&self.config.sensor_addr).await?);
        self.actuator_socket = Some(UdpSocket::bind(&self.config.actuator_addr).await?);
        Ok(())
    }

    async fn receive_sensors(&mut self) -> Result<Option<SensorData>, SimulatorError> {
        // Parse JSON from Gazebo, convert to SensorData
    }

    async fn send_actuators(&mut self, commands: &ActuatorCommands) -> Result<(), SimulatorError> {
        // Convert to JSON, send to Gazebo
    }

    // ... remaining methods
}
```

### SITL Bridge Usage

```rust
fn main() {
    let mut bridge = SitlBridge::new();

    // Register adapters
    bridge.register_adapter(Box::new(GazeboAdapter::new(gazebo_config)));
    bridge.register_adapter(Box::new(LightweightAdapter::new(lightweight_config)));

    // Spawn vehicles
    let v1 = bridge.spawn_vehicle(VehicleConfig { id: VehicleId(1), .. });
    let v2 = bridge.spawn_vehicle(VehicleConfig { id: VehicleId(2), .. });

    // Assign vehicles to adapters
    bridge.assign_vehicle_to_adapter(v1, "gazebo");
    bridge.assign_vehicle_to_adapter(v2, "lightweight");

    // Run simulation
    bridge.run().await;
}
```

### Normalized Data Structures

```rust
/// Sensor data from any simulator
pub struct SensorData {
    pub timestamp_us: u64,
    pub vehicle_id: VehicleId,
    pub imu: Option<ImuData>,
    pub gps: Option<GpsData>,
    pub compass: Option<CompassData>,
    pub barometer: Option<BarometerData>,
}

/// Actuator commands to any simulator
pub struct ActuatorCommands {
    pub timestamp_us: u64,
    pub vehicle_id: VehicleId,
    pub motors: Vec<f32>,   // Normalized -1.0 to 1.0
    pub servos: Vec<f32>,
}
```

### Time Synchronization Modes

```rust
pub enum TimeMode {
    /// Simulators run at wall-clock time
    FreeRunning,

    /// All simulators step together (deterministic)
    Lockstep { step_size_us: u64 },

    /// Scaled real-time (e.g., 2x, 10x)
    Scaled { factor: f32 },
}
```

## Crate Structure

```text
crates/
├── core/           # Existing - autopilot logic (unchanged)
├── firmware/       # Existing - embedded binary (unchanged)
└── sitl/           # NEW - SITL infrastructure
    ├── src/
    │   ├── lib.rs
    │   ├── adapter/
    │   │   ├── mod.rs          # SimulatorAdapter trait
    │   │   ├── gazebo.rs       # GazeboAdapter
    │   │   └── lightweight.rs  # LightweightAdapter
    │   ├── bridge.rs           # SitlBridge
    │   ├── platform.rs         # SitlPlatform (impl Platform)
    │   ├── vehicle.rs          # VehicleInstance
    │   └── time.rs             # Time synchronization
    └── Cargo.toml
```

## Platform Considerations

- **Host Only**: `crates/sitl` is not `no_std`, uses `tokio` for async
- **Feature Flag**: `--features sitl` enables SITL tests in firmware crate
- **CI Integration**: GitHub Actions runs `cargo test --features sitl` without Gazebo

## Open Questions

- [x] Should we use `async_trait` or native async in traits? → **Decision**: Use `async_trait` for object safety
- [ ] Should adapters be in separate crates for independent versioning?
- [ ] What is the minimum Gazebo version to support? (Harmonic recommended)

## External References

- [ArduPilot SITL Architecture](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [ardupilot_gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo)
- [ADR-00003 Platform Abstraction](ADR-00003-platform-abstraction.md)
