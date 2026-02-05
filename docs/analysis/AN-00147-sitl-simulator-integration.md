# AN-00147 SITL Simulator Integration

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses:
  - [AN-00004-platform-abstraction](AN-00004-platform-abstraction.md)
  - [AN-00001-ardupilot-analysis](AN-00001-ardupilot-analysis.md)
  - [AN-00006-mavlink-network-transport](AN-00006-mavlink-network-transport.md)
- Related Requirements:
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
- Related ADRs:
  - [ADR-00156-sitl-pluggable-adapter-architecture](../adr/ADR-00156-sitl-pluggable-adapter-architecture.md)
- Related Tasks:
  - [T-00157-sitl-simulator-integration](../tasks/T-00157-sitl-simulator-integration/README.md)
  - [T-00158-sitl-platform-and-lightweight-adapter](../tasks/T-00158-sitl-platform-and-lightweight-adapter/README.md)
  - [T-00159-sitl-gazebo-adapter](../tasks/T-00159-sitl-gazebo-adapter/README.md)
  - [T-00160-sitl-multi-vehicle-lockstep-ci](../tasks/T-00160-sitl-multi-vehicle-lockstep-ci/README.md)

## Executive Summary

This analysis explores implementing Software-In-The-Loop (SITL) simulation for pico_trail with a **pluggable simulator adapter architecture**. Rather than coupling to a single simulator, the design abstracts the simulator interface behind a `SimulatorAdapter` trait, enabling:

1. **Multiple simulator backends** - Gazebo, JSBSim, custom lightweight simulators, or future options
2. **Multiple simultaneous instances** - Run several rovers in one simulation, or compare different simulator behaviors
3. **Protocol flexibility** - Each adapter implements its own communication protocol

The recommended approach uses a **SITL Bridge** with pluggable adapters: the bridge manages simulation lifecycle and routing, while adapters handle protocol-specific communication with each simulator backend.

Estimated effort: **8-12 weeks** for core infrastructure + first adapter (Gazebo).

## Problem Space

### Current State

pico_trail currently has:

- **Complete Mock Platform** (`crates/firmware/src/platform/mock/`) - 26 tests passing
- **Host-testable core logic** (`crates/core/`) - 126 unit tests, no embedded dependencies
- **MAVLink protocol stack** - Full parameter, mission, telemetry support
- **Platform abstraction** - Clean trait-based design enabling mock implementations

However, unit tests with mocks cannot verify:

- End-to-end autonomous navigation behavior
- Sensor fusion under realistic noise conditions
- GCS integration (Mission Planner, QGroundControl)
- Multi-waypoint mission execution
- Failsafe triggering and recovery
- Multi-vehicle coordination scenarios

### ArduPilot SITL Architecture (Reference)

ArduPilot's SITL supports multiple external simulators through adapter classes:

| Simulator    | File                 | Protocol    |
| ------------ | -------------------- | ----------- |
| Gazebo       | `SIM_Gazebo.cpp`     | UDP/JSON    |
| JSBSim       | `SIM_JSBSim.cpp`     | Socket/FDM  |
| FlightAxis   | `SIM_FlightAxis.cpp` | SOAP/XML    |
| X-Plane      | `SIM_XPlane.cpp`     | UDP binary  |
| JSON Generic | `SIM_JSON.cpp`       | UDP/JSON    |
| Morse        | `SIM_Morse.cpp`      | Socket/JSON |
| Webots       | `SIM_Webots.cpp`     | TCP/JSON    |

**Key insight**: ArduPilot uses a common `Aircraft` base class with simulator-specific subclasses. Each subclass handles its protocol while presenting a unified sensor/actuator interface.

### Why Pluggable Architecture

| Consideration          | Single Simulator          | Pluggable Architecture         |
| ---------------------- | ------------------------- | ------------------------------ |
| Future flexibility     | Locked to one choice      | Add simulators as needed       |
| Testing diversity      | Single physics model      | Compare across simulators      |
| CI options             | Heavy dependency (Gazebo) | Can use lightweight sim for CI |
| Multi-vehicle          | Limited by simulator      | Independent per-vehicle        |
| Community contribution | Fixed protocol            | Contributors add adapters      |

## Proposed Architecture

### High-Level Design

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                     External Simulators (Multiple)                       │
│                                                                          │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐                │
│  │   Gazebo     │   │   JSBSim     │   │   Custom     │   ...          │
│  │  (Physics,   │   │  (Aircraft   │   │ (Lightweight │                │
│  │   Sensors,   │   │   Dynamics)  │   │   for CI)    │                │
│  │   Visual)    │   │              │   │              │                │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘                │
│         │                  │                  │                         │
│         │ UDP/JSON         │ Socket/FDM       │ UDP/Binary              │
└─────────┼──────────────────┼──────────────────┼─────────────────────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                      Simulator Adapters                                  │
│                                                                          │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐                │
│  │ GazeboAdapter│   │JSBSimAdapter │   │CustomAdapter │   ...          │
│  │              │   │              │   │              │                │
│  │ impl         │   │ impl         │   │ impl         │                │
│  │ SimulatorAdapter  SimulatorAdapter  SimulatorAdapter                │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘                │
│         │                  │                  │                         │
│         └──────────────────┴──────────────────┘                         │
│                            │                                            │
│                            ▼                                            │
│  ┌───────────────────────────────────────────────────────────────────┐ │
│  │                      SITL Bridge                                   │ │
│  │                                                                    │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐   │ │
│  │  │ Adapter     │  │ Vehicle     │  │ Time                    │   │ │
│  │  │ Registry    │  │ Manager     │  │ Coordinator             │   │ │
│  │  │             │  │ (multi-veh) │  │ (lockstep/free-run)     │   │ │
│  │  └─────────────┘  └─────────────┘  └─────────────────────────┘   │ │
│  └───────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
                                   │
          ┌────────────────────────┼────────────────────────┐
          │                        │                        │
          ▼                        ▼                        ▼
┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐
│  Vehicle 1       │    │  Vehicle 2       │    │  Vehicle N       │
│  (SITL Platform) │    │  (SITL Platform) │    │  (SITL Platform) │
│                  │    │                  │    │                  │
│  pico_trail core │    │  pico_trail core │    │  pico_trail core │
│  (unchanged)     │    │  (unchanged)     │    │  (unchanged)     │
└────────┬─────────┘    └────────┬─────────┘    └────────┬─────────┘
         │                       │                       │
         │ MAVLink               │ MAVLink               │ MAVLink
         │ (UDP 14550)           │ (UDP 14551)           │ (UDP 14552)
         ▼                       ▼                       ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    Ground Control Station(s)                             │
│                Mission Planner / QGroundControl                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Core Abstractions

#### SimulatorAdapter Trait

```rust
/// Adapter for external physics simulators
/// Each simulator backend implements this trait
#[async_trait]
pub trait SimulatorAdapter: Send + Sync {
    /// Unique identifier for this adapter type
    fn adapter_type(&self) -> &'static str;

    /// Human-readable name
    fn name(&self) -> &str;

    /// Connect to the simulator
    async fn connect(&mut self) -> Result<(), SimulatorError>;

    /// Disconnect from the simulator
    async fn disconnect(&mut self) -> Result<(), SimulatorError>;

    /// Check if connected
    fn is_connected(&self) -> bool;

    /// Receive sensor data from simulator
    /// Returns None if no data available (non-blocking)
    async fn receive_sensors(&mut self) -> Result<Option<SensorData>, SimulatorError>;

    /// Send actuator commands to simulator
    async fn send_actuators(&mut self, commands: &ActuatorCommands) -> Result<(), SimulatorError>;

    /// Request simulator to advance one time step (lockstep mode)
    async fn step(&mut self) -> Result<(), SimulatorError>;

    /// Get current simulation time
    fn sim_time_us(&self) -> u64;

    /// Check if simulator supports lockstep
    fn supports_lockstep(&self) -> bool;

    /// Get simulator capabilities
    fn capabilities(&self) -> SimulatorCapabilities;
}

/// Capabilities advertised by a simulator adapter
#[derive(Debug, Clone)]
pub struct SimulatorCapabilities {
    /// Supported sensor types
    pub sensors: SensorCapabilities,
    /// Maximum update rate (Hz)
    pub max_rate_hz: u32,
    /// Supports multiple vehicles
    pub multi_vehicle: bool,
    /// Supports terrain/collision
    pub terrain: bool,
    /// Supports wind simulation
    pub wind: bool,
}

#[derive(Debug, Clone, Default)]
pub struct SensorCapabilities {
    pub imu: bool,
    pub gps: bool,
    pub compass: bool,
    pub barometer: bool,
    pub rangefinder: bool,
    pub optical_flow: bool,
    pub camera: bool,
}
```

#### Unified Sensor/Actuator Data

```rust
/// Normalized sensor data from any simulator
#[derive(Debug, Clone)]
pub struct SensorData {
    pub timestamp_us: u64,
    pub vehicle_id: VehicleId,

    /// IMU data (body frame)
    pub imu: Option<ImuData>,

    /// GPS data
    pub gps: Option<GpsData>,

    /// Compass data (body frame magnetic field)
    pub compass: Option<CompassData>,

    /// Barometer data
    pub barometer: Option<BarometerData>,

    /// Rangefinder distances
    pub rangefinders: Option<Vec<RangefinderData>>,
}

/// Normalized actuator commands to any simulator
#[derive(Debug, Clone)]
pub struct ActuatorCommands {
    pub timestamp_us: u64,
    pub vehicle_id: VehicleId,

    /// Motor outputs (-1.0 to 1.0, or 0.0 to 1.0 depending on type)
    pub motors: Vec<f32>,

    /// Servo outputs (normalized -1.0 to 1.0)
    pub servos: Vec<f32>,
}

/// Vehicle identifier for multi-vehicle scenarios
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VehicleId(pub u8);
```

#### SITL Bridge (Orchestrator)

```rust
/// SITL Bridge - orchestrates simulation across adapters and vehicles
pub struct SitlBridge {
    /// Registered simulator adapters
    adapters: HashMap<String, Box<dyn SimulatorAdapter>>,

    /// Active vehicles
    vehicles: HashMap<VehicleId, VehicleInstance>,

    /// Vehicle-to-adapter mapping
    vehicle_adapter_map: HashMap<VehicleId, String>,

    /// Time coordination mode
    time_mode: TimeMode,

    /// Global simulation state
    state: SimulationState,
}

pub enum TimeMode {
    /// Simulators run at wall-clock time
    FreeRunning,
    /// All simulators step together
    Lockstep {
        step_size_us: u64,
    },
    /// Scaled real-time (e.g., 2x, 10x)
    Scaled {
        factor: f32,
    },
}

impl SitlBridge {
    /// Register a new simulator adapter
    pub fn register_adapter(&mut self, adapter: Box<dyn SimulatorAdapter>);

    /// Spawn a new vehicle instance
    pub fn spawn_vehicle(&mut self, config: VehicleConfig) -> Result<VehicleId, SimulatorError>;

    /// Assign vehicle to specific adapter
    pub fn assign_vehicle_to_adapter(&mut self, vehicle_id: VehicleId, adapter_name: &str);

    /// Run one simulation step (all vehicles)
    pub async fn step(&mut self) -> Result<(), SimulatorError>;

    /// Run simulation loop
    pub async fn run(&mut self) -> Result<(), SimulatorError>;
}
```

### Adapter Implementations

#### Gazebo Adapter

```rust
/// Adapter for Gazebo simulator (Harmonic/Garden)
pub struct GazeboAdapter {
    config: GazeboConfig,
    sensor_socket: Option<UdpSocket>,
    actuator_socket: Option<UdpSocket>,
    sim_time_us: u64,
    connected: bool,
}

pub struct GazeboConfig {
    /// Sensor data receive port
    pub sensor_port: u16,        // default: 9002
    /// Actuator command send port
    pub actuator_port: u16,      // default: 9003
    /// Gazebo server address
    pub server_addr: SocketAddr,
    /// Use ardupilot_gazebo compatible protocol
    pub ardupilot_compat: bool,
}

impl SimulatorAdapter for GazeboAdapter {
    fn adapter_type(&self) -> &'static str { "gazebo" }
    fn supports_lockstep(&self) -> bool { true }
    // ... implementation
}
```

#### JSBSim Adapter (Future)

```rust
/// Adapter for JSBSim flight dynamics
pub struct JSBSimAdapter {
    config: JSBSimConfig,
    fdm_socket: Option<TcpStream>,
    // ...
}

impl SimulatorAdapter for JSBSimAdapter {
    fn adapter_type(&self) -> &'static str { "jsbsim" }
    fn supports_lockstep(&self) -> bool { true }
    // ...
}
```

#### Lightweight CI Adapter

```rust
/// Minimal simulator for CI testing (no external dependency)
/// Implements basic differential drive kinematics
pub struct LightweightAdapter {
    state: VehicleState,
    config: LightweightConfig,
}

pub struct LightweightConfig {
    /// Wheel separation (meters)
    pub wheel_base: f32,
    /// Max speed (m/s)
    pub max_speed: f32,
    /// GPS noise stddev (meters)
    pub gps_noise: f32,
    /// IMU noise parameters
    pub imu_noise: ImuNoiseConfig,
}

impl SimulatorAdapter for LightweightAdapter {
    fn adapter_type(&self) -> &'static str { "lightweight" }
    fn supports_lockstep(&self) -> bool { true }

    // Implements simple kinematics internally - no external process needed
    async fn step(&mut self) -> Result<(), SimulatorError> {
        // Integrate differential drive equations
        // Add sensor noise
        // Update internal state
    }
}
```

### Multi-Vehicle Architecture

```text
┌─────────────────────────────────────────────────────────────┐
│                      Gazebo World                            │
│                                                              │
│   ┌─────────┐    ┌─────────┐    ┌─────────┐               │
│   │ Rover 1 │    │ Rover 2 │    │ Rover 3 │               │
│   │ model   │    │ model   │    │ model   │               │
│   └────┬────┘    └────┬────┘    └────┬────┘               │
│        │              │              │                     │
│        └──────────────┴──────────────┘                     │
│                       │                                     │
│              Gazebo Multi-Vehicle Plugin                    │
│                       │                                     │
└───────────────────────┼─────────────────────────────────────┘
                        │ UDP (vehicle_id in packet)
                        ▼
┌───────────────────────────────────────────────────────────────┐
│                    GazeboAdapter                               │
│  - Demultiplexes by vehicle_id                                │
│  - Routes sensor data to correct vehicle                      │
│  - Aggregates actuator commands                               │
└───────────────────────┬───────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        ▼               ▼               ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│  Vehicle 1  │ │  Vehicle 2  │ │  Vehicle 3  │
│  instance   │ │  instance   │ │  instance   │
└─────────────┘ └─────────────┘ └─────────────┘
```

### Communication Protocol (Normalized)

All adapters convert their native protocol to this internal format:

**Sensor Data (Simulator → pico_trail)**:

```rust
SensorData {
    timestamp_us: 1234567890,
    vehicle_id: VehicleId(1),
    imu: Some(ImuData {
        accel_mss: [0.01, -0.02, -9.81],  // m/s²
        gyro_rads: [0.001, -0.002, 0.003], // rad/s
        temperature_c: 25.0,
    }),
    gps: Some(GpsData {
        lat_deg: 35.681236,
        lon_deg: 139.767125,
        alt_m: 40.0,
        speed_ms: 1.5,
        course_deg: 90.0,
        fix_type: GpsFixType::Fix3D,
        satellites: 12,
        hdop: 1.2,
    }),
    compass: Some(CompassData {
        mag_gauss: [0.25, 0.10, -0.45],
    }),
    barometer: None,
    rangefinders: None,
}
```

**Actuator Commands (pico_trail → Simulator)**:

```rust
ActuatorCommands {
    timestamp_us: 1234567890,
    vehicle_id: VehicleId(1),
    motors: vec![0.75, 0.65],  // left, right for diff drive
    servos: vec![],
}
```

### Time Synchronization

#### Lockstep Mode (Multi-Adapter)

```text
SitlBridge           GazeboAdapter        JSBSimAdapter       Vehicles
    │                     │                    │                  │
    │── request_step ────▶│                    │                  │
    │                     │── step_physics ───▶│                  │
    │                     │                    │                  │
    │◀── sensors (V1) ────│                    │                  │
    │◀── sensors (V2) ────│                    │                  │
    │                     │◀── sensors (V3) ───│                  │
    │                                                             │
    │── distribute_sensors ──────────────────────────────────────▶│
    │                                                             │
    │◀── collect_actuators ───────────────────────────────────────│
    │                                                             │
    │── send_actuators ──▶│                    │                  │
    │                     │── send_actuators ─▶│                  │
    │                                                             │
    │ (next step)                                                 │
```

#### Free-Running Mode

- Each adapter runs independently at wall-clock time
- Sensors buffered with timestamps
- Actuators sent immediately
- Useful for real-time visualization and GCS testing

## Existing Infrastructure Reuse

### Already Available (No Changes Needed)

| Component                 | Location                                     | Status |
| ------------------------- | -------------------------------------------- | ------ |
| Platform trait            | `crates/firmware/src/platform/traits/`       | Ready  |
| Mock platform (reference) | `crates/firmware/src/platform/mock/`         | Ready  |
| MAVLink protocol          | `crates/firmware/src/communication/mavlink/` | Ready  |
| Navigation controller     | `crates/core/src/navigation/`                | Ready  |
| AHRS algorithms           | `crates/core/src/ahrs/`                      | Ready  |
| Mode implementations      | `crates/firmware/src/rover/mode/`            | Ready  |
| Motor abstractions        | `crates/core/src/motor/`                     | Ready  |
| Parameter store           | `crates/core/src/parameters/`                | Ready  |

### Needs Implementation

| Component               | Effort    | Complexity | Priority |
| ----------------------- | --------- | ---------- | -------- |
| SimulatorAdapter trait  | 1 week    | Low        | P0       |
| SITL Bridge core        | 2 weeks   | Medium     | P0       |
| SITL Platform impl      | 1.5 weeks | Medium     | P0       |
| GazeboAdapter           | 2 weeks   | Medium     | P1       |
| LightweightAdapter (CI) | 1.5 weeks | Low        | P1       |
| Multi-vehicle support   | 1.5 weeks | Medium     | P2       |
| Lockstep coordinator    | 1 week    | Medium     | P2       |
| JSBSimAdapter           | 2 weeks   | Medium     | P3       |

## Risk Assessment

| Risk                     | Probability | Impact | Mitigation                                          |
| ------------------------ | ----------- | ------ | --------------------------------------------------- |
| Abstraction leakage      | Medium      | High   | Clear trait boundaries, comprehensive tests         |
| Protocol version drift   | Low         | Medium | Version field in packets, adapter-specific handling |
| Lockstep deadlocks       | Medium      | Medium | Timeouts, fallback to free-running                  |
| Multi-vehicle complexity | Medium      | Medium | Start single-vehicle, add multi incrementally       |
| Performance overhead     | Low         | Low    | Async I/O, zero-copy where possible                 |

## Stakeholder Analysis

| Stakeholder   | Interest/Need                     | Impact | Priority |
| ------------- | --------------------------------- | ------ | -------- |
| Developer     | Rapid iteration, simulator choice | High   | P0       |
| CI/QA         | Lightweight, deterministic tests  | High   | P0       |
| Advanced User | Multi-vehicle, custom simulators  | Medium | P2       |
| Community     | Contribute new adapters           | Medium | P2       |

## Design Considerations

### Option A: Pluggable Adapter Architecture (Recommended)

Abstract simulator interface with trait-based adapters.

**Pros**:

- Maximum flexibility - add simulators without core changes
- Multi-vehicle natural fit
- Community can contribute adapters
- Lightweight adapter enables fast CI

**Cons**:

- More upfront design work
- Adapter maintenance per simulator

### Option B: Protocol-Only Abstraction

Single SITL implementation with configurable protocol (JSON, binary, etc.).

**Pros**:

- Simpler initial implementation
- Less code to maintain

**Cons**:

- Limited to protocol variations, not simulator features
- Multi-vehicle harder to add later
- Tight coupling to specific assumptions

### Option C: ROS2 as Universal Bridge

Use ROS2 topics as the abstraction layer.

**Pros**:

- Rich ecosystem, standard messages
- Many simulators have ROS2 bridges

**Cons**:

- Heavy dependency
- Overkill for embedded testing
- Complex setup

### Recommendation

**Option A** provides the best long-term value:

1. Trait-based design aligns with pico_trail's existing patterns
2. LightweightAdapter enables fast CI without external dependencies
3. GazeboAdapter provides full-featured simulation
4. Community can extend with new adapters
5. Multi-vehicle support is built-in

## Discovered Requirements

### Functional Requirements

- [ ] **FR-TBD-1**: SITL shall define a `SimulatorAdapter` trait that abstracts communication with external physics simulators
  - Rationale: Enable multiple simulator backends without core changes
  - Acceptance Criteria: Trait supports connect/disconnect, sensor receive, actuator send, time step

- [ ] **FR-TBD-2**: SITL Bridge shall support registering multiple simulator adapters simultaneously
  - Rationale: Enable comparing simulators or using different simulators for different purposes
  - Acceptance Criteria: Adapters registered by name, vehicles assigned to specific adapters

- [ ] **FR-TBD-3**: SITL shall support multiple vehicle instances, each with independent autopilot state
  - Rationale: Enable multi-vehicle testing scenarios (formation, coordination)
  - Acceptance Criteria: Up to 255 vehicles, each with unique VehicleId, independent MAVLink ports

- [ ] **FR-TBD-4**: GazeboAdapter shall implement SimulatorAdapter using UDP/JSON protocol compatible with ardupilot_gazebo
  - Rationale: Reuse existing Gazebo models and community resources
  - Acceptance Criteria: Works with unmodified ardupilot_gazebo plugin

- [ ] **FR-TBD-5**: LightweightAdapter shall implement SimulatorAdapter with built-in differential drive kinematics
  - Rationale: Enable CI testing without external simulator dependency
  - Acceptance Criteria: No external process required, deterministic, configurable noise

- [ ] **FR-TBD-6**: SITL shall support lockstep time synchronization across all adapters
  - Rationale: Enable deterministic, repeatable test scenarios
  - Acceptance Criteria: All adapters step together, identical inputs produce identical outputs

- [ ] **FR-TBD-7**: SITL Platform shall implement Platform trait, enabling existing pico_trail code to run unchanged
  - Rationale: Validate actual autopilot logic, not simulation-specific code
  - Acceptance Criteria: All rover modes work in SITL with any adapter

- [ ] **FR-TBD-8**: Each vehicle instance shall support independent MAVLink communication on configurable UDP ports
  - Rationale: Enable GCS connection to multiple vehicles
  - Acceptance Criteria: Vehicle N uses port 14550+N by default, configurable

### Non-Functional Requirements

- [ ] **NFR-TBD-1**: SimulatorAdapter trait shall be object-safe and thread-safe (Send + Sync)
  - Category: Extensibility
  - Rationale: Enable dynamic adapter registration and async usage
  - Target: `Box<dyn SimulatorAdapter>` usable across threads

- [ ] **NFR-TBD-2**: SITL sensor-to-actuator latency shall be <20ms in lockstep mode
  - Category: Performance
  - Rationale: Control loop stability requires bounded latency
  - Target: <20ms 99th percentile per vehicle

- [ ] **NFR-TBD-3**: LightweightAdapter shall run without external dependencies for CI
  - Category: Testability
  - Rationale: CI must not require Gazebo installation
  - Target: `cargo test` sufficient, no external process

- [ ] **NFR-TBD-4**: Adding a new simulator adapter shall not require modifying SITL Bridge core
  - Category: Extensibility
  - Rationale: Enable community contributions without core changes
  - Target: New adapter = new struct implementing trait, register at runtime

## Effort Estimate

| Phase                           | Tasks                                                     | Effort          |
| ------------------------------- | --------------------------------------------------------- | --------------- |
| **Phase 1: Core Abstractions**  | SimulatorAdapter trait, SensorData/ActuatorCommands types | 1 week          |
| **Phase 2: SITL Bridge**        | Adapter registry, vehicle management, basic routing       | 2 weeks         |
| **Phase 3: SITL Platform**      | Platform trait implementation for SITL                    | 1.5 weeks       |
| **Phase 4: LightweightAdapter** | Built-in kinematics, sensor noise, CI-ready               | 1.5 weeks       |
| **Phase 5: GazeboAdapter**      | UDP/JSON protocol, ardupilot_gazebo compat                | 2 weeks         |
| **Phase 6: Gazebo Setup**       | Rover model, sensor plugins, world files                  | 1.5 weeks       |
| **Phase 7: Multi-Vehicle**      | VehicleId routing, MAVLink port assignment                | 1 week          |
| **Phase 8: Lockstep**           | Time coordinator, multi-adapter sync                      | 1 week          |
| **Buffer**                      | Integration, debugging, documentation                     | 1.5 weeks       |
| **Total**                       |                                                           | **13-14 weeks** |

Note: Phases 4-5 can be parallelized (different developers), reducing wall-clock time to **10-12 weeks**.

## Open Questions

- [ ] Should adapter configuration be via TOML/JSON file or programmatic API?
- [ ] What is the maximum supported vehicle count? (Proposed: 255, matching MAVLink system ID)
- [ ] Should LightweightAdapter include terrain collision, or remain purely kinematic?
- [ ] Priority order for additional adapters after Gazebo? (JSBSim, Webots, AirSim, custom)

## Recommendations

### Recommended Approach

**Option A** - Pluggable adapter architecture with:

1. **SimulatorAdapter trait** as the core abstraction
2. **LightweightAdapter** for CI (no external deps, fast, deterministic)
3. **GazeboAdapter** for full simulation (physics, visualization, ardupilot_gazebo compat)
4. **Multi-vehicle support** built into the design from start

### Implementation Order

1. **Core abstractions first** - Trait, data types, SITL Bridge skeleton
2. **LightweightAdapter + Platform** - Get end-to-end working without Gazebo
3. **GazeboAdapter** - Add full simulation capability
4. **Multi-vehicle + Lockstep** - Advanced features

### Next Steps After Approval

1. Create formal requirements (FR/NFR) from discovered requirements
2. Create ADR for adapter architecture (cross-cutting architectural decision)
3. Create task package with design and phased implementation plan

### Out of Scope

- Specific Gazebo model creation (separate task)
- JSBSim/Webots/AirSim adapters (future tasks)
- Camera/vision simulation (future enhancement)
- Real-time performance guarantees (SITL is for testing, not production)

## Appendix

### ArduPilot Multi-Simulator Support

From ArduPilot's `libraries/SITL/`:

| Class          | Purpose                       |
| -------------- | ----------------------------- |
| `SIM_Aircraft` | Base class for all simulators |
| `SIM_Frame`    | Vehicle frame configuration   |
| `SIM_Gazebo`   | Gazebo bridge                 |
| `SIM_JSBSim`   | JSBSim bridge                 |
| `SIM_JSON`     | Generic JSON protocol         |
| `SIM_Morse`    | Morse simulator bridge        |
| `SIM_Webots`   | Webots bridge                 |

ArduPilot instantiates one `Aircraft` subclass per configuration. Our adapter pattern provides similar flexibility.

### Adapter Registration Example

```rust
fn main() {
    let mut bridge = SitlBridge::new();

    // Register available adapters
    bridge.register_adapter(Box::new(LightweightAdapter::new(
        LightweightConfig::default()
    )));

    bridge.register_adapter(Box::new(GazeboAdapter::new(
        GazeboConfig {
            sensor_port: 9002,
            actuator_port: 9003,
            server_addr: "127.0.0.1:9001".parse().unwrap(),
            ardupilot_compat: true,
        }
    )));

    // Spawn vehicles
    let v1 = bridge.spawn_vehicle(VehicleConfig {
        id: VehicleId(1),
        mavlink_port: 14550,
        ..Default::default()
    })?;

    let v2 = bridge.spawn_vehicle(VehicleConfig {
        id: VehicleId(2),
        mavlink_port: 14551,
        ..Default::default()
    })?;

    // Assign to adapters
    bridge.assign_vehicle_to_adapter(v1, "gazebo")?;
    bridge.assign_vehicle_to_adapter(v2, "lightweight")?;

    // Run simulation
    bridge.run().await?;
}
```

### CI Configuration Example

```yaml
# .github/workflows/sitl-test.yml
jobs:
  sitl-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Run SITL tests
        run: |
          # No Gazebo needed - uses LightweightAdapter
          cargo test --features sitl -- --test-threads=1
```

### Gazebo World Example (Multi-Vehicle)

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="multi_rover_world">
    <!-- Rover 1 -->
    <include>
      <uri>model://pico_trail_rover</uri>
      <name>rover_1</name>
      <pose>0 0 0 0 0 0</pose>
      <plugin name="ardupilot_plugin" filename="libArduPilotPlugin.so">
        <fdm_addr>127.0.0.1</fdm_addr>
        <fdm_port_in>9002</fdm_port_in>
        <fdm_port_out>9003</fdm_port_out>
        <vehicle_id>1</vehicle_id>
      </plugin>
    </include>

    <!-- Rover 2 -->
    <include>
      <uri>model://pico_trail_rover</uri>
      <name>rover_2</name>
      <pose>5 0 0 0 0 0</pose>
      <plugin name="ardupilot_plugin" filename="libArduPilotPlugin.so">
        <fdm_addr>127.0.0.1</fdm_addr>
        <fdm_port_in>9004</fdm_port_in>
        <fdm_port_out>9005</fdm_port_out>
        <vehicle_id>2</vehicle_id>
      </plugin>
    </include>
  </world>
</sdf>
```
