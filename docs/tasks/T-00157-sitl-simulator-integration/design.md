# T-00157 SITL Core Abstractions and Bridge Design

## Metadata

- Type: Design
- Status: Implementation Complete

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

Define the core SITL abstractions and bridge orchestrator that form the foundation for all simulator integration. The `SimulatorAdapter` trait provides a pluggable interface for different physics backends. The `SitlBridge` manages adapter registration and vehicle lifecycle. All subsequent SITL tasks (T-00158, T-00159, T-00160) build on these abstractions.

## Success Metrics

- [x] `SimulatorAdapter` trait defined and object-safe
- [x] `SitlBridge` manages adapters and vehicles
- [x] Normalized data types compile and can be instantiated
- [x] Error types cover all failure modes
- [x] New adapters require 0 lines of bridge change

## Background and Current State

- Context: pico_trail needs integration testing beyond unit tests with mocks
- Current state:
  - `MockPlatform` exists for unit tests but cannot verify end-to-end behavior
  - Platform abstraction (ADR-00003) provides trait-based hardware isolation
  - MAVLink protocol stack complete for GCS communication
- Pain points:
  - Cannot verify autonomous navigation logic
  - Cannot test sensor fusion under realistic conditions
  - CI relies only on unit tests
- Constraints:
  - SITL runs on host only (not embedded)
  - Must not modify core autopilot code
  - CI must work without Gazebo

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                       crates/sitl (New Crate)                           │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                    SimulatorAdapter Trait                          │  │
│  │  connect(), receive_sensors(), send_actuators(), step()           │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│           ▲                        ▲                        ▲           │
│           │                        │                        │           │
│  ┌────────┴────────┐     ┌────────┴────────┐     ┌────────┴────────┐  │
│  │ LightweightAdptr│     │  GazeboAdapter  │     │  (Future)       │  │
│  │ (T-00158)       │     │  (T-00159)      │     │  JSBSim, etc.   │  │
│  └─────────────────┘     └─────────────────┘     └─────────────────┘  │
│                                                                          │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                         SITL Bridge                                │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐   │  │
│  │  │ Adapter     │  │ Vehicle     │  │ Time                    │   │  │
│  │  │ Registry    │  │ Management  │  │ Mode                    │   │  │
│  │  └─────────────┘  └─────────────┘  └─────────────────────────┘   │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Crate Structure (This Task)

```text
crates/sitl/
├── Cargo.toml
├── src/
│   ├── lib.rs                 # Public API, re-exports
│   ├── error.rs               # SimulatorError enum
│   ├── types.rs               # SensorData, ActuatorCommands, VehicleId
│   ├── adapter/
│   │   ├── mod.rs             # SimulatorAdapter trait
│   │   └── capabilities.rs    # SimulatorCapabilities
│   ├── bridge/
│   │   ├── mod.rs             # SitlBridge
│   │   ├── registry.rs        # Adapter registry
│   │   └── time.rs            # TimeMode
│   └── vehicle/
│       ├── mod.rs             # VehicleInstance
│       └── config.rs          # VehicleConfig
└── tests/
    ├── adapter_tests.rs
    └── bridge_tests.rs
```

### Component: SimulatorAdapter Trait

**File**: `crates/sitl/src/adapter/mod.rs`

```rust
use async_trait::async_trait;

#[async_trait]
pub trait SimulatorAdapter: Send + Sync {
    /// Unique identifier for this adapter type (e.g., "gazebo", "lightweight")
    fn adapter_type(&self) -> &'static str;

    /// Human-readable name
    fn name(&self) -> &str;

    /// Connect to the simulator
    async fn connect(&mut self) -> Result<(), SimulatorError>;

    /// Disconnect from the simulator
    async fn disconnect(&mut self) -> Result<(), SimulatorError>;

    /// Check if connected
    fn is_connected(&self) -> bool;

    /// Receive sensor data (non-blocking, returns None if no data available)
    async fn receive_sensors(&mut self) -> Result<Option<SensorData>, SimulatorError>;

    /// Send actuator commands
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
```

### Component: Normalized Data Types

**File**: `crates/sitl/src/types.rs`

```rust
/// Vehicle identifier (matches MAVLink system ID range)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct VehicleId(pub u8);

/// Sensor data from any simulator
#[derive(Debug, Clone)]
pub struct SensorData {
    pub timestamp_us: u64,
    pub vehicle_id: VehicleId,
    pub imu: Option<ImuData>,
    pub gps: Option<GpsData>,
    pub compass: Option<CompassData>,
    pub barometer: Option<BarometerData>,
}

#[derive(Debug, Clone)]
pub struct ImuData {
    pub accel_mss: [f32; 3],      // m/s² in body frame
    pub gyro_rads: [f32; 3],      // rad/s in body frame
    pub temperature_c: f32,
}

#[derive(Debug, Clone)]
pub struct GpsData {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f32,
    pub speed_ms: f32,
    pub course_deg: f32,
    pub fix_type: GpsFixType,
    pub satellites: u8,
    pub hdop: f32,
}

#[derive(Debug, Clone)]
pub struct CompassData {
    pub mag_gauss: [f32; 3],      // Gauss in body frame
}

/// Actuator commands to any simulator
#[derive(Debug, Clone)]
pub struct ActuatorCommands {
    pub timestamp_us: u64,
    pub vehicle_id: VehicleId,
    pub motors: Vec<f32>,        // Normalized -1.0 to 1.0
    pub servos: Vec<f32>,        // Normalized -1.0 to 1.0
}
```

### Component: SITL Bridge

**File**: `crates/sitl/src/bridge/mod.rs`

```rust
pub struct SitlBridge {
    /// Registered adapters by name
    adapters: HashMap<String, Box<dyn SimulatorAdapter>>,

    /// Active vehicle instances
    vehicles: HashMap<VehicleId, VehicleInstance>,

    /// Vehicle-to-adapter assignment
    vehicle_adapter_map: HashMap<VehicleId, String>,

    /// Time synchronization mode
    time_mode: TimeMode,

    /// Current simulation time (microseconds)
    sim_time_us: u64,
}

pub enum TimeMode {
    FreeRunning,
    Lockstep { step_size_us: u64 },
    Scaled { factor: f32 },
}

impl SitlBridge {
    pub fn new() -> Self;

    // Adapter management
    pub fn register_adapter(&mut self, adapter: Box<dyn SimulatorAdapter>) -> Result<(), SitlError>;
    pub fn unregister_adapter(&mut self, name: &str) -> Result<(), SitlError>;
    pub fn list_adapters(&self) -> Vec<&str>;

    // Vehicle management
    pub fn spawn_vehicle(&mut self, config: VehicleConfig) -> Result<VehicleId, SitlError>;
    pub fn despawn_vehicle(&mut self, id: VehicleId) -> Result<(), SitlError>;
    pub fn assign_vehicle_to_adapter(&mut self, id: VehicleId, adapter: &str) -> Result<(), SitlError>;

    // Simulation control (basic, single-step placeholder)
    pub async fn step(&mut self) -> Result<(), SitlError>;
    pub fn set_time_mode(&mut self, mode: TimeMode);
}
```

### Error Handling

```rust
#[derive(Debug, thiserror::Error)]
pub enum SimulatorError {
    #[error("Connection failed: {0}")]
    ConnectionFailed(String),

    #[error("Protocol error: {0}")]
    ProtocolError(String),

    #[error("Timeout waiting for {0}")]
    Timeout(&'static str),

    #[error("Adapter not found: {0}")]
    AdapterNotFound(String),

    #[error("Vehicle not found: {0}")]
    VehicleNotFound(VehicleId),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}
```

### Alternatives Considered

1. **Single Gazebo Integration**: Simpler but no CI without Gazebo
2. **ROS2 Bridge**: Too heavy, complex setup
3. **Protocol-Only Abstraction**: Less flexible for future simulators

### Decision Rationale

Pluggable adapter architecture provides:

- Fast CI with LightweightAdapter (no external deps)
- Full simulation with GazeboAdapter (ardupilot_gazebo compatible)
- Future extensibility (JSBSim, AirSim, custom)
- Multi-vehicle from day one

## Testing Strategy

### Unit Tests

- **Adapter trait safety**: `Box<dyn SimulatorAdapter>` compiles
- **Bridge adapter registry**: Register, unregister, list
- **Bridge vehicle management**: Spawn, despawn, assign
- **Data types**: VehicleId derives, SensorData construction/clone
- **Error types**: All variants constructible

## External References

- [ADR-00156 SITL Pluggable Adapter Architecture](../../adr/ADR-00156-sitl-pluggable-adapter-architecture.md)
- [ADR-00003 Platform Abstraction](../../adr/ADR-00003-platform-abstraction.md)
