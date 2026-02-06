# SITL (Software-In-The-Loop) Guide

## Overview

The SITL subsystem (`crates/sitl/`) enables testing the pico_trail autopilot
without physical hardware. It provides:

- **SitlBridge** -- orchestrator that manages adapters, vehicles, and time
- **SimulatorAdapter** trait -- pluggable interface for physics backends
- **SitlPlatform** -- simulated peripherals (UART, PWM, GPIO, Timer) that
  mirror the embedded `Platform` trait
- **TimeCoordinator** -- lockstep, free-running, and scaled time modes

## Quick Start

```rust
use pico_trail_sitl::{
    LightweightAdapter, LightweightConfig, SitlBridge,
    TimeMode, VehicleConfig, VehicleId, VehicleType,
};

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let mut bridge = SitlBridge::new();
    bridge.set_time_mode(TimeMode::Lockstep { step_size_us: 10_000 });

    // Register adapter
    let adapter = LightweightAdapter::with_defaults(VehicleId(1));
    bridge.register_adapter(Box::new(adapter)).unwrap();

    // Spawn vehicle and assign
    let id = bridge
        .spawn_vehicle(VehicleConfig::new(VehicleId(1), VehicleType::Rover))
        .unwrap();
    bridge.assign_vehicle_to_adapter(id, "lightweight").unwrap();

    // Connect and run
    bridge.get_adapter_mut("lightweight").unwrap().connect().await.unwrap();
    for _ in 0..100 {
        bridge.step().await.unwrap();
    }
}
```

See `crates/sitl/examples/basic_sitl.rs` for a complete example.

## Adapter Types

### LightweightAdapter

Built-in differential drive physics with no external dependencies. Ideal for
CI and unit testing.

- Deterministic mode via `seed` parameter
- Configurable sensor noise (GPS, IMU, compass)
- GPS rate limiting
- Lockstep support

### GazeboAdapter

Connects to Gazebo simulator over UDP/JSON. Requires an external Gazebo
instance.

- Real physics simulation
- Multi-sensor support (IMU, GPS, compass, barometer)
- Configurable UDP host/port

## Multi-Vehicle Setup

Each vehicle needs a unique `VehicleId` and MAVLink port. The bridge detects
port conflicts at spawn time.

```rust
// Spawn multiple vehicles with different adapters
for i in 1..=3 {
    let adapter = LightweightAdapter::new(
        &format!("sim{i}"),
        VehicleId(i),
        LightweightConfig { seed: Some(i as u64), ..Default::default() },
    );
    bridge.register_adapter(Box::new(adapter)).unwrap();

    let config = VehicleConfig::new(VehicleId(i), VehicleType::Rover);
    let id = bridge.spawn_vehicle(config).unwrap();
    bridge.assign_vehicle_to_adapter(id, &format!("sim{i}")).unwrap();
}
```

## Time Modes

| Mode                        | Description                     | Use Case                     |
| --------------------------- | ------------------------------- | ---------------------------- |
| `Lockstep { step_size_us }` | Fixed time increment per step   | Deterministic testing        |
| `FreeRunning`               | Wall-clock time                 | Real-time visualization      |
| `Scaled { factor }`         | Wall-clock multiplied by factor | Faster/slower than real-time |

## Adding Custom Adapters

Implement the `SimulatorAdapter` trait:

```rust
use async_trait::async_trait;
use pico_trail_sitl::{SimulatorAdapter, SimulatorError, SensorData, ActuatorCommands, SimulatorCapabilities};

struct MyAdapter { /* ... */ }

#[async_trait]
impl SimulatorAdapter for MyAdapter {
    fn adapter_type(&self) -> &'static str { "my_adapter" }
    fn name(&self) -> &str { "my-instance" }
    async fn connect(&mut self) -> Result<(), SimulatorError> { Ok(()) }
    async fn disconnect(&mut self) -> Result<(), SimulatorError> { Ok(()) }
    fn is_connected(&self) -> bool { true }
    async fn receive_sensors(&mut self) -> Result<Option<SensorData>, SimulatorError> { Ok(None) }
    async fn send_actuators(&mut self, _commands: &ActuatorCommands) -> Result<(), SimulatorError> { Ok(()) }
    async fn step(&mut self) -> Result<(), SimulatorError> { Ok(()) }
    fn sim_time_us(&self) -> u64 { 0 }
    fn supports_lockstep(&self) -> bool { false }
    fn capabilities(&self) -> SimulatorCapabilities {
        SimulatorCapabilities {
            sensors: SensorCapabilities { imu: false, gps: false, compass: false, barometer: false },
            max_rate_hz: 100,
            multi_vehicle: false,
            terrain: false,
            wind: false,
        }
    }
}
```

Then register it with the bridge:

```rust
bridge.register_adapter(Box::new(MyAdapter { /* ... */ })).unwrap();
```

## CI Integration

SITL tests run as part of CI:

```bash
cargo test -p pico_trail_sitl --lib
```

The lightweight adapter requires no external dependencies, making it suitable
for headless CI environments.
