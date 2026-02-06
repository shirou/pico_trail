# pico_trail_sitl

Software-In-The-Loop (SITL) simulator integration for the pico_trail autopilot.

## Overview

This crate provides a simulation environment that mirrors the firmware's `Platform` abstraction, enabling testing of autopilot logic on the host without embedded hardware. The core component is `SitlBridge`, which orchestrates adapters, vehicles, and timing.

### Adapters

| Adapter | Description |
|---|---|
| `LightweightAdapter` | Built-in 2D physics (differential drive kinematics, deterministic seeded RNG). No external dependencies. |
| `GazeboAdapter` | Connects to Gazebo via UDP/JSON, compatible with `ardupilot_gazebo` protocol. |

### Time Modes

- **Lockstep** - Simulation advances in fixed steps (e.g., 10 ms). Deterministic and reproducible.
- **Realtime** - Simulation tracks wall-clock time.

## Quick Start

### Run the built-in example

```bash
cargo run -p pico_trail_sitl --example basic_sitl
```

Expected output:

```text
=== pico_trail SITL Basic Example ===

Spawned Vehicle(1)
Adapter 'sim1' connected

Running 100 steps (1 second at 100 Hz)...

Step  20 | time: 200000 us
  IMU: accel=[-0.21, 0.06, -9.77] m/s^2
  GPS: lat=35.676200, lon=139.650305, speed=0.50 m/s
...
Simulation complete. Final time: 1000000 us
```

### Minimal code example

```rust
use pico_trail_sitl::{
    LightweightAdapter, LightweightConfig, SitlBridge, TimeMode,
    VehicleConfig, VehicleId, VehicleType,
};

#[tokio::main(flavor = "current_thread")]
async fn main() {
    // 1. Create bridge with lockstep timing at 100 Hz
    let mut bridge = SitlBridge::new();
    bridge.set_time_mode(TimeMode::Lockstep { step_size_us: 10_000 });

    // 2. Register a lightweight adapter (deterministic with seed)
    let config = LightweightConfig {
        seed: Some(42),
        gps_noise_m: 0.5,
        step_size_us: 10_000,
        ..Default::default()
    };
    let adapter = LightweightAdapter::new("sim1", VehicleId(1), config);
    bridge.register_adapter(Box::new(adapter)).unwrap();

    // 3. Spawn a rover and assign it to the adapter
    let vid = bridge
        .spawn_vehicle(VehicleConfig::new(VehicleId(1), VehicleType::Rover))
        .unwrap();
    bridge.assign_vehicle_to_adapter(vid, "sim1").unwrap();

    // 4. Connect the adapter
    bridge.get_adapter_mut("sim1").unwrap().connect().await.unwrap();

    // 5. Set motor outputs via the vehicle's SitlPlatform
    {
        let v = bridge.get_vehicle(vid).unwrap();
        v.platform.create_pwm(0, 50).unwrap(); // left motor
        v.platform.create_pwm(1, 50).unwrap(); // right motor
        v.platform.set_pwm_duty(0, 0.75);      // 75% forward
        v.platform.set_pwm_duty(1, 0.75);
    }

    // 6. Run simulation loop
    for _ in 0..100 {
        bridge.step().await.unwrap();
    }

    // 7. Read sensor data
    let v = bridge.get_vehicle(vid).unwrap();
    if let Some(sensors) = v.platform.take_sensors() {
        println!("IMU: {:?}", sensors.imu);
        println!("GPS: {:?}", sensors.gps);
    }

    println!("Final time: {} us", bridge.sim_time_us());
}
```

### Using GazeboAdapter

```rust
use pico_trail_sitl::adapter::{GazeboAdapter, GazeboConfig};
use pico_trail_sitl::{SitlBridge, VehicleConfig, VehicleId, VehicleType};

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let mut bridge = SitlBridge::new();

    let config = GazeboConfig {
        sensor_port: 9002,
        actuator_port: 9003,
        ..Default::default()
    };
    let adapter = GazeboAdapter::new("gazebo1", VehicleId(1), config);
    bridge.register_adapter(Box::new(adapter)).unwrap();

    let vid = bridge
        .spawn_vehicle(VehicleConfig::new(VehicleId(1), VehicleType::Rover))
        .unwrap();
    bridge.assign_vehicle_to_adapter(vid, "gazebo1").unwrap();

    // Connect starts UDP listeners; Gazebo must be running
    bridge.get_adapter_mut("gazebo1").unwrap().connect().await.unwrap();

    // Simulation loop exchanges sensor/actuator data with Gazebo
    for _ in 0..1000 {
        bridge.step().await.unwrap();
    }
}
```

## Running tests

```bash
cargo test -p pico_trail_sitl --lib --quiet
```

## Architecture

```text
SitlBridge
  +-- AdapterRegistry     (named adapters: Lightweight, Gazebo, ...)
  +-- VehicleInstance[]    (each with SitlPlatform + VehicleConfig)
  +-- TimeCoordinator     (Lockstep / Realtime)
```

Each `bridge.step()` call:

1. Reads actuator commands from each vehicle's `SitlPlatform`
2. Sends them to the assigned adapter (`send_actuators`)
3. Advances the adapter's physics or receives external sensor data (`step` / `receive_sensors`)
4. Delivers sensor data back to the vehicle's `SitlPlatform`
