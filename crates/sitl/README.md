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

## Running 3 Vehicles with Gazebo

This section describes how to run 3 simulated rovers using Gazebo Harmonic and the `GazeboAdapter`.

### Prerequisites

- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install) (gz-sim 8.x)
- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) plugin (for sensor/actuator UDP bridge)
- Rust toolchain with `cargo`

### 1. Prepare the Gazebo World

Create a world file that spawns 3 rover models, each configured with the `ardupilot_gazebo` plugin on separate UDP port pairs.

Save the following as `worlds/multi_rover.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="multi_rover">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <!-- Ground plane -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>

    <!-- Rover 1: sensor_port=9002, actuator_port=9003 -->
    <include>
      <uri>model://rover</uri>
      <name>rover1</name>
      <pose>0 0 0.1 0 0 0</pose>
      <plugin filename="libArduPilotPlugin.so" name="ardupilot_gazebo::Plugin">
        <fdm_addr>127.0.0.1</fdm_addr>
        <fdm_port_in>9003</fdm_port_in>
        <fdm_port_out>9002</fdm_port_out>
      </plugin>
    </include>

    <!-- Rover 2: sensor_port=9012, actuator_port=9013 -->
    <include>
      <uri>model://rover</uri>
      <name>rover2</name>
      <pose>3 0 0.1 0 0 0</pose>
      <plugin filename="libArduPilotPlugin.so" name="ardupilot_gazebo::Plugin">
        <fdm_addr>127.0.0.1</fdm_addr>
        <fdm_port_in>9013</fdm_port_in>
        <fdm_port_out>9012</fdm_port_out>
      </plugin>
    </include>

    <!-- Rover 3: sensor_port=9022, actuator_port=9023 -->
    <include>
      <uri>model://rover</uri>
      <name>rover3</name>
      <pose>6 0 0.1 0 0 0</pose>
      <plugin filename="libArduPilotPlugin.so" name="ardupilot_gazebo::Plugin">
        <fdm_addr>127.0.0.1</fdm_addr>
        <fdm_port_in>9023</fdm_port_in>
        <fdm_port_out>9022</fdm_port_out>
      </plugin>
    </include>
  </world>
</sdf>
```

Port convention: each vehicle uses `base + (id - 1) * 10` for the sensor port and `base + 1` for the actuator port.

| Vehicle | Sensor Port (Gazebo → SITL) | Actuator Port (SITL → Gazebo) |
| ------- | --------------------------- | ----------------------------- |
| 1       | 9002                        | 9003                          |
| 2       | 9012                        | 9013                          |
| 3       | 9022                        | 9023                          |

### 2. Launch Gazebo

```bash
gz sim worlds/multi_rover.sdf
```

Gazebo starts and waits for actuator commands on the configured ports.

### 3. Run the SITL Bridge

Create a Rust program (or add to an example) that connects 3 `GazeboAdapter` instances to the running Gazebo world:

```rust
use pico_trail_sitl::adapter::{GazeboAdapter, GazeboConfig};
use pico_trail_sitl::{
    GcsLink, SitlBridge, TimeMode, VehicleConfig, VehicleId, VehicleType,
};

const VEHICLE_COUNT: u8 = 3;
const SENSOR_PORT_BASE: u16 = 9002;
const PORT_STRIDE: u16 = 10;
const MAVLINK_PORT_BASE: u16 = 14551;

#[tokio::main(flavor = "current_thread")]
async fn main() {
    println!("=== pico_trail SITL — 3 Rovers with Gazebo ===\n");

    let mut bridge = SitlBridge::new();
    bridge.set_time_mode(TimeMode::Scaled { factor: 1.0 });

    let mut gcs_links = Vec::new();

    for i in 1..=VEHICLE_COUNT {
        let id = VehicleId(i);
        let adapter_name = format!("gazebo{i}");

        let sensor_port = SENSOR_PORT_BASE + (i as u16 - 1) * PORT_STRIDE;
        let actuator_port = sensor_port + 1;

        let config = GazeboConfig {
            sensor_port,
            actuator_port,
            server_addr: format!("127.0.0.1:{actuator_port}").parse().unwrap(),
            ..Default::default()
        };
        let adapter = GazeboAdapter::new(&adapter_name, id, config);
        bridge.register_adapter(Box::new(adapter)).unwrap();

        let vehicle_config = VehicleConfig::new(id, VehicleType::Rover);
        bridge.spawn_vehicle(vehicle_config).unwrap();
        bridge.assign_vehicle_to_adapter(id, &adapter_name).unwrap();

        bridge
            .get_adapter_mut(&adapter_name)
            .unwrap()
            .connect()
            .await
            .unwrap();

        // Set up PWM channels
        let v = bridge.get_vehicle(id).unwrap();
        v.platform.create_pwm(0, 50).unwrap(); // left motor
        v.platform.create_pwm(1, 50).unwrap(); // right motor

        // GCS telemetry (Mission Planner / QGroundControl)
        let mav_port = MAVLINK_PORT_BASE + (i - 1) as u16;
        gcs_links.push(GcsLink::new(i, mav_port).unwrap());

        println!(
            "Vehicle {i}: sensor={sensor_port}, actuator={actuator_port}, MAVLink={mav_port}"
        );
    }

    println!("\nBridge running. Press Ctrl+C to stop.\n");

    let mut interval = tokio::time::interval(tokio::time::Duration::from_millis(10));

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => break,
            _ = interval.tick() => {
                bridge.step().await.unwrap();
                let sim_time = bridge.sim_time_us();

                for i in 1..=VEHICLE_COUNT {
                    let idx = (i - 1) as usize;
                    let vehicle = bridge.get_vehicle(VehicleId(i)).unwrap();
                    if let Some(sensors) = vehicle.platform.peek_sensors() {
                        gcs_links[idx].update(&sensors, sim_time);
                    }
                }
            }
        }
    }

    println!("Shutdown.");
}
```

### 4. Connect a Ground Control Station

While both Gazebo and the SITL bridge are running, connect Mission Planner or QGroundControl to the MAVLink UDP ports:

| Vehicle | UDP Port |
| ------- | -------- |
| 1       | 14551    |
| 2       | 14552    |
| 3       | 14553    |

In **Mission Planner**: use CTRL+L → Add UDP connections to each port.

In **QGroundControl**: Settings → Comm Links → Add UDP connections to each port.

### Architecture

```text
Gazebo Harmonic (gz-sim)
  rover1        rover2        rover3
  :9002/:9003   :9012/:9013   :9022/:9023
    |               |               |
    v               v               v
+-------------+-------------+-------------+
| GazeboAdapter| GazeboAdapter| GazeboAdapter|
| "gazebo1"   | "gazebo2"   | "gazebo3"   |
+------+------+------+------+------+------+
       |             |             |
   Vehicle 1     Vehicle 2     Vehicle 3
   SitlPlatform  SitlPlatform  SitlPlatform
       |             |             |
   GcsLink :14551  GcsLink :14552  GcsLink :14553
       |             |             |
       v             v             v
   Mission Planner / QGroundControl
```

### Troubleshooting

- **"Failed to bind sensor socket"**: Another process is using the port. Check with `ss -ulnp | grep 900`.
- **No sensor data received**: Verify Gazebo is running and the `ardupilot_gazebo` plugin is loaded. Check the Gazebo console for plugin errors.
- **Vehicles not moving**: Ensure PWM channels are created and duty is set (see step 3 code). Verify the rover model in Gazebo responds to actuator commands.
- **GCS not connecting**: Confirm the MAVLink ports (14551–14553) are not blocked by a firewall.

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
