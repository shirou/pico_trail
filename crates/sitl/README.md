# pico_trail_sitl

Software-In-The-Loop (SITL) simulator integration for the pico_trail autopilot.

## Overview

This crate provides a simulation environment that mirrors the firmware's `Platform` abstraction, enabling testing of autopilot logic on the host without embedded hardware. The core component is `SitlBridge`, which orchestrates adapters, vehicles, and timing. A built-in `GcsLink` multiplexes MAVLink telemetry for all vehicles over a single TCP connection, so Mission Planner auto-detects every vehicle without extra configuration.

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
        gazebo_addr: "127.0.0.1:9002".parse().unwrap(),
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

- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install) (gz-sim 8.x) — native install or Docker (see below)
- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) plugin (for sensor/actuator UDP bridge)
- Rust toolchain with `cargo`

### 1. Prepare the Gazebo World

The `r1_rover` model from [ArduPilot/SITL\_Models](https://github.com/ArduPilot/SITL_Models) has `ArduPilotPlugin` with `fdm_port_in` hardcoded to 9002. For multi-vehicle, each rover needs a unique port, so we generate per-vehicle model copies.

Save the following as `scripts/setup_gazebo_models.sh`:

```bash
#!/bin/bash
# Generate per-vehicle r1_rover models with unique fdm_port_in values.
# Usage: ./scripts/setup_gazebo_models.sh [COUNT] [BASE_PORT] [STRIDE]

COUNT=${1:-3}
BASE_PORT=${2:-9002}
STRIDE=${3:-10}

MODELS_SRC="${GZ_SIM_RESOURCE_PATH%%:*}"  # first path entry
if [ ! -d "$MODELS_SRC/r1_rover" ]; then
    echo "Error: r1_rover model not found in $MODELS_SRC" >&2
    exit 1
fi

for i in $(seq 1 "$COUNT"); do
    PORT=$(( BASE_PORT + (i - 1) * STRIDE ))
    DEST="$MODELS_SRC/r1_rover_$i"
    cp -r "$MODELS_SRC/r1_rover" "$DEST"
    sed -i "s|<fdm_port_in>9002</fdm_port_in>|<fdm_port_in>${PORT}</fdm_port_in>|" "$DEST/model.sdf"
    sed -i "s|<name>r1_rover</name>|<name>r1_rover_$i</name>|" "$DEST/model.config"
    echo "Created r1_rover_$i (fdm_port_in=$PORT)"
done
```

Then save the following as `worlds/multi_rover.sdf`:

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

    <!-- Rover 1: fdm_port_in=9002 -->
    <include>
      <uri>model://r1_rover_1</uri>
      <name>rover1</name>
      <pose>0 0 0.1 0 0 0</pose>
    </include>

    <!-- Rover 2: fdm_port_in=9012 -->
    <include>
      <uri>model://r1_rover_2</uri>
      <name>rover2</name>
      <pose>3 0 0.1 0 0 0</pose>
    </include>

    <!-- Rover 3: fdm_port_in=9022 -->
    <include>
      <uri>model://r1_rover_3</uri>
      <name>rover3</name>
      <pose>6 0 0.1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

Port convention: each vehicle uses `base + (id - 1) * 10` for its `fdm_port_in`. The ArduPilot plugin auto-detects the response port.

| Vehicle | Gazebo `fdm_port_in` |
| ------- | -------------------- |
| 1       | 9002                 |
| 2       | 9012                 |
| 3       | 9022                 |

### 2. Launch Gazebo

#### Option A: Native install

```bash
gz sim worlds/multi_rover.sdf
```

#### Option B: Docker (recommended)

The Docker Hub `gazebo` image is Gazebo Classic only. For Gazebo Harmonic use the [OCI images](https://github.com/j-rivero/gz_oci_images) as a base.

Save the following as `docker/Dockerfile.gazebo`:

```dockerfile
FROM ghcr.io/j-rivero/gazebo:harmonic-full

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git libgz-sim8-dev rapidjson-dev libopencv-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Build ardupilot_gazebo plugin
WORKDIR /gz_ws/src
RUN git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git
WORKDIR /gz_ws/src/ardupilot_gazebo/build
RUN cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && make -j"$(nproc)" \
    && make install

# Rover models from ArduPilot SITL_Models
WORKDIR /gz_ws/src
RUN git clone --depth 1 --filter=blob:none --sparse \
        https://github.com/ArduPilot/SITL_Models.git \
    && cd SITL_Models && git sparse-checkout set Gazebo/models Gazebo/worlds

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/gz_ws/src/ardupilot_gazebo/build
ENV GZ_SIM_RESOURCE_PATH=/gz_ws/src/SITL_Models/Gazebo/models:/gz_ws/src/ardupilot_gazebo/models:/gz_ws/src/ardupilot_gazebo/worlds:/gz_ws/src/SITL_Models/Gazebo/worlds

WORKDIR /worlds
```

Build once:

```bash
docker build -t gazebo-ardupilot -f docker/Dockerfile.gazebo .
```

Run with GUI (X11 forwarding):

```bash
xhost +local:docker

docker run --rm -it \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/worlds:/worlds" \
    -v "$(pwd)/scripts:/scripts" \
    gazebo-ardupilot \
    bash -c "/scripts/setup_gazebo_models.sh 3 9002 10 && gz sim /worlds/multi_rover.sdf"
```

For NVIDIA GPU, add `--gpus all` and `--runtime=nvidia`:

```bash
docker run --rm -it \
    --network host \
    --gpus all --runtime=nvidia \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/worlds:/worlds" \
    -v "$(pwd)/scripts:/scripts" \
    gazebo-ardupilot \
    bash -c "/scripts/setup_gazebo_models.sh 3 9002 10 && gz sim /worlds/multi_rover.sdf"
```

`--network host` shares UDP ports with the host so that `gazebo_bridge` can communicate directly.

After launch, Gazebo starts and waits for actuator commands on the configured ports.

### 3. Run the SITL Bridge

The `gazebo_bridge` binary connects `GazeboAdapter` instances to the running Gazebo world. All vehicles share a single MAVLink TCP port for GCS communication.

```bash
# 3 vehicles (default), MAVLink on TCP :14550
cargo run -p pico_trail_sitl --bin gazebo_bridge

# 5 vehicles
cargo run -p pico_trail_sitl --bin gazebo_bridge -- -n 5

# Custom port configuration
cargo run -p pico_trail_sitl --bin gazebo_bridge -- -n 3 --gazebo-port-base 9002 --port-stride 10

# Custom MAVLink TCP port
cargo run -p pico_trail_sitl --bin gazebo_bridge -- --mavlink-port 14551

# Show all options
cargo run -p pico_trail_sitl --bin gazebo_bridge -- --help
```

Source: `src/bin/gazebo_bridge.rs`

### 4. Connect a Ground Control Station

While both Gazebo and the SITL bridge are running, connect Mission Planner or QGroundControl to the MAVLink TCP port. All vehicles share a single TCP connection — the GCS auto-detects them via distinct MAVLink `system_id` values.

Default port: **14550** (configurable via `--mavlink-port`).

In **Mission Planner**: Connection type → TCP, then enter `127.0.0.1` and port `14550`. All vehicles appear automatically in the vehicle selector dropdown.

In **QGroundControl**: Settings → Comm Links → Add a TCP connection to `127.0.0.1:14550`. All vehicles are discovered automatically.

### Architecture

```text
Gazebo Harmonic (gz-sim)
  r1_rover_1      r1_rover_2      r1_rover_3
  fdm_port:9002   fdm_port:9012   fdm_port:9022
    |                |                |
    v (UDP)          v (UDP)          v (UDP)
+---------------+---------------+---------------+
| GazeboAdapter | GazeboAdapter | GazeboAdapter |
| "gazebo1"     | "gazebo2"     | "gazebo3"     |
+-------+-------+-------+-------+-------+-------+
        |               |               |
    Vehicle 1       Vehicle 2       Vehicle 3
    SitlPlatform    SitlPlatform    SitlPlatform
        \               |               /
         \              |              /
          +--- GcsLink (TCP :14550) --+
                        |
                        v
              Mission Planner / QGC
           (auto-detects all vehicles
            via MAVLink system_id)
```

### Troubleshooting

- **"Failed to bind sensor socket"**: Another process is using the port. Check with `ss -ulnp | grep 900`.
- **No sensor data received**: Verify Gazebo is running and the `ardupilot_gazebo` plugin is loaded. Check the Gazebo console for plugin errors.
- **Vehicles not moving**: Ensure PWM channels are created and duty is set (see step 3 code). Verify the rover model in Gazebo responds to actuator commands.
- **GCS not connecting**: Confirm the MAVLink TCP port (default 14550) is not blocked by a firewall. On WSL2, use TCP (not UDP) — UDP return packets may not reach the Windows host.

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
