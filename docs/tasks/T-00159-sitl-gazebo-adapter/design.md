# T-00159 SITL Gazebo Adapter Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

Implement `GazeboAdapter` that communicates with Gazebo Harmonic through UDP/JSON, following the ardupilot_gazebo plugin protocol. Sensor data arrives on a dedicated UDP port, actuator commands are sent on a separate port. The adapter translates between Gazebo's format and pico_trail's normalized `SensorData`/`ActuatorCommands` types.

## Success Metrics

- [ ] `GazeboAdapter` implements `SimulatorAdapter` trait
- [ ] JSON protocol matches ardupilot_gazebo format
- [ ] UDP socket management with timeout handling
- [ ] Unit tests pass with mock sockets

## Background and Current State

- ardupilot_gazebo uses a UDP/JSON protocol for SITL communication
- Sensor data (IMU, GPS, etc.) is sent from Gazebo to the autopilot
- Actuator commands (motor PWM) are sent from the autopilot to Gazebo
- Default ports: sensors on 9002, actuators on 9003
- This adapter requires Gazebo Harmonic for integration testing but unit tests use mock sockets

## Proposed Design

### Component: GazeboAdapter

**File**: `crates/sitl/src/adapter/gazebo.rs`

```rust
pub struct GazeboAdapter {
    config: GazeboConfig,
    sensor_socket: Option<UdpSocket>,
    actuator_socket: Option<UdpSocket>,
    sim_time_us: u64,
    connected: bool,
}

pub struct GazeboConfig {
    pub sensor_port: u16,          // default: 9002
    pub actuator_port: u16,        // default: 9003
    pub server_addr: SocketAddr,
    pub ardupilot_compat: bool,    // use ardupilot_gazebo protocol
    pub timeout_ms: u32,           // default: 1000
}

impl GazeboAdapter {
    /// Parse JSON sensor data from Gazebo
    fn parse_sensor_json(&self, json: &str) -> Result<SensorData, SimulatorError> { ... }

    /// Serialize actuator commands to JSON
    fn serialize_actuator_json(&self, commands: &ActuatorCommands) -> String { ... }
}
```

### Protocol Details

**Sensor JSON format** (ardupilot_gazebo):

```json
{
  "timestamp": 1234567890,
  "imu": {
    "linear_acceleration": [0.0, 0.0, -9.81],
    "angular_velocity": [0.0, 0.0, 0.0]
  },
  "gps": {
    "lat": 35.6762,
    "lon": 139.6503,
    "alt": 40.0,
    "speed": 0.0,
    "course": 0.0
  }
}
```

**Actuator JSON format**:

```json
{
  "timestamp": 1234567890,
  "motors": [1500, 1500, 1500, 1500]
}
```

### PWM Conversion

- Normalized `[-1.0, 1.0]` → PWM `[1000, 2000]`
- Formula: `pwm = (normalized + 1.0) * 500.0 + 1000.0`
- Neutral: `0.0` → `1500`

## Testing Strategy

### Unit Tests (mock socket)

- JSON parsing: Valid sensor data, missing fields, malformed JSON
- JSON serialization: Correct format, PWM conversion accuracy
- PWM conversion: Boundary values (-1.0, 0.0, 1.0)

### Integration Tests (optional, requires Gazebo)

- Connection to real Gazebo instance
- Sensor data flow
- Actuator command delivery

## External References

- [ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
- [ADR-00156 SITL Pluggable Adapter Architecture](../../adr/ADR-00156-sitl-pluggable-adapter-architecture.md)
