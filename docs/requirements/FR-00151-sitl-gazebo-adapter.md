# FR-00151 SITL Gazebo Adapter

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Prerequisite Requirements:
  - [FR-00148-sitl-simulator-adapter-trait](FR-00148-sitl-simulator-adapter-trait.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00159-sitl-gazebo-adapter](../tasks/T-00159-sitl-gazebo-adapter/README.md)

## Requirement Statement

`GazeboAdapter` shall implement the `SimulatorAdapter` trait using UDP/JSON protocol compatible with the `ardupilot_gazebo` plugin. This enables pico_trail to use existing ArduPilot Gazebo models and community resources.

## Rationale

Gazebo provides high-fidelity physics simulation, 3D visualization, and a mature sensor plugin ecosystem. Compatibility with `ardupilot_gazebo` allows reuse of existing rover models and sensor configurations from the ArduPilot community.

## User Story (if applicable)

As a developer, I want to run pico_trail in Gazebo with realistic physics and 3D visualization so that I can validate autonomous navigation before testing on real hardware.

## Acceptance Criteria

- [ ] `GazeboAdapter` implements `SimulatorAdapter` trait
- [ ] Configurable UDP ports for sensor receive and actuator send
- [ ] JSON sensor data parsing (IMU, GPS, compass fields)
- [ ] JSON actuator command serialization
- [ ] Compatible with `ardupilot_gazebo` plugin (unmodified)
- [ ] Supports lockstep synchronization via protocol
- [ ] Connection timeout and reconnection handling
- [ ] Error handling for malformed JSON or network failures
- [ ] Configurable Gazebo server address
- [ ] Unit tests with mock UDP socket
- [ ] Integration test with actual Gazebo (CI optional)

## Technical Details (if applicable)

### Configuration

```rust
pub struct GazeboConfig {
    pub sensor_port: u16,        // default: 9002
    pub actuator_port: u16,      // default: 9003
    pub server_addr: SocketAddr,
    pub ardupilot_compat: bool,  // use ardupilot_gazebo protocol
    pub timeout_ms: u32,         // default: 1000
}
```

### JSON Protocol (ardupilot_gazebo compatible)

**Sensor Data (Gazebo → pico_trail)**:

```json
{
  "timestamp": 1234567890,
  "imu": {
    "gyro": [0.001, -0.002, 0.003],
    "accel": [0.01, -0.02, -9.81]
  },
  "position": [35.681236, 139.767125, 40.0],
  "velocity": [1.0, 0.5, 0.0],
  "attitude": [1.0, 0.0, 0.0, 0.0]
}
```

**Actuator Commands (pico_trail → Gazebo)**:

```json
{
  "timestamp": 1234567890,
  "servo1": 1500,
  "servo2": 1500,
  "servo3": 1600,
  "servo4": 1400
}
```

### Implementation

```rust
pub struct GazeboAdapter {
    config: GazeboConfig,
    sensor_socket: Option<UdpSocket>,
    actuator_socket: Option<UdpSocket>,
    sim_time_us: u64,
    connected: bool,
}

impl SimulatorAdapter for GazeboAdapter {
    fn adapter_type(&self) -> &'static str { "gazebo" }
    fn supports_lockstep(&self) -> bool { true }
    // ... full implementation
}
```

## Platform Considerations

### Host Only

- Gazebo adapter runs on host (Linux/macOS), not embedded
- Requires `tokio` or `async-std` for async UDP
- JSON parsing via `serde_json`

### Gazebo Requirements

- Gazebo Harmonic (LTS) or Garden recommended
- `ardupilot_gazebo` plugin installed
- Rover model with sensor plugins

## Risks & Mitigation

| Risk                                    | Impact | Likelihood | Mitigation                              | Validation                |
| --------------------------------------- | ------ | ---------- | --------------------------------------- | ------------------------- |
| Protocol mismatch with ardupilot_gazebo | High   | Low        | Follow existing protocol exactly        | Test with ArduPilot first |
| Gazebo version incompatibility          | Medium | Medium     | Document supported versions             | CI with specific version  |
| JSON parsing performance                | Low    | Low        | Use serde with zero-copy where possible | Benchmark                 |

## Implementation Notes

- PWM values in actuator commands follow ArduPilot convention (1000-2000 µs)
- Convert pico_trail's normalized motor values (-1.0 to 1.0) to PWM
- Consider supporting both ardupilot_gazebo protocol and a simplified custom protocol

## External References

- [ardupilot_gazebo plugin](https://github.com/ArduPilot/ardupilot_gazebo)
- [Gazebo Sim](https://gazebosim.org/)
- [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
