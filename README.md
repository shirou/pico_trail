# pico_trail

An embedded autopilot system for Raspberry Pi Pico W and Pico 2 W, targeting rover and boat autonomous navigation. Built with embedded Rust for memory safety and real-time performance, pico_trail implements a subset of ArduPilot/Pixhawk functionality optimized for resource-constrained microcontrollers.

## Features

- **Autonomous Navigation**: Waypoint-based navigation using S-curve path planning for smooth trajectory generation
- **MAVLink Compatible**: Full compatibility with ground control stations (QGroundControl, Mission Planner)
- **Wireless Communication**: WiFi-enabled UDP transport for wireless telemetry and control (Pico 2 W)
- **Multi-Transport Support**: Concurrent UART and UDP operation with automatic GCS discovery
- **Real-time Control**: Deterministic control loops with 50Hz minimum performance
- **Memory Safe**: Built with Rust's ownership model, prioritizing safety over micro-optimizations
- **Platform Abstraction**: Clean separation between hardware and application logic for portability
- **Safety First**: GPS failsafe, RC failsafe, battery monitoring, and geofencing capabilities

## Supported Platforms

| Platform              | CPU                 | Flash | RAM    | Status  |
| --------------------- | ------------------- | ----- | ------ | ------- |
| Raspberry Pi Pico W   | RP2040 (Cortex-M0+) | 2 MB  | 264 KB | Planned |
| Raspberry Pi Pico 2 W | RP2350 (Cortex-M33) | 4 MB  | 520 KB | Planned |

## Design Goals

1. **Extensibility**: Support multiple hardware platforms through clean abstraction layers
2. **Memory Safety**: Prioritize Rust's ownership model over micro-optimizations
3. **Real-time Performance**: Maintain deterministic control loop timing (50Hz minimum)
4. **MAVLink Compatibility**: Full compatibility with ground control stations
5. **Rover/Boat Focus**: Exclude aerial drone features to reduce complexity

## Prerequisites

### 1. Rust Toolchain

Install Rust and add the target for your platform:

```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Add ARM Cortex-M target
rustup target add thumbv8m.main-none-eabihf  # For Pico 2 W (RP2350)
```

### 2. probe-rs

Install probe-rs for flashing and debugging:

```bash
# Follow instructions at https://probe.rs/docs/getting-started/installation/
```

### 3. flip-link

Install flip-link for stack overflow protection:

```bash
cargo install flip-link
```

### 4. Bun (for documentation tooling)

Install Bun for markdown formatting and linting:

```bash
# Follow instructions at https://bun.sh/
curl -fsSL https://bun.sh/install | bash
```

## Quick Start

### 1. Configure Hardware

Edit `.cargo/config.toml` to match your hardware:

```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = ["probe-rs", "run", "--chip", "RP2350", "--log-format=oneline"]

[build]
target = "thumbv8m.main-none-eabihf"  # For Pico 2 W
```

### 2. Build and Flash

**UART-Only Example:**

```bash
# Build for RP2350 (Pico 2 W)
./scripts/build-rp2350.sh --release mavlink_demo

# Flash to device
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo
```

**WiFi-Enabled Example (Pico 2 W):**

Option 1: Build-time WiFi configuration (recommended for development):

```bash
# 1. Configure WiFi credentials
cp .env.example .env
# Edit .env with your WiFi credentials

# 2. Build (automatically loads .env)
./scripts/build-rp2350.sh --release mavlink_demo_network

# 3. Flash - device connects to WiFi automatically
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo_network

# 4. Connect QGroundControl via UDP (port 14550, listening mode)
```

Option 2: Runtime WiFi configuration (recommended for production):

```bash
# 1. Build without .env file
./scripts/build-rp2350.sh --release mavlink_demo_network

# 2. Flash to device
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/mavlink_demo_network

# 3. Configure WiFi via UART:
#    - Connect QGroundControl to UART (115200 baud)
#    - Open Parameters tab
#    - Set NET_SSID = "YourWiFiNetwork"
#    - Set NET_PASS = "YourPassword"
#    - Reboot device
#    - Connect QGroundControl via UDP (port 14550, listening mode)
```

See [WiFi Configuration Guide](docs/wifi-configuration.md) for detailed setup instructions.

### 3. Run Tests

```bash
# Run core crate unit tests (pure no_std logic)
cargo test -p pico_trail_core --lib --quiet
```

## Development

### Project Structure

pico_trail uses a Cargo workspace with two crates:

```
crates/
├── core/              # Pure no_std business logic (platform-independent)
│   └── src/
│       ├── traits/    # Time, platform abstractions
│       ├── kinematics/# Differential drive math
│       ├── parameters/# Parameter types and CRC
│       ├── arming/    # Arming error types
│       ├── scheduler/ # Task scheduling types
│       ├── navigation/# Navigation types
│       ├── ahrs/      # Calibration math
│       ├── mission/   # Waypoint and mission storage
│       ├── mode/      # Mode trait and state types
│       ├── rc/        # RC normalization functions
│       ├── servo/     # Servo PWM conversion
│       └── motor/     # Motor driver traits
│
└── firmware/          # Embassy/RP2350 binary (platform-specific)
    └── src/
        ├── platform/  # Hardware abstraction layer (UART, I2C, SPI, PWM)
        ├── devices/   # Device drivers (GPS, IMU, Motor, Servo)
        ├── subsystems/# Functional subsystems (AHRS, Navigation)
        ├── rover/     # Rover vehicle modes
        ├── communication/ # MAVLink protocol
        ├── parameters/# ArduPilot parameter definitions
        └── core/      # Logging, parameter saver, arming tasks
```

See [docs/architecture.md](docs/architecture.md) for detailed architecture documentation.

### Development Workflow

This project follows the Traceable Development Lifecycle (TDL), inspired by [kopi](https://github.com/kopi-vm/kopi). See [docs/tdl.md](docs/tdl.md) for the complete workflow.

#### Code Quality Checks

When finishing any Rust coding task, always run:

```bash
cargo fmt                                           # Auto-format code
cargo clippy -p pico_trail_core -- -D warnings      # Lint core crate
cargo test -p pico_trail_core --lib --quiet         # Run unit tests
./scripts/build-rp2350.sh pico_trail_rover          # Verify embedded build
```

#### Documentation

When working on markdown documentation:

```bash
bun format  # Auto-format markdown files
bun lint    # Check markdown linting
```

### Development Principles

- **Memory Safety Over Micro-optimization**: Accept reasonable overhead to avoid memory leaks
- **Code Clarity**: Write clear, readable code with descriptive names
- **Clean Code Maintenance**: Remove unused code promptly
- **Prefer Functions Over Structs Without State**: Only create structs when managing state
- **Avoid Generic Naming**: Use specific names instead of "Manager" or "Utils"

## Architecture

pico_trail follows a 5-layer architecture:

```
┌─────────────────────────────────────────────────────────┐
│              Application Layer                          │
│         (Rover/Boat Modes & Mission Management)         │
├─────────────────────────────────────────────────────────┤
│            Communication Layer                          │
│    (MAVLink Protocol, Telemetry, Parameters)            │
├─────────────────────────────────────────────────────────┤
│        Control & Navigation Layer                       │
│  (AHRS, PID, Waypoint Navigation, Path Following)       │
├─────────────────────────────────────────────────────────┤
│            Device Driver Layer                          │
│    (GPS, IMU, Motor, Servo Device Abstractions)         │
├─────────────────────────────────────────────────────────┤
│      Hardware Abstraction Layer (Platform HAL)          │
│    (UART, I2C, SPI, PWM - Platform Specific)            │
└─────────────────────────────────────────────────────────┘
```

See [docs/architecture.md](docs/architecture.md) for complete details.

## Navigation & Control

### Path Following

- **S-Curve Path Planning**: Primary method for smooth waypoint transitions with velocity/acceleration limits
- **Position Controller**: Follows S-curve generated paths with calculated speed and turn rate
- **L1 Controller**: Legacy path following algorithm (fallback/alternative for simple navigation)

### AHRS (Attitude & Heading Reference System)

- **DCM (Direction Cosine Matrix)**: Lightweight attitude estimation
- **Complementary Filter**: Accelerometer + gyroscope fusion
- **Compass Integration**: Heading correction from magnetometer

### Compass Calibration

Compass calibration is supported via Mission Planner. **Only "Large Vehicle" mode is available** - standard "Onboard Mag Calibration" is not supported due to resource constraints on the microcontroller.

## Communication

### MAVLink Protocol

Full MAVLink 2.0 implementation supporting:

- Common messages (HEARTBEAT, ATTITUDE, GPS_RAW_INT, RC_CHANNELS)
- Mission protocol (upload/download, waypoint management)
- Parameter protocol (PARAM_REQUEST_LIST, PARAM_SET)
- Command protocol (navigation and action commands)

### Transport Options

**UART Transport (All platforms):**

- Reliable wired communication (115200 baud)
- Primary transport for initial configuration
- Single GCS connection

**UDP Network Transport (Pico 2 W only):**

- Wireless communication over WiFi (port 14550)
- Multiple GCS support (up to 4 simultaneous)
- Automatic GCS endpoint discovery
- Concurrent UART + UDP operation
- WiFi configuration via MAVLink parameters

See [WiFi Configuration Guide](docs/wifi-configuration.md) for setup instructions.

### Telemetry

Configurable telemetry streams (default 10Hz):

- Position and attitude
- Battery status
- System health
- Mode and armed status

Telemetry is broadcast to all active transports (UART and UDP).

## Safety Features

- **Failsafe**: GPS loss, RC loss, battery low actions
- **Geofence**: Maximum distance/speed limits
- **Pre-arm checks**: Sensor calibration, GPS lock, battery status
- **State machine**: Disarmed → Armed → Emergency transitions

## Documentation

- [Architecture & Structure](docs/architecture.md) - Project structure, components, and design
- [WiFi Configuration Guide](docs/wifi-configuration.md) - WiFi and UDP transport setup
- [MAVLink Guide](docs/mavlink.md) - MAVLink protocol usage and GCS setup
- [TDL Process](docs/tdl.md) - Traceable Development Lifecycle workflow
- [Analysis Documents](docs/analysis/) - Problem exploration and requirements discovery
- [Requirements](docs/requirements/) - Functional and non-functional requirements
- [ADRs](docs/adr/) - Architecture Decision Records
- [Tasks](docs/tasks/) - Implementation tasks with design and plans

## Known Bugs

### Mission Upload with Mission Planner

Standard "Write" mission upload does not work correctly with Mission Planner. The GCS repeatedly requests the same waypoint (seq=0) despite receiving valid MISSION_REQUEST_INT responses with correct target addressing.

**Workaround**: Use **"Write Fast"** instead of "Write" in Mission Planner. Write Fast works correctly.

**Status**: Under investigation. The issue may be related to timing or protocol differences between Write and Write Fast modes.

## Contributing

Contributions are welcome! Please ensure:

- All code passes `cargo fmt`, `cargo clippy -p pico_trail_core -- -D warnings`, and `cargo test -p pico_trail_core --lib --quiet`
- Embedded build passes: `./scripts/build-rp2350.sh pico_trail_rover`
- Documentation is written in English
- Changes follow the TDL process (see [docs/tdl.md](docs/tdl.md))
- Traceability is maintained (`bun scripts/trace-status.ts --check`)

## License

This project is dual-licensed:

- **Non-commercial use**: Licensed under Apache License, Version 2.0 (the "License")
- **Commercial use**: Requires a separate commercial license. Please contact the project maintainer for licensing terms and agreements.

If you intend to use this software for commercial purposes, you must obtain explicit permission and a commercial license before deployment. Commercial use without a proper license is not permitted.

See [LICENSE](LICENSE) for the non-commercial license terms.

## Acknowledgments

- Inspired by [ArduPilot](https://ardupilot.org/) and [Pixhawk](https://px4.io/)
- Built with [Embassy](https://embassy.dev/) async framework
- Hardware abstraction based on [embedded-hal](https://github.com/rust-embedded/embedded-hal)
- Initial template from [knurling-rs/app-template](https://github.com/knurling-rs/app-template)
- Traceable Development Lifecycle (TDL) inspired by [kopi](https://github.com/kopi-vm/kopi)
