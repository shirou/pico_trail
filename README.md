# pico_trail

An embedded autopilot system for Raspberry Pi Pico W and Pico 2 W, targeting rover and boat autonomous navigation. Built with embedded Rust for memory safety and real-time performance, pico_trail implements a subset of ArduPilot/Pixhawk functionality optimized for resource-constrained microcontrollers.

## Features

- **Autonomous Navigation**: Waypoint-based navigation using S-curve path planning for smooth trajectory generation
- **MAVLink Compatible**: Full compatibility with ground control stations (QGroundControl, Mission Planner)
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

```bash
# Build the project
cargo build --release

# Run an example (flashes and executes)
cargo run --bin hello --release
```

### 3. Run Tests

```bash
# Run unit tests
cargo test --lib --quiet
```

## Development

### Project Structure

```
src/
├── platform/          # Hardware abstraction layer (UART, I2C, SPI, PWM)
├── devices/           # Device drivers (GPS, IMU, Motor, Servo)
├── subsystems/        # Functional subsystems (AHRS, Control, Navigation)
├── vehicle/           # Vehicle logic and flight modes
└── core/              # Cross-cutting concerns (Scheduler, Parameters, Logger)
```

See [docs/architecture.md](docs/architecture.md) for detailed architecture documentation.

### Development Workflow

This project follows the Traceable Development Lifecycle (TDL). See [docs/tdl.md](docs/tdl.md) for the complete workflow.

#### Code Quality Checks

When finishing any Rust coding task, always run:

```bash
cargo fmt                                        # Auto-format code
cargo clippy --all-targets -- -D warnings        # Lint with strict warnings
cargo test --lib --quiet                         # Run unit tests
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

## Communication

### MAVLink Protocol

Full MAVLink 2.0 implementation supporting:

- Common messages (HEARTBEAT, ATTITUDE, GPS_RAW_INT, RC_CHANNELS)
- Mission protocol (upload/download, waypoint management)
- Parameter protocol (PARAM_REQUEST_LIST, PARAM_SET)
- Command protocol (navigation and action commands)

### Telemetry

Configurable telemetry streams (default 10Hz):

- Position and attitude
- Battery status
- System health
- Mode and armed status

## Safety Features

- **Failsafe**: GPS loss, RC loss, battery low actions
- **Geofence**: Maximum distance/speed limits
- **Pre-arm checks**: Sensor calibration, GPS lock, battery status
- **State machine**: Disarmed → Armed → Emergency transitions

## Documentation

- [Architecture & Structure](docs/architecture.md) - Project structure, components, and design
- [TDL Process](docs/tdl.md) - Traceable Development Lifecycle workflow
- [Analysis Documents](docs/analysis/) - Problem exploration and requirements discovery
- [Requirements](docs/requirements/) - Functional and non-functional requirements
- [ADRs](docs/adr/) - Architecture Decision Records
- [Tasks](docs/tasks/) - Implementation tasks with design and plans

## Contributing

Contributions are welcome! Please ensure:

- All code passes `cargo fmt`, `cargo clippy --all-targets -- -D warnings`, and `cargo test --lib --quiet`
- Documentation is written in English
- Changes follow the TDL process (see [docs/tdl.md](docs/tdl.md))
- Traceability is maintained (`bun scripts/trace-status.ts --check`)

## License

Licensed under the BSD 2-Clause License. See [LICENSE](LICENSE) for details.

## Acknowledgments

- Inspired by [ArduPilot](https://ardupilot.org/) and [Pixhawk](https://px4.io/)
- Built with [Embassy](https://embassy.dev/) async framework
- Hardware abstraction based on [embedded-hal](https://github.com/rust-embedded/embedded-hal)
- Initial template from [knurling-rs/app-template](https://github.com/knurling-rs/app-template)
