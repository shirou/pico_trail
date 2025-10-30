# Architecture

## Overview

pico_trail is an embedded autopilot system for Raspberry Pi Pico W and Pico 2 W, targeting rover and boat autonomous navigation. The project implements a subset of ArduPilot/Pixhawk functionality optimized for resource-constrained microcontrollers, using embedded Rust for memory safety and real-time performance.

## Design Goals

1. **Extensibility**: Support multiple hardware platforms through clean abstraction layers
2. **Memory Safety**: Prioritize Rust's ownership model over micro-optimizations
3. **Real-time Performance**: Maintain deterministic control loop timing (50Hz minimum)
4. **MAVLink Compatibility**: Full compatibility with ground control stations
5. **Rover/Boat Focus**: Exclude aerial drone features to reduce complexity

## System Architecture

### Layered Architecture

The system follows a 5-layer architecture for clear separation of concerns:

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

**Layer Principles**:

- Each layer depends only on layers below it
- Platform-specific code is isolated in the Platform HAL
- Device drivers use platform-independent traits
- Control algorithms are pure functions where possible

### Abstraction Strategy

The project uses a 3-tier abstraction approach:

```
Application/Control Code (Platform-independent)
        ↓
Device & Platform Traits (Project-specific abstractions)
        ↓
Platform Implementations (Pico W, Pico 2W, future: ESP32, STM32)
        ↓
embedded-hal (External standard)
```

This strategy provides:

- **Flexibility**: Easy to add new hardware platforms
- **Testability**: Mock implementations for unit tests
- **Clarity**: Clear boundaries between hardware and logic

## Directory Structure

```
src/
├── platform/              # Level 1: Platform abstraction
│   ├── mod.rs            # Core platform traits
│   ├── traits/           # Peripheral trait definitions
│   │   ├── uart.rs       # UART interface
│   │   ├── i2c.rs        # I2C interface
│   │   ├── spi.rs        # SPI interface
│   │   ├── pwm.rs        # PWM interface
│   │   ├── timer.rs      # Timer interface
│   │   ├── storage.rs    # Flash/EEPROM interface
│   │   └── gpio.rs       # GPIO for LED/Buzzer
│   ├── pico_w/           # RP2040 (Cortex-M0+) implementation
│   │   ├── mod.rs
│   │   └── hal_adapter.rs
│   └── pico2_w/          # RP2350 (Cortex-M33) implementation
│       ├── mod.rs
│       └── hal_adapter.rs
│
├── devices/              # Level 2: Device abstraction
│   ├── traits/           # Device trait definitions
│   │   ├── gps.rs       # GPS sensor trait
│   │   ├── imu.rs       # IMU sensor trait
│   │   ├── motor.rs     # Motor controller trait
│   │   └── servo.rs     # Servo controller trait
│   ├── gps/             # GPS implementations
│   │   ├── mod.rs
│   │   ├── nmea.rs      # NMEA protocol parser
│   │   └── ublox.rs     # u-blox specific driver
│   ├── imu/             # IMU implementations
│   │   ├── mod.rs
│   │   ├── mpu6050.rs   # MPU6050 driver
│   │   └── bno055.rs    # BNO055 driver
│   ├── motor/           # Motor/ESC drivers
│   └── servo/           # Servo drivers
│
├── subsystems/          # Level 3: Functional subsystems
│   ├── ahrs/            # Attitude & Heading Reference System
│   │   ├── mod.rs
│   │   ├── dcm.rs       # Direction Cosine Matrix
│   │   └── ekf_simple.rs # Simplified EKF (full EKF too heavy)
│   ├── control/         # Control algorithms
│   │   ├── pid.rs       # PID controller
│   │   ├── attitude.rs  # Attitude control
│   │   └── throttle.rs  # Throttle/speed control
│   ├── navigation/      # Navigation subsystem
│   │   ├── waypoint.rs  # Waypoint management
│   │   ├── heading.rs   # Heading control
│   │   ├── scurve.rs    # S-Curve path planning (primary)
│   │   ├── position_control.rs # Position controller
│   │   ├── l1_controller.rs # L1 path following (legacy/fallback)
│   │   └── path.rs      # Path representation
│   └── communication/   # Communication protocols
│       ├── mavlink/     # MAVLink implementation
│       │   ├── mod.rs
│       │   ├── messages.rs # Message encoding/decoding
│       │   ├── params.rs   # Parameter protocol
│       │   └── mission.rs  # Mission upload/download
│       └── telemetry.rs # Telemetry streaming
│
├── vehicle/            # Level 4: Vehicle logic
│   ├── mod.rs
│   ├── modes/          # Flight modes
│   │   ├── manual.rs   # Manual control
│   │   ├── hold.rs     # Position hold
│   │   ├── auto.rs     # Autonomous mission
│   │   ├── rtl.rs      # Return to launch
│   │   └── guided.rs   # GCS-guided control
│   ├── rover.rs        # Rover-specific logic
│   ├── boat.rs         # Boat-specific logic
│   └── common.rs       # Shared vehicle logic
│
├── core/              # Cross-cutting concerns
│   ├── scheduler.rs   # Task scheduler
│   ├── parameters.rs  # Parameter system
│   ├── logger.rs      # Data logging
│   ├── storage.rs     # Persistent storage
│   ├── safety.rs      # Failsafe & geofence
│   ├── calibration.rs # Sensor calibration
│   ├── state.rs       # System state machine
│   ├── notify.rs      # Notification system
│   ├── config.rs      # Configuration management
│   └── error.rs       # Error definitions
│
└── lib.rs            # Library root
```

## Core Systems

### Task Scheduler

The scheduler manages periodic and event-driven tasks with fixed priorities:

- **High Priority (400Hz)**: IMU data acquisition
- **Medium Priority (50Hz)**: Control loops, attitude estimation
- **Low Priority (10Hz)**: Telemetry, logging, parameter updates
- **Background**: Mission planning, storage operations

Based on Embassy async executor for efficient task management.

### Parameter System

Runtime-configurable parameters with Flash-backed persistence:

**Architecture**:

```
┌──────────────────────────────────────────┐
│      Application Layer                   │
│   (MAVLink PARAM_SET/GET handlers)       │
└────────────────┬─────────────────────────┘
                 │
┌────────────────▼─────────────────────────┐
│   Parameter Registry (RAM Cache)         │
│   - Type-safe parameter access           │
│   - Bounds validation                    │
│   - Modified flag tracking               │
└────────────────┬─────────────────────────┘
                 │
┌────────────────▼─────────────────────────┐
│   Flash Parameter Storage                │
│   - 4-block round-robin rotation         │
│   - CRC32 validation                     │
│   - Wear leveling statistics             │
└────────────────┬─────────────────────────┘
                 │
┌────────────────▼─────────────────────────┐
│   Platform Flash Interface               │
│   - RP2040/RP2350 ROM functions          │
│   - Critical section protection          │
└──────────────────────────────────────────┘
```

**Features**:

- **Fast Loading**: Parameters load in < 100ms during initialization
- **Non-Blocking Saves**: Async save with < 5ms blocking time
- **Wear Leveling**: 4-block rotation distributes writes evenly (40,000+ saves)
- **Corruption Recovery**: CRC32 validation with automatic fallback
- **Power-Loss Protection**: Redundant blocks survive unexpected power loss
- **Capacity**: Supports 200+ parameters per platform

**Flash Layout** (RP2040/RP2350):

```
Flash Address       Size      Purpose
─────────────────────────────────────────────
0x000000-0x03FFFF  256 KB    Firmware (protected)
0x040000-0x040FFF    4 KB    Parameter Block 0
0x041000-0x041FFF    4 KB    Parameter Block 1
0x042000-0x042FFF    4 KB    Parameter Block 2
0x043000-0x043FFF    4 KB    Parameter Block 3
0x044000-0x045FFF    8 KB    Mission Storage (future)
0x046000-0x3FFFFF  ~3.7 MB   Log Storage (future)
```

**Usage**:

- Parameter groups (e.g., `RATE_ROLL_P`, `SYSID_THISMAV`, `SR_EXTRA1`)
- Type-safe access (Float32, UInt32)
- MAVLink parameter protocol support (PARAM_REQUEST_LIST, PARAM_SET, PARAM_VALUE)
- Range validation and default values
- Debounced saves (5-second delay to reduce Flash wear)

For detailed documentation, see [docs/parameters.md](parameters.md).

### Data Logger

High-frequency binary logging to Flash:

- Timestamped sensor data (IMU, GPS, control outputs)
- Efficient binary format for storage optimization
- Log download via MAVLink
- Selective logging based on flight modes

### Safety Features

Critical safety systems:

- **Failsafe**: GPS loss, RC loss, battery low actions
- **Geofence**: Maximum distance/speed limits
- **Pre-arm checks**: Sensor calibration, GPS lock, battery status
- **State machine**: Disarmed → Armed → Emergency transitions

## Hardware Platform Support

### Platform Abstraction Layer

The platform abstraction layer provides zero-cost hardware independence through Rust traits and compile-time dispatch. All platform-specific code is isolated to `src/platform/` per NFR-nmmu0.

**Architecture**:

```
Application/Device Code (Platform-independent)
        ↓ uses traits
Platform Traits (UART, I2C, SPI, PWM, GPIO, Timer)
        ↓ implemented by
Platform Implementations (RP2350, RP2040, Mock)
        ↓ uses
HAL Crates (rp235x-hal, rp2040-hal)
```

**Key Features**:

- **Zero HAL Imports**: No HAL imports outside `src/platform/`
- **Mock Testing**: Complete mock implementations for unit tests
- **Compile-time Dispatch**: Zero-cost abstractions via trait monomorphization
- **CI Enforcement**: Automated checks prevent HAL leakage

**Platform Trait Hierarchy**:

```rust
// Root platform trait
pub trait Platform {
    type Uart: UartInterface;
    type I2c: I2cInterface;
    type Spi: SpiInterface;
    type Pwm: PwmInterface;
    type Gpio: GpioInterface;
    type Timer: TimerInterface;

    fn init() -> Result<Self>;
    fn create_uart(&mut self, id: u8, config: UartConfig) -> Result<Self::Uart>;
    // ... other peripheral creation methods
}
```

**Example Usage**:

```rust
// Device driver generic over UartInterface
pub struct GpsDriver<U: UartInterface> {
    uart: U,
}

impl<U: UartInterface> GpsDriver<U> {
    pub fn read_position(&mut self) -> Result<Option<GpsPosition>> {
        // Works with any UART implementation (RP2350, RP2040, Mock)
    }
}
```

**Implementation Status**:

| Component          | Status      | Tests |
| ------------------ | ----------- | ----- |
| Platform Traits    | ✅ Complete | -     |
| Mock Platform      | ✅ Complete | 26    |
| RP2350 Platform    | 🚧 Partial  | -     |
| RP2040 Platform    | ⏸️ Planned  | -     |
| Example GPS Driver | ✅ Complete | 4     |
| CI HAL Isolation   | ✅ Complete | -     |

### Supported Platforms

| Platform              | CPU                 | Flash | RAM    | Status      |
| --------------------- | ------------------- | ----- | ------ | ----------- |
| Raspberry Pi Pico W   | RP2040 (Cortex-M0+) | 2 MB  | 264 KB | Planned     |
| Raspberry Pi Pico 2 W | RP2350 (Cortex-M33) | 4 MB  | 520 KB | In Progress |
| Mock (Testing)        | Host CPU            | -     | -      | Complete    |

### Platform Selection Strategy

- **Pico 2 W Primary**: Focus development on Pico 2 W (ARM Cortex-M33) for better performance
- **Pico W Support**: Maintain compatibility via platform abstraction layer
- **Future Expansion**: Architecture supports ESP32, STM32F4 through trait implementation
- **Mock Platform**: Enables hardware-free unit testing in CI

### Adding a New Platform

To add support for a new hardware platform:

1. **Create Platform Module**: `src/platform/<platform_name>/`

   ```rust
   // src/platform/<platform_name>/mod.rs
   #[cfg(feature = "<platform_name>")]
   pub mod uart;
   pub mod i2c;
   // ... other peripherals
   pub mod platform;
   ```

2. **Implement Peripheral Traits**: Each peripheral (UART, I2C, etc.) must implement the corresponding trait

   ```rust
   // src/platform/<platform_name>/uart.rs
   use crate::platform::traits::UartInterface;

   pub struct MyPlatformUart { /* HAL wrapper */ }

   impl UartInterface for MyPlatformUart {
       fn write(&mut self, data: &[u8]) -> Result<usize> {
           // Use platform HAL
       }
       // ... implement other trait methods
   }
   ```

3. **Implement Platform Trait**: Create the root platform struct

   ```rust
   // src/platform/<platform_name>/platform.rs
   use crate::platform::traits::Platform;

   pub struct MyPlatform { /* peripheral state */ }

   impl Platform for MyPlatform {
       type Uart = MyPlatformUart;
       // ... other associated types

       fn init() -> Result<Self> {
           // Initialize clocks, peripherals
       }

       fn create_uart(&mut self, id: u8, config: UartConfig) -> Result<Self::Uart> {
           // Configure and return UART instance
       }
   }
   ```

4. **Add Feature Flag**: Update `Cargo.toml`

   ```toml
   [features]
   my_platform = ["<hal-crate>", "cortex-m", ...]
   ```

5. **Update Platform Module**: Add feature gate in `src/platform/mod.rs`
   ```rust
   #[cfg(feature = "my_platform")]
   pub mod my_platform;
   ```

**Requirements**:

- All HAL imports must stay within `src/platform/<platform_name>/`
- Implement all platform traits completely
- Provide platform-specific initialization sequence
- Document peripheral pin mappings and configuration

### Memory Constraints

**RP2040 (Pico W)** - 264 KB RAM:

- Minimal EKF (no full 15-state EKF)
- Limited log buffer (4 KB circular buffer)
- Simplified mission storage (max 50 waypoints)

**RP2350 (Pico 2 W)** - 520 KB RAM:

- Extended EKF with 9 states
- Larger log buffer (16 KB)
- Extended mission capacity (200 waypoints)

## Communication

### MAVLink Protocol

Full MAVLink 2.0 implementation:

- **Common Messages**: HEARTBEAT, ATTITUDE, GPS_RAW_INT, RC_CHANNELS
- **Mission Protocol**: Mission upload/download, current waypoint
- **Parameter Protocol**: PARAM_REQUEST_LIST, PARAM_SET
- **Command Protocol**: MAV_CMD_NAV\_\_, MAV_CMD_DO\_\_

### Telemetry Streams

Configurable telemetry rates (default 10Hz):

- Position and attitude
- Battery status
- System health
- Mode and armed status

## Navigation & Control

### AHRS (Attitude & Heading Reference System)

- **DCM (Direction Cosine Matrix)**: Lightweight attitude estimation
- **Complementary Filter**: Accelerometer + gyroscope fusion
- **Compass Integration**: Heading correction from magnetometer

### Control Loops

- **Attitude Control**: Roll/pitch/yaw PID controllers
- **Speed Control**: Throttle PID with acceleration limits
- **Steering Control**: Lateral acceleration to steering angle conversion

### Path Following

- **S-Curve Path Planning**: Smooth trajectory generation between waypoints (ArduPilot 4.3+ primary method)
  - Considers velocity and acceleration limits
  - Generates continuous position and velocity targets
  - Prevents abrupt direction changes at waypoints
- **Position Controller**: Follows S-curve generated path
  - Calculates desired speed and turn rate
  - Feeds lower-level steering and throttle controllers
- **L1 Controller**: Legacy path following algorithm (fallback/alternative)
  - Converts origin/destination to lateral acceleration
  - Still useful for simple point-to-point navigation

## Development Workflow

### Build System

- **Cargo**: Standard Rust build system
- **Platform Selection**: Feature flags (`pico_w`, `pico2_w`)
- **probe-rs**: Flash and debug tool

### Testing Strategy

- **Unit Tests**: Pure logic tested on host
- **Hardware-in-Loop**: Device-specific tests on target
- **Simulation**: SITL (Software In The Loop) support planned

## Related Documentation

- [TDL Process](tdl.md) - Traceable Development Lifecycle
- Analysis Documents:
  - [AN-cp76d: ArduPilot Analysis](analysis/AN-cp76d-ardupilot-analysis.md)
  - [AN-kir7h: Platform Abstraction](analysis/AN-kir7h-platform-abstraction.md)
  - [AN-5nucb: Core Systems](analysis/AN-5nucb-core-systems.md)
  - [AN-7ix56: Navigation Approach (S-Curve vs L1)](analysis/AN-7ix56-navigation-approach.md)
- [ADRs](adr/) - Architecture Decision Records
- [Requirements](requirements/) - Functional and non-functional requirements
