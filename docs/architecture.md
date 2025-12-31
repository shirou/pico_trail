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
│   ├── gps.rs           # UART GPS driver (legacy)
│   ├── gps_i2c.rs       # I2C/DDC GPS driver (NEO-M8N)
│   ├── gps_operation.rs # GPS polling, validation, error recovery
│   ├── imu/             # IMU implementations
│   │   ├── mod.rs
│   │   ├── mpu6050.rs   # MPU6050 driver
│   │   └── bno055.rs    # BNO055 driver (future)
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
├── libraries/          # Level 4: Common vehicle libraries
│   ├── rc_channel/     # RC input processing (vehicle-agnostic)
│   │   └── mod.rs      # RcInput, RC_INPUT global, normalization
│   └── srv_channel/    # Servo/actuator output (vehicle-agnostic)
│       └── mod.rs      # ActuatorInterface, Actuators, calibration
│
├── rover/              # Level 5: Rover vehicle implementation
│   ├── mod.rs          # Module root
│   ├── mode/           # Control mode implementations
│   │   ├── mod.rs      # Mode trait definition
│   │   ├── manual.rs   # Manual mode (RC pass-through)
│   │   ├── circle.rs   # Circle mode (autonomous orbit)
│   │   └── loiter.rs   # Loiter mode (position hold)
│   └── mode_manager.rs # Mode lifecycle management
│
├── boat/               # Level 5: Boat vehicle implementation (future)
│   └── mod.rs
│
├── copter/             # Level 5: Copter vehicle implementation (future)
│   └── mod.rs
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
| I2C Interface      | ✅ Complete | 4     |
| GPS I2C Driver     | ✅ Complete | 6     |
| GPS Operation      | ✅ Complete | 6     |
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

### Peripheral Interfaces

#### I2C0 Bus

The I2C0 bus provides a shared communication channel for multiple sensors, addressing GPIO pin scarcity on resource-constrained boards.

**Hardware Configuration**:

- **Pins**: GPIO 0 (SDA), GPIO 1 (SCL)
- **Clock Speed**: 400 kHz (I2C Fast Mode)
- **Pull-ups**: 4.7kΩ external resistors required on SDA and SCL lines
- **Devices**:
  - NEO-M8N GPS module (address 0x42)
  - BNO085 9-axis IMU (address 0x4A, future)

**Software Architecture**:

```
┌──────────────────────────────────────────┐
│   Navigation Subsystem                   │
│   (Waypoint following, path planning)    │
└────────────────┬─────────────────────────┘
                 │ GpsState
┌────────────────▼─────────────────────────┐
│   GPS Operation Manager                  │
│   - Async polling (1-10 Hz)              │
│   - NMEA validation                      │
│   - Error recovery (3 retries)           │
│   (src/devices/gps_operation.rs)         │
└────────────────┬─────────────────────────┘
                 │ I2cInterface
┌────────────────▼─────────────────────────┐
│   GPS I2C Driver (NEO-M8N)               │
│   - DDC protocol (reg 0xFF, 0xFD)        │
│   - NMEA sentence buffering              │
│   - Checksum validation                  │
│   (src/devices/gps_i2c.rs)               │
└────────────────┬─────────────────────────┘
                 │ I2cInterface trait
┌────────────────▼─────────────────────────┐
│   I2C Platform Abstraction               │
│   - RP2350: src/platform/rp2350/i2c.rs   │
│   - RP2040: src/platform/rp2040/i2c.rs   │
│   - Mock: src/platform/mock/i2c.rs       │
└────────────────┬─────────────────────────┘
                 │ embassy-rp I2C HAL
┌────────────────▼─────────────────────────┐
│   I2C0 Hardware (GPIO 0/1, 400 kHz)      │
└──────────────────────────────────────────┘
```

**GPS I2C/DDC Protocol**:

The NEO-M8N GPS module supports I2C/DDC interface as an alternative to UART:

- **Register 0xFF**: Number of bytes available (2-byte big-endian)
- **Register 0xFD**: Data stream (NMEA sentences)
- **D_SEL Pin**: Must be HIGH or OPEN to enable I2C/DDC mode
- **Data Format**: NMEA 0183 sentences (GPGGA, GPRMC)

**GPS Operation**:

The GPS Operation manager (`src/devices/gps_operation.rs`) handles polling, validation, and error recovery:

- **Polling Rates**: Configurable 1Hz, 5Hz, or 10Hz via Embassy Ticker
- **NMEA Parsing**: Uses `nmea0183` crate for sentence parsing and validation
- **Error Recovery**: 3 retries with exponential backoff (100ms, 200ms, 400ms)
- **Fix Validation**: Rejects NoFix status, validates position ranges
- **Failsafe**: Triggers GPS failsafe after 3 consecutive NoFix readings

**Performance Characteristics**:

- **Latency**: <300ms (GPS fix → position available in GpsState)
- **I2C Transaction Time**: 10-50ms per poll
- **NMEA Parse Time**: <10ms
- **Memory Usage**: \~1 KB (256-byte circular buffer + state)
- **CPU Overhead**: <2% at 1Hz, <5% at 10Hz

**Hardware Requirements**:

- 4.7kΩ pull-up resistors on SDA and SCL lines (to 3.3V)
- NEO-M8N D_SEL pin: HIGH or OPEN (enables I2C/DDC mode)
- Outdoor environment for GPS fix acquisition (indoor may not achieve fix)

**Implementation Status**:

- ✅ I2C Platform Abstraction (RP2350, RP2040, Mock)
- ✅ GPS I2C/DDC Driver (NEO-M8N)
- ✅ GPS Operation Manager (polling, validation, recovery)
- ✅ Unit Tests (16 tests passing)
- ⏸️ Hardware Validation (pending NEO-M8N module)
- ⏸️ IMU Driver (BNO085, future task)

For detailed design and implementation, see:

- [ADR-00mjv: I2C0 Multi-Sensor Bus](adr/ADR-00mjv-i2c0-gps-imu-integration.md)
- [FR-2f599: I2C0 Multi-Sensor Bus Requirements](requirements/FR-2f599-i2c0-multi-sensor-bus.md)
- [Task T-meox8: I2C0 GPS/IMU Integration](tasks/T-meox8-i2c0-gps-imu-integration/)

## Communication

### MAVLink Protocol

Full MAVLink 2.0 implementation using the rust-mavlink crate with custom message handlers. The implementation provides GCS compatibility with QGroundControl 4.x and Mission Planner 1.3.x.

**Architecture**:

```
┌──────────────────────────────────────────┐
│      Application Layer                   │
│   (Scheduler, AHRS, GPS, Parameters)     │
└────────────────┬─────────────────────────┘
                 │ (Data queries)
┌────────────────▼─────────────────────────┐
│   MAVLink Message Handlers               │
│   - ParamHandler (PARAM_*)               │
│   - TelemetryStreamer (HEARTBEAT, etc.)  │
│   - CommandHandler (COMMAND_LONG)        │
│   - MissionHandler (MISSION_*)           │
└────────────────┬─────────────────────────┘
                 │ (MAVLink messages)
┌────────────────▼─────────────────────────┐
│   rust-mavlink Crate                     │
│   - Message parsing (read_v2_msg)        │
│   - Message encoding (serialize)         │
│   - CRC validation                       │
└────────────────┬─────────────────────────┘
                 │ (Byte stream)
┌────────────────▼─────────────────────────┐
│   Platform UART Interface                │
│   (115200 baud, 8N1)                     │
└──────────────────────────────────────────┘
```

**Message Support**:

- **Telemetry (Outbound)**:
  - HEARTBEAT (ID 0) - 1Hz - Vehicle type, armed status, system status
  - SYS_STATUS (ID 1) - 1Hz - Battery voltage/current, CPU load
  - ATTITUDE (ID 30) - 10Hz (configurable via SR_EXTRA1) - Roll, pitch, yaw, rates
  - GPS_RAW_INT (ID 24) - 5Hz (configurable via SR_POSITION) - Lat, lon, alt, fix type

- **Parameter Protocol (Bidirectional)**:
  - PARAM_REQUEST_LIST (ID 21) - Request all parameters
  - PARAM_REQUEST_READ (ID 20) - Request specific parameter
  - PARAM_SET (ID 23) - Set parameter value
  - PARAM_VALUE (ID 22) - Parameter value response

- **Command Protocol (Inbound)**:
  - COMMAND_LONG (ID 76) - Execute command
  - COMMAND_ACK (ID 77) - Command acknowledgment
  - Supported commands:
    - MAV_CMD_COMPONENT_ARM_DISARM (400) - Arm/disarm vehicle
    - MAV_CMD_DO_SET_MODE (176) - Change flight mode
    - MAV_CMD_PREFLIGHT_CALIBRATION (241) - Sensor calibration

- **Mission Protocol (Bidirectional)**:
  - MISSION_COUNT (ID 44) - Mission item count
  - MISSION_ITEM_INT (ID 73) - Mission waypoint (scaled integer coordinates)
  - MISSION_REQUEST_INT (ID 51) - Request specific waypoint
  - MISSION_REQUEST_LIST (ID 43) - Request mission download
  - MISSION_ACK (ID 47) - Mission operation acknowledgment

**Stream Rate Control**:

Telemetry rates are configurable via MAVLink parameters:

| Parameter   | Default | Range   | Controls                 |
| ----------- | ------- | ------- | ------------------------ |
| SR_EXTRA1   | 10 Hz   | 0-50 Hz | ATTITUDE message rate    |
| SR_POSITION | 5 Hz    | 0-50 Hz | GPS_RAW_INT message rate |
| SR_RC_CHAN  | 5 Hz    | 0-50 Hz | RC_CHANNELS message rate |
| SR_RAW_SENS | 5 Hz    | 0-50 Hz | IMU_SCALED message rate  |

**Performance**:

- Memory usage: \~8 KB RAM (buffers + state)
- CPU overhead: \~5% at default rates
- Bandwidth usage: \~40 KB/minute at default rates (\~5% of 115200 baud UART)
- Message latency: < 10ms (COMMAND_LONG → COMMAND_ACK)
- Connection timeout: 5 seconds (missed HEARTBEAT)

**Implementation Status**:

| Component           | Status      | Tests |
| ------------------- | ----------- | ----- |
| Core Infrastructure | ✅ Complete | 24    |
| Parameter Protocol  | ✅ Complete | 14    |
| Telemetry Streaming | ✅ Complete | 15    |
| Command Protocol    | ✅ Complete | 13    |
| Mission Protocol    | ✅ Complete | 21    |
| Hardware Validation | 🚧 Pending  | -     |

**Location**: `src/communication/mavlink/`

For detailed usage guide, see [MAVLink Documentation](mavlink.md).

### Telemetry Streams

Configurable telemetry rates (default 10Hz):

- **Position and attitude**: GPS position, roll/pitch/yaw angles, rotation rates
- **Battery status**: Voltage, current, remaining capacity
- **System health**: CPU load, sensor status, error flags
- **Mode and armed status**: Current flight mode, armed/disarmed state

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

## Vehicle Layer

The vehicle layer implements vehicle-specific control logic following ArduPilot's architecture with common libraries and vehicle-specific implementations. This layer provides the foundation for manual and autonomous control modes.

### Module Structure

```
src/
├── libraries/              # Common libraries (vehicle-agnostic)
│   ├── rc_channel/        # RC input processing
│   │   └── mod.rs         # RcInput, RC_INPUT global, normalization
│   └── srv_channel/       # Servo/actuator output
│       └── mod.rs         # ActuatorInterface, Actuators, calibration
│
├── rover/                  # Rover vehicle implementation
│   ├── mod.rs             # Module root
│   ├── mode/              # Control mode implementations
│   │   ├── mod.rs         # Mode trait definition
│   │   └── manual.rs      # Manual mode (RC pass-through)
│   └── mode_manager.rs    # Mode lifecycle management
│
└── core/scheduler/tasks/
    └── control.rs         # Control loop task (50 Hz, vehicle-agnostic)
```

**Design Rationale**:

- `libraries/`: Vehicle-agnostic functionality shared across Rover, Boat, Copter (analogous to ArduPilot's libraries/)
- `rover/`: Rover-specific control logic and modes (analogous to ArduPilot's Rover/)
- Enables future vehicle types (Boat, Copter) to reuse RC/servo libraries

### RC Input Processing

**Source**: `src/libraries/rc_channel/mod.rs`

RC input is processed from MAVLink RC_CHANNELS messages sent by ground control stations (Mission Planner, QGroundControl).

**Components**:

- `RcInput`: Stores 18 normalized channels (-1.0 to +1.0), timestamp, status
- `RC_INPUT`: Global static Mutex-protected shared state
- `RcStatus`: Enum (Active, Lost, NeverConnected)

**Processing Flow**:

1. MAVLink RC_CHANNELS handler receives message (5-10 Hz)
2. Channels normalized: 0-65535 → -1.0 to +1.0 (center: 32768 → 0.0)
3. RcInput updated with normalized values and timestamp
4. Control loop task checks timeout every 50 Hz (1 second threshold)
5. On timeout: RcStatus::Lost, all channels zeroed

**Channel Mapping** (MAVLink/ArduPilot convention):

- Channel 1: Steering (Roll)
- Channel 2: (Pitch, unused for rovers)
- Channel 3: Throttle
- Channel 4: Yaw (unused for Ackermann steering)

### Actuator Abstraction

**Source**: `src/libraries/srv_channel/mod.rs`

Actuator abstraction provides normalized commands (-1.0 to +1.0) with automatic PWM conversion and safety enforcement.

**Components**:

- `ActuatorInterface`: Trait for steering/throttle commands
- `Actuators`: Implements ActuatorInterface with PWM backend
- `ActuatorConfig`: Calibration parameters (min/neutral/max pulse width)

**Safety Features**:

- **Armed State Enforcement**: All actuator commands check `system_state.is_armed()`
- **Automatic Override**: Disarmed state forces neutral outputs (0.0) regardless of command
- **PWM Conversion**: Normalized value → pulse width (1000-2000 μs) → duty cycle (5-10%)

**Conversion Flow**:

```
Command: set_throttle(0.5)
  ↓
Armed Check: if disarmed → override to 0.0
  ↓
Normalize Clamp: [-1.0, +1.0]
  ↓
Pulse Width: 0.5 → 1750 μs (linear interpolation)
  ↓
Duty Cycle: 1750 μs / 20000 μs = 8.75% (50 Hz PWM)
  ↓
Platform PWM: pwm.set_duty_cycle(0.0875)
```

### Control Mode Framework

**Source**: `src/rover/mode/mod.rs`, `src/rover/mode_manager.rs`

The control mode framework uses trait-based polymorphism for extensible mode implementations.

**Mode Trait**:

```rust
pub trait Mode {
    fn enter(&mut self) -> Result<(), &'static str>;
    fn update(&mut self, dt: f32) -> Result<(), &'static str>;
    fn exit(&mut self) -> Result<(), &'static str>;
    fn name(&self) -> &'static str;
}
```

**Mode Manager**:

- Owns current mode: `Box<dyn Mode>`
- Handles transitions: exit → validate → enter
- Executes active mode at 50 Hz
- Reverts to Manual fallback on mode entry failure

**Control Loop Task** (`src/core/scheduler/tasks/control.rs`):

- Vehicle-agnostic Embassy task (reusable for Boat, Copter)
- Runs at 50 Hz (20ms period)
- Checks RC timeout every iteration
- Calls `mode_manager.execute(current_time_us)`
- Calculates delta time for physics-based updates

### Manual Mode

**Source**: `src/rover/mode/manual.rs`

Manual mode provides direct RC pass-through control with no stabilization.

**Implementation**:

1. Lock RC input (brief lock, <100 μs)
2. Check RC timeout: if lost → unlock, neutral outputs, return
3. Read channel 1 (steering), channel 3 (throttle)
4. Unlock RC input
5. Command actuators: `set_steering()`, `set_throttle()`

**Safety Layers**:

- **Mode Layer**: RC timeout → neutral outputs
- **Actuator Layer**: Disarmed → neutral outputs
- **Platform Layer**: Hardware failsafe (servo/ESC neutral on signal loss)

### Circle Mode

**Source**: `src/rover/mode/circle.rs`

Circle mode provides autonomous circular orbit around a center point. Uses hybrid approach with continuous circle generator feeding look-ahead targets to the SimpleNavigationController.

**Parameters**:

- `CIRC_RADIUS`: Circle radius in meters (default: 20m)
- `CIRC_SPEED`: Target speed in m/s (default: 2.0)
- `CIRC_DIR`: Direction (0=Clockwise, 1=Counter-clockwise)

**Implementation**:

1. On enter: Calculate center point (CIRC_RADIUS ahead in heading direction)
2. Each update: Generate look-ahead target point on circle perimeter
3. Delegate path following to SimpleNavigationController
4. Support stationary mode (CIRC_RADIUS=0)

**References**: ADR-897ov-circle-mode-path-generation, FR-khjpl-circle-mode-implementation

### Loiter Mode

**Source**: `src/rover/mode/loiter.rs`

Loiter mode provides position holding for ground rovers with two behavior types.

**Parameters**:

- `LOIT_TYPE`: Behavior type (0=stop, 1=active hold)
- `LOIT_RADIUS`: Acceptable drift radius in meters (default: 2.0m)

**Type 0 (Stop)**:

- Simply stops motors and records loiter position
- No active correction for drift
- Suitable for flat terrain

**Type 1 (Active Hold)**:

- Monitors drift from loiter position using Haversine distance
- Hysteresis prevents oscillation (0.8 factor)
- Navigates back when drift exceeds LOIT_RADIUS
- Degrades to Type 0 on GPS loss

**Implementation**:

1. On enter: Validate GPS fix, calculate loiter point (current or projected stop)
2. Each update (Type 1):
   - Calculate distance to loiter point
   - Hysteresis state machine: start correcting > radius, stop < radius\*0.8
   - Navigate back using SimpleNavigationController when correcting
3. On GPS loss: Degrade to Type 0 (stop motors)

**References**: ADR-8icsq-vehicle-type-separation, FR-aw3h3-rover-loiter-mode

### MAVLink Integration

**Mode Switching**:

- `MAV_CMD_DO_SET_MODE`: Switch between Manual, Hold, Auto, RTL, Guided
- Mode validation: Reject invalid modes (e.g., Auto without GPS)
- Response: `COMMAND_ACK` with `MAV_RESULT_ACCEPTED` or `MAV_RESULT_FAILED`

**RC Input**:

- `RC_CHANNELS`: 18-channel RC input from ground control station
- Update rate: 5-10 Hz (configurable in Mission Planner/QGC)
- Handler: `src/communication/mavlink/handlers/rc_input.rs`

### Performance Characteristics

- **RC Input Latency**: < 100ms (RC_CHANNELS reception → actuator response)
- **Mode Update Latency**: < 1ms average, < 5ms max
- **Control Loop Frequency**: 50 Hz (20ms period)
- **Memory Usage**: \~4 KB RAM (vehicle layer total)

### Future Enhancements

Planned additions (deferred from current implementation):

- **Additional Modes**: Hold, Auto, RTL, Guided (Circle and Loiter implemented)
- **Physical RC Receiver**: SBUS/PPM input support
- **RC Input Filtering**: Low-pass filter for noisy inputs
- **Actuator Features**: Rate limiting, deadband, expo curves
- **Differential Steering**: Skid-steer and differential drive support
- **Boat-specific Loiter**: Different loiter behavior for marine vehicles

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
