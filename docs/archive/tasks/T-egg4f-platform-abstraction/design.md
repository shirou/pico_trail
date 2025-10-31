# T-egg4f Platform Abstraction Layer

## Metadata

- Type: Design
- Status: Approved

## Links

- Associated Plan Document:
  - [T-egg4f-platform-abstraction-plan](plan.md)
- Related ADRs:
  - [ADR-oa2qa-platform-abstraction](../../../adr/ADR-oa2qa-platform-abstraction.md)
- Related Requirements:
  - [NFR-nmmu0-platform-code-isolation](../../../requirements/NFR-nmmu0-platform-code-isolation.md)
  - [NFR-pj11s-no-unsafe-rust](../../../requirements/NFR-pj11s-no-unsafe-rust.md)

## Overview

Implement a three-tier platform abstraction layer that isolates all hardware-specific code to `src/platform/`, enabling the autopilot to support multiple microcontroller platforms (Raspberry Pi Pico W, Pico 2 W, and future ESP32/STM32) without changing device drivers or subsystems. The design uses Rust generics for zero-cost abstractions, maintaining compile-time dispatch while providing clean separation of concerns and testability through mock implementations.

## Success Metrics

- [ ] Zero HAL imports outside `src/platform/` (verified via CI check)
- [ ] Device drivers compile and function identically across RP2040 and RP2350 platforms
- [ ] Control loop overhead < 1% compared to direct HAL usage (measured via cycle counter)
- [ ] Unit tests pass with mock platform implementation (no hardware required)
- [ ] Adding ESP32 support requires only `src/platform/esp32/` implementation (< 1000 LOC estimate)

## Background and Current State

- Context: This is a greenfield embedded autopilot system targeting resource-constrained microcontrollers (RP2040: 264KB RAM, RP2350: 520KB RAM). The system must maintain 50Hz control loop performance while supporting multiple hardware platforms.
- Current behavior: No platform abstraction exists yet. This is the foundational task for the project.
- Pain points: Without abstraction, platform-specific code would scatter throughout device drivers and subsystems, making porting difficult and violating memory safety guarantees.
- Constraints:
  - `no_std` environment (no standard library, heap allocation limited)
  - Zero-cost abstraction requirement (< 1% overhead)
  - Real-time constraints (50Hz minimum control loop)
  - Memory budget limitations
- Related ADRs: [ADR-oa2qa-platform-abstraction](../../../adr/ADR-oa2qa-platform-abstraction.md) - Selected three-tier design over two-tier embedded-hal approach

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────┐
│  Tier 3: Subsystems (Platform-Independent)  │
│  (AHRS, Navigation, Control, MAVLink)       │
└───────────────┬─────────────────────────────┘
                │ Uses Device Traits
┌───────────────▼─────────────────────────────┐
│  Tier 2: Device Drivers                     │
│  (GPS, IMU, Motor, Servo - Generic over     │
│   Platform Traits)                          │
└───────────────┬─────────────────────────────┘
                │ Uses Platform Traits
┌───────────────▼─────────────────────────────┐
│  Tier 1: Platform Abstraction Traits        │
│  (UART, I2C, SPI, PWM, GPIO)                │
└───────────────┬─────────────────────────────┘
                │ Implemented by
┌───────────────▼─────────────────────────────┐
│  Platform Implementations                   │
│  src/platform/rp2040/                       │
│  src/platform/rp2350/                       │
│  src/platform/mock/ (testing)               │
│  src/platform/esp32/ (future)               │
└─────────────────────────────────────────────┘
```

### Components

**Tier 1 - Platform Traits** (`src/platform/traits/`):

- `UartInterface` - Asynchronous serial communication trait
- `I2cInterface` - I2C bus communication trait
- `SpiInterface` - SPI bus communication trait
- `PwmInterface` - PWM output control trait
- `GpioInterface` - Digital GPIO control trait
- `TimerInterface` - System timer and delay trait
- `Platform` - Root trait aggregating all peripheral types

**Tier 1 - Platform Implementations**:

- `src/platform/rp2040/` - Raspberry Pi Pico W (RP2040) implementation using `rp2040-hal`
- `src/platform/rp2350/` - Raspberry Pi Pico 2 W (RP2350) implementation using `rp235x-hal`
- `src/platform/mock/` - Mock implementation for unit testing without hardware

**Tier 2 - Device Traits** (`src/devices/traits/`):

- `GpsSensor` - GPS position and fix trait
- `ImuSensor` - IMU accelerometer/gyroscope trait
- `MagnetometerSensor` - Magnetometer/compass trait
- `MotorController` - Motor speed control trait
- `ServoController` - Servo position control trait

**Tier 2 - Device Drivers** (`src/devices/`):

- `devices/gps/ublox.rs` - u-blox GPS driver (generic over `UartInterface`)
- `devices/imu/mpu6050.rs` - MPU6050 IMU driver (generic over `I2cInterface`)
- `devices/motor/brushed.rs` - Brushed motor driver (generic over `PwmInterface`)
- `devices/servo/standard.rs` - Standard servo driver (generic over `PwmInterface`)

**Tier 3 - Subsystems** (`src/subsystems/`):

- Platform-independent navigation, AHRS, and control logic (generic over device traits)

### Data Flow

1. **Initialization**: Main binary selects platform via feature flag (`--features pico2_w`)
2. **Platform Setup**: Platform implementation initializes peripherals and returns trait objects
3. **Device Initialization**: Device drivers are constructed with platform trait instances
4. **Runtime**: Subsystems call device trait methods, which call platform trait methods, which call HAL-specific code

### Storage Layout and Paths (if applicable)

N/A - This is an embedded system with no filesystem.

### CLI/API Design (if applicable)

N/A - This is an embedded library, not a CLI application.

### Data Models and Types

**Platform Trait:**

```rust
pub trait Platform: Sized {
    type Uart: UartInterface;
    type I2c: I2cInterface;
    type Spi: SpiInterface;
    type Pwm: PwmInterface;
    type Gpio: GpioInterface;
    type Timer: TimerInterface;

    fn init() -> Self;
    fn system_clock_hz(&self) -> u32;
    fn create_uart(&mut self, config: UartConfig) -> Result<Self::Uart>;
    fn create_i2c(&mut self, config: I2cConfig) -> Result<Self::I2c>;
    fn create_spi(&mut self, config: SpiConfig) -> Result<Self::Spi>;
    fn create_pwm(&mut self, config: PwmConfig) -> Result<Self::Pwm>;
    fn create_gpio(&mut self, pin: u8) -> Result<Self::Gpio>;
}
```

**UART Interface:**

```rust
pub trait UartInterface {
    async fn write(&mut self, data: &[u8]) -> Result<usize>;
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize>;
    fn set_baud_rate(&mut self, baud: u32) -> Result<()>;
}
```

**I2C Interface:**

```rust
pub trait I2cInterface {
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<()>;
    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<()>;
    async fn write_read(&mut self, addr: u8, write_data: &[u8], read_buffer: &mut [u8]) -> Result<()>;
}
```

**Device Trait Example (GPS):**

```rust
pub trait GpsSensor {
    async fn read_position(&mut self) -> Result<GpsPosition>;
    fn has_fix(&self) -> bool;
    fn satellite_count(&self) -> u8;
}

pub struct GpsPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f32,
    pub speed: f32,
}
```

### Error Handling

- Use `PlatformError` enum with variants for each peripheral type (Uart, I2c, Spi, Pwm, Gpio, Timer)
- Each platform implementation maps HAL-specific errors to `PlatformError`
- Device drivers propagate platform errors or wrap them in device-specific error types
- No `unsafe` code outside `src/platform/` per NFR-pj11s
- All `unsafe` blocks in platform implementations must include SAFETY comments

### Security Considerations

- Hardware register access is isolated to platform implementations
- All `unsafe` code is confined to `src/platform/` and audited
- No dynamic dispatch (no vtables that could be exploited)

### Performance Considerations

- **Zero-cost Abstraction**: Use Rust generics for compile-time dispatch, not `dyn Trait`
- **Hot Paths**: UART read/write, I2C transactions in control loop must have < 1% overhead
- **No Heap Allocation**: Platform traits use stack buffers only
- **Async Runtime**: Use Embassy framework for async/await support in `no_std`
- **Verification**: Measure cycle counts with RP2350 cycle counter in control loop

### Platform Considerations

#### RP2040 (Raspberry Pi Pico W)

- Cortex-M0+ (no FPU, single-cycle multiply, 133MHz)
- 264KB RAM, 2MB Flash
- 2x UART, 2x I2C, 2x SPI, 16x PWM channels
- HAL crate: `rp2040-hal` v0.9
- DMA support for UART/I2C/SPI

#### RP2350 (Raspberry Pi Pico 2 W)

- Cortex-M33 (FPU, DSP extensions, 150MHz)
- 520KB RAM, 4MB Flash
- 2x UART, 2x I2C, 2x SPI, 24x PWM channels
- HAL crate: `rp235x-hal` v0.3
- Enhanced DMA capabilities

#### Async Runtime (Embassy)

- Embassy provides async/await for embedded systems
- Each platform implementation wraps HAL peripherals with Embassy async adapters
- Executor runs in main task

## Alternatives Considered

1. **Two-Tier (embedded-hal + Device Traits)**
   - Pros: Reuse standard embedded-hal traits, less custom code
   - Cons: embedded-hal is byte-oriented (less ergonomic), doesn't cover platform initialization, still needs custom traits for devices
2. **Direct HAL Usage (No Abstraction)**
   - Pros: Simplest, zero overhead, fastest development initially
   - Cons: Platform-specific code scattered everywhere, difficult to port, violates NFR-nmmu0
3. **Runtime Polymorphism (dyn Trait)**
   - Pros: Simpler type signatures, no generics
   - Cons: Runtime overhead (vtables), larger binary size, violates zero-cost requirement

Decision Rationale

- Three-tier with generics chosen for portability, testability, and zero-cost abstraction
- Upfront design effort is acceptable for long-term maintainability
- Mock implementations enable hardware-free testing per NFR-nmmu0

## Migration and Compatibility

N/A - This is the first implementation (greenfield project).

## Testing Strategy

### Unit Tests

- Mock platform implementations in `src/platform/mock/`
- Device drivers tested against mock UART/I2C/SPI
- Verify parsing logic, state machines, error handling without hardware

### Integration Tests

- Tests run on actual hardware (Pico 2 W) using `probe-rs` for flashing
- Validate UART communication with loopback test
- Validate I2C with MPU6050 IMU on test bench
- Validate PWM with oscilloscope measurements

### External API Parsing (if applicable)

N/A - This is a platform abstraction, not an external API client.

### Performance & Benchmarks (if applicable)

- Measure cycle count overhead in control loop (target < 1%)
- Compare direct HAL calls vs trait-based calls in disassembly
- Verify zero-cost abstraction via `cargo asm` inspection

## Documentation Impact

- Update `docs/architecture.md` with platform abstraction details
- Document platform trait API in code comments
- Add examples in `docs/examples/` for device driver implementation

## External References

- Embassy Async Framework: <https://embassy.dev/>
- embedded-hal: <https://docs.rs/embedded-hal/>
- ArduPilot AP_HAL: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL>
- RP2040 Datasheet: <https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf>
- RP2350 Datasheet: <https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf>

## Open Questions

- [ ] Should we use Embassy's UART/I2C drivers directly or wrap HAL drivers? → Next step: Prototype both approaches and measure overhead
- [ ] How to handle platform-specific capabilities (RP2350 has more PWM channels)? → Method: Use associated constants in Platform trait
- [ ] Should device drivers take ownership of platform peripherals or borrow them? → Decision: Take ownership for simplicity (no lifetime complexity)

## Appendix

### Diagrams

```text
Example Usage:

┌─────────────────────────┐
│   Application Entry     │
│  (main.rs / bin/)       │
└───────────┬─────────────┘
            │
            │ 1. Select platform via feature flag
            ▼
┌─────────────────────────┐
│  Platform::init()       │
│  (rp2350::Rp2350)       │
└───────────┬─────────────┘
            │
            │ 2. Create peripherals
            ▼
┌─────────────────────────┐
│  platform.create_uart() │
│  Returns: Rp2350Uart    │
└───────────┬─────────────┘
            │
            │ 3. Pass to device driver
            ▼
┌─────────────────────────┐
│  UbloxGps::new(uart)    │
│  Generic<U: UartInterface>
└───────────┬─────────────┘
            │
            │ 4. Device trait
            ▼
┌─────────────────────────┐
│  gps.read_position()    │
│  Returns: GpsPosition   │
└─────────────────────────┘
```

### Examples

```rust
// Platform-independent device driver
pub struct UbloxGps<U: UartInterface> {
    uart: U,
    parser: UbxParser,
}

impl<U: UartInterface> GpsSensor for UbloxGps<U> {
    async fn read_position(&mut self) -> Result<GpsPosition> {
        let mut buffer = [0u8; 256];
        let len = self.uart.read(&mut buffer).await?;
        self.parser.parse(&buffer[..len])
    }
}

// Usage in subsystem (platform-independent)
pub struct WaypointNavigator<G: GpsSensor> {
    gps: G,
}

impl<G: GpsSensor> WaypointNavigator<G> {
    pub async fn update(&mut self) -> Result<NavigationOutput> {
        let position = self.gps.read_position().await?;
        // Navigation logic...
    }
}
```

### Glossary

- HAL: Hardware Abstraction Layer - Platform-specific crate for peripheral access
- Platform: In this design, a specific microcontroller (RP2040, RP2350, ESP32)
- Device: External sensor/actuator connected via peripheral (GPS, IMU, motor)
- Subsystem: High-level logic (navigation, AHRS, control) using multiple devices
- Zero-cost Abstraction: Rust generics that compile to identical code as direct calls

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../../templates/README.md#design-template-designmd) in the templates README.
