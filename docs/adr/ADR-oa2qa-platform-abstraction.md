# ADR-oa2qa Platform Abstraction Layer: Three-Tier Design

## Metadata

- Type: ADR
- Status: Approved

## Links

- Related Analyses:
  - [AN-kir7h-platform-abstraction](../analysis/AN-kir7h-platform-abstraction.md)
- Impacted Requirements:
  - [NFR-nmmu0-platform-code-isolation](../requirements/NFR-nmmu0-platform-code-isolation.md)
  - [NFR-pj11s-no-unsafe-rust](../requirements/NFR-pj11s-no-unsafe-rust.md)
- Supersedes ADRs: N/A
- Related Tasks:
  - [T-qwvco-bmi088-imu-driver-implementation](../tasks/T-qwvco-bmi088-imu-driver-implementation/README.md)

## Context

The autopilot must support multiple hardware platforms:

- **Primary**: Raspberry Pi Pico W (RP2040, Cortex-M0+) and Pico 2 W (RP2350, Cortex-M33)
- **Future**: ESP32, STM32F4, other embedded platforms

Each platform has different:

- **HAL crates**: `rp2040-hal`, `rp235x-hal`, `esp-hal`, `stm32f4xx-hal`
- **Peripherals**: UART, I2C, SPI, PWM, GPIO counts and capabilities
- **Features**: FPU availability, clock speeds, memory sizes

### Problem

We need a platform abstraction layer that:

- Isolates platform-specific code to `src/platform/` directory
- Enables adding new platforms by implementing traits only
- Maintains zero-cost abstractions (no runtime overhead)
- Keeps device drivers and subsystems platform-independent

### Constraints

- **Zero-cost**: No vtable/dyn trait overhead in control loops
- **no_std**: Must work without standard library
- **Compile-time Dispatch**: Use generics, not dynamic dispatch
- **Memory Budget**: Abstraction must add < 1% overhead

### Prior Art

- **ArduPilot AP_HAL**: Platform abstraction across 20+ boards, C++ polymorphism
- **embedded-hal**: Rust traits for peripherals, but low-level (byte-oriented)
- **Embassy**: Platform-specific HALs with async drivers

## Success Metrics

- **Zero HAL Imports Outside Platform**: `grep -r "rp2040_hal\|rp235x_hal" src/ | grep -v src/platform` returns empty
- **Zero Runtime Overhead**: Disassembly shows direct function calls, not indirect
- **Easy Porting**: Adding ESP32 support requires only implementing `src/platform/esp32/` (< 1000 LOC)

## Decision

**We will implement a three-tier platform abstraction:**

1. **Tier 1 (Platform Traits)**: Low-level peripheral interfaces (UART, I2C, SPI, PWM, GPIO)
2. **Tier 2 (Device Traits)**: Mid-level device abstractions (GpsSensor, ImuSensor, Motor)
3. **Tier 3 (Subsystems)**: High-level logic completely platform-independent

### Architecture

```
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
│  src/platform/esp32/ (future)               │
└─────────────────────────────────────────────┘
```

### Decision Drivers

1. **Portability**: Add new platforms without changing application code
2. **Zero-cost**: Use generics (compile-time dispatch), not dyn traits (runtime)
3. **Isolation**: All platform-specific code in `src/platform/` only
4. **Testability**: Mock implementations for unit tests (no hardware required)

### Considered Options

- **Option A: Three-Tier Abstraction (Platform + Device + Subsystem)** ⭐ Selected
- **Option B: Two-Tier (embedded-hal + Device Traits)**
- **Option C: Direct HAL Usage (No Abstraction)**

### Option Analysis

**Option A: Three-Tier Abstraction**

- **Pros**:
  - Maximum portability and testability
  - Clean separation of concerns
  - Device drivers reusable across platforms
  - Subsystems completely platform-independent
- **Cons**:
  - More upfront design effort
  - Three layers to understand
- **Estimated Overhead**: < 1% (generics compile to direct calls)

**Option B: Two-Tier (embedded-hal + Device Traits)**

- **Pros**:
  - Uses standard embedded-hal traits
  - Less custom code
- **Cons**:
  - embedded-hal is very low-level (byte-oriented)
  - Platform initialization still needed (not covered by embedded-hal)
  - Device drivers less ergonomic
- **Estimated Overhead**: < 1%

**Option C: Direct HAL Usage**

- **Pros**:
  - Simplest approach
  - Zero abstraction overhead
- **Cons**:
  - Platform-specific code scattered throughout codebase
  - Difficult to port to new platforms
  - Violates portability requirement
- **Estimated Overhead**: 0%

## Rationale

Three-tier abstraction was chosen over two-tier and direct HAL usage for:

1. **Portability**: Device drivers and subsystems are 100% platform-independent
2. **Testability**: Mock platform implementations enable unit tests without hardware
3. **Clean Architecture**: Clear boundaries between platform, device, and subsystem layers
4. **Zero-cost**: Generics compile to direct calls (no runtime overhead)

### Trade-offs Accepted

- **Upfront Effort**: More design work to define traits (vs direct HAL usage)
- **Three Layers**: More abstraction layers to understand (vs simpler two-tier)

**Decision**: We accept the upfront effort for long-term portability and maintainability.

## Consequences

### Positive

- **Portability**: Adding ESP32 support requires only `src/platform/esp32/` (< 1000 LOC)
- **Testability**: Mock implementations enable unit tests without hardware
- **Clean Code**: Zero HAL imports outside `src/platform/` enforced by CI
- **Zero-cost**: Generics compile to direct function calls (verified via disassembly)

### Negative

- **Upfront Design**: Must define traits for all peripherals before implementing
- **Learning Curve**: Developers must understand three-tier architecture

### Neutral

- **Compile-time Platform Selection**: Feature flags choose platform (`--features pico2_w`)

## Implementation Notes

### Tier 1: Platform Traits

```rust
// src/platform/traits/uart.rs
pub trait UartInterface {
    async fn write(&mut self, data: &[u8]) -> Result<usize>;
    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize>;
    fn set_baud_rate(&mut self, baud: u32) -> Result<()>;
}

// src/platform/traits/i2c.rs
pub trait I2cInterface {
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<()>;
    async fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<()>;
    async fn write_read(&mut self, addr: u8, write_data: &[u8], read_buffer: &mut [u8]) -> Result<()>;
}

// src/platform/traits/mod.rs
pub trait Platform: Sized {
    type Uart: UartInterface;
    type I2c: I2cInterface;
    type Spi: SpiInterface;
    type Pwm: PwmInterface;
    type Gpio: GpioInterface;

    fn init() -> Self;
    fn system_clock_hz(&self) -> u32;
}
```

### Tier 2: Device Traits

```rust
// src/devices/traits/gps.rs
pub trait GpsSensor {
    async fn read_position(&mut self) -> Result<GpsPosition>;
    fn has_fix(&self) -> bool;
    fn satellite_count(&self) -> u8;
}

// src/devices/gps/ublox.rs (Generic over UART)
pub struct UbloxGps<U: UartInterface> {
    uart: U,
    parser: UbxParser,
}

impl<U: UartInterface> GpsSensor for UbloxGps<U> {
    async fn read_position(&mut self) -> Result<GpsPosition> {
        // Uses UART trait, not platform-specific HAL
        let mut buffer = [0u8; 256];
        let len = self.uart.read(&mut buffer).await?;
        self.parser.parse(&buffer[..len])
    }
}
```

### Tier 3: Subsystems (Platform-Independent)

```rust
// src/subsystems/navigation/waypoint.rs
pub struct WaypointNavigator<G: GpsSensor> {
    gps: G,  // Generic over GPS sensor
    waypoints: Vec<Waypoint>,
    current_waypoint: usize,
}

impl<G: GpsSensor> WaypointNavigator<G> {
    pub async fn update(&mut self) -> Result<NavigationOutput> {
        let position = self.gps.read_position().await?;
        // Navigation logic, completely platform-independent
        Ok(self.calculate_steering(position))
    }
}
```

### Platform Selection (Compile-Time)

```toml
# Cargo.toml
[features]
default = ["pico2_w"]
pico_w = ["rp2040-hal"]
pico2_w = ["rp235x-hal"]
esp32 = ["esp-hal"]

[dependencies]
rp2040-hal = { version = "0.9", optional = true }
rp235x-hal = { version = "0.3", optional = true }
esp-hal = { version = "0.15", optional = true }
```

```rust
// src/platform/mod.rs
#[cfg(feature = "pico_w")]
pub use rp2040::*;

#[cfg(feature = "pico2_w")]
pub use rp2350::*;

#[cfg(feature = "esp32")]
pub use esp32::*;
```

### CI Enforcement

```yaml
# .github/workflows/ci.yml
- name: Check for HAL imports outside platform layer
  run: |
    if grep -rn "use.*rp2040_hal\|use.*rp235x_hal\|use.*esp_hal" src/ | \
       grep -v "src/platform/"; then
      echo "ERROR: HAL imports found outside src/platform/"
      exit 1
    fi
```

## Platform Considerations

- **Pico W**: Implement `src/platform/rp2040/` using `rp2040-hal`
- **Pico 2 W**: Implement `src/platform/rp2350/` using `rp235x-hal`
- **Future ESP32**: Implement `src/platform/esp32/` using `esp-hal`

## Open Questions

- [ ] Should we use embedded-hal traits or custom traits? → Decision: Custom traits (embedded-hal too low-level, doesn't cover platform init)
- [ ] How do we handle platform-specific features (e.g., Pico 2 W has more PWM channels)? → Method: Use associated constants in Platform trait
- [ ] Should we support runtime platform detection? → Decision: No, compile-time only (reduces binary size)

## External References

- embedded-hal: <https://docs.rs/embedded-hal/>
- ArduPilot AP_HAL: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL>

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../templates/README.md#adr-templates-adrmd-and-adr-litemd) in the templates README.
