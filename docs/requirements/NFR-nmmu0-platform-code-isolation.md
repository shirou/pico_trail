# NFR-nmmu0 Platform Code Isolation

## Metadata

- Type: Non-Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements: N/A
- Dependent Requirements:
  - [FR-jpmdj-trait-based-async-abstraction](../requirements/FR-jpmdj-trait-based-async-abstraction.md)
  - [NFR-wl974-feature-gate-reduction](../requirements/NFR-wl974-feature-gate-reduction.md)
- Related Tasks:
  - [T-d9rim-trait-based-async-abstraction](../tasks/T-d9rim-trait-based-async-abstraction/README.md)

## Requirement Statement

Platform-specific code shall be isolated to the `src/platform/` directory with zero hardware-specific code outside this boundary, enabling easy porting to new platforms and clear separation of concerns.

## Rationale

Isolating platform-specific code provides:

- **Portability**: Adding new platforms requires only implementing platform traits, no changes to application/subsystem layers
- **Maintainability**: Clear boundary makes code organization intuitive
- **Auditability**: Platform-specific code (which may use `unsafe`) is confined to a small surface area
- **Testability**: Application logic can be tested without hardware (mock platform implementation)

ArduPilot's AP_HAL demonstrates this pattern successfully across 20+ platforms.

## User Story (if applicable)

The system shall isolate all platform-specific code to `src/platform/` to ensure that porting to new hardware (ESP32, STM32) requires only implementing platform traits without modifying higher layers.

## Acceptance Criteria

- [ ] Zero direct imports of `rp2040-hal`, `rp235x-hal`, or other platform HAL crates outside `src/platform/`
- [ ] All hardware access (UART, I2C, SPI, PWM, GPIO, Timer, Flash) goes through platform traits
- [ ] Device drivers (`src/devices/`) use only platform trait interfaces, not HAL types directly
- [ ] Subsystems (`src/subsystems/`) have no platform-specific code or `#[cfg(feature = "...")]` blocks
- [ ] Application layer (`src/vehicle/`, `src/communication/`, `src/core/`) is 100% platform-independent
- [ ] Automated check in CI: `grep -r "rp2040_hal\|rp235x_hal" src/ | grep -v "src/platform"` returns no matches
- [ ] New platform can be added by implementing traits in `src/platform/new_platform/` only

## Technical Details (if applicable)

### Non-Functional Requirement Details

**Directory Structure:**

```
src/
├── platform/               # Platform-specific code (ONLY place HAL imports allowed)
│   ├── traits/            # Platform abstraction traits
│   │   ├── uart.rs        # UART interface trait
│   │   ├── i2c.rs         # I2C interface trait
│   │   ├── spi.rs         # SPI interface trait
│   │   ├── pwm.rs         # PWM interface trait
│   │   ├── gpio.rs        # GPIO interface trait
│   │   ├── timer.rs       # Timer interface trait
│   │   └── flash.rs       # Flash storage interface trait
│   ├── rp2040/            # Pico W implementation
│   │   ├── uart.rs        # UART implementation using rp2040-hal
│   │   ├── i2c.rs         # I2C implementation
│   │   └── ...
│   ├── rp2350/            # Pico 2 W implementation
│   │   ├── uart.rs        # UART implementation using rp235x-hal
│   │   ├── i2c.rs         # I2C implementation
│   │   └── ...
│   └── mock/              # Mock implementation for testing
│       ├── uart.rs        # Mock UART for unit tests
│       └── ...
├── devices/               # Device drivers (platform-independent)
│   ├── gps/               # GPS drivers (use UART trait, not HAL)
│   ├── imu/               # IMU drivers (use I2C/SPI trait)
│   └── ...
├── subsystems/            # Subsystems (platform-independent)
│   ├── ahrs/              # AHRS implementation
│   ├── navigation/        # Navigation algorithms
│   └── ...
├── vehicle/               # Application logic (platform-independent)
├── communication/         # MAVLink, telemetry (platform-independent)
└── core/                  # Core systems (platform-independent)
```

**Platform Trait Example:**

```rust
// src/platform/traits/uart.rs - Platform-independent trait
pub trait UartInterface {
    fn write(&mut self, data: &[u8]) -> Result<usize>;
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize>;
    fn set_baud_rate(&mut self, baud: u32) -> Result<()>;
}

// src/platform/rp2350/uart.rs - Platform-specific implementation
use rp235x_hal as hal; // HAL import ONLY allowed in platform layer

pub struct Rp2350Uart {
    uart: hal::uart::UartPeripheral</* ... */>,
}

impl UartInterface for Rp2350Uart {
    fn write(&mut self, data: &[u8]) -> Result<usize> {
        // Use rp235x-hal UART implementation
        self.uart.write(data).map_err(|e| Error::Uart(e))
    }
    // ...
}
```

**Device Driver (Platform-Independent):**

```rust
// src/devices/gps/ublox.rs - Uses trait, NOT HAL
use crate::platform::traits::UartInterface;

pub struct UbloxGps<U: UartInterface> {
    uart: U, // Generic over any UART implementation
}

impl<U: UartInterface> UbloxGps<U> {
    pub fn read_position(&mut self) -> Result<GpsPosition> {
        // Read from UART using trait interface
        let mut buffer = [0u8; 256];
        let len = self.uart.read(&mut buffer)?;
        // Parse UBX protocol...
    }
}
```

**Enforcement:**

CI check:

```bash
# Fail build if HAL imports found outside platform/
if grep -rn "use.*rp2040_hal\|use.*rp235x_hal\|use.*embassy_rp" src/ | \
   grep -v "src/platform/"; then
    echo "ERROR: HAL imports found outside src/platform/"
    exit 1
fi
```

## Platform Considerations

### Pico W (RP2040)

HAL imports (`use rp2040_hal`) allowed only in `src/platform/rp2040/`. All other code uses platform traits.

### Pico 2 W (RP2350)

HAL imports (`use rp235x_hal`) allowed only in `src/platform/rp2350/`. All other code uses platform traits.

### Cross-Platform

All code outside `src/platform/` must compile for any platform with only the platform implementation swapped.

## Risks & Mitigation

| Risk                                           | Impact | Likelihood | Mitigation                                                 | Validation                                    |
| ---------------------------------------------- | ------ | ---------- | ---------------------------------------------------------- | --------------------------------------------- |
| Platform abstraction too restrictive           | Medium | Medium     | Design traits based on common embedded patterns            | Prototype ESP32 port early to validate traits |
| Performance overhead from trait dispatch       | Low    | Low        | Use static dispatch (generics), not dynamic (dyn Trait)    | Benchmark with and without trait abstraction  |
| Developer bypasses abstraction for convenience | Medium | Medium     | Enforce via CI, code review, reject PRs violating boundary | Automated CI check for HAL imports            |
| Platform traits become too complex             | Medium | Low        | Keep traits focused (single responsibility), iterate       | Review trait complexity during ADR process    |

## Implementation Notes

**Platform Trait Design Principles:**

- Keep traits **simple and focused** (single responsibility)
- Use **zero-cost abstractions** (static dispatch via generics)
- Provide **common interface** across platforms (no platform-specific methods in traits)
- **Document invariants** clearly (e.g., "UART must be initialized before use")

**Testing with Mock Platform:**

```rust
// src/platform/mock/uart.rs - For unit tests
pub struct MockUart {
    tx_buffer: Vec<u8>,
    rx_buffer: Vec<u8>,
}

impl UartInterface for MockUart {
    fn write(&mut self, data: &[u8]) -> Result<usize> {
        self.tx_buffer.extend_from_slice(data);
        Ok(data.len())
    }
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize> {
        let len = buffer.len().min(self.rx_buffer.len());
        buffer[..len].copy_from_slice(&self.rx_buffer[..len]);
        self.rx_buffer.drain(..len);
        Ok(len)
    }
}

// Unit test (platform-independent)
#[test]
fn test_gps_parsing() {
    let mut mock_uart = MockUart::new();
    mock_uart.rx_buffer = ubx_test_data();
    let mut gps = UbloxGps::new(mock_uart);
    let pos = gps.read_position().unwrap();
    assert_eq!(pos.lat, 37.7749);
}
```

Related code areas:

- `src/platform/traits/` - Platform abstraction trait definitions
- `src/platform/*/` - Platform-specific implementations
- All other `src/` directories - Must use traits only, no HAL imports

## External References

- Embedded HAL Traits: <https://docs.rs/embedded-hal/>
- ArduPilot AP_HAL: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
