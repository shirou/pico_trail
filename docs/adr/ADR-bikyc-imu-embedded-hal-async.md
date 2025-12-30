# ADR-bikyc IMU Drivers Use embedded-hal-async I2C Trait

## Metadata

- Type: ADR
- Status: Draft

## Links

- Impacted Requirements:
  - [FR-svawa-imu-hal-abstraction](../requirements/FR-svawa-imu-hal-abstraction.md)
  - [NFR-96m2s-imu-driver-location](../requirements/NFR-96m2s-imu-driver-location.md)
- Source Analysis:
  - [AN-aruub-imu-driver-hal-abstraction](../analysis/AN-aruub-imu-driver-hal-abstraction.md)
- Related Tasks:
  - [T-cj02a-imu-hal-refactor](../tasks/T-cj02a-imu-hal-refactor/README.md)

## Context

IMU drivers (MPU9250, ICM20948) currently depend directly on `embassy_rp::i2c::I2c`:

```rust
use embassy_rp::i2c::{Async, I2c};

pub struct Mpu9250Driver<'d, T: embassy_rp::i2c::Instance> {
    i2c: I2c<'d, T, Async>,
}
```

This creates several problems:

1. **Platform coupling**: Drivers cannot be used with ESP32, STM32, or other platforms
2. **Incorrect placement**: Drivers are in `platform/rp2350/devices/` despite being generic I2C devices
3. **Architectural violation**: Sensor logic (platform-agnostic) mixed with platform-specific I2C types

The Rust embedded ecosystem provides `embedded-hal-async` as the standard abstraction layer for async peripheral access. All major embassy HALs implement these traits.

## Decision

We will refactor IMU drivers to use `embedded_hal_async::i2c::I2c` trait instead of concrete `embassy_rp` I2C types.

### Decision Drivers

- Platform portability across microcontroller families
- Clear separation between device logic and platform HAL
- Alignment with Rust embedded ecosystem standards
- Enable future multi-platform support (ESP32, STM32)

### Considered Options

- **Option A**: Use `embedded_hal_async::i2c::I2c` trait (chosen)
- **Option B**: Keep `embassy_rp` direct dependency
- **Option C**: Create custom I2C abstraction trait

### Option Analysis

- **Option A** (embedded_hal_async) — Pros: Industry standard, all HALs implement it, zero runtime cost | Cons: Slightly more complex generics
- **Option B** (keep as-is) — Pros: No work required | Cons: Platform lock-in, architectural violation
- **Option C** (custom trait) — Pros: Full control | Cons: Reinventing the wheel, maintenance burden, ecosystem incompatible

## Rationale

`embedded_hal_async::i2c::I2c` is the de-facto standard for async I2C in the Rust embedded ecosystem:

- `embassy_rp::i2c::I2c` implements `embedded_hal_async::i2c::I2c`
- `embassy_stm32::i2c::I2c` implements `embedded_hal_async::i2c::I2c`
- `esp_hal::i2c::I2c` implements `embedded_hal_async::i2c::I2c`

Using the trait:

- **No runtime cost**: Trait methods are monomorphized and inlined
- **Maximum flexibility**: Works with any conforming I2C implementation
- **Testability**: Can use mock I2C for host tests

## Consequences

### Positive

- IMU drivers become truly platform-agnostic
- Drivers can move to `src/devices/imu/` (correct location)
- Future ESP32/STM32 support requires no driver changes
- Cleaner module boundaries
- Easier unit testing with mock I2C

### Negative

- Type signatures become more complex (generic I2C parameter)
- Error type handling requires trait bounds

### Neutral

- No runtime behavior change
- API surface (`ImuSensor` trait) unchanged

## Implementation Notes

**New driver signature**:

```rust
use embedded_hal_async::i2c::I2c;

pub struct Mpu9250Driver<I2C>
where
    I2C: I2c,
{
    i2c: I2C,
    config: Mpu9250Config,
    // ...
}

impl<I2C> Mpu9250Driver<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C, config: Mpu9250Config) -> Self { ... }
}
```

**Error handling**:

```rust
use embedded_hal_async::i2c::Error as I2cError;

pub enum ImuError<E> {
    I2c(E),
    InvalidWhoAmI { expected: u8, got: u8 },
    NotInitialized,
}
```

**Instantiation at platform level**:

```rust
// In example or platform-specific code
use embassy_rp::i2c::I2c;

let i2c = I2c::new_async(i2c0, scl, sda, Irqs, config);
let imu = Mpu9250Driver::new(i2c, Mpu9250Config::default());
```

## External References

- [embedded-hal-async I2C trait](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/i2c/trait.I2c.html)
- [Embassy embedded-hal-async support](https://embassy.dev/book/dev/hal.html)
