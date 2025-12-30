# T-cj02a IMU HAL Refactor Design

## Metadata

- Type: Design
- Status: Complete

## Links

- Associated Plan Document:
  - [T-cj02a-imu-hal-refactor-plan](plan.md)

## Overview

Refactor MPU9250 and ICM20948 IMU drivers to use `embedded_hal_async::i2c::I2c` trait instead of `embassy_rp::i2c::I2c` concrete type. This decouples device logic from platform HAL and allows drivers to work with any embedded-hal-async I2C implementation.

## Success Metrics

- [x] Zero `embassy_rp::*` imports in `src/devices/imu/`
- [x] Zero `#[cfg(feature = "pico2_w")]` in IMU modules
- [x] All existing tests pass
- [x] Embedded build succeeds with `./scripts/build-rp2350.sh`

## Background and Current State

- Context: IMU drivers provide sensor fusion data for AHRS subsystem
- Current behavior:
  - Drivers in `src/platform/rp2350/devices/imu/`
  - Use `embassy_rp::i2c::I2c<'d, T, Async>` directly
  - Re-exported via `src/devices/imu/mod.rs` with `pico2_w` gate
- Pain points:
  - Platform lock-in despite generic I2C devices
  - Misleading directory structure
  - Cannot test with mock I2C on host
- Related ADRs:
  - [ADR-bikyc-imu-embedded-hal-async](../../adr/ADR-bikyc-imu-embedded-hal-async.md)

## Proposed Design

### High-Level Architecture

```text
Before:
  src/devices/imu/mod.rs
    └── #[cfg(pico2_w)] re-export from platform
  src/platform/rp2350/devices/imu/
    ├── mpu9250/driver.rs  (uses embassy_rp::i2c)
    └── icm20948/driver.rs (uses embassy_rp::i2c)

After:
  src/devices/imu/
    ├── mod.rs             (direct exports, no cfg)
    ├── mpu9250/driver.rs  (uses embedded_hal_async::i2c::I2c)
    └── icm20948/driver.rs (uses embedded_hal_async::i2c::I2c)
  src/platform/rp2350/devices/imu/
    └── (removed)
```

### Components

**Driver struct changes**:

```rust
// Before
pub struct Mpu9250Driver<'d, T: embassy_rp::i2c::Instance> {
    i2c: embassy_rp::i2c::I2c<'d, T, Async>,
}

// After
pub struct Mpu9250Driver<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    i2c: I2C,
}
```

**Time abstraction**:

```rust
// Before
use embassy_time::Instant;
let timestamp = Instant::now().as_micros();

// After
// Use core::traits or make timestamp a parameter
fn timestamp_us() -> u64 {
    #[cfg(feature = "embassy")]
    { embassy_time::Instant::now().as_micros() }
    #[cfg(not(feature = "embassy"))]
    { 0 } // Host test stub
}
```

### Data Models and Types

**Error type**:

```rust
/// IMU driver errors
pub enum ImuError<E> {
    /// I2C communication error
    I2c(E),
    /// Invalid WHO_AM_I response
    InvalidWhoAmI { expected: u8, got: u8 },
    /// Driver not initialized
    NotInitialized,
    /// Sensor not healthy
    Unhealthy,
}
```

### Error Handling

- Generic error parameter `E` from I2C trait
- Map to `ImuError<E>` for driver-level errors
- Maintain compatibility with `ImuSensor` trait

## Alternatives Considered

1. **Keep as-is**
   - Pros: No work
   - Cons: Architecture violation, platform lock-in

2. **Custom I2C trait**
   - Pros: Full control
   - Cons: Reinventing embedded-hal, ecosystem incompatible

Decision: Use `embedded_hal_async::i2c::I2c` - industry standard, zero cost

## Testing Strategy

### Unit Tests

- Existing tests in `icm20948/driver.rs` remain (temperature conversion, scale factors)
- Add mock I2C tests for read/write operations

### Integration Tests

- Embedded build verification via `./scripts/build-rp2350.sh`
- Host tests via `cargo test --lib`

## Documentation Impact

- Update `docs/architecture.md` if needed
- No user-facing documentation changes

## Open Questions

- [x] Use `embedded_hal_async::i2c::I2c` trait - confirmed in ADR
- [ ] Time abstraction approach - use `embassy` feature gate for `timestamp_us()`
