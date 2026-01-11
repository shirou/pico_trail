# FR-svawa IMU Driver HAL Abstraction

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Source Analysis:
  - [AN-aruub-imu-driver-hal-abstraction](../analysis/AN-aruub-imu-driver-hal-abstraction.md)
- Dependent Requirements:
  - [NFR-96m2s-imu-driver-location](../requirements/NFR-96m2s-imu-driver-location.md)
- Related ADRs:
  - [ADR-bikyc-imu-embedded-hal-async](../adr/ADR-bikyc-imu-embedded-hal-async.md)
- Related Tasks:
  - [T-cj02a-imu-hal-refactor](../tasks/T-cj02a-imu-hal-refactor/README.md)
  - [T-7khm3-ahrs-abstraction-layer](../tasks/T-7khm3-ahrs-abstraction-layer/README.md)

## Requirement Statement

IMU drivers (MPU9250, ICM20948) shall accept any I2C bus implementation conforming to the `embedded_hal_async::i2c::I2c` trait, enabling platform-agnostic operation.

## Rationale

Current drivers directly depend on `embassy_rp::i2c::I2c`, coupling platform-agnostic sensor logic to RP2350-specific implementation. This prevents:

- Reuse on other platforms (ESP32, STM32, etc.)
- Clear architectural separation between devices and platform
- Future multi-platform support

## User Story (if applicable)

As a developer, I want IMU drivers that work with any embedded-hal-async I2C implementation, so that I can use the same driver code across different microcontroller platforms.

## Acceptance Criteria

- [ ] `Mpu9250Driver<I2C>` accepts generic I2C type bounded by `embedded_hal_async::i2c::I2c`
- [ ] `Icm20948Driver<I2C>` accepts generic I2C type bounded by `embedded_hal_async::i2c::I2c`
- [ ] Drivers compile with `embassy_rp::i2c::I2c` as the I2C implementation
- [ ] Drivers compile with mock I2C implementation for host tests
- [ ] `ImuSensor` trait implementation remains unchanged
- [ ] All existing unit tests pass
- [ ] Embedded build (`./scripts/build-rp2350.sh`) succeeds

## Technical Details (if applicable)

### Functional Requirement Details

**Current signature**:

```rust
pub struct Mpu9250Driver<'d, T: embassy_rp::i2c::Instance> {
    i2c: embassy_rp::i2c::I2c<'d, T, Async>,
}
```

**Required signature**:

```rust
pub struct Mpu9250Driver<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    i2c: I2C,
}
```

**Error handling**:

- Use `embedded_hal_async::i2c::Error` trait for I2C errors
- Map to driver-specific `ImuError` type

## Platform Considerations

### Cross-Platform

This requirement explicitly aims for platform independence:

- Driver code contains no platform-specific imports
- I2C implementation injected at instantiation time
- Works with any `embedded_hal_async::i2c::I2c` implementor

## Risks & Mitigation

| Risk                   | Impact | Likelihood | Mitigation                | Validation               |
| ---------------------- | ------ | ---------- | ------------------------- | ------------------------ |
| Error type mismatch    | Medium | Low        | Use generic error bounds  | Unit tests with mock I2C |
| Performance regression | Low    | Low        | Trait methods are inlined | Benchmark if needed      |

## Implementation Notes

- `embassy_rp::i2c::I2c` already implements `embedded_hal_async::i2c::I2c`
- No runtime behavior change expected, only type signature change
- Consider adding `+ 'static` bound if needed for async contexts

## External References

- [embedded-hal-async I2C trait](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/i2c/trait.I2c.html) - Official trait definition
