# AN-aruub IMU Driver HAL Abstraction

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Requirements:
  - [FR-svawa-imu-hal-abstraction](../requirements/FR-svawa-imu-hal-abstraction.md)
  - [NFR-96m2s-imu-driver-location](../requirements/NFR-96m2s-imu-driver-location.md)
- Related ADRs:
  - [ADR-bikyc-imu-embedded-hal-async](../adr/ADR-bikyc-imu-embedded-hal-async.md)
- Related Tasks:
  - [T-cj02a-imu-hal-refactor](../tasks/T-cj02a-imu-hal-refactor/README.md)

## Executive Summary

The MPU9250 and ICM20948 IMU drivers are currently located in `src/platform/rp2350/devices/imu/` and directly depend on `embassy_rp::i2c::I2c`. This couples platform-agnostic sensor logic to RP2350-specific I2C implementation. Refactoring to use `embedded_hal_async::i2c::I2c` trait will make drivers truly portable across any platform supporting embedded-hal-async.

## Problem Space

### Current State

IMU drivers are in `src/platform/rp2350/devices/imu/`:

```rust
// src/platform/rp2350/devices/imu/mpu9250/driver.rs
use embassy_rp::i2c::{Async, I2c};

pub struct Mpu9250Driver<'d, T: embassy_rp::i2c::Instance> {
    i2c: I2c<'d, T, Async>,  // RP2350-specific type
    ...
}
```

Problems:

1. **Incorrect placement**: Drivers are under `platform/rp2350/` but MPU9250/ICM20948 are generic I2C devices
2. **Platform coupling**: Direct dependency on `embassy_rp::i2c::I2c` prevents use with ESP32, STM32, etc.
3. **Misleading architecture**: Implies these are RP2350-specific when they're not

### Desired State

Drivers should:

1. Live in `src/devices/imu/` (platform-agnostic location)
2. Accept any I2C implementation via `embedded_hal_async::i2c::I2c` trait
3. Be usable with any microcontroller that provides embedded-hal-async I2C

```rust
// Desired design
use embedded_hal_async::i2c::I2c;

pub struct Mpu9250Driver<I2C: I2c> {
    i2c: I2C,  // Generic - works with any I2C
    ...
}
```

### Gap Analysis

| Aspect    | Current                        | Desired                        | Change Required   |
| --------- | ------------------------------ | ------------------------------ | ----------------- |
| Location  | `platform/rp2350/devices/imu/` | `devices/imu/`                 | Move files        |
| I2C trait | `embassy_rp::i2c::Instance`    | `embedded_hal_async::i2c::I2c` | Refactor generics |
| Time      | `embassy_time::Instant`        | `core::traits::time::Instant`  | Use abstraction   |

## Research & Discovery

### Technical Investigation

**embedded_hal_async I2C trait**:

```rust
// From embedded-hal-async crate
pub trait I2c: ErrorType {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error>;
    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error>;
    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error>;
}
```

**embassy_rp compatibility**:

- `embassy_rp::i2c::I2c` implements `embedded_hal_async::i2c::I2c`
- Switching to trait-based design maintains full compatibility

**Time abstraction**:

- `embassy_time::Instant` used for timestamps
- Already have `crate::core::traits::time` abstraction available

## Discovered Requirements

### Functional Requirements (Potential)

- [x] **FR-DRAFT-1**: IMU drivers shall accept any I2C implementation conforming to `embedded_hal_async::i2c::I2c`
  - Rationale: Enable cross-platform use
  - Acceptance Criteria: Drivers compile and work with both embassy_rp and mock I2C

### Non-Functional Requirements (Potential)

- [x] **NFR-DRAFT-1**: Platform-agnostic drivers shall reside outside `platform/` directory
  - Category: Architecture
  - Rationale: Clear separation of platform-specific and generic code
  - Target: Zero platform-specific imports in IMU driver modules

## Design Considerations

### Technical Constraints

- Must maintain `ImuSensor` trait implementation
- Error type must be compatible with driver error handling
- No performance regression from trait abstraction

### Potential Approaches

1. **Generic I2C parameter**
   - Pros: Clean, standard approach, full flexibility
   - Cons: Slightly more complex type signatures
   - Effort: Medium

2. **Keep as-is (rejected)**
   - Pros: No work required
   - Cons: Architecture violation, limits future platform support
   - Effort: None

### Architecture Impact

- Drivers move from `platform/rp2350/devices/imu/` to `devices/imu/`
- Re-export from platform module removed
- `pico2_w` feature gate on IMU re-export becomes unnecessary

## Risk Assessment

| Risk                    | Probability | Impact | Mitigation Strategy                      |
| ----------------------- | ----------- | ------ | ---------------------------------------- |
| API breakage            | Low         | Medium | Maintain `ImuSensor` trait interface     |
| Error handling mismatch | Low         | Low    | Use `embedded_hal_async::i2c::ErrorType` |

## Recommendations

### Immediate Actions

1. Create FR/NFR documents
2. Create ADR for embedded-hal-async decision
3. Create task package for refactoring

### Next Steps

1. [x] Create formal requirements: FR-svawa, NFR-96m2s
2. [x] Draft ADR for: embedded-hal-async I2C abstraction
3. [ ] Create task for: IMU driver refactoring
4. [ ] Implementation

### Out of Scope

- Adding new IMU chip support (future task)
- SPI interface support (different analysis needed)
