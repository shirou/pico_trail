# T-qwvco BMI088 IMU Driver Implementation

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-yhnjd-imu-sensor-selection](../../analysis/AN-yhnjd-imu-sensor-selection.md)
- Related Requirements:
  - [FR-eyuh8-ahrs-attitude-estimation](../../requirements/FR-eyuh8-ahrs-attitude-estimation.md)
  - [NFR-3wlo1-imu-sampling-rate](../../requirements/NFR-3wlo1-imu-sampling-rate.md)
- Related ADRs:
  - [ADR-wjcrn-imu-driver-architecture](../../adr/ADR-wjcrn-imu-driver-architecture.md)
  - [ADR-6twis-ahrs-algorithm-selection](../../adr/ADR-6twis-ahrs-algorithm-selection.md)
  - [ADR-oa2qa-platform-abstraction](../../adr/ADR-oa2qa-platform-abstraction.md)
- Associated Design Document:
  - [design.md](./design.md)
- Associated Plan Document:
  - [plan.md](./plan.md)

## Summary

Implement BMI088-based IMU driver using SPI communication, interrupt-driven sampling, and trait-based abstraction layer. Provides gyroscope and accelerometer data to AHRS subsystem at 400Hz with low jitter (<1ms) for accurate attitude estimation.

## Scope

- In scope:
  - BMI088 SPI driver implementation with separate gyro and accel interfaces
  - Interrupt-driven data-ready sampling at 400Hz
  - `ImuSensor` trait abstraction for AHRS integration
  - Platform-abstracted SPI communication via existing `SpiInterface` trait
  - Error handling with retry logic and exponential backoff
  - Self-test and sensor verification functionality
  - Calibration data integration with parameter system
  - Mock IMU implementation for unit testing
  - Support for both RP2040 (Pico W) and RP2350 (Pico 2 W)
- Out of scope:
  - Magnetometer integration (separate task)
  - IMU FIFO buffering (deferred to Phase 2)
  - DMA optimization (initial implementation uses interrupt-driven SPI)
  - Temperature compensation (evaluate in Phase 2)
  - Sensor redundancy (not required for rover/boat)

## Success Metrics

- **Sampling Rate**: Sustained 400Hz ± 5Hz over 10-second window
- **Jitter**: Standard deviation < 1ms measured via timestamps
- **Latency**: Sensor read completes in < 0.5ms (SPI transaction + conversion)
- **CPU Overhead**: < 10% CPU time on RP2040 @ 133MHz for IMU task
- **Reliability**: < 1 read error per 1000 samples under normal operation
- **AHRS Integration**: AHRS achieves ±2° roll/pitch accuracy with IMU data
- **Platform Support**: Works on both RP2040 and RP2350 without modification
- **Testing**: Mock IMU enables AHRS unit testing without hardware

## Implementation Phases

1. **Phase 1**: Driver Foundation (register definitions, SPI communication, chip ID verification)
2. **Phase 2**: Sensor Reading (gyro/accel data reading, interrupt handling, error retry logic)
3. **Phase 3**: AHRS Integration (trait implementation, calibration, testing, documentation)

## Key Design Decisions

- **BMI088 Sensor**: High-performance 6-axis IMU with 1600Hz ODR, low jitter via SPI
- **SPI Communication**: 1-8 MHz SPI for deterministic timing and low latency (<0.3ms)
- **Interrupt-Driven Sampling**: Hardware data-ready interrupts ensure precise 400Hz timing
- **Trait-Based Abstraction**: `ImuSensor` trait decouples AHRS from specific sensor implementation
- **Platform Abstraction**: Uses existing `SpiInterface` trait for RP2040/RP2350 portability
- **Error Handling**: Max 3 retries with exponential backoff, sensor reset on persistent failure
- **Mock Implementation**: `MockImu` enables AHRS testing without hardware
- **Calibration Integration**: Gyro bias and accel calibration loaded from parameter system

## Progress Tracking

See [Implementation Plan](plan.md) for detailed task checklists and phase status.

## Notes

- Follows ADR-wjcrn: BMI088 selected for technical superiority over MPU6050/MPU9250
- External magnetometer (QMC5883L/HMC5883L) to be added in separate task
- BMI088 has separate gyro and accel datasheets, requiring dual chip select pins
- ArduPilot uses BMI088 extensively, providing proven reference implementation
- Platform-specific optimizations (DMA, higher SPI clock) deferred to Phase 2
- Self-test and sensor verification critical for safety-critical applications
- Gyro/accel ranges configurable via `Bmi088Config` (±2000°/s gyro, ±24g accel recommended)
- Compatible with both RP2040 (no FPU, software float) and RP2350 (hardware FPU)
