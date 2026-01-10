# T-x8mq2 BNO086 Driver Implementation

## Metadata

- Type: Task
- Status: Complete

## Links

- Related Analyses:
  - [AN-srhcj-bno086-imu-integration](../../analysis/AN-srhcj-bno086-imu-integration.md)
- Related Requirements:
  - [FR-z1fdo-imu-sensor-trait](../../requirements/FR-z1fdo-imu-sensor-trait.md)
  - [NFR-3wlo1-imu-sampling-rate](../../requirements/NFR-3wlo1-imu-sampling-rate.md)
- Related ADRs:
  - [ADR-t5cq4-mpu9250-i2c-driver-architecture](../../adr/ADR-t5cq4-mpu9250-i2c-driver-architecture.md)
- Associated Design Document:
  - [design.md](./design.md)
- Associated Plan Document:
  - [plan.md](./plan.md)
- Supersedes Task:
  - ~~[T-0kbo4-icm20948-driver-implementation](../T-0kbo4-icm20948-driver-implementation/README.md)~~ (Cancelled - counterfeit hardware)

## Summary

Implement BNO086 9-axis IMU driver with on-chip sensor fusion, providing direct quaternion output via a new `QuaternionSensor` trait. Unlike raw IMU sensors, the BNO086 performs sensor fusion internally using its ARM Cortex-M0+ processor, outputting pre-computed quaternions at 100Hz.

## Scope

- In scope:
  - New `QuaternionSensor` trait for quaternion-native sensors
  - SHTP (Sensor Hub Transport Protocol) implementation over I2C
  - Rotation Vector report (0x05) parsing for quaternion output
  - Interrupt-driven (INT pin) async data acquisition
  - Hardware reset (RST pin) based error recovery
  - Unit tests with mock data for host testing
- Out of scope:
  - Full SHTP protocol (only Rotation Vector needed)
  - BNO086 DMP reprogramming (use default firmware)
  - Raw sensor data exposure (quaternion output is sufficient)
  - `ImuSensor` trait implementation (different paradigm)
  - External EKF (BNO086 handles fusion internally)

## Success Metrics

- **Sensor Initialization**: Product ID verification passes
- **Quaternion Rate**: 100Hz ± 5Hz over 10-second measurement window
- **I2C Latency**: SHTP packet read < 2ms (95th percentile)
- **Error Recovery**: Automatic recovery within 200ms of timeout
- **Memory Usage**: Driver state < 2KB
- **Platform Support**: Works on RP2350 (Pico 2 W)
- **Code Quality**: All tests pass, no clippy warnings, embedded build succeeds

## Implementation Phases

1. **Phase 1**: QuaternionSensor trait and SHTP foundation
2. **Phase 2**: BNO086 driver core (initialization, basic reading)
3. **Phase 3**: INT/RST handling and error recovery
4. **Phase 4**: Testing and integration

## Key Design Decisions

- **New QuaternionSensor trait**: Cleanly separates quaternion-native sensors from raw IMU sensors
- **Minimal SHTP implementation**: Only implements what's needed (Rotation Vector report)
- **INT-driven async**: Efficient async design using Embassy GPIO interrupts
- **RST-based recovery**: Hardware reset capability for robust long-term operation
- **No external EKF needed**: BNO086's on-chip fusion eliminates CPU overhead

## Pin Configuration

| BNO086 Pin | RP2350 Pin | Function                    |
| ---------- | ---------- | --------------------------- |
| VIN        | 3.3V       | Power supply                |
| GND        | GND        | Ground                      |
| SDA        | GPIO4      | I2C0 Data                   |
| SCL        | GPIO5      | I2C0 Clock                  |
| INT        | GPIO6      | Data ready (active low)     |
| RST        | GPIO7      | Hardware reset (active low) |

## Progress Tracking

See [Implementation Plan](plan.md) for detailed task checklists and phase status.

## Notes

- BNO086 selected after ICM-20948 counterfeit issues (see AN-srhcj)
- SHTP protocol is more complex than standard I2C register access
- Quaternion output at 100Hz is sufficient for navigation (vs 400Hz raw IMU)
- On-chip calibration eliminates need for external calibration routines
- Compatible with BNO085 (same SHTP protocol)

---

## Template Usage

For detailed instructions and key principles, see [Template Usage Instructions](../../templates/README.md#task-template-taskmd) in the templates README.
