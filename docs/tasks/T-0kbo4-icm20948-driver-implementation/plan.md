# T-0kbo4 ICM-20948 Driver Implementation - Plan

## Metadata

- Type: Task Plan
- Status: Cancelled

## Links

- Parent Task:
  - [T-0kbo4-icm20948-driver-implementation](./README.md)
- Related Design:
  - [T-0kbo4-icm20948-driver-implementation-design](./design.md)

## Implementation Phases

### Phase 1: Register Definitions and Basic I2C

**Objective**: Establish register definitions and basic I2C communication.

**Deliverables**:

- [x] `src/devices/imu/icm20948/mod.rs` - Module exports
- [x] `src/devices/imu/icm20948/registers.rs` - All 4 bank register definitions
- [x] `src/devices/imu/icm20948/config.rs` - Configuration structs
- [x] Bank switching implementation
- [x] WHO_AM_I verification

**Verification**:

- [x] `cargo test --lib` passes
- [x] `cargo clippy` passes (warnings for unused constants expected until Phase 2)
- [ ] I2C scan example detects ICM-20948 at 0x68 or 0x69

### Phase 2: Sensor Reading and Initialization

**Objective**: Implement full sensor initialization and data reading.

**Deliverables**:

- [x] `src/devices/imu/icm20948/driver.rs` - Core driver implementation
- [x] Initialization sequence (reset, wake, configure)
- [x] Gyroscope reading with unit conversion (rad/s)
- [x] Accelerometer reading with unit conversion (m/s²)
- [x] Temperature reading with unit conversion (°C)

**Verification**:

- [ ] Example reads gyro/accel/temp successfully
- [ ] Values change when sensor is moved
- [ ] Unit conversions match expected values (gravity = 9.81 m/s²)

### Phase 3: Magnetometer Integration and Hardware Calibration

**Objective**: Implement AK09916 magnetometer access via bypass mode and hardware calibration API.

**Deliverables**:

- [x] I2C bypass mode enable
- [x] AK09916 WHO_AM_I verification (0x09)
- [x] AK09916 continuous mode configuration
- [x] Magnetometer reading with unit conversion (µT)
- [x] DRDY and overflow handling
- [x] Hardware calibration API for User Bank offset registers:
  - [x] Bank 1 accelerometer offset registers (XA_OFFS_H/L, YA_OFFS_H/L, ZA_OFFS_H/L)
  - [x] `set_gyro_offset()` / `get_gyro_offset()` (Bank 2, 16-bit)
  - [x] `set_accel_offset()` / `get_accel_offset()` (Bank 1, 15-bit)

**Verification**:

- [ ] Magnetometer reads different values when sensor is rotated
- [ ] Values are in expected µT range (Earth's field: 25-65 µT)
- [ ] Hardware offsets can be written and read back correctly

### Phase 4: ImuSensor Trait Implementation

**Objective**: Implement the `ImuSensor` trait for EKF integration.

**Deliverables**:

- [x] `ImuSensor` trait implementation for `Icm20948Driver`
- [x] `read_all()` - Combined 9-axis read
- [x] `read_gyro()`, `read_accel()`, `read_mag()` - Individual reads
- [x] `set_calibration()` - Calibration data loading
- [x] `is_healthy()` - Health status query
- [x] Calibration application to raw readings

**Verification**:

- [x] Trait implementation compiles
- [ ] Mock tests verify trait behavior
- [x] Calibration offsets correctly applied

### Phase 5: Embassy Task and Integration

**Objective**: Create Embassy async task for 400Hz sampling.

**Deliverables**:

- [ ] Embassy task for IMU sampling loop
- [ ] Global `IMU_DATA` state (if needed)
- [ ] Error handling and retry logic
- [ ] Health monitoring (stuck sensor detection)

**Verification**:

- [ ] 400Hz sampling rate achieved (±5Hz)
- [ ] I2C read latency < 1.5ms (95th percentile)
- [ ] Error recovery works (sensor disconnect/reconnect)

### Phase 6: Testing and Documentation

**Objective**: Complete unit tests and documentation.

**Deliverables**:

- [x] Unit tests for all public APIs
- [x] Mock I2C tests for register sequences (N/A - driver is feature-gated)
- [x] Integration example (`examples/icm20948_demo.rs`)
- [x] Code documentation (rustdoc comments)

**Verification**:

- [x] `cargo test --lib` passes (530 tests passing)
- [x] `./scripts/build-rp2350.sh icm20948_demo` builds successfully
- [ ] Example runs on hardware and displays sensor data

## Risk Mitigation

| Risk                      | Mitigation                                  |
| ------------------------- | ------------------------------------------- |
| Bank switching overhead   | Cache current bank, minimize transitions    |
| I2C bus contention        | Dedicated I2C bus or careful timing         |
| Magnetometer interference | Document mounting requirements, calibration |
| VDDIO voltage issues      | Use level-shifted breakout boards           |

## Dependencies

- Phase 2 depends on Phase 1
- Phase 3 depends on Phase 2
- Phase 4 depends on Phases 2 and 3
- Phase 5 depends on Phase 4
- Phase 6 can run in parallel with Phase 5

## Estimated Effort

- Phase 1: 2-3 hours
- Phase 2: 3-4 hours
- Phase 3: 2-3 hours
- Phase 4: 2-3 hours
- Phase 5: 3-4 hours
- Phase 6: 2-3 hours

**Total**: 14-20 hours

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../../templates/README.md#plan-template-planmd) in the templates README.
