# T-7khm3 AHRS Abstraction Layer Design

## Metadata

- Type: Design
- Status: Approved

## Links

- Associated Plan Document:
  - [T-7khm3-ahrs-abstraction-layer-plan](./plan.md)

## Overview

This design implements a unified AHRS abstraction layer that decouples flight control from specific attitude estimation implementations. The abstraction supports two primary use cases: External AHRS (sensors like BNO086 with on-chip fusion providing quaternion output) and Software AHRS (raw IMU sensors processed by software EKF). Flight control code depends only on the abstract `Ahrs` trait, enabling sensor-agnostic operation and optimal resource utilization.

## Success Metrics

- [x] `Ahrs` trait compiles with both `ExternalAhrs` and future `SoftwareAhrs` implementations
- [x] `Bno086ExternalAhrs` produces valid attitude at 100Hz with < 1ms overhead
- [x] Memory overhead for abstraction layer < 256 bytes
- [x] Zero regressions in existing BNO086 functionality

## Background and Current State

- Context: pico_trail supports BNO086 IMU with on-chip fusion via `QuaternionSensor` trait. Future support for raw IMUs (ICM-42688 or else) requires software EKF. Flight control needs a common interface regardless of AHRS source.
- Current behavior:
  - `src/devices/traits/quaternion.rs` - `QuaternionSensor` trait for BNO086
  - `src/devices/imu/bno086/` - BNO086 driver implementing `QuaternionSensor`
  - `src/subsystems/ahrs/state.rs` - `AttitudeState` (Euler angles, no quaternion)
  - `src/subsystems/ahrs/dcm.rs` - DCM algorithm (to be superseded by EKF)
- Pain points:
  - No unified interface for attitude consumers
  - `AttitudeState` lacks quaternion field
  - Flight control cannot switch between AHRS sources
- Constraints:
  - RP2040: No FPU, 264KB RAM
  - RP2350: Hardware FPU, 520KB RAM
  - Embassy async runtime required
  - < 10KB RAM for AHRS state (per ADR-ymkzt)
- Related ADRs:
  - [ADR-nzvfy-ahrs-abstraction-architecture](../../adr/ADR-nzvfy-ahrs-abstraction-architecture.md)
  - [ADR-ymkzt-ekf-ahrs-implementation](../../adr/ADR-ymkzt-ekf-ahrs-implementation.md)

## Proposed Design

### High-Level Architecture

```text
                           ┌──────────────────┐
                           │  Flight Control  │
                           │ (AttitudeController) │
                           └────────▲─────────┘
                                    │
                           ┌────────┴─────────┐
                           │    Ahrs Trait    │
                           │ (Common Interface) │
                           └────────▲─────────┘
                                    │
          ┌─────────────────────────┼─────────────────────────┐
          │                         │                         │
 ┌────────┴────────┐       ┌────────┴────────┐       ┌────────┴────────┐
 │  SoftwareAhrs   │       │  ExternalAhrs   │       │  ExternalAhrs   │
 │  (EKF/DCM)      │       │  (BNO086)       │       │  (Future: VN-100) │
 │  [T-p8w8f]      │       │  [This Task]    │       │                 │
 └────────▲────────┘       └────────▲────────┘       └─────────────────┘
          │                         │
 ┌────────┴────────┐       ┌────────┴────────┐
 │   RawImu Trait  │       │ QuaternionSensor│
 │  (Future)       │       │  (Existing)     │
 └────────▲────────┘       └─────────────────┘
          │
 ┌────────┴────────┐
 │  ICM-42688      │
 │  (Future)       │
 └─────────────────┘
```

### Components

#### Core Traits and Types (`src/ahrs/mod.rs`)

```rust
/// AHRS output - common interface for all attitude sources
#[allow(async_fn_in_trait)]
pub trait Ahrs {
    /// Get current attitude as quaternion with angular rates
    async fn get_attitude(&mut self) -> Result<AhrsState, AhrsError>;

    /// Check if AHRS is healthy and producing valid data
    fn is_healthy(&self) -> bool;

    /// Get AHRS type identifier (for logging/diagnostics)
    fn ahrs_type(&self) -> AhrsType;
}

/// AHRS type for runtime identification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AhrsType {
    /// Software EKF/DCM processing raw IMU data
    Software,
    /// External AHRS (BNO086, VectorNav, etc.)
    External,
}

/// AHRS error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AhrsError {
    /// Sensor communication failed
    SensorError,
    /// Sensor not initialized
    NotInitialized,
    /// Invalid data received
    InvalidData,
    /// Timeout waiting for data
    Timeout,
    /// AHRS not converged (software EKF only)
    NotConverged,
}
```

#### AHRS State (`src/ahrs/state.rs`)

```rust
use nalgebra::{Quaternion, Vector3};

/// AHRS state output - unified attitude representation
#[derive(Debug, Clone, Copy)]
pub struct AhrsState {
    /// Attitude quaternion (NED frame, scalar-first: w, x, y, z)
    pub quaternion: Quaternion<f32>,

    /// Euler angles in radians (roll, pitch, yaw)
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,

    /// Angular rates in body frame (rad/s)
    pub angular_rate: Vector3<f32>,

    /// Linear acceleration in body frame (m/s²)
    pub acceleration: Vector3<f32>,

    /// Timestamp (microseconds since boot)
    pub timestamp_us: u64,

    /// Health indicator
    pub healthy: bool,
}
```

#### External AHRS for BNO086 (`src/ahrs/external/bno086.rs`)

```rust
use crate::communication::shtp::ShtpTransport;
use crate::devices::imu::bno086::Bno086Driver;
use crate::devices::traits::{QuaternionSensor, QuaternionError};

/// BNO086 as External AHRS
pub struct Bno086ExternalAhrs<T: ShtpTransport> {
    driver: Bno086Driver<T>,
    last_state: Option<AhrsState>,
}

impl<T: ShtpTransport> Ahrs for Bno086ExternalAhrs<T> {
    async fn get_attitude(&mut self) -> Result<AhrsState, AhrsError> {
        let reading = self.driver.read_quaternion().await
            .map_err(Self::map_error)?;

        let state = AhrsState::from_quaternion_reading(&reading);
        self.last_state = Some(state);
        Ok(state)
    }

    fn is_healthy(&self) -> bool {
        self.driver.is_healthy()
    }

    fn ahrs_type(&self) -> AhrsType {
        AhrsType::External
    }
}
```

#### RawImu Trait (`src/devices/traits/raw_imu.rs`)

```rust
use nalgebra::Vector3;

/// Raw IMU sensor interface for sensors without on-chip fusion
#[allow(async_fn_in_trait)]
pub trait RawImu {
    /// Read calibrated accelerometer data (m/s²)
    async fn read_accel(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read calibrated gyroscope data (rad/s)
    async fn read_gyro(&mut self) -> Result<Vector3<f32>, ImuError>;

    /// Read calibrated magnetometer data (optional, µT)
    async fn read_mag(&mut self) -> Result<Option<Vector3<f32>>, ImuError>;

    /// Get sample rate (Hz)
    fn sample_rate(&self) -> u32;

    /// Check if sensor is healthy
    fn is_healthy(&self) -> bool;
}
```

### Data Flow

1. **External AHRS (BNO086)**:
   - BNO086 on-chip ARM Cortex-M0+ runs sensor fusion at 400Hz
   - `Bno086ExternalAhrs::get_attitude()` reads quaternion via SHTP/I2C
   - Quaternion converted to `AhrsState` (adds Euler angles)
   - Flight control receives unified `AhrsState`

2. **Software AHRS (Future)**:
   - Raw IMU implements `RawImu` trait (accel, gyro, mag)
   - `SoftwareAhrs<I: RawImu>` wraps raw IMU with EKF
   - EKF predict/update produces quaternion and bias estimates
   - Result converted to same `AhrsState` format

### Coordinate Frame Contract

**All `Ahrs` trait implementations MUST output data in NED (North-East-Down) frame.**

This is a critical contract for flight control consistency:

- **NED Frame Definition**:
  - X-axis: North (forward)
  - Y-axis: East (right)
  - Z-axis: Down

- **BNO086 Considerations**:
  - BNO086 outputs in a right-handed coordinate system with Z-up by default
  - The `Bno086ExternalAhrs` implementation is responsible for converting to NED
  - Conversion: `(x_ned, y_ned, z_ned) = (x_bno, y_bno, -z_bno)` and quaternion adjustment

- **Verification**: Unit tests must verify that a sensor placed flat (Z-up) reports roll=0, pitch=0

### Angular Rate Requirement

**Angular rate (`angular_rate` field) is REQUIRED, not optional.**

PID control for attitude requires:

- **P-term**: Uses attitude error (quaternion/Euler angles)
- **D-term**: Uses angular rate (gyroscope data) for damping

If `angular_rate` is zero, the D-term becomes ineffective, leading to oscillation and poor control performance.

**BNO086 Implementation**:

- Must enable `GYROSCOPE_CALIBRATED` report (Report ID 0x05) alongside rotation vector
- Driver must read both reports and combine into `AhrsState`
- `Bno086ExternalAhrs::get_attitude()` must populate `angular_rate` with actual gyro data

### Data Models and Types

```rust
// Quaternion convention: scalar-first (w, x, y, z)
// Frame: NED (North-East-Down) - ALL implementations must conform
// Euler sequence: ZYX (yaw-pitch-roll)

/// Convert quaternion to Euler angles (ZYX convention)
impl AhrsState {
    pub fn from_quaternion(q: Quaternion<f32>, timestamp_us: u64) -> Self {
        let (roll, pitch, yaw) = quaternion_to_euler_zyx(&q);
        Self {
            quaternion: q,
            roll,
            pitch,
            yaw,
            angular_rate: Vector3::zeros(), // Must be populated by caller
            acceleration: Vector3::zeros(),
            timestamp_us,
            healthy: true,
        }
    }
}
```

### Error Handling

- `AhrsError` maps from sensor-specific errors (`QuaternionError`, `ImuError`)
- External AHRS: Communication errors propagate as `AhrsError::SensorError`
- Software AHRS: Divergence detected via covariance trace, returns `AhrsError::NotConverged`
- Health status (`is_healthy()`) indicates ongoing validity, not transient errors

### Performance Considerations

- **Hot Path**: `get_attitude()` called at 100Hz by flight control
  - External AHRS: Single I2C read (\~1ms with DMA)
  - Software AHRS: EKF predict/update (\~2-5ms on RP2350 with FPU)
- **Memory**: `AhrsState` is 64 bytes; trait objects avoided (generics used)
- **No Allocation**: All operations use stack allocation

### Platform Considerations

#### RP2040

- No FPU: Quaternion-to-Euler conversion uses software float (\~50µs)
- Embassy async I2C for non-blocking sensor reads

#### RP2350

- Hardware FPU: Quaternion operations 3-5x faster
- Same code paths, automatic optimization

## Alternatives Considered

1. **Trait Object (`dyn Ahrs`)**
   - Pros: Runtime polymorphism, simpler type signatures
   - Cons: Vtable overhead, no inlining, heap allocation for Box<dyn Ahrs>

2. **Enum Dispatch**
   - Pros: No heap, known variants at compile time
   - Cons: Must enumerate all AHRS types, less extensible

Decision Rationale

- Generics (`impl Ahrs`) chosen for zero-cost abstraction
- Flight control generic over `A: Ahrs`, monomorphized at compile time
- Matches ArduPilot's template-based approach

## Testing Strategy

### Unit Tests

- `AhrsState::from_quaternion()` - Euler angle extraction accuracy
- `Bno086ExternalAhrs` - Mock transport, verify state conversion
- Error mapping - All `QuaternionError` variants map correctly

### Integration Tests

- BNO086 on hardware - Verify 100Hz update rate achieved
- Attitude plausibility - Static orientation within ±2°

## Documentation Impact

- Update `docs/architecture.md` to document `src/ahrs/` module
- Add module-level documentation in `src/ahrs/mod.rs`

## Open Questions

- [x] Should `AhrsState` include covariance/uncertainty estimates? → Method: Only for SoftwareAhrs; add optional field
- [x] BNO086 angular rate output requires separate report? → Resolved: Yes, must enable GYROSCOPE_CALIBRATED (0x05) alongside rotation vector. Added as required Phase 2 task.
- [x] BNO086 coordinate frame (ENU vs NED)? → Resolved: BNO086 outputs Z-up frame. Conversion to NED required in `Bno086ExternalAhrs`. Added as Phase 2 task.

## Appendix

### File Structure

```text
src/
├── subsystems/
│   └── ahrs/
│       ├── mod.rs          # Module exports, Ahrs trait re-export
│       ├── traits.rs       # Ahrs trait, AhrsType, AhrsError, AhrsState
│       ├── state.rs        # Legacy AttitudeState (existing)
│       ├── dcm.rs          # DCM algorithm (existing)
│       ├── external/
│       │   ├── mod.rs
│       │   └── bno086.rs   # Bno086ExternalAhrs (Phase 2)
│       └── ...             # Other existing files
├── devices/
│   └── traits/
│       ├── mod.rs          # Add RawImu export (Phase 3)
│       ├── quaternion.rs   # Existing (unchanged)
│       └── raw_imu.rs      # New RawImu trait (Phase 3)
```

---

## Template Usage

For detailed instructions on using this template, see [Template Usage Instructions](../../templates/README.md#design-template-designmd) in the templates README.
