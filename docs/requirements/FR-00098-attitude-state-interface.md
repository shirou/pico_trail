# FR-00098 Attitude State Interface

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00104-quaternion-ekf-ahrs](FR-00104-quaternion-ekf-ahrs.md)
  - [FR-00001-ahrs-attitude-estimation](FR-00001-ahrs-attitude-estimation.md)
- Dependent Requirements:
  - [FR-00004-gps-waypoint-navigation](FR-00004-gps-waypoint-navigation.md)
  - [FR-00031-ekf-health-validation](FR-00031-ekf-health-validation.md)
- Related ADRs:
  - [ADR-00025-ekf-ahrs-implementation](../adr/ADR-00025-ekf-ahrs-implementation.md)
- Related Tasks:
  - [T-00022-ekf-ahrs-implementation](../tasks/T-00022-ekf-ahrs-implementation/README.md)

## Requirement Statement

The system shall provide a global `ATTITUDE_STATE` interface that makes EKF-computed attitude data accessible to navigation, control, and telemetry subsystems through a thread-safe mutex-protected structure.

## Rationale

Multiple subsystems need access to attitude data:

- **Navigation**: Uses heading for waypoint tracking
- **Control**: Uses roll/pitch/yaw for vehicle stabilization
- **Telemetry**: Reports attitude to ground station
- **Failsafe**: Monitors attitude for anomaly detection

A global state pattern (following GPS_STATE precedent) provides:

- Consistent access pattern across subsystems
- Thread-safe access via mutex
- Decoupling between EKF and consumers
- Single source of truth for attitude

## User Story (if applicable)

As a subsystem developer, I want to access current attitude data through a well-defined global interface, so that I can use attitude in navigation and control without direct EKF coupling.

## Acceptance Criteria

- [ ] `AttitudeState` struct contains quaternion, Euler angles, and angular rates
- [ ] `ATTITUDE_STATE` global accessible from any module
- [ ] Thread-safe access via `CriticalSectionRawMutex`
- [ ] State updated by EKF task at 100Hz
- [ ] Consumers can read state asynchronously
- [ ] State includes timestamp for staleness detection
- [ ] State includes health flag from EKF
- [ ] State includes gyro bias estimate for diagnostics

## Technical Details (if applicable)

### Functional Requirement Details

**AttitudeState Structure:**

```rust
/// Global attitude state accessible by all subsystems
#[derive(Clone, Copy, Default)]
pub struct AttitudeState {
    // Quaternion representation (for gimbal-lock-free math)
    /// Unit quaternion: [w, x, y, z]
    pub quaternion: [f32; 4],

    // Euler angles (for human readability and MAVLink)
    /// Roll angle (radians, positive = right wing down)
    pub roll: f32,

    /// Pitch angle (radians, positive = nose up)
    pub pitch: f32,

    /// Yaw/heading angle (radians, 0 = North, positive = clockwise)
    pub yaw: f32,

    // Angular rates from gyro (body frame)
    /// Roll rate (rad/s)
    pub roll_rate: f32,

    /// Pitch rate (rad/s)
    pub pitch_rate: f32,

    /// Yaw rate (rad/s)
    pub yaw_rate: f32,

    // EKF diagnostics
    /// Estimated gyro bias (rad/s)
    pub gyro_bias: [f32; 3],

    /// EKF covariance diagonal (uncertainty)
    pub covariance: [f32; 7],

    // Status
    /// EKF healthy and converged
    pub healthy: bool,

    /// Timestamp of last update (ms since boot)
    pub timestamp_ms: u32,
}
```

**Global State Declaration:**

```rust
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// Global attitude state - updated by EKF, read by other subsystems
pub static ATTITUDE_STATE: Mutex<CriticalSectionRawMutex, AttitudeState> =
    Mutex::new(AttitudeState::new());
```

**EKF Update:**

```rust
// In EKF task, update global state at 100Hz
async fn update_global_state(&self, gyro: Vector3<f32>) {
    let state = AttitudeState {
        quaternion: self.quaternion().as_slice().try_into().unwrap(),
        roll: self.roll(),
        pitch: self.pitch(),
        yaw: self.yaw(),
        roll_rate: gyro.x,
        pitch_rate: gyro.y,
        yaw_rate: gyro.z,
        gyro_bias: self.gyro_bias().as_slice().try_into().unwrap(),
        covariance: self.covariance_diagonal(),
        healthy: self.is_healthy(),
        timestamp_ms: now_ms(),
    };

    ATTITUDE_STATE.lock().await.copy_from(&state);
}
```

**Consumer Access:**

```rust
// Navigation controller reads attitude
async fn navigation_update() {
    let attitude = ATTITUDE_STATE.lock().await;

    // Check staleness
    if now_ms() - attitude.timestamp_ms > 100 {
        log_warn!("Attitude data stale");
        return;
    }

    // Use heading for waypoint navigation
    let heading = attitude.yaw;
    // ...
}

// Telemetry reads attitude for MAVLink
async fn send_attitude_telemetry() {
    let attitude = ATTITUDE_STATE.lock().await;

    let msg = mavlink::common::ATTITUDE {
        time_boot_ms: boot_time_ms(),
        roll: attitude.roll,
        pitch: attitude.pitch,
        yaw: attitude.yaw,
        rollspeed: attitude.roll_rate,
        pitchspeed: attitude.pitch_rate,
        yawspeed: attitude.yaw_rate,
    };
    // Send message...
}
```

**Staleness Detection:**

```rust
impl AttitudeState {
    const MAX_AGE_MS: u32 = 100; // 100ms = 10 updates

    pub fn is_stale(&self, current_time_ms: u32) -> bool {
        current_time_ms.wrapping_sub(self.timestamp_ms) > Self::MAX_AGE_MS
    }
}
```

## Platform Considerations

### Cross-Platform

- Uses `embassy_sync` for thread-safe mutex
- Same interface on both RP2040 and RP2350
- `CriticalSectionRawMutex` works in interrupt context

## Risks & Mitigation

| Risk                                   | Impact | Likelihood | Mitigation                                      | Validation                |
| -------------------------------------- | ------ | ---------- | ----------------------------------------------- | ------------------------- |
| Mutex contention delays critical tasks | Medium | Low        | Keep lock duration minimal, copy data out       | Profile lock hold times   |
| Stale data used after EKF failure      | High   | Low        | Check timestamp and healthy flag                | Test with EKF task killed |
| Memory overhead from copying           | Low    | Low        | AttitudeState is small (\~80 bytes)             | Verify memory usage       |
| Euler angle singularity                | Medium | Low        | Use quaternion for math, Euler for display only | Test near ±90° pitch      |

## Implementation Notes

**Module Location:**

```
src/subsystems/ahrs/
├── mod.rs              # Public exports including ATTITUDE_STATE
├── ekf.rs              # EKF implementation
└── state.rs            # AttitudeState definition
```

**Quaternion to Euler Conversion:**

```rust
impl AttitudeState {
    pub fn from_quaternion(q: &[f32; 4]) -> (f32, f32, f32) {
        let (w, x, y, z) = (q[0], q[1], q[2], q[3]);

        // Roll (x-axis rotation)
        let sinr_cosp = 2.0 * (w * x + y * z);
        let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        let roll = sinr_cosp.atan2(cosr_cosp);

        // Pitch (y-axis rotation)
        let sinp = 2.0 * (w * y - z * x);
        let pitch = if sinp.abs() >= 1.0 {
            core::f32::consts::FRAC_PI_2.copysign(sinp)
        } else {
            sinp.asin()
        };

        // Yaw (z-axis rotation)
        let siny_cosp = 2.0 * (w * z + x * y);
        let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        let yaw = siny_cosp.atan2(cosy_cosp);

        (roll, pitch, yaw)
    }
}
```

## External References

- [Embassy Sync Documentation](https://docs.embassy.dev/embassy-sync/git/default/mutex/struct.Mutex.html)
- [Quaternion to Euler Conversion](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
