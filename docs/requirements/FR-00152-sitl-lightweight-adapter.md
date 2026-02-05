# FR-00152 SITL Lightweight Adapter

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Prerequisite Requirements:
  - [FR-00148-sitl-simulator-adapter-trait](FR-00148-sitl-simulator-adapter-trait.md)
- Dependent Requirements:
  - [NFR-00096-sitl-lightweight-no-deps](NFR-00096-sitl-lightweight-no-deps.md)
- Related Tasks:
  - [T-00158-sitl-platform-and-lightweight-adapter](../tasks/T-00158-sitl-platform-and-lightweight-adapter/README.md)

## Requirement Statement

`LightweightAdapter` shall implement the `SimulatorAdapter` trait with built-in differential drive kinematics and configurable sensor noise. It shall require no external simulator process, enabling fast CI testing without Gazebo dependencies.

## Rationale

CI pipelines need fast, deterministic tests without heavy dependencies. A lightweight adapter with simple kinematics provides:

1. Sub-second test startup (no Gazebo launch)
2. Deterministic behavior for regression tests
3. No external process management
4. Runs anywhere Rust compiles

## User Story (if applicable)

As a CI system, I want to run SITL navigation tests in under 10 seconds so that every commit gets fast feedback without requiring Gazebo installation.

## Acceptance Criteria

- [ ] `LightweightAdapter` implements `SimulatorAdapter` trait
- [ ] Built-in differential drive kinematics (wheel_base, max_speed configurable)
- [ ] Integrates motor commands to position/velocity
- [ ] Generates IMU data from vehicle motion (accel, gyro)
- [ ] Generates GPS data from position with configurable noise
- [ ] Generates compass data from heading with configurable noise
- [ ] Configurable sensor noise parameters (stddev for each sensor)
- [ ] Deterministic mode with seeded RNG for reproducible tests
- [ ] Supports lockstep synchronization (single-threaded, no external deps)
- [ ] No external process required — pure Rust implementation
- [ ] `cargo test` sufficient to run SITL tests
- [ ] Performance: <1ms per simulation step
- [ ] Unit tests for kinematics accuracy

## Technical Details (if applicable)

### Configuration

```rust
pub struct LightweightConfig {
    /// Wheel separation (meters)
    pub wheel_base: f32,          // default: 0.15 (Freenove 4WD)
    /// Maximum speed (m/s)
    pub max_speed: f32,           // default: 1.0
    /// Maximum turn rate (rad/s)
    pub max_turn_rate: f32,       // default: 2.0

    /// GPS noise standard deviation (meters)
    pub gps_noise_m: f32,         // default: 0.5
    /// GPS update rate (Hz)
    pub gps_rate_hz: u32,         // default: 5

    /// IMU noise parameters
    pub accel_noise_mss: f32,     // default: 0.1
    pub gyro_noise_rads: f32,     // default: 0.01

    /// Compass noise (radians)
    pub compass_noise_rad: f32,   // default: 0.05

    /// Random seed for deterministic mode (None = random)
    pub seed: Option<u64>,
}
```

### Kinematics Model

```rust
impl LightweightAdapter {
    fn integrate_kinematics(&mut self, dt: f32) {
        // Differential drive: v = (v_left + v_right) / 2
        //                     ω = (v_right - v_left) / wheel_base
        let v_left = self.motor_left * self.config.max_speed;
        let v_right = self.motor_right * self.config.max_speed;

        let v = (v_left + v_right) / 2.0;
        let omega = (v_right - v_left) / self.config.wheel_base;

        // Update heading
        self.heading += omega * dt;

        // Update position
        self.position.x += v * self.heading.cos() * dt;
        self.position.y += v * self.heading.sin() * dt;

        // Update velocity
        self.velocity = Vector3::new(v * self.heading.cos(), v * self.heading.sin(), 0.0);
    }
}
```

### Sensor Synthesis

```rust
impl LightweightAdapter {
    fn synthesize_imu(&self) -> ImuData {
        let accel = Vector3::new(
            self.linear_accel.x + self.rng.normal(0.0, self.config.accel_noise_mss),
            self.linear_accel.y + self.rng.normal(0.0, self.config.accel_noise_mss),
            -9.81 + self.rng.normal(0.0, self.config.accel_noise_mss),
        );
        let gyro = Vector3::new(
            0.0,
            0.0,
            self.angular_velocity + self.rng.normal(0.0, self.config.gyro_noise_rads),
        );
        ImuData { accel_mss: accel.into(), gyro_rads: gyro.into(), temperature_c: 25.0 }
    }
}
```

## Platform Considerations

### Cross-Platform

- Pure Rust, no FFI or external processes
- Uses `rand` crate for noise generation
- Compiles on any platform supporting Rust std

### CI Integration

```yaml
# No special setup needed
- name: Run SITL tests
  run: cargo test --features sitl
```

## Risks & Mitigation

| Risk                                 | Impact | Likelihood | Mitigation                                          | Validation         |
| ------------------------------------ | ------ | ---------- | --------------------------------------------------- | ------------------ |
| Kinematics too simple for some tests | Medium | Medium     | Document limitations, recommend Gazebo for fidelity | Comparison tests   |
| Non-deterministic test failures      | Medium | Low        | Seeded RNG, deterministic mode                      | Repeated test runs |

## Implementation Notes

- Start with basic differential drive, consider adding Ackermann steering later
- Sensor noise can be disabled by setting stddev to 0.0
- Consider adding terrain support (elevation map) in future
- No collision detection in initial version

## External References

- [Differential Drive Kinematics](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)
