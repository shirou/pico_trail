# T-00158 SITL Platform and Lightweight Adapter Design

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

Implement two components that together provide a minimum viable SITL environment: `SitlPlatform` bridges the autopilot's `Platform` trait to simulated hardware, and `LightweightAdapter` provides built-in differential drive physics with sensor noise. This combination enables end-to-end testing of autopilot logic in CI without external simulators.

## Success Metrics

- [ ] `SitlPlatform` implements `Platform` trait
- [ ] Sensor injection and actuator collection work correctly
- [ ] `LightweightAdapter` passes kinematics tests
- [ ] Deterministic mode produces repeatable results
- [ ] No external dependencies required for testing

## Background and Current State

- `MockPlatform` exists in `crates/firmware/src/platform/mock/` for unit tests
- `MockPlatform` provides basic resource tracking but no physics simulation
- `SitlPlatform` differs from `MockPlatform` in that it connects to a live simulation loop
- The `Platform` trait uses associated types for compile-time dispatch (9 methods, 5 associated types)

## Proposed Design

### Component: SitlPlatform

**File**: `crates/sitl/src/platform/mod.rs`

```rust
pub struct SitlPlatform {
    vehicle_id: VehicleId,
    time_source: SitlTimeSource,

    // Injected sensor state
    imu_state: Mutex<ImuState>,
    gps_state: Mutex<GpsState>,
    compass_state: Mutex<CompassState>,

    // Captured actuator commands
    motor_commands: Mutex<MotorCommands>,

    // Simulated peripherals
    uart_ports: Vec<SitlUart>,
    pwm_channels: Vec<SitlPwm>,

    // Configuration
    battery_voltage: f32,
}

impl Platform for SitlPlatform {
    fn system_clock_hz(&self) -> u32 { 125_000_000 }

    fn create_uart(&mut self, id: u8, config: UartConfig) -> Result<Self::Uart, PlatformError>;
    fn create_spi(&mut self, id: u8, config: SpiConfig) -> Result<Self::Spi, PlatformError>;
    fn create_pwm(&mut self, pin: u8, config: PwmConfig) -> Result<Self::Pwm, PlatformError>;
    fn create_gpio(&mut self, pin: u8) -> Result<Self::Gpio, PlatformError>;
    fn read_battery_adc(&mut self) -> u16;
    fn timer(&self) -> &Self::Timer;
    fn timer_mut(&mut self) -> &mut Self::Timer;
}

impl SitlPlatform {
    /// Inject sensor data from bridge
    pub fn inject_sensors(&self, data: &SensorData) { ... }

    /// Collect actuator commands for bridge
    pub fn collect_actuator_commands(&self) -> ActuatorCommands { ... }
}
```

### Simulated Peripherals

**SitlTimeSource**: Wraps `Arc<AtomicU64>` for shared sim time, implements `TimerInterface`.

**SitlUart**: Ring buffer for RX/TX with `read()`, `write()` methods.

**SitlPwm**: Stores duty cycle (0.0-1.0) with `set_duty()`, `get_duty()`.

**SitlGpio**: Stores pin state (high/low) with input/output direction.

### Component: LightweightAdapter

**File**: `crates/sitl/src/adapter/lightweight.rs`

Built-in differential drive kinematics with configurable noise:

```rust
pub struct LightweightAdapter {
    config: LightweightConfig,
    state: VehicleState,
    rng: StdRng,
    sim_time_us: u64,
}

pub struct LightweightConfig {
    pub wheel_base: f32,           // meters (default: 0.15)
    pub max_speed: f32,            // m/s (default: 1.0)
    pub max_turn_rate: f32,        // rad/s (default: 2.0)
    pub gps_noise_m: f32,          // stddev (default: 0.5)
    pub gps_rate_hz: u32,          // (default: 5)
    pub accel_noise_mss: f32,      // stddev (default: 0.1)
    pub gyro_noise_rads: f32,      // stddev (default: 0.01)
    pub compass_noise_rad: f32,    // stddev (default: 0.05)
    pub seed: Option<u64>,         // for deterministic tests
}

impl LightweightAdapter {
    /// Integrate differential drive kinematics
    fn integrate(&mut self, dt: f32) {
        let v_left = self.state.motor_left * self.config.max_speed;
        let v_right = self.state.motor_right * self.config.max_speed;

        let v = (v_left + v_right) / 2.0;
        let omega = (v_right - v_left) / self.config.wheel_base;

        self.state.heading += omega * dt;
        self.state.position.x += v * self.state.heading.cos() * dt;
        self.state.position.y += v * self.state.heading.sin() * dt;
    }

    /// Synthesize sensor data with noise
    fn synthesize_sensors(&self) -> SensorData { ... }
}
```

### Interaction: Bridge ↔ Platform ↔ Adapter

```text
SitlBridge          Adapter              SitlPlatform
    │                  │                        │
    │── step() ───────▶│                        │
    │                  │ physics_step()         │
    │                  │                        │
    │◀── sensors ──────│                        │
    │                                           │
    │── inject_sensors() ──────────────────────▶│
    │                                           │
    │                           control_loop() ─│
    │                                           │
    │◀── collect_actuator_commands() ───────────│
    │                                           │
    │── send_actuators() ▶│                     │
    │                     │                     │
```

## Testing Strategy

### Unit Tests

- **SitlPlatform**: Platform trait implementation compiles, UART read/write, PWM duty cycle, sensor injection, actuator collection
- **LightweightAdapter kinematics**: Straight line, rotation in place, arc turn
- **Deterministic mode**: Same seed produces same results
- **Noise**: Outputs vary with noise enabled

## External References

- [ADR-00156 SITL Pluggable Adapter Architecture](../../adr/ADR-00156-sitl-pluggable-adapter-architecture.md)
- [ADR-00003 Platform Abstraction](../../adr/ADR-00003-platform-abstraction.md)
