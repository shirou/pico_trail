# FR-00154 SITL Platform Trait Implementation

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
  - [AN-00004-platform-abstraction](../analysis/AN-00004-platform-abstraction.md)
- Prerequisite Requirements:
  - [FR-00148-sitl-simulator-adapter-trait](FR-00148-sitl-simulator-adapter-trait.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00158-sitl-platform-and-lightweight-adapter](../tasks/T-00158-sitl-platform-and-lightweight-adapter/README.md)

## Requirement Statement

SITL shall implement the `Platform` trait, enabling existing pico_trail autopilot code to run unchanged in simulation. Sensors return data injected by the simulator adapter; actuators capture commands for transmission to the simulator.

## Rationale

The Platform trait abstraction already exists for embedded targets (RP2350) and testing (Mock). Implementing it for SITL allows the same mode implementations, navigation logic, and control algorithms to run in simulation without modification.

## User Story (if applicable)

As a developer, I want to test my Guided mode implementation in SITL without any code changes so that I know it will work the same on real hardware.

## Acceptance Criteria

- [ ] `SitlPlatform` implements the `Platform` trait
- [ ] `create_uart()` returns simulated UART (for MAVLink)
- [ ] `create_spi()` returns stub SPI (IMU typically via SPI on real hardware)
- [ ] `create_pwm()` captures PWM values for actuator commands
- [ ] `create_gpio()` returns simulated GPIO
- [ ] `read_battery_adc()` returns configurable simulated voltage
- [ ] `timer()` returns simulation time from adapter
- [ ] Sensors (IMU, GPS, compass) populated from `SensorData`
- [ ] Actuator outputs collected into `ActuatorCommands`
- [ ] All rover modes work in SITL (Manual, Guided, Auto, RTL, Loiter, Circle)
- [ ] Unit tests using SitlPlatform with LightweightAdapter

## Technical Details (if applicable)

### Platform Implementation

```rust
pub struct SitlPlatform {
    /// Simulation time source
    time_source: SitlTimeSource,

    /// Injected sensor data
    imu_state: ImuState,
    gps_state: GpsState,
    compass_state: CompassState,

    /// Captured actuator commands
    motor_commands: MotorCommands,

    /// Simulated peripherals
    uart_ports: Vec<SitlUart>,
    pwm_channels: Vec<SitlPwm>,

    /// Configuration
    battery_voltage: f32,
}

impl Platform for SitlPlatform {
    fn system_clock_hz(&self) -> u32 {
        125_000_000  // Match RP2350
    }

    fn create_uart(&mut self, id: u8, config: UartConfig) -> Result<Box<dyn Uart>, PlatformError> {
        // Return simulated UART for MAVLink
        Ok(Box::new(SitlUart::new(id, config)))
    }

    fn create_pwm(&mut self, channel: u8) -> Result<Box<dyn Pwm>, PlatformError> {
        // PWM captures duty cycle for actuator commands
        let pwm = SitlPwm::new(channel);
        self.pwm_channels.push(pwm.clone());
        Ok(Box::new(pwm))
    }

    fn read_battery_adc(&self) -> u16 {
        // Convert voltage to ADC value
        voltage_to_adc(self.battery_voltage)
    }

    fn timer(&self) -> &dyn Timer {
        &self.time_source
    }
}
```

### Sensor Injection

```rust
impl SitlPlatform {
    /// Called by SITL Bridge to inject sensor data
    pub fn inject_sensors(&mut self, data: &SensorData) {
        if let Some(imu) = &data.imu {
            self.imu_state.update(imu);
        }
        if let Some(gps) = &data.gps {
            self.gps_state.update(gps);
        }
        if let Some(compass) = &data.compass {
            self.compass_state.update(compass);
        }
    }

    /// Called by SITL Bridge to collect actuator commands
    pub fn collect_actuator_commands(&self) -> ActuatorCommands {
        ActuatorCommands {
            timestamp_us: self.time_source.now_us(),
            vehicle_id: self.vehicle_id,
            motors: self.pwm_channels.iter().map(|p| p.get_duty()).collect(),
            servos: vec![],
        }
    }
}
```

### Time Source

```rust
pub struct SitlTimeSource {
    /// Current simulation time (microseconds)
    sim_time_us: Arc<AtomicU64>,
}

impl Timer for SitlTimeSource {
    fn now_us(&self) -> u64 {
        self.sim_time_us.load(Ordering::Relaxed)
    }

    fn delay_us(&mut self, us: u32) -> Result<(), TimerError> {
        // In SITL, delays are simulated - just advance time
        // (or block in free-running mode)
        Ok(())
    }
}
```

## Platform Considerations

### Host Only

- SitlPlatform runs on host, not embedded
- Uses std library features (Arc, threads)
- No Embassy or embedded HAL dependencies

### Relationship to MockPlatform

- MockPlatform: For unit tests, manually controlled state
- SitlPlatform: For integration tests, state driven by simulator

## Risks & Mitigation

| Risk                          | Impact | Likelihood | Mitigation                                | Validation       |
| ----------------------------- | ------ | ---------- | ----------------------------------------- | ---------------- |
| Platform trait mismatch       | High   | Low        | Keep in sync with embedded Platform       | CI tests both    |
| Timing differences vs real HW | Medium | Medium     | Document differences, use same time units | Comparison tests |

## Implementation Notes

- SitlPlatform reuses much of MockPlatform's structure
- Sensor traits (ImuSensor, GpsSensor) wrap injected state
- Consider making sensor update thread-safe for free-running mode

## External References

- [Platform trait definition](../analysis/AN-00004-platform-abstraction.md)
