# FR-00153 SITL Lockstep Synchronization

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Prerequisite Requirements:
  - [FR-00148-sitl-simulator-adapter-trait](FR-00148-sitl-simulator-adapter-trait.md)
  - [FR-00149-sitl-multi-adapter-registration](FR-00149-sitl-multi-adapter-registration.md)
- Dependent Requirements:
  - [NFR-00095-sitl-latency](NFR-00095-sitl-latency.md)
- Related Tasks:
  - [T-00160-sitl-multi-vehicle-lockstep-ci](../tasks/T-00160-sitl-multi-vehicle-lockstep-ci/README.md)

## Requirement Statement

SITL shall support lockstep time synchronization across all registered adapters. In lockstep mode, simulation time advances only when all adapters have completed their physics step and exchanged sensor/actuator data.

## Rationale

Lockstep synchronization enables:

1. Deterministic, reproducible test results
2. Identical behavior across runs with same inputs
3. Debugging with predictable timing
4. Comparison tests between simulators

## User Story (if applicable)

As a developer debugging a navigation issue, I want to replay the exact same scenario repeatedly with identical timing so that I can isolate the root cause.

## Acceptance Criteria

- [ ] `TimeMode::Lockstep` configuration option with configurable step size
- [ ] `TimeMode::FreeRunning` for real-time operation
- [ ] `TimeMode::Scaled` for faster/slower than real-time
- [ ] All adapters step together in lockstep mode
- [ ] Sensor data collected from all adapters before control loop runs
- [ ] Actuator commands distributed to all adapters after control loop
- [ ] Timeout handling if an adapter fails to respond
- [ ] Fallback to free-running if adapter doesn't support lockstep
- [ ] `sim_time_us()` returns consistent time across all vehicles
- [ ] Unit tests for time synchronization correctness

## Technical Details (if applicable)

### Time Mode Configuration

```rust
pub enum TimeMode {
    /// Simulators run at wall-clock time
    FreeRunning,

    /// All simulators step together, deterministic
    Lockstep {
        /// Simulation step size
        step_size_us: u64,  // default: 20000 (50 Hz)
    },

    /// Scaled real-time (faster or slower)
    Scaled {
        /// Time multiplier (2.0 = 2x speed)
        factor: f32,
    },
}
```

### Lockstep Protocol

```text
SitlBridge              Adapter1            Adapter2            Vehicles
    │                      │                   │                   │
    │── step_request() ───▶│                   │                   │
    │── step_request() ────────────────────────▶│                   │
    │                      │                   │                   │
    │                   physics              physics               │
    │                      │                   │                   │
    │◀── sensors_ready ────│                   │                   │
    │◀── sensors_ready ────────────────────────│                   │
    │                                                              │
    │ (all adapters ready)                                         │
    │                                                              │
    │── distribute_sensors ───────────────────────────────────────▶│
    │                                                              │
    │                                          control_loop()      │
    │                                                              │
    │◀── collect_actuators ────────────────────────────────────────│
    │                                                              │
    │── send_actuators ───▶│                   │                   │
    │── send_actuators ────────────────────────▶│                   │
    │                                                              │
    │ (advance sim_time by step_size)                              │
```

### Bridge Implementation

```rust
impl SitlBridge {
    pub async fn step_lockstep(&mut self) -> Result<(), SitlError> {
        // 1. Request all adapters to step
        for adapter in self.adapters.values_mut() {
            if adapter.supports_lockstep() {
                adapter.step().await?;
            }
        }

        // 2. Collect sensor data from all adapters
        let mut all_sensors = Vec::new();
        for adapter in self.adapters.values_mut() {
            while let Some(data) = adapter.receive_sensors().await? {
                all_sensors.push(data);
            }
        }

        // 3. Route sensor data to vehicles, run control loops
        for sensor_data in all_sensors {
            if let Some(vehicle) = self.vehicles.get_mut(&sensor_data.vehicle_id) {
                vehicle.inject_sensors(sensor_data);
                vehicle.run_control_loop();
            }
        }

        // 4. Collect actuator commands, send to adapters
        for (vehicle_id, vehicle) in &self.vehicles {
            let commands = vehicle.get_actuator_commands();
            let adapter_name = self.vehicle_adapter_map.get(vehicle_id).unwrap();
            let adapter = self.adapters.get_mut(adapter_name).unwrap();
            adapter.send_actuators(&commands).await?;
        }

        // 5. Advance simulation time
        self.sim_time_us += self.time_mode.step_size_us();

        Ok(())
    }
}
```

## Platform Considerations

### Cross-Platform

- Lockstep logic is pure async Rust
- Uses `tokio::select!` or similar for multi-adapter coordination
- Timeout via `tokio::time::timeout`

## Risks & Mitigation

| Risk                        | Impact | Likelihood | Mitigation                                     | Validation            |
| --------------------------- | ------ | ---------- | ---------------------------------------------- | --------------------- |
| Deadlock if adapter hangs   | High   | Medium     | Configurable timeout, fallback to free-running | Fault injection tests |
| Time drift between adapters | Medium | Low        | Single source of truth for sim_time            | Synchronization tests |
| Performance bottleneck      | Low    | Low        | Parallel adapter communication                 | Benchmark             |

## Implementation Notes

- Lockstep requires all adapters to support it; check `supports_lockstep()` at startup
- Consider adding a "soft lockstep" mode that tolerates missing adapters
- Log timing statistics for debugging synchronization issues

## External References

- [ArduPilot SITL Lockstep](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html#lockstep-mode)
