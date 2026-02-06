# T-00160 SITL Multi-Vehicle, Lockstep and CI Integration Design

## Metadata

- Type: Design
- Status: Implementation Complete

## Links

- Associated Plan Document:
  - [plan.md](plan.md)

## Overview

Extend the SITL Bridge with full multi-vehicle routing, lockstep time synchronization, and per-vehicle MAVLink ports. Then integrate the complete SITL system into CI with automated tests, examples, and documentation.

## Success Metrics

- [ ] Multi-vehicle support with VehicleId routing (10 vehicles tested)
- [ ] Lockstep produces deterministic results
- [ ] Per-vehicle MAVLink ports work
- [ ] <20ms p99 latency in lockstep mode
- [ ] CI runs `cargo test --features sitl` without Gazebo
- [ ] All rover modes work in SITL

## Background and Current State

- After T-00157: Core abstractions and bridge exist
- After T-00158: SitlPlatform and LightweightAdapter are functional
- After T-00159 (optional): GazeboAdapter available
- Bridge currently has basic step() placeholder and single-vehicle assumption
- Multi-vehicle routing and lockstep logic not yet implemented

## Proposed Design

### Component: TimeCoordinator

**File**: `crates/sitl/src/bridge/time.rs`

```rust
pub struct TimeCoordinator {
    mode: TimeMode,
    sim_time_us: u64,
}

impl TimeCoordinator {
    /// Advance time based on mode
    pub fn advance(&mut self, adapters: &[&dyn SimulatorAdapter]) -> u64 {
        match self.mode {
            TimeMode::Lockstep { step_size_us } => {
                // Wait for all adapters before advancing
                self.sim_time_us += step_size_us;
                self.sim_time_us
            }
            TimeMode::FreeRunning => {
                // Use wall-clock time
                ...
            }
            TimeMode::Scaled { factor } => {
                // Multiply wall-clock by factor
                ...
            }
        }
    }
}
```

### Multi-Vehicle Routing

```text
Sensor packet: { vehicle_id: 2, imu: {...}, gps: {...} }
                      │
                      ▼
              SitlBridge.step()
                      │
           ┌─────────┼─────────┐
           │         │         │
           ▼         ▼         ▼
       Vehicle 1  Vehicle 2  Vehicle 3
       (ignored)  (receives) (ignored)
```

### Per-Vehicle MAVLink Ports

- Port assignment: `14550 + vehicle_index`
- Each vehicle gets its own UDP socket for GCS connection
- Vehicle 0: port 14550, Vehicle 1: port 14551, etc.
- MAVLink messages routed by port

### Multi-Adapter Step Sequence

```text
SitlBridge.step()
    │
    ├── Parallel: adapter_1.step(), adapter_2.step(), ...
    │
    ├── Collect sensors from all adapters
    │
    ├── Route sensors by vehicle_id
    │       vehicle_1.platform.inject_sensors(data_1)
    │       vehicle_2.platform.inject_sensors(data_2)
    │
    ├── Run control loops (all vehicles)
    │
    ├── Collect actuator commands from all vehicles
    │       commands_1 = vehicle_1.platform.collect_actuator_commands()
    │       commands_2 = vehicle_2.platform.collect_actuator_commands()
    │
    ├── Send actuator commands to respective adapters
    │
    └── TimeCoordinator.advance()
```

### Performance Considerations

| Operation           | Target | Notes                   |
| ------------------- | ------ | ----------------------- |
| Sensor injection    | <1ms   | Mutex lock + memcpy     |
| Control loop        | <15ms  | Same as embedded target |
| Actuator collection | <1ms   | Mutex lock + clone      |
| Total step latency  | <20ms  | NFR-00095 requirement   |

## Testing Strategy

### Unit Tests

- Multi-vehicle spawn (10 vehicles)
- Sensor routing (correct vehicle receives data)
- Lockstep timing (all adapters step together)
- MAVLink port assignment
- TimeCoordinator modes

### Integration Tests

- 3 vehicles on LightweightAdapter
- Deterministic multi-vehicle scenario
- End-to-end Manual mode: RC input → motor output
- End-to-end Guided mode: Position target → navigation
- GCS connection via MAVLink

## External References

- [ADR-00156 SITL Pluggable Adapter Architecture](../../adr/ADR-00156-sitl-pluggable-adapter-architecture.md)
