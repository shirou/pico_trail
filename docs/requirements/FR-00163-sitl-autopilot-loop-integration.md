# FR-00163 SITL Autopilot Loop Integration

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00147-sitl-simulator-integration](../analysis/AN-00147-sitl-simulator-integration.md)
- Prerequisite Requirements:
  - [FR-00162-sitl-gcs-command-reception](FR-00162-sitl-gcs-command-reception.md)
  - [FR-00154-sitl-platform-trait](FR-00154-sitl-platform-trait.md)
- Dependent Requirements: N/A – Terminal requirement in the SITL chain
- Related Tasks:
  - [T-00164-sitl-autopilot-core-integration](../tasks/T-00164-sitl-autopilot-core-integration/README.md)
  - [T-00166-sitl-per-process-multi-vehicle](../tasks/T-00166-sitl-per-process-multi-vehicle/README.md)

## Requirement Statement

Each SITL vehicle instance shall integrate GCS command reception, sensor processing, and motor output into a closed-loop autopilot cycle, such that GCS commands produce observable vehicle motion in the simulator within a single simulation step.

## Rationale

Individual components (GCS link, sensor injection, actuator output) exist but are not connected into a control loop. Without closed-loop integration, GCS commands do not produce vehicle motion. This requirement ensures the full pipeline works end-to-end: GCS command → motor mixing → PWM → Gazebo actuators → vehicle motion → sensor update → telemetry → GCS display.

## User Story (if applicable)

As a developer, I want to arm the rover in Mission Planner and steer it with the joystick, seeing it move in Gazebo in real time, so that I can validate vehicle behavior before deploying to hardware.

## Acceptance Criteria

- [x] GCS RC_CHANNELS_OVERRIDE commands produce motor PWM changes within one simulation step
- [x] Motor PWM values flow through `collect_actuator_commands()` to the Gazebo adapter
- [x] Gazebo responds with updated sensor data reflecting vehicle motion
- [x] Updated sensor data is reflected in telemetry sent back to GCS
- [x] End-to-end latency (GCS command → telemetry update) is under 100ms at 100Hz step rate
- [x] Arming the vehicle enables motor output; disarming returns motors to neutral
- [ ] Multiple vehicles can be controlled independently from the same GCS (requires manual validation with Mission Planner + mavp2p)
- [x] Integration test demonstrates closed-loop control with LightweightAdapter

## Technical Details (if applicable)

### Closed-Loop Step Sequence

```text
1. gcs.poll_incoming()          → receive GCS commands
2. gcs.apply_commands(vehicle)  → update PWM duty cycles
3. bridge.step()                → Gazebo physics step
   a. send_actuators()          → PWM → Gazebo motors
   b. receive_sensors()         → Gazebo → SensorData
   c. inject_sensors(vehicle)   → update vehicle state
4. gcs.send_telemetry(vehicle)  → send updated state to GCS
```

### Data Flow

```text
Mission Planner ──RC_CHANNELS_OVERRIDE──→ GcsLink
                                           │
                                     motor mixing
                                     (DifferentialDrive)
                                           │
                                     PWM duty cycles
                                           │
                                    SitlPlatform ──→ GazeboAdapter
                                                        │
                                                   Gazebo physics
                                                        │
                                                   SensorData
                                                        │
                                    SitlPlatform ←── inject_sensors
                                           │
                                     telemetry build
                                           │
Mission Planner ←──ATTITUDE/GPS──── GcsLink
```

## Platform Considerations

### Host Only

- Runs on host with tokio runtime
- GcsLink uses TCP for MAVLink
- Gazebo adapter uses UDP for simulator communication

## Risks & Mitigation

| Risk                                 | Impact | Likelihood | Mitigation                                 | Validation           |
| ------------------------------------ | ------ | ---------- | ------------------------------------------ | -------------------- |
| Step ordering causes stale data      | Medium | Medium     | Document and enforce step sequence         | Integration test     |
| Multi-vehicle command routing errors | Medium | Low        | Match system_id in MAVLink headers         | Multi-vehicle test   |
| PWM duty cycle range mismatch        | High   | Medium     | Use existing PWM-to-normalized conversions | Unit test round-trip |

## Implementation Notes

- ADR-00161 migrates the full autopilot integration layer (MessageDispatcher, ModeManager, mode implementations) from firmware to core
- All rover modes (Manual, Auto, Guided, RTL, Loiter, Circle, SmartRTL) are in scope via the core migration
- The integration test should use LightweightAdapter (no Gazebo required for CI)

## External References

- [ArduPilot SITL Architecture](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
