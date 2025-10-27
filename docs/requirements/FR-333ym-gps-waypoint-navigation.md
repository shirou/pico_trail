# FR-333ym GPS Waypoint Navigation with S-Curve Path Planning

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
  - [FR-eyuh8-ahrs-attitude-estimation](FR-eyuh8-ahrs-attitude-estimation.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall support GPS-based autonomous waypoint navigation using S-curve path planning as the primary method, generating smooth trajectories that respect configurable velocity and acceleration limits.

## Rationale

S-curve path planning is the modern standard in ArduPilot 4.3+ for rovers and boats. Compared to legacy L1 controller, S-curves provide:

- **Smoother acceleration/deceleration**: Reduces mechanical stress, prevents wheel slip and propeller cavitation
- **Explicit velocity/acceleration limits**: Respects physical constraints (`SPEED_MAX`, `ATC_ACCEL_MAX`)
- **Better corner handling**: Automatically reduces speed before sharp turns
- **Continuous path**: No abrupt direction changes at waypoints

This results in safer, more comfortable, and more efficient autonomous operation.

## User Story (if applicable)

As a rover/boat operator, I want the vehicle to navigate smoothly between waypoints without abrupt speed changes or sharp turns, so that cargo remains stable, mechanical components experience less stress, and the vehicle operates safely within its physical limits.

## Acceptance Criteria

- [ ] System accepts mission waypoints via MAVLink protocol
- [ ] S-curve path planner generates smooth trajectories connecting waypoints
- [ ] Path following error remains below 2 meters at speeds up to 5 m/s
- [ ] Smooth acceleration and deceleration within configured limits (no abrupt changes)
- [ ] Speed automatically reduces before sharp corners (>90° turn)
- [ ] Vehicle transitions through waypoints without stopping (continuous path)
- [ ] Waypoint acceptance radius configurable (default 2 meters)
- [ ] Compatible with QGroundControl and Mission Planner waypoint format

## Technical Details (if applicable)

### Functional Requirement Details

**S-Curve Path Planning Algorithm:**

1. **Path Generation**: Calculate smooth curve connecting consecutive waypoints
2. **Velocity Profile**: Determine speed along path respecting `SPEED_MAX` and `ATC_ACCEL_MAX`
3. **Trajectory Output**: Generate position + velocity targets at each control timestep (50Hz)
4. **Position Control**: Calculate steering/throttle commands to follow trajectory

**Navigation Parameters:**

```
WP_SPEED: Maximum speed (m/s), default 2.0
WP_ACCEL: Maximum acceleration (m/s²), default 1.0
WP_RADIUS: Waypoint acceptance radius (m), default 2.0
WP_OVERSHOOT: Maximum overshoot distance (m), default 2.0
WP_SCURVE_TC: S-curve time constant (s), default 0.5
WP_SCURVE_JERK: Maximum jerk limit (m/s³), default 1.0
```

**Inputs:**

- Mission waypoints: List of latitude/longitude/altitude points
- Current GPS position: Latitude, longitude, altitude
- Current velocity: North/East velocity from GPS
- Vehicle heading: From AHRS

**Outputs:**

- Desired position: Target latitude/longitude
- Desired velocity: Target speed and direction
- Steering command: Calculated by position controller
- Throttle command: Calculated by speed controller

**Error Handling:**

- If GPS fix lost, trigger failsafe (transition to Hold or RTL mode)
- If waypoint unreachable (beyond maximum distance), reject mission upload
- If path following error exceeds threshold, slow down or trigger failsafe

## Platform Considerations

### Pico W (RP2040)

S-curve calculations may be CPU-intensive without FPU. May need to optimize with lookup tables or reduce update rate to 10Hz.

### Pico 2 W (RP2350)

Hardware FPU accelerates trajectory calculations. Can comfortably run S-curve planner at 50Hz.

### Cross-Platform

Navigation algorithm should be platform-independent. Use platform abstraction for GPS and control interfaces.

## Risks & Mitigation

| Risk                                         | Impact | Likelihood | Mitigation                                                 | Validation                                  |
| -------------------------------------------- | ------ | ---------- | ---------------------------------------------------------- | ------------------------------------------- |
| S-curve too CPU-intensive for Pico W         | Medium | Medium     | Benchmark on Pico W, optimize or reduce update rate        | Profile CPU usage during S-curve generation |
| Path following error too large at high speed | Medium | Medium     | Tune position controller gains, reduce max speed if needed | Test at various speeds (1-5 m/s)            |
| GPS accuracy insufficient (<2m error)        | High   | Low        | Increase waypoint acceptance radius, use GPS with SBAS     | Test with consumer GPS (u-blox NEO-M8N)     |
| Vehicle overshoots waypoints                 | Low    | Medium     | Tune S-curve parameters, reduce speed before waypoints     | Measure overshoot distance in field tests   |

## Implementation Notes

Preferred approaches:

- Implement S-curve as primary method (ArduPilot 4.3+ standard)
- Use ArduPilot-compatible parameter naming for GCS compatibility
- Implement position controller (converts trajectory to steering/throttle)

Known pitfalls:

- GPS latency (100-200ms) requires prediction of future position
- GPS accuracy degrades at low speeds (use dead reckoning from IMU)
- Magnetic heading unreliable when stationary (use GPS velocity for heading when moving)

Related code areas:

- `src/subsystems/navigation/scurve.rs` - S-curve path planner
- `src/subsystems/control/position.rs` - Position controller
- `src/devices/gps/` - GPS device drivers
- `src/vehicle/modes/auto.rs` - Auto mode using navigation

Suggested libraries:

- `nalgebra` for vector math (waypoint calculations)
- `libm` for transcendental functions (sin, cos, atan2)

## External References

- ArduPilot S-Curve Documentation: <https://ardupilot.org/rover/docs/rover-tuning-navigation.html>
- ArduPilot AR_WPNav Library: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AR_WPNav>
- ArduPilot AR_PosControl Library: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AR_PosControl>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
