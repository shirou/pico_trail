# FR-eyuh8 AHRS Attitude Estimation

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Prerequisite Requirements:
  - [FR-5inw2-task-scheduler](FR-5inw2-task-scheduler.md)
  - [NFR-3wlo1-imu-sampling-rate](NFR-3wlo1-imu-sampling-rate.md)
- Dependent Requirements:
  - [FR-333ym-gps-waypoint-navigation](FR-333ym-gps-waypoint-navigation.md)
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

The system shall implement an Attitude and Heading Reference System (AHRS) using either Direction Cosine Matrix (DCM) or simplified Extended Kalman Filter (EKF) for attitude estimation, fusing data from gyroscope, accelerometer, and magnetometer sensors.

## Rationale

Accurate attitude estimation is fundamental for navigation and control. AHRS provides the vehicle's orientation (roll, pitch, heading) by fusing multiple sensors to compensate for individual sensor weaknesses:

- **Gyroscope**: Accurate short-term rotation, but drifts over time
- **Accelerometer**: Provides gravity reference for roll/pitch, but noisy during motion
- **Magnetometer**: Provides heading reference, but susceptible to magnetic interference

ArduPilot uses either DCM (lightweight, proven) or EKF (more accurate but memory-intensive). For embedded constraints, DCM or a 6-9 state EKF is appropriate.

## User Story (if applicable)

As an autopilot system, I want to accurately estimate the vehicle's attitude (roll, pitch, heading) in real-time, so that navigation and control algorithms can make correct steering and throttle decisions.

## Acceptance Criteria

- [ ] Roll and pitch accuracy within ±2 degrees during static conditions
- [ ] Heading accuracy within ±5 degrees with calibrated magnetometer
- [ ] AHRS update rate at 100Hz minimum (consumes IMU data sampled at 400Hz)
- [ ] Convergence to correct attitude within 5 seconds of system startup
- [ ] Support for magnetometer calibration (offsets and scaling)
- [ ] Support for accelerometer calibration (offsets and scaling)
- [ ] Graceful degradation when magnetometer unavailable (gyro drift warning)
- [ ] Compatible with common IMU sensors (MPU6050, MPU9250, BMI088)

## Technical Details (if applicable)

### Functional Requirement Details

**Algorithm Options:**

1. **DCM (Direction Cosine Matrix)**:
   - Memory: \~2 KB RAM
   - CPU: Low-moderate
   - Accuracy: ±2-3 degrees roll/pitch, ±5-10 degrees heading
   - Best for: Pico W (limited resources)

2. **Simplified EKF (6-9 states)**:
   - Memory: \~8 KB RAM
   - CPU: Moderate
   - Accuracy: ±1-2 degrees roll/pitch, ±3-5 degrees heading
   - Best for: Pico 2 W (more resources, FPU available)

3. **Complementary Filter**:
   - Memory: \~1 KB RAM
   - CPU: Very low
   - Accuracy: ±3-5 degrees roll/pitch, heading from mag only
   - Best for: Fallback/debugging

**Sensor Fusion:**

- Gyroscope integration for short-term attitude
- Accelerometer for long-term roll/pitch correction
- Magnetometer for heading correction
- GPS velocity for heading aid (when moving)

**Inputs:**

- IMU data: 3-axis gyro (rad/s), 3-axis accel (m/s²), 3-axis mag (μT)
- GPS velocity: North/East velocity (m/s) for heading aid
- Update rate: 100Hz

**Outputs:**

- Euler angles: roll, pitch, yaw (radians or degrees)
- Rotation matrix or quaternion representation
- Angular rates: roll rate, pitch rate, yaw rate (rad/s)
- Confidence/quality indicators

**Calibration:**

- Accelerometer: 6-position calibration (±X, ±Y, ±Z)
- Magnetometer: Sphere fitting or ellipsoid calibration
- Gyroscope: Bias estimation during initialization

## Platform Considerations

### Pico W (RP2040)

No FPU - Use DCM or optimize EKF with fixed-point math where possible. May need to reduce AHRS update rate to 50Hz if CPU-bound.

### Pico 2 W (RP2350)

Hardware FPU available - Can use full EKF at 100Hz with minimal CPU impact. FPU accelerates matrix operations and trigonometric functions.

### Cross-Platform

AHRS implementation should abstract platform differences. Use conditional compilation for FPU vs soft-float paths.

## Risks & Mitigation

| Risk                                     | Impact | Likelihood | Mitigation                                                  | Validation                                       |
| ---------------------------------------- | ------ | ---------- | ----------------------------------------------------------- | ------------------------------------------------ |
| AHRS drift without magnetometer          | Medium | Medium     | Require magnetometer for heading, implement mag calibration | Test with mag disabled, verify drift warning     |
| IMU noise causes attitude oscillation    | Medium | Medium     | Tune filter gains, implement outlier rejection              | Record IMU data during motion, analyze in MAVLog |
| CPU insufficient for 100Hz EKF on Pico W | Medium | Medium     | Use DCM on Pico W, EKF on Pico 2 W, or reduce to 50Hz       | Profile CPU usage on both platforms              |
| Magnetic interference corrupts heading   | High   | Medium     | Implement mag health monitoring, fallback to GPS heading    | Test near motors/batteries, validate detection   |

## Implementation Notes

Preferred approaches:

- Use **DCM** for initial implementation (simpler, proven, lower resource usage)
- Add **EKF** as optional upgrade for Pico 2 W after DCM validated
- Implement as compile-time feature flag: `features = ["ahrs-dcm"]` or `features = ["ahrs-ekf"]`

Known pitfalls:

- Magnetometer must be calibrated before use (uncalibrated mag worse than no mag)
- Accelerometer only accurate for roll/pitch when vehicle is not accelerating
- Gyro bias drifts with temperature - re-estimate periodically
- Euler angles have gimbal lock at ±90° pitch - use quaternions internally

Related code areas:

- `src/subsystems/ahrs/` - AHRS implementation
- `src/devices/imu/` - IMU device drivers
- `src/devices/magnetometer/` - Magnetometer drivers
- `src/core/calibration/` - Sensor calibration procedures

Suggested libraries:

- `nalgebra` (with `no_std`) for matrix operations
- `micromath` for fast approximations of sin/cos/atan2

## External References

- ArduPilot DCM Library: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_AHRS>
- DCM Tutorial: <https://www.instructables.com/id/Guide-to-gyro-and-accelerometer-with-Arduino-incl/>
- Madgwick Filter Paper: <http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/>

---

## Template Usage

For detailed instructions, see [Template Usage Instructions](../templates/README.md#individual-requirement-template-requirementsmd) in the templates README.
