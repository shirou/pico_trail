# AHRS (Attitude and Heading Reference System)

## Overview

The AHRS subsystem provides real-time attitude estimation (roll, pitch, yaw) by fusing data from the IMU (gyroscope, accelerometer, magnetometer) using a Direction Cosine Matrix (DCM) algorithm.

## Requirements

- **FR-00001**: AHRS Attitude Estimation
- **ADR-00001**: DCM Algorithm Selection
- **NFR-00002**: IMU Sampling Rate (400Hz)

## Architecture

### Components

- **`dcm.rs`**: Core DCM algorithm with gyro integration and PI controller
- **`calibration.rs`**: IMU calibration data structures and persistence
- **`state.rs`**: Thread-safe shared attitude state
- **`task.rs`**: Embassy async task for 100Hz execution

### Data Flow

```
IMU Sensors (400Hz)
    ↓
AHRS Task (100Hz decimation)
    ↓
DCM Algorithm
    ↓
SharedAttitudeState
    ↓
Navigation/Control Subsystems
```

## Usage

### Basic Usage

```rust
use pico_trail::subsystems::ahrs::{
    Dcm, DcmConfig, CalibrationData, SharedAttitudeState
};
use nalgebra::Vector3;

// Create shared state
static ATTITUDE: SharedAttitudeState = SharedAttitudeState::new();

// Initialize DCM
let config = DcmConfig::default();
let calibration = CalibrationData::default();
let mut dcm = Dcm::new(config);
dcm.set_gyro_bias(calibration.gyro_bias);

// Main loop (100Hz)
loop {
    // Read sensors
    let gyro = Vector3::new(gx, gy, gz);  // rad/s
    let accel = Vector3::new(ax, ay, az); // m/s²
    let dt = 0.01; // 10ms = 100Hz

    // Update DCM
    let gyro_cal = gyro - calibration.gyro_bias;
    let accel_cal = calibration.apply_accel_calibration(accel);
    dcm.update(gyro_cal, accel_cal, dt);

    // Publish attitude
    let (roll, pitch, yaw) = dcm.get_euler_angles();
    ATTITUDE.update_attitude(roll, pitch, yaw, current_time_ms);
}
```

### Embassy Task Integration

```rust
use pico_trail::subsystems::ahrs::{run_ahrs_task, AhrsTaskConfig};

#[embassy_executor::task]
async fn ahrs_task() {
    let config = AhrsTaskConfig::default();
    let calibration = load_calibration_from_params();

    run_ahrs_task(&ATTITUDE, config, calibration, || async {
        // Read IMU asynchronously
        read_imu().await
    }).await
}
```

### Reading Attitude from Other Subsystems

```rust
// Read full state
let state = ATTITUDE.read();
println!("Roll: {:.2}°, Pitch: {:.2}°, Yaw: {:.2}°",
    state.roll_deg(), state.pitch_deg(), state.yaw_deg());

// Or read individual values
let roll = ATTITUDE.get_roll();
let pitch = ATTITUDE.get_pitch();
let yaw = ATTITUDE.get_yaw();
```

## Calibration

Proper calibration is critical for accurate attitude estimation. Perform calibration before first flight and periodically thereafter.

### Gyroscope Bias Calibration

Gyro bias is estimated automatically during initialization:

```rust
use pico_trail::subsystems::ahrs::estimate_gyro_bias;

// Collect 100 samples at 100Hz (1 second) while stationary
let mut samples = Vec::new();
for _ in 0..100 {
    samples.push(read_gyro());
    Timer::after(Duration::from_millis(10)).await;
}

let bias = estimate_gyro_bias(&samples);
calibration.gyro_bias = bias;
```

**Requirements**:

- Vehicle must be completely stationary
- Place on stable surface (avoid vibration)
- Duration: 1-2 seconds minimum
- Expected bias magnitude: < 0.05 rad/s (< 3°/s)

### Accelerometer Calibration (6-Position Method)

Accelerometer calibration corrects for mounting offsets and scale errors.

**Procedure**:

1. Place vehicle level (Z-axis up) → record 100 samples
2. Rotate 180° (Z-axis down) → record 100 samples
3. Right side down (X-axis up) → record 100 samples
4. Left side down (X-axis down) → record 100 samples
5. Nose up (Y-axis up) → record 100 samples
6. Nose down (Y-axis down) → record 100 samples

**Computation** (least squares fit):

```rust
// For each axis, solve for offset and scale:
// measured = (actual - offset) * scale
// where actual = ±9.81 m/s² (gravity)

let accel_offset = Vector3::new(
    (avg_x_up + avg_x_down) / 2.0,
    (avg_y_up + avg_y_down) / 2.0,
    (avg_z_up + avg_z_down) / 2.0,
);

let accel_scale = Vector3::new(
    (GRAVITY * 2.0) / (avg_x_up - avg_x_down),
    (GRAVITY * 2.0) / (avg_y_up - avg_y_down),
    (GRAVITY * 2.0) / (avg_z_up - avg_z_down),
);
```

**Expected values**:

- Offset: < 0.5 m/s²
- Scale: 0.95 to 1.05 (ideally 1.0)

### Magnetometer Calibration (Sphere Fitting)

Magnetometer calibration corrects for hard iron (constant offsets) and soft iron (scale/rotation) distortions.

**Procedure**:

1. Rotate vehicle slowly through all orientations
2. Record 200-500 samples covering full sphere
3. Fit sphere to data (find center and radius)

**Simple hard iron calibration** (center of sphere):

```rust
let mag_offset = Vector3::new(
    (mag_x_max + mag_x_min) / 2.0,
    (mag_y_max + mag_y_min) / 2.0,
    (mag_z_max + mag_z_min) / 2.0,
);

let mag_scale = Vector3::new(
    2.0 * avg_radius / (mag_x_max - mag_x_min),
    2.0 * avg_radius / (mag_y_max - mag_y_min),
    2.0 * avg_radius / (mag_z_max - mag_z_min),
);
```

**Expected values**:

- Field strength: 20-60 µT (location dependent)
- Offset: -50 to +50 µT
- Scale: 0.8 to 1.2

### Saving Calibration

```rust
// Save to parameter registry
calibration.save_to_params(&mut registry)?;
registry.save_to_flash()?;

// Load on startup
let calibration = CalibrationData::load_from_params(&registry);
```

## Tuning Parameters

### PI Controller Gains

The DCM algorithm uses PI controllers to correct gyro drift using accelerometer and magnetometer.

```rust
let config = DcmConfig {
    kp_roll_pitch: 0.2,      // Proportional gain for roll/pitch
    ki_roll_pitch: 0.0005,   // Integral gain for roll/pitch
    kp_yaw: 1.0,             // Proportional gain for yaw
    ki_yaw: 0.00005,         // Integral gain for yaw
};
```

**Effects of increasing gains**:

- **P gain**: Faster convergence, more responsive to corrections
  - Too high: Oscillation, noise sensitivity
  - Too low: Slow convergence, drift
- **I gain**: Removes steady-state drift
  - Too high: Integral wind-up, overshoot
  - Too low: Residual drift remains

**Tuning procedure**:

1. Start with default values
2. Adjust P gains first for desired responsiveness
3. Adjust I gains to eliminate long-term drift
4. Test in flight conditions (vibration, dynamics)

**Typical values**:

- Roll/pitch kp: 0.1 to 0.5
- Roll/pitch ki: 0.0001 to 0.001
- Yaw kp: 0.5 to 2.0
- Yaw ki: 0.00001 to 0.0001

## Performance

### Timing Budget

- **Update rate**: 100Hz (10ms period)
- **Target latency**: < 8ms per cycle
- **Pico W (RP2040)**: \~6-8ms (software FPU)
- **Pico 2 W (RP2350)**: \~3-5ms (hardware FPU)

### Memory Usage

- **DCM state**: \~200 bytes (Matrix3, vectors)
- **Calibration data**: \~100 bytes
- **Shared state**: \~50 bytes
- **Total RAM**: < 2 KB

### Accuracy

- **Roll/pitch**: ±2° during static conditions
- **Yaw (with mag)**: ±5° with calibrated magnetometer
- **Convergence time**: 5 seconds from startup
- **Drift rate**: < 1°/minute (with proper calibration)

## Troubleshooting

### High Roll/Pitch Error

**Symptoms**: Attitude drifts during static conditions

**Causes**:

- Poor accelerometer calibration
- Vibration interference
- kp_roll_pitch too low

**Solutions**:

- Re-run 6-position accel calibration
- Add vibration damping to IMU
- Increase kp_roll_pitch to 0.3-0.4

### High Yaw Error

**Symptoms**: Heading drifts or jumps

**Causes**:

- Poor magnetometer calibration
- Magnetic interference (motors, wires)
- kp_yaw too low

**Solutions**:

- Re-run magnetometer sphere fitting
- Move mag sensor away from interference
- Increase kp_yaw to 1.5-2.0

### Oscillation

**Symptoms**: Attitude oscillates around true value

**Causes**:

- P gains too high
- I gains too high (wind-up)
- Noise in sensor readings

**Solutions**:

- Reduce P gains by 50%
- Reduce I gains by 50%
- Add low-pass filter to sensors

### Slow Convergence

**Symptoms**: Takes > 10 seconds to stabilize

**Causes**:

- P gains too low
- Poor initial conditions

**Solutions**:

- Increase P gains by 50%
- Initialize DCM from accel/mag reading

## References

- **ADR-00001**: AHRS Algorithm Selection
- **FR-00001**: AHRS Attitude Estimation Requirements
- **DCM Tutorial**: <http://www.starlino.com/dcm_tutorial.html>
- **ArduPilot DCM**: <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_AHRS>
