# T-00005 AHRS DCM Implementation

## Metadata

- Type: Task
- Status: Phase 3 Complete

## Links

- Related Analyses:
  - [AN-00005-imu-sensor-selection](../../analysis/AN-00005-imu-sensor-selection.md)
- Related Requirements:
  - [FR-00001-ahrs-attitude-estimation](../../requirements/FR-00001-ahrs-attitude-estimation.md)
  - [NFR-00002-imu-sampling-rate](../../requirements/NFR-00002-imu-sampling-rate.md)
  - [NFR-00003-memory-limits](../../requirements/NFR-00003-memory-limits.md)
- Related ADRs:
  - [ADR-00001-ahrs-algorithm-selection](../../adr/ADR-00001-ahrs-algorithm-selection.md)
  - [ADR-00006-imu-driver-architecture](../../adr/ADR-00006-imu-driver-architecture.md)
- Associated Design Document:
  - [design.md](./design.md)
- Associated Plan Document:
  - [plan.md](./plan.md)

## Summary

Implement Direction Cosine Matrix (DCM) based Attitude and Heading Reference System (AHRS) for real-time attitude estimation. Fuses gyroscope, accelerometer, and magnetometer data to provide accurate roll, pitch, and heading estimates at 100Hz for autonomous navigation and control.

## Implementation Phases

1. **Phase 1**: Core DCM Algorithm (matrix operations, gyro integration, normalization, Euler extraction)
2. **Phase 2**: Sensor Fusion & Corrections (accelerometer/magnetometer PI corrections, calibration structures)
3. **Phase 3**: AHRS Task Integration (Embassy task, shared state, performance profiling, documentation)

## Success Criteria

- Roll and pitch accuracy within ±2 degrees during static conditions
- Heading accuracy within ±5 degrees with calibrated magnetometer
- AHRS update rate at 100Hz (< 10ms per cycle)
- Convergence to correct attitude within 5 seconds of startup
- Memory footprint < 2 KB RAM for DCM state
- Works on both Pico W (RP2040, no FPU) and Pico 2 W (RP2350, with FPU)

## Key Design Decisions

- **DCM over EKF**: Simpler implementation, lower memory (2 KB vs 8 KB), works efficiently without FPU
- **PI controller corrections**: Proportional-Integral feedback from accel/mag to correct gyro drift
- **100Hz update rate**: Consumes 400Hz IMU data, decimates magnetometer to 10Hz
- **Matrix normalization**: Gram-Schmidt orthonormalization prevents DCM matrix drift
- **Calibration persistence**: Accel/mag offsets and scales stored via parameter system
- **Graceful degradation**: Continue with gyro-only if accel/mag unavailable (quality flags)
- **Platform-specific tuning**: Lower PI gains on Pico W for stability with software float math

## Progress Tracking

See [Implementation Plan](plan.md) for detailed task checklists and phase status.

## Notes

- Follows ADR-00001: DCM selected as Phase 1 implementation; EKF deferred to Phase 2 for Pico 2 W
- ArduPilot DCM reference used for algorithm structure and tuning baselines
- Uses `nalgebra` (matrix ops) and `micromath` (fast trig approximations)
- Abstracts attitude interface (`AttitudeProvider` trait) for future EKF upgrade
- Gyro bias estimated during 1-second initialization period
- Handles gimbal lock at ±90° pitch via quaternion representation internally
- Compatible with RP2040 (Pico W) and RP2350 (Pico 2 W)
