# T-49k7n AHRS DCM Implementation

## Overview

Implement Direction Cosine Matrix (DCM) based Attitude and Heading Reference System (AHRS) for real-time attitude estimation. Fuses gyroscope, accelerometer, and magnetometer data to provide accurate roll, pitch, and heading estimates at 100Hz for autonomous navigation and control.

## Status

- **Current Phase**: Draft
- **Last Updated**: 2025-10-30

## Quick Links

- [Design Document](design.md) - Architecture and technical design
- [Implementation Plan](plan.md) - Phased implementation tasks

## Related Artifacts

### Requirements

- [FR-eyuh8-ahrs-attitude-estimation](../../requirements/FR-eyuh8-ahrs-attitude-estimation.md) - AHRS attitude estimation requirements
- [NFR-3wlo1-imu-sampling-rate](../../requirements/NFR-3wlo1-imu-sampling-rate.md) - IMU sampling rate requirements
- [NFR-z2iuk-memory-limits](../../requirements/NFR-z2iuk-memory-limits.md) - Memory constraints

### Architecture Decisions

- [ADR-6twis-ahrs-algorithm-selection](../../adr/ADR-6twis-ahrs-algorithm-selection.md) - DCM algorithm selection over EKF and complementary filters

### Dependencies

- **Upstream**: T-g729p (Embassy Task Scheduler - provides 100Hz execution slot), T-ex2h7 (Parameter Persistence - stores calibration data)
- **Downstream**: FR-333ym (GPS Waypoint Navigation - consumes attitude estimates)

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

- Follows ADR-6twis: DCM selected as Phase 1 implementation; EKF deferred to Phase 2 for Pico 2 W
- ArduPilot DCM reference used for algorithm structure and tuning baselines
- Uses `nalgebra` (matrix ops) and `micromath` (fast trig approximations)
- Abstracts attitude interface (`AttitudeProvider` trait) for future EKF upgrade
- Gyro bias estimated during 1-second initialization period
- Handles gimbal lock at ±90° pitch via quaternion representation internally
- Compatible with RP2040 (Pico W) and RP2350 (Pico 2 W)
