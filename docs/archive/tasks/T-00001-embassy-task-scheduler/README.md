# T-00001: Embassy Task Scheduler

## Status

**Draft** - Task package created, awaiting approval to begin implementation

## Quick Links

- **Design**: [design.md](design.md) - Architecture and component design
- **Plan**: [plan.md](plan.md) - Implementation phases and verification
- **Requirement**: [FR-00007-task-scheduler](../../../requirements/FR-00007-task-scheduler.md)
- **ADR**: [ADR-00005-task-scheduler-selection](../../../adr/ADR-00005-task-scheduler-selection.md)

## Overview

Implement Embassy-based task scheduler for periodic task execution at configurable rates (1Hz-400Hz) with real-time performance guarantees. This is the foundational infrastructure for all autopilot control loops, sensor sampling, and telemetry.

## Key Objectives

- Execute tasks at target rates within 5% period deviation
- Provide task execution time monitoring and CPU load tracking
- Meet real-time requirements: 400Hz IMU sampling, ≤20ms control loop latency
- Maintain scheduler overhead < 5% CPU
- Zero deadline misses under 75% CPU load

## Implementation Phases

1. **Phase 1**: Core scheduler infrastructure (types, registry, statistics)
2. **Phase 2**: Task execution and monitoring (Embassy tasks, CPU load calculation)
3. **Phase 3**: Hardware validation and optimization (deploy, benchmark, document)

## Dependencies

- **Upstream**:
  - FR-00007-task-scheduler (Approved)
  - ADR-00005-task-scheduler-selection (Approved)
  - T-00004-platform-abstraction (Complete - provides timer interface)
- **Downstream**:
  - Future tasks: IMU driver, AHRS, control loops, telemetry (blocked until scheduler complete)

## Success Metrics

- Tasks execute within 5% of target period over 10-second window
- Control loop latency ≤ 20ms (NFR-00001)
- IMU sampling rate = 400Hz ± 5% (NFR-00002)
- Scheduler overhead < 5% CPU
- Zero deadline misses under 75% CPU load

## Notes

- Embassy async framework selected over RTIC for development velocity and async I/O support
- Hardware validation required on both Pico W (RP2040) and Pico 2 W (RP2350)
- Performance profiling is critical - if Embassy overhead exceeds targets, may need to revisit RTIC

---

**Last Updated**: 2025-10-28
**Task Owner**: (TBD)
