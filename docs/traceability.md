# pico_trail Traceability Overview

Generated on 2025-11-01T07:53:15.434Z

## Summary

| Metric | Count |
| --- | ---: |
| Analyses | 6 |
| Requirements | 20 |
| ADRs | 9 |
| Tasks | 2 |
| Requirements with tasks | 0 (0%) |

## Traceability Matrix

| Analyses | ADRs | Requirement | Status | Tasks |
| --- | --- | --- | --- | --- |
| — | — | [FR-333ym](requirements/FR-333ym-gps-waypoint-navigation.md) - FR-333ym GPS Waypoint Navigation with S-Curve Path Planning | Approved | — |
| — | [ADR-hj79f](adr/ADR-hj79f-storage-strategy.md) | [FR-4e922](requirements/FR-4e922-data-logging.md) - FR-4e922 Data Logging to Flash Storage | Approved | — |
| — | [ADR-vywkw](adr/ADR-vywkw-task-scheduler-selection.md) | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md) - FR-5inw2 Task Scheduler with Configurable Rates | Approved | — |
| [AN-808o3](analysis/AN-808o3-mavlink-network-transport.md) | [ADR-ckv8z](adr/ADR-ckv8z-transport-abstraction.md) | [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md) - FR-6jkia MAVLink Transport Abstraction | Approved | — |
| — | [ADR-hj79f](adr/ADR-hj79f-storage-strategy.md)<br>[ADR-ggou4](adr/ADR-ggou4-mavlink-implementation.md) | [FR-a1cuu](requirements/FR-a1cuu-runtime-parameters.md) - FR-a1cuu Runtime Parameter Configuration | Approved | — |
| [AN-808o3](analysis/AN-808o3-mavlink-network-transport.md) | [ADR-dxdj0](adr/ADR-dxdj0-wifi-config-strategy.md) | [FR-dxvrs](requirements/FR-dxvrs-wifi-configuration.md) - FR-dxvrs WiFi Configuration | Approved | — |
| [AN-808o3](analysis/AN-808o3-mavlink-network-transport.md) | [ADR-ckv8z](adr/ADR-ckv8z-transport-abstraction.md) | [FR-eutkf](requirements/FR-eutkf-concurrent-transports.md) - FR-eutkf Concurrent MAVLink Transports | Approved | — |
| [AN-yhnjd](analysis/AN-yhnjd-imu-sensor-selection.md) | [ADR-6twis](adr/ADR-6twis-ahrs-algorithm-selection.md)<br>[ADR-wjcrn](adr/ADR-wjcrn-imu-driver-architecture.md) | [FR-eyuh8](requirements/FR-eyuh8-ahrs-attitude-estimation.md) - FR-eyuh8 AHRS Attitude Estimation | Approved | — |
| [AN-808o3](analysis/AN-808o3-mavlink-network-transport.md) | [ADR-ggou4](adr/ADR-ggou4-mavlink-implementation.md) | [FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md) - FR-gpzpz MAVLink Protocol Communication | Approved | — |
| — | — | [FR-sp3at](requirements/FR-sp3at-vehicle-modes.md) - FR-sp3at Vehicle Operational Modes | Approved | — |
| — | — | [FR-sxsvw](requirements/FR-sxsvw-failsafe-mechanisms.md) - FR-sxsvw Failsafe Mechanisms | Approved | — |
| [AN-808o3](analysis/AN-808o3-mavlink-network-transport.md) | [ADR-aul2v](adr/ADR-aul2v-udp-primary-transport.md) | [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) - FR-ydttj UDP Network Transport | Approved | — |
| [AN-yhnjd](analysis/AN-yhnjd-imu-sensor-selection.md) | [ADR-vywkw](adr/ADR-vywkw-task-scheduler-selection.md)<br>[ADR-6twis](adr/ADR-6twis-ahrs-algorithm-selection.md)<br>[ADR-wjcrn](adr/ADR-wjcrn-imu-driver-architecture.md) | [NFR-3wlo1](requirements/NFR-3wlo1-imu-sampling-rate.md) - NFR-3wlo1 IMU Sampling Rate | Approved | — |
| [AN-808o3](analysis/AN-808o3-mavlink-network-transport.md) | [ADR-aul2v](adr/ADR-aul2v-udp-primary-transport.md) | [NFR-eddfs](requirements/NFR-eddfs-network-latency-limits.md) - NFR-eddfs Network Transport Latency Limits | Approved | — |
| — | [ADR-oa2qa](adr/ADR-oa2qa-platform-abstraction.md) | [NFR-nmmu0](requirements/NFR-nmmu0-platform-code-isolation.md) - NFR-nmmu0 Platform Code Isolation | Approved | — |
| — | [ADR-oa2qa](adr/ADR-oa2qa-platform-abstraction.md) | [NFR-pj11s](requirements/NFR-pj11s-no-unsafe-rust.md) - NFR-pj11s Memory Safety - No Unsafe Rust | Approved | — |
| — | [ADR-vywkw](adr/ADR-vywkw-task-scheduler-selection.md) | [NFR-ukjvr](requirements/NFR-ukjvr-control-loop-latency.md) - NFR-ukjvr Control Loop Latency | Approved | — |
| [AN-808o3](analysis/AN-808o3-mavlink-network-transport.md) | [ADR-aul2v](adr/ADR-aul2v-udp-primary-transport.md) | [NFR-ukx3a](requirements/NFR-ukx3a-network-memory-overhead.md) - NFR-ukx3a Network Transport Memory Overhead | Approved | — |
| [AN-808o3](analysis/AN-808o3-mavlink-network-transport.md) | [ADR-ckv8z](adr/ADR-ckv8z-transport-abstraction.md) | [NFR-ypgpm](requirements/NFR-ypgpm-transport-independence.md) - NFR-ypgpm Transport Independence and Reliability | Approved | — |
| — | [ADR-6twis](adr/ADR-6twis-ahrs-algorithm-selection.md)<br>[ADR-hj79f](adr/ADR-hj79f-storage-strategy.md)<br>[ADR-ggou4](adr/ADR-ggou4-mavlink-implementation.md) | [NFR-z2iuk](requirements/NFR-z2iuk-memory-limits.md) - NFR-z2iuk Memory Usage Limits | Approved | — |

### Requirement Dependencies

| Requirement | Depends On | Blocks | Blocked By |
| --- | --- | --- | --- |
| [FR-333ym](requirements/FR-333ym-gps-waypoint-navigation.md) - FR-333ym GPS Waypoint Navigation with S-Curve Path Planning | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md)<br>[FR-eyuh8](requirements/FR-eyuh8-ahrs-attitude-estimation.md) | [FR-sp3at](requirements/FR-sp3at-vehicle-modes.md) | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md)<br>[FR-eyuh8](requirements/FR-eyuh8-ahrs-attitude-estimation.md) |
| [FR-4e922](requirements/FR-4e922-data-logging.md) - FR-4e922 Data Logging to Flash Storage | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md)<br>[FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md) | — | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md)<br>[FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md) |
| [FR-5inw2](requirements/FR-5inw2-task-scheduler.md) - FR-5inw2 Task Scheduler with Configurable Rates | — | [FR-333ym](requirements/FR-333ym-gps-waypoint-navigation.md)<br>[FR-4e922](requirements/FR-4e922-data-logging.md)<br>[FR-eyuh8](requirements/FR-eyuh8-ahrs-attitude-estimation.md)<br>[FR-sp3at](requirements/FR-sp3at-vehicle-modes.md)<br>[FR-sxsvw](requirements/FR-sxsvw-failsafe-mechanisms.md)<br>[NFR-3wlo1](requirements/NFR-3wlo1-imu-sampling-rate.md)<br>[NFR-ukjvr](requirements/NFR-ukjvr-control-loop-latency.md) | — |
| [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md) - FR-6jkia MAVLink Transport Abstraction | [FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md)<br>[FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) (inferred) | [FR-eutkf](requirements/FR-eutkf-concurrent-transports.md)<br>[FR-ydttj](requirements/FR-ydttj-udp-network-transport.md)<br>[NFR-ukx3a](requirements/NFR-ukx3a-network-memory-overhead.md) (inferred)<br>[NFR-ypgpm](requirements/NFR-ypgpm-transport-independence.md) (inferred) | [FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md)<br>[FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) (inferred) |
| [FR-a1cuu](requirements/FR-a1cuu-runtime-parameters.md) - FR-a1cuu Runtime Parameter Configuration | [FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md) | — | [FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md) |
| [FR-dxvrs](requirements/FR-dxvrs-wifi-configuration.md) - FR-dxvrs WiFi Configuration | [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) (inferred) | [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) | [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) (inferred) |
| [FR-eutkf](requirements/FR-eutkf-concurrent-transports.md) - FR-eutkf Concurrent MAVLink Transports | [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md)<br>[FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md)<br>[FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) | [NFR-ypgpm](requirements/NFR-ypgpm-transport-independence.md) (inferred) | [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md)<br>[FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md)<br>[FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) |
| [FR-eyuh8](requirements/FR-eyuh8-ahrs-attitude-estimation.md) - FR-eyuh8 AHRS Attitude Estimation | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md)<br>[NFR-3wlo1](requirements/NFR-3wlo1-imu-sampling-rate.md) | [FR-333ym](requirements/FR-333ym-gps-waypoint-navigation.md) | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md)<br>[NFR-3wlo1](requirements/NFR-3wlo1-imu-sampling-rate.md) |
| [FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md) - FR-gpzpz MAVLink Protocol Communication | — | [FR-4e922](requirements/FR-4e922-data-logging.md)<br>[FR-6jkia](requirements/FR-6jkia-transport-abstraction.md) (inferred)<br>[FR-a1cuu](requirements/FR-a1cuu-runtime-parameters.md)<br>[FR-eutkf](requirements/FR-eutkf-concurrent-transports.md) (inferred)<br>[FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) (inferred) | — |
| [FR-sp3at](requirements/FR-sp3at-vehicle-modes.md) - FR-sp3at Vehicle Operational Modes | [FR-333ym](requirements/FR-333ym-gps-waypoint-navigation.md)<br>[FR-5inw2](requirements/FR-5inw2-task-scheduler.md) | [FR-sxsvw](requirements/FR-sxsvw-failsafe-mechanisms.md) | [FR-333ym](requirements/FR-333ym-gps-waypoint-navigation.md)<br>[FR-5inw2](requirements/FR-5inw2-task-scheduler.md) |
| [FR-sxsvw](requirements/FR-sxsvw-failsafe-mechanisms.md) - FR-sxsvw Failsafe Mechanisms | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md)<br>[FR-sp3at](requirements/FR-sp3at-vehicle-modes.md) | — | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md)<br>[FR-sp3at](requirements/FR-sp3at-vehicle-modes.md) |
| [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) - FR-ydttj UDP Network Transport | [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md) (inferred)<br>[FR-dxvrs](requirements/FR-dxvrs-wifi-configuration.md) (inferred)<br>[FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md) | [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md)<br>[FR-dxvrs](requirements/FR-dxvrs-wifi-configuration.md)<br>[FR-eutkf](requirements/FR-eutkf-concurrent-transports.md) (inferred)<br>[NFR-eddfs](requirements/NFR-eddfs-network-latency-limits.md) (inferred)<br>[NFR-ukx3a](requirements/NFR-ukx3a-network-memory-overhead.md) (inferred) | [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md) (inferred)<br>[FR-dxvrs](requirements/FR-dxvrs-wifi-configuration.md) (inferred)<br>[FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md) |
| [NFR-3wlo1](requirements/NFR-3wlo1-imu-sampling-rate.md) - NFR-3wlo1 IMU Sampling Rate | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md) | [FR-eyuh8](requirements/FR-eyuh8-ahrs-attitude-estimation.md) | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md) |
| [NFR-eddfs](requirements/NFR-eddfs-network-latency-limits.md) - NFR-eddfs Network Transport Latency Limits | [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) | [NFR-ukjvr](requirements/NFR-ukjvr-control-loop-latency.md) | [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) |
| [NFR-nmmu0](requirements/NFR-nmmu0-platform-code-isolation.md) - NFR-nmmu0 Platform Code Isolation | — | — | — |
| [NFR-pj11s](requirements/NFR-pj11s-no-unsafe-rust.md) - NFR-pj11s Memory Safety - No Unsafe Rust | — | — | — |
| [NFR-ukjvr](requirements/NFR-ukjvr-control-loop-latency.md) - NFR-ukjvr Control Loop Latency | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md)<br>[NFR-eddfs](requirements/NFR-eddfs-network-latency-limits.md) (inferred) | — | [FR-5inw2](requirements/FR-5inw2-task-scheduler.md)<br>[NFR-eddfs](requirements/NFR-eddfs-network-latency-limits.md) (inferred) |
| [NFR-ukx3a](requirements/NFR-ukx3a-network-memory-overhead.md) - NFR-ukx3a Network Transport Memory Overhead | [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md)<br>[FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) | [NFR-z2iuk](requirements/NFR-z2iuk-memory-limits.md) | [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md)<br>[FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) |
| [NFR-ypgpm](requirements/NFR-ypgpm-transport-independence.md) - NFR-ypgpm Transport Independence and Reliability | [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md)<br>[FR-eutkf](requirements/FR-eutkf-concurrent-transports.md) | — | [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md)<br>[FR-eutkf](requirements/FR-eutkf-concurrent-transports.md) |
| [NFR-z2iuk](requirements/NFR-z2iuk-memory-limits.md) - NFR-z2iuk Memory Usage Limits | [NFR-ukx3a](requirements/NFR-ukx3a-network-memory-overhead.md) (inferred) | — | [NFR-ukx3a](requirements/NFR-ukx3a-network-memory-overhead.md) (inferred) |

### Dependency Consistency

- [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md) and [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) list each other as dependents; remove the contradiction.
- [FR-dxvrs](requirements/FR-dxvrs-wifi-configuration.md) and [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) list each other as dependents; remove the contradiction.
- [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md) - FR-6jkia MAVLink Transport Abstraction: add Prerequisite Requirements entry for [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) (inferred)
- [FR-dxvrs](requirements/FR-dxvrs-wifi-configuration.md) - FR-dxvrs WiFi Configuration: add Prerequisite Requirements entry for [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) (inferred)
- [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) - FR-ydttj UDP Network Transport: add Prerequisite Requirements entry for [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md) (inferred)<br>[FR-dxvrs](requirements/FR-dxvrs-wifi-configuration.md) (inferred)
- [NFR-ukjvr](requirements/NFR-ukjvr-control-loop-latency.md) - NFR-ukjvr Control Loop Latency: add Prerequisite Requirements entry for [NFR-eddfs](requirements/NFR-eddfs-network-latency-limits.md) (inferred)
- [NFR-z2iuk](requirements/NFR-z2iuk-memory-limits.md) - NFR-z2iuk Memory Usage Limits: add Prerequisite Requirements entry for [NFR-ukx3a](requirements/NFR-ukx3a-network-memory-overhead.md) (inferred)
- [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md) - FR-6jkia MAVLink Transport Abstraction: add Dependent Requirements entry for [NFR-ukx3a](requirements/NFR-ukx3a-network-memory-overhead.md) (inferred)<br>[NFR-ypgpm](requirements/NFR-ypgpm-transport-independence.md) (inferred)
- [FR-eutkf](requirements/FR-eutkf-concurrent-transports.md) - FR-eutkf Concurrent MAVLink Transports: add Dependent Requirements entry for [NFR-ypgpm](requirements/NFR-ypgpm-transport-independence.md) (inferred)
- [FR-gpzpz](requirements/FR-gpzpz-mavlink-protocol.md) - FR-gpzpz MAVLink Protocol Communication: add Dependent Requirements entry for [FR-6jkia](requirements/FR-6jkia-transport-abstraction.md) (inferred)<br>[FR-eutkf](requirements/FR-eutkf-concurrent-transports.md) (inferred)<br>[FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) (inferred)
- [FR-ydttj](requirements/FR-ydttj-udp-network-transport.md) - FR-ydttj UDP Network Transport: add Dependent Requirements entry for [FR-eutkf](requirements/FR-eutkf-concurrent-transports.md) (inferred)<br>[NFR-eddfs](requirements/NFR-eddfs-network-latency-limits.md) (inferred)<br>[NFR-ukx3a](requirements/NFR-ukx3a-network-memory-overhead.md) (inferred)

## Traceability Gaps

- FR-333ym: No upstream analysis or ADR references (Status: Approved)
- FR-sp3at: No upstream analysis or ADR references (Status: Approved)
- FR-sxsvw: No upstream analysis or ADR references (Status: Approved)
- ADR-hj79f: No upstream analysis references (Status: Approved)
- ADR-oa2qa: No upstream analysis references (Status: Approved)
- ADR-vywkw: No upstream analysis references (Status: Approved)
- ADR-wjcrn: No upstream analysis references (Status: Approved)
- T-49k7n: No upstream analysis, requirement, or ADR references (Status: Phase 3 Complete)
- T-oq110: No upstream analysis, requirement, or ADR references (Status: Approved)

_This file is generated by `scripts/trace-status.ts`. Do not commit generated outputs to avoid merge conflicts._