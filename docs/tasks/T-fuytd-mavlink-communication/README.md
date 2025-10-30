# T-fuytd MAVLink Protocol Communication

## Overview

Implement MAVLink 2.0 protocol communication for ground control station (GCS) integration. Support telemetry streaming (HEARTBEAT, ATTITUDE, GPS_RAW_INT), parameter management, mission protocol, and command execution. Compatible with QGroundControl and Mission Planner.

## Status

- **Current Phase**: Complete
- **Last Updated**: 2025-10-30

## Quick Links

- [Design Document](design.md) - Architecture and technical design
- [Implementation Plan](plan.md) - Phased implementation tasks

## Related Artifacts

### Requirements

- [FR-gpzpz-mavlink-protocol](../../requirements/FR-gpzpz-mavlink-protocol.md) - MAVLink protocol requirement
- [FR-a1cuu-runtime-parameters](../../requirements/FR-a1cuu-runtime-parameters.md) - Parameter configuration
- [NFR-z2iuk-memory-limits](../../requirements/NFR-z2iuk-memory-limits.md) - Memory constraints

### Architecture Decisions

- [ADR-ggou4-mavlink-implementation](../../adr/ADR-ggou4-mavlink-implementation.md) - rust-mavlink with custom handlers

### Dependencies

- **Upstream**: T-egg4f (Platform Abstraction - provides UART), T-g729p (Task Scheduler - provides periodic execution)
- **Downstream**: FR-333ym (Mission execution)

## Implementation Phases

1. **Phase 1**: Core MAVLink infrastructure (message parsing, writing, router)
2. **Phase 2**: Parameter protocol (registry, PARAM\_\* handlers in RAM only)
3. **Phase 3**: Telemetry streaming (HEARTBEAT, ATTITUDE, GPS, SYS_STATUS)
4. **Phase 4**: Command protocol (arm/disarm, mode change)
5. **Phase 5**: Mission protocol (upload/download waypoints)
6. **Phase 6**: Hardware validation and GCS compatibility (QGroundControl, Mission Planner)

## Success Criteria

- Compatible with QGroundControl 4.x and Mission Planner 1.3.x
- MAVLink state < 10 KB RAM
- 10Hz telemetry streams without dropped messages
- < 0.1% message corruption rate
- All existing tests pass

## Key Design Decisions

- **rust-mavlink crate**: Official Rust implementation with code generation from XML
- **Common dialect**: Use `common` message set for maximum GCS compatibility and minimal code size
- **Custom handlers**: Parameter, mission, command, telemetry handlers built on top of rust-mavlink
- **Embassy async**: Non-blocking UART I/O to avoid blocking scheduler tasks
- **RAM-only parameters**: Parameters stored in RAM (persistence deferred to FR-a1cuu)

## Progress Tracking

See [Implementation Plan](plan.md) for detailed task checklists and phase status.

## Notes

- Parameter persistence deferred to FR-a1cuu task (RAM-only for this task)
- Mission execution deferred to FR-333ym implementation (storage only for this task)
- Message signing deferred to future task (optional security feature)
- USB CDC transport deferred to future enhancement (UART only for initial implementation)
- Using common dialect only (no ArduPilot-specific extensions)
