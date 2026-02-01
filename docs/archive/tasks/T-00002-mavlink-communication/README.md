# T-00002 MAVLink Protocol Communication

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

- [FR-00005-mavlink-protocol](../../../requirements/FR-00005-mavlink-protocol.md) - MAVLink protocol requirement
- [FR-00006-runtime-parameters](../../../requirements/FR-00006-runtime-parameters.md) - Parameter configuration
- [NFR-00003-memory-limits](../../../requirements/NFR-00003-memory-limits.md) - Memory constraints

### Architecture Decisions

- [ADR-00002-mavlink-implementation](../../../adr/ADR-00002-mavlink-implementation.md) - rust-mavlink with custom handlers

### Dependencies

- **Upstream**: T-00004 (Platform Abstraction - provides UART), T-00001 (Task Scheduler - provides periodic execution)
- **Downstream**: FR-00004 (Mission execution)

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
- **RAM-only parameters**: Parameters stored in RAM (persistence deferred to FR-00006)

## Progress Tracking

See [Implementation Plan](plan.md) for detailed task checklists and phase status.

## Notes

- Parameter persistence deferred to FR-00006 task (RAM-only for this task)
- Mission execution deferred to FR-00004 implementation (storage only for this task)
- Message signing deferred to future task (optional security feature)
- USB CDC transport deferred to future enhancement (UART only for initial implementation)
- Using common dialect only (no ArduPilot-specific extensions)
