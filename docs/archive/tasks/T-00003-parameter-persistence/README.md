# T-00003 Parameter Persistence to Flash Storage

## Overview

Implement Flash-backed parameter persistence with redundant block rotation for wear leveling. Parameters survive reboots and power cycles, load in < 100ms, and support 10,000+ save cycles with async writes.

## Status

- **Current Phase**: Draft
- **Last Updated**: 2025-10-30

## Quick Links

- [Design Document](design.md) - Architecture and technical design
- [Implementation Plan](plan.md) - Phased implementation tasks

## Related Artifacts

### Requirements

- [FR-00006-runtime-parameters](../../../requirements/FR-00006-runtime-parameters.md) - Runtime parameter configuration
- [NFR-00003-memory-limits](../../../requirements/NFR-00003-memory-limits.md) - Memory constraints

### Architecture Decisions

- [ADR-00004-storage-strategy](../../../adr/ADR-00004-storage-strategy.md) - Redundant block rotation with wear leveling

### Dependencies

- **Upstream**: T-00002 (MAVLink Communication - provides RAM-based parameter registry), T-00004 (Platform Abstraction - provides Flash interface)
- **Downstream**: None

## Implementation Phases

1. **Phase 1**: Platform Flash abstraction (Flash trait, RP2350 implementation, mock)
2. **Phase 2**: Flash storage implementation (block format, serialization, CRC32)
3. **Phase 3**: Wear leveling and block rotation (round-robin, corruption recovery)
4. **Phase 4**: Integration with parameter registry (load/save, debouncing)
5. **Phase 5**: Hardware validation and endurance testing

## Success Criteria

- Support 200+ parameters persisted to Flash
- Parameter load time < 100ms
- Async parameter save (< 5ms blocking time)
- Support 10,000+ save cycles (wear leveling)
- < 0.1% corruption rate during power-loss testing
- RAM usage < 2 KB for parameter cache

## Key Design Decisions

- **Redundant block rotation**: Use 4 Flash blocks (16 KB total) with round-robin rotation for 4x wear leveling
- **CRC32 validation**: Detect corrupted blocks and fall back to backup
- **Async Flash writes**: Wrap blocking Flash operations in async tasks to avoid stalling control loops
- **Debounced saves**: Batch multiple parameter changes into single Flash write (5-second delay)
- **Sequence numbers**: Track newest block via monotonically increasing sequence number

## Progress Tracking

See [Implementation Plan](plan.md) for detailed task checklists and phase status.

## Notes

- Extends T-00002 RAM-based parameter registry with Flash persistence
- Uses ADR-00004 storage strategy (redundant blocks, wear leveling)
- Flash layout: 4 blocks at 0x040000-0x043FFF (16 KB total)
- Estimated endurance: 40,000 parameter saves (11 years @ 10 saves/day)
- Compatible with RP2040 (Pico W) and RP2350 (Pico 2 W)
