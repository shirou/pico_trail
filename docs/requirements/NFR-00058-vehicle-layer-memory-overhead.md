# NFR-00058 Vehicle Layer Memory Overhead

## Metadata

- Type: Non-Functional Requirement
- Status: Draft

## Links

- Prerequisite Requirements:
  - [FR-00043-manual-mode-implementation](FR-00043-manual-mode-implementation.md)
  - [FR-00047-mode-execution-framework](FR-00047-mode-execution-framework.md)
- Dependent Requirements: N/A
- Related Tasks: N/A - Tasks will be created after ADRs

## Requirement Statement

Vehicle layer (modes, actuators, RC input) shall add no more than 5 KB RAM and 20 KB Flash overhead, maintaining memory budget for other subsystems on RP2040/RP2350 platforms.

## Rationale

Memory efficiency essential on RP2040/RP2350 (264 KB RAM, 2 MB Flash). Vehicle layer budget allows comprehensive mode framework while preserving memory for communication, navigation, and sensors.

## User Story (if applicable)

The vehicle control layer shall be implemented efficiently to preserve memory resources for future features and subsystems.

## Acceptance Criteria

- [ ] Total vehicle layer RAM < 5 KB
- [ ] Total vehicle layer Flash < 20 KB
- [ ] Includes: mode framework, actuators, RC input, mode instances
- [ ] Measured via memory profiling tools
- [ ] No excessive dynamic allocation

## Technical Details (if applicable)

**Memory Budget**:

- RAM: < 5 KB
- Flash: < 20 KB

**Estimated Usage**:

- RC input state: \~100 B
- Mode framework: \~200 B
- Actuator layer: \~100 B
- Manual mode instance: \~50 B

**Total Estimated**: \~450 B RAM (well under 5 KB target)

Flash budget easily accommodated by minimal code footprint.

## Platform Considerations

### Pico W (RP2040)

- RAM: 264 KB total
- Flash: 2 MB total
- Vehicle layer: < 2% RAM, < 1% Flash

### Pico 2 W (RP2350)

- RAM: 520 KB total (even more margin)
- Flash: 4 MB total

## Risks & Mitigation

| Risk                            | Impact | Likelihood | Mitigation                                           |
| ------------------------------- | ------ | ---------- | ---------------------------------------------------- |
| Memory budget exceeded          | Medium | Low        | Profile early, optimize hot paths, static allocation |
| Flash code size grows over time | Low    | Medium     | Code review for bloat, minimize dependencies         |

## Implementation Notes

- Use static allocation where possible
- Profile memory usage during development
- Keep mode implementations lean
- Minimize trait object overhead

Related code areas:

- `src/vehicle/` - All vehicle layer code

## External References

- Analysis: [AN-00007-manual-control-implementation](../analysis/AN-00007-manual-control-implementation.md)
