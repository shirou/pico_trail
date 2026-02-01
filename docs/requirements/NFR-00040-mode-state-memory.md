# NFR-00040 Mode Lifecycle System Memory Under 100 Bytes Per Mode

## Metadata

- Type: Non-Functional Requirement
- Status: Draft
- Priority: P2
- Category: Resource Constraints

## Links

- Parent Analysis: [AN-00014-mode-lifecycle-management](../analysis/AN-00014-mode-lifecycle-management.md)
- Related Requirements: [FR-00048-mode-lifecycle-management](FR-00048-mode-lifecycle-management.md)
- Related ADRs: TBD
- Related Tasks: TBD
- Dependent Requirements:
  N/A – No dependent requirements

## Requirement Statement

Mode lifecycle system shall add no more than 100 bytes RAM per mode because limited RAM on RP2040/RP2350 (264 KB) requires mode state to be lean, with only one mode active at a time minimizing total overhead.

## Rationale

Limited RAM requires efficient per-mode state usage. Only one mode active at a time, so total overhead = one mode's state + mode manager overhead. Keeping per-mode state small ensures scalability.

## Measurement Criteria

- Per-mode RAM usage measured via size profiling
- Target: < 100 bytes per mode instance
- Components measured:
  - Mode trait vtable: \~8 bytes
  - Mode-specific state: < 92 bytes
  - Entry timestamp: 4 bytes
  - Mode-local variables: varies per mode
- Measurement method: cargo bloat, linker map analysis

## Acceptance Criteria

1. Mode state size targets:
   - ManualMode: < 20 bytes (minimal state)
   - HoldMode: < 40 bytes (position + flags)
   - AutoMode: < 80 bytes (waypoint + mission state)
   - RtlMode: < 40 bytes (home + flags)
   - LoiterMode: < 40 bytes (position + flags)
2. Memory usage analysis:
   - Profile each mode's state size
   - Identify large allocations
   - Optimize if mode exceeds 100 byte budget
3. Mode manager overhead:
   - Current mode pointer: 8 bytes
   - Previous mode number: 1 byte
   - Total manager: < 16 bytes
4. Memory regression testing:
   - CI checks mode state sizes
   - Fail if mode exceeds 100 byte budget

## Success Metrics

- All modes under 100 byte limit
- Total active mode overhead < 120 bytes (mode + manager)
- No unexpected allocations
- Memory usage stable over mode transitions

## Verification Methods

- Size profiling: cargo bloat per mode
- Linker map analysis: Identify mode allocations
- HITL testing: Monitor RAM usage on hardware
- Static analysis: Verify no hidden allocations

## ArduPilot Comparison

ArduPilot mode state sizes (approximate):

- Manual: 10-20 bytes
- Auto: 50-80 bytes (waypoint state)
- RTL: 30-50 bytes (return state)
- Mode manager: 10-20 bytes

## Notes

- Only one mode active at a time (no per-mode array)
- Mode state should be minimal, necessary fields only
- Consider sharing state between similar modes (Phase 2)
- Target hardware: RP2350 with 264 KB RAM
