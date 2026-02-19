# NFR-00170 Hold Mode Performance

## Metadata

- Type: Non-Functional Requirement
- Status: Implemented

## Links

- Prerequisite Requirements:
  - [FR-00168-hold-mode-actuator-stop](FR-00168-hold-mode-actuator-stop.md)
- Dependent Requirements: N/A – no downstream requirements depend on this
- Related Tasks:
  - [T-00171-hold-mode](../tasks/T-00171-hold-mode/README.md)

## Requirement Statement

The Hold mode struct shall use no more than 32 bytes of memory and its `update()` method shall execute in constant time with no heap allocations.

## Rationale

Hold mode is the primary failsafe fallback. It must be:

- **Lightweight**: Minimizes RAM usage on the RP2350 (264KB SRAM) to leave resources for other subsystems
- **Deterministic**: Constant-time execution ensures predictable behavior at 50 Hz update rate
- **Allocation-free**: No heap allocations during `update()` prevents allocation failures in low-memory failsafe scenarios

## User Story

The system shall ensure Hold mode has minimal resource footprint and deterministic execution, so that it remains reliable as a failsafe fallback even under resource-constrained conditions.

## Acceptance Criteria

- [x] `core::mem::size_of::<HoldMode>()` is ≤ 32 bytes
- [x] `update()` performs no heap allocations (only two actuator writes)
- [x] `update()` has no conditional branches dependent on external state (constant-time)

## Technical Details

### Non-Functional Requirement Details

- Performance: `update()` executes 2 actuator writes per cycle (\~microsecond range)
- Reliability: No failure path in `update()` other than actuator write errors (which are logged and propagated)
- Compatibility: Works on all targets (RP2350 embedded, host tests, SITL)

**Expected struct layout:**

```
HoldMode {
    actuators: Box<dyn ActuatorInterface>,  // 16 bytes (fat pointer: data + vtable)
}
// Total: 16 bytes (well within 32-byte limit)
```

## Platform Considerations

N/A - Platform agnostic

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                       | Validation               |
| ---------------------------------- | ------ | ---------- | -------------------------------- | ------------------------ |
| Future fields increase struct size | Low    | Low        | Review struct size in unit tests | `size_of` assertion test |

## Implementation Notes

- Add `assert!(core::mem::size_of::<HoldMode>() <= 32)` in unit tests
- The struct only holds a `Box<dyn ActuatorInterface>` (16 bytes), well under the limit
- No dynamic dispatch needed beyond the `ActuatorInterface` trait object already present

## External References

N/A – No external references
