# T-00014 STATUSTEXT Notification System Implementation

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-00020-statustext-notifications](../../analysis/AN-00020-statustext-notifications.md)
- Related Requirements:
  - [FR-00073-statustext-public-api](../../requirements/FR-00073-statustext-public-api.md)
  - [FR-00071-statustext-chunking](../../requirements/FR-00071-statustext-chunking.md)
  - [FR-00072-statustext-message-queue](../../requirements/FR-00072-statustext-message-queue.md)
  - [FR-00074-statustext-router-integration](../../requirements/FR-00074-statustext-router-integration.md)
  - [FR-00070-statustext-ardupilot-conventions](../../requirements/FR-00070-statustext-ardupilot-conventions.md)
  - [NFR-00066-statustext-performance-memory](../../requirements/NFR-00066-statustext-performance-memory.md)
  - [NFR-00065-statustext-nostd](../../requirements/NFR-00065-statustext-nostd.md)
  - [NFR-00064-statustext-length-limits](../../requirements/NFR-00064-statustext-length-limits.md)
  - [NFR-00063-statustext-host-tests](../../requirements/NFR-00063-statustext-host-tests.md)
- Related ADRs:
  - [ADR-00018-global-statusnotifier](../../adr/ADR-00018-global-statusnotifier.md)
- Associated Design Document:
  - [T-00014-statustext-implementation-design](design.md)
- Associated Plan Document:
  - [T-00014-statustext-implementation-plan](plan.md)

## Summary

Implement a public API for sending MAVLink STATUSTEXT messages to ground control stations, enabling all system components (arming system, failsafes, mode controllers, sensors) to report status, errors, and warnings to operators. The implementation uses a global static StatusNotifier with heapless message queue, MAVLink v2 chunking for long messages, and integration with the telemetry router.

## Scope

- In scope:
  - Public API with severity-specific functions (`send_error()`, `send_warning()`, etc.)
  - Global StatusNotifier with `Mutex<RefCell<StatusNotifier>>`
  - Message queue using `heapless::Deque<QueuedMessage, 16>`
  - MAVLink v2 chunking algorithm for messages up to 200 characters
  - Integration with MAVLink router for message transmission
  - Atomic chunk ID counter for multi-chunk messages
  - Host test support with mock message sink
  - Performance profiling to verify <100µs target
  - Migration of existing force-arm/disarm warnings to new API
- Out of scope:
  - Message deduplication (future enhancement)
  - Rate limiting per severity level (future enhancement)
  - Priority queue with bypass for EMERGENCY/ALERT (future enhancement)
  - Message logging to flash storage (separate subsystem)
  - GCS-side message reassembly (handled by GCS software)

## Success Metrics

- API call overhead: <100 µs average, <150 µs worst-case (measured on RP2350 @ 150 MHz)
- Memory footprint: ≤4 KB static RAM for notifier and queue
- Zero heap allocations: Verified via linker map (no allocator symbols)
- Message delivery: 100% of non-overflow messages appear in Mission Planner
- Chunking accuracy: Long messages (51-200 chars) reassemble correctly in GCS
- Integration: All 9 requirements' acceptance criteria satisfied
