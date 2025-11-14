# T-eiuvv STATUSTEXT Notification System Implementation

## Metadata

- Type: Task
- Status: Draft

## Links

- Related Analyses:
  - [AN-e8x8h-statustext-notifications](../../analysis/AN-e8x8h-statustext-notifications.md)
- Related Requirements:
  - [FR-onj2m-statustext-public-api](../../requirements/FR-onj2m-statustext-public-api.md)
  - [FR-7qki7-statustext-chunking](../../requirements/FR-7qki7-statustext-chunking.md)
  - [FR-e9yu9-statustext-message-queue](../../requirements/FR-e9yu9-statustext-message-queue.md)
  - [FR-krvqy-statustext-router-integration](../../requirements/FR-krvqy-statustext-router-integration.md)
  - [FR-dbjjx-statustext-ardupilot-conventions](../../requirements/FR-dbjjx-statustext-ardupilot-conventions.md)
  - [NFR-ssp9q-statustext-performance-memory](../../requirements/NFR-ssp9q-statustext-performance-memory.md)
  - [NFR-e82vp-statustext-nostd](../../requirements/NFR-e82vp-statustext-nostd.md)
  - [NFR-1wo70-statustext-length-limits](../../requirements/NFR-1wo70-statustext-length-limits.md)
  - [NFR-fc2tw-statustext-host-tests](../../requirements/NFR-fc2tw-statustext-host-tests.md)
- Related ADRs:
  - [ADR-jxvhf-global-statusnotifier](../../adr/ADR-jxvhf-global-statusnotifier.md)
- Associated Design Document:
  - [T-eiuvv-statustext-implementation-design](design.md)
- Associated Plan Document:
  - [T-eiuvv-statustext-implementation-plan](plan.md)

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
