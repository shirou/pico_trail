# FR-00072 Async STATUSTEXT Message Queue

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00020-statustext-notifications](../analysis/AN-00020-statustext-notifications.md)
- Prerequisite Requirements:
  - [FR-00073-statustext-public-api](../requirements/FR-00073-statustext-public-api.md)
- Dependent Requirements:
  - [FR-00074-statustext-router-integration](../requirements/FR-00074-statustext-router-integration.md)
  - [NFR-00066-statustext-performance-memory](../requirements/NFR-00066-statustext-performance-memory.md)
- Related Tasks:
  - [T-00014-statustext-implementation](../tasks/T-00014-statustext-implementation/README.md)

## Requirement Statement

The system shall queue STATUSTEXT messages for asynchronous transmission, allowing components to generate status notifications without blocking time-critical operations, with priority handling for EMERGENCY and ALERT severity levels.

## Rationale

Status messages may be generated during time-critical operations such as the 400 Hz scheduler interrupt, arming checks, or failsafe handling. Synchronous message transmission would block these operations. An asynchronous queue decouples message generation from transmission, ensuring real-time performance is maintained while still delivering notifications to operators.

## User Story

As a **system component running in a time-critical context**, I want **to queue status messages for later transmission**, so that **I can report errors and warnings without blocking the real-time scheduler or interrupt handlers**.

## Acceptance Criteria

- [ ] Heapless queue with 16-message capacity (see NFR-00066)
- [ ] Messages enqueued by notification API (`send_error()`, `send_warning()`, etc.)
- [ ] Messages dequeued and sent by MAVLink telemetry handler
- [ ] EMERGENCY severity bypasses queue and sends immediately
- [ ] ALERT severity bypasses queue and sends immediately
- [ ] Queue full condition logs internal warning
- [ ] Queue full condition drops oldest message (FIFO eviction)
- [ ] Queue operations complete in O(1) worst-case time
- [ ] Queue state visible for diagnostics (depth, drops count)

## Technical Details

### Functional Requirement Details

**Queue Data Structure:**

```rust
use heapless::Deque;

pub struct StatusNotifier {
    queue: Deque<QueuedMessage, 16>,
    drops: u32,  // Count of dropped messages due to full queue
}

struct QueuedMessage {
    severity: MavSeverity,
    text: heapless::String<200>,  // Max 200 chars per NFR-00064
}
```

**Queue Operations:**

```rust
// Enqueue (called by send_error(), send_warning(), etc.)
fn enqueue(&mut self, severity: MavSeverity, text: &str) {
    if severity == MavSeverity::EMERGENCY || severity == MavSeverity::ALERT {
        // Bypass queue, send immediately
        self.send_immediate(severity, text);
    } else {
        if self.queue.is_full() {
            self.queue.pop_front();  // Drop oldest
            self.drops += 1;
            log_warn!("STATUSTEXT queue full, dropped message");
        }
        self.queue.push_back(QueuedMessage { severity, text });
    }
}

// Dequeue (called by MAVLink router during telemetry generation)
fn dequeue(&mut self) -> Option<QueuedMessage> {
    self.queue.pop_front()
}
```

**Priority Handling:**

- EMERGENCY (severity 0): Immediate transmission, bypass queue
- ALERT (severity 1): Immediate transmission, bypass queue
- CRITICAL through DEBUG (severity 2-7): Queue for async transmission

**Queue Eviction Policy:**

- FIFO (First In, First Out): Drop oldest message when full
- Rationale: Recent errors more relevant than old errors
- Alternative considered: Drop by priority (rejected - complexity)

## Platform Considerations

### Embedded (RP2350)

- Must use heapless::Deque (no heap allocation, see NFR-00066)
- Queue capacity fixed at compile time (16 messages)
- Must be interrupt-safe with proper synchronization

### Host Tests

- Mock queue can verify message ordering
- Test suite can inject messages and verify FIFO behavior
- Can test queue overflow scenarios

### Cross-Platform

- Queue behavior identical on all platforms
- Message ordering guaranteed consistent

## Risks & Mitigation

| Risk                                                   | Impact | Likelihood | Mitigation                                          | Validation                                     |
| ------------------------------------------------------ | ------ | ---------- | --------------------------------------------------- | ---------------------------------------------- |
| Queue overflow loses critical error messages           | High   | Medium     | Priority bypass for EMERGENCY/ALERT, increase size  | Test with burst of 20+ messages                |
| Message reordering confuses operator                   | Low    | Low        | Strict FIFO ordering, document behavior             | Verify message timestamps in GCS               |
| Queue size too small for typical startup burst         | Medium | Medium     | Analyze ArduPilot startup patterns, adjust capacity | Monitor queue depth during rover startup       |
| Queue size too large wastes RAM                        | Low    | Low        | Benchmark typical usage, tune to actual needs       | Memory profiling on target hardware            |
| Dequeue rate slower than enqueue rate (sustained load) | Medium | Low        | Rate limiting on enqueue side (10 msgs/sec max)     | Stress test with continuous message generation |

## Implementation Notes

**Preferred Patterns:**

- Use `heapless::Deque` for fixed-capacity queue
- Store message text as `heapless::String<200>` (no allocation)
- Track dropped message count for diagnostics

**Known Pitfalls:**

- Do not assume message is sent immediately after `send_error()` call
- Do not rely on message ordering for time-critical logic
- Do not queue EMERGENCY/ALERT (must be immediate)

**Related Code:**

- heapless crate: `heapless::Deque`
- MAVLink router: `src/communication/mavlink/router.rs` (dequeue integration)

**Suggested Approach:**

- Start with 16-message capacity, monitor actual usage
- If queue overflows in production, increase to 32
- If queue never exceeds 8, consider reducing to 8

## External References

- [heapless crate documentation](https://docs.rs/heapless/)
- [MAVLink Best Practices](https://mavlink.io/en/guide/best_practices.html)
