# ADR-00018 Global StatusNotifier for MAVLink STATUSTEXT Messages

## Metadata

- Type: ADR
- Status: Draft

## Links

- Related Analyses:
  - [AN-00020-statustext-notifications](../analysis/AN-00020-statustext-notifications.md)
- Impacted Requirements:
  - [FR-00073-statustext-public-api](../requirements/FR-00073-statustext-public-api.md)
  - [FR-00071-statustext-chunking](../requirements/FR-00071-statustext-chunking.md)
  - [FR-00072-statustext-message-queue](../requirements/FR-00072-statustext-message-queue.md)
  - [FR-00074-statustext-router-integration](../requirements/FR-00074-statustext-router-integration.md)
  - [FR-00070-statustext-ardupilot-conventions](../requirements/FR-00070-statustext-ardupilot-conventions.md)
  - [NFR-00066-statustext-performance-memory](../requirements/NFR-00066-statustext-performance-memory.md)
  - [NFR-00065-statustext-nostd](../requirements/NFR-00065-statustext-nostd.md)
  - [NFR-00064-statustext-length-limits](../requirements/NFR-00064-statustext-length-limits.md)
  - [NFR-00063-statustext-host-tests](../requirements/NFR-00063-statustext-host-tests.md)
- Supersedes ADRs: None
- Related Tasks:
  - [T-00014-statustext-implementation](../tasks/T-00014-statustext-implementation/README.md)

## Context

The project needs a centralized system for sending MAVLink STATUSTEXT messages to ground control stations. Currently, only the CommandHandler can generate STATUSTEXT messages during force-arm/disarm operations via a private `create_statustext()` function at `src/communication/mavlink/handlers/command.rs:275`. System components (arming system, failsafes, mode controllers, sensors) have no way to communicate status, errors, or warnings to GCS operators.

**Problem:**

- No public API for STATUSTEXT message generation
- System components cannot report status to operators
- Critical information (pre-arm failures, failsafe triggers, sensor errors) invisible to GCS
- Existing implementation lacks MAVLink v2 chunking support for messages >50 characters

**Constraints:**

- Embedded target (RP2350) with 264 KB RAM
- Must use no_std (no heap allocation)
- Real-time scheduler runs at 400 Hz (2.5 ms period)
- Notification calls may happen during time-critical operations
- Must not block or introduce jitter in control loops
- Must work in interrupt context with proper guards

**Forces in Tension:**

- **Accessibility vs. Testability**: Global state is easy to access but harder to test vs. dependency injection is more testable but requires threading through many functions
- **Simplicity vs. Safety**: Global mutex is simple but risks deadlocks vs. message passing is safer but more complex
- **Performance vs. Features**: Immediate sending is fast but limited vs. queuing enables chunking/batching but adds overhead
- **Memory Safety vs. Flexibility**: Fixed-capacity queue is predictable but can overflow vs. dynamic allocation is flexible but risky on embedded

**Prior Art:**

- ArduPilot uses global GCS_MAVLINK singleton accessed via `gcs().send_text()`
- PX4 uses uORB publish/subscribe for status messages
- Existing project patterns use global static with Mutex for shared state

## Success Metrics

- API call overhead: <100 µs average, <150 µs worst-case (measured on RP2350 @ 150 MHz)
- Memory footprint: ≤4 KB static RAM for notifier and queue
- Zero heap allocations: Verified via linker map (no allocator symbols)
- Message delivery: 100% of non-overflow messages appear in Mission Planner
- Developer adoption: All new system components use notification API within 1 sprint of implementation

## Decision

We will implement a **global static StatusNotifier with Mutex-protected queue** for MAVLink STATUSTEXT message generation and transmission.

**Core Architecture:**

1. **Global Access Pattern**: `static` notifier with `Mutex<RefCell<StatusNotifier>>` for thread-safe access
2. **Message Queue**: `heapless::Deque<QueuedMessage, 16>` for fixed-capacity no-allocation storage
3. **Public API**: Severity-specific free functions (`send_error()`, `send_warning()`, etc.) that enqueue messages
4. **MAVLink v2 Chunking**: Deferred to transmission phase, messages up to 200 characters supported
5. **Router Integration**: Telemetry handler drains queue and sends STATUSTEXT via MAVLink router

**API Design:**

```rust
// Public API (free functions for easy access)
pub fn send_emergency(text: &str);
pub fn send_alert(text: &str);
pub fn send_critical(text: &str);
pub fn send_error(text: &str);
pub fn send_warning(text: &str);
pub fn send_notice(text: &str);
pub fn send_info(text: &str);
pub fn send_debug(text: &str);
```

**Internal Structure:**

```rust
use heapless::{Deque, String};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

struct StatusNotifier {
    queue: Deque<QueuedMessage, 16>,
    next_chunk_id: u16,
    dropped_count: u32,
}

struct QueuedMessage {
    severity: MavSeverity,
    text: String<200>,
}

static NOTIFIER: Mutex<CriticalSectionRawMutex, RefCell<StatusNotifier>> = ...;
```

### Decision Drivers

- **Immediate Value**: Lowest effort (2-3 days) to deliver public API
- **Consistency**: Matches existing project patterns for global state
- **Accessibility**: Simple call from any module without parameter threading
- **Performance**: Direct queue push without async overhead
- **Memory Safety**: heapless structures guarantee no allocation

### Considered Options

- **Option A**: Global Static Notifier with Mutex (CHOSEN)
- **Option B**: Notifier as Component in SystemState
- **Option C**: Message Queue in MAVLink State with Helper Functions
- **Option D**: Embassy Channel for Async Notification

### Option Analysis

**Option A — Global Static Notifier with Mutex**

- Pros: Simple access, matches project patterns, low effort (2-3 days), no function parameter changes
- Cons: Global mutable state harder to test, requires careful lock management, cannot call from interrupt with blocking mutex

**Option B — Notifier as Component in SystemState**

- Pros: Explicit dependencies, easier to test with mocks, no global state
- Cons: Must thread `&mut SystemState` through many functions, larger refactoring (5-7 days), harder to call from utility functions

**Option C — Message Queue in MAVLink State**

- Pros: Notifier state lives with MAVLink layer, router drains queue naturally
- Cons: Tight coupling to MAVLink, still requires global access, harder to use independently

**Option D — Embassy Channel for Async Notification**

- Pros: Async-friendly, built-in backpressure, clean producer/consumer separation
- Cons: Requires async runtime, more complex code (7-10 days), may require task restructuring

## Rationale

**Option A (Global Static Notifier with Mutex) selected because:**

1. **Lowest Implementation Cost**: 2-3 days vs. 5-10 days for alternatives, delivers immediate value
2. **Established Pattern**: Project already uses this pattern for other global state (consistent with codebase conventions)
3. **Developer Experience**: Simple `send_error("message")` call without dependency injection or parameter threading
4. **Performance**: Direct queue push is O(1) and <100 µs, meeting NFR-00066 requirements
5. **Embedded Compatibility**: Works with or without async runtime, compatible with interrupt context via try_lock variant
6. **Incremental Evolution**: Can refactor to Option B or D later if testing or concurrency needs change

**Why Not Option B (SystemState Component)?**

- Requires threading `&mut SystemState` through arming checks, failsafe handlers, mode controllers - large refactoring effort
- Utility functions and static helpers cannot easily access SystemState
- Testing benefits not critical for this feature (can mock via test-only feature flag)

**Why Not Option C (MAVLink State)?**

- Unnecessarily couples notification API to MAVLink implementation details
- Still requires global access to MavlinkState (doesn't solve global state concern)
- Makes future non-MAVLink notifications (e.g., logging to flash) harder

**Why Not Option D (Embassy Channel)?**

- Async complexity not justified for simple queue-and-send pattern
- Requires async context for producers (some callers may be sync)
- Higher implementation and testing effort without clear benefits

**Trade-offs Accepted:**

- Global mutable state makes unit testing harder → Mitigate with test-only mock backend
- Mutex contention possible under heavy load → Mitigate with priority bypass for EMERGENCY/ALERT
- Cannot call directly from interrupt → Mitigate with try_lock variant and fallback

## Consequences

### Positive

- System components can report status to GCS operators (primary requirement fulfilled)
- Simple API (`send_error("message")`) requires minimal code changes
- No heap allocation (heapless structures) ensures predictable memory usage
- MAVLink v2 chunking enables messages >50 characters
- Consistent with existing project global state patterns
- Fast implementation (2-3 days) delivers value quickly
- Zero performance regression (O(1) queue push, <100 µs)

### Negative

- Global mutable state harder to unit test (requires test-only mocking)
- Mutex introduces potential for deadlock (must carefully manage lock acquisition)
- Cannot call from interrupt context with blocking mutex (need try_lock variant)
- Fixed queue capacity (16 messages) can overflow during error bursts (messages dropped)
- Future refactoring to dependency injection would require API changes

### Neutral

- Message queue introduces slight latency (queued until next telemetry cycle)
- Requires enabling `mavlink2` feature in Cargo.toml (already planned)
- Static memory footprint \~3.2 KB (acceptable for 264 KB target)

## Implementation Notes

**Implementation Phases:**

1. **Phase 1: Core Notifier** (1 day)
   - Create `src/communication/mavlink/status_notifier.rs`
   - Implement `StatusNotifier` with `heapless::Deque<QueuedMessage, 16>`
   - Implement global static with `Mutex<RefCell<StatusNotifier>>`
   - Implement severity-specific API functions
   - Unit tests for queue operations

2. **Phase 2: MAVLink v2 Chunking** (1 day)
   - Enable `mavlink2` feature in `Cargo.toml`
   - Implement chunking algorithm (messages up to 200 chars)
   - Implement atomic chunk ID counter (u16, wraps at MAX)
   - Unit tests for chunking edge cases

3. **Phase 3: Router Integration** (0.5 days)
   - Integrate queue draining in telemetry handler
   - Send STATUSTEXT messages via `MavlinkRouter`
   - Remove private `create_statustext()` from CommandHandler
   - Update force-arm/disarm warnings to use new API

4. **Phase 4: Testing & Validation** (0.5 days)
   - Host tests with mock message sink
   - Embedded test with Mission Planner
   - Performance profiling (<100 µs verification)
   - Long-running stress test (queue overflow scenarios)

**Key Interfaces:**

```rust
// Public API (in src/communication/mavlink/status_notifier.rs)
pub fn send_emergency(text: &str);
pub fn send_error(text: &str);
pub fn send_warning(text: &str);
// ... other severity levels

// Internal API (used by telemetry handler)
pub(crate) fn drain_messages() -> impl Iterator<Item = Vec<STATUSTEXT_DATA>>;
```

**Storage Locations:**

- Notifier implementation: `src/communication/mavlink/status_notifier.rs`
- Queue draining: `src/communication/mavlink/router.rs` (telemetry handler)
- Tests: `src/communication/mavlink/status_notifier.rs` (unit tests), `tests/integration/statustext.rs` (integration tests)

**Error Handling:**

- Queue full: Drop oldest message, increment `dropped_count` metric, log warning via defmt
- Message too long (>200 chars): Truncate to 200 chars, log warning
- Invalid UTF-8: Replace invalid sequences with � (replacement character)
- Mutex poisoned: Panic (critical invariant violation)

## Examples

**Usage from Arming System:**

```rust
use crate::communication::mavlink::status_notifier::send_error;

fn check_battery_voltage(&self) -> ArmingCheckResult {
    if voltage < BATT_ARM_VOLT {
        send_error("PreArm: Battery voltage low");
        return ArmingCheckResult::Failed;
    }
    ArmingCheckResult::Passed
}
```

**Usage from Failsafe System:**

```rust
use crate::communication::mavlink::status_notifier::send_warning;

fn trigger_gcs_failsafe(&mut self) {
    send_warning("Failsafe: GCS connection lost");
    self.execute_failsafe_action();
}
```

**Long Message with Automatic Chunking:**

```rust
use crate::communication::mavlink::status_notifier::send_error;

// Message: 109 characters
send_error("PreArm: Battery voltage 9.8V is below minimum arming voltage 10.5V configured in BATT_ARM_VOLT parameter");

// Automatically chunked into 3 STATUSTEXT messages:
// Chunk 0 (id=1, chunk_seq=0): "PreArm: Battery voltage 9.8V is below minimum arm"
// Chunk 1 (id=1, chunk_seq=1): "ing voltage 10.5V configured in BATT_ARM_VOLT par"
// Chunk 2 (id=1, chunk_seq=2): "ameter\0\0\0..."
```

**Test Mock (Host Tests):**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_message_queued() {
        let messages = capture_messages(|| {
            send_error("Test error message");
        });

        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].severity, MavSeverity::ERROR);
        assert_eq!(messages[0].text_str(), "Test error message");
    }
}
```

## Platform Considerations

**Embedded (RP2350):**

- Uses `embassy_sync::blocking_mutex::Mutex` with `CriticalSectionRawMutex` for no_std compatibility
- heapless structures stored in static `.bss` section
- Performance profiled with `embassy_time::Instant` to verify <100 µs
- Interrupt-safe variant uses `try_lock()` with fallback (log drop count)

**Host Tests:**

- Mock backend captures messages for test assertions
- Test-only feature flag enables message interception
- Performance tests verify O(1) complexity but not absolute timing

**Cross-Platform:**

- API signature identical on all targets
- heapless crate works on both embedded and host
- MAVLink v2 chunking protocol independent of platform

## Security & Privacy

- No sensitive data in STATUSTEXT messages (all messages visible to GCS operator)
- Message content should not include secrets, keys, or personally identifiable information
- Developers must avoid logging sensitive parameter values in status messages

## Monitoring & Logging

**Diagnostics:**

- `dropped_count` metric tracks queue overflow events
- defmt/log warning when queue full (rate-limited to avoid log spam)
- defmt/log warning when message truncated (>200 chars)
- Performance profiling via `embassy_time::Instant` in debug builds

**Verbosity:**

- `send_debug()` messages only sent if debug telemetry enabled
- EMERGENCY/ALERT messages always sent (bypass queue if full)
- Configurable rate limiting (future enhancement)

## Open Questions

- [ ] Should we implement message deduplication to suppress repeated identical messages? → Next step: Research ArduPilot's deduplication strategy, refine in task design docs/tasks/T-<id>-statustext/design.md
- [ ] Should EMERGENCY/ALERT bypass queue and send immediately? → Next step: Prototype both approaches, measure latency difference
- [ ] Should we rate-limit messages per severity level differently? → Method: Review MAVLink best practices documentation
- [ ] How should we handle queue full condition for non-critical messages? → Current decision: Drop oldest, can revisit if problematic

## External References

- [MAVLink STATUSTEXT Message Specification](https://mavlink.io/en/messages/common.html#STATUSTEXT)
- [MAVLink v2 Protocol Specification](https://mavlink.io/en/guide/mavlink_2.html)
- [RFC-5424 Syslog Severity Levels](https://datatracker.ietf.org/doc/html/rfc5424#section-6.2.1)
- [rust-mavlink GitHub Repository](https://github.com/mavlink/rust-mavlink)
- [heapless crate documentation](https://docs.rs/heapless/)
- [ArduPilot GCS_MAVLink Implementation](https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink)
