# T-00014 STATUSTEXT Notification System

## Metadata

- Type: Design
- Status: Draft

## Links

- Associated Plan Document:
  - [T-00014-statustext-implementation-plan](plan.md)

## Overview

This design implements a centralized STATUSTEXT notification system for MAVLink, enabling system components to send status messages, error reports, and warnings to ground control station operators. The system uses a global static notifier with heapless queue, MAVLink v2 chunking for long messages, and seamless integration with the existing telemetry router. The design prioritizes no-allocation embedded compatibility, <100µs performance, and developer ergonomics with severity-specific API functions.

## Success Metrics

- [ ] API call overhead <100 µs (average), <150 µs (worst-case) on RP2350 @ 150 MHz
- [ ] Zero heap allocations (verified via linker map inspection)
- [ ] Static memory footprint ≤4 KB
- [ ] Messages >50 chars chunk correctly and reassemble in Mission Planner
- [ ] All 9 requirements' acceptance criteria satisfied
- [ ] No regressions in existing MAVLink communication

## Background and Current State

- Context: MAVLink communication layer in pico_trail provides protocol handling for GCS interaction
- Current behavior:
  - Private `create_statustext()` function in `src/communication/mavlink/handlers/command.rs:275`
  - Only force-arm/disarm operations generate STATUSTEXT messages
  - Messages truncated at 50 characters (no MAVLink v2 chunking)
  - No public API for system components to send status notifications
- Pain points:
  - Arming system cannot report pre-arm failures to operators
  - Failsafe system cannot notify GCS of failsafe triggers
  - Sensor drivers cannot report hardware errors
  - Mode system cannot announce mode changes
  - Critical information invisible to GCS operators
- Constraints:
  - Embedded target (RP2350) with 264 KB RAM
  - Must use no_std (no heap allocation)
  - Real-time scheduler at 400 Hz (2.5 ms period)
  - Notification calls during time-critical operations
  - MAVLink v2 chunking requires `mavlink2` feature
- Related ADRs: [ADR-00018-global-statusnotifier](../../adr/ADR-00018-global-statusnotifier.md)

## Proposed Design

### High-Level Architecture

```text
┌─────────────────────────────────────────────────────────────────┐
│                    System Components                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │ Arming   │  │Failsafe  │  │  Mode    │  │ Sensors  │       │
│  │ System   │  │ System   │  │ System   │  │          │       │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘       │
│       │             │              │             │              │
│       │ send_error()│ send_warning()│ send_info()│             │
│       └─────────────┴──────────────┴─────────────┘              │
│                          │                                      │
└──────────────────────────┼──────────────────────────────────────┘
                           ▼
         ┌─────────────────────────────────────┐
         │   Global StatusNotifier (Static)    │
         │   Mutex<RefCell<StatusNotifier>>    │
         │                                     │
         │  ┌────────────────────────────┐    │
         │  │ heapless::Deque<Message,16>│    │
         │  │ - severity: MavSeverity    │    │
         │  │ - text: String<200>        │    │
         │  │ - timestamp                │    │
         │  └────────────────────────────┘    │
         │  next_chunk_id: AtomicU16          │
         │  dropped_count: u32                │
         └─────────────┬───────────────────────┘
                       │ drain_messages()
                       ▼
         ┌─────────────────────────────────────┐
         │    MAVLink Router (Telemetry)       │
         │                                     │
         │  ┌─────────────────────────┐       │
         │  │  Chunking Algorithm     │       │
         │  │  - Split >50 char msgs  │       │
         │  │  - Assign chunk_id      │       │
         │  │  - Sequential chunk_seq │       │
         │  └────────┬────────────────┘       │
         │           │                         │
         │           ▼                         │
         │  ┌─────────────────────────┐       │
         │  │ STATUSTEXT_DATA msgs    │       │
         │  │ (MAVLink v2 format)     │       │
         │  └────────┬────────────────┘       │
         └───────────┼─────────────────────────┘
                     │
                     ▼
         ┌─────────────────────────────────────┐
         │     Transport Layer (UDP/UART)      │
         └─────────────────────────────────────┘
                     │
                     ▼
         ┌─────────────────────────────────────┐
         │  Ground Control Station (GCS)       │
         │  - Mission Planner                  │
         │  - QGroundControl                   │
         └─────────────────────────────────────┘
```

### Components

**1. StatusNotifier (`src/communication/mavlink/status_notifier.rs`)**

- Responsibilities:
  - Queue pending STATUSTEXT messages
  - Assign unique chunk IDs for multi-chunk messages
  - Track dropped messages due to queue overflow
  - Provide thread-safe access via global static
- Key structures:
  - `StatusNotifier`: Main struct with queue and counters
  - `QueuedMessage`: Message stored in queue (severity + text)
  - `NOTIFIER`: Global static `Mutex<RefCell<StatusNotifier>>`
- API functions:
  - `send_emergency(text: &str)` through `send_debug(text: &str)`
  - `drain_messages() -> impl Iterator<Item = QueuedMessage>` (internal)

**2. Chunking Algorithm (`src/communication/mavlink/status_notifier.rs`)**

- Responsibilities:
  - Split messages >50 characters into chunks
  - Assign sequential chunk_seq (0, 1, 2, ...)
  - Pad last chunk with null bytes
  - Generate unique chunk IDs (atomic counter)
- Implementation:
  - `chunk_message(severity, text) -> Vec<STATUSTEXT_DATA, 4>` (heapless)
  - Uses `heapless::Vec` to avoid allocation
  - Maximum 4 chunks (200 chars / 50 chars = 4)

**3. Router Integration (`src/communication/mavlink/router.rs`)**

- Responsibilities:
  - Drain notification queue during telemetry cycle
  - Chunk and send STATUSTEXT messages
  - Integrate with existing heartbeat/telemetry streaming
- Integration point:
  - Called from telemetry handler (existing 1 Hz loop)
  - Sends STATUSTEXT alongside other telemetry messages

**4. Test Infrastructure**

- Host tests:
  - Mock message sink captures messages for verification
  - Test-only feature flag enables message interception
  - Unit tests for chunking, queue overflow, API functions
- Embedded tests:
  - Performance profiling with `embassy_time::Instant`
  - Integration test with Mission Planner (manual verification)

### Data Flow

1. **Message Enqueue**:
   - Component calls `send_error("PreArm: Battery voltage low")`
   - Function acquires mutex lock on global NOTIFIER
   - Creates `QueuedMessage` with severity and text (heapless::String<200>)
   - Pushes message to queue (O(1) operation)
   - Releases mutex lock
   - Returns to caller (<100 µs total)

2. **Message Dequeue and Transmission**:
   - Telemetry handler calls `drain_messages()` (1 Hz loop)
   - Drains all queued messages from NOTIFIER
   - For each message:
     - If ≤50 chars: Create single STATUSTEXT (id=0, chunk_seq=0)
     - If >50 chars: Call `chunk_message()` to split into chunks
     - Send STATUSTEXT messages via MavlinkRouter
   - Queue empties, ready for next cycle

3. **Chunking**:
   - Input: "PreArm: Battery voltage 9.8V is below minimum arming voltage 10.5V configured in BATT_ARM_VOLT parameter" (109 chars)
   - Assign chunk_id = next_chunk_id.fetch_add(1)
   - Split into 50-byte chunks:
     - Chunk 0 (chunk_seq=0): "PreArm: Battery voltage 9.8V is below minimum arm"
     - Chunk 1 (chunk_seq=1): "ing voltage 10.5V configured in BATT_ARM_VOLT par"
     - Chunk 2 (chunk_seq=2): "ameter\0\0\0..." (padded)
   - Return `Vec<STATUSTEXT_DATA, 4>` with 3 messages

4. **GCS Reassembly**:
   - GCS receives chunks with same chunk_id
   - Reassembles using chunk_seq ordering
   - Displays full message: "PreArm: Battery voltage 9.8V is below minimum arming voltage 10.5V configured in BATT_ARM_VOLT parameter"

### Data Models and Types

**StatusNotifier Structure:**

```rust
use heapless::{Deque, String};
use core::sync::atomic::{AtomicU16, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use core::cell::RefCell;

pub struct StatusNotifier {
    queue: Deque<QueuedMessage, 16>,
    next_chunk_id: AtomicU16,
    dropped_count: u32,
}

struct QueuedMessage {
    severity: MavSeverity,
    text: String<200>,
}

static NOTIFIER: Mutex<CriticalSectionRawMutex, RefCell<StatusNotifier>> =
    Mutex::new(RefCell::new(StatusNotifier::new()));
```

**API Functions:**

```rust
pub fn send_emergency(text: &str) {
    send_statustext(MavSeverity::EMERGENCY, text);
}

pub fn send_alert(text: &str) {
    send_statustext(MavSeverity::ALERT, text);
}

pub fn send_critical(text: &str) {
    send_statustext(MavSeverity::CRITICAL, text);
}

pub fn send_error(text: &str) {
    send_statustext(MavSeverity::ERROR, text);
}

pub fn send_warning(text: &str) {
    send_statustext(MavSeverity::WARNING, text);
}

pub fn send_notice(text: &str) {
    send_statustext(MavSeverity::NOTICE, text);
}

pub fn send_info(text: &str) {
    send_statustext(MavSeverity::INFORMATIONAL, text);
}

pub fn send_debug(text: &str) {
    send_statustext(MavSeverity::DEBUG, text);
}

fn send_statustext(severity: MavSeverity, text: &str) {
    NOTIFIER.lock(|notifier| {
        let mut n = notifier.borrow_mut();
        n.enqueue(severity, text);
    });
}
```

**Chunking Function:**

```rust
use heapless::Vec;

fn chunk_message(severity: MavSeverity, text: &str) -> Vec<STATUSTEXT_DATA, 4> {
    const CHUNK_SIZE: usize = 50;
    let bytes = text.as_bytes();
    let len = bytes.len().min(200); // Enforce 200-char limit
    let mut chunks = Vec::new();

    if len <= CHUNK_SIZE {
        // Single message (id=0)
        let mut text_bytes = [0u8; 50];
        text_bytes[..len].copy_from_slice(&bytes[..len]);
        chunks.push(STATUSTEXT_DATA {
            severity,
            text: text_bytes.into(),
            #[cfg(feature = "mavlink2")]
            id: 0,
            #[cfg(feature = "mavlink2")]
            chunk_seq: 0,
        }).ok();
        return chunks;
    }

    // Multi-chunk message
    let chunk_id = NOTIFIER.lock(|notifier| {
        notifier.borrow_mut().next_chunk_id.fetch_add(1, Ordering::Relaxed)
    });
    if chunk_id == 0 {
        // Wrapped to 0, skip to 1
        NOTIFIER.lock(|notifier| {
            notifier.borrow_mut().next_chunk_id.store(1, Ordering::Relaxed);
        });
    }

    let mut offset = 0;
    let mut chunk_seq = 0;
    while offset < len {
        let chunk_len = (len - offset).min(CHUNK_SIZE);
        let mut text_bytes = [0u8; 50];
        text_bytes[..chunk_len].copy_from_slice(&bytes[offset..offset + chunk_len]);

        chunks.push(STATUSTEXT_DATA {
            severity,
            text: text_bytes.into(),
            #[cfg(feature = "mavlink2")]
            id: chunk_id,
            #[cfg(feature = "mavlink2")]
            chunk_seq,
        }).ok();

        offset += chunk_len;
        chunk_seq += 1;
    }

    chunks
}
```

### Error Handling

- **Queue Full**:
  - Drop oldest message from queue
  - Increment `dropped_count` metric
  - Log warning via defmt: `log_warn!("STATUSTEXT queue full, dropped {} messages", dropped_count)`
  - Continue operation (non-fatal)

- **Message Too Long (>200 chars)**:
  - Truncate to 200 characters
  - Log warning via defmt: `log_warn!("STATUSTEXT truncated to 200 chars")`
  - Proceed with chunking

- **Invalid UTF-8**:
  - Replace invalid sequences with � (U+FFFD)
  - Use `String::from_utf8_lossy()` equivalent
  - Log warning if replacement occurred

- **Mutex Poisoned**:
  - Panic (critical invariant violation)
  - Should never occur in normal operation

- **Chunk ID Wraparound**:
  - Skip 0 (reserved for non-chunked)
  - Wrap to 1 at u16::MAX
  - Tested with long-running stress test

### Security Considerations

- No sensitive data in STATUSTEXT messages (visible to GCS operator)
- Message content must not include secrets, keys, or PII
- Developer documentation warns against logging sensitive parameter values
- No external input validation required (internal API only)

### Performance Considerations

**Hot Paths:**

- `send_statustext()`: Called frequently from arming checks, failsafes
  - Target: <100 µs average, <150 µs worst-case
  - Optimizations:
    - Simple string copy (no formatting)
    - O(1) queue push with `heapless::Deque`
    - Minimal lock hold time
    - No chunking during enqueue (deferred to transmission)

- `drain_messages()`: Called at 1 Hz from telemetry handler
  - Target: <1 ms for 16 messages
  - Optimizations:
    - Drain all messages in single lock acquisition
    - Chunking performed outside critical section
    - Batch send via MavlinkRouter

**Caching Strategy:**

- No caching required (messages are ephemeral)
- Queue acts as temporary buffer

**Async/Concurrency:**

- Mutex protects global NOTIFIER state
- `CriticalSectionRawMutex` for no_std compatibility
- No async context required for enqueue
- Telemetry handler drains synchronously

**Profiling:**

```rust
fn profile_notification() {
    use embassy_time::Instant;

    let start = Instant::now();
    send_error("PreArm: Battery voltage 9.8V below minimum 10.5V");
    let elapsed = start.elapsed().as_micros();

    log_info!("STATUSTEXT enqueue: {} µs", elapsed);
    assert!(elapsed < 100, "Notification too slow: {} µs", elapsed);
}
```

### Platform Considerations

#### Embedded (RP2350)

- Uses `embassy_sync::blocking_mutex::Mutex` with `CriticalSectionRawMutex`
- heapless structures stored in static `.bss` section
- Performance profiled with `embassy_time::Instant`
- Interrupt-safe variant uses `try_lock()` with fallback
- No heap allocator required
- Compatible with embassy async runtime

#### Host Tests

- Mock backend captures messages for test assertions
- Test-only feature flag enables message interception
- Performance tests verify O(1) complexity (not absolute timing)
- Uses `std::time::Instant` for profiling
- Can simulate queue overflow, chunking, concurrent access

#### Cross-Platform

- API signature identical on all targets
- heapless crate works on both embedded and host
- MAVLink v2 chunking protocol platform-independent
- Mutex primitive differs (CriticalSection vs. std::sync)

## Alternatives Considered

1. **SystemState Component** (instead of global static)
   - Pros: Explicit dependencies, easier to test with mocks
   - Cons: Must thread `&mut SystemState` through many functions, larger refactoring (5-7 days)
   - Rejected: Higher implementation cost, lower developer ergonomics

2. **Embassy Channel** (instead of Mutex + Deque)
   - Pros: Async-friendly, built-in backpressure
   - Cons: Requires async context, more complex (7-10 days)
   - Rejected: Async complexity not justified for simple queue pattern

3. **Dynamic Vec** (instead of heapless::Deque)
   - Pros: Unlimited queue capacity
   - Cons: Heap allocation, unpredictable memory usage, fragmentation risk
   - Rejected: Violates no_std and NFR-00066 requirements

Decision Rationale:

- Global static with Mutex balances simplicity, accessibility, and performance
- heapless structures ensure predictable memory usage
- Matches existing project patterns (consistency)
- Lowest implementation cost (2-3 days vs. 5-10 days)
- See [ADR-00018-global-statusnotifier](../../adr/ADR-00018-global-statusnotifier.md) for full rationale

## Migration and Compatibility

**Backward Compatibility:**

- Existing force-arm/disarm warnings migrate to new API
- No breaking changes to MAVLink protocol
- GCS software (Mission Planner, QGroundControl) already supports MAVLink v2 chunking

**Migration Steps:**

1. Add `mavlink2` feature to `Cargo.toml`
2. Create `status_notifier.rs` module
3. Migrate `CommandHandler::create_statustext()` calls to `send_warning()`
4. Remove private `create_statustext()` function
5. Update arming system to use `send_error()` for pre-arm failures (future task)

**Rollout Plan:**

- Phase 1: Core notifier and API (internal only)
- Phase 2: Router integration (STATUSTEXT visible in GCS)
- Phase 3: Migrate existing warnings
- Phase 4: Enable system components (arming, failsafe, mode)

**Deprecation:**

- `CommandHandler::create_statustext()` removed after migration
- No user-facing API changes (internal only)

## Testing Strategy

### Unit Tests

**Queue Operations:**

- Test enqueue/dequeue cycle
- Test queue full (overflow handling)
- Test dropped count increment

**Chunking Algorithm:**

- Test single message (≤50 chars, id=0)
- Test 2-chunk message (51-100 chars)
- Test 3-chunk message (101-150 chars)
- Test 4-chunk message (151-200 chars)
- Test truncation (>200 chars)
- Test chunk ID assignment (sequential, wraparound)
- Test chunk_seq ordering

**API Functions:**

- Test all severity levels (emergency through debug)
- Test concurrent access (multiple threads)
- Test performance (<100 µs)

### Integration Tests

**End-to-End Flow:**

- Test message enqueue → drain → chunk → send
- Test integration with MavlinkRouter
- Test multiple messages in single drain cycle
- Test messages from multiple components

**GCS Compatibility:**

- Manual test with Mission Planner (chunked message reassembly)
- Manual test with QGroundControl
- Verify ArduPilot-style message prefixes display correctly

### Performance Benchmarks

**Profiling Tests:**

```rust
#[cfg(test)]
mod perf_tests {
    #[test]
    fn bench_send_error() {
        let start = Instant::now();
        for _ in 0..100 {
            send_error("Test error message");
        }
        let elapsed = start.elapsed().as_micros() / 100;
        assert!(elapsed < 100, "Average too slow: {} µs", elapsed);
    }

    #[test]
    fn bench_chunking() {
        let long_msg = "PreArm: Battery voltage 9.8V is below minimum arming voltage 10.5V configured in BATT_ARM_VOLT parameter";
        let start = Instant::now();
        let chunks = chunk_message(MavSeverity::ERROR, long_msg);
        let elapsed = start.elapsed().as_micros();
        assert!(elapsed < 50, "Chunking too slow: {} µs", elapsed);
        assert_eq!(chunks.len(), 3);
    }
}
```

## Documentation Impact

- Update `docs/mavlink.md` with STATUSTEXT notification section
- Add developer guide for using notification API
- Document message prefix conventions (ArduPilot style)
- Update architecture diagrams with StatusNotifier component

## External References

- [MAVLink STATUSTEXT Message Specification](https://mavlink.io/en/messages/common.html#STATUSTEXT)
- [MAVLink v2 Protocol Specification](https://mavlink.io/en/guide/mavlink_2.html)
- [RFC-5424 Syslog Severity Levels](https://datatracker.ietf.org/doc/html/rfc5424#section-6.2.1)
- [rust-mavlink GitHub Repository](https://github.com/mavlink/rust-mavlink)
- [heapless crate documentation](https://docs.rs/heapless/)
- [Embassy Sync Documentation](https://docs.embassy.dev/embassy-sync/)

## Open Questions

- [ ] Should EMERGENCY/ALERT bypass queue and send immediately? → Next step: Prototype both approaches in Phase 2, measure latency difference
- [ ] Should we implement message deduplication? → Method: Research ArduPilot's strategy, defer to future enhancement if not critical
- [ ] What is optimal queue size (8, 16, 32)? → Decision: Start with 16 (analysis recommendation), adjust if overflow observed in testing

## Appendix

### Diagrams

**Queue Overflow Behavior:**

```text
Queue (capacity 16):
[M1][M2][M3]...[M15][M16]

New message M17 arrives:
- Drop M1 (oldest)
- Enqueue M17
- Increment dropped_count

Result:
[M2][M3][M4]...[M16][M17]
dropped_count: 1
```

**Chunk ID Wraparound:**

```text
AtomicU16 counter:
- Start: 1
- After 65535 messages: 65535 (u16::MAX)
- Next fetch_add(1): 0 (wrapped)
- Skip 0 (reserved): set to 1
- Continue: 1, 2, 3, ...
```

### Examples

**Usage from Arming System:**

```rust
use crate::communication::mavlink::status_notifier::send_error;

fn check_battery_voltage(&self) -> ArmingCheckResult {
    if self.voltage < self.min_voltage {
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

**Host Test Example:**

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_chunking_long_message() {
        let msg = "PreArm: Battery voltage 9.8V is below minimum arming voltage 10.5V configured in BATT_ARM_VOLT parameter";
        let chunks = chunk_message(MavSeverity::ERROR, msg);

        assert_eq!(chunks.len(), 3);
        assert_eq!(chunks[0].chunk_seq, 0);
        assert_eq!(chunks[1].chunk_seq, 1);
        assert_eq!(chunks[2].chunk_seq, 2);
        assert_eq!(chunks[0].id, chunks[1].id);
        assert_eq!(chunks[1].id, chunks[2].id);
        assert_ne!(chunks[0].id, 0); // Multi-chunk uses non-zero ID
    }
}
```

### Glossary

- **STATUSTEXT**: MAVLink message type (ID 253) for human-readable status messages
- **Chunking**: Splitting long messages into multiple 50-byte segments
- **chunk_id**: Unique identifier for reassembling multi-chunk messages (MAVLink v2)
- **chunk_seq**: Sequential index of chunk within multi-chunk message (0, 1, 2, ...)
- **GCS**: Ground Control Station (Mission Planner, QGroundControl)
- **MAVLink v2**: Version 2 of MAVLink protocol with extended message support
- **heapless**: Rust crate providing fixed-capacity data structures (no heap allocation)
- **RFC-5424**: Syslog protocol specification defining severity levels (0-7)
