# T-00021 Multi-Destination Log Storage and Routing

## Metadata

- Type: Design
- Status: Complete

## Links

- Associated Plan Document:
  - [T-00021-log-storage-routing-plan](plan.md)

## Overview

This design implements a multi-destination logging system that stores logs in a RAM ring buffer, routes WARNING/ERROR messages to MAVLink STATUSTEXT, and maintains USB serial streaming. The system extends the existing `logging.rs` infrastructure with a LogRouter that dispatches to multiple sinks: RingBufferSink for retention, StatusTextSink for GCS notifications, and the existing USB serial output. The design prioritizes no-allocation embedded compatibility, sub-50µs performance, and clean separation between logging and notification subsystems.

## Success Metrics

- [x] Log push completes within 50 µs worst-case on RP2350 @ 150 MHz (no observable latency)
- [x] Zero heap allocations in hot path (verified via code inspection)
- [x] Ring buffer uses approximately 8.3 KB RAM (32 × \~260 bytes)
- [x] WARNING and ERROR logs appear in Mission Planner HUD
- [x] All 4 requirements' acceptance criteria satisfied
- [x] No regressions in existing logging behavior

## Background and Current State

- Context: Logging system in pico_trail provides debug output via USB serial or RTT
- Current behavior:
  - `src/core/logging.rs` provides unified macros (`log_info!`, `log_warn!`, etc.)
  - Three output modes based on compile-time features:
    - `pico2_w` + `usb_serial`: USB Serial via embassy channel
    - `pico2_w` only: defmt via RTT
    - Host tests: println!
  - Embassy channel with 16-message capacity
  - Log message buffer size: 256 bytes
- Pain points:
  - No log retention: all logs immediately streamed and lost
  - No MAVLink integration: cannot send logs to GCS
  - No post-hoc retrieval: cannot download logs after mission
  - Single output only: cannot simultaneously output to USB and MAVLink
- Constraints:
  - Embedded target (RP2350) with 520 KB RAM
  - Must use no_std (no heap allocation in hot path)
  - Real-time scheduler at 400 Hz (2.5 ms period)
  - Log calls during time-critical operations
- Related Analysis: [AN-00026-logging-storage-and-routing](../../analysis/AN-00026-logging-storage-and-routing.md)

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
│       │log_warn!()  │ log_error!() │ log_info!() │             │
│       └─────────────┴──────────────┴─────────────┘              │
│                          │                                      │
└──────────────────────────┼──────────────────────────────────────┘
                           ▼
         ┌─────────────────────────────────────┐
         │     Global LogRouter (Static)       │
         │   Mutex<CriticalSectionRawMutex>    │
         │                                     │
         │  route(LogMessage):                 │
         │  ├─► RingBufferSink (all levels)   │
         │  ├─► StatusTextSink (WARN/ERROR)   │
         │  └─► UsbSerialSink (all, optional) │
         └─────────────┬───────────────────────┘
                       │
         ┌─────────────┼─────────────────────────┐
         │             │                         │
         ▼             ▼                         ▼
┌─────────────┐ ┌─────────────────┐ ┌─────────────────────────┐
│RingBufferSink│ │ StatusTextSink │ │     UsbSerialSink       │
│             │ │                 │ │  (usb_serial feature)   │
│HistoryBuffer│ │ StatusNotifier  │ │                         │
│  <32 msgs>  │ │   Integration   │ │   Existing USB CDC      │
│             │ │  (WARN/ERROR)   │ │                         │
└─────────────┘ └────────┬────────┘ └─────────────────────────┘
      │                  │
      │                  ▼
      │         ┌─────────────────────────────────────┐
      │         │     MAVLink Router (Telemetry)      │
      │         │     STATUSTEXT transmission         │
      │         └─────────────────────────────────────┘
      │                  │
      ▼                  ▼
┌──────────────┐  ┌─────────────────────────────────────┐
│Log Retrieval │  │  Ground Control Station (GCS)       │
│    API       │  │  - Mission Planner                  │
│              │  │  - QGroundControl                   │
└──────────────┘  └─────────────────────────────────────┘
```

### Components

**1. LogRouter (`src/core/log_router.rs`)**

- Responsibilities:
  - Dispatch log messages to multiple sinks
  - Apply fixed routing rules (all to buffer, WARN/ERROR to STATUSTEXT)
  - Maintain global static with thread-safe access
- Key structures:
  - `LogRouter`: Main struct with sink references
  - `LOG_ROUTER`: Global static `Mutex<CriticalSectionRawMutex, RefCell<LogRouter>>`
- Routing rules (hardcoded):
  - All levels → RingBufferSink
  - WARN and ERROR → StatusTextSink
  - All levels → UsbSerialSink (when `usb_serial` feature enabled)

**2. RingBufferSink (`src/core/log_buffer.rs`)**

- Responsibilities:
  - Store logs in fixed-capacity ring buffer
  - Automatically evict oldest when full
  - Track overflow count for diagnostics
  - Provide retrieval API (iterate, drain, clear)
- Key structures:
  - `RingBufferSink`: Wrapper around `heapless::HistoryBuffer`
  - Buffer capacity: 32 messages (constant `LOG_BUFFER_SIZE`)
  - Message size: \~260 bytes (256 text + level + padding)

**3. StatusTextSink (`src/core/log_statustext.rs`)**

- Responsibilities:
  - Bridge logs to StatusNotifier from T-00014
  - Filter: only forward WARN and ERROR levels
  - Convert LogLevel to MavSeverity
- Integration:
  - Uses existing `status_notifier::send_warning()` and `send_error()`
  - No queue duplication (StatusNotifier has its own queue)

**4. Log Retrieval API (`src/core/log_buffer.rs`)**

- Responsibilities:
  - Provide iteration over buffered messages
  - Support read-only peek and read-and-clear drain
  - Report buffer statistics (count, overflow)
- Functions:
  - `get_buffered_logs()`: Drain all messages
  - `peek_buffered_logs(f)`: Read without clearing
  - `buffer_len()`: Current message count
  - `overflow_count()`: Messages lost due to full buffer

### Data Flow

1. **Log Macro Invocation**:
   - Component calls `log_warn!("Battery voltage low")`
   - Macro formats message to `heapless::String<256>`
   - Calls `LOG_ROUTER.route(LogMessage { level, message })`

2. **Router Dispatch**:
   - Router acquires mutex lock (CriticalSection)
   - Pushes to RingBufferSink (all levels)
   - If level >= WARN: calls StatusTextSink.push()
   - If `usb_serial` feature: calls UsbSerialSink.push()
   - Releases mutex lock
   - Total: <50 µs

3. **STATUSTEXT Transmission**:
   - StatusTextSink converts LogLevel to MavSeverity
   - Calls `status_notifier::send_warning()` or `send_error()`
   - StatusNotifier queues for transmission
   - MAVLink router drains and sends to GCS

4. **Log Retrieval**:
   - User code calls `get_buffered_logs()`
   - Router acquires lock, drains RingBufferSink
   - Returns `heapless::Vec<LogMessage, 32>`
   - Messages available for analysis

### Data Models and Types

**LogMessage Structure (existing):**

```rust
pub const LOG_MSG_SIZE: usize = 256;

#[derive(Clone)]
pub struct LogMessage {
    pub level: LogLevel,
    pub message: heapless::String<LOG_MSG_SIZE>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
}
```

**RingBufferSink Structure:**

```rust
use heapless::HistoryBuffer;

pub const LOG_BUFFER_SIZE: usize = 32;

pub struct RingBufferSink {
    buffer: HistoryBuffer<LogMessage, LOG_BUFFER_SIZE>,
    overflow_count: u32,
}

impl RingBufferSink {
    pub const fn new() -> Self {
        Self {
            buffer: HistoryBuffer::new(),
            overflow_count: 0,
        }
    }

    pub fn push(&mut self, msg: LogMessage) {
        if self.buffer.len() == LOG_BUFFER_SIZE {
            self.overflow_count += 1;
        }
        self.buffer.write(msg);
    }

    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    pub fn iter(&self) -> impl Iterator<Item = &LogMessage> {
        self.buffer.oldest_ordered()
    }

    pub fn drain(&mut self) -> heapless::Vec<LogMessage, LOG_BUFFER_SIZE> {
        let mut result = heapless::Vec::new();
        for msg in self.buffer.oldest_ordered() {
            let _ = result.push(msg.clone());
        }
        self.buffer.clear();
        result
    }

    pub fn clear(&mut self) {
        self.buffer.clear();
    }
}
```

**LogRouter Structure:**

```rust
pub struct LogRouter {
    buffer_sink: RingBufferSink,
}

static LOG_ROUTER: Mutex<CriticalSectionRawMutex, RefCell<LogRouter>> =
    Mutex::new(RefCell::new(LogRouter::new()));

impl LogRouter {
    pub const fn new() -> Self {
        Self {
            buffer_sink: RingBufferSink::new(),
        }
    }

    pub fn route(&mut self, msg: LogMessage) {
        // Always buffer all messages
        self.buffer_sink.push(msg.clone());

        // STATUSTEXT: WARNING and above only (fixed threshold)
        if msg.level >= LogLevel::Warn {
            route_to_statustext(&msg);
        }

        // USB serial handled separately via existing channel
    }
}

fn route_to_statustext(msg: &LogMessage) {
    match msg.level {
        LogLevel::Warn => status_notifier::send_warning(&msg.message),
        LogLevel::Error => status_notifier::send_error(&msg.message),
        _ => {} // INFO, DEBUG, TRACE not routed
    }
}
```

**Public Retrieval API:**

```rust
pub fn get_buffered_logs() -> heapless::Vec<LogMessage, LOG_BUFFER_SIZE> {
    LOG_ROUTER.lock(|router| {
        router.borrow_mut().buffer_sink.drain()
    })
}

pub fn peek_buffered_logs<F, R>(f: F) -> R
where
    F: FnOnce(&RingBufferSink) -> R,
{
    LOG_ROUTER.lock(|router| f(&router.borrow().buffer_sink))
}

pub fn buffer_len() -> usize {
    LOG_ROUTER.lock(|router| router.borrow().buffer_sink.len())
}

pub fn overflow_count() -> u32 {
    LOG_ROUTER.lock(|router| router.borrow().buffer_sink.overflow_count())
}
```

### Error Handling

- **Buffer Full**:
  - Oldest message automatically evicted (HistoryBuffer behavior)
  - Increment `overflow_count` for diagnostics
  - Continue operation (non-fatal)

- **Message Too Long (>256 chars)**:
  - Already handled by existing logging macros (truncation)
  - No additional handling needed in router

- **StatusNotifier Queue Full**:
  - Handled by StatusNotifier (drops oldest, tracks count)
  - Router does not retry or block

- **Mutex Contention**:
  - CriticalSection provides non-blocking access
  - Short critical sections minimize contention

### Security Considerations

- Log messages may contain parameter values visible in GCS
- Do not log secrets, keys, or sensitive configuration
- Existing documentation warns against logging sensitive data

### Performance Considerations

**Hot Path:**

- `LOG_ROUTER.route()`: Called from log macros in scheduler tasks
  - Target: <50 µs worst-case
  - Breakdown:
    - Format message: \~20 µs (existing)
    - Acquire mutex: \~5 µs
    - Push to buffer: \~10 µs
    - StatusText routing: \~10 µs
    - Release mutex: \~5 µs

**Memory Budget:**

- RingBufferSink: 32 × \~260 bytes = \~8.3 KB
- LogRouter overhead: \~50 bytes
- Total: \~8.4 KB (acceptable for 520 KB available)

**Optimizations:**

- No heap allocation in any code path
- O(1) buffer push with `HistoryBuffer::write()`
- Minimal critical section duration
- Clone message only when needed for STATUSTEXT

### Platform Considerations

#### Embedded (RP2350)

- Uses `embassy_sync::blocking_mutex::Mutex` with `CriticalSectionRawMutex`
- heapless structures stored in static `.bss` section
- No heap allocator required
- Compatible with embassy async runtime
- Integrates with existing StatusNotifier

#### Host Tests

- Same implementation works on host
- Can verify buffer ordering and overflow behavior
- Mock StatusNotifier for isolated testing
- Performance tests verify O(1) complexity

#### Cross-Platform

- API identical on all platforms
- Buffer behavior consistent
- Message ordering guaranteed

## Alternatives Considered

1. **Embassy Channel Multi-Consumer** (instead of LogRouter with sinks)
   - Pros: Leverages existing async patterns
   - Cons: Embassy Channel is single-consumer, would need broadcast
   - Rejected: Higher complexity, not needed for simple dispatch

2. **Direct Multi-Destination in Macros** (instead of LogRouter)
   - Pros: Simple, no intermediate abstraction
   - Cons: Complex macros with many feature gates, hard to maintain
   - Rejected: Maintenance burden, inflexible

3. **Dynamic Sink Registration** (instead of fixed sinks)
   - Pros: Maximum flexibility
   - Cons: Heap allocation or complex static arrays, runtime overhead
   - Rejected: Overkill for fixed routing requirements

Decision Rationale:

- LogRouter with fixed sinks provides clean separation
- Matches existing StatusNotifier pattern (consistency)
- Lowest complexity for current requirements
- Fixed routing rules eliminate configuration overhead

## Migration and Compatibility

**Backward Compatibility:**

- Existing log macros continue to work
- USB serial output unchanged
- No breaking changes to public API

**Migration Steps:**

1. Create `log_buffer.rs` with RingBufferSink
2. Create `log_router.rs` with LogRouter
3. Integrate router with existing log macros
4. Add StatusTextSink bridge to StatusNotifier
5. Add retrieval API functions

**Rollout Plan:**

- Phase 1: RingBufferSink and retrieval API (internal testing)
- Phase 2: LogRouter integration with existing logging
- Phase 3: StatusTextSink bridge to StatusNotifier
- Phase 4: Testing and validation

## Testing Strategy

### Unit Tests

**RingBufferSink:**

- Test push/pop cycle
- Test buffer full (32 messages)
- Test overflow count increment
- Test oldest-first iteration order
- Test drain returns all messages
- Test clear empties buffer

**LogRouter:**

- Test routing to buffer (all levels)
- Test routing to STATUSTEXT (WARN/ERROR only)
- Test INFO/DEBUG/TRACE not routed to STATUSTEXT
- Test concurrent access (thread safety)

**Retrieval API:**

- Test `get_buffered_logs()` drains buffer
- Test `peek_buffered_logs()` preserves buffer
- Test `buffer_len()` accuracy
- Test `overflow_count()` accuracy

### Integration Tests

- Test end-to-end: log_warn!() → buffer → STATUSTEXT
- Test multiple log levels in sequence
- Test buffer overflow during burst logging
- Test retrieval after overflow

### Performance Benchmarks

```rust
#[cfg(test)]
fn benchmark_log_push() {
    let start = Instant::now();
    for _ in 0..100 {
        log_info!("test message with value: {}", 42);
    }
    let elapsed = start.elapsed().as_micros() / 100;
    assert!(elapsed < 50, "Average too slow: {} µs", elapsed);
}
```

## Documentation Impact

- Update `docs/architecture.md` with log_router component
- Add developer guide for log retrieval API
- Document routing rules (WARN/ERROR to STATUSTEXT)

## External References

- [heapless HistoryBuffer documentation](https://docs.rs/heapless/latest/heapless/struct.HistoryBuffer.html)
- [Embassy Sync Primitives](https://docs.embassy.dev/embassy-sync/)
- [MAVLink STATUSTEXT Specification](https://mavlink.io/en/messages/common.html#STATUSTEXT)
- Project code:
  - `src/core/logging.rs` - Current logging implementation
  - `src/communication/mavlink/status_notifier.rs` - STATUSTEXT notifier

## Open Questions

- [ ] Should logs include timestamps? → Method: Evaluate `embassy_time::Instant` overhead
- [ ] Should we support future flash persistence? → Decision: Design interface to allow future extension
- [ ] What MAVLink command should retrieve logs? → Defer: Out of scope for initial implementation
