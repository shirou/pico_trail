# FR-e301z RAM Ring Buffer Log Storage

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-nzsiz-logging-storage-and-routing](../analysis/AN-nzsiz-logging-storage-and-routing.md)
- Prerequisite Requirements: None
- Dependent Requirements:
  - [FR-c6zp6-log-statustext-routing](../requirements/FR-c6zp6-log-statustext-routing.md)
  - [FR-no1gs-log-retrieval-api](../requirements/FR-no1gs-log-retrieval-api.md)
  - [NFR-jsjz6-log-push-performance](../requirements/NFR-jsjz6-log-push-performance.md)
- Related Tasks:
  - [T-mrt6j-log-storage-routing](../tasks/T-mrt6j-log-storage-routing/design.md)

## Requirement Statement

The system shall buffer all log messages in a RAM-based ring buffer with a fixed capacity of 32 messages, overwriting the oldest messages when the buffer is full.

## Rationale

Log retention enables post-event analysis and debugging. When issues occur during field testing or autonomous operation, having access to recent log messages is critical for understanding what happened. A ring buffer provides bounded memory usage while automatically managing storage by discarding old messages.

## User Story

As a **developer or operator analyzing rover behavior**, I want **recent log messages to be retained in memory**, so that **I can retrieve and examine logs after an event occurs without requiring a USB connection during the event**.

## Acceptance Criteria

- [ ] Ring buffer capacity fixed at 32 messages (constant `LOG_BUFFER_SIZE`)
- [ ] All log levels (TRACE, DEBUG, INFO, WARN, ERROR) stored in buffer
- [ ] FIFO eviction: oldest message overwritten when buffer full
- [ ] Thread-safe access from multiple producers (log macros)
- [ ] Non-blocking push operation (never waits for space)
- [ ] Buffer state accessible (message count, overflow count)
- [ ] Uses heapless data structures (no dynamic allocation)

## Technical Details

### Functional Requirement Details

**Buffer Data Structure:**

```rust
use heapless::HistoryBuffer;

const LOG_BUFFER_SIZE: usize = 32;
const LOG_MSG_SIZE: usize = 256;

pub struct LogMessage {
    pub level: LogLevel,
    pub message: heapless::String<LOG_MSG_SIZE>,
}

pub struct RingBufferSink {
    buffer: HistoryBuffer<LogMessage, LOG_BUFFER_SIZE>,
    overflow_count: u32,
}
```

**Buffer Operations:**

```rust
impl RingBufferSink {
    /// Push message to buffer (overwrites oldest if full)
    pub fn push(&mut self, msg: LogMessage) {
        if self.buffer.len() == LOG_BUFFER_SIZE {
            self.overflow_count += 1;
        }
        self.buffer.write(msg);
    }

    /// Get number of messages in buffer
    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    /// Get count of messages lost due to overflow
    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }
}
```

**Memory Budget:**

- LogMessage: \~257 bytes (256 message + 1 level)
- Buffer: 32 × 257 = \~8.2 KB
- Overhead: \~50 bytes (counters, pointers)
- Total: \~8.3 KB RAM

## Platform Considerations

### Embedded (RP2350)

- Uses `heapless::HistoryBuffer` for fixed-capacity storage
- Protected by `Mutex<CriticalSectionRawMutex, RefCell<...>>`
- No heap allocation in any code path

### Host Tests

- Same implementation works on host
- Mock can verify buffer ordering and overflow behavior

### Cross-Platform

- Buffer behavior identical on all platforms
- Message ordering guaranteed consistent

## Risks & Mitigation

| Risk                                     | Impact | Likelihood | Mitigation                                               | Validation                 |
| ---------------------------------------- | ------ | ---------- | -------------------------------------------------------- | -------------------------- |
| 32 messages too small for error bursts   | Low    | Medium     | Sufficient for most scenarios; oldest lost is acceptable | Test with 50-message burst |
| Memory overhead too high                 | Low    | Low        | 8.3 KB is small relative to 520 KB available             | Memory profiling           |
| Thread contention blocks real-time tasks | High   | Low        | Non-blocking push, short critical sections               | Latency benchmarking       |

## Implementation Notes

**Preferred Patterns:**

- Use `heapless::HistoryBuffer` for automatic FIFO eviction
- Track overflow count for diagnostics
- Keep critical sections minimal

**Known Pitfalls:**

- Do not use `heapless::Deque` (requires manual eviction)
- Do not allocate strings on heap before copying to buffer

**Related Code:**

- Current logging: `src/core/logging.rs`
- heapless crate: `heapless::HistoryBuffer`

## External References

- [heapless HistoryBuffer documentation](https://docs.rs/heapless/latest/heapless/struct.HistoryBuffer.html)
