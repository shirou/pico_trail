# FR-00095 Log Retrieval API

## Metadata

- Type: Functional Requirement
- Status: Approved

## Links

- Related Analyses:
  - [AN-00026-logging-storage-and-routing](../analysis/AN-00026-logging-storage-and-routing.md)
- Prerequisite Requirements:
  - [FR-00096-log-ring-buffer](../requirements/FR-00096-log-ring-buffer.md)
- Dependent Requirements: None
- Related Tasks:
  - [T-00021-log-storage-routing](../tasks/T-00021-log-storage-routing/design.md)

## Requirement Statement

The system shall provide an API to retrieve buffered log messages from the ring buffer, with options to iterate over messages and optionally clear the buffer after retrieval.

## Rationale

Post-event analysis requires access to buffered logs. Without a retrieval API, stored logs are inaccessible and the buffering provides no value. The API must support both inspection (read-only) and consumption (read-and-clear) patterns to suit different use cases.

## User Story

As a **developer debugging rover behavior**, I want **to retrieve recent log messages from the buffer**, so that **I can analyze what happened during an autonomous mission or after an unexpected event**.

## Acceptance Criteria

- [ ] Function to iterate over all buffered messages (oldest to newest)
- [ ] Function to get current buffer message count
- [ ] Function to get overflow count (messages lost due to full buffer)
- [ ] Option to clear buffer after retrieval
- [ ] Thread-safe read access
- [ ] Non-blocking operation
- [ ] Messages returned in chronological order (FIFO)

## Technical Details

### Functional Requirement Details

**Retrieval API:**

```rust
impl RingBufferSink {
    /// Get number of messages currently in buffer
    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    /// Get count of messages lost due to overflow
    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    /// Iterate over messages (oldest to newest)
    pub fn iter(&self) -> impl Iterator<Item = &LogMessage> {
        self.buffer.oldest_ordered()
    }

    /// Clear all messages from buffer
    pub fn clear(&mut self) {
        self.buffer.clear();
    }

    /// Get all messages and clear buffer
    pub fn drain(&mut self) -> heapless::Vec<LogMessage, LOG_BUFFER_SIZE> {
        let mut result = heapless::Vec::new();
        for msg in self.buffer.oldest_ordered() {
            let _ = result.push(msg.clone());
        }
        self.buffer.clear();
        result
    }
}
```

**Global Access Pattern:**

```rust
// Access via global log router
pub fn get_buffered_logs() -> heapless::Vec<LogMessage, LOG_BUFFER_SIZE> {
    LOG_ROUTER.lock(|router| {
        router.buffer_sink.drain()
    })
}

pub fn peek_buffered_logs<F, R>(f: F) -> R
where
    F: FnOnce(&RingBufferSink) -> R,
{
    LOG_ROUTER.lock(|router| f(&router.buffer_sink))
}
```

**Future MAVLink Integration (Out of Scope):**

A future enhancement could add MAVLink commands to request logs:

- LOG_REQUEST_LIST: Request log buffer status
- LOG_REQUEST_DATA: Download log messages

This is documented but not part of the current requirement.

## Platform Considerations

### Embedded (RP2350)

- Uses heapless::Vec for drain result
- Protected by Mutex for thread-safe access
- Short critical section to minimize lock contention

### Host Tests

- Same API works on host
- Can verify message ordering and drain behavior
- Test with mock router

### Cross-Platform

- API identical on all platforms
- Ordering guaranteed consistent

## Risks & Mitigation

| Risk                               | Impact | Likelihood | Mitigation                               | Validation                |
| ---------------------------------- | ------ | ---------- | ---------------------------------------- | ------------------------- |
| Long lock hold during iteration    | Medium | Medium     | Use short critical sections, copy to Vec | Benchmark lock duration   |
| Stack overflow from 32-message Vec | Low    | Low        | heapless::Vec is stack-safe              | Test on target hardware   |
| Race condition during drain        | High   | Low        | Mutex protection, single-writer pattern  | Concurrent access testing |

## Implementation Notes

**Preferred Patterns:**

- Use `oldest_ordered()` for chronological iteration
- Copy messages to heapless::Vec to minimize lock time
- Provide both peek (read-only) and drain (read-and-clear) options

**Known Pitfalls:**

- Do not hold lock while processing messages (copy first)
- Do not use std::Vec (requires heap)

**Related Code:**

- Ring buffer: `src/core/log_buffer.rs` (to be created)
- heapless crate: `heapless::HistoryBuffer::oldest_ordered()`

## External References

- [heapless HistoryBuffer::oldest_ordered](https://docs.rs/heapless/latest/heapless/struct.HistoryBuffer.html#method.oldest_ordered)
