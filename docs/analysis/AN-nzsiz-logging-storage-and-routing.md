# AN-nzsiz Logging Storage and Routing | Multi-destination Log System

## Metadata

- Type: Analysis
- Status: Complete

## Links

- Related Analyses:
  - [AN-e8x8h-statustext-notifications](../analysis/AN-e8x8h-statustext-notifications.md)
- Related Requirements:
  - [FR-e301z-log-ring-buffer](../requirements/FR-e301z-log-ring-buffer.md)
  - [FR-c6zp6-log-statustext-routing](../requirements/FR-c6zp6-log-statustext-routing.md)
  - [FR-no1gs-log-retrieval-api](../requirements/FR-no1gs-log-retrieval-api.md)
  - [NFR-jsjz6-log-push-performance](../requirements/NFR-jsjz6-log-push-performance.md)
- Related ADRs: None
- Related Tasks:
  - [T-mrt6j-log-storage-routing](../tasks/T-mrt6j-log-storage-routing/design.md)

## Executive Summary

This analysis explores redesigning the logging system to support multiple output destinations: internal buffering for later retrieval, Mission Planner transmission via MAVLink STATUSTEXT, and USB serial streaming. The current implementation only supports USB serial streaming (when `usb_serial` feature is enabled) or defmt output via RTT, with no ability to store logs internally or transmit them via MAVLink.

Key findings: The existing `logging.rs` module provides a channel-based architecture that could be extended to support multiple sinks. The main gaps are: (1) no persistent storage mechanism, (2) no MAVLink integration, and (3) no ability to retrieve buffered logs. A multi-sink architecture with fixed routing rules (WARNING and above to STATUSTEXT, all levels to buffer) would address all requirements while maintaining simplicity and low-latency streaming capability.

## Problem Space

### Current State

**Existing Infrastructure:**

- `src/core/logging.rs` provides unified logging macros (`log_info!`, `log_warn!`, etc.)
- Three output modes based on compile-time features:
  - `pico2_w` + `usb_serial`: USB Serial via embassy channel
  - `pico2_w` only: defmt via RTT
  - Host tests: println!
- Embassy channel with 16-message capacity (`LOG_CHANNEL_SIZE`)
- Log message buffer size: 256 bytes (`LOG_MSG_SIZE`)
- `usb_logger_task` consumes channel and writes to USB CDC ACM

**Pain Points:**

1. **No Log Retention**: All logs are immediately streamed and lost if not captured
2. **No MAVLink Integration**: Cannot send logs to Mission Planner/QGroundControl
3. **No Post-hoc Retrieval**: Cannot download logs after flight for analysis
4. **Single Output Only**: Cannot simultaneously output to USB and MAVLink
5. **No Filtering by Destination**: All logs go to the same destination regardless of severity

### Desired State

**Multi-destination Logging:**

- **Internal Buffer**: Store logs in RAM ring buffer for later retrieval
- **MAVLink STATUSTEXT**: Send important logs (WARNING and above) to GCS
- **USB Serial**: Continue streaming all logs for development/debugging
- **Retrieval Mechanism**: API to read buffered logs (via MAVLink command or dedicated endpoint)

**Fixed Routing Rules:**

- All log levels stored in ring buffer (32 messages, oldest overwritten)
- WARNING and above sent to MAVLink STATUSTEXT
- All levels streamed to USB serial (when `usb_serial` feature enabled)

**Persistence Options (Future):**

- Optional flash storage for crash logs
- SD card logging for extended missions (if hardware available)

### Gap Analysis

| Component         | Current State                | Target State                       | Gap                            |
| ----------------- | ---------------------------- | ---------------------------------- | ------------------------------ |
| Log Storage       | None (streaming only)        | RAM ring buffer (32 messages)      | Need heapless ring buffer      |
| MAVLink Output    | None                         | STATUSTEXT for WARNING and above   | Need integration with AN-e8x8h |
| USB Serial Output | Single-destination streaming | One of multiple destinations       | Need multi-sink architecture   |
| Log Retrieval     | Not possible                 | Command-based retrieval API        | Need MAVLink command handler   |
| Routing Rules     | Compile-time only            | Fixed rules (hardcoded thresholds) | Simple dispatch logic          |

## Stakeholder Analysis

| Stakeholder          | Interest/Need                         | Impact | Priority |
| -------------------- | ------------------------------------- | ------ | -------- |
| Developers           | Debug logs during development         | High   | P0       |
| GCS Operators        | Real-time status notifications        | High   | P0       |
| Post-flight Analysis | Retrieve logs after mission           | High   | P0       |
| Crash Investigation  | Access logs after unexpected behavior | Medium | P1       |
| Remote Monitoring    | Status updates without USB connection | High   | P0       |

## Research & Discovery

### User Feedback

The current development workflow requires USB connection and terminal monitoring for all log access. This is impractical for:

- Field testing where USB connection is inconvenient
- Long-duration missions where USB terminal may disconnect
- Post-incident analysis when logs were not captured in real-time

### Competitive Analysis

**ArduPilot Logging Architecture:**

ArduPilot uses a sophisticated multi-layer logging system:

1. **DataFlash (Binary Logs)**: High-frequency data logging to flash/SD card
   - Structured binary format for efficient storage
   - Downloaded via MAVLink LOG_REQUEST\_\* commands
   - Primary source for post-flight analysis

2. **STATUSTEXT (Text Logs)**: Human-readable status messages to GCS
   - RFC-5424 severity levels
   - Real-time display in Mission Planner
   - Not stored persistently

3. **Debug Console**: Serial output for development
   - Hal.console->printf() style output
   - USB or UART output

**PX4 Logging:**

- ULog binary format for flight data
- Similar STATUSTEXT for GCS notifications
- MAVLink shell for interactive debug access

**Key Insight**: Successful autopilot systems separate:

- High-frequency binary logging (for analysis)
- Human-readable notifications (for operators)
- Debug output (for developers)

### Technical Investigation

**Embassy Channel Capacity:**

Current implementation uses `Channel<CriticalSectionRawMutex, LogMessage, 16>` which provides:

- Non-blocking `try_send()` for producers
- Async `receive()` for consumers
- Fixed capacity prevents unbounded memory growth

**heapless Ring Buffer Options:**

```rust
const LOG_BUFFER_SIZE: usize = 32;

// Option 1: heapless::HistoryBuffer (FIFO, overwrites oldest)
use heapless::HistoryBuffer;
static LOG_BUFFER: Mutex<CriticalSectionRawMutex, RefCell<HistoryBuffer<LogMessage, LOG_BUFFER_SIZE>>> = ...;

// Option 2: heapless::Deque (double-ended queue)
use heapless::Deque;
static LOG_BUFFER: Mutex<CriticalSectionRawMutex, RefCell<Deque<LogMessage, LOG_BUFFER_SIZE>>> = ...;
```

**Memory Budget (Fixed at 32 messages):**

- Log message: \~256 bytes (message) + 1 byte (level) = \~260 bytes
- Buffer capacity: 32 messages × 260 bytes = \~8.3 KB
- RP2350 RAM: 520 KB available, 8.3 KB is acceptable overhead

**MAVLink STATUSTEXT Integration:**

The existing STATUSTEXT notification system (AN-e8x8h) provides:

- `StatusNotifier` for queuing STATUSTEXT messages
- MAVLink v2 chunking for long messages
- Integration with MAVLink router

Logs could flow: `log_info!()` → LogSink → STATUSTEXT queue → MAVLink Router

### Data Analysis

**Expected Log Volumes:**

- **Startup**: 20-50 messages (initialization, sensor detection)
- **Armed Operation**: 1-5 messages/minute (nominal)
- **Error Conditions**: 10-50 messages (burst during issues)
- **Debug Mode**: 100+ messages/minute

**Storage Duration at 32-message buffer:**

- Nominal: 6-30 minutes of logs
- Error burst: May lose older logs within seconds
- Startup: First \~20 messages may be overwritten

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: System shall buffer logs in RAM ring buffer
  - Rationale: Enable log retrieval after events occur
  - Acceptance Criteria:
    - Fixed buffer size of 32 messages (constant `LOG_BUFFER_SIZE`)
    - FIFO with oldest-message overwrite when full
    - Thread-safe access from multiple producers
    - Non-blocking push operation

- [ ] **FR-DRAFT-2**: System shall route WARNING and above to MAVLink STATUSTEXT
  - Rationale: GCS operators need visibility into system issues
  - Acceptance Criteria:
    - WARNING and ERROR levels sent via STATUSTEXT (fixed threshold)
    - Integration with StatusNotifier from AN-e8x8h
    - INFO/DEBUG/TRACE never sent to STATUSTEXT

- [ ] **FR-DRAFT-3**: System shall continue USB serial streaming when feature enabled
  - Rationale: Maintain developer debugging capability
  - Acceptance Criteria:
    - All log levels streamed to USB serial
    - No change in existing `usb_serial` feature behavior
    - Concurrent with other destinations

- [ ] **FR-DRAFT-4**: System shall provide API to retrieve buffered logs
  - Rationale: Post-event analysis requires log access
  - Acceptance Criteria:
    - Function to iterate over buffered logs
    - Option to clear buffer after retrieval
    - Thread-safe read access
    - MAVLink command to request logs (future)

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Log push shall complete within 50 microseconds
  - Category: Performance
  - Rationale: Logging must not block real-time operations
  - Target: Non-blocking channel send with immediate return

- [ ] **NFR-DRAFT-2**: Log buffer shall use approximately 8.3 KB RAM
  - Category: Resource
  - Rationale: Preserve RAM for flight-critical operations
  - Target: Fixed 32 messages at \~260 bytes each

- [ ] **NFR-DRAFT-3**: Logging shall work without heap allocation in hot path
  - Category: Performance
  - Rationale: Embedded no_std constraint
  - Target: Use heapless data structures only

- [ ] **NFR-DRAFT-4**: Log system shall be testable on host
  - Category: Testability
  - Rationale: Unit tests need mock log destinations
  - Target: Abstract LogSink trait with test implementation

## Design Considerations

### Technical Constraints

1. **Memory Constraints:**
   - RP2350 has 520 KB RAM, but most needed for flight operations
   - Log buffer must be statically sized (no heap)
   - Trade-off between buffer size and available RAM

2. **Real-time Constraints:**
   - Log operations called from scheduler (400 Hz)
   - Must not block or introduce jitter
   - Non-blocking channel push is essential

3. **Feature Flag Complexity:**
   - Current system has complex feature gates
   - Additional destinations add more combinations
   - Need clean abstraction to manage complexity

4. **MAVLink Integration:**
   - Depends on STATUSTEXT notification system (AN-e8x8h)
   - Must not duplicate functionality
   - Clean boundary between logging and notifications

### Potential Approaches

**Option A: Multi-Consumer Channel Architecture**

- Description: Single log channel with multiple async consumers (USB, MAVLink, Buffer)
- Pros:
  - Clean separation of concerns
  - Each consumer handles its own destination
  - Leverages existing channel infrastructure
- Cons:
  - Embassy Channel is single-consumer
  - Would need broadcast pattern or multiple channels
  - More complex task coordination
- Effort: High

**Option B: LogSink Trait with Router**

- Description: Abstract LogSink trait with router that dispatches to multiple implementations
- Pros:
  - Flexible destination configuration
  - Easy to add new destinations
  - Testable with mock implementations
  - Single channel, router handles distribution
- Cons:
  - Router logic adds overhead
  - Must handle async vs sync sinks
  - Trait object or static dispatch decision
- Effort: Medium

**Option C: Direct Multi-Destination in Macros**

- Description: Expand log macros to call multiple destinations directly
- Pros:
  - Simple implementation
  - No intermediate abstractions
  - Compile-time destination selection
- Cons:
  - Complex macros with many feature gates
  - Hard to add destinations without macro changes
  - No runtime configuration
- Effort: Low (initial), High (maintenance)

**Recommendation: Option B (LogSink Trait with Router)**

- Provides clean separation between log producers and consumers
- Maintainable abstraction layer
- Fixed routing rules simplify implementation (no runtime configuration needed)
- Good balance of effort and capability

### Architecture Impact

**New Components:**

- `src/core/log_sink.rs` - LogSink trait and implementations
- `src/core/log_router.rs` - Multi-destination log router
- `src/core/log_buffer.rs` - Ring buffer for log retention

**Modified Components:**

- `src/core/logging.rs` - Integrate with router, simplify macros
- `src/communication/mavlink/status_notifier.rs` - Receive logs from router

**Integration Points:**

- Log macros push to router
- Router dispatches to configured sinks:
  - `UsbSerialSink` - existing USB output
  - `RingBufferSink` - new internal storage
  - `StatusTextSink` - bridge to STATUSTEXT (via AN-e8x8h)

## Risk Assessment

| Risk                                    | Probability | Impact | Mitigation Strategy                             |
| --------------------------------------- | ----------- | ------ | ----------------------------------------------- |
| Buffer too small for error bursts       | Medium      | Low    | 32 messages is sufficient for most cases        |
| Performance degradation from multi-sink | Low         | Medium | Benchmark each sink, optimize routing           |
| Deadlock in concurrent log access       | Medium      | High   | Use try_lock with timeout, single writer        |
| Feature flag complexity explosion       | Low         | Medium | Fixed routing rules minimize cfg gates          |
| Dependency on AN-e8x8h STATUSTEXT       | Low         | Medium | Design interface first, implement independently |

## Open Questions

- [ ] Should logs be timestamped? What time source? → Next step: Evaluate embassy_time availability and overhead
- [ ] Should we support log persistence to flash? → Next step: Analyze flash write limits and available space
- [ ] What MAVLink command should retrieve buffered logs? → Method: Research ArduPilot LOG_REQUEST commands

## Recommendations

### Immediate Actions

1. **Define LogSink Trait**: Create abstract interface for log destinations
2. **Implement RingBufferSink**: Build heapless ring buffer for log retention
3. **Integrate with AN-e8x8h**: Ensure STATUSTEXT can receive logs from router

### Next Steps

1. [ ] Create formal requirements: FR for multi-sink logging, NFR for performance constraints
2. [ ] Draft ADR for: Log router architecture and sink abstraction
3. [ ] Wait for AN-e8x8h completion: STATUSTEXT implementation provides foundation
4. [ ] Create task for: Implement multi-destination logging system
5. [ ] Prototype: Build minimal LogSink trait and test with mock implementations

### Out of Scope

- **Flash Persistence**: Deferred to future analysis (requires flash driver, wear leveling)
- **SD Card Logging**: Hardware-dependent, not currently supported
- **Binary Log Format**: ArduPilot-style DataFlash is complex; text logs sufficient initially
- **Log Compression**: Adds complexity without clear benefit at current volumes
- **Remote Log Streaming**: MAVLink transport bandwidth limited for high-volume logs

## Appendix

### References

- [ArduPilot Logging Overview](https://ardupilot.org/dev/docs/logging.html)
- [MAVLink STATUSTEXT Specification](https://mavlink.io/en/messages/common.html#STATUSTEXT)
- [Embassy Sync Primitives](https://docs.embassy.dev/embassy-sync/)
- [heapless Data Structures](https://docs.rs/heapless/)
- Project code:
  - `src/core/logging.rs` - Current logging implementation
  - AN-e8x8h-statustext-notifications - STATUSTEXT analysis

### Raw Data

**Current Log Channel Configuration:**

```rust
const LOG_MSG_SIZE: usize = 256;
const LOG_CHANNEL_SIZE: usize = 16;

pub struct LogMessage {
    pub level: LogLevel,
    pub message: heapless::String<LOG_MSG_SIZE>,
}

pub enum LogLevel {
    Info,
    Warn,
    Error,
    Debug,
    Trace,
}
```

**Proposed LogSink Trait:**

```rust
pub trait LogSink {
    /// Push log message to sink (non-blocking)
    fn push(&mut self, msg: &LogMessage) -> bool;
}
```

**Proposed Log Router (Fixed Routing Rules):**

```rust
const LOG_BUFFER_SIZE: usize = 32;

pub struct LogRouter {
    buffer_sink: RingBufferSink<LOG_BUFFER_SIZE>,
    usb_sink: Option<UsbSerialSink>,
    statustext_sink: Option<StatusTextSink>,
}

impl LogRouter {
    pub fn route(&mut self, msg: &LogMessage) {
        // Always buffer all messages
        self.buffer_sink.push(msg);

        // USB serial: all levels (when usb_serial feature enabled)
        if let Some(ref mut usb) = self.usb_sink {
            usb.push(msg);
        }

        // STATUSTEXT: WARNING and above only (fixed threshold)
        if msg.level >= LogLevel::Warn {
            if let Some(ref mut st) = self.statustext_sink {
                st.push(msg);
            }
        }
    }
}
```
