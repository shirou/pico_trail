# AN-00020 MAVLink Status Notifications | STATUSTEXT Message Support

## Metadata

- Type: Analysis
- Status: Draft

## Links

- Related Analyses: None
- Related Requirements:
  - [FR-00073-statustext-public-api](../requirements/FR-00073-statustext-public-api.md)
  - [FR-00071-statustext-chunking](../requirements/FR-00071-statustext-chunking.md)
  - [FR-00072-statustext-message-queue](../requirements/FR-00072-statustext-message-queue.md)
  - [FR-00074-statustext-router-integration](../requirements/FR-00074-statustext-router-integration.md)
  - [FR-00070-statustext-ardupilot-conventions](../requirements/FR-00070-statustext-ardupilot-conventions.md)
  - [NFR-00066-statustext-performance-memory](../requirements/NFR-00066-statustext-performance-memory.md)
  - [NFR-00065-statustext-nostd](../requirements/NFR-00065-statustext-nostd.md)
  - [NFR-00064-statustext-length-limits](../requirements/NFR-00064-statustext-length-limits.md)
  - [NFR-00063-statustext-host-tests](../requirements/NFR-00063-statustext-host-tests.md)
- Related ADRs:
  - [ADR-00018-global-statusnotifier](../adr/ADR-00018-global-statusnotifier.md)
- Related Tasks:
  - [T-00014-statustext-implementation](../tasks/T-00014-statustext-implementation/README.md)

## Executive Summary

This analysis investigates implementing a public API for sending STATUSTEXT messages to ground control stations like Mission Planner. While the project already has internal STATUSTEXT support for force-arm/disarm warnings, there is no general-purpose mechanism for system components to send status notifications, error reports, or informational messages to operators. This document examines how to create a centralized status notification system following ArduPilot conventions, enabling components (arming system, sensors, failsafes, modes) to communicate with GCS operators.

Key findings: A private `create_statustext()` function exists in `CommandHandler` but is not accessible to other system components. Implementation requires extracting this into a shared notification API with support for MAVLink v2 chunking for messages exceeding 50 characters, severity level mapping, and integration with the telemetry streaming system.

## Problem Space

### Current State

**Existing Infrastructure:**

- `CommandHandler::create_statustext()` private function at `src/communication/mavlink/handlers/command.rs:275`
- STATUSTEXT messages sent for force-arm/disarm warnings at `src/communication/mavlink/handlers/command.rs:135` and `src/communication/mavlink/handlers/command.rs:168`
- STATUSTEXT_DATA imported from `mavlink::common` at `src/communication/mavlink/handlers/command.rs:30`
- Message dispatcher supports additional messages alongside COMMAND_ACK at `src/communication/mavlink/dispatcher.rs:125`

**Pain Points:**

1. **No Public API**: System components (arming checks, failsafes, sensors) cannot send status messages to GCS
2. **Limited to Command Handler**: Only force-arm/disarm operations can generate STATUSTEXT messages
3. **No MAVLink v2 Support**: Current implementation omits `id` and `chunk_seq` fields required for long messages
4. **No Severity Mapping**: Limited use of severity levels (only MAV_SEVERITY_WARNING currently used)
5. **No Message Queue**: STATUSTEXT messages must be generated synchronously during command handling

### Desired State

**Public Notification API:**

- Centralized `StatusNotifier` or similar component that any system module can access
- Simple API like `send_info()`, `send_warning()`, `send_error()` for common severity levels
- Support for long messages via automatic chunking (MAVLink v2 protocol)
- Thread-safe message queue for async notification generation

**System Integration:**

- Arming system sends pre-arm check failures to GCS (e.g., "PreArm: Battery voltage low")
- Failsafe system reports failsafe triggers (e.g., "Failsafe: GCS connection lost")
- Mode system announces mode changes (e.g., "Mode changed to MANUAL")
- Sensor drivers report hardware errors (e.g., "Error: IMU initialization failed")
- Parameter system confirms parameter changes (e.g., "Param: ARMING_CHECK set to 1")

**ArduPilot Compatibility:**

- Follow ArduPilot STATUSTEXT conventions (severity levels, message prefixes)
- Support streaming STATUSTEXT via MAVLink router
- Display correctly in Mission Planner, QGroundControl, and other GCS software

### Gap Analysis

| Component              | Current State                     | Target State                          | Gap                                           |
| ---------------------- | --------------------------------- | ------------------------------------- | --------------------------------------------- |
| Status API             | Private helper in CommandHandler  | Public notification service           | Need to extract and generalize                |
| MAVLink v2 Support     | Only severity + text fields       | Add id + chunk_seq for chunking       | Need to implement long message support        |
| Message Queue          | Synchronous only                  | Async notification queue              | Need heapless queue for pending notifications |
| Severity Levels        | Only WARNING used                 | Full RFC-5424 severity mapping        | Need helper functions for all levels          |
| System Integration     | Force-arm/disarm only             | All components can send notifications | Need shared access pattern                    |
| Streaming Support      | Generated only during command     | Stream via telemetry handler          | Need integration with router                  |
| Message Chunking       | Manual truncation to 50 chars     | Automatic chunking for long messages  | Need chunking algorithm                       |
| Testing Infrastructure | No dedicated tests for STATUSTEXT | Unit tests for notification API       | Need test coverage                            |

## Stakeholder Analysis

| Stakeholder                                    | Interest/Need                                  | Impact | Priority |
| ---------------------------------------------- | ---------------------------------------------- | ------ | -------- |
| Rover operators                                | Real-time status messages for troubleshooting  | High   | P0       |
| Ground control software (Mission Planner, QGC) | Standard STATUSTEXT messages for display       | High   | P0       |
| System developers                              | Easy API to report errors and status from code | High   | P0       |
| Arming system                                  | Report pre-arm check failures to operator      | High   | P0       |
| Failsafe system                                | Notify operator of failsafe triggers           | High   | P0       |
| Mode system                                    | Announce mode changes and failures             | Medium | P1       |
| Parameter system                               | Confirm parameter changes to operator          | Medium | P1       |

## Research & Discovery

### User Feedback

STATUSTEXT is a fundamental MAVLink feature expected by all GCS operators. ArduPilot sends hundreds of STATUSTEXT messages during normal operation (startup messages, pre-arm checks, mode changes, errors). Users rely on these messages for troubleshooting and situational awareness.

### Competitive Analysis

**ArduPilot STATUSTEXT Usage Patterns:**

ArduPilot extensively uses STATUSTEXT for:

- **Startup messages**: "APM:Copter V4.3.0", "Frame: QUAD"
- **Pre-arm checks**: "PreArm: Gyro not calibrated", "PreArm: Battery below minimum"
- **Mode changes**: "Mode changed to STABILIZE", "Unable to enter AUTO mode"
- **Failsafe events**: "Failsafe: GCS", "Failsafe: Battery"
- **Parameter changes**: "Param: ARMING_CHECK = 1"
- **Calibration status**: "Compass calibration started", "Gyro calibration complete"
- **Mission events**: "Mission started", "Waypoint reached"
- **Error reporting**: "Error: GPS lost", "Error: Motor failure"

**Message Prefix Conventions:**

ArduPilot uses consistent prefixes:

- `PreArm:` - Pre-arm check failures
- `Failsafe:` - Failsafe triggers
- `Mode:` - Mode changes
- `Param:` - Parameter changes
- `Error:` - Error conditions
- `Warning:` - Warning conditions

**Severity Level Mapping (RFC-5424):**

| Level | Name          | ArduPilot Usage                                           |
| ----- | ------------- | --------------------------------------------------------- |
| 0     | EMERGENCY     | Critical system failure requiring immediate landing       |
| 1     | ALERT         | Serious error requiring immediate attention               |
| 2     | CRITICAL      | Critical hardware failure (motor, sensor)                 |
| 3     | ERROR         | Error conditions (GPS lost, pre-arm failure)              |
| 4     | WARNING       | Warning conditions (battery low, mode change failed)      |
| 5     | NOTICE        | Normal but significant events (mode change, armed status) |
| 6     | INFORMATIONAL | Informational messages (startup, calibration)             |
| 7     | DEBUG         | Debug-level messages (usually disabled)                   |

### Technical Investigation

**MAVLink v2 STATUSTEXT Specification:**

```c
// Message ID: 253
typedef struct __mavlink_statustext_t {
    uint8_t severity;      // RFC-5424 severity level (0-7)
    char text[50];         // Status message without null termination
    uint16_t id;           // Unique ID for message reassembly (0 = single message)
    uint8_t chunk_seq;     // Chunk sequence number (0-indexed)
} mavlink_statustext_t;
```

**Chunking Protocol:**

For messages longer than 50 characters:

1. Assign unique `id` (non-zero)
2. Split message into 50-character chunks
3. Send chunks with sequential `chunk_seq` (0, 1, 2, ...)
4. Last chunk contains null character to signal completion
5. GCS reassembles chunks using `id` to match

**Example Long Message:**

Message: "PreArm: Battery voltage 9.8V is below minimum threshold 10.5V configured in BATT_ARM_VOLT parameter"

```
Chunk 0: "PreArm: Battery voltage 9.8V is below minimum t" (id=1, chunk_seq=0)
Chunk 1: "hreshold 10.5V configured in BATT_ARM_VOLT para" (id=1, chunk_seq=1)
Chunk 2: "meter\0" (id=1, chunk_seq=2)
```

**Existing Code Pattern:**

Current implementation at `src/communication/mavlink/handlers/command.rs:275`:

```rust
fn create_statustext(severity: MavSeverity, text: &str) -> STATUSTEXT_DATA {
    let mut text_bytes = [0u8; 50];
    let bytes = text.as_bytes();
    let len = bytes.len().min(50);
    text_bytes[..len].copy_from_slice(&bytes[..len]);

    STATUSTEXT_DATA {
        severity,
        text: text_bytes.into(),
    }
}
```

**Limitations:**

- Only fills `severity` and `text` fields
- No `id` or `chunk_seq` (MAVLink v1 format)
- Truncates messages longer than 50 characters without chunking
- Private function within CommandHandler

**rust-mavlink Library Support:**

The `mavlink` crate (rust-mavlink) provides STATUSTEXT_DATA with all fields:

```rust
pub struct STATUSTEXT_DATA {
    pub severity: MavSeverity,
    pub text: MavArray<u8, 50>,
    #[cfg(feature = "mavlink2")]
    pub id: u16,
    #[cfg(feature = "mavlink2")]
    pub chunk_seq: u8,
}
```

The `id` and `chunk_seq` fields are behind `mavlink2` feature flag.

### Data Analysis

**Message Frequency in ArduPilot:**

- **Startup**: 10-20 messages during initialization
- **Pre-arm**: 1-5 messages per arming attempt with failures
- **Armed operation**: 1-2 messages per minute (nominal)
- **Failsafe**: 1-3 messages per failsafe event
- **Mode change**: 1-2 messages per mode change attempt
- **Error conditions**: Variable (1-10 messages during fault)

**Recommended Design:**

- **Queue depth**: 8-16 messages (typical burst of pre-arm checks or startup messages)
- **Message priority**: EMERGENCY/ALERT bypass queue and send immediately
- **Deduplication**: Suppress repeated identical messages within time window
- **Rate limiting**: Limit messages to prevent flooding GCS (10 messages/second max)

## Discovered Requirements

### Functional Requirements (Potential)

- [ ] **FR-DRAFT-1**: System shall provide public API for sending STATUSTEXT messages
  - Rationale: All system components need ability to report status to GCS operators
  - Acceptance Criteria:
    - Public functions `send_emergency()`, `send_alert()`, `send_critical()`, `send_error()`, `send_warning()`, `send_notice()`, `send_info()`, `send_debug()`
    - Each function accepts `&str` message text
    - API accessible from arming system, failsafes, modes, sensors
    - Thread-safe (can be called from interrupt handlers with proper guards)

- [ ] **FR-DRAFT-2**: System shall support MAVLink v2 chunking for long messages
  - Rationale: Error messages with parameter names and values often exceed 50 characters
  - Acceptance Criteria:
    - Messages up to 50 chars sent as single STATUSTEXT (id=0)
    - Messages 51-100 chars split into 2 chunks with same id
    - Messages 101-200 chars split into 4 chunks with same id
    - Chunks sent with sequential chunk_seq (0, 1, 2, ...)
    - Last chunk padded with null bytes

- [ ] **FR-DRAFT-3**: System shall queue STATUSTEXT messages for async transmission
  - Rationale: Status messages may be generated during time-critical operations
  - Acceptance Criteria:
    - Heapless queue with 16-message capacity
    - Messages queued by notification API, sent by telemetry handler
    - EMERGENCY/ALERT severity bypass queue and send immediately
    - Queue full condition logs warning and drops oldest message

- [ ] **FR-DRAFT-4**: System shall integrate STATUSTEXT with MAVLink router
  - Rationale: GCS must receive messages via existing transport (UART/USB)
  - Acceptance Criteria:
    - STATUSTEXT messages sent via MavlinkRouter
    - Messages routed to all connected transports
    - Compatible with existing heartbeat/telemetry streaming

- [ ] **FR-DRAFT-5**: System shall follow ArduPilot STATUSTEXT conventions
  - Rationale: GCS operators expect familiar message format and prefixes
  - Acceptance Criteria:
    - Pre-arm failures prefixed with "PreArm: "
    - Failsafe events prefixed with "Failsafe: "
    - Mode changes prefixed with "Mode: "
    - Parameter changes prefixed with "Param: "
    - Error conditions prefixed with "Error: "

### Non-Functional Requirements (Potential)

- [ ] **NFR-DRAFT-1**: Message queueing shall not allocate heap memory
  - Category: Performance
  - Rationale: Embedded system with limited memory
  - Target: Use heapless::Queue or similar no_std container

- [ ] **NFR-DRAFT-2**: Message generation shall complete within 100µs
  - Category: Performance
  - Rationale: Must not block real-time scheduler or interrupt handlers
  - Target: Simple string copy and queue push operation

- [ ] **NFR-DRAFT-3**: STATUSTEXT API shall be usable in no_std environments
  - Category: Portability
  - Rationale: Must work on embedded targets without standard library
  - Target: No dependencies on std::\*, use core and alloc only where needed

- [ ] **NFR-DRAFT-4**: Long messages shall not exceed 200 characters
  - Category: Reliability
  - Rationale: Limit chunking overhead and queue consumption
  - Target: Truncate messages at 200 chars (4 chunks max)

- [ ] **NFR-DRAFT-5**: Status notification system shall work in host tests
  - Category: Usability
  - Rationale: Developers need to test notification logic without hardware
  - Target: Mock notification sink for capturing messages in tests

## Design Considerations

### Technical Constraints

1. **Memory Constraints:**
   - Embedded target with limited RAM (264 KB on RP2350)
   - Must use heapless data structures (no dynamic allocation)
   - Message queue must have fixed capacity

2. **Real-time Constraints:**
   - Notification API may be called from 400 Hz scheduler interrupt
   - Must not block or perform slow operations
   - Queue push must be O(1) worst-case

3. **MAVLink v2 Constraints:**
   - rust-mavlink `id` and `chunk_seq` fields behind `mavlink2` feature
   - Project must enable mavlink2 feature for full support
   - Must maintain compatibility with MAVLink v1 GCS software

4. **Threading Constraints:**
   - No OS threads on bare-metal embedded
   - Must use embassy async or interrupt-safe primitives
   - Global state requires careful synchronization

### Potential Approaches

**Option A: Global Static Notifier with Mutex**

- Description: Global `static` notifier with `Mutex<RefCell<StatusNotifier>>` for synchronization
- Pros:
  - Simple access from any module via `get_notifier()`
  - No need to thread notifier through function parameters
  - Pattern already used in project for other global state
- Cons:
  - Requires careful lock management to avoid deadlocks
  - Cannot call from interrupt context with mutex
  - Global mutable state is harder to test
- Effort: Low (2-3 days)

**Option B: Notifier as Component in SystemState**

- Description: Add `StatusNotifier` field to `SystemState` struct, pass via parameters
- Pros:
  - Explicit dependency makes data flow clear
  - Easier to test (inject mock notifier)
  - No global state or locking concerns
- Cons:
  - Must thread `&mut SystemState` through many function calls
  - Cannot easily call from static utility functions
  - Larger refactoring of existing code
- Effort: Medium (5-7 days)

**Option C: Message Queue in MAVLink State, API via Helpers**

- Description: Add `heapless::Deque<StatusMessage>` to `MavlinkState`, provide free functions
- Pros:
  - Notifier state lives with other MAVLink state
  - Free functions like `send_error()` enqueue to global queue
  - MAVLink router drains queue during telemetry generation
- Cons:
  - Tight coupling between notifier and MAVLink layer
  - Still requires global access to MavlinkState
  - Harder to use notifier without MAVLink
- Effort: Medium (4-6 days)

**Option D: Embassy Channel for Async Notification**

- Description: Use `embassy_sync::channel::Channel` for async message passing
- Pros:
  - Async-friendly (no blocking)
  - Built-in backpressure handling
  - Clean separation between producer and consumer
- Cons:
  - Requires embassy async runtime
  - More complex async code
  - May require task restructuring
- Effort: High (7-10 days)

**Recommendation: Option A (Global Static Notifier with Mutex)**

- Balances simplicity with accessibility
- Pattern already established in codebase
- Low effort for immediate value
- Can be refactored to Option B or D later if needed

### Architecture Impact

**New Components:**

- `src/communication/mavlink/status_notifier.rs` - StatusNotifier implementation
- Message queue with heapless::Deque
- MAVLink v2 chunking algorithm
- Severity level helper functions

**Modified Components:**

- `src/communication/mavlink/handlers/command.rs` - Remove private `create_statustext()`, use public API
- `src/communication/mavlink/router.rs` - Drain notification queue during telemetry generation
- `src/communication/mavlink/state.rs` - Add notifier access method
- `Cargo.toml` - Enable `mavlink2` feature for rust-mavlink

**Integration Points:**

- Arming system calls `send_error("PreArm: ...")`
- Failsafe system calls `send_warning("Failsafe: ...")`
- Mode system calls `send_notice("Mode: ...")`
- Parameter system calls `send_info("Param: ...")`

## Risk Assessment

| Risk                                                      | Probability | Impact | Mitigation Strategy                                           |
| --------------------------------------------------------- | ----------- | ------ | ------------------------------------------------------------- |
| Message queue overflow during burst of errors             | Medium      | Medium | Implement rate limiting and priority queue                    |
| Chunked messages arrive out-of-order at GCS               | Low         | Medium | Follow MAVLink v2 protocol exactly, test with Mission Planner |
| Mutex deadlock when calling from interrupt context        | Medium      | High   | Provide interrupt-safe try_lock variant                       |
| mavlink2 feature not available in rust-mavlink            | Low         | High   | Verify feature exists in current version (it does)            |
| Message generation overhead impacts real-time performance | Low         | Medium | Benchmark notification API, ensure <100µs                     |
| GCS does not support MAVLink v2 chunking                  | Medium      | Low    | Gracefully truncate to 50 chars for v1-only GCS               |

## Open Questions

- [ ] Should we implement message deduplication to suppress repeated identical messages? → Next step: Research ArduPilot's deduplication strategy
- [ ] What should be the queue capacity (8, 16, 32 messages)? → Method: Analyze typical message burst patterns in ArduPilot logs
- [ ] Should we rate-limit messages per severity level differently? → Next step: Review MAVLink best practices documentation
- [ ] How should we handle queue full condition (drop oldest, drop newest, drop by priority)? → Next step: Test with intentional queue overflow scenarios
- [ ] Should we provide both sync and async APIs for notification? → Next step: Prototype both patterns and measure performance

## Recommendations

### Immediate Actions

1. **Verify MAVLink v2 Support**: Confirm rust-mavlink version supports `mavlink2` feature and STATUSTEXT_DATA includes `id` and `chunk_seq` fields
2. **Extract create_statustext**: Move `CommandHandler::create_statustext()` to standalone function as foundation for public API
3. **Prototype Message Queue**: Create simple proof-of-concept with heapless::Deque for storing pending STATUSTEXT messages

### Next Steps

1. [ ] Create formal requirements: FR-<id> for notification API, chunking support, queue integration
2. [ ] Draft ADR for: Status notification architecture (global vs. component-based)
3. [ ] Draft ADR for: Message queue strategy (size, overflow policy, priority)
4. [ ] Create task for: Implement STATUSTEXT notification system with MAVLink v2 support
5. [ ] Further investigation: ArduPilot message prefix conventions and severity mappings

### Out of Scope

- **Message Localization**: All messages in English only (standard for MAVLink)
- **Message Logging to Flash**: STATUSTEXT is for real-time GCS display, not persistent logging
- **Custom Message IDs**: Use standard STATUSTEXT (253), not custom message types
- **Message Acknowledgment**: STATUSTEXT is fire-and-forget, no ACK expected
- **Message Encryption**: MAVLink signing/encryption handled at protocol layer

## Appendix

### References

- [MAVLink STATUSTEXT Message Specification](https://mavlink.io/en/messages/common.html#STATUSTEXT)
- [RFC-5424 Syslog Severity Levels](https://datatracker.ietf.org/doc/html/rfc5424#section-6.2.1)
- [rust-mavlink GitHub Repository](https://github.com/mavlink/rust-mavlink)
- [ArduPilot GCS_MAVLink Statustext Implementation](https://github.com/ArduPilot/ardupilot/tree/master/libraries/GCS_MAVLink)
- Project existing STATUSTEXT code:
  - `src/communication/mavlink/handlers/command.rs:275` - Private create_statustext()
  - `src/communication/mavlink/handlers/command.rs:135` - Force-arm warning
  - `src/communication/mavlink/handlers/command.rs:168` - Force-disarm warning

### Raw Data

**MAVLink v2 STATUSTEXT Structure:**

```rust
pub struct STATUSTEXT_DATA {
    pub severity: MavSeverity,     // uint8_t: RFC-5424 level (0-7)
    pub text: MavArray<u8, 50>,    // char[50]: Message text
    #[cfg(feature = "mavlink2")]
    pub id: u16,                   // uint16_t: Chunk reassembly ID
    #[cfg(feature = "mavlink2")]
    pub chunk_seq: u8,             // uint8_t: Chunk sequence number
}
```

**RFC-5424 Severity Levels:**

```
0 = EMERGENCY   - System is unusable
1 = ALERT       - Action must be taken immediately
2 = CRITICAL    - Critical conditions
3 = ERROR       - Error conditions
4 = WARNING     - Warning conditions
5 = NOTICE      - Normal but significant condition
6 = INFORMATIONAL - Informational messages
7 = DEBUG       - Debug-level messages
```

**Example ArduPilot STATUSTEXT Messages:**

```
[CRITICAL] "PreArm: Gyros not healthy"
[ERROR] "PreArm: Battery voltage 9.8V below 10.5V"
[WARNING] "Mode change to AUTO failed"
[NOTICE] "Armed"
[INFO] "EKF2 IMU0 is using GPS"
```

**Chunking Example (101-character message):**

```
Message: "PreArm: Battery voltage 9.8V is below minimum arming voltage 10.5V configured in BATT_ARM_VOLT parameter"

Chunk 0 (id=1234, chunk_seq=0):
"PreArm: Battery voltage 9.8V is below minimum arm"

Chunk 1 (id=1234, chunk_seq=1):
"ing voltage 10.5V configured in BATT_ARM_VOLT par"

Chunk 2 (id=1234, chunk_seq=2):
"ameter\0"
```
