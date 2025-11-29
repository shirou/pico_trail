# T-eiuvv STATUSTEXT Notification System Implementation

## Metadata

- Type: Implementation Plan
- Status: Completed (Software)

**Completion Date:** 2025-11-29

**Note:** Hardware validation (Performance Profiling, GCS Compatibility) deferred pending hardware availability.

## Links

- Associated Design Document:
  - [T-eiuvv-statustext-implementation-design](design.md)

## Overview

Implement a public API for sending MAVLink STATUSTEXT messages to ground control stations, with heapless queue, MAVLink v2 chunking, and router integration. The implementation follows a 4-phase approach: Core Notifier, MAVLink v2 Chunking, Router Integration, and Testing & Validation.

## Success Metrics

- [ ] API call overhead <100 µs (average), <150 µs (worst-case) on RP2350 @ 150 MHz
- [ ] Zero heap allocations verified via linker map inspection
- [ ] Static memory footprint ≤4 KB
- [ ] Long messages (>50 chars) reassemble correctly in Mission Planner
- [ ] All existing tests pass; no regressions in MAVLink communication
- [ ] All 9 requirements' acceptance criteria satisfied

## Scope

- Goal: Enable all system components to send STATUSTEXT messages to GCS operators
- Non-Goals:
  - Message deduplication (future enhancement)
  - Rate limiting per severity level (future enhancement)
  - Priority queue with bypass for EMERGENCY/ALERT (future enhancement)
  - Message logging to flash storage (separate subsystem)
- Assumptions:
  - GCS software supports MAVLink v2 chunking protocol
  - Telemetry handler runs at 1 Hz minimum
  - Queue capacity of 16 messages sufficient for typical burst patterns
- Constraints:
  - No heap allocation (no_std compatibility)
  - Performance <100 µs for enqueue operation
  - Static memory ≤4 KB for notifier and queue

## ADR & Legacy Alignment

- [ ] Confirm ADR-jxvhf-global-statusnotifier referenced in design.md
- [ ] Note legacy pattern: Private `CommandHandler::create_statustext()` at `src/communication/mavlink/handlers/command.rs:275`
- [ ] Subtask in Phase 3: Migrate force-arm/disarm warnings to new API and remove private function

## Plan Summary

- Phase 1 – Core Notifier Implementation (queue, API, global static)
- Phase 2 – MAVLink v2 Chunking Algorithm (chunk_message, ID counter)
- Phase 3 – Router Integration and Migration (drain_messages, remove legacy code)
- Phase 4 – Testing and Performance Validation (unit tests, profiling, GCS verification)

### Phase Status Tracking

Mark checkboxes (`[x]`) immediately after completing each task. Annotate skipped items with strikethrough and brief note.

---

## Phase 1: Core Notifier Implementation

### Goal

- Create StatusNotifier with heapless queue and global static access
- Implement severity-specific API functions
- Verify queue operations and mutex synchronization

### Inputs

- Documentation:
  - [ADR-jxvhf-global-statusnotifier](../../adr/ADR-jxvhf-global-statusnotifier.md)
  - [NFR-ssp9q-statustext-performance-memory](../../requirements/NFR-ssp9q-statustext-performance-memory.md)
- Source Code to Modify:
  - New file: `src/communication/mavlink/status_notifier.rs`
  - Update: `src/communication/mavlink/mod.rs` (add module declaration)
- Dependencies:
  - External crates: `heapless`, `embassy_sync`
  - Internal: `mavlink::common::MavSeverity`

### Tasks

- [x] **Module Setup**
  - [x] Create `src/communication/mavlink/status_notifier.rs`
  - [x] Add `pub mod status_notifier;` to `src/communication/mavlink/mod.rs`
  - [x] Add `heapless` dependency to `Cargo.toml` if not present
  - [x] Add `embassy_sync` dependency to `Cargo.toml` if not present

- [x] **StatusNotifier Structure**
  - [x] Define `QueuedMessage` struct (severity: MavSeverity, text: String<200>)
  - [x] Define `StatusNotifier` struct (queue: Deque\<QueuedMessage, 16>, next_chunk_id: AtomicU16, dropped_count: u32)
  - [x] Implement `StatusNotifier::new()` const constructor
  - [x] Declare global static `NOTIFIER: Mutex<CriticalSectionRawMutex, RefCell<StatusNotifier>>`

- [x] **Queue Operations**
  - [x] Implement `StatusNotifier::enqueue(severity, text)` private method
  - [x] Handle queue full: drop oldest, increment dropped_count
  - [x] Handle message >200 chars: truncate, log warning
  - [x] Implement `StatusNotifier::drain_messages()` internal method (returns iterator)

- [x] **Public API Functions**
  - [x] Implement `send_emergency(text: &str)`
  - [x] Implement `send_alert(text: &str)`
  - [x] Implement `send_critical(text: &str)`
  - [x] Implement `send_error(text: &str)`
  - [x] Implement `send_warning(text: &str)`
  - [x] Implement `send_notice(text: &str)`
  - [x] Implement `send_info(text: &str)`
  - [x] Implement `send_debug(text: &str)`
  - [x] Implement internal `send_statustext(severity, text)` helper

- [x] **Unit Tests**
  - [x] Test enqueue/drain cycle
  - [x] Test queue overflow (17th message drops oldest)
  - [x] Test dropped_count increment
  - [x] Test message truncation (>200 chars)
  - [x] Test all severity-specific functions
  - [x] Test concurrent access (if host test supports threading)

### Deliverables

- `src/communication/mavlink/status_notifier.rs` with StatusNotifier, queue, and API
- Unit tests verifying queue operations and API functions
- No heap allocations (verified manually via code inspection)

### Verification

```bash
# Format and lint
cargo fmt
cargo clippy --all-targets -- -D warnings

# Unit tests
cargo test --lib --quiet status_notifier

# Embedded build (verify no_std compatibility)
./scripts/build-rp2350.sh
```

### Acceptance Criteria (Phase Gate)

- All unit tests pass
- No heap-allocating types (Vec, String, Box) in status_notifier.rs
- Embedded build succeeds without errors
- Queue operations work correctly (enqueue, drain, overflow)

### Rollback/Fallback

- Delete `src/communication/mavlink/status_notifier.rs`
- Remove module declaration from `mod.rs`
- Revert to private `CommandHandler::create_statustext()` pattern

---

## Phase 2: MAVLink v2 Chunking Algorithm

### Phase 2 Goal

- Implement chunking algorithm for messages >50 characters
- Enable `mavlink2` feature for STATUSTEXT id/chunk_seq fields
- Verify chunk generation and ID assignment

### Phase 2 Inputs

- Dependencies:
  - Phase 1: StatusNotifier with queue (provides next_chunk_id counter)
  - [FR-7qki7-statustext-chunking](../../requirements/FR-7qki7-statustext-chunking.md)
  - [NFR-1wo70-statustext-length-limits](../../requirements/NFR-1wo70-statustext-length-limits.md)
- Source Code to Modify:
  - `src/communication/mavlink/status_notifier.rs` (add chunking function)
  - `Cargo.toml` (enable mavlink2 feature)

### Phase 2 Tasks

- [x] **Enable MAVLink v2**
  - [x] ~~Add `features = ["mavlink2"]` to `mavlink` dependency in `Cargo.toml`~~ (Not needed - v0.16.1 includes id/chunk_seq by default)
  - [x] Verify `STATUSTEXT_DATA` includes `id` and `chunk_seq` fields
  - [x] Test compilation with and without `mavlink2` feature

- [x] **Chunking Algorithm**
  - [x] Implement `chunk_message(severity, text) -> Vec<STATUSTEXT_DATA, 4>`
  - [x] Handle single message (≤50 chars): id=0, chunk_seq=0
  - [x] Handle multi-chunk (>50 chars): assign unique id, sequential chunk_seq
  - [x] Fetch and increment next_chunk_id atomically
  - [x] Handle chunk_id wraparound (skip 0, wrap to 1)
  - [x] Pad last chunk with null bytes
  - [x] Use `heapless::Vec` (no allocation)

- [x] **Unit Tests**
  - [x] Test single message (50 chars): 1 chunk, id=0
  - [x] Test 2-chunk message (51-100 chars): same id, chunk_seq 0,1
  - [x] Test 3-chunk message (101-150 chars): same id, chunk_seq 0,1,2
  - [x] Test 4-chunk message (151-200 chars): same id, chunk_seq 0,1,2,3
  - [x] Test 200-char message (edge case)
  - [x] Test chunk_id uniqueness (consecutive calls have different ids)
  - [x] Test chunk_id wraparound (mock counter at u16::MAX-1)

### Phase 2 Deliverables

- `chunk_message()` function in status_notifier.rs
- MAVLink v2 feature enabled in Cargo.toml
- Unit tests verifying chunking correctness

### Phase 2 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet status_notifier::tests::chunking
./scripts/build-rp2350.sh
```

### Phase 2 Acceptance Criteria

- Messages ≤50 chars produce single chunk (id=0)
- Messages >50 chars produce multiple chunks with same non-zero id
- chunk_seq increments sequentially (0, 1, 2, ...)
- Last chunk padded with null bytes
- No heap allocation (heapless::Vec used)

### Phase 2 Rollback/Fallback

- Remove `chunk_message()` function
- Revert `mavlink2` feature in Cargo.toml
- Fall back to single-message only (truncate at 50 chars)

---

## Phase 3: Router Integration and Migration

### Phase 3 Goal

- Integrate StatusNotifier with MAVLink router telemetry handler
- Drain queue and send STATUSTEXT messages
- Migrate existing force-arm/disarm warnings to new API
- Remove legacy private `create_statustext()` function

### Phase 3 Inputs

- Dependencies:
  - Phase 1: StatusNotifier with queue and API
  - Phase 2: Chunking algorithm
  - [FR-krvqy-statustext-router-integration](../../requirements/FR-krvqy-statustext-router-integration.md)
- Source Code to Modify:
  - `src/communication/mavlink/router.rs` (add queue draining to telemetry handler)
  - `src/communication/mavlink/handlers/command.rs` (migrate warnings, remove private function)

### Phase 3 Tasks

- [x] **Router Integration**
  - [x] Add `take_statustext_messages()` method to `MavlinkRouter`
  - [x] Method calls `status_notifier::take_pending_statustext_messages()`
  - [x] Converts `STATUSTEXT_DATA` to `MavMessage::STATUSTEXT`
  - [x] Returns `heapless::Vec<MavMessage, 32>` for sending to GCS
  - [x] Add unit tests for `take_statustext_messages()`

- [x] **Migrate Force-Arm Warning**
  - [x] Force-arm warning uses `status_notifier::send_warning("Armed (FORCED)")`
  - [x] Normal arm uses `status_notifier::send_info("Armed")`
  - [x] Arm rejection uses `status_notifier::send_error("Arm rejected")`

- [x] **Migrate Force-Disarm Warning**
  - [x] Force-disarm warning uses `status_notifier::send_warning("Disarmed (FORCED)")`
  - [x] Normal disarm uses `status_notifier::send_info("Disarmed")`
  - [x] Disarm rejection uses `status_notifier::send_error("Disarm rejected")`

- [x] **Remove Legacy Code**
  - [x] Legacy `create_statustext()` function already removed
  - [x] All STATUSTEXT creation now uses `status_notifier` module
  - [x] No compilation errors

### Phase 3 Deliverables

- Router integration with queue draining and STATUSTEXT transmission
- Migrated force-arm/disarm warnings using new API
- Removed legacy `create_statustext()` function

### Phase 3 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings
cargo test --lib --quiet
./scripts/build-rp2350.sh

# Manual test with GCS:
# 1. Flash firmware to RP2350
# 2. Connect Mission Planner via UDP
# 3. Send force-arm command (check warning appears)
# 4. Send force-disarm command (check warning appears)
```

### Phase 3 Acceptance Criteria

- [x] `take_statustext_messages()` method added to `MavlinkRouter`
- [x] Force-arm/disarm status messages use `status_notifier` API
- [x] No compilation errors after legacy code removal
- [x] All existing MAVLink tests pass (387 tests)
- [ ] STATUSTEXT messages appear in Mission Planner (hardware test pending)

### Phase 3 Rollback/Fallback

- Revert router.rs changes (remove queue draining)
- Restore private `create_statustext()` function
- Revert force-arm/disarm migrations

---

## Phase 4: Testing and Performance Validation

### Phase 4 Goal

- Create comprehensive unit and integration tests
- Validate performance targets (<100 µs)
- Verify GCS compatibility (Mission Planner, QGroundControl)
- Measure memory footprint and verify zero heap allocation

### Phase 4 Tasks

- [x] **Unit Tests - API Coverage**
  - [x] Test `send_emergency()` enqueues with EMERGENCY severity
  - [x] Test `send_alert()` enqueues with ALERT severity
  - [x] Test `send_critical()` enqueues with CRITICAL severity
  - [x] Test `send_error()` enqueues with ERROR severity
  - [x] Test `send_warning()` enqueues with WARNING severity
  - [x] Test `send_notice()` enqueues with NOTICE severity
  - [x] Test `send_info()` enqueues with INFORMATIONAL severity
  - [x] Test `send_debug()` enqueues with DEBUG severity
  - All 8 severities tested in `test_all_severity_functions`

- [x] **Unit Tests - Edge Cases**
  - [x] Test empty string "" (`test_empty_string`)
  - [x] Test single character "X" (`test_single_character`)
  - [x] Test exactly 50 characters (`test_message_exactly_50_chars_no_chunking`)
  - [x] Test exactly 51 characters (`test_message_exactly_51_chars_triggers_chunking`)
  - [x] Test exactly 200 characters (`test_exactly_200_characters`)
  - [x] Test exactly 201 characters (`test_exactly_201_characters`)
  - [x] Test UTF-8 multibyte characters (`test_utf8_multibyte_characters`, `test_utf8_emoji`)
  - [x] Test null bytes in message (`test_null_bytes_in_message`)

- [x] **Integration Tests**
  - [x] Test end-to-end flow: enqueue → drain → chunk (`test_end_to_end_flow`)
  - [x] Test multiple messages in single drain cycle (`test_multiple_messages_single_drain`)
  - [x] Test queue overflow recovery (`test_queue_overflow_recovery`)
  - [x] Router integration tests (`test_take_statustext_messages_*`)

- [ ] **Performance Profiling** (deferred - requires hardware)
  - [ ] Add profiling test with `embassy_time::Instant` (embedded only)
  - [ ] Measure `send_error()` average time over 100 iterations
  - [ ] Measure `chunk_message()` time for 200-char message
  - [ ] Verify <100 µs average, <150 µs worst-case
  - [ ] Log results with `log_info!()`

- [x] **Memory Verification**
  - [x] Build with `--release` for RP2350 (UF2 generated: 974,848 bytes)
  - [x] No allocator symbols found in binary
  - [x] Static memory analysis: StatusNotifier ≈ 3,300 bytes (< 4 KB)
    - Queue: 16 × 204 bytes (QueuedMessage) = 3,264 bytes
    - Atomic counter + dropped_count: 6 bytes
    - Mutex/RefCell overhead: \~30 bytes

- [ ] **GCS Compatibility** (deferred - requires hardware)
  - [ ] Manual test: Send 30-char message, verify displays in Mission Planner
  - [ ] Manual test: Send 75-char message, verify chunks reassemble correctly
  - [ ] Manual test: Send 150-char message, verify 3 chunks reassemble
  - [ ] Manual test: Send 200-char message, verify 4 chunks reassemble
  - [ ] Optional: Test with QGroundControl if available

- [x] **Concurrency & Cleanup**
  - [x] All tests use `#[serial]` to prevent concurrent access issues
  - [x] Mutex (CriticalSection) prevents data corruption
  - [x] Test queue state after overflow and recovery (`test_queue_overflow_recovery`)
  - [x] Verify dropped_count accuracy (`test_dropped_count_increment`)

### Phase 4 Deliverables

- Comprehensive test suite (unit + integration)
- Performance profiling results documenting <100 µs achievement
- Memory verification confirming zero heap allocation and ≤4 KB footprint
- GCS compatibility verification (Mission Planner)

### Phase 4 Verification

```bash
cargo fmt
cargo clippy --all-targets -- -D warnings

# Full test suite
cargo test --lib --quiet
cargo test --quiet  # Include integration tests

# Embedded build and profiling
./scripts/build-rp2350.sh pico_trail_rover
probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/pico_trail_rover

# Memory verification
arm-none-eabi-nm target/thumbv8m.main-none-eabihf/release/examples/pico_trail_rover | grep -i alloc
cargo size --release --target thumbv8m.main-none-eabihf --bin pico_trail_rover
```

### Phase 4 Acceptance Criteria

- [x] All unit tests pass (26 status_notifier tests + 2 router tests = 395 total)
- [ ] Performance <100 µs average verified via embedded profiling (deferred)
- [x] Zero heap allocation verified (no alloc symbols in binary)
- [ ] Long messages reassemble correctly in Mission Planner (hardware test pending)
- [x] Static memory footprint ≤4 KB (measured: \~3,300 bytes)

---

## Definition of Done

- [x] `cargo fmt` passes
- [x] `cargo clippy --all-targets -- -D warnings` passes
- [x] `cargo test --lib --quiet` passes (395 tests)
- [x] `./scripts/build-rp2350.sh` succeeds
- [x] All 4 phases completed with acceptance criteria met (software tests complete)
- [ ] Performance verified <100 µs on RP2350 (deferred - requires hardware)
- [x] Memory verified ≤4 KB static (\~3,300 bytes), zero heap allocation
- [ ] GCS compatibility verified (Mission Planner) (deferred - requires hardware)
- [x] Legacy `create_statustext()` removed
- [x] Documentation updated (N/A for internal API)
- [x] No unsafe Rust code introduced
- [x] All software-testable requirements satisfied

## Open Questions

- [ ] Should EMERGENCY/ALERT bypass queue and send immediately? → Next step: Monitor queue overflow in Phase 4 testing; if critical messages dropped, implement bypass in follow-up task
- [ ] Should we implement message deduplication? → Decision: Defer to future enhancement unless GCS feedback indicates spam issue
- [ ] What is optimal queue size? → Method: Monitor dropped_count metric in Phase 4; if frequent overflows, increase to 32 in follow-up

---

## Known Issues

### Duplicate STATUSTEXT Messages (Future Work)

**Issue**: ARM/DISARM status messages sometimes appear twice in GCS (Mission Planner).

**Observed Behavior**:

- Single ARM or DISARM command results in two identical STATUSTEXT messages
- Messages appear with slightly different timestamps (e.g., 22:52:15 and 22:52:16)
- All message types affected: "Armed", "Arm rejected", "Disarmed", "Disarm rejected"

**Possible Root Causes**:

1. **GCS Command Retry**: Mission Planner may be sending duplicate ARM/DISARM commands (retry mechanism on timeout)
2. **Message Routing**: Potential duplicate processing in dispatcher/router pipeline
3. **Queue Consumer Issue**: Multiple consumers of status_notifier queue
4. **Telemetry Loop**: update_telemetry() being called multiple times per command

**Investigation TODO** (`command.rs:147-153`):

```rust
// TODO: Investigate duplicate STATUSTEXT messages
// Current observation: ARM/DISARM status messages sometimes appear twice in GCS
// Possible causes:
// - GCS sending duplicate commands (retry mechanism)
// - Message routing issue in dispatcher/router
// - Multiple consumers of status_notifier queue
// Future work: Add message deduplication or investigate root cause
```

**Mitigation Options** (for future task):

- Add message deduplication based on content hash + timestamp window
- Add sequence number to detect duplicate commands from GCS
- Add debug logging to trace message flow through dispatcher/router/telemetry
- Implement exponential backoff for GCS command retries

**Impact**: Low - Messages are informational and duplication doesn't affect vehicle operation

**Priority**: P3 - Enhancement for better UX, not blocking current functionality
