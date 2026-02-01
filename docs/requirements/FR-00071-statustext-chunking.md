# FR-00071 MAVLink v2 Chunking for Long STATUSTEXT Messages

## Metadata

- Type: Functional Requirement
- Status: Draft

## Links

- Related Analyses:
  - [AN-00020-statustext-notifications](../analysis/AN-00020-statustext-notifications.md)
- Prerequisite Requirements:
  - [FR-00073-statustext-public-api](../requirements/FR-00073-statustext-public-api.md)
- Dependent Requirements:
  - [NFR-00064-statustext-length-limits](../requirements/NFR-00064-statustext-length-limits.md)
- Related Tasks:
  - [T-00014-statustext-implementation](../tasks/T-00014-statustext-implementation/README.md)

## Requirement Statement

The system shall support MAVLink v2 chunking protocol for STATUSTEXT messages exceeding 50 characters, automatically splitting messages into multiple chunks with unique ID and sequential chunk_seq fields for reassembly by ground control stations.

## Rationale

Error messages with parameter names, values, and context often exceed the 50-character limit of a single STATUSTEXT message (e.g., "PreArm: Battery voltage 9.8V is below minimum threshold 10.5V configured in BATT_ARM_VOLT parameter" is 101 characters). The current implementation truncates these messages, losing critical information. MAVLink v2 provides chunking protocol specifically for this use case.

## User Story

As an **operator using Mission Planner or QGroundControl**, I want **to receive complete error messages longer than 50 characters**, so that **I have full diagnostic information for troubleshooting without missing critical details**.

## Acceptance Criteria

- [ ] Messages up to 50 characters sent as single STATUSTEXT with `id=0`, `chunk_seq=0`
- [ ] Messages 51-100 characters split into 2 chunks with same non-zero `id`
- [ ] Messages 101-150 characters split into 3 chunks with same non-zero `id`
- [ ] Messages 151-200 characters split into 4 chunks with same non-zero `id`
- [ ] Chunks sent with sequential `chunk_seq` starting at 0 (0, 1, 2, ...)
- [ ] Last chunk padded with null bytes after message content
- [ ] Unique `id` assigned per multi-chunk message (non-zero, incrementing)
- [ ] Chunked messages reassemble correctly in Mission Planner
- [ ] Chunked messages reassemble correctly in QGroundControl

## Technical Details

### Functional Requirement Details

**MAVLink v2 STATUSTEXT Structure:**

```rust
pub struct STATUSTEXT_DATA {
    pub severity: MavSeverity,
    pub text: MavArray<u8, 50>,
    #[cfg(feature = "mavlink2")]
    pub id: u16,              // 0 = single message, non-zero = chunked
    #[cfg(feature = "mavlink2")]
    pub chunk_seq: u8,        // 0-indexed chunk sequence
}
```

**Chunking Algorithm:**

1. If message ≤ 50 chars: Send single message with `id=0`, `chunk_seq=0`
2. If message > 50 chars:
   - Assign unique non-zero `id` (incrementing counter)
   - Split into 50-byte chunks
   - Send chunks with `id=<assigned>`, `chunk_seq=0, 1, 2, ...`
   - Pad last chunk with null bytes

**Example:**

Message: "PreArm: Battery voltage 9.8V is below minimum arming voltage 10.5V configured in BATT_ARM_VOLT parameter" (109 chars)

```
Chunk 0 (id=1, chunk_seq=0): "PreArm: Battery voltage 9.8V is below minimum arm"
Chunk 1 (id=1, chunk_seq=1): "ing voltage 10.5V configured in BATT_ARM_VOLT par"
Chunk 2 (id=1, chunk_seq=2): "ameter\0\0\0..." (padded to 50 bytes)
```

**ID Management:**

- Global counter starts at 1 (0 reserved for non-chunked)
- Wraps at u16::MAX back to 1
- Thread-safe increment

## Platform Considerations

### Embedded (RP2350)

- Must verify `mavlink2` feature enabled in Cargo.toml
- Chunking algorithm must not allocate heap (use stack buffer)
- ID counter must be atomic for thread safety

### Host Tests

- Mock GCS can verify chunk reassembly
- Test suite validates chunk boundaries and padding

### Cross-Platform

- Chunking behavior identical on all platforms
- rust-mavlink library handles platform-specific serialization

## Risks & Mitigation

| Risk                                                   | Impact | Likelihood | Mitigation                                             | Validation                                   |
| ------------------------------------------------------ | ------ | ---------- | ------------------------------------------------------ | -------------------------------------------- |
| Chunks arrive out-of-order at GCS                      | Medium | Low        | Follow MAVLink v2 protocol exactly, sequential sending | Test with actual GCS software                |
| ID counter overflow (u16::MAX)                         | Low    | Low        | Wrap to 1 (not 0), test wrap behavior                  | Long-running test, verify wrap handling      |
| GCS does not support MAVLink v2 chunking               | Medium | Medium     | Gracefully truncate to 50 chars for MAVLink v1 GCS     | Test with both v1 and v2 GCS implementations |
| Chunk padding incorrect (GCS misdetects end)           | High   | Low        | Follow spec exactly (null terminator + null padding)   | Verify with Mission Planner chunked messages |
| mavlink2 feature not available in rust-mavlink version | High   | Low        | Verify feature exists (it does in current version)     | Check Cargo.toml and rust-mavlink docs       |

## Implementation Notes

**Preferred Patterns:**

- Extract chunking logic into separate function `chunk_statustext(severity, text) -> Vec<STATUSTEXT_DATA>`
- Use atomic u16 counter for ID generation
- Reuse chunking logic for all severity levels

**Known Pitfalls:**

- Do not split on UTF-8 character boundaries (split bytes, not chars)
- Do not forget null terminator in last chunk
- Do not use `id=0` for chunked messages (reserved for single messages)

**Related Code:**

- rust-mavlink STATUSTEXT_DATA: `mavlink::common::STATUSTEXT_DATA`
- MAVLink v2 feature gate: `Cargo.toml` `features = ["mavlink2"]`

**Implementation Guidance:**

- Maximum message length should be 200 characters (see NFR-00064)
- Truncate messages exceeding 200 chars to prevent excessive chunks

## External References

- [MAVLink STATUSTEXT Message Specification](https://mavlink.io/en/messages/common.html#STATUSTEXT)
- [MAVLink v2 Protocol Specification](https://mavlink.io/en/guide/mavlink_2.html)
- [rust-mavlink GitHub Repository](https://github.com/mavlink/rust-mavlink)
