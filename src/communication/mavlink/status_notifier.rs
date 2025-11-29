//! MAVLink STATUSTEXT Notification System
//!
//! This module provides a centralized API for sending STATUSTEXT messages to Ground Control
//! Stations (GCS). It enables all system components (arming, failsafe, mode, sensors) to
//! report status, errors, and warnings to operators.
//!
//! # Architecture
//!
//! - **Global Static**: StatusNotifier accessible via `send_*()` functions
//! - **Heapless Queue**: Fixed-capacity queue (16 messages) for no_std compatibility
//! - **Severity API**: Dedicated functions for each severity level (emergency through debug)
//! - **MAVLink v2 Chunking**: Support for messages up to 200 characters (Phase 2)
//!
//! # Usage
//!
//! ```ignore
//! use crate::communication::mavlink::status_notifier::send_error;
//!
//! fn check_battery_voltage(&self) -> ArmingCheckResult {
//!     if self.voltage < self.min_voltage {
//!         send_error("PreArm: Battery voltage low");
//!         return ArmingCheckResult::Failed;
//!     }
//!     ArmingCheckResult::Passed
//! }
//! ```
//!
//! # Performance
//!
//! - API call overhead: <100 µs average, <150 µs worst-case (RP2350 @ 150 MHz)
//! - Zero heap allocations (heapless data structures)
//! - Thread-safe access via Mutex

use core::sync::atomic::{AtomicU16, Ordering};
use critical_section::Mutex;
use heapless::{Deque, String, Vec};
use mavlink::common::{MavSeverity, STATUSTEXT_DATA};

/// Maximum message length (200 characters)
const MAX_MESSAGE_LEN: usize = 200;

/// Queue capacity (16 messages)
const QUEUE_CAPACITY: usize = 16;

/// Chunk size for MAVLink STATUSTEXT messages (50 bytes)
const CHUNK_SIZE: usize = 50;

/// Maximum number of chunks per message (200 / 50 = 4)
const MAX_CHUNKS: usize = 4;

/// Queued STATUSTEXT message with severity and text
#[derive(Debug)]
pub(crate) struct QueuedMessage {
    #[allow(dead_code)] // Used in Phase 3 (router integration)
    pub(crate) severity: MavSeverity,
    #[allow(dead_code)] // Used in Phase 3 (router integration)
    pub(crate) text: String<MAX_MESSAGE_LEN>,
}

/// StatusNotifier manages a queue of pending STATUSTEXT messages
pub struct StatusNotifier {
    queue: Deque<QueuedMessage, QUEUE_CAPACITY>,
    #[allow(dead_code)] // Used in Phase 2 (chunking)
    next_chunk_id: AtomicU16,
    dropped_count: u32,
}

impl StatusNotifier {
    /// Create a new StatusNotifier (const constructor for static initialization)
    const fn new() -> Self {
        Self {
            queue: Deque::new(),
            next_chunk_id: AtomicU16::new(1), // Start at 1 (0 reserved for non-chunked)
            dropped_count: 0,
        }
    }

    /// Enqueue a message with the given severity and text
    ///
    /// If the queue is full, the oldest message is dropped and `dropped_count` is incremented.
    /// If the message exceeds 200 characters, it is truncated.
    fn enqueue(&mut self, severity: MavSeverity, text: &str) {
        // Truncate if message too long
        let text = if text.len() > MAX_MESSAGE_LEN {
            crate::log_warn!(
                "STATUSTEXT truncated to {} chars (was {} chars)",
                MAX_MESSAGE_LEN,
                text.len()
            );
            &text[..MAX_MESSAGE_LEN]
        } else {
            text
        };

        // Create message
        let message = QueuedMessage {
            severity,
            text: String::try_from(text).unwrap_or_default(),
        };

        // If queue full, drop oldest message
        if self.queue.is_full() {
            self.queue.pop_front();
            self.dropped_count += 1;
            crate::log_warn!(
                "STATUSTEXT queue full, dropped {} messages",
                self.dropped_count
            );
        }

        // Enqueue new message
        self.queue
            .push_back(message)
            .expect("Queue should have space after drop");
    }

    /// Drain all messages from the queue (internal use only)
    ///
    /// Returns an iterator over the drained messages.
    /// Called by the telemetry router to send pending messages.
    #[allow(dead_code)] // Used in Phase 3 (router integration)
    pub(crate) fn drain_messages(&mut self) -> impl Iterator<Item = QueuedMessage> + '_ {
        core::iter::from_fn(move || self.queue.pop_front())
    }
}

/// Global StatusNotifier instance
static NOTIFIER: Mutex<core::cell::RefCell<StatusNotifier>> =
    Mutex::new(core::cell::RefCell::new(StatusNotifier::new()));

/// Send an EMERGENCY severity message (severity 0)
///
/// Emergency messages indicate system is unusable and requires immediate attention.
/// Examples: "Flight termination triggered", "Critical hardware failure"
pub fn send_emergency(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_EMERGENCY, text);
}

/// Send an ALERT severity message (severity 1)
///
/// Alert messages indicate action must be taken immediately.
/// Examples: "Battery critical - land immediately", "GPS lost - switch to stabilize"
pub fn send_alert(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_ALERT, text);
}

/// Send a CRITICAL severity message (severity 2)
///
/// Critical messages indicate critical conditions that should be addressed soon.
/// Examples: "Low battery warning", "Sensor degraded performance"
pub fn send_critical(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_CRITICAL, text);
}

/// Send an ERROR severity message (severity 3)
///
/// Error messages indicate error conditions that prevent normal operation.
/// Examples: "PreArm: Battery voltage low", "Compass calibration failed"
pub fn send_error(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_ERROR, text);
}

/// Send a WARNING severity message (severity 4)
///
/// Warning messages indicate warning conditions that may affect operation.
/// Examples: "Forced arming - safety checks bypassed", "Radio signal weak"
pub fn send_warning(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_WARNING, text);
}

/// Send a NOTICE severity message (severity 5)
///
/// Notice messages indicate normal but significant conditions.
/// Examples: "Mode changed to AUTO", "Mission upload complete"
pub fn send_notice(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_NOTICE, text);
}

/// Send an INFORMATIONAL severity message (severity 6)
///
/// Informational messages provide general status information.
/// Examples: "GPS acquired 12 satellites", "Compass calibration started"
pub fn send_info(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_INFO, text);
}

/// Send a DEBUG severity message (severity 7)
///
/// Debug messages provide detailed diagnostic information.
/// Examples: "PID gains: P=0.5 I=0.1 D=0.05", "Sensor reading: 123.45"
pub fn send_debug(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_DEBUG, text);
}

/// Internal helper to send a STATUSTEXT message
///
/// Acquires mutex lock, enqueues message, and releases lock.
/// Called by all public severity-specific functions.
fn send_statustext(severity: MavSeverity, text: &str) {
    critical_section::with(|cs| {
        let notifier = NOTIFIER.borrow(cs);
        let mut n = notifier.borrow_mut();
        n.enqueue(severity, text);
    });
}

/// Drain all pending messages from queue and convert to STATUSTEXT_DATA messages
///
/// Called by telemetry router to retrieve and send pending status notifications.
/// Drains the queue and chunks each message as needed.
///
/// # Returns
///
/// A heapless Vec of STATUSTEXT_DATA messages ready to send to GCS.
/// Maximum 32 messages per call (if queue is full with long messages, remaining
/// messages will be retrieved in next call).
pub fn take_pending_statustext_messages() -> Vec<STATUSTEXT_DATA, 32> {
    let mut result = Vec::new();

    // Drain all messages from queue
    let messages: Vec<QueuedMessage, QUEUE_CAPACITY> = critical_section::with(|cs| {
        let notifier = NOTIFIER.borrow(cs);
        let mut n = notifier.borrow_mut();
        let mut msgs = Vec::new();
        while let Some(msg) = n.queue.pop_front() {
            let _ = msgs.push(msg);
        }
        msgs
    });

    // Chunk each message and add to result
    let _message_count = messages.len();
    for msg in messages {
        let chunks = chunk_message(msg.severity, msg.text.as_str());
        for chunk in chunks {
            if result.push(chunk).is_err() {
                // Result Vec is full, remaining messages will be processed next cycle
                crate::log_warn!(
                    "STATUSTEXT result buffer full, {} messages pending",
                    _message_count
                );
                break;
            }
        }
        if result.is_full() {
            break;
        }
    }

    result
}

/// Chunk a message into STATUSTEXT_DATA messages
///
/// Messages ≤50 chars produce a single message with id=0, chunk_seq=0.
/// Messages >50 chars are split into multiple chunks with unique id and sequential chunk_seq.
///
/// # Arguments
///
/// * `severity` - MAVLink severity level
/// * `text` - Message text (up to 200 characters)
///
/// # Returns
///
/// A heapless Vec of STATUSTEXT_DATA messages (1-4 chunks)
#[allow(dead_code)] // Used in Phase 3 (router integration)
pub(crate) fn chunk_message(severity: MavSeverity, text: &str) -> Vec<STATUSTEXT_DATA, MAX_CHUNKS> {
    let bytes = text.as_bytes();
    let len = bytes.len().min(MAX_MESSAGE_LEN);
    let mut chunks = Vec::new();

    // Single message (≤50 chars)
    if len <= CHUNK_SIZE {
        let mut text_bytes = [0u8; CHUNK_SIZE];
        text_bytes[..len].copy_from_slice(&bytes[..len]);
        chunks
            .push(STATUSTEXT_DATA {
                severity,
                text: text_bytes.into(),
                id: 0,
                chunk_seq: 0,
            })
            .ok();
        return chunks;
    }

    // Multi-chunk message: assign unique chunk ID
    let chunk_id = critical_section::with(|cs| {
        let notifier = NOTIFIER.borrow(cs);
        let n = notifier.borrow();
        let id = n.next_chunk_id.fetch_add(1, Ordering::Relaxed);

        // Handle wraparound: skip 0 (reserved for non-chunked)
        if id == 0 {
            n.next_chunk_id.store(1, Ordering::Relaxed);
            1
        } else {
            id
        }
    });

    // Split into chunks
    let mut offset = 0;
    let mut chunk_seq = 0;
    while offset < len && chunk_seq < MAX_CHUNKS {
        let chunk_len = (len - offset).min(CHUNK_SIZE);
        let mut text_bytes = [0u8; CHUNK_SIZE];
        text_bytes[..chunk_len].copy_from_slice(&bytes[offset..offset + chunk_len]);

        chunks
            .push(STATUSTEXT_DATA {
                severity,
                text: text_bytes.into(),
                id: chunk_id,
                chunk_seq: chunk_seq as u8,
            })
            .ok();

        offset += chunk_len;
        chunk_seq += 1;
    }

    chunks
}

#[cfg(test)]
mod tests {
    use super::*;

    // Use serial_test to prevent concurrent access to global NOTIFIER
    use serial_test::serial;

    /// Helper to drain all messages from the global notifier
    fn drain_global_notifier() -> std::vec::Vec<QueuedMessage> {
        critical_section::with(|cs| {
            let notifier = NOTIFIER.borrow(cs);
            let mut n = notifier.borrow_mut();
            n.drain_messages().collect()
        })
    }

    /// Helper to get dropped count
    fn get_dropped_count() -> u32 {
        critical_section::with(|cs| {
            let notifier = NOTIFIER.borrow(cs);
            notifier.borrow().dropped_count
        })
    }

    /// Helper to reset notifier state between tests
    fn reset_notifier() {
        critical_section::with(|cs| {
            let notifier = NOTIFIER.borrow(cs);
            let mut n = notifier.borrow_mut();
            n.queue.clear();
            n.dropped_count = 0;
        });
    }

    #[test]
    #[serial]
    fn test_enqueue_drain_cycle() {
        reset_notifier();

        send_error("Test error message");
        send_warning("Test warning message");

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 2);
        assert_eq!(messages[0].text.as_str(), "Test error message");
        assert_eq!(messages[1].text.as_str(), "Test warning message");
    }

    #[test]
    #[serial]
    fn test_queue_overflow() {
        reset_notifier();

        // Enqueue 17 messages (capacity is 16)
        for i in 0..17 {
            send_info(&format!("Message {}", i));
        }

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 16, "Queue should hold 16 messages");

        // First message should be dropped (oldest)
        assert_eq!(messages[0].text.as_str(), "Message 1");
        assert_eq!(messages[15].text.as_str(), "Message 16");

        // Check dropped count
        let dropped = get_dropped_count();
        assert_eq!(dropped, 1, "Should have dropped 1 message");
    }

    #[test]
    #[serial]
    fn test_dropped_count_increment() {
        reset_notifier();

        // Fill queue
        for i in 0..16 {
            send_info(&format!("Message {}", i));
        }

        // Add 3 more messages (should drop 3)
        send_info("Message 16");
        send_info("Message 17");
        send_info("Message 18");

        let dropped = get_dropped_count();
        assert_eq!(dropped, 3, "Should have dropped 3 messages");
    }

    #[test]
    #[serial]
    fn test_message_truncation() {
        reset_notifier();

        // Create a message >200 characters
        let long_message = "A".repeat(250);
        send_error(&long_message);

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 1);
        assert_eq!(
            messages[0].text.len(),
            MAX_MESSAGE_LEN,
            "Message should be truncated to 200 chars"
        );
    }

    #[test]
    #[serial]
    fn test_all_severity_functions() {
        reset_notifier();

        send_emergency("Emergency test");
        send_alert("Alert test");
        send_critical("Critical test");
        send_error("Error test");
        send_warning("Warning test");
        send_notice("Notice test");
        send_info("Info test");
        send_debug("Debug test");

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 8);

        // Verify severity levels
        assert!(matches!(
            messages[0].severity,
            MavSeverity::MAV_SEVERITY_EMERGENCY
        ));
        assert!(matches!(
            messages[1].severity,
            MavSeverity::MAV_SEVERITY_ALERT
        ));
        assert!(matches!(
            messages[2].severity,
            MavSeverity::MAV_SEVERITY_CRITICAL
        ));
        assert!(matches!(
            messages[3].severity,
            MavSeverity::MAV_SEVERITY_ERROR
        ));
        assert!(matches!(
            messages[4].severity,
            MavSeverity::MAV_SEVERITY_WARNING
        ));
        assert!(matches!(
            messages[5].severity,
            MavSeverity::MAV_SEVERITY_NOTICE
        ));
        assert!(matches!(
            messages[6].severity,
            MavSeverity::MAV_SEVERITY_INFO
        ));
        assert!(matches!(
            messages[7].severity,
            MavSeverity::MAV_SEVERITY_DEBUG
        ));
    }

    #[test]
    #[serial]
    fn test_empty_string() {
        reset_notifier();

        send_error("");

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].text.as_str(), "");
    }

    #[test]
    #[serial]
    fn test_single_character() {
        reset_notifier();

        send_error("X");

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].text.as_str(), "X");
    }

    #[test]
    #[serial]
    fn test_exactly_200_characters() {
        reset_notifier();

        let message = "A".repeat(200);
        send_error(&message);

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].text.len(), 200);
    }

    #[test]
    #[serial]
    fn test_exactly_201_characters() {
        reset_notifier();

        let message = "A".repeat(201);
        send_error(&message);

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 1);
        assert_eq!(
            messages[0].text.len(),
            200,
            "Should be truncated to 200 chars"
        );
    }

    // Chunking tests
    #[test]
    #[serial]
    fn test_chunk_single_message_50_chars() {
        reset_notifier();

        let msg = "A".repeat(50);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg);

        assert_eq!(chunks.len(), 1, "Should produce 1 chunk");
        assert_eq!(chunks[0].id, 0, "Single message should have id=0");
        assert_eq!(chunks[0].chunk_seq, 0, "Should have chunk_seq=0");
        assert_eq!(
            chunks[0].severity,
            MavSeverity::MAV_SEVERITY_ERROR,
            "Should preserve severity"
        );
    }

    #[test]
    #[serial]
    fn test_chunk_2_chunk_message() {
        reset_notifier();

        // 75 chars = 2 chunks (50 + 25)
        let msg = "A".repeat(75);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_WARNING, &msg);

        assert_eq!(chunks.len(), 2, "Should produce 2 chunks");
        assert_ne!(chunks[0].id, 0, "Multi-chunk should have non-zero id");
        assert_eq!(
            chunks[0].id, chunks[1].id,
            "Both chunks should have same id"
        );
        assert_eq!(chunks[0].chunk_seq, 0, "First chunk should be seq 0");
        assert_eq!(chunks[1].chunk_seq, 1, "Second chunk should be seq 1");
    }

    #[test]
    #[serial]
    fn test_chunk_3_chunk_message() {
        reset_notifier();

        // 125 chars = 3 chunks (50 + 50 + 25)
        let msg = "B".repeat(125);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_INFO, &msg);

        assert_eq!(chunks.len(), 3, "Should produce 3 chunks");
        assert_eq!(chunks[0].id, chunks[1].id, "All chunks should have same id");
        assert_eq!(chunks[0].id, chunks[2].id, "All chunks should have same id");
        assert_eq!(chunks[0].chunk_seq, 0);
        assert_eq!(chunks[1].chunk_seq, 1);
        assert_eq!(chunks[2].chunk_seq, 2);
    }

    #[test]
    #[serial]
    fn test_chunk_4_chunk_message() {
        reset_notifier();

        // 200 chars = 4 chunks (50 + 50 + 50 + 50)
        let msg = "C".repeat(200);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_CRITICAL, &msg);

        assert_eq!(chunks.len(), 4, "Should produce 4 chunks");
        assert_eq!(chunks[0].id, chunks[1].id);
        assert_eq!(chunks[0].id, chunks[2].id);
        assert_eq!(chunks[0].id, chunks[3].id);
        assert_eq!(chunks[0].chunk_seq, 0);
        assert_eq!(chunks[1].chunk_seq, 1);
        assert_eq!(chunks[2].chunk_seq, 2);
        assert_eq!(chunks[3].chunk_seq, 3);
    }

    #[test]
    #[serial]
    fn test_chunk_exactly_50_characters() {
        reset_notifier();

        let msg = "X".repeat(50);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_DEBUG, &msg);

        assert_eq!(chunks.len(), 1);
        assert_eq!(chunks[0].id, 0);
        assert_eq!(chunks[0].chunk_seq, 0);
    }

    #[test]
    #[serial]
    fn test_chunk_exactly_51_characters() {
        reset_notifier();

        // 51 chars triggers chunking
        let msg = "Y".repeat(51);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_NOTICE, &msg);

        assert_eq!(chunks.len(), 2, "51 chars should trigger chunking");
        assert_ne!(chunks[0].id, 0, "Should have non-zero id");
    }

    #[test]
    #[serial]
    fn test_chunk_id_uniqueness() {
        reset_notifier();

        let msg1 = "A".repeat(51);
        let msg2 = "B".repeat(51);

        let chunks1 = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg1);
        let chunks2 = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg2);

        assert_ne!(
            chunks1[0].id, chunks2[0].id,
            "Consecutive multi-chunk messages should have different IDs"
        );
    }

    #[test]
    #[serial]
    fn test_chunk_id_wraparound() {
        reset_notifier();

        // Set counter near wraparound
        critical_section::with(|cs| {
            let notifier = NOTIFIER.borrow(cs);
            let n = notifier.borrow();
            n.next_chunk_id.store(u16::MAX - 1, Ordering::Relaxed);
        });

        // First call: u16::MAX - 1 -> fetch_add(1) returns old value (u16::MAX - 1)
        let msg1 = "A".repeat(51);
        let chunks1 = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg1);
        assert_eq!(chunks1[0].id, u16::MAX - 1, "Should get MAX - 1 value");

        // Second call: u16::MAX -> fetch_add(1) returns u16::MAX, counter wraps to 0
        let msg2 = "B".repeat(51);
        let chunks2 = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg2);
        assert_eq!(chunks2[0].id, u16::MAX, "Should get MAX value");

        // Third call: 0 -> fetch_add(1) returns 0 (wrapped), counter becomes 1 -> skip to 1
        let msg3 = "C".repeat(51);
        let chunks3 = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg3);
        assert_eq!(chunks3[0].id, 1, "Should wrap to 1 (skipping 0)");
    }

    #[test]
    #[serial]
    fn test_chunk_last_chunk_padding() {
        reset_notifier();

        // 75 chars: first chunk 50, second chunk 25 (padded with nulls)
        let msg = "Test message for padding check which is exactly seventy-five chars long!";
        assert_eq!(msg.len(), 72); // Verify length
        let msg = format!("{}XXX", msg); // Make it exactly 75
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_INFO, &msg);

        assert_eq!(chunks.len(), 2);

        // Second chunk should have 25 bytes of content + 25 bytes of null padding
        let text_bytes: &[u8] = chunks[1].text.as_ref();
        assert_eq!(text_bytes.len(), 50, "Text field should be 50 bytes");

        // Last 25 bytes should be null (padding)
        for (offset, &byte) in text_bytes[25..50].iter().enumerate() {
            let i = 25 + offset;
            assert_eq!(byte, 0, "Byte {} should be null padding, got {}", i, byte);
        }
    }

    // Additional edge case tests for Phase 4

    #[test]
    #[serial]
    fn test_utf8_multibyte_characters() {
        reset_notifier();

        // Test with Japanese characters (3 bytes each in UTF-8)
        send_info("日本語テスト");

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].text.as_str(), "日本語テスト");
    }

    #[test]
    #[serial]
    fn test_utf8_emoji() {
        reset_notifier();

        // Test with emoji (4 bytes each in UTF-8)
        send_warning("Status: ✅ OK 🚀");

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].text.as_str(), "Status: ✅ OK 🚀");
    }

    #[test]
    #[serial]
    fn test_null_bytes_in_message() {
        reset_notifier();

        // Test with embedded null bytes
        send_error("Before\0After");

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 1);
        // The message should contain the null byte
        assert!(messages[0].text.as_str().contains('\0'));
    }

    #[test]
    #[serial]
    fn test_end_to_end_flow() {
        reset_notifier();

        // Enqueue a long message that requires chunking
        let long_msg =
            "This is a test message that is longer than 50 characters and will be chunked";
        send_info(long_msg);

        // Drain and chunk via take_pending_statustext_messages
        let statustext_messages = take_pending_statustext_messages();

        // Should produce 2 chunks (message is ~77 chars)
        assert_eq!(
            statustext_messages.len(),
            2,
            "Should produce 2 STATUSTEXT chunks"
        );

        // Verify chunks have same ID (multi-chunk message)
        assert!(
            statustext_messages[0].id != 0,
            "Multi-chunk should have non-zero id"
        );
        assert_eq!(
            statustext_messages[0].id, statustext_messages[1].id,
            "All chunks should have same id"
        );
        assert_eq!(statustext_messages[0].chunk_seq, 0);
        assert_eq!(statustext_messages[1].chunk_seq, 1);
    }

    #[test]
    #[serial]
    fn test_multiple_messages_single_drain() {
        reset_notifier();

        // Enqueue multiple messages of different severities
        send_error("Error message");
        send_warning("Warning message");
        send_info("Info message");

        // Drain all at once
        let statustext_messages = take_pending_statustext_messages();

        // Should have 3 messages (each ≤50 chars, so 1 chunk each)
        assert_eq!(statustext_messages.len(), 3);

        // Verify severities are preserved in order
        assert!(matches!(
            statustext_messages[0].severity,
            MavSeverity::MAV_SEVERITY_ERROR
        ));
        assert!(matches!(
            statustext_messages[1].severity,
            MavSeverity::MAV_SEVERITY_WARNING
        ));
        assert!(matches!(
            statustext_messages[2].severity,
            MavSeverity::MAV_SEVERITY_INFO
        ));
    }

    #[test]
    #[serial]
    fn test_queue_overflow_recovery() {
        reset_notifier();

        // Fill queue to capacity (16 messages)
        for i in 0..16 {
            send_info(&format!("Message {}", i));
        }

        // Drain all
        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 16);

        // Queue should be empty, dropped_count should be 0
        assert_eq!(get_dropped_count(), 0);

        // Add more messages after overflow recovery
        send_error("New error after drain");
        send_warning("New warning after drain");

        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 2);
    }

    #[test]
    #[serial]
    fn test_message_exactly_50_chars_no_chunking() {
        reset_notifier();

        // Exactly 50 characters should NOT trigger chunking
        let msg = "A".repeat(50);
        send_info(&msg);

        let statustext_messages = take_pending_statustext_messages();
        assert_eq!(
            statustext_messages.len(),
            1,
            "50 chars should produce 1 chunk"
        );
        assert_eq!(statustext_messages[0].id, 0, "Single message has id=0");
        assert_eq!(statustext_messages[0].chunk_seq, 0);
    }

    #[test]
    #[serial]
    fn test_message_exactly_51_chars_triggers_chunking() {
        reset_notifier();

        // Exactly 51 characters SHOULD trigger chunking
        let msg = "A".repeat(51);
        send_info(&msg);

        let statustext_messages = take_pending_statustext_messages();
        assert_eq!(
            statustext_messages.len(),
            2,
            "51 chars should produce 2 chunks"
        );
        assert!(
            statustext_messages[0].id != 0,
            "Multi-chunk has non-zero id"
        );
        assert_eq!(statustext_messages[0].chunk_seq, 0);
        assert_eq!(statustext_messages[1].chunk_seq, 1);
    }
}
