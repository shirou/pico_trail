//! MAVLink STATUSTEXT Notification System
//!
//! Provides a centralized API for sending STATUSTEXT messages to Ground Control
//! Stations (GCS). Enables all system components (arming, failsafe, mode, sensors) to
//! report status, errors, and warnings to operators.
//!
//! # Architecture
//!
//! - **Global Static**: StatusNotifier accessible via `send_*()` functions
//! - **Heapless Queue**: Fixed-capacity queue (16 messages) for no_std compatibility
//! - **Severity API**: Dedicated functions for each severity level (emergency through debug)
//! - **MAVLink v2 Chunking**: Support for messages up to 200 characters

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
pub struct QueuedMessage {
    pub severity: MavSeverity,
    pub text: String<MAX_MESSAGE_LEN>,
}

/// StatusNotifier manages a queue of pending STATUSTEXT messages
pub struct StatusNotifier {
    queue: Deque<QueuedMessage, QUEUE_CAPACITY>,
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
        }

        // Enqueue new message
        self.queue
            .push_back(message)
            .expect("Queue should have space after drop");
    }

    /// Drain all messages from the queue (used by tests)
    #[cfg(test)]
    pub(crate) fn drain_messages(&mut self) -> impl Iterator<Item = QueuedMessage> + '_ {
        core::iter::from_fn(move || self.queue.pop_front())
    }
}

/// Global StatusNotifier instance
static NOTIFIER: Mutex<core::cell::RefCell<StatusNotifier>> =
    Mutex::new(core::cell::RefCell::new(StatusNotifier::new()));

/// Send an EMERGENCY severity message (severity 0)
pub fn send_emergency(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_EMERGENCY, text);
}

/// Send an ALERT severity message (severity 1)
pub fn send_alert(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_ALERT, text);
}

/// Send a CRITICAL severity message (severity 2)
pub fn send_critical(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_CRITICAL, text);
}

/// Send an ERROR severity message (severity 3)
pub fn send_error(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_ERROR, text);
}

/// Send a WARNING severity message (severity 4)
pub fn send_warning(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_WARNING, text);
}

/// Send a NOTICE severity message (severity 5)
pub fn send_notice(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_NOTICE, text);
}

/// Send an INFORMATIONAL severity message (severity 6)
pub fn send_info(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_INFO, text);
}

/// Send a DEBUG severity message (severity 7)
pub fn send_debug(text: &str) {
    send_statustext(MavSeverity::MAV_SEVERITY_DEBUG, text);
}

/// Internal helper to send a STATUSTEXT message
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
    for msg in messages {
        let chunks = chunk_message(msg.severity, msg.text.as_str());
        for chunk in chunks {
            if result.push(chunk).is_err() {
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
pub fn chunk_message(severity: MavSeverity, text: &str) -> Vec<STATUSTEXT_DATA, MAX_CHUNKS> {
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
    extern crate std;
    use std::{format, vec};

    use super::*;

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

    #[test]
    #[serial]
    fn test_chunk_single_message_50_chars() {
        reset_notifier();
        let msg = "A".repeat(50);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg);
        assert_eq!(chunks.len(), 1);
        assert_eq!(chunks[0].id, 0);
        assert_eq!(chunks[0].chunk_seq, 0);
    }

    #[test]
    #[serial]
    fn test_chunk_2_chunk_message() {
        reset_notifier();
        let msg = "A".repeat(75);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_WARNING, &msg);
        assert_eq!(chunks.len(), 2);
        assert_ne!(chunks[0].id, 0);
        assert_eq!(chunks[0].id, chunks[1].id);
        assert_eq!(chunks[0].chunk_seq, 0);
        assert_eq!(chunks[1].chunk_seq, 1);
    }

    #[test]
    #[serial]
    fn test_chunk_4_chunk_message() {
        reset_notifier();
        let msg = "C".repeat(200);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_CRITICAL, &msg);
        assert_eq!(chunks.len(), 4);
        assert_eq!(chunks[0].chunk_seq, 0);
        assert_eq!(chunks[1].chunk_seq, 1);
        assert_eq!(chunks[2].chunk_seq, 2);
        assert_eq!(chunks[3].chunk_seq, 3);
    }

    #[test]
    #[serial]
    fn test_chunk_id_uniqueness() {
        reset_notifier();
        let msg1 = "A".repeat(51);
        let msg2 = "B".repeat(51);
        let chunks1 = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg1);
        let chunks2 = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg2);
        assert_ne!(chunks1[0].id, chunks2[0].id);
    }

    #[test]
    #[serial]
    fn test_end_to_end_flow() {
        reset_notifier();
        let long_msg =
            "This is a test message that is longer than 50 characters and will be chunked";
        send_info(long_msg);
        let statustext_messages = take_pending_statustext_messages();
        assert_eq!(statustext_messages.len(), 2);
        assert!(statustext_messages[0].id != 0);
        assert_eq!(statustext_messages[0].id, statustext_messages[1].id);
    }

    #[test]
    #[serial]
    fn test_multiple_messages_single_drain() {
        reset_notifier();
        send_error("Error message");
        send_warning("Warning message");
        send_info("Info message");
        let statustext_messages = take_pending_statustext_messages();
        assert_eq!(statustext_messages.len(), 3);
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
    fn test_chunk_3_chunk_message() {
        reset_notifier();
        let msg = "B".repeat(125);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_INFO, &msg);
        assert_eq!(chunks.len(), 3);
        assert_eq!(chunks[0].id, chunks[1].id);
        assert_eq!(chunks[0].id, chunks[2].id);
        assert_eq!(chunks[0].chunk_seq, 0);
        assert_eq!(chunks[1].chunk_seq, 1);
        assert_eq!(chunks[2].chunk_seq, 2);
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
        let msg = "Y".repeat(51);
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_NOTICE, &msg);
        assert_eq!(chunks.len(), 2);
        assert_ne!(chunks[0].id, 0);
    }

    #[test]
    #[serial]
    fn test_chunk_id_wraparound() {
        reset_notifier();
        critical_section::with(|cs| {
            let notifier = NOTIFIER.borrow(cs);
            let n = notifier.borrow();
            n.next_chunk_id.store(u16::MAX - 1, Ordering::Relaxed);
        });
        let msg1 = "A".repeat(51);
        let chunks1 = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg1);
        assert_eq!(chunks1[0].id, u16::MAX - 1);
        let msg2 = "B".repeat(51);
        let chunks2 = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg2);
        assert_eq!(chunks2[0].id, u16::MAX);
        let msg3 = "C".repeat(51);
        let chunks3 = chunk_message(MavSeverity::MAV_SEVERITY_ERROR, &msg3);
        assert_eq!(chunks3[0].id, 1);
    }

    #[test]
    #[serial]
    fn test_chunk_last_chunk_padding() {
        reset_notifier();
        let msg = format!(
            "{}XXX",
            "Test message for padding check which is exactly seventy-five chars long!"
        );
        let chunks = chunk_message(MavSeverity::MAV_SEVERITY_INFO, &msg);
        assert_eq!(chunks.len(), 2);
        let text_bytes: &[u8] = chunks[1].text.as_ref();
        assert_eq!(text_bytes.len(), 50);
        for &byte in &text_bytes[25..50] {
            assert_eq!(byte, 0);
        }
    }

    #[test]
    #[serial]
    fn test_utf8_multibyte_characters() {
        reset_notifier();
        send_info("日本語テスト");
        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].text.as_str(), "日本語テスト");
    }

    #[test]
    #[serial]
    fn test_null_bytes_in_message() {
        reset_notifier();
        send_error("Before\0After");
        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 1);
        assert!(messages[0].text.as_str().contains('\0'));
    }

    #[test]
    #[serial]
    fn test_queue_overflow_recovery() {
        reset_notifier();
        for i in 0..16 {
            send_info(&format!("Message {}", i));
        }
        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 16);
        assert_eq!(get_dropped_count(), 0);
        send_error("New error after drain");
        send_warning("New warning after drain");
        let messages = drain_global_notifier();
        assert_eq!(messages.len(), 2);
    }

    #[test]
    #[serial]
    fn test_message_exactly_50_chars_no_chunking() {
        reset_notifier();
        let msg = "A".repeat(50);
        send_info(&msg);
        let statustext_messages = take_pending_statustext_messages();
        assert_eq!(statustext_messages.len(), 1);
        assert_eq!(statustext_messages[0].id, 0);
        assert_eq!(statustext_messages[0].chunk_seq, 0);
    }

    #[test]
    #[serial]
    fn test_message_exactly_51_chars_triggers_chunking() {
        reset_notifier();
        let msg = "A".repeat(51);
        send_info(&msg);
        let statustext_messages = take_pending_statustext_messages();
        assert_eq!(statustext_messages.len(), 2);
        assert!(statustext_messages[0].id != 0);
        assert_eq!(statustext_messages[0].chunk_seq, 0);
        assert_eq!(statustext_messages[1].chunk_seq, 1);
    }
}
