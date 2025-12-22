//! Log Ring Buffer
//!
//! Provides a fixed-capacity ring buffer for storing log messages.
//! Uses heapless HistoryBuffer for no-allocation storage.
//!
//! ## Features
//!
//! - Fixed capacity of 32 messages (~8.3 KB RAM)
//! - Automatic oldest message eviction when full
//! - Overflow tracking for diagnostics
//! - Oldest-first iteration order

use heapless::{HistoryBuf, String, Vec};

/// Buffer capacity in number of messages
pub const LOG_BUFFER_SIZE: usize = 32;

/// Maximum message size in bytes
pub const LOG_MSG_SIZE: usize = 256;

/// Log level with ordering: Trace < Debug < Info < Warn < Error
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum LogLevel {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
}

/// Log message containing level and text
#[derive(Clone)]
pub struct LogMessage {
    pub level: LogLevel,
    pub message: String<LOG_MSG_SIZE>,
}

impl LogMessage {
    /// Create a new log message
    pub fn new(level: LogLevel, message: String<LOG_MSG_SIZE>) -> Self {
        Self { level, message }
    }
}

/// Ring buffer sink for log messages
///
/// Stores up to LOG_BUFFER_SIZE messages. When full, the oldest message
/// is automatically evicted to make room for new messages.
pub struct RingBufferSink {
    buffer: HistoryBuf<LogMessage, LOG_BUFFER_SIZE>,
    overflow_count: u32,
}

impl RingBufferSink {
    /// Create a new empty ring buffer sink
    pub const fn new() -> Self {
        Self {
            buffer: HistoryBuf::new(),
            overflow_count: 0,
        }
    }

    /// Push a message to the buffer
    ///
    /// If the buffer is full, the oldest message is evicted and
    /// overflow_count is incremented.
    pub fn push(&mut self, msg: LogMessage) {
        if self.buffer.len() == LOG_BUFFER_SIZE {
            self.overflow_count = self.overflow_count.saturating_add(1);
        }
        self.buffer.write(msg);
    }

    /// Return the current number of messages in the buffer
    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    /// Return true if the buffer is empty
    pub fn is_empty(&self) -> bool {
        self.buffer.len() == 0
    }

    /// Return the number of messages lost due to buffer overflow
    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    /// Iterate over messages in oldest-first order
    pub fn iter(&self) -> impl Iterator<Item = &LogMessage> {
        self.buffer.oldest_ordered()
    }

    /// Drain all messages from the buffer, returning them in oldest-first order
    ///
    /// The buffer is cleared after draining.
    pub fn drain(&mut self) -> Vec<LogMessage, LOG_BUFFER_SIZE> {
        let mut result = Vec::new();
        for msg in self.buffer.oldest_ordered() {
            let _ = result.push(msg.clone());
        }
        self.buffer.clear();
        result
    }

    /// Clear all messages from the buffer
    ///
    /// Does not reset overflow_count.
    pub fn clear(&mut self) {
        self.buffer.clear();
    }
}

impl Default for RingBufferSink {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_msg(level: LogLevel, text: &str) -> LogMessage {
        let mut message = String::new();
        let _ = message.push_str(text);
        LogMessage::new(level, message)
    }

    #[test]
    fn test_push_single_message() {
        let mut sink = RingBufferSink::new();
        assert!(sink.is_empty());

        sink.push(make_msg(LogLevel::Info, "test message"));

        assert_eq!(sink.len(), 1);
        assert!(!sink.is_empty());
        assert_eq!(sink.overflow_count(), 0);
    }

    #[test]
    fn test_push_fill_buffer() {
        let mut sink = RingBufferSink::new();

        for i in 0..LOG_BUFFER_SIZE {
            sink.push(make_msg(LogLevel::Info, &format!("msg {}", i)));
        }

        assert_eq!(sink.len(), LOG_BUFFER_SIZE);
        assert_eq!(sink.overflow_count(), 0);
    }

    #[test]
    fn test_push_overflow() {
        let mut sink = RingBufferSink::new();

        // Fill the buffer
        for i in 0..LOG_BUFFER_SIZE {
            sink.push(make_msg(LogLevel::Info, &format!("msg {}", i)));
        }
        assert_eq!(sink.overflow_count(), 0);

        // Push one more (should overflow)
        sink.push(make_msg(LogLevel::Info, "overflow msg"));

        assert_eq!(sink.len(), LOG_BUFFER_SIZE);
        assert_eq!(sink.overflow_count(), 1);

        // Verify oldest was evicted (first message should be "msg 1")
        let first = sink.iter().next().unwrap();
        assert_eq!(first.message.as_str(), "msg 1");
    }

    #[test]
    fn test_overflow_count_increment() {
        let mut sink = RingBufferSink::new();

        // Fill the buffer
        for i in 0..LOG_BUFFER_SIZE {
            sink.push(make_msg(LogLevel::Info, &format!("msg {}", i)));
        }

        // Push 5 more messages
        for i in 0..5 {
            sink.push(make_msg(LogLevel::Info, &format!("overflow {}", i)));
        }

        assert_eq!(sink.overflow_count(), 5);
    }

    #[test]
    fn test_iteration_order() {
        let mut sink = RingBufferSink::new();

        sink.push(make_msg(LogLevel::Info, "first"));
        sink.push(make_msg(LogLevel::Warn, "second"));
        sink.push(make_msg(LogLevel::Error, "third"));

        let messages: std::vec::Vec<_> = sink.iter().collect();
        assert_eq!(messages.len(), 3);
        assert_eq!(messages[0].message.as_str(), "first");
        assert_eq!(messages[1].message.as_str(), "second");
        assert_eq!(messages[2].message.as_str(), "third");
    }

    #[test]
    fn test_drain_returns_all_messages() {
        let mut sink = RingBufferSink::new();

        sink.push(make_msg(LogLevel::Info, "first"));
        sink.push(make_msg(LogLevel::Warn, "second"));
        sink.push(make_msg(LogLevel::Error, "third"));

        let drained = sink.drain();

        assert_eq!(drained.len(), 3);
        assert_eq!(drained[0].message.as_str(), "first");
        assert_eq!(drained[1].message.as_str(), "second");
        assert_eq!(drained[2].message.as_str(), "third");
    }

    #[test]
    fn test_drain_empties_buffer() {
        let mut sink = RingBufferSink::new();

        sink.push(make_msg(LogLevel::Info, "message"));
        assert!(!sink.is_empty());

        let _ = sink.drain();

        assert!(sink.is_empty());
        assert_eq!(sink.len(), 0);
    }

    #[test]
    fn test_clear_empties_buffer() {
        let mut sink = RingBufferSink::new();

        sink.push(make_msg(LogLevel::Info, "first"));
        sink.push(make_msg(LogLevel::Info, "second"));
        assert_eq!(sink.len(), 2);

        sink.clear();

        assert!(sink.is_empty());
        assert_eq!(sink.len(), 0);
    }

    #[test]
    fn test_log_level_ordering() {
        assert!(LogLevel::Trace < LogLevel::Debug);
        assert!(LogLevel::Debug < LogLevel::Info);
        assert!(LogLevel::Info < LogLevel::Warn);
        assert!(LogLevel::Warn < LogLevel::Error);

        // Test direct comparison
        assert!(LogLevel::Warn >= LogLevel::Warn);
        assert!(LogLevel::Error >= LogLevel::Warn);
        assert!(LogLevel::Info < LogLevel::Warn);
    }

    // =========================================================================
    // Phase 4: Edge Case Tests
    // =========================================================================

    #[test]
    fn test_empty_message() {
        let mut sink = RingBufferSink::new();

        sink.push(make_msg(LogLevel::Info, ""));

        assert_eq!(sink.len(), 1);
        let first = sink.iter().next().unwrap();
        assert_eq!(first.message.as_str(), "");
    }

    #[test]
    fn test_single_character_message() {
        let mut sink = RingBufferSink::new();

        sink.push(make_msg(LogLevel::Info, "X"));

        assert_eq!(sink.len(), 1);
        let first = sink.iter().next().unwrap();
        assert_eq!(first.message.as_str(), "X");
    }

    #[test]
    fn test_max_length_message_256_chars() {
        let mut sink = RingBufferSink::new();

        // Create exactly 256 character message
        let max_msg: std::string::String = "A".repeat(LOG_MSG_SIZE);
        let mut message = String::<LOG_MSG_SIZE>::new();
        let _ = message.push_str(&max_msg);

        sink.push(LogMessage::new(LogLevel::Info, message));

        assert_eq!(sink.len(), 1);
        let first = sink.iter().next().unwrap();
        assert_eq!(first.message.len(), LOG_MSG_SIZE);
    }

    #[test]
    fn test_message_over_256_chars_fails_push() {
        // heapless::String returns Err when push_str would exceed capacity
        // The string remains unchanged (empty in this case)
        let long_msg: std::string::String = "B".repeat(300);
        let mut message = String::<LOG_MSG_SIZE>::new();
        let result = message.push_str(&long_msg);

        // push_str should fail when entire string doesn't fit
        assert!(result.is_err());
        // Message remains empty because push failed
        assert_eq!(message.len(), 0);
    }

    #[test]
    fn test_utf8_multibyte_characters() {
        let mut sink = RingBufferSink::new();

        // Japanese characters (3 bytes each in UTF-8)
        let utf8_text = "日本語テスト🚀";
        sink.push(make_msg(LogLevel::Info, utf8_text));

        assert_eq!(sink.len(), 1);
        let first = sink.iter().next().unwrap();
        assert_eq!(first.message.as_str(), utf8_text);
    }

    #[test]
    fn test_utf8_over_capacity_fails_push() {
        // heapless::String returns Err when UTF-8 string doesn't fit
        // Same behavior as ASCII - entire push_str fails
        let long_utf8 = "日".repeat(100); // Each 日 is 3 bytes, 100 = 300 bytes
        let mut message = String::<LOG_MSG_SIZE>::new();
        let result = message.push_str(&long_utf8);

        // push_str fails because entire string (300 bytes) doesn't fit
        assert!(result.is_err());
        // Message remains empty
        assert_eq!(message.len(), 0);
    }

    #[test]
    fn test_manual_truncation_to_fit() {
        // Demonstration: How to properly truncate a long message
        let long_msg = "A".repeat(300);
        let mut message = String::<LOG_MSG_SIZE>::new();

        // Truncate to capacity before pushing
        let truncated = if long_msg.len() > LOG_MSG_SIZE {
            &long_msg[..LOG_MSG_SIZE]
        } else {
            &long_msg
        };
        let result = message.push_str(truncated);

        assert!(result.is_ok());
        assert_eq!(message.len(), LOG_MSG_SIZE);
    }

    #[test]
    fn test_utf8_safe_truncation() {
        // UTF-8 safe truncation by character iteration
        let long_utf8 = "日本語".repeat(50); // 50 * 3 chars * 3 bytes = 450 bytes
        let mut message = String::<LOG_MSG_SIZE>::new();

        // Push characters one by one until capacity
        for ch in long_utf8.chars() {
            if message.push(ch).is_err() {
                break;
            }
        }

        // Message is filled with as many complete UTF-8 characters as fit
        assert!(message.len() <= LOG_MSG_SIZE);
        assert!(!message.is_empty());
        // Should contain complete characters (divisible by 3 for these chars)
        assert!(message.len().is_multiple_of(3));
    }

    #[test]
    fn test_rapid_burst_logging_50_messages() {
        let mut sink = RingBufferSink::new();

        // Push 50 messages rapidly
        for i in 0..50 {
            sink.push(make_msg(LogLevel::Info, &format!("burst msg {}", i)));
        }

        // Buffer should only hold 32 (latest 32 messages)
        assert_eq!(sink.len(), LOG_BUFFER_SIZE);
        // Should have 18 overflows (50 - 32)
        assert_eq!(sink.overflow_count(), 18);

        // First message in buffer should be "burst msg 18" (oldest after overflow)
        let first = sink.iter().next().unwrap();
        assert_eq!(first.message.as_str(), "burst msg 18");

        // Last message should be "burst msg 49"
        let last = sink.iter().last().unwrap();
        assert_eq!(last.message.as_str(), "burst msg 49");
    }
}
