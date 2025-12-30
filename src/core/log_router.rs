//! Log Router
//!
//! Routes log messages to multiple destinations:
//! - RingBufferSink: Stores all messages for later retrieval
//! - STATUSTEXT: Routes WARNING and ERROR to GCS (Phase 3)
//! - USB Serial: Existing output (unchanged)
//!
//! ## Usage
//!
//! The log router is automatically called by the log macros.
//! Use the retrieval API to access buffered logs:
//!
//! ```ignore
//! use pico_trail::core::log_router::{get_buffered_logs, buffer_len, overflow_count};
//!
//! // Get all buffered logs (drains buffer)
//! let logs = get_buffered_logs();
//!
//! // Check buffer status
//! let count = buffer_len();
//! let overflows = overflow_count();
//! ```

use crate::core::log_buffer::{LogMessage, RingBufferSink};

/// Log router that dispatches messages to multiple sinks
pub struct LogRouter {
    buffer_sink: RingBufferSink,
}

impl LogRouter {
    /// Create a new log router
    pub const fn new() -> Self {
        Self {
            buffer_sink: RingBufferSink::new(),
        }
    }

    /// Route a log message to the ring buffer only
    ///
    /// STATUSTEXT routing is handled by `route_log()` to avoid recursive borrow issues.
    pub fn route(&mut self, msg: LogMessage) {
        // Buffer the message (STATUSTEXT routing is done outside the lock)
        self.buffer_sink.push(msg);
    }

    /// Get a reference to the buffer sink
    pub fn buffer_sink(&self) -> &RingBufferSink {
        &self.buffer_sink
    }

    /// Get a mutable reference to the buffer sink
    pub fn buffer_sink_mut(&mut self) -> &mut RingBufferSink {
        &mut self.buffer_sink
    }
}

impl Default for LogRouter {
    fn default() -> Self {
        Self::new()
    }
}

// =============================================================================
// Embassy Implementation (embedded targets)
// =============================================================================

#[cfg(feature = "embassy")]
mod embassy_impl {
    use super::{LogMessage, LogRouter, RingBufferSink};
    use crate::communication::mavlink::status_notifier;
    use crate::core::log_buffer::{LogLevel, LOG_BUFFER_SIZE};
    use crate::core::traits::{EmbassyState, SharedState};
    use heapless::Vec;

    /// Global log router (protected by EmbassyState)
    pub static LOG_ROUTER: EmbassyState<LogRouter> = EmbassyState::new(LogRouter::new());

    /// Route a log message through the global router
    ///
    /// - Buffers the message in the ring buffer (inside lock)
    /// - Routes WARNING/ERROR to STATUSTEXT (outside lock to prevent recursion)
    pub fn route_log(msg: LogMessage) {
        // Capture level and message before entering the lock
        let level = msg.level;
        let message_clone = msg.message.clone();

        // Buffer the message inside the lock
        LOG_ROUTER.with_mut(|router| {
            router.route(msg);
        });

        // Route to STATUSTEXT outside the lock to prevent recursive borrow
        match level {
            LogLevel::Warn => status_notifier::send_warning(message_clone.as_str()),
            LogLevel::Error => status_notifier::send_error(message_clone.as_str()),
            // INFO, DEBUG, TRACE are not routed to STATUSTEXT
            _ => {}
        }
    }

    /// Get all buffered logs (drains the buffer)
    pub fn get_buffered_logs() -> Vec<LogMessage, LOG_BUFFER_SIZE> {
        LOG_ROUTER.with_mut(|router| router.buffer_sink_mut().drain())
    }

    /// Peek at buffered logs without clearing
    ///
    /// Calls the provided closure with a reference to the buffer sink.
    pub fn peek_buffered_logs<F, R>(f: F) -> R
    where
        F: FnOnce(&RingBufferSink) -> R,
    {
        LOG_ROUTER.with(|router| f(router.buffer_sink()))
    }

    /// Get the current number of buffered messages
    pub fn buffer_len() -> usize {
        LOG_ROUTER.with(|router| router.buffer_sink().len())
    }

    /// Get the number of messages lost due to buffer overflow
    pub fn overflow_count() -> u32 {
        LOG_ROUTER.with(|router| router.buffer_sink().overflow_count())
    }

    /// Clear all buffered messages
    pub fn clear_buffer() {
        LOG_ROUTER.with_mut(|router| {
            router.buffer_sink_mut().clear();
        });
    }
}

#[cfg(feature = "embassy")]
pub use embassy_impl::{
    buffer_len, clear_buffer, get_buffered_logs, overflow_count, peek_buffered_logs, route_log,
};

// =============================================================================
// Test Implementation (host tests without embassy)
// =============================================================================

#[cfg(all(test, not(feature = "embassy")))]
mod test_impl {
    use super::{LogMessage, LogRouter, RingBufferSink};
    use crate::communication::mavlink::status_notifier;
    use crate::core::log_buffer::{LogLevel, LOG_BUFFER_SIZE};
    use heapless::Vec;
    use std::cell::RefCell;

    thread_local! {
        pub static TEST_ROUTER: RefCell<LogRouter> = const { RefCell::new(LogRouter::new()) };
    }

    // Thread-local flag to enable/disable STATUSTEXT routing during tests.
    // By default, STATUSTEXT routing is disabled to avoid parallel test issues
    // with the global NOTIFIER. Tests that specifically need STATUSTEXT routing
    // should call `enable_statustext_routing()` first.
    thread_local! {
        static STATUSTEXT_ROUTING_ENABLED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    }

    pub fn route_log(msg: LogMessage) {
        // Capture level and message before entering the borrow
        let level = msg.level;
        let message_clone = msg.message.clone();

        // Buffer the message inside the borrow
        TEST_ROUTER.with(|router| {
            router.borrow_mut().route(msg);
        });

        // Route to STATUSTEXT outside the borrow to prevent recursive borrow
        // Only enabled for specific tests that need it
        STATUSTEXT_ROUTING_ENABLED.with(|enabled| {
            if enabled.get() {
                match level {
                    LogLevel::Warn => status_notifier::send_warning(message_clone.as_str()),
                    LogLevel::Error => status_notifier::send_error(message_clone.as_str()),
                    _ => {}
                }
            }
        });
    }

    /// Enable STATUSTEXT routing for the current test thread
    pub fn enable_statustext_routing() {
        STATUSTEXT_ROUTING_ENABLED.with(|enabled| enabled.set(true));
    }

    /// Disable STATUSTEXT routing for the current test thread
    pub fn disable_statustext_routing() {
        STATUSTEXT_ROUTING_ENABLED.with(|enabled| enabled.set(false));
    }

    pub fn get_buffered_logs() -> Vec<LogMessage, LOG_BUFFER_SIZE> {
        TEST_ROUTER.with(|router| router.borrow_mut().buffer_sink_mut().drain())
    }

    pub fn peek_buffered_logs<F, R>(f: F) -> R
    where
        F: FnOnce(&RingBufferSink) -> R,
    {
        TEST_ROUTER.with(|router| f(router.borrow().buffer_sink()))
    }

    pub fn buffer_len() -> usize {
        TEST_ROUTER.with(|router| router.borrow().buffer_sink().len())
    }

    pub fn overflow_count() -> u32 {
        TEST_ROUTER.with(|router| router.borrow().buffer_sink().overflow_count())
    }

    pub fn clear_buffer() {
        TEST_ROUTER.with(|router| {
            router.borrow_mut().buffer_sink_mut().clear();
        });
    }
}

#[cfg(all(test, not(feature = "embassy")))]
pub use test_impl::{
    buffer_len, clear_buffer, disable_statustext_routing, enable_statustext_routing,
    get_buffered_logs, overflow_count, peek_buffered_logs, route_log,
};

// =============================================================================
// Unit Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::status_notifier::take_pending_statustext_messages;
    use crate::core::log_buffer::LogLevel;
    use heapless::String;

    fn make_msg(level: LogLevel, text: &str) -> LogMessage {
        let mut message = String::new();
        let _ = message.push_str(text);
        LogMessage::new(level, message)
    }

    fn reset_router() {
        clear_buffer();
        // Also drain any pending STATUSTEXT messages
        let _ = take_pending_statustext_messages();
    }

    #[test]
    fn test_route_stores_in_buffer() {
        reset_router();

        route_log(make_msg(LogLevel::Info, "test message"));

        assert_eq!(buffer_len(), 1);
    }

    #[test]
    fn test_route_all_levels() {
        reset_router();

        route_log(make_msg(LogLevel::Trace, "trace"));
        route_log(make_msg(LogLevel::Debug, "debug"));
        route_log(make_msg(LogLevel::Info, "info"));
        route_log(make_msg(LogLevel::Warn, "warn"));
        route_log(make_msg(LogLevel::Error, "error"));

        assert_eq!(buffer_len(), 5);
    }

    #[test]
    fn test_get_buffered_logs_returns_messages() {
        reset_router();

        route_log(make_msg(LogLevel::Info, "first"));
        route_log(make_msg(LogLevel::Warn, "second"));

        let logs = get_buffered_logs();

        assert_eq!(logs.len(), 2);
        assert_eq!(logs[0].message.as_str(), "first");
        assert_eq!(logs[1].message.as_str(), "second");
    }

    #[test]
    fn test_get_buffered_logs_drains() {
        reset_router();

        route_log(make_msg(LogLevel::Info, "message"));
        assert_eq!(buffer_len(), 1);

        let _ = get_buffered_logs();

        assert_eq!(buffer_len(), 0);
    }

    #[test]
    fn test_peek_buffered_logs_preserves() {
        reset_router();

        route_log(make_msg(LogLevel::Info, "message"));

        let count = peek_buffered_logs(|sink| sink.len());
        assert_eq!(count, 1);

        // Buffer should still have the message
        assert_eq!(buffer_len(), 1);
    }

    #[test]
    fn test_buffer_len_accuracy() {
        reset_router();

        assert_eq!(buffer_len(), 0);

        route_log(make_msg(LogLevel::Info, "one"));
        assert_eq!(buffer_len(), 1);

        route_log(make_msg(LogLevel::Info, "two"));
        assert_eq!(buffer_len(), 2);
    }

    #[test]
    fn test_overflow_count_accuracy() {
        use crate::core::log_buffer::LOG_BUFFER_SIZE;

        reset_router();

        // Fill the buffer
        for i in 0..LOG_BUFFER_SIZE {
            route_log(make_msg(LogLevel::Info, &format!("msg {}", i)));
        }
        assert_eq!(overflow_count(), 0);

        // Push 3 more messages
        route_log(make_msg(LogLevel::Info, "overflow 1"));
        route_log(make_msg(LogLevel::Info, "overflow 2"));
        route_log(make_msg(LogLevel::Info, "overflow 3"));

        assert_eq!(overflow_count(), 3);
    }

    #[test]
    fn test_clear_buffer_empties() {
        reset_router();

        route_log(make_msg(LogLevel::Info, "message"));
        assert_eq!(buffer_len(), 1);

        clear_buffer();

        assert_eq!(buffer_len(), 0);
    }

    // =========================================================================
    // STATUSTEXT Routing Tests (Phase 3)
    // =========================================================================

    /// Helper to reset both router and enable STATUSTEXT routing for tests
    fn reset_for_statustext_test() {
        reset_router();
        enable_statustext_routing();
    }

    #[test]
    fn test_warn_routes_to_statustext() {
        reset_for_statustext_test();

        route_log(make_msg(LogLevel::Warn, "warning message"));

        // Check that message is in buffer
        assert_eq!(buffer_len(), 1);

        // Check that message was routed to STATUSTEXT
        let statustext_msgs = take_pending_statustext_messages();
        assert!(
            !statustext_msgs.is_empty(),
            "WARN should route to STATUSTEXT"
        );

        disable_statustext_routing();
    }

    #[test]
    fn test_error_routes_to_statustext() {
        reset_for_statustext_test();

        route_log(make_msg(LogLevel::Error, "error message"));

        // Check that message is in buffer
        assert_eq!(buffer_len(), 1);

        // Check that message was routed to STATUSTEXT
        let statustext_msgs = take_pending_statustext_messages();
        assert!(
            !statustext_msgs.is_empty(),
            "ERROR should route to STATUSTEXT"
        );

        disable_statustext_routing();
    }

    #[test]
    fn test_info_not_routed_to_statustext() {
        reset_for_statustext_test();

        route_log(make_msg(LogLevel::Info, "info message"));

        // Check that message is in buffer
        assert_eq!(buffer_len(), 1);

        // Check that message was NOT routed to STATUSTEXT
        let statustext_msgs = take_pending_statustext_messages();
        assert!(
            statustext_msgs.is_empty(),
            "INFO should NOT route to STATUSTEXT"
        );

        disable_statustext_routing();
    }

    #[test]
    fn test_debug_not_routed_to_statustext() {
        reset_for_statustext_test();

        route_log(make_msg(LogLevel::Debug, "debug message"));

        // Check that message is in buffer
        assert_eq!(buffer_len(), 1);

        // Check that message was NOT routed to STATUSTEXT
        let statustext_msgs = take_pending_statustext_messages();
        assert!(
            statustext_msgs.is_empty(),
            "DEBUG should NOT route to STATUSTEXT"
        );

        disable_statustext_routing();
    }

    #[test]
    fn test_trace_not_routed_to_statustext() {
        reset_for_statustext_test();

        route_log(make_msg(LogLevel::Trace, "trace message"));

        // Check that message is in buffer
        assert_eq!(buffer_len(), 1);

        // Check that message was NOT routed to STATUSTEXT
        let statustext_msgs = take_pending_statustext_messages();
        assert!(
            statustext_msgs.is_empty(),
            "TRACE should NOT route to STATUSTEXT"
        );

        disable_statustext_routing();
    }

    #[test]
    fn test_statustext_routing_boundary() {
        reset_for_statustext_test();

        // INFO is the highest level NOT routed to STATUSTEXT
        route_log(make_msg(LogLevel::Info, "info boundary"));
        let statustext_after_info = take_pending_statustext_messages();
        assert!(
            statustext_after_info.is_empty(),
            "INFO should NOT route to STATUSTEXT"
        );

        // WARN is the lowest level routed to STATUSTEXT
        route_log(make_msg(LogLevel::Warn, "warn boundary"));
        let statustext_after_warn = take_pending_statustext_messages();
        assert!(
            !statustext_after_warn.is_empty(),
            "WARN should route to STATUSTEXT"
        );

        disable_statustext_routing();
    }

    // =========================================================================
    // Phase 4: Integration Tests
    // =========================================================================

    #[test]
    fn test_end_to_end_warn_to_buffer_and_statustext() {
        reset_for_statustext_test();

        // Route a warning through the system
        route_log(make_msg(LogLevel::Warn, "end-to-end warning"));

        // Verify in buffer
        let logs = get_buffered_logs();
        assert_eq!(logs.len(), 1);
        assert_eq!(logs[0].message.as_str(), "end-to-end warning");
        assert_eq!(logs[0].level, LogLevel::Warn);

        // Verify in STATUSTEXT queue
        let statustext_msgs = take_pending_statustext_messages();
        assert!(!statustext_msgs.is_empty());

        disable_statustext_routing();
    }

    #[test]
    fn test_end_to_end_error_to_buffer_and_statustext() {
        reset_for_statustext_test();

        // Route an error through the system
        route_log(make_msg(LogLevel::Error, "end-to-end error"));

        // Verify in buffer
        let logs = get_buffered_logs();
        assert_eq!(logs.len(), 1);
        assert_eq!(logs[0].message.as_str(), "end-to-end error");
        assert_eq!(logs[0].level, LogLevel::Error);

        // Verify in STATUSTEXT queue
        let statustext_msgs = take_pending_statustext_messages();
        assert!(!statustext_msgs.is_empty());

        disable_statustext_routing();
    }

    #[test]
    fn test_multiple_levels_in_sequence() {
        reset_router();

        // Log messages at all levels in sequence
        route_log(make_msg(LogLevel::Trace, "trace msg"));
        route_log(make_msg(LogLevel::Debug, "debug msg"));
        route_log(make_msg(LogLevel::Info, "info msg"));
        route_log(make_msg(LogLevel::Warn, "warn msg"));
        route_log(make_msg(LogLevel::Error, "error msg"));

        // All should be in buffer
        let logs = get_buffered_logs();
        assert_eq!(logs.len(), 5);

        // Verify order and levels
        assert_eq!(logs[0].level, LogLevel::Trace);
        assert_eq!(logs[1].level, LogLevel::Debug);
        assert_eq!(logs[2].level, LogLevel::Info);
        assert_eq!(logs[3].level, LogLevel::Warn);
        assert_eq!(logs[4].level, LogLevel::Error);
    }

    #[test]
    fn test_buffer_overflow_during_burst() {
        use crate::core::log_buffer::LOG_BUFFER_SIZE;

        reset_router();

        // Burst 50 messages
        for i in 0..50 {
            route_log(make_msg(LogLevel::Info, &format!("burst {}", i)));
        }

        // Buffer holds 32, overflow is 18
        assert_eq!(buffer_len(), LOG_BUFFER_SIZE);
        assert_eq!(overflow_count(), 18);
    }

    #[test]
    fn test_retrieval_after_overflow() {
        use crate::core::log_buffer::LOG_BUFFER_SIZE;

        reset_router();

        // Fill buffer and overflow
        for i in 0..40 {
            route_log(make_msg(LogLevel::Info, &format!("msg {}", i)));
        }

        // Retrieve and verify oldest messages were evicted
        let logs = get_buffered_logs();
        assert_eq!(logs.len(), LOG_BUFFER_SIZE);

        // First message should be "msg 8" (oldest after 8 overflows)
        assert_eq!(logs[0].message.as_str(), "msg 8");

        // Last message should be "msg 39"
        assert_eq!(logs[31].message.as_str(), "msg 39");

        // Buffer should be empty after get_buffered_logs
        assert_eq!(buffer_len(), 0);

        // Overflow count persists (it's not reset by drain)
        assert_eq!(overflow_count(), 8);
    }

    #[test]
    fn test_mixed_levels_during_burst_with_statustext() {
        reset_for_statustext_test();

        // Burst with mixed levels
        for i in 0..10 {
            match i % 5 {
                0 => route_log(make_msg(LogLevel::Trace, &format!("trace {}", i))),
                1 => route_log(make_msg(LogLevel::Debug, &format!("debug {}", i))),
                2 => route_log(make_msg(LogLevel::Info, &format!("info {}", i))),
                3 => route_log(make_msg(LogLevel::Warn, &format!("warn {}", i))),
                4 => route_log(make_msg(LogLevel::Error, &format!("error {}", i))),
                _ => unreachable!(),
            }
        }

        // All 10 in buffer
        assert_eq!(buffer_len(), 10);

        // Only WARN (i=3, i=8) and ERROR (i=4, i=9) should be in STATUSTEXT = 4 messages
        let statustext_msgs = take_pending_statustext_messages();
        assert_eq!(statustext_msgs.len(), 4);

        disable_statustext_routing();
        let _ = get_buffered_logs(); // clean up
    }
}
