//! MAVLink STATUSTEXT Notification System (firmware wrapper)
//!
//! Re-exports from `pico_trail_core::communication::status_notifier`.

pub use pico_trail_core::communication::status_notifier::{
    chunk_message, send_alert, send_critical, send_debug, send_emergency, send_error, send_info,
    send_notice, send_warning, take_pending_statustext_messages, QueuedMessage, StatusNotifier,
};
