//! MAVLink Message Dispatcher - re-exported from core
//!
//! See `pico_trail_core::communication::dispatcher` for implementation.

pub use pico_trail_core::communication::dispatcher::{
    ConnectionState, DispatcherStats, MessageDispatcher, MAX_RESPONSES,
};
