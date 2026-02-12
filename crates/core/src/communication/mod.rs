//! Communication layer (MAVLink protocol handling)
//!
//! Platform-independent MAVLink message processing, status notification,
//! and dispatch infrastructure.
//!
//! The `handlers` and `dispatcher` modules require the `embassy` feature
//! since they access global state via `EmbassyState`.

#[cfg(feature = "embassy")]
pub mod dispatcher;
#[cfg(feature = "embassy")]
pub mod handlers;
pub mod status_notifier;
