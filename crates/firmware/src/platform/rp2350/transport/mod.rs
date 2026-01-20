//! RP2350 Platform Transport Implementations
//!
//! This module contains transport implementations that require platform-specific
//! features (embassy-net for UDP).

pub mod udp;

pub use udp::{GcsEndpoint, Rp2350UdpTransport};
