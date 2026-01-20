//! UDP Transport for MAVLink Communication
//!
//! Provides UDP socket-based transport for MAVLink messages over WiFi.
//!
//! # Features
//!
//! - **GCS Endpoint Tracking**: Automatically tracks up to 4 Ground Control Stations
//! - **Broadcast Send**: Messages sent to all active GCS endpoints
//! - **Timeout Management**: Inactive GCS removed after 10-second timeout
//! - **Standard Port**: Binds to UDP port 14550 (MAVLink standard)
//!
//! # Platform Implementation
//!
//! The actual UDP transport implementation for embedded targets lives in
//! `src/platform/rp2350/transport/udp.rs`. This module re-exports it and
//! provides a mock implementation for host tests.
//!
//! # GCS Discovery
//!
//! GCS endpoints are discovered automatically:
//! 1. GCS sends MAVLink message to port 14550
//! 2. UDP transport receives message and extracts sender endpoint
//! 3. Endpoint added to active GCS list (max 4)
//! 4. All outbound messages broadcast to active GCS list
//!
//! # Memory Usage
//!
//! - GCS endpoint tracking: ~64 bytes (4 endpoints × 16 bytes)
//! - UDP socket buffers: ~8 KB (4 KB RX + 4 KB TX)
//! - Total: ~8 KB RAM
//!
//! # Example
//!
//! ```no_run
//! use pico_trail::communication::mavlink::transport::udp::UdpTransport;
//! use pico_trail::communication::mavlink::transport::MavlinkTransport;
//!
//! async fn send_heartbeat(transport: &mut UdpTransport) {
//!     let heartbeat = [0xFD, 0x09, 0x00, /* ... */];
//!     transport.write(&heartbeat).await.unwrap();
//!     transport.flush().await.unwrap();
//! }
//! ```

/// MAVLink UDP port (standard)
pub const MAVLINK_UDP_PORT: u16 = 14550;

/// Maximum number of concurrent GCS connections
pub const MAX_GCS_ENDPOINTS: usize = 4;

/// GCS endpoint timeout (10 seconds)
pub const GCS_TIMEOUT_MS: u32 = 10_000;

// Re-export platform-specific implementation
pub use crate::platform::rp2350::transport::udp::{
    GcsEndpoint, Rp2350UdpTransport as UdpTransport,
};

/// UDP socket endpoint (IP + port)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SocketAddr {
    /// IPv4 address (4 bytes)
    pub ip: [u8; 4],
    /// UDP port
    pub port: u16,
}

impl SocketAddr {
    /// Create new socket address
    pub fn new(ip: [u8; 4], port: u16) -> Self {
        Self { ip, port }
    }
}
