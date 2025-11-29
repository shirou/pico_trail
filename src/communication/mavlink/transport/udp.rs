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

use super::{MavlinkTransport, TransportError};
use heapless::Vec;

#[cfg(feature = "pico2_w")]
use embassy_net::{
    udp::{PacketMetadata, UdpSocket},
    IpEndpoint, Stack,
};
#[cfg(feature = "pico2_w")]
use embassy_time::Instant;

/// MAVLink UDP port (standard)
pub const MAVLINK_UDP_PORT: u16 = 14550;

/// Maximum number of concurrent GCS connections
pub const MAX_GCS_ENDPOINTS: usize = 4;

/// GCS endpoint timeout (10 seconds)
pub const GCS_TIMEOUT_MS: u32 = 10_000;

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

/// GCS endpoint with activity tracking
#[cfg(feature = "pico2_w")]
#[derive(Debug, Clone, Copy)]
struct GcsEndpoint {
    /// Socket address
    addr: IpEndpoint,
    /// Last activity timestamp
    last_activity: Instant,
}

/// GCS endpoint with activity tracking (mock version)
#[cfg(not(feature = "pico2_w"))]
#[derive(Debug, Clone, Copy)]
struct GcsEndpoint {
    /// Socket address
    addr: SocketAddr,
    /// Last activity timestamp (milliseconds)
    last_activity_ms: u32,
}

/// UDP transport for MAVLink communication
#[cfg(feature = "pico2_w")]
pub struct UdpTransport<'a> {
    /// UDP socket
    socket: UdpSocket<'a>,
    /// Active GCS endpoints
    gcs_endpoints: Vec<GcsEndpoint, MAX_GCS_ENDPOINTS>,
    /// Last cleanup time
    last_cleanup: Instant,
}

/// UDP transport for MAVLink communication (mock version for non-pico2_w builds)
#[cfg(not(feature = "pico2_w"))]
pub struct UdpTransport {
    /// Bind port (default 14550)
    _bind_port: u16,
    /// Active GCS endpoints
    gcs_endpoints: Vec<GcsEndpoint, MAX_GCS_ENDPOINTS>,
    /// Current time (milliseconds, for timeout tracking)
    current_time_ms: u32,
}

#[cfg(feature = "pico2_w")]
impl<'a> UdpTransport<'a> {
    /// Create new UDP transport with embassy-net stack
    ///
    /// # Arguments
    ///
    /// * `stack` - embassy-net network stack
    /// * `port` - UDP port to bind (default: 14550)
    /// * `rx_meta` - RX packet metadata buffer
    /// * `rx_buffer` - RX data buffer
    /// * `tx_meta` - TX packet metadata buffer
    /// * `tx_buffer` - TX data buffer
    ///
    /// # Returns
    ///
    /// UDP transport instance
    pub fn new(
        stack: &'a Stack<'a>,
        port: u16,
        rx_meta: &'a mut [PacketMetadata],
        rx_buffer: &'a mut [u8],
        tx_meta: &'a mut [PacketMetadata],
        tx_buffer: &'a mut [u8],
    ) -> Self {
        let mut socket = UdpSocket::new(*stack, rx_meta, rx_buffer, tx_meta, tx_buffer);

        // Bind to port
        if let Err(_) = socket.bind(port) {
            crate::log_error!("Failed to bind UDP socket to port {}", port);
        } else {
            crate::log_info!("UDP socket bound to port {}", port);
        }

        Self {
            socket,
            gcs_endpoints: Vec::new(),
            last_cleanup: Instant::now(),
        }
    }

    /// Track GCS endpoint
    ///
    /// Adds endpoint to active list or updates last activity time.
    ///
    /// # Arguments
    ///
    /// * `addr` - GCS socket address
    pub fn track_endpoint(&mut self, addr: IpEndpoint) {
        // Update existing endpoint
        if let Some(endpoint) = self.gcs_endpoints.iter_mut().find(|e| e.addr == addr) {
            endpoint.last_activity = Instant::now();
            return;
        }

        // Add new endpoint (if space available)
        let endpoint = GcsEndpoint {
            addr,
            last_activity: Instant::now(),
        };

        let _ = self.gcs_endpoints.push(endpoint);

        crate::log_info!(
            "GCS endpoint registered: port {} (total: {})",
            addr.port,
            self.gcs_endpoints.len()
        );
    }

    /// Cleanup inactive GCS endpoints
    ///
    /// Removes endpoints with no activity for more than GCS_TIMEOUT_MS.
    pub fn cleanup_inactive(&mut self) {
        let now = Instant::now();
        let timeout = embassy_time::Duration::from_millis(GCS_TIMEOUT_MS as u64);

        self.gcs_endpoints
            .retain(|endpoint| now.duration_since(endpoint.last_activity) < timeout);
    }

    /// Get active GCS count
    pub fn gcs_count(&self) -> usize {
        self.gcs_endpoints.len()
    }
}

#[cfg(not(feature = "pico2_w"))]
impl UdpTransport {
    /// Create new UDP transport (mock version)
    pub fn new(port: u16) -> Self {
        Self {
            _bind_port: port,
            gcs_endpoints: Vec::new(),
            current_time_ms: 0,
        }
    }
}

// Mock version helper methods
#[cfg(not(feature = "pico2_w"))]
impl UdpTransport {
    /// Track GCS endpoint (mock version)
    pub fn track_endpoint(&mut self, addr: SocketAddr) {
        if let Some(endpoint) = self.gcs_endpoints.iter_mut().find(|e| e.addr == addr) {
            endpoint.last_activity_ms = self.current_time_ms;
            return;
        }

        let endpoint = GcsEndpoint {
            addr,
            last_activity_ms: self.current_time_ms,
        };

        let _ = self.gcs_endpoints.push(endpoint);
    }

    /// Cleanup inactive GCS endpoints (mock version)
    pub fn cleanup_inactive(&mut self) {
        let current_time = self.current_time_ms;
        self.gcs_endpoints.retain(|endpoint| {
            let elapsed = current_time.saturating_sub(endpoint.last_activity_ms);
            elapsed < GCS_TIMEOUT_MS
        });
    }

    /// Get active GCS count (mock version)
    pub fn gcs_count(&self) -> usize {
        self.gcs_endpoints.len()
    }

    /// Update current time (mock version)
    pub fn update_time(&mut self, time_ms: u32) {
        self.current_time_ms = time_ms;
    }
}

#[cfg(feature = "pico2_w")]
impl<'a> MavlinkTransport for UdpTransport<'a> {
    async fn available(&self) -> usize {
        // UDP sockets don't have a direct "available bytes" query
        // Return 0 to indicate we should always try to read
        0
    }

    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError> {
        // Receive packet from socket
        match self.socket.recv_from(buf).await {
            Ok((n, metadata)) => {
                // Track sender endpoint for future broadcasts
                self.track_endpoint(metadata.endpoint);

                // Periodically cleanup inactive endpoints
                let now = Instant::now();
                if now.duration_since(self.last_cleanup) > embassy_time::Duration::from_secs(1) {
                    self.cleanup_inactive();
                    self.last_cleanup = now;
                }

                Ok(n)
            }
            Err(_) => Err(TransportError::IoError),
        }
    }

    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError> {
        if self.gcs_endpoints.is_empty() {
            // No GCS connected yet - silently drop
            return Ok(buf.len());
        }

        // Broadcast to all active GCS endpoints
        let mut total_sent = 0;
        for endpoint in &self.gcs_endpoints {
            match self.socket.send_to(buf, endpoint.addr).await {
                Ok(()) => {
                    total_sent += buf.len();
                }
                Err(_) => {
                    crate::log_warn!("Failed to send to endpoint");
                }
            }
        }

        // Return total bytes sent (even if some sends failed)
        Ok(total_sent)
    }

    async fn flush(&mut self) -> Result<(), TransportError> {
        // UDP is connectionless, no buffering to flush
        Ok(())
    }
}

#[cfg(not(feature = "pico2_w"))]
impl MavlinkTransport for UdpTransport {
    async fn available(&self) -> usize {
        0
    }

    async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, TransportError> {
        Err(TransportError::IoError)
    }

    async fn write(&mut self, _buf: &[u8]) -> Result<usize, TransportError> {
        Err(TransportError::IoError)
    }

    async fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_udp_transport_creation() {
        let transport = UdpTransport::new(MAVLINK_UDP_PORT);
        assert_eq!(transport.gcs_count(), 0);
    }

    #[test]
    fn test_track_endpoint() {
        let mut transport = UdpTransport::new(MAVLINK_UDP_PORT);
        let addr1 = SocketAddr::new([192, 168, 1, 100], 14550);
        let addr2 = SocketAddr::new([192, 168, 1, 101], 14550);

        transport.track_endpoint(addr1);
        assert_eq!(transport.gcs_count(), 1);

        transport.track_endpoint(addr2);
        assert_eq!(transport.gcs_count(), 2);

        // Tracking same endpoint again should not add duplicate
        transport.track_endpoint(addr1);
        assert_eq!(transport.gcs_count(), 2);
    }

    #[test]
    fn test_cleanup_inactive() {
        let mut transport = UdpTransport::new(MAVLINK_UDP_PORT);
        let addr = SocketAddr::new([192, 168, 1, 100], 14550);

        transport.update_time(0);
        transport.track_endpoint(addr);
        assert_eq!(transport.gcs_count(), 1);

        // Advance time beyond timeout
        transport.update_time(GCS_TIMEOUT_MS + 1000);
        transport.cleanup_inactive();
        assert_eq!(transport.gcs_count(), 0);
    }

    #[test]
    fn test_max_gcs_endpoints() {
        let mut transport = UdpTransport::new(MAVLINK_UDP_PORT);

        // Add 4 endpoints (max)
        for i in 0..MAX_GCS_ENDPOINTS {
            let addr = SocketAddr::new([192, 168, 1, 100 + i as u8], 14550);
            transport.track_endpoint(addr);
        }
        assert_eq!(transport.gcs_count(), MAX_GCS_ENDPOINTS);

        // Try to add 5th endpoint (should be ignored due to capacity)
        let addr5 = SocketAddr::new([192, 168, 1, 200], 14550);
        transport.track_endpoint(addr5);
        assert_eq!(transport.gcs_count(), MAX_GCS_ENDPOINTS);
    }
}
