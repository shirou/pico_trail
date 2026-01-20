//! UDP Transport for MAVLink Communication (RP2350 Platform)
//!
//! Provides UDP socket-based transport for MAVLink messages over WiFi using embassy-net.
//!
//! # Features
//!
//! - **GCS Endpoint Tracking**: Automatically tracks up to 4 Ground Control Stations
//! - **Broadcast Send**: Messages sent to all active GCS endpoints
//! - **Timeout Management**: Inactive GCS removed after 10-second timeout
//! - **Standard Port**: Binds to UDP port 14550 (MAVLink standard)

use crate::communication::mavlink::transport::{MavlinkTransport, TransportError};
use embassy_net::{
    udp::{PacketMetadata, UdpSocket},
    IpEndpoint, Stack,
};
use embassy_time::Instant;
use heapless::Vec;

/// Maximum number of concurrent GCS connections
pub const MAX_GCS_ENDPOINTS: usize = 4;

/// GCS endpoint timeout (10 seconds)
pub const GCS_TIMEOUT_MS: u32 = 10_000;

/// GCS endpoint with activity tracking
#[derive(Debug, Clone, Copy)]
pub struct GcsEndpoint {
    /// Socket address
    pub addr: IpEndpoint,
    /// Last activity timestamp
    pub last_activity: Instant,
}

/// UDP transport for MAVLink communication (RP2350 platform)
pub struct Rp2350UdpTransport<'a> {
    /// UDP socket
    socket: UdpSocket<'a>,
    /// Active GCS endpoints
    gcs_endpoints: Vec<GcsEndpoint, MAX_GCS_ENDPOINTS>,
    /// Last cleanup time
    last_cleanup: Instant,
}

impl<'a> Rp2350UdpTransport<'a> {
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
        if socket.bind(port).is_err() {
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

impl<'a> MavlinkTransport for Rp2350UdpTransport<'a> {
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
