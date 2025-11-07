//! MAVLink Transport Router
//!
//! Routes MAVLink messages between multiple transports (UART, UDP, TCP).
//!
//! # Architecture
//!
//! The transport router provides multi-transport support by:
//! - Managing multiple `MavlinkTransport` instances
//! - Routing incoming messages from any transport to the protocol layer
//! - Broadcasting outgoing messages to all active transports
//! - Tracking per-transport statistics
//!
//! # Design Pattern
//!
//! ```text
//! ┌────────────────────────────────────────────┐
//! │         MAVLink Protocol Layer              │
//! │   (parser, writer, message handlers)        │
//! └──────────────────┬─────────────────────────┘
//!                    │
//!                    │ receive_message()
//!                    │ send_message()
//!                    ▼
//! ┌────────────────────────────────────────────┐
//! │          TransportRouter                    │
//! │  - Manages multiple transports              │
//! │  - Routes messages                          │
//! │  - Tracks statistics                        │
//! └─────┬──────────┬──────────┬────────────────┘
//!       │          │          │
//!       ▼          ▼          ▼
//!   UART 0      UDP        TCP (future)
//! ```
//!
//! # Phase 1 Implementation
//!
//! Phase 1 supports single UART transport only:
//! - Direct message passing (no routing needed yet)
//! - Statistics tracking foundation
//! - Prepares for Phase 2 multi-transport support
//!
//! # Phase 2 Enhancement
//!
//! Phase 2 adds concurrent multi-transport support:
//! - UART + UDP simultaneous operation
//! - `embassy_futures::select_array` for concurrent reads
//! - Message broadcasting to all transports
//! - Transport-specific error handling
//!
//! # Usage
//!
//! ```ignore
//! use pico_trail::communication::mavlink::transport_router::TransportRouter;
//!
//! // Phase 1: Single UART transport
//! let mut router = TransportRouter::new();
//! router.set_uart_transport(uart_transport);
//!
//! // Read from transport
//! let (data, transport_id) = router.receive_bytes().await?;
//!
//! // Send to transport
//! router.send_bytes(&message_bytes).await?;
//! ```

use super::transport::{MavlinkTransport, TransportError};

/// Transport statistics for monitoring
#[derive(Debug, Clone, Copy, Default)]
pub struct TransportStats {
    /// Total bytes received
    pub bytes_received: u32,
    /// Total bytes sent
    pub bytes_sent: u32,
    /// Number of receive errors
    pub receive_errors: u32,
    /// Number of send errors
    pub send_errors: u32,
}

/// Transport router for managing multiple MAVLink transports
///
/// # Phase 1: Single UART Transport
///
/// Initially supports one UART transport. Message routing is trivial
/// (all messages go to/from the single transport).
///
/// # Phase 2: Multi-Transport Support
///
/// Will be extended to support:
/// - Multiple transports (UART + UDP + TCP)
/// - Concurrent message reception using `embassy_futures::select_array`
/// - Message broadcasting to all transports
/// - Transport enumeration and management
///
/// # Type Parameters
///
/// - `'a`: Lifetime for network stack references (UDP transport)
/// - `R`: UART receiver type
/// - `W`: UART transmitter type
pub struct TransportRouter<'a, R, W> {
    /// UART transport (Phase 1: single transport)
    uart_transport: Option<super::transport::uart::UartTransport<R, W>>,
    /// UDP transport (Phase 2: network transport)
    #[cfg(feature = "pico2_w")]
    udp_transport: Option<super::transport::udp::UdpTransport<'a>>,
    /// Transport statistics
    stats: TransportStats,
    /// Lifetime marker (ensures 'a is used even without UDP feature)
    _phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a, R, W> TransportRouter<'a, R, W>
where
    R: embedded_io_async::Read,
    W: embedded_io_async::Write,
{
    /// Create a new transport router
    ///
    /// Initially has no transports. Use `set_uart_transport()` and `set_udp_transport()` to add transports.
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let mut router = TransportRouter::new();
    /// router.set_uart_transport(uart_transport);
    /// router.set_udp_transport(udp_transport);
    /// ```
    pub fn new() -> Self {
        Self {
            uart_transport: None,
            #[cfg(feature = "pico2_w")]
            udp_transport: None,
            stats: TransportStats::default(),
            _phantom: core::marker::PhantomData,
        }
    }

    /// Set the UART transport
    ///
    /// Replaces any existing UART transport.
    ///
    /// # Arguments
    ///
    /// * `transport` - UART transport instance
    pub fn set_uart_transport(&mut self, transport: super::transport::uart::UartTransport<R, W>) {
        self.uart_transport = Some(transport);
    }

    /// Set the UDP transport (Phase 2)
    ///
    /// Replaces any existing UDP transport.
    ///
    /// # Arguments
    ///
    /// * `transport` - UDP transport instance
    #[cfg(feature = "pico2_w")]
    pub fn set_udp_transport(&mut self, transport: super::transport::udp::UdpTransport<'a>) {
        self.udp_transport = Some(transport);
    }

    /// Check if any transport is available
    ///
    /// # Returns
    ///
    /// `true` if at least one transport is configured
    pub fn has_transport(&self) -> bool {
        #[cfg(feature = "pico2_w")]
        {
            self.uart_transport.is_some() || self.udp_transport.is_some()
        }
        #[cfg(not(feature = "pico2_w"))]
        {
            self.uart_transport.is_some()
        }
    }

    /// Get transport statistics
    ///
    /// Returns statistics for all transports combined.
    ///
    /// # Returns
    ///
    /// Copy of current statistics
    pub fn stats(&self) -> TransportStats {
        self.stats
    }

    /// Reset transport statistics
    ///
    /// Resets all counters to zero.
    pub fn reset_stats(&mut self) {
        self.stats = TransportStats::default();
    }

    /// Receive bytes from any transport
    ///
    /// Blocks until data is available from a transport or an error occurs.
    ///
    /// # Arguments
    ///
    /// * `buf` - Buffer to read into
    ///
    /// # Returns
    ///
    /// - `Ok((n, transport_id))` - Number of bytes read and transport ID (0 = UART, 1 = UDP)
    /// - `Err(RouterError)` - No transport available or read error
    ///
    /// # Phase 2 Implementation
    ///
    /// Uses `embassy_futures::select` to read from multiple transports concurrently
    /// and returns the first available message.
    pub async fn receive_bytes(&mut self, buf: &mut [u8]) -> Result<(usize, usize), RouterError> {
        #[cfg(feature = "pico2_w")]
        {
            // Phase 2: Concurrent read from UART and UDP
            match (&mut self.uart_transport, &mut self.udp_transport) {
                (Some(uart), Some(udp)) => {
                    // Both transports available - use select with separate buffers
                    let mut uart_buf = [0u8; 256];
                    let mut udp_buf = [0u8; 256];
                    match embassy_futures::select::select(
                        uart.read(&mut uart_buf),
                        udp.read(&mut udp_buf),
                    )
                    .await
                    {
                        embassy_futures::select::Either::First(result) => match result {
                            Ok(n) => {
                                buf[..n].copy_from_slice(&uart_buf[..n]);
                                self.stats.bytes_received += n as u32;
                                Ok((n, 0)) // Transport ID 0 = UART
                            }
                            Err(e) => {
                                self.stats.receive_errors += 1;
                                Err(RouterError::TransportError(e))
                            }
                        },
                        embassy_futures::select::Either::Second(result) => match result {
                            Ok(n) => {
                                buf[..n].copy_from_slice(&udp_buf[..n]);
                                self.stats.bytes_received += n as u32;
                                Ok((n, 1)) // Transport ID 1 = UDP
                            }
                            Err(e) => {
                                self.stats.receive_errors += 1;
                                Err(RouterError::TransportError(e))
                            }
                        },
                    }
                }
                (Some(uart), None) => {
                    // Only UART available
                    match uart.read(buf).await {
                        Ok(n) => {
                            self.stats.bytes_received += n as u32;
                            Ok((n, 0)) // Transport ID 0 = UART
                        }
                        Err(e) => {
                            self.stats.receive_errors += 1;
                            Err(RouterError::TransportError(e))
                        }
                    }
                }
                (None, Some(udp)) => {
                    // Only UDP available
                    match udp.read(buf).await {
                        Ok(n) => {
                            self.stats.bytes_received += n as u32;
                            Ok((n, 1)) // Transport ID 1 = UDP
                        }
                        Err(e) => {
                            self.stats.receive_errors += 1;
                            Err(RouterError::TransportError(e))
                        }
                    }
                }
                (None, None) => Err(RouterError::NoTransport),
            }
        }
        #[cfg(not(feature = "pico2_w"))]
        {
            // Phase 1: Single UART transport only
            if let Some(ref mut transport) = self.uart_transport {
                match transport.read(buf).await {
                    Ok(n) => {
                        self.stats.bytes_received += n as u32;
                        Ok((n, 0)) // Transport ID 0 = UART
                    }
                    Err(e) => {
                        self.stats.receive_errors += 1;
                        Err(RouterError::TransportError(e))
                    }
                }
            } else {
                Err(RouterError::NoTransport)
            }
        }
    }

    /// Send bytes to all transports
    ///
    /// Broadcasts data to all active transports.
    ///
    /// # Arguments
    ///
    /// * `buf` - Data to send
    ///
    /// # Returns
    ///
    /// - `Ok(n)` - Number of bytes sent (buf.len() if successful)
    /// - `Err(RouterError)` - No transport available or all transports failed
    ///
    /// # Phase 2 Implementation
    ///
    /// Broadcasts to all transports (UART + UDP). Continues sending even if
    /// some transports fail (best-effort delivery). Returns error only if
    /// all transports fail.
    pub async fn send_bytes(&mut self, buf: &[u8]) -> Result<usize, RouterError> {
        #[cfg(feature = "pico2_w")]
        {
            let mut any_success = false;
            let mut total_sent = 0;

            // Send to UART if available
            if let Some(ref mut uart) = self.uart_transport {
                match uart.write(buf).await {
                    Ok(n) => {
                        self.stats.bytes_sent += n as u32;
                        total_sent = n;
                        any_success = true;
                    }
                    Err(_) => {
                        self.stats.send_errors += 1;
                        crate::log_warn!("UART send failed");
                    }
                }
            }

            // Send to UDP if available
            if let Some(ref mut udp) = self.udp_transport {
                match udp.write(buf).await {
                    Ok(n) => {
                        self.stats.bytes_sent += n as u32;
                        total_sent = n;
                        any_success = true;
                    }
                    Err(_) => {
                        self.stats.send_errors += 1;
                        crate::log_warn!("UDP send failed");
                    }
                }
            }

            if any_success {
                Ok(total_sent)
            } else {
                Err(RouterError::NoTransport)
            }
        }
        #[cfg(not(feature = "pico2_w"))]
        {
            // Phase 1: Single UART transport only
            if let Some(ref mut transport) = self.uart_transport {
                match transport.write(buf).await {
                    Ok(n) => {
                        self.stats.bytes_sent += n as u32;
                        Ok(n)
                    }
                    Err(e) => {
                        self.stats.send_errors += 1;
                        Err(RouterError::TransportError(e))
                    }
                }
            } else {
                Err(RouterError::NoTransport)
            }
        }
    }

    /// Flush all transports
    ///
    /// Ensures all buffered data is transmitted.
    ///
    /// # Returns
    ///
    /// - `Ok(())` - All transports flushed successfully
    /// - `Err(RouterError)` - No transport or flush error
    pub async fn flush(&mut self) -> Result<(), RouterError> {
        #[cfg(feature = "pico2_w")]
        {
            let mut any_transport = false;
            let mut last_error = None;

            // Flush UART if available
            if let Some(ref mut uart) = self.uart_transport {
                any_transport = true;
                if let Err(e) = uart.flush().await {
                    last_error = Some(e);
                }
            }

            // Flush UDP if available
            if let Some(ref mut udp) = self.udp_transport {
                any_transport = true;
                if let Err(e) = udp.flush().await {
                    last_error = Some(e);
                }
            }

            if !any_transport {
                Err(RouterError::NoTransport)
            } else if let Some(e) = last_error {
                Err(RouterError::TransportError(e))
            } else {
                Ok(())
            }
        }
        #[cfg(not(feature = "pico2_w"))]
        {
            if let Some(ref mut transport) = self.uart_transport {
                transport.flush().await.map_err(RouterError::TransportError)
            } else {
                Err(RouterError::NoTransport)
            }
        }
    }
}

impl<'a, R, W> Default for TransportRouter<'a, R, W>
where
    R: embedded_io_async::Read,
    W: embedded_io_async::Write,
{
    fn default() -> Self {
        Self::new()
    }
}

/// Router error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RouterError {
    /// No transport available
    NoTransport,
    /// Transport error occurred
    TransportError(TransportError),
}

impl core::fmt::Display for RouterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RouterError::NoTransport => write!(f, "No transport available"),
            RouterError::TransportError(e) => write!(f, "Transport error: {}", e),
        }
    }
}

// Note: RouterError implements Display, sufficient for embedded use

#[cfg(test)]
mod tests {
    use super::*;
    use crate::communication::mavlink::transport::uart::UartTransport;

    /// Mock UART types for testing
    struct MockUartRx {
        data: &'static [u8],
        pos: usize,
    }

    impl MockUartRx {
        fn new(data: &'static [u8]) -> Self {
            Self { data, pos: 0 }
        }
    }

    impl embedded_io_async::ErrorType for MockUartRx {
        type Error = core::convert::Infallible;
    }

    impl embedded_io_async::Read for MockUartRx {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            if self.pos >= self.data.len() {
                return Ok(0);
            }

            let remaining = &self.data[self.pos..];
            let to_read = core::cmp::min(buf.len(), remaining.len());
            buf[..to_read].copy_from_slice(&remaining[..to_read]);
            self.pos += to_read;
            Ok(to_read)
        }
    }

    struct MockUartTx {
        buffer: heapless::Vec<u8, 256>,
    }

    impl MockUartTx {
        fn new() -> Self {
            Self {
                buffer: heapless::Vec::new(),
            }
        }
    }

    impl embedded_io_async::ErrorType for MockUartTx {
        type Error = core::convert::Infallible;
    }

    impl embedded_io_async::Write for MockUartTx {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, core::convert::Infallible> {
            self.buffer.extend_from_slice(buf).unwrap();
            Ok(buf.len())
        }

        async fn flush(&mut self) -> Result<(), core::convert::Infallible> {
            Ok(())
        }
    }

    #[tokio::test]
    async fn test_router_creation() {
        let router: TransportRouter<'_, MockUartRx, MockUartTx> = TransportRouter::new();
        assert!(!router.has_transport());
        assert_eq!(router.stats().bytes_received, 0);
        assert_eq!(router.stats().bytes_sent, 0);
    }

    #[tokio::test]
    async fn test_router_set_transport() {
        let mut router = TransportRouter::new();
        let rx = MockUartRx::new(b"");
        let tx = MockUartTx::new();
        let transport = UartTransport::new(rx, tx);

        router.set_uart_transport(transport);
        assert!(router.has_transport());
    }

    #[tokio::test]
    async fn test_router_receive_no_transport() {
        let mut router: TransportRouter<'_, MockUartRx, MockUartTx> = TransportRouter::new();
        let mut buf = [0u8; 32];

        let result = router.receive_bytes(&mut buf).await;
        assert!(matches!(result, Err(RouterError::NoTransport)));
    }

    #[tokio::test]
    async fn test_router_receive_with_transport() {
        let mut router = TransportRouter::new();
        let rx = MockUartRx::new(b"Hello");
        let tx = MockUartTx::new();
        let transport = UartTransport::new(rx, tx);

        router.set_uart_transport(transport);

        let mut buf = [0u8; 32];
        let (n, transport_id) = router.receive_bytes(&mut buf).await.unwrap();
        assert_eq!(n, 5);
        assert_eq!(transport_id, 0); // UART = 0
        assert_eq!(&buf[..n], b"Hello");
        assert_eq!(router.stats().bytes_received, 5);
    }

    #[tokio::test]
    async fn test_router_send_no_transport() {
        let mut router: TransportRouter<'_, MockUartRx, MockUartTx> = TransportRouter::new();
        let data = b"Test";

        let result = router.send_bytes(data).await;
        assert!(matches!(result, Err(RouterError::NoTransport)));
    }

    #[tokio::test]
    async fn test_router_send_with_transport() {
        let mut router = TransportRouter::new();
        let rx = MockUartRx::new(b"");
        let tx = MockUartTx::new();
        let transport = UartTransport::new(rx, tx);

        router.set_uart_transport(transport);

        let data = b"Test message";
        let n = router.send_bytes(data).await.unwrap();
        assert_eq!(n, data.len());
        assert_eq!(router.stats().bytes_sent, 12);
    }

    #[tokio::test]
    async fn test_router_stats_reset() {
        let mut router = TransportRouter::new();
        let rx = MockUartRx::new(b"data");
        let tx = MockUartTx::new();
        let transport = UartTransport::new(rx, tx);

        router.set_uart_transport(transport);

        // Generate some stats
        let mut buf = [0u8; 32];
        router.receive_bytes(&mut buf).await.unwrap();

        assert_eq!(router.stats().bytes_received, 4);

        // Reset
        router.reset_stats();
        assert_eq!(router.stats().bytes_received, 0);
    }

    #[tokio::test]
    async fn test_router_flush() {
        let mut router = TransportRouter::new();
        let rx = MockUartRx::new(b"");
        let tx = MockUartTx::new();
        let transport = UartTransport::new(rx, tx);

        router.set_uart_transport(transport);

        // Should not error
        router.flush().await.unwrap();
    }
}
