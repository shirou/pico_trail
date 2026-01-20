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
    pub fn set_udp_transport(&mut self, transport: super::transport::udp::UdpTransport<'a>) {
        self.udp_transport = Some(transport);
    }

    /// Check if any transport is available
    ///
    /// # Returns
    ///
    /// `true` if at least one transport is configured
    pub fn has_transport(&self) -> bool {
        self.uart_transport.is_some() || self.udp_transport.is_some()
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
        // Concurrent read from UART and UDP
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

    /// Flush all transports
    ///
    /// Ensures all buffered data is transmitted.
    ///
    /// # Returns
    ///
    /// - `Ok(())` - All transports flushed successfully
    /// - `Err(RouterError)` - No transport or flush error
    pub async fn flush(&mut self) -> Result<(), RouterError> {
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
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
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
